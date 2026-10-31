// tb_tx_agg.v —— Q1「发送端持续吞吐」口径：10 条 VL 同时满载的聚合吞吐
`timescale 1ns/1ps

module tb_tx_agg;

    reg clk = 1'b0, reset = 1'b1;
    always #4 clk = ~clk;                  // 125 MHz
    localparam integer TICK = 1000;        // 8 µs @125 MHz

    localparam integer NP   = 10;          // 发送端口数（与 NPT_TX 一致）
    localparam integer NPW  = 4;
    localparam integer AW   = 14;          // 每端口 4 KB（NW=10，与生产一致）
    localparam integer PLEN = 1471;        // AFDX 最大载荷
    localparam integer NWDS = (PLEN+3)/4;  // 368 个字
    localparam integer ROUNDS = 8;         // 每条 VL 发 8 条（第 1 轮当预热）

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    // ---- 8 µs 时基 ----
    reg  [31:0] tcnt = 0;
    reg  [15:0] ts_r = 0;
    always @(posedge clk) begin
        if (reset) begin tcnt <= 0; ts_r <= 0; end
        else if (tcnt >= TICK-1) begin tcnt <= 0; ts_r <= ts_r + 1; end
        else tcnt <= tcnt + 1;
    end
    wire [15:0] ts_now = ts_r;

    // =====================================================================
    // VL 配置表（本台**自己写表**：把 10 条 VL 全部配成 BAG=1 ms / lmax=1471）
    // =====================================================================
    reg         cwr = 0;
    reg  [3:0]  crs = 0;
    reg  [31:0] cwd = 0;
    wire        cfg_hit; wire [127:0] cfg_w; wire [15:0] cfg_res_vlid, cfg_vlid_req;

    wire        ir_lk_req = 1'b0; wire [15:0] rx_lk_vlid = 16'h0;
    wire [15:0] perf_lk_vlid = ir_lk_req ? rx_lk_vlid : cfg_vlid_req;

    afdx_cfg #(.NENT(4096), .IDXW(12)) u_cfg (
        .clk(clk), .reset(reset),
        .wr(cwr), .rd(1'b0), .reg_sel(crs), .wdata(cwd), .rdata(),
        .vlid_i(perf_lk_vlid), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(cfg_res_vlid), .hit_o(cfg_hit), .cfg_o(cfg_w)
    );

    task cfg_wr;
        input [3:0] r; input [31:0] d;
        begin
            @(negedge clk); cwr = 1; crs = r; cwd = d;
            @(posedge clk);
            @(negedge clk); cwr = 0;
        end
    endtask

    // 写一条 VL：BAG=1 ms（bag_shift=0）、lmax=1471、A+B 双发、IC/RM 使能
    task cfg_vl;
        input [15:0] vlid; input [15:0] dport;
        begin
            cfg_wr(4'd0, {16'h0, vlid});                                   // R_IDX
            // w0: sn_vld(31)=1 ram(30)=1 ic(29)=1 net_en(28:27)=0 lmax(26:16) vlid(15:0)
            cfg_wr(4'd1, {1'b1, 1'b1, 1'b1, 2'b00, 11'd1471, vlid});
            // w1: dport(31:16) skew(15:4) bag_shift(3:0)=0  → 5 ms = 625 刻度
            cfg_wr(4'd1, {dport, 12'd625, 4'd0});
            // w2: sport = vlid，buf_idx=0，port_type=0
            cfg_wr(4'd1, {9'h0, 4'h0, 3'd0, vlid});            // w2: port_type(23:20)|buf_idx(18:16)|sport(15:0)
            // w3: 目的 IP = 224.224.VLID
            cfg_wr(4'd1, {16'hE0E0, vlid});
        end
    endtask

    // =====================================================================
    // 发送端口层 + 发送路径
    // =====================================================================
    reg         hw_wr = 0; reg [NPW-1:0] hw_pi = 0; reg [2:0] hw_reg = 0; reg [31:0] hw_wdata = 0;
    wire [31:0] hw_rdata;
    wire        txv, txsop, txeop, txrdy, tx_replay, tx_hold;
    wire [7:0]  txd;
    wire [15:0] tx_vlid, tx_dport;
    wire [31:0] tx_dip;
    wire [10:0] tx_len;
    wire [NP-1:0] tx_busy;

    afdx_tx_ports #(.NP(NP), .NPW(NPW), .AW(AW)) u_txp (
        .clk(clk), .reset(reset),
        .wr(hw_wr), .pi(hw_pi), .reg_sel(hw_reg), .wdata(hw_wdata), .rdata(hw_rdata),
        .tx_valid(txv), .tx_ready(txrdy), .tx_sop(txsop), .tx_eop(txeop), .tx_data(txd),
        .tx_vlid(tx_vlid), .tx_dip(tx_dip), .tx_dport(tx_dport), .tx_len(tx_len),
        .tx_hold(tx_hold), .tx_busy(tx_busy), .tx_pend(),
        .tx_drop(1'b0)
    );

    wire        fr_valid, fr_ready, fr_sop, fr_eop, fr_net;
    wire [7:0]  fr_data;

    afdx_tx_path #(.NVL(4096), .NVLW(12)) u_tx (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .ov_dip(tx_dip), .ov_dport(tx_dport),
        .msg_valid(txv), .msg_ready(txrdy), .msg_sop(txsop), .msg_eop(txeop),
        .msg_data(txd), .msg_port(tx_vlid), .msg_len({3'b000, tx_len}),
        .tx_replay_o(tx_replay), .pay_hold_o(tx_hold),
        .cfg_vlid_o(cfg_vlid_req), .cfg_hit_i(cfg_hit), .cfg_i(cfg_w),
        .cfg_res_vlid_i(cfg_res_vlid),
        .fr_valid(fr_valid), .fr_ready(1'b1), .fr_sop(fr_sop),
        .fr_eop(fr_eop), .fr_data(fr_data), .fr_net(fr_net),
        .bag_hold_o()
    );


    integer n_a = 0;                       // A 网帧总数
    integer vfr [0:8];
    integer vcnt[0:8];
    integer vi0;
    integer last_sop [0:9];                // 每条 VL 上一帧的拍
    integer int_cnt = 0, int_sum = 0;      // 间隔样本数与总拍数
    integer v_idx;
    integer ii;
    initial for (ii = 0; ii < 10; ii = ii + 1) begin last_sop[ii] = -1; end
    initial for (ii = 0; ii < 9;  ii = ii + 1) begin vfr[ii] = 0; vcnt[ii] = 0; end

    always @(posedge clk) begin
        if (fr_valid && fr_sop && (fr_net == 1'b0)) begin
            n_a = n_a + 1;
            // 端口号 → 0..9 的紧凑下标（端口 5 不参与）
            v_idx = (tx_vlid == 16'h0123) ? 0 :
                    (tx_vlid == 16'h0456) ? 1 :
                    (tx_vlid == 16'h0789) ? 2 :
                    (tx_vlid == 16'h0161) ? 3 :
                    (tx_vlid == 16'h0BC5) ? 4 :
                    (tx_vlid == 16'h0A01) ? 5 :
                    (tx_vlid == 16'h0A02) ? 6 :
                    (tx_vlid == 16'h0A03) ? 7 :
                    (tx_vlid == 16'h0A04) ? 8 : 9;
            if (v_idx < 9) vfr[v_idx] = vfr[v_idx] + 1;
            if (last_sop[v_idx] >= 0 && v_idx < 9) begin
                int_cnt = int_cnt + 1;
                int_sum = int_sum + (cyc - last_sop[v_idx]);
                vcnt[v_idx] = vcnt[v_idx] + 1;
            end
            last_sop[v_idx] = cyc;
        end
    end

    // ---- 等到"全局第 target 拍"（时间驱动，不依赖 STATUS 轮询）----
    //   为什么要时间驱动：`tx_busy = pdone | ppend` 在**帧正在传输**时是 **0**
    //   （ppend 在帧首已清、pdone 到帧末才置）⇒ "等 !tx_busy" 会写进正在发的端口，
    //   写被 `wr && !(act_vld && act==pi)` 静默丢弃，而 CTRL 又拿旧 plen 提交 → 帧数变少。
    //   一帧 1518 B ≈ 1538 拍（12.3 µs），9 条串起来 < 120 µs ⇒
    //   在每轮起点 +40 µs 写，上一轮的帧一定都发完了。
    task wait_until;
        input integer target;
        begin
            while (cyc < target) @(posedge clk);
        end
    endtask

    // ---- 写一条 1471 字节的消息到端口 p ----
    integer k;
    task send_big;
        input [NPW-1:0] p;
        begin
            for (k = 0; k < NWDS; k = k + 1)
                begin
                    @(negedge clk); hw_wr = 1; hw_pi = p; hw_reg = 3'd0;
                    hw_wdata = {8'hA5, k[7:0], 8'h5A, 8'hC3};
                    @(posedge clk);
                    @(negedge clk); hw_wr = 0;
                end
            @(negedge clk); hw_wr = 1; hw_pi = p; hw_reg = 3'd1; hw_wdata = PLEN; @(posedge clk);
            @(negedge clk); hw_reg = 3'd2; hw_wdata = 32'd1; @(posedge clk);
            @(negedge clk); hw_wr = 0; hw_reg = 3'd0;
        end
    endtask

    integer p, r, wt, errs = 0, nvl = 0;
    real mbps;

    task chk; input c; input [255:0] nm;
        begin if (c) $display("  [ok]   %0s", nm);
              else begin errs = errs + 1; $display("  [FAIL] %0s", nm); end end
    endtask

    initial begin
        #200 reset = 1'b0;
        repeat (20) @(posedge clk);

        // 等 BAG 表（nxt_t）清零完成 —— 与 tb_perf 同款（不等会把 4098 拍算进时延）
        begin : w_tbl
            integer w; w = 0;
            while (!u_tx.tbl_rdy && w < 200000) begin @(posedge clk); w = w + 1; end
            chk(u_tx.tbl_rdy, "T0 BAG table ready");
        end

        // ---- 配 10 条 VL，全部 BAG=1 ms / lmax=1471 ----
        cfg_vl(16'h0123, 16'd1024);
        cfg_vl(16'h0456, 16'd1025);
        cfg_vl(16'h0789, 16'd69);
        cfg_vl(16'h0161, 16'd161);
        cfg_vl(16'h0BC5, 16'd3001);
        cfg_vl(16'h0A01, 16'd2001);
        cfg_vl(16'h0A02, 16'd2002);
        cfg_vl(16'h0A03, 16'd2003);
        cfg_vl(16'h0A04, 16'd2004);
        // 端口 5 是保留端口（vlid=0），不配表；本台只跑 0..4 + 6..9 = 9 条 VL
        repeat (50) @(posedge clk);

        $display("");
        $display("======== Q1 TX aggregate throughput (9 VL / BAG=1 ms / 1471 B) ========");

        // ---- 主循环：每轮给每条 VL 补一条消息（端口空闲才写）----
        // 每轮 1 ms（= 125,000 拍）；在每轮 +40 µs 处把 9 条 VL 各写一条
        for (r = 0; r < ROUNDS; r = r + 1) begin
            wait_until(r * 125000 + 5000);
            for (p = 0; p < 10; p = p + 1) begin
                if (p != 5) send_big(p[NPW-1:0]);       // 端口 5 无 VL，跳过
            end
        end

        // 等最后几条发完（同样看 ppend，不看 tx_busy 的 pdone 锁存位）
        repeat (150000) @(posedge clk);          // 1.2 ms，足够最后 9 条各发一帧


        nvl = 0;
        for (ii = 0; ii < 9; ii = ii + 1) if (vcnt[ii] > 0) nvl = nvl + 1;

        if (int_cnt > 0 && nvl > 0) begin
            mbps = nvl * PLEN * 8.0 / ((int_sum * 1.0 / int_cnt) * 8.0 / 1000.0);
            $display("  [T1] A-net frames = %0d, VLs = %0d, interval samples = %0d, avg interval = %.3f us",
                     n_a, nvl, int_cnt, (int_sum * 1.0 / int_cnt) * 8.0 / 1000.0);
            for (ii = 0; ii < 9; ii = ii + 1)
                $display("        VL#%0d frames = %0d (interval samples %0d)", ii, vfr[ii], vcnt[ii]);
            $display("  [T1] aggregate payload throughput = %.1f Mbps (%0d VL concurrent, BAG = 1 ms)", mbps, nvl);
        end else begin
            mbps = 0.0;
            $display("  [T1] **no interval samples** (A-net frames=%0d)", n_a);
        end

        chk(nvl == 9, "T1 all 9 VLs participated");
        for (ii = 0; ii < 9; ii = ii + 1)
            chk(vfr[ii] >= ROUNDS, "T2 every VL sent >= ROUNDS");
        chk(mbps >= 100.0, "T3 throughput >= 100 Mbps");
        chk(mbps >= 9.0 * 11.5, "T4 reaches 9-VL bound");

        $display("================================================================");
        if (errs == 0) $display(">> PASS: TX aggregate throughput %.1f Mbps (9 VL x BAG=1 ms)", mbps);
        else           $display(">> FAIL: %0d item(s) failed (aggregate %.1f Mbps)", errs, mbps);
        $finish;
    end

    initial begin #400000000; $display(">> TIMEOUT n_a=%0d", n_a); $finish; end
endmodule
