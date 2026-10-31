// tb_bag_far.v —— BAG 的「隔得久也要能发」专项
`timescale 1ns/1ps
module tb_bag_far;

    localparam TICK_CYCLES = 2;
    localparam BAG_TICKS   = 125;       // bag_shift=0 → 125 刻度 = 1 ms
    localparam BAG_CYC     = BAG_TICKS * TICK_CYCLES;
    localparam FAR_TICKS   = 33000;
    localparam NP = 5, NPW = 3;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    // ---- 时基（连续走，不跳变）----
    reg [31:0] tickc = 0;
    reg [15:0] ts_now = 0;
    always @(posedge clk) begin
        if (reset) begin tickc <= 0; ts_now <= 0; end
        else if (tickc >= TICK_CYCLES-1) begin tickc <= 0; ts_now <= ts_now + 1; end
        else tickc <= tickc + 1;
    end

    // ---- CPU 侧 ----
    reg         wr = 0;
    reg  [2:0]  pi = 0;
    reg  [2:0]  reg_sel = 0;
    reg  [31:0] wdata = 0;
    wire [31:0] rdata;
    localparam R_DATA=0, R_LEN=1, R_CTRL=2;

    // ---- 端口层 → 发送路径 ----
    wire        txv, txsop, txeop, txrdy, tx_replay, tx_hold, tx_drop;
    wire [7:0]  txd;
    wire [15:0] tx_vlid, tx_dport;
    wire [31:0] tx_dip;
    wire [10:0] tx_len;
    wire [NP-1:0] tx_busy_w;
    wire        tx_hit;

    // ---- 查表 ----
    reg         cfg_wr = 0, cfg_rd = 0;
    reg  [3:0]  cfg_rs = 0;
    reg  [31:0] cfg_wd = 0;
    wire [31:0] cfg_rv;
    wire [15:0] tx_cfg_vlid16, cfg_res_vlid;
    wire [127:0] tx_cfg;

    // ---- 帧输出 ----
    wire        fr_valid, fr_sop, fr_eop, fr_net;
    wire [7:0]  fr_data;
    wire        fr_ready = 1'b1;

    afdx_tx_ports #(.NP(NP), .NPW(NPW), .AW(13)) u_ports (
        .clk(clk), .reset(reset),
        .wr(wr), .pi(pi), .reg_sel(reg_sel), .wdata(wdata), .rdata(rdata),
        .tx_valid(txv), .tx_ready(txrdy), .tx_sop(txsop), .tx_eop(txeop), .tx_data(txd),
        .tx_vlid(tx_vlid), .tx_dip(tx_dip), .tx_dport(tx_dport), .tx_len(tx_len),
        .tx_hold(tx_hold), .tx_busy(tx_busy_w), .tx_pend(),
        .tx_drop(tx_drop)
    );

    afdx_tx_path #(.NVL(4096), .NVLW(12)) u_tx (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .ov_dip(32'h0), .ov_dport(16'h0),
        .msg_valid(txv), .msg_ready(txrdy), .msg_sop(txsop), .msg_eop(txeop),
        .msg_data(txd), .msg_port(tx_vlid), .msg_len({3'b000, tx_len}),
        .tx_replay_o(tx_replay), .pay_hold_o(tx_hold), .bag_hold_o(),
        .msg_drop(tx_drop),
        .cfg_vlid_o(tx_cfg_vlid16), .cfg_hit_i(tx_hit), .cfg_i(tx_cfg),
        .cfg_res_vlid_i(cfg_res_vlid),
        .fr_valid(fr_valid), .fr_ready(fr_ready), .fr_sop(fr_sop),
        .fr_eop(fr_eop), .fr_data(fr_data), .fr_net(fr_net)
    );

    afdx_cfg #(.NENT(4096), .IDXW(12)) u_cfg (
        .clk(clk), .reset(reset), .wr(cfg_wr), .rd(cfg_rd), .reg_sel(cfg_rs),
        .wdata(cfg_wd), .rdata(cfg_rv),
        .vlid_i(tx_cfg_vlid16), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(cfg_res_vlid), .hit_o(tx_hit), .cfg_o(tx_cfg)
    );

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    // 帧计数 + 每帧起始的刻度
    integer nfr = 0;
    integer cyc = 0;
    integer last_start_tick = -1, this_start_tick = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (fr_valid && fr_sop) begin nfr = nfr + 1; last_start_tick = ts_now; end
    end

    task cfg_write;
        input [11:0] idx; input [31:0] w0, w1, w2, w3;
        begin
            @(negedge clk); cfg_wr=1; cfg_rs=4'd0; cfg_wd={20'h0, idx};
            @(negedge clk); cfg_rs=4'd1; cfg_wd=w0;
            @(negedge clk);             cfg_wd=w1;
            @(negedge clk);             cfg_wd=w2;
            @(negedge clk);             cfg_wd=w3;
            @(negedge clk); cfg_wr=0; cfg_rs=0; cfg_wd=0;
        end
    endtask


    task wait_idle;
        integer q;
        begin
            q = 0;
            while (((u_ports.act_vld && (u_ports.act == 3'd0)) || u_ports.ppend[0])
                   && q < 200000) begin @(posedge clk); q = q + 1; end
            repeat (4) @(posedge clk);
        end
    endtask

    // 提交一条 8 字节消息到端口 0（VL 0x0123）
    task send_msg_p;
        input [2:0] p;
        begin
            @(negedge clk); wr=1; pi=p; reg_sel=R_DATA; wdata=32'hA1A2A3A4; @(posedge clk);
            @(negedge clk);                                  wdata=32'hB1B2B3B4; @(posedge clk);
            @(negedge clk); reg_sel=R_LEN;  wdata=32'd8;                       @(posedge clk);
            @(negedge clk); reg_sel=R_CTRL; wdata=32'd1;                       @(posedge clk);
            @(negedge clk); wr=0; reg_sel=0; wdata=0;                          @(posedge clk);
        end
    endtask
    task send_msg; begin send_msg_p(3'd0); end endtask

    integer w, ref_nfr, nfr4;
    initial begin
        #200 reset = 0;
        repeat(6) @(posedge clk);
        $display("");
        $display("======== BAG：隔得久也必须能发（半区 32768 刻度）========");
        $display("  本台 1 刻度 = %0d 拍；BAG=1ms = %0d 刻度 = %0d 拍；推过 %0d 刻度",
                 TICK_CYCLES, BAG_TICKS, BAG_CYC, FAR_TICKS);

        // net_en = 1（仅 A）→ 一条消息只出一帧，便于数帧
        //   w0 = sn_vld<<31 | rm<<30 | ic<<29 | net_en<<27 | lmax<<16 | vlid
        cfg_write(12'h123, 32'hEDBF0123, 32'h04002710, 32'h00100123, 32'hE0E00123);
        cfg_write(12'h456, 32'hEDBF0456, 32'h04002710, 32'h00100456, 32'hE0E00456);
        repeat(4) @(posedge clk);

        // 等 BAG 表清零完成（复位后逐项清零，约 4098 拍）
        w = 0;
        while (!u_tx.tbl_rdy && w < 100000) begin @(posedge clk); w = w + 1; end
        chk(u_tx.tbl_rdy, "T0 BAG 表清零完成");

        // ---- T1：复位后的第一条立刻能发 ----
        send_msg;
        w = 0;
        while ((nfr == 0) && w < 100000) begin @(posedge clk); w = w + 1; end
        chk(nfr >= 1, "T1 ★复位后的首条消息立刻起帧（BAG 表清零后标志 = 可发）");

        this_start_tick = last_start_tick;

        // ---- T2：第二条必须等满一个 BAG ----
        wait_idle;
        ref_nfr = nfr;
        send_msg;
        w = 0;
        while ((nfr == ref_nfr) && w < 100000) begin @(posedge clk); w = w + 1; end
        $display("  [dbg] T2 后: tbl_rdy=%b can=%b nxt_q=%0d ts_now=%0d lk_vlid=%04x port_lat=%04x ppend0=%b act_vld=%b msgv=%b mrdy=%b",
                 u_tx.tbl_rdy, u_tx.nxt_can_q, u_tx.nxt_q, ts_now,
                 u_tx.lk_vlid, u_tx.port_lat, u_ports.ppend[0], u_ports.act_vld, txv, txrdy);
        chk(nfr == ref_nfr + 1, "T2 第二条也起来了（对照）");
        chk((((last_start_tick - this_start_tick) & 16'hFFFF) >= BAG_TICKS),
            "T2b 与上一帧的间隔 ≥ BAG（BAG 限速仍然成立）");


        wait_idle;
        ref_nfr = nfr;
        this_start_tick = last_start_tick;          // 以第 2 帧为基准
        repeat (FAR_TICKS * TICK_CYCLES) @(posedge clk);
        chk(nfr == ref_nfr, "T3 这段空闲里没有多余帧冒出来（对照：源没有消息就不会有帧）");

        wait_idle;
        send_msg;
        w = 0;
        while ((nfr == ref_nfr) && w < 100000) begin @(posedge clk); w = w + 1; end
        $display("  [info] 距上一帧 %0d 个刻度（半区 = 32768）时提交 → 帧数从 %0d 变成 %0d（等了 %0d 拍）",
                 (last_start_tick - this_start_tick) & 16'hFFFF, ref_nfr, nfr, w);
        chk(nfr == ref_nfr + 1,
            "T4 ★★距上一次发帧 > 262 ms 之后再发，帧**立刻**起来（旧实现要压到 ~524 ms 才发）");
        chk(w < 400, "T4b ★★而且只等了很少几拍（不是被 BAG 压后的那种等待）");


        wait_idle;
        nfr4 = nfr;                                  // 记录当前帧数
        send_msg_p(3'd1);                            // 端口 1 → VL 0x0456：把 port_lat 带走
        w = 0;
        while ((nfr == nfr4) && w < 100000) begin @(posedge clk); w = w + 1; end
        chk(nfr > nfr4, "T5 第二条 VL（0x0456）也能发（对照，说明 port_lat 会被它带走）");
        wait_idle;
        // 现在两条 VL 各有一次起帧，标志都是 0；把时基**连续**推过 33000 个刻度
        repeat (FAR_TICKS * TICK_CYCLES) @(posedge clk);
        nfr4 = nfr;
        send_msg_p(3'd0);                            // 回到 VL 0x0123（它的 port_lat 早就被带走了）
        w = 0;
        while ((nfr == nfr4) && w < 100000) begin @(posedge clk); w = w + 1; end
        $display("  [info] 多 VL 并存下时隔 %0d 刻度再发 VL 0x0123 → 等了 %0d 拍",
                 FAR_TICKS, w);
        chk(nfr == nfr4 + 1, "T6 ★★多 VL 并存时的稀疏 VL 也能**立刻**发（标志必须对所有 VL 都维护）");
        chk(w < 400, "T6b ★★而且只等了很少几拍");

        $display("====================================================");
        if (errs == 0) $display(">> PASS: BAG 在「隔得久」的路径上也正确（不再被半个时基周期误判）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #20000000; $display(">> TIMEOUT"); $finish; end
endmodule
