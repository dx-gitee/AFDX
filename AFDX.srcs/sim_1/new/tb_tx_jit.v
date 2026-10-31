// tb_tx_jit.v —— 多 VL 并发发送的抖动上界（ARINC 664 P7 §3.2.4.3）
`timescale 1ns/1ps

module tb_tx_jit;

    // ---- 参数 ----
    localparam NROUND      = 12;      // 跑多少轮
    localparam TICK_CYCLES = 20;      // 1 个时基刻度 = 20 拍（真实系统 1 刻度 = 8 µs @12.5MHz，这里压缩时间）
    localparam BAG_TICKS   = 125;     // bag_shift=0 → 125 刻度 × 8 µs = **1 ms**
    localparam BAG_CYC     = BAG_TICKS * TICK_CYCLES;   // 2500 拍
    // 标准上界（换算成"拍"）：60.16 µs / 8 µs × 20 拍 = 150.4 拍
    localparam JIT_MAX_CYC = 151;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;            // 100 ns 周期（与其它台一致）

    // ---- 时基 ----
    reg [31:0] tickc = 0;
    reg [15:0] ts_now = 0;
    reg        ts_jump = 0;
    always @(posedge clk) begin
        if (reset) begin tickc <= 0; ts_now <= 0; end
        else if (ts_jump) begin ts_jump <= 1'b0; ts_now <= 16'h9000; end
        else if (tickc >= TICK_CYCLES-1) begin tickc <= 0; ts_now <= ts_now + 1; end
        else tickc <= tickc + 1;
    end

    // ---- CPU 侧（写端口层）----
    reg         wr = 0;
    reg  [2:0]  pi = 0;
    reg  [2:0]  reg_sel = 0;
    reg  [31:0] wdata = 0;
    wire [31:0] rdata;
    localparam R_DATA=0, R_LEN=1, R_CTRL=2, R_ST=5;

    // ---- 端口层 → 发送路径 ----
    wire        txv, txsop, txeop, txrdy, tx_replay, tx_hold, tx_drop;
    wire [7:0]  txd;
    wire [15:0] tx_vlid, tx_dport;
    wire [31:0] tx_dip;
    wire [10:0] tx_len;
    wire [4:0]  tx_busy_w;

    // ---- 查表 ----
    reg         cfg_wr = 0, cfg_rd = 0;
    reg  [3:0]  cfg_rs = 0;
    reg  [31:0] cfg_wd = 0;
    wire [31:0] cfg_rv;
    wire [15:0] tx_cfg_vlid, cfg_res_vlid;
    wire        tx_hit;
    wire [127:0] tx_cfg;

    // ---- 帧输出 ----
    wire        fr_valid, fr_sop, fr_eop, fr_net;
    wire        fr_ready = 1'b1;     // 本台不反压（反压另见 tb_tx_build 的阶段 2）
    wire [7:0]  fr_data;
    wire        tbl_rdy;

    // ================= DUT =================
    afdx_tx_ports #(.NP(5), .NPW(3), .AW(13)) u_ports (
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
        .cfg_vlid_o(tx_cfg_vlid), .cfg_hit_i(tx_hit), .cfg_i(tx_cfg),
        .cfg_res_vlid_i(cfg_res_vlid),
        .fr_valid(fr_valid), .fr_ready(fr_ready), .fr_sop(fr_sop),
        .fr_eop(fr_eop), .fr_data(fr_data), .fr_net(fr_net)
    );

    afdx_cfg #(.NENT(4096), .IDXW(12)) u_cfg (
        .clk(clk), .reset(reset), .wr(cfg_wr), .rd(cfg_rd), .reg_sel(cfg_rs),
        .wdata(cfg_wd), .rdata(cfg_rv),
        .vlid_i(tx_cfg_vlid), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(cfg_res_vlid), .hit_o(tx_hit), .cfg_o(tx_cfg)
    );

    assign tbl_rdy = u_tx.tbl_rdy;


    integer cyc = 0;
    integer last [0:2];          // 各 VL 上次帧起始的拍
    integer minv [0:2], maxv [0:2], cnt [0:2];
    integer k;
    integer fpos = 0;            // 帧内字节偏移（只在被接受的拍上前进，与组帧器的 pos 同步）
    reg  [7:0] sn_exp [0:2];     // 各 VL 期望的下一个 SN
    integer sn_bad = 0;          // SN 不符的帧数
    integer nfr3 = 0;            // 端口 3（VL 0x0161，本台从未用过）的帧数
    integer nfr4 = 0;            // 端口 4（VL 0x0BC5，预设 lmax=64）的帧数

    reg  [15:0] vl_sop = 16'h0;
    integer sn_frames = 0;       // 检查过的帧数（自证用例非空）
    initial begin
        for (k = 0; k < 3; k = k + 1) begin
            last[k] = -1; minv[k] = 0; maxv[k] = 0; cnt[k] = 0; sn_exp[k] = 8'd0;
        end
    end

    always @(posedge clk) if (fr_valid && fr_sop && (u_tx.u_build.cfg_vlid == 16'h0161)) nfr3 = nfr3 + 1;
    always @(posedge clk) if (fr_valid && fr_sop && (u_tx.u_build.cfg_vlid == 16'h0BC5)) nfr4 = nfr4 + 1;
    integer w3 = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (fr_valid && fr_ready) fpos = fr_sop ? 0 : (fpos + 1);
        if (fr_valid && fr_sop) begin
            vl_sop = u_tx.u_build.cfg_vlid;
            case (u_tx.u_build.cfg_vlid)
                16'h0123: k = 0;
                16'h0456: k = 1;
                default:  k = 2;                 // 0x0789
            endcase
            if (last[k] >= 0) begin
                if (minv[k] == 0 || (cyc - last[k]) < minv[k]) minv[k] = cyc - last[k];
                if ((cyc - last[k]) > maxv[k]) maxv[k] = cyc - last[k];
            end
            cnt[k] = cnt[k] + 1;
            last[k] = cyc;
        end

        if (fr_valid && fr_ready && (fpos == 59)) begin
            case (vl_sop)
                16'h0123: k = 0;
                16'h0456: k = 1;
                default:  k = 2;
            endcase
            sn_frames = sn_frames + 1;
            if (fr_data !== sn_exp[k][7:0]) begin
                sn_bad = sn_bad + 1;
                $display("  [sn] cyc=%0d VL=%04x frame#%0d SN=%02x expect %02x",
                         cyc, vl_sop, cnt[k], fr_data, sn_exp[k]);
            end
            sn_exp[k] = (sn_exp[k] == 8'd255) ? 8'd1 : (sn_exp[k] + 8'd1);
        end
    end

    // ================= 工具 =================
    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask

    task cfg_write;
        input [11:0] idx;
        input [31:0] w0, w1, w2, w3;
        begin
            @(negedge clk); cfg_wr=1; cfg_rs=4'd0; cfg_wd={20'h0, idx};
            @(negedge clk); cfg_rs=4'd1; cfg_wd=w0;
            @(negedge clk);            cfg_wd=w1;
            @(negedge clk);            cfg_wd=w2;
            @(negedge clk);            cfg_wd=w3;
            @(negedge clk); cfg_wr=0; cfg_rs=0; cfg_wd=0;
        end
    endtask

    // 往某端口提交一条 8 字节消息（DATA×2 → LEN → CTRL）
    task send_msg;
        input [2:0] p; input [7:0] base;
        begin
            @(negedge clk); wr=1; pi=p; reg_sel=R_DATA; wdata={base+8'd3,base+8'd2,base+8'd1,base};
            @(posedge clk);
            @(negedge clk);            wdata={8'h0,8'h0,8'h0,base+8'd4};
            @(posedge clk);
            @(negedge clk); reg_sel=R_LEN;  wdata=32'd8;
            @(posedge clk);
            @(negedge clk); reg_sel=R_CTRL; wdata=32'd1;
            @(posedge clk);
            @(negedge clk); wr=0; reg_sel=0; wdata=0;
            @(posedge clk);
        end
    endtask

    // 提交一条**任意长度**的消息（只需长度正确 —— Lmax 校验只看 msg_len）
    task send_msg_n;
        input [2:0] p; input integer n;
        integer q, nw;
        begin
            nw = (n + 3) / 4;
            for (q = 0; q < nw; q = q + 1) begin
                @(negedge clk); wr=1; pi=p; reg_sel=R_DATA; wdata=32'hA5A5A5A5;
                @(posedge clk);
            end
            @(negedge clk); reg_sel=R_LEN;  wdata=n;
            @(posedge clk);
            @(negedge clk); reg_sel=R_CTRL; wdata=32'd1;
            @(posedge clk);
            @(negedge clk); wr=0; reg_sel=0; wdata=0;
            @(posedge clk);
        end
    endtask

    integer r, w;
    real jit0, jit1, jit2;

    initial begin
        #200 reset = 0;
        repeat(4) @(posedge clk);
        $display("");
        $display("======= Multi-VL concurrent TX jitter (P7 3.2.4.3) =======");
        $display(" 3 VLs all BAG=1ms (%0d cyc), submitted together each round -> 3 pending on one TX channel", BAG_CYC);

        // ---- 配 3 条 VL：net_en=1（仅 A，每条消息只出一帧，便于量间隔）----
        //   w0 = rm_en<<30 | ic_en<<29 | net_en<<27 | lmax<<16 | vlid
        //   w1 = dport<<16 | skew<<4 | bag_shift   (skew = 5ms = 625 刻度)
        //   w2 = port_type<<20 | buf_idx<<16 | sport
        cfg_write(12'h123, 32'hEDBF0123, 32'h04002710, 32'h00100123, 32'hE0E00123);
        cfg_write(12'h456, 32'hEDBF0456, 32'h04012710, 32'h00200456, 32'hE0E00456);
        cfg_write(12'h789, 32'hEDBF0789, 32'h00042710, 32'h00300789, 32'hE0E00789);
        repeat(4) @(posedge clk);


        w = 0;
        while (!tbl_rdy && w < 100000) begin @(posedge clk); w = w + 1; end
        chk(tbl_rdy, "T0 BAG table cleared");

        // ---- 跑 NROUND 轮：每轮同时提交 3 条 ----
        for (r = 0; r < NROUND; r = r + 1) begin
            send_msg(3'd0, 8'hA0);
            send_msg(3'd1, 8'hB0);
            send_msg(3'd2, 8'hC0);
            // 等三条都起帧（帧计数都到 r+1）
            w = 0;
            while (!(cnt[0] >= r+1 && cnt[1] >= r+1 && cnt[2] >= r+1) && w < 200000) begin
                @(posedge clk); w = w + 1;
            end
            repeat(250) @(posedge clk);      // 让最后一条帧发完（64 字节 ≈ 70 拍）+ 端口回空闲
        end

        // ---- 结果 ----
        $display("");
        $display(" [VL 0x0123] frames=%0d  interval min=%0d max=%0d  jitter=%0d cyc (%.2f us)",
                 cnt[0], minv[0], maxv[0], maxv[0]-BAG_CYC,
                 (maxv[0]-BAG_CYC)*8.0/TICK_CYCLES);
        $display(" [VL 0x0456] frames=%0d  interval min=%0d max=%0d  jitter=%0d cyc (%.2f us)",
                 cnt[1], minv[1], maxv[1], maxv[1]-BAG_CYC,
                 (maxv[1]-BAG_CYC)*8.0/TICK_CYCLES);
        $display(" [VL 0x0789] frames=%0d  interval min=%0d max=%0d  jitter=%0d cyc (%.2f us)",
                 cnt[2], minv[2], maxv[2], maxv[2]-BAG_CYC,
                 (maxv[2]-BAG_CYC)*8.0/TICK_CYCLES);
        $display(" [bound] 40us + 3x(20+64)x8/1e8 = 60.16 us = %0d cyc", JIT_MAX_CYC);

        chk(cnt[0] >= NROUND && cnt[1] >= NROUND && cnt[2] >= NROUND,
            "T1 all 3 VLs scheduled");

        chk(minv[0] >= BAG_CYC, "T2 VL0x0123 interval >= BAG");
        chk(minv[1] >= BAG_CYC, "T3 VL0x0456 interval >= BAG");
        chk(minv[2] >= BAG_CYC, "T4 VL0x0789 interval >= BAG");

        chk((maxv[0]-BAG_CYC) <= JIT_MAX_CYC, "T5 jitter VL0x0123 <= bound");
        chk((maxv[1]-BAG_CYC) <= JIT_MAX_CYC, "T6 jitter VL0x0456 <= bound");
        chk((maxv[2]-BAG_CYC) <= JIT_MAX_CYC, "T7 jitter VL0x0789 <= bound");
        // 三条 VL 的抖动都应该远小于"各自独占"的情形 —— 至少证明并发确实发生了
        chk(maxv[1] > BAG_CYC || maxv[2] > BAG_CYC,
            "T8 concurrency really happened");

        $display(" [SN] checked %0d frames (3 VLs total), mismatch = %0d", sn_frames, sn_bad);
        chk(sn_frames >= 3*NROUND, "T9 SN check covers all frames");
        chk(sn_bad == 0,           "T10 SN +1 per frame, no skip");


        ts_jump = 1'b1;
        w3 = 0;
        while ((ts_now != 16'h9000) && w3 < 1000) begin @(posedge clk); w3 = w3 + 1; end
        repeat(4) @(posedge clk);
        send_msg(3'd3, 8'hD0);                // 端口 3 → VL 0x0161，本台从未发过
        w3 = 0;
        while ((nfr3 == 0) && w3 < 200000) begin @(posedge clk); w3 = w3 + 1; end
        $display(" [BAG first frame] at ts_now=0x%04x submit first frame of unused VL(0x0161) -> frames=%0d",
                 ts_now, nfr3);
        chk(nfr3 >= 1, "T11 first frame at ts>=0x8000");


        nfr4 = 0;
        send_msg_n(3'd4, 16);                 // 端口 4：16 字节（< 64，合法）
        w3 = 0;
        while ((nfr4 == 0) && w3 < 200000) begin @(posedge clk); w3 = w3 + 1; end
        nfr3 = 0;
        send_msg_n(3'd3, 128);                // 端口 3：128 字节（< 1471，合法）
        w3 = 0;
        while ((nfr3 == 0) && w3 < 200000) begin @(posedge clk); w3 = w3 + 1; end
        $display(" [Lmax attrib] port4(lmax=64) frames=%0d -> port3 128B msg frames=%0d", nfr4, nfr3);
        chk(nfr4 >= 1, "T12a port4 16B msg sent");
        chk(nfr3 >= 1, "T12b port3 128B msg sent");

        $display("==============================================");
        if (errs == 0) $display(">> PASS: multi-VL concurrent TX: BAG independence & jitter bound OK");
        else           $display(">> FAIL: %0d item(s) failed", errs);
        $finish;
    end

    initial begin #40000000; $display(">> TIMEOUT cnt=%0d/%0d/%0d", cnt[0],cnt[1],cnt[2]); $finish; end

endmodule
