// tb_tx_novl.v —— 端口 5（保留、无 VL）发消息 的行为
`timescale 1ns/1ps

module tb_tx_novl;

    reg clk = 1'b0, reset = 1'b1;
    always #4 clk = ~clk;                  // 125 MHz
    localparam integer TICK = 1000;        // 8 µs

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;

    reg  [31:0] tcnt = 0; reg [15:0] ts_r = 0;
    always @(posedge clk) begin
        if (reset) begin tcnt <= 0; ts_r <= 0; end
        else if (tcnt >= TICK-1) begin tcnt <= 0; ts_r <= ts_r + 1; end
        else tcnt <= tcnt + 1;
    end
    wire [15:0] ts_now = ts_r;

    // ---- 配置表（用预设初值；本台只靠"第 0 项全零"这一点）----
    wire        cfg_hit; wire [127:0] cfg_w; wire [15:0] cfg_res_vlid, cfg_vlid_req;
    wire        ir_lk_req = 1'b0; wire [15:0] rx_lk_vlid = 16'h0;
    wire [15:0] lk_vlid = ir_lk_req ? rx_lk_vlid : cfg_vlid_req;

    afdx_cfg #(.NENT(4096), .IDXW(12)) u_cfg (
        .clk(clk), .reset(reset),
        .wr(1'b0), .rd(1'b0), .reg_sel(4'd0), .wdata(32'h0), .rdata(),
        .vlid_i(lk_vlid), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(cfg_res_vlid), .hit_o(cfg_hit), .cfg_o(cfg_w)
    );

    // ---- 端口层 + 发送路径 ----
    reg        wr = 0; reg [3:0] pi = 0; reg [2:0] rsel = 0; reg [31:0] wd = 0;
    wire [31:0] rd;
    wire        txv, txsop, txeop, txrdy, tx_replay, tx_hold;
    wire [7:0]  txd;
    wire [15:0] tx_vlid, tx_dport;
    wire [31:0] tx_dip;
    wire [10:0] tx_len;
    wire [9:0]  tx_busy;
    wire        tx_drop;

    afdx_tx_ports #(.NP(10), .NPW(4), .AW(14)) u_txp (
        .clk(clk), .reset(reset),
        .wr(wr), .pi(pi), .reg_sel(rsel), .wdata(wd), .rdata(rd),
        .tx_valid(txv), .tx_ready(txrdy), .tx_sop(txsop), .tx_eop(txeop), .tx_data(txd),
        .tx_vlid(tx_vlid), .tx_dip(tx_dip), .tx_dport(tx_dport), .tx_len(tx_len),
        .tx_hold(tx_hold), .tx_busy(tx_busy), .tx_pend(),
        .tx_drop(tx_drop)
    );

    wire        fr_valid, fr_sop, fr_eop, fr_net;
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
        .bag_hold_o(), .msg_drop(tx_drop)
    );

    integer nfr = 0, ndrop = 0;
    always @(posedge clk) begin
        if (fr_valid && fr_sop) nfr = nfr + 1;
        if (tx_drop) ndrop = ndrop + 1;
    end

    task cpu_wr; input [3:0] p; input [2:0] r; input [31:0] d;
        begin
            @(negedge clk); wr = 1; pi = p; rsel = r; wd = d;
            @(posedge clk);
            @(negedge clk); wr = 0; rsel = 0;
        end
    endtask

    task send_msg; input [3:0] p; input integer nbytes;
        integer k;
        begin
            for (k = 0; k < (nbytes+3)/4; k = k + 1)
                cpu_wr(p, 3'd0, 32'hC0DE_0000 | k);
            cpu_wr(p, 3'd1, nbytes);
            cpu_wr(p, 3'd2, 32'd1);
        end
    endtask

    integer errs = 0, w, n0;
    reg [31:0] st;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    initial begin
        #200 reset = 0;
        repeat (20) @(posedge clk);
        begin : w_tbl
            w = 0;
            while (!u_tx.tbl_rdy && w < 200000) begin @(posedge clk); w = w + 1; end
            chk(u_tx.tbl_rdy, "T0 BAG 表就绪");
        end

        $display("");
        $display("======== 端口 5（保留、无 VL）发消息 ========");

        // ---- 1. 往端口 5 发 64 字节 ----
        n0 = nfr;
        send_msg(4'd5, 64);
        w = 0;
        while (ndrop == 0 && w < 200000) begin @(posedge clk); w = w + 1; end
        $display("  [dbg] 端口5：nfr=%0d ndrop=%0d", nfr - n0, ndrop);
        chk(nfr == n0,   "T1 ★端口 5 的消息**一帧都不发**（VLID 0 假命中不能用）");
        chk(ndrop >= 1,  "T2 ★判超长并抬起 msg_drop（体现为 tx_drop 脉冲）");

        // ---- 2. 端口必须被释放（不忙、不挂待发）----
        @(negedge clk); pi = 4'd5; rsel = 3'd5; #1 st = rd;
        $display("  [dbg] 端口5 STATUS = %08x", st);
        chk(st[0] === 1'b0 && st[2] === 1'b0, "T3 ★端口 5 被正常释放（不忙、无待发）");
        chk(st[3] === 1'b1, "T4 端口 5 记录『上一条超长被丢』（perr）");

        // ---- 3. 对照：端口 6（真 VL）照常能发 ----
        n0 = nfr;
        send_msg(4'd6, 8);
        w = 0;
        while ((nfr - n0) < 2 && w < 200000) begin @(posedge clk); w = w + 1; end  // A+B 两帧
        $display("  [dbg] 端口6：新起帧 = %0d", nfr - n0);
        chk((nfr - n0) >= 2, "T5 ★端口 5 的丢弃没有把发送通路带死（端口 6 照常发 A/B）");

        $display("============================================");
        if (errs == 0) $display(">> PASS: 无 VL 端口被安全丢弃，且不影响其它端口");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #100000000; $display(">> TIMEOUT nfr=%0d", nfr); $finish; end
endmodule
