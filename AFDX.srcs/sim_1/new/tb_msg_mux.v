// tb_msg_mux.v —— 发送消息流源仲裁的单元测试
`timescale 1ns/1ps

module tb_msg_mux;
    reg clk = 0, reset = 1;
    always #4 clk = ~clk;

    reg        tv = 0, tsop = 0, teop = 0; reg [7:0] td = 0; reg [15:0] tvl = 0; reg [13:0] tln = 0;
    reg        tpend = 0;
    wire       tready;
    reg        av = 0, asop = 0, aeop = 0; reg [7:0] ad = 0; reg [15:0] avl = 0; reg [13:0] aln = 0;
    wire       aready;
    reg        payhold = 0, mready = 1, mdrop = 0;
    wire       mv, msop, meop; wire [7:0] md; wire [15:0] mvl; wire [13:0] mln;
    wire       owner_txp, owner_app;

    afdx_msg_mux dut (
        .clk(clk), .reset(reset),
        .txp_valid(tv), .txp_sop(tsop), .txp_eop(teop), .txp_data(td),
        .txp_vlid(tvl), .txp_len(tln), .txp_pend(tpend), .txp_ready(tready),
        .app_valid(av), .app_sop(asop), .app_eop(aeop), .app_data(ad),
        .app_vlid(avl), .app_len(aln), .app_ready(aready),
        .pay_hold(payhold),
        .msg_valid(mv), .msg_sop(msop), .msg_eop(meop), .msg_data(md),
        .msg_vlid(mvl), .msg_len(mln), .msg_ready(mready), .msg_drop(mdrop),
        .owner_txp(owner_txp), .owner_app(owner_app)
    );

    integer errs = 0, w;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    initial begin
        #50 reset = 0;
        repeat (5) @(posedge clk);
        $display("");
        $display("======== 消息流源仲裁（afdx_msg_mux）========");

        // ---- 01 两源都有效（不握手）→ 端口层优先 ----
        @(negedge clk); mready = 0;
        tpend = 1; tv = 1; tsop = 1; teop = 1; td = 8'hAA; tvl = 16'h0123; tln = 1;   // 1 字节消息
        av = 1; asop = 1; aeop = 0; ad = 8'h55; avl = 16'h0456; aln = 5;
        repeat (2) @(posedge clk);
        chk(mv && owner_txp && (md == 8'hAA), "01 两源同时有效 → 端口层优先（数据 = AA）");

        // ---- 02 端口层这条**整条收完**（sop+eop 同拍）→ 释放；应用侧随即拿到口 ----
        @(negedge clk); mready = 1;                          // 接受 txp 的单字节消息（sop=eop=1）
        @(posedge clk);
        @(negedge clk); tpend = 0; tv = 0; tsop = 0; teop = 0;
        repeat (3) @(posedge clk);
        $display("  [dbg02] cur=%0d mv=%b md=%02x owner_app=%b", dut.cur, mv, md, owner_app);
        chk(mv && owner_app && (md == 8'h55), "02 端口层消息收完后释放 → 应用侧拿到消息口（55）");

        // ---- 03 应用消息期内，端口层再来活 → 不换源 ----
        @(negedge clk); tpend = 1; tv = 1; tsop = 1; teop = 0; td = 8'hBB;
        repeat (3) @(posedge clk);
        chk(mv && owner_app && (md == 8'h55), "03 应用消息期内端口层再来活也不换源（仍是 55）");

        // ---- 04 应用侧收尾（eop）→ 释放 → 端口层拿到口 ----
        @(negedge clk); asop = 0; aeop = 1;
        @(posedge clk);
        @(negedge clk); av = 0; aeop = 0;
        repeat (4) @(posedge clk);
        chk(mv && owner_txp && (md == 8'hBB), "04 eop 后释放 → 端口层消息拿到消息口（BB）");


        @(negedge clk); teop = 1;                            // 让端口层这条也收完
        @(posedge clk);
        @(negedge clk); tpend = 0; tv = 0; tsop = 0; teop = 0;
        repeat (4) @(posedge clk);
        @(negedge clk); av = 1; asop = 1; aeop = 0; ad = 8'h77; avl = 16'h0456; aln = 100;
        repeat (3) @(posedge clk);
        $display("  [dbg05] cur=%0d mv=%b md=%02x owner_app=%b", dut.cur, mv, md, owner_app);
        chk(mv && owner_app, "05a 应用侧消息开始（77）");
        // 组帧器判超长那拍：桥立刻退出发送态（m_valid 落），所以要**同拍**把 av 撤掉
        @(negedge clk); mdrop = 1; av = 0; asop = 0; aeop = 0;
        @(posedge clk); @(negedge clk); mdrop = 0;
        repeat (4) @(posedge clk);
        @(negedge clk); tpend = 1; tv = 1; tsop = 1; teop = 0; td = 8'hCC;
        repeat (4) @(posedge clk);
        chk(mv && owner_txp && (md == 8'hCC), "05b ★drop 之后消息口被释放（端口层照常发 CC）");


        @(negedge clk); tpend = 1; tv = 0; tsop = 0; teop = 0; av = 0; ad = 8'h00;
        repeat (1100000) @(posedge clk);          // 超过 20'hFFFFF 拍
        @(negedge clk); tpend = 0; av = 1; asop = 1; aeop = 1; ad = 8'hEE; avl = 16'h0456; aln = 1;
        repeat (6) @(posedge clk);
        chk(owner_app && mv && (md === 8'hEE), "06 stuck source times out");
        @(negedge clk); av = 0; asop = 0; aeop = 0;


        @(negedge clk); tpend = 1; tv = 0; tsop = 0; teop = 0;
        repeat (1100000) @(posedge clk);          // 触发超时并置 mask
        @(negedge clk); mready = 0;               // 先不接，观察它是否被选上
        tv = 1; tsop = 1; teop = 1; td = 8'hDD; tvl = 16'h0123; tln = 1;
        repeat (4) @(posedge clk);
        chk(owner_txp && mv && (md === 8'hDD), "07 mask cleared on valid");
        @(negedge clk); mready = 1;               // 收掉，免得影响结尾
        @(posedge clk);
        @(negedge clk); tpend = 0; tv = 0; tsop = 0; teop = 0;

        $display("============================================");
        if (errs == 0) $display(">> PASS: 源仲裁（优先级/锁定/释放/丢弃释放）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #32000000; $display(">> TIMEOUT"); $finish; end
endmodule
