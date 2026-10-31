// tb_app_bridge.v —— 应用侧桥的单元测试
`timescale 1ns/1ps

module tb_app_bridge;
    reg clk = 0, reset = 1;
    always #4 clk = ~clk;                  // 125 MHz

    reg        a_v = 0; reg [7:0] a_d = 0; reg [3:0] ap_in = 0;   // 接到 DUT.app_port（可置 X 模拟未驱动）
    reg  [3:0] a_p = 0;
    wire       t_v; wire [7:0] t_d; wire [3:0] t_p; wire err;
    wire       m_v, m_r; wire m_sop, m_eop; wire [7:0] m_d; wire [15:0] m_vlid; wire [13:0] m_len; wire m_busy;
    reg        m_ready = 1;
    reg        r_v = 0, r_sop = 0, r_eop = 0; reg [7:0] r_d = 0; reg [3:0] r_p = 0;
    reg [15:0] app_vlid = 16'h0;      // 本 TB 跑的是 VLID_MODE=0（旧端口语义）
    reg [15:0] r_vlid = 16'h0;                                    // 接收消息的 VLID
    reg        r_commit = 0, r_abort = 0;
    reg        r_vb = 1'b1;      // 判决对应的消息是否流出过字节（默认 1）

    afdx_app_bridge #(.NPW(4), .MAXLEN(1471), .APP_PORT_DEF(0)) dut (
        .clk(clk), .reset(reset),
        .app_rx_valid(a_v), .app_rx_data(a_d), .app_port(ap_in),
        .app_vlid(app_vlid),
        .app_tx_valid(t_v), .app_tx_data(t_d), .app_tx_port(t_p), .app_tx_vlid(), .err(err),
        .m_valid(m_v), .m_ready(m_ready), .m_sop(m_sop), .m_eop(m_eop),
        .m_data(m_d), .m_vlid(m_vlid), .m_len(m_len), .m_busy(m_busy),
        .r_valid(r_v), .r_sop(r_sop), .r_eop(r_eop), .r_data(r_d), .r_port(r_p),
        .r_commit(r_commit), .r_abort(r_abort), .r_vbytes(r_vb), .r_vlid(r_vlid)
    );

    // ---- 记录发往 tx_path 的消息 ----
    reg [7:0] mg [0:2047];
    integer   mn = 0, msg_cnt = 0;
    reg [15:0] g_vlid; reg [13:0] g_len;
    always @(posedge clk) begin
        if (m_v && m_ready) begin
            if (m_sop) begin mn = 0; g_vlid = m_vlid; g_len = m_len; msg_cnt = msg_cnt + 1; end
            mg[mn] = m_d; mn = mn + 1;
        end
    end

    // ---- 记录交付给应用的消息 ----
    reg [7:0] ag [0:8191];
    integer   an = 0, app_cnt = 0;
    reg [3:0] g_tp;
    reg [15:0] g_tvl;
    always @(posedge clk) begin
        if (t_v) begin
            if (an == 0) begin g_tp = t_p; g_tvl = dut.app_tx_vlid; end
            ag[an] = t_d; an = an + 1;
            app_cnt = app_cnt + 1;
        end
    end


    reg        d_push_q = 0, xbad = 0;
    reg [2:0]  dtail_q  = 0;
    always @(posedge clk) begin
        d_push_q <= dut.d_push;
        if (dut.d_push) dtail_q <= dut.dtail;
        if (d_push_q &&
            ((^{dut.dkind[dtail_q], dut.dlen[dtail_q], dut.dbank[dtail_q],
                dut.dbyte[dtail_q], dut.dport[dtail_q]}) === 1'bx))
            xbad = 1'b1;
    end

    // err 是 DUT 的输出（粘滞位），TB 不能给它赋值 —— 用本地"看见过"标志
    reg err_seen = 0;
    always @(posedge clk) if (err) err_seen = 1;


    reg [3:0] dmax = 0;
    always @(posedge clk) if (dut.dcnt > dmax) dmax = dut.dcnt;


    integer n_conflict = 0;
    wire w_push_now = (dut.ist == 2'd1) && !a_v && !dut.idrop && (dut.ilen != 0);
    wire w_pop_now  = (dut.sst == 3'd4) &&
                      ((m_ready && (dut.ca_i == dut.plen_i - 1'b1)) || dut.m_drop);
    always @(posedge clk) if (!reset && w_push_now && w_pop_now) n_conflict = n_conflict + 1;

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    task push_msg; input integer nbytes; input [7:0] base; input [3:0] port;
        integer k;
        begin
            ap_in = port;
            for (k = 0; k < nbytes; k = k + 1) begin
                @(negedge clk); a_v = 1; a_d = base + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); a_v = 0;
        end
    endtask


    task reset_dut;
        begin
            @(negedge clk);
            reset = 1; r_v = 0; r_sop = 0; r_eop = 0; r_commit = 0; r_abort = 0; r_vb = 1'b1;
            repeat (4) @(posedge clk);
            @(negedge clk); reset = 0;
            repeat (4) @(posedge clk);
            an = 0; app_cnt = 0; err_seen = 0; dmax = 0;
        end
    endtask

    integer k, kk, n0, c0;
    reg [1:0] mode;

    initial begin
        #100 reset = 0;
        repeat (5) @(posedge clk);
        $display("");
        $display("======== 应用侧桥单元测试 ========");

        // ---------- 01 入向 5 字节，端口 0 ----------
        n0 = msg_cnt; err_seen = 0;
        push_msg(5, 8'hA0, 4'd0);
        repeat (20) @(posedge clk);
        $display("  [dbg] 01 mn=%0d msg_cnt=%0d vlid=%04x len=%0d bytes=%02x %02x %02x %02x %02x",
                 mn, msg_cnt, g_vlid, g_len, mg[0],mg[1],mg[2],mg[3],mg[4]);
        chk(msg_cnt == n0 + 1, "01a 一条消息被送出（m_v 流）");
        chk(g_len == 14'd5,    "01b 长度 = 5");
        chk(g_vlid == 16'h0123, "01c 端口 0 → VLID 0x0123");
        chk(mg[0] === 8'hA0 && mg[1] === 8'hA1 && mg[4] === 8'hA4, "01d 字节逐一对（A0..A4）");
        chk(err_seen == 1'b0, "01e 正常路径不置 err");

        // ---------- 02 端口 1 ----------
        push_msg(3, 8'hB0, 4'd1);
        repeat (20) @(posedge clk);
        chk(msg_cnt == n0 + 2, "02a 第二条消息送出");
        chk(g_vlid == 16'h0456, "02b 端口 1 → VLID 0x0456");

        // ---------- 03 app_port 为 X → 回落默认端口 0 ----------
        //   把 app_port 打成 X（模拟"上层根本没驱动这根线"）
        ap_in = 4'bxxxx;
        push_msg(2, 8'hC0, 4'd0);
        repeat (20) @(posedge clk);
        ap_in = 4'd0;
        $display("  [dbg] 03 vlid=%04x msg_cnt=%0d", g_vlid, msg_cnt);
        chk(msg_cnt == n0 + 3, "03a X 的 app_port 仍能发（不挂）");
        chk(g_vlid == 16'h0123, "03b X → 回落到 APP_PORT_DEF(0) 的 VLID 0x0123");

        // ---------- 04 超长（1472 > 1471）→ 丢弃 + err ----------
        c0 = msg_cnt; err_seen = 0;
        push_msg(1472, 8'h10, 4'd0);
        repeat (40) @(posedge clk);
        chk(msg_cnt == c0, "04a 超长消息**一条都不发**");
        chk(err_seen == 1'b1, "04b 超长置 err（粘滞）");


        reset_dut;                         // err 是粘滞位：先清掉 04 的超长错误
        m_ready = 0;                       // 让第一条挂在消息流上
        push_msg(8, 8'h20, 4'd0);          // 第一条：进入 S_PRES，等待 m_ready
        repeat (10) @(posedge clk);
        c0 = msg_cnt;
        push_msg(8, 8'h30, 4'd0);          // 第二条：应排进 FIFO（不再被丢）
        repeat (10) @(posedge clk);
        chk(err_seen == 1'b0, "05a busy: 2nd msg queued (no drop)");
        m_ready = 1;
        repeat (80) @(posedge clk);
        chk(msg_cnt == c0 + 2, "05b both msgs sent (FIFO)");
        chk(g_len == 14'd8 && mn == 8 && mg[0] === 8'h30,
            "05c 2nd msg sent after 1st, payload correct");

        // ---------- 06 出向：commit + 端口匹配 ----------
        an = 0; app_cnt = 0; ap_in = 4'd0;
        //   发一条 4 字节的帧给端口 0
        for (k = 0; k < 4; k = k + 1) begin
            @(negedge clk); r_v = 1; r_d = 8'hE0 + k[7:0]; r_sop = (k == 0); r_eop = (k == 3);
            @(posedge clk);
        end
        @(negedge clk); r_v = 0; r_sop = 0; r_eop = 0;
        repeat (3) @(posedge clk);
        @(negedge clk); r_p = 4'd0; r_commit = 1; @(posedge clk);
        @(negedge clk); r_commit = 0;
        repeat (30) @(posedge clk);
        $display("  [dbg] 06 app_cnt=%0d tp=%0d bytes=%02x %02x %02x %02x", app_cnt, g_tp, ag[0],ag[1],ag[2],ag[3]);
        chk(app_cnt == 4, "06a 交付 4 字节给应用");
        chk(ag[0] === 8'hE0 && ag[3] === 8'hE3, "06b 交付字节逐一对（E0..E3）");
        chk(g_tp == 4'd0, "06c app_tx_port = 0");


        an = 0; app_cnt = 0;
        for (k = 0; k < 5; k = k + 1) begin
            @(negedge clk); r_v = 1; r_d = 8'hA1 + k[7:0]; r_sop = (k == 0); r_eop = (k == 4);
            @(posedge clk);
        end
        @(negedge clk); r_v = 0; r_sop = 0; r_eop = 0;
        repeat (3) @(posedge clk);
        @(negedge clk); r_p = 4'd0; r_commit = 1; @(posedge clk);
        @(negedge clk); r_commit = 0;
        repeat (30) @(posedge clk);
        $display("  [dbg] 06d app_cnt=%0d bytes=%02x %02x %02x %02x %02x", app_cnt,
                 ag[0],ag[1],ag[2],ag[3],ag[4]);
        chk(app_cnt == 5, "06d 连续第二条出向消息交付 5 字节");
        chk(ag[0] === 8'hA1 && ag[1] === 8'hA2 && ag[2] === 8'hA3 &&
            ag[3] === 8'hA4 && ag[4] === 8'hA5,
            "06e 第二条消息首字不再错位（A1..A5）");

        // ---------- 07 出向端口不匹配 → 不交付 ----------
        an = 0; app_cnt = 0; r_p = 4'd3;
        for (k = 0; k < 4; k = k + 1) begin
            @(negedge clk); r_v = 1; r_d = 8'hF0 + k[7:0]; r_sop = (k == 0); r_eop = (k == 3);
            @(posedge clk);
        end
        @(negedge clk); r_v = 0; r_sop = 0; r_eop = 0;
        repeat (3) @(posedge clk);
        @(negedge clk); r_p = 4'd3; r_commit = 1; @(posedge clk);   // 端口 3 ≠ 应用端口 0
        @(negedge clk); r_commit = 0;
        repeat (30) @(posedge clk);
        chk(app_cnt == 0, "07 端口不匹配的消息不交付给应用");

        // ---------- 08 出向 abort → 不交付 ----------
        r_p = 4'd0;
        for (k = 0; k < 4; k = k + 1) begin
            @(negedge clk); r_v = 1; r_d = 8'hD0 + k[7:0]; r_sop = (k == 0); r_eop = (k == 3);
            @(posedge clk);
        end
        @(negedge clk); r_v = 0; r_sop = 0; r_eop = 0;
        repeat (3) @(posedge clk);
        @(negedge clk); r_p = 4'd0; r_abort = 1; @(posedge clk);
        @(negedge clk); r_abort = 0;
        repeat (30) @(posedge clk);
        chk(app_cnt == 0, "08 abort 的消息不交付");

        // ================= 09 边界：1 字节消息 =================
        n0 = msg_cnt;
        push_msg(1, 8'h7E, 4'd0);
        repeat (40) @(posedge clk);
        $display("  [dbg] 09 mn=%0d len=%0d", mn, g_len);
        chk(msg_cnt == n0 + 1 && g_len == 14'd1 && mg[0] === 8'h7E,
            "09 1 字节消息：长度 1、字节正确（最小边界）");

        // ================= 10 边界：最大 1471 字节 =================
        //   1471 = 367 字 + 3 字节 ⇒ **尾字不满**，读写两侧的尾字处理都走到
        n0 = msg_cnt;
        push_msg(1471, 8'h10, 4'd0);
        repeat (4000) @(posedge clk);
        $display("  [dbg] 10 mn=%0d len=%0d first=%02x last=%02x", mn, g_len,
                 mg[0], (mn>0)?mg[mn-1]:8'hxx);
        chk(msg_cnt == n0 + 1, "10a 1471 字节消息被送出");
        chk(g_len == 14'd1471, "10b 长度 = 1471（未截断）");
        chk(mn == 1471,         "10c 字节数 = 1471（一个不多一个不少）");
        chk(mg[0] === 8'h10 && mg[1470] === 8'hCE,
            "10d 首字节与末字节正确（尾字不满也正确）");

        // ================= 11 段后改 app_port 不生效（段首锁存）=================
        //   先按端口 0 推一条，**段结束立即**把 app_port 改成 2；
        //   期望：这条消息仍走端口 0 的 VLID（0x0123）
        n0 = msg_cnt;
        ap_in = 4'd0;
        @(negedge clk); a_v = 1; a_d = 8'hA7; @(posedge clk);
        @(negedge clk); a_v = 0; ap_in = 4'd2;    // 段一结束就改端口
        repeat (300) @(posedge clk);
        $display("  [dbg] 11 vlid=%04x msg_cnt=%0d", g_vlid, msg_cnt);
        chk(msg_cnt == n0 + 1 && g_vlid == 16'h0123,
            "11 段结束后改 app_port 不影响本条（段首锁存生效）");


        an = 0; app_cnt = 0; r_p = 4'd0; ap_in = 4'd0;
        // msgA = A1 A2
        @(negedge clk); r_v=1; r_d=8'hA1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hA2; r_sop=0; r_eop=1; @(posedge clk);
        // msgB = B1..B5（紧接着）
        @(negedge clk);        r_d=8'hB1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hB2; r_sop=0; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hB3; r_sop=0; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hB4; r_sop=0; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hB5; r_sop=0; r_eop=1; @(posedge clk);
        // msgC = C1 C2 C3
        @(negedge clk);        r_d=8'hC1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hC2; r_sop=0; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hC3; r_sop=0; r_eop=1; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        // 三条判决按序到达（间隔 4 拍）
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        $display("  [dbg] 12 app_cnt=%0d bytes=%02x %02x | %02x %02x %02x %02x %02x | %02x %02x %02x",
                 app_cnt, ag[0],ag[1], ag[2],ag[3],ag[4],ag[5],ag[6], ag[7],ag[8],ag[9]);
        chk(app_cnt == 10, "12a 背靠背三条消息共交付 10 字节");
        chk(ag[0]===8'hA1 && ag[1]===8'hA2 &&
            ag[2]===8'hB1 && ag[3]===8'hB2 && ag[4]===8'hB3 && ag[5]===8'hB4 && ag[6]===8'hB5 &&
            ag[7]===8'hC1 && ag[8]===8'hC2 && ag[9]===8'hC3,
            "12b 三条消息按序、逐字节正确（A/B/C）");


        an = 0; app_cnt = 0; ap_in = 4'd0;
        @(negedge clk); r_v=1; r_d=8'hD1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hD2; r_sop=0; r_eop=0; @(posedge clk);
        // E 紧接着来（首字节就带 sop）
        @(negedge clk);        r_d=8'hE1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hE2; r_sop=0; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hE3; r_sop=0; r_eop=1; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);

        r_vb = 1'b1;
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        $display("  [dbg] 13 app_cnt=%0d bytes=%02x %02x %02x", app_cnt, ag[0],ag[1],ag[2]);
        chk(app_cnt == 3 && ag[0]===8'hE1 && ag[1]===8'hE2 && ag[2]===8'hE3,
            "13 半条后插入的新帧正常交付（E1..E3），半条被丢");


        an = 0; app_cnt = 0; r_p = 4'd0; ap_in = 4'd0; r_vb = 1'b1;
        @(negedge clk); r_v=1; r_d=8'hF1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hF2; r_sop=0; r_eop=1; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (3) @(posedge clk);
        r_vb = 1'b0;                       // 无字节判决（应被忽略）
        @(negedge clk); r_abort=1; @(posedge clk); @(negedge clk); r_abort=0;
        repeat (2) @(posedge clk);
        r_vb = 1'b1;
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (40) @(posedge clk);
        $display("  [dbg] 14 app_cnt=%0d bytes=%02x %02x", app_cnt, ag[0], ag[1]);
        chk(app_cnt == 2 && ag[0]===8'hF1 && ag[1]===8'hF2,
            "14 无字节判决被忽略，已捕获消息仍正常交付");


        an = 0; app_cnt = 0; ap_in = 4'd0;
        @(negedge clk); r_v=1; r_d=8'hC1; r_sop=1; r_eop=1; @(posedge clk);
        @(negedge clk);        r_d=8'hC2; r_sop=1; r_eop=1; @(posedge clk);
        @(negedge clk);        r_d=8'hC3; r_sop=1; r_eop=1; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (40) @(posedge clk);
        $display("  [dbg] 15 app_cnt=%0d bytes=%02x %02x %02x", app_cnt, ag[0],ag[1],ag[2]);
        chk(app_cnt == 3 && ag[0]===8'hC1 && ag[1]===8'hC2 && ag[2]===8'hC3,
            "15 三条 1 字节消息背靠背全部按序交付（不占 bank）");


        an = 0; app_cnt = 0; ap_in = 4'd0;
        for (k = 0; k < 3000; k = k + 1) begin
            @(negedge clk); r_v=1; r_d=k[7:0]; r_sop=(k==0); r_eop=(k==2999);
            @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (10000) @(posedge clk);
        $display("  [dbg] 16 app_cnt=%0d first=%02x last=%02x", app_cnt, ag[0], ag[2999]);
        chk(app_cnt == 3000 && ag[0]===8'h00 && ag[2999]===8'hB7,
            "16 3000 字节消息完整交付（8 KB bank，无绕回）");


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        for (k = 0; k < 10; k = k + 1) begin
            @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h10 + k[7:0];
            @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        for (k = 0; k < 10; k = k + 1) begin      // 10 条判决按序到达
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end
        repeat (80) @(posedge clk);
        $display("  [dbg] 17 app_cnt=%0d bytes=%02x %02x %02x %02x err=%b",
                 app_cnt, ag[0], ag[1], ag[2], ag[3], err_seen);
        chk(err_seen === 1'b1, "17a queue full sets err");
        chk(app_cnt === 8,     "17b only 8 enqueued/delivered");
        chk(ag[0]===8'h10 && ag[7]===8'h17, "17c payload in order");


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        for (k = 0; k < 10; k = k + 1) begin
            @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h30 + k[7:0];
            @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (8*65536 + 5000) @(posedge clk);      // 等 8 条全部超时放弃
        $display("  [dbg] 18 timeout: v_abandon=%0d v_drop=%0d vcnt=%0d",
                 dut.v_abandon, dut.v_drop, dut.vcnt);
        chk(dut.v_abandon != 0, "18a abandon debt recorded");
        for (k = 0; k < 10; k = k + 1) begin         // 10 条迟到判决
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end

        @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d=8'h55; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        $display("  [dbg] 18 end: app_cnt=%0d byte=%02x v_abandon=%0d",
                 app_cnt, ag[0], dut.v_abandon);
        chk(app_cnt === 1 && ag[0]===8'h55, "18b new msg delivered");


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        for (k = 0; k < 3; k = k + 1) begin
            @(negedge clk); r_v=1; r_sop=(k==0); r_eop=(k==2); r_d = 8'h60 + k[7:0];
            @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        ap_in = 4'd1;
        repeat (2) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        chk(app_cnt === 3 && ag[0]===8'h60, "19 kept after port switch");
        chk(xbad === 1'b0, "20 d-queue no X");
        ap_in = 4'd0;


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        for (k = 0; k < 8193; k = k + 1) begin
            @(negedge clk); r_v=1; r_d = k[7:0]; r_sop=(k==0); r_eop=(k==8192);
            @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (200) @(posedge clk);
        chk(err_seen === 1'b1, "21a overflow sets err");
        chk(app_cnt === 0,     "21b over-long not delivered");


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d=8'hA1; @(posedge clk);   // A: sop 不 eop
        @(negedge clk);         r_sop=0;          r_d=8'hA2; @(posedge clk);   // A 第 2 字节 → C_WR
        @(negedge clk);         r_sop=1; r_eop=1; r_d=8'hB1; @(posedge clk);   // B: 1 字节完整帧
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        $display("  [dbg] 22 vcnt=%0d vkind=%0d vdrop=%0d", dut.vcnt, dut.vkind[dut.vhead], dut.v_drop);
        chk(dut.vcnt === 4'd1, "22a half frame takes no slot");

        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        chk(app_cnt === 1 && ag[0] === 8'hB1, "22b verdict pairs real frame");


        reset_dut;
        for (k = 0; k < 3000; k = k + 1) begin
            @(negedge clk); r_v=1; r_d=k[7:0]; r_sop=(k==0); r_eop=(k==2999);
            @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (12) @(posedge clk);           // presenter 已进入呈现
        for (k = 0; k < 9; k = k + 1) begin   // 呈现窗口内连来 9 条 1 字节消息 + 判决
            @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h30 + k[7:0]; @(posedge clk);
            @(negedge clk); r_v=0; r_sop=0; r_eop=0;
            repeat (2) @(posedge clk);
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end
        repeat (4200) @(posedge clk);
        $display("  [dbg] 25 app_cnt=%0d b3000=%02x b3007=%02x dmax=%0d err=%b",
                 app_cnt, ag[3000], ag[3007], dmax, err_seen);
        chk(app_cnt === 3008, "25a d-full: 3000+8 delivered");
        chk(ag[3000]===8'h30 && ag[3007]===8'h37, "25b d-full: 8 shorts in order");
        chk(dmax <= 4'd8, "25c d-full: dcnt<=8");
        chk(err_seen === 1'b1, "25d d-full: err set");


        reset_dut;
        for (k = 0; k < 10; k = k + 1) begin        // 8 条入队 + 2 条记欠账
            @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h40 + k[7:0]; @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        for (k = 0; k < 2; k = k + 1) begin          // 前 2 条判决 → 弹出 2 条
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end
        // 新帧 Y：有欠账 ⇒ 必须**不入队**（否则它会被别人的判决弹掉/被别人交付）
        @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h59; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (2) @(posedge clk);
        for (k = 0; k < 9; k = k + 1) begin          // 剩 6 条 + 2 条欠账 + Y 的判决
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end
        repeat (60) @(posedge clk);
        $display("  [dbg] 26 app_cnt=%0d last=%02x vdrop=%0d", app_cnt, ag[8], dut.v_drop);
        chk(app_cnt === 8, "26a vdrop: only 8 delivered");
        chk(ag[0]===8'h40 && ag[7]===8'h47, "26b vdrop: 8 msgs in order");
        chk(dut.v_drop === 8'd0, "26c vdrop: debt cleared in order");

        @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d=8'h5A; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        $display("  [dbg] 26z app_cnt=%0d ag8=%02x", app_cnt, ag[8]);
        chk(app_cnt === 9 && ag[8] === 8'h5A, "26d vdrop: later msg delivered");


        reset_dut;
        @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d=8'h71; @(posedge clk);
        @(negedge clk);         r_sop=0; r_eop=0; r_d=8'h72; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;

        @(negedge clk);
        while (!((dut.cst === 2'd0) && (dut.cst_tmo === 16'hFFFF))) @(negedge clk);

        r_v=1; r_sop=1; r_eop=0; r_d=8'h73; @(posedge clk);
        @(negedge clk);         r_sop=0; r_eop=0; r_d=8'h74; @(posedge clk);
        @(negedge clk);         r_sop=0; r_eop=1; r_d=8'h75; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        $display("  [dbg] 29 app_cnt=%0d bank=%b ag0=%02x", app_cnt, dut.bank_used, ag[0]);
        chk(app_cnt === 3 && ag[0] === 8'h73, "29a abandon: new frame ok");
        chk(dut.bank_used === 4'b0000, "29b cst_abandon: no bank leak");


        reset_dut;
        @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d=8'h81; @(posedge clk);
        @(negedge clk);         r_sop=0; r_eop=0; r_d=8'h82; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (66000) @(posedge clk);          // > 65536：cst_tmo 触发，X 被放弃
        @(negedge clk); r_v=1; r_sop=0; r_eop=0; r_d=8'h83; @(posedge clk);   // X 的剩余字节恢复
        @(negedge clk);         r_sop=0; r_eop=1; r_d=8'h84; @(posedge clk);   // X 的 eop（无 sop）
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d=8'h5B; @(posedge clk);   // Y：1 字节完整帧
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (3) @(posedge clk);
        @(negedge clk); r_abort=1; @(posedge clk); @(negedge clk); r_abort=0; // X 的判决（abort）
        repeat (3) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0; // Y 的判决
        repeat (60) @(posedge clk);
        $display("  [dbg] 30 app_cnt=%0d ag0=%02x vcnt=%0d", app_cnt, ag[0], dut.vcnt);
        chk(app_cnt === 1 && ag[0] === 8'h5B, "30 resume: X aborted, Y ok");


        reset_dut;
        for (k = 0; k < 9; k = k + 1) begin
            @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h70 + k[7:0]; @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        for (k = 0; k < 8; k = k + 1) begin
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end
        @(negedge clk);
        while (!((dut.vcnt === 4'd0) && (dut.v_drop === 8'd1) && (dut.v_tmo === 16'hFFFF))) @(negedge clk);
        // 这一拍 v_tmo 已是 0xFFFF ⇒ 下一个 posedge 就是"超时清账"那一拍：同拍送一帧
        r_v=1; r_sop=1; r_eop=1; r_d=8'h6C; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (2) @(posedge clk);
        $display("  [dbg] 31 v_drop=%0d vcnt=%0d", dut.v_drop, dut.vcnt);

        chk(dut.v_drop === 8'd0 && dut.vcnt === 4'd1, "31 tmo clears debt, keeps new frame");
        reset_dut;


        reset_dut;
        m_ready = 0;
        for (k = 0; k < 8; k = k + 1) push_msg(4, 8'h40 + k[7:0], 4'd0);
        repeat (20) @(posedge clk);
        $display("  [dbg] 34 wcnt=%0d err=%b", dut.wcnt, err_seen);
        chk(err_seen === 1'b0 && dut.wcnt === 4'd8, "34a 8 msgs queued");
        push_msg(4, 8'h5F, 4'd0);
        repeat (20) @(posedge clk);
        chk(err_seen === 1'b1 && dut.wcnt === 4'd8, "34b 9th dropped+err");
        m_ready = 1;
        repeat (300) @(posedge clk);
        chk(dut.wcnt === 4'd0, "34c FIFO drained");
        reset_dut;


        reset_dut;
        for (k = 0; k < 8; k = k + 1) begin
            @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h70 + k[7:0]; @(posedge clk);
        end
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        // 第 9 条：队满 ⇒ 记 1 条欠账
        @(negedge clk); r_v=1; r_sop=1; r_eop=1; r_d = 8'h7F; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (4) @(posedge clk);
        // 给 8 条判决 ⇒ vcnt=0、v_drop=1
        for (k = 0; k < 8; k = k + 1) begin
            @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
            repeat (2) @(posedge clk);
        end
        @(negedge clk);
        while (!((dut.vcnt === 4'd0) && (dut.v_drop !== 8'd0) && (dut.v_tmo === 16'hFFFF))) @(negedge clk);

        r_v=1; r_sop=1; r_eop=1; r_d=8'h5C; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (2) @(posedge clk);
        $display("  [dbg] 35 vcnt=%0d v_drop=%0d", dut.vcnt, dut.v_drop);
        chk(dut.v_drop === 8'd0, "35a tmo cycle clears debt");
        chk(dut.vcnt === 4'd1, "35b new frame accepted at tmo cycle");
        // 它的判决必须交付它自己
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        chk(app_cnt >= 1 && ag[app_cnt-1] === 8'h5C, "35c new frame delivered");
        reset_dut;


        reset_dut;
        for (k = 0; k < 4; k = k + 1) begin          // 4 条 8 字节，4 个 bank 全占满
            @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d = 8'hC0 + k[7:0]; @(posedge clk);
            for (kk = 1; kk < 7; kk = kk + 1) begin
                @(negedge clk); r_sop=0; r_d = 8'hC0 + kk[7:0]; @(posedge clk);
            end
            @(negedge clk); r_eop=1; r_d=8'hC7; @(posedge clk);
            @(negedge clk); r_eop=0;
        end
        repeat (4) @(posedge clk);
        @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d=8'hD0; @(posedge clk);   // 第 5 帧只给 sop
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (10) @(posedge clk);
        $display("  [dbg] 27 vcnt=%0d cst=%0d bank=%b", dut.vcnt, dut.cst, dut.bank_used);
        chk(dut.vcnt === 4'd4, "27 no orphan slot");
        reset_dut;


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        // 4 条 8 字节消息、不给判决 ⇒ 4 个 bank 全占满
        for (k = 0; k < 4; k = k + 1) begin
            @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d = 8'hC0 + k[7:0];
            @(posedge clk);
            for (kk = 1; kk < 7; kk = kk + 1) begin
                @(negedge clk); r_sop=0; r_d = 8'hC0 + kk[7:0]; @(posedge clk);
            end
            @(negedge clk); r_eop=1; r_d=8'hC7; @(posedge clk);
            @(negedge clk); r_eop=0;
        end
        repeat (4) @(posedge clk);
        // 第 5 帧：只给 sop（没有 eop）→ 无空 bank → tombstone + C_DRAIN，然后就"断了"
        @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d=8'hD0; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (10) @(posedge clk);
        $display("  [dbg] 23 before tmo: cst=%0d bank=%b", dut.cst, dut.bank_used);
        chk(dut.cst === 2'd2, "23a entered C_DRAIN");
        repeat (70000) @(posedge clk);          // > 65536 拍
        $display("  [dbg] 23 after tmo: cst=%0d err=%b", dut.cst, err_seen);
        chk(dut.cst === 2'd0, "23b truncated frame times out");
        chk(err_seen === 1'b1, "23c timeout sets err");


        an = 0; app_cnt = 0; err_seen = 0; ap_in = 4'd0; r_p = 4'd0;
        for (k = 0; k < 4; k = k + 1) begin          // 4 条 8 字节，4 个 bank 占满
            @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d = 8'hE0 + k[7:0];
            @(posedge clk);
            for (kk = 1; kk < 7; kk = kk + 1) begin
                @(negedge clk); r_sop=0; r_d = 8'hE0 + kk[7:0]; @(posedge clk);
            end
            @(negedge clk); r_eop=1; r_d=8'hE7; @(posedge clk);
            @(negedge clk); r_eop=0;
        end
        repeat (4) @(posedge clk);
        @(negedge clk); r_v=1; r_sop=1; r_eop=0; r_d=8'hF0; @(posedge clk);   // 第 5 帧 sop → C_DRAIN
        repeat (10) @(posedge clk);
        chk(dut.cst === 2'd2, "24a entered C_DRAIN");

        for (k = 0; k < 70000; k = k + 1) begin
            @(negedge clk); r_sop=0; r_eop=0; r_d = k[7:0];
            @(posedge clk);
        end
        $display("  [dbg] 24 after 70000 valid-only cycles: cst=%0d err=%b", dut.cst, err_seen);
        chk(dut.cst === 2'd0, "24b timeout despite valid high");
        chk(err_seen === 1'b1, "24c timeout sets err");
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;


        begin : t36
            integer w36, d36, c36;
            an = 0; app_cnt = 0; ap_in = 4'd0;
            @(negedge clk); r_v = 0; r_sop = 0; r_eop = 0; r_commit = 0;
            repeat (6) @(posedge clk);

            for (d36 = 0; d36 < 32 && (dut.vcnt != 0 || dut.v_drop != 0); d36 = d36 + 1) begin
                @(negedge clk); r_p = 4'd0; r_commit = 1; @(posedge clk);
                @(negedge clk); r_commit = 0; repeat (3) @(posedge clk);
            end
            $display("  [dbg] 36 pre-drain: vcnt=%0d v_drop=%0d v_abandon=%0d (loops=%0d)",
                     dut.vcnt, dut.v_drop, dut.v_abandon, d36);
            chk(dut.vcnt === 0 && dut.v_drop === 0, "36z pre-drain clean");
            // 起一帧：sop 字节 0x11、第二字节 0x22，然后断流
            @(negedge clk); r_v = 1; r_sop = 1; r_eop = 0; r_d = 8'h11; @(posedge clk);
            @(negedge clk); r_sop = 0; r_d = 8'h22; @(posedge clk);
            @(negedge clk); r_v = 0; r_d = 8'h00;

            w36 = 0;
            @(negedge clk);
            while (!((dut.cst_abandon === 1'b1) && (dut.cst === 2'd1)) && (w36 < 70000)) begin
                @(negedge clk); w36 = w36 + 1;
            end
            $display("  [dbg] 36 aband&WR after %0d cyc: cst=%0d tmo=%0d aband=%b bank_used=%b wbank=%0d vcnt=%0d v_drop=%0d v_accept=%b elen=%0d",
                     w36, dut.cst, dut.cst_tmo, dut.cst_abandon, dut.bank_used, dut.wbank,
                     dut.vcnt, dut.v_drop, dut.v_accept, dut.elen);
            // 当前就在这个 negedge 上 ⇒ 直接驱动，下一拍（abandon=1 那一拍）就能看到
            r_v = 1; r_sop = 0; r_eop = 1; r_d = 8'h33; @(posedge clk);
            $display("  [dbg] 36 eop-cycle: cst=%0d bank_used=%b wbank=%0d vtail=%0d vhead=%0d",
                     dut.cst, dut.bank_used, dut.wbank, dut.vtail, dut.vhead);
            @(negedge clk); r_v = 0; r_eop = 0;
            repeat (3) @(posedge clk);
            $display("  [dbg] 36 after eop: cst=%0d wbank=%0d bank_used=%b vcnt=%0d sel_hit=%b wsel=%0d sel_port=%0d vkind0=%0d vbank0=%0d",
                     dut.cst, dut.wbank, dut.bank_used, dut.vcnt,
                     dut.sel_hit, dut.wsel, dut.sel_port, dut.vkind[0], dut.vbank[0]);
            chk(dut.cst === 2'd0, "36a frame completes (returns to C_IDLE)");
            chk(dut.bank_used[dut.wbank] === 1'b1, "36b bank still held by captured entry");

            for (k = 0; k < 4; k = k + 1) begin
                @(negedge clk); r_v = 1; r_sop = 1; r_eop = 0; r_d = 8'h70 + k[7:0]; @(posedge clk);
                @(negedge clk); r_sop = 0; r_eop = 1; r_d = 8'h71; @(posedge clk);
            end
            @(negedge clk); r_v = 0; r_sop = 0; r_eop = 0;
            repeat (4) @(posedge clk);
            $display("  [dbg] 36 after 4 more frames: bank_used=%b vcnt=%0d", dut.bank_used, dut.vcnt);

            c36 = app_cnt;
            @(negedge clk); r_p = 4'd0; r_commit = 1; @(posedge clk);
            @(negedge clk); r_commit = 0;
            repeat (30) @(posedge clk);
            $display("  [dbg] 36 app_cnt %0d -> %0d, tail=%02x %02x %02x", c36, app_cnt,
                     ag[app_cnt-3], ag[app_cnt-2], ag[app_cnt-1]);
            chk((app_cnt - c36) == 3, "36c delivered 3 bytes");
            chk(ag[app_cnt-3] === 8'h11 && ag[app_cnt-2] === 8'h22 && ag[app_cnt-1] === 8'h33,
                "36d payload 11/22/33");
        end


        begin : t37
            integer w37, c37;
            an = 0; app_cnt = 0; ap_in = 4'd0;
            repeat (6) @(posedge clk);
            for (c37 = 0; c37 < 32 && (dut.vcnt != 0 || dut.v_drop != 0); c37 = c37 + 1) begin
                @(negedge clk); r_p = 4'd0; r_commit = 1; @(posedge clk);
                @(negedge clk); r_commit = 0; repeat (3) @(posedge clk);
            end
            chk(dut.vcnt === 0 && dut.v_drop === 0, "37z pre-drain clean");
            // 起一条**截断帧**（只有 sop + 1 字节，没有 eop）
            @(negedge clk); r_v = 1; r_sop = 1; r_eop = 0; r_d = 8'h41; @(posedge clk);
            @(negedge clk); r_sop = 0; r_d = 8'h42; @(posedge clk);
            @(negedge clk); r_v = 0; r_d = 8'h00;
            // 等超时电平（cst_abandon 且仍在 C_WR）
            w37 = 0; @(negedge clk);
            while (!((dut.cst_abandon === 1'b1) && (dut.cst === 2'd1)) && (w37 < 70000)) begin
                @(negedge clk); w37 = w37 + 1;
            end
            $display("  [dbg] 37 timeout cycle reached after %0d cyc (cst=%0d)", w37, dut.cst);

            r_v = 1; r_sop = 1; r_eop = 0; r_d = 8'h51; @(posedge clk);
            @(negedge clk); r_sop = 0; r_d = 8'h52; @(posedge clk);
            @(negedge clk); r_d = 8'h53; @(posedge clk);
            @(negedge clk); r_eop = 1; r_d = 8'h54; @(posedge clk);
            @(negedge clk); r_v = 0; r_eop = 0;
            repeat (4) @(posedge clk);
            $display("  [dbg] 37 after: cst=%0d vcnt=%0d kind=%0d len=%0d",
                     dut.cst, dut.vcnt, dut.vkind[dut.vhead], dut.vlen[dut.vhead]);
            chk(dut.cst === 2'd0, "37a back to C_IDLE");
            chk(dut.vcnt === 1 && dut.vkind[dut.vhead] === 2'd0 && dut.vlen[dut.vhead] === 14'd4,
                "37b new frame captured (kind=0 len=4)");
            c37 = app_cnt;
            @(negedge clk); r_p = 4'd0; r_commit = 1; @(posedge clk);
            @(negedge clk); r_commit = 0; repeat (30) @(posedge clk);
            $display("  [dbg] 37 app_cnt %0d -> %0d", c37, app_cnt);
            chk((app_cnt - c37) == 4 && ag[app_cnt-4] === 8'h51 && ag[app_cnt-1] === 8'h54,
                "37c payload 51..54 delivered");
        end


        begin : c38
            integer d, c0_, m0_, hit, bad38;
            hit = 0; bad38 = 0;
            for (d = 0; d < 32; d = d + 1) begin
                reset_dut;
                m_ready = 1'b1;
                c0_ = n_conflict; m0_ = msg_cnt;
                push_msg(8, 8'hA0, 4'd1);              // 消息 1：8 字节
                repeat (d) @(posedge clk);             // 可调的相位差
                push_msg(4, 8'hB0, 4'd1);              // 消息 2：4 字节
                repeat (400) @(posedge clk);           // 足够两条都送完
                if (n_conflict > c0_) begin
                    hit = hit + 1;
                    $display("  [dbg] 38 d=%0d 命中同拍冲突，msg_cnt %0d -> %0d", d, m0_, msg_cnt);
                end
                if ((msg_cnt - m0_) != 2) begin
                    bad38 = bad38 + 1;
                    $display("  [dbg] 38 d=%0d 只送出 %0d 条（应为 2）", d, msg_cnt - m0_);
                end
            end
            $display("  [dbg] 38 命中窗口 %0d/32，丢消息窗口 %0d/32", hit, bad38);
            chk(hit > 0,  "38a same-cycle push+pop window reachable (probe)");
            chk(bad38 == 0, "38b every phase: both messages delivered");
        end

        $display("  [probe] wcnt 同拍 push+pop 冲突次数 = %0d", n_conflict);
        $display("==================================");
        if (errs == 0) $display(">> PASS: 应用侧桥（入向/端口映射/超长/忙丢/出向交付过滤/背靠背多消息/8KB bank）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end


    initial begin #15000000; $display(">> TIMEOUT"); $finish; end
endmodule
