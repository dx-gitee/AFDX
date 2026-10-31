// tb_app_vlid.v —— 应用侧桥的 VLID 模式单元测试
`timescale 1ns/1ps

module tb_app_vlid;
    reg clk = 0, reset = 1;
    always #4 clk = ~clk;                  // 125 MHz

    reg        a_v = 0; reg [7:0] a_d = 0; reg [15:0] a_vlid = 0; reg [3:0] ap = 4'd0;
    wire       t_v; wire [7:0] t_d; wire [3:0] t_p; wire [15:0] t_vl; wire err;
    wire       m_v; reg m_ready = 1;
    wire       m_sop, m_eop; wire [7:0] m_d; wire [15:0] m_vlid; wire [13:0] m_len; wire m_busy;
    reg        r_v = 0, r_sop = 0, r_eop = 0; reg [7:0] r_d = 0; reg [3:0] r_p = 0;
    reg  [15:0] r_vlid = 0; reg r_commit = 0, r_abort = 0, r_vb = 1'b1;

    afdx_app_bridge #(.NPW(4), .MAXLEN(1471), .APP_PORT_DEF(0), .VLID_MODE(1)) dut (
        .clk(clk), .reset(reset),
        .app_rx_valid(a_v), .app_rx_data(a_d), .app_vlid(a_vlid), .app_port(ap),
        .app_tx_valid(t_v), .app_tx_data(t_d), .app_tx_port(t_p), .app_tx_vlid(t_vl),
        .err(err),
        .m_valid(m_v), .m_ready(m_ready), .m_sop(m_sop), .m_eop(m_eop),
        .m_data(m_d), .m_vlid(m_vlid), .m_len(m_len), .m_busy(m_busy),
        .r_valid(r_v), .r_sop(r_sop), .r_eop(r_eop), .r_data(r_d), .r_port(r_p),
        .r_commit(r_commit), .r_abort(r_abort), .r_vbytes(r_vb), .r_vlid(r_vlid)
    );

    // ---- 发往 tx_path 的消息流 ----
    reg [7:0] mg [0:2047];
    integer   mn = 0, msg_cnt = 0;
    reg [15:0] g_vlid;
    always @(posedge clk) if (m_v && m_ready) begin
        if (m_sop) begin mn = 0; g_vlid = m_vlid; msg_cnt = msg_cnt + 1; end
        mg[mn] = m_d; mn = mn + 1;
    end

    // ---- 交付给应用的消息 ----
    reg [7:0] ag [0:2047];
    integer   acnt = 0;
    reg [15:0] g_tvl;
    always @(posedge clk) if (t_v) begin
        if (acnt == 0) g_tvl = t_vl;
        ag[acnt] = t_d; acnt = acnt + 1;
    end

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask

    task push_msg; input integer nbytes; input [7:0] base; input [15:0] vl;
        integer k;
        begin
            a_vlid = vl;
            for (k = 0; k < nbytes; k = k + 1) begin
                @(negedge clk); a_v = 1; a_d = base + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); a_v = 0;
        end
    endtask

    integer k, n0;

    initial begin
        #100 reset = 0;
        repeat (5) @(posedge clk);
        $display("");
        $display("======== 应用侧桥 · VLID 模式 ========");

        // ---------- 01 按 VLID 注入 ----------
        n0 = msg_cnt;
        push_msg(5, 8'hA0, 16'h0456);
        repeat (40) @(posedge clk);
        $display("  [dbg] 01 cnt=%0d vlid=%04x len=%0d", msg_cnt, g_vlid, m_len);
        chk(msg_cnt == n0 + 1 && g_vlid === 16'h0456 && m_len === 14'd5, "01a VLID injection 0x0456");
        chk(mg[0] === 8'hA0 && mg[4] === 8'hA4, "01b bytes correct");

        // ---------- 02 换一个 VLID ----------
        n0 = msg_cnt;
        push_msg(3, 8'hB0, 16'h0201);
        repeat (40) @(posedge clk);
        chk(msg_cnt == n0 + 1 && g_vlid === 16'h0201 && m_len === 14'd3, "02 VLID 0x0201");

        // ---------- 03 app_vlid=0 → 回落端口映射 ----------
        //   本 TB 的 app_port 恒 0 ⇒ VLID 应为 0x0123
        n0 = msg_cnt;
        push_msg(2, 8'hC0, 16'h0000);
        repeat (40) @(posedge clk);
        $display("  [dbg] 03 cnt=%0d vlid=%04x sel=%04x", msg_cnt, g_vlid, dut.sel_vlid);
        chk(msg_cnt == n0 + 1 && g_vlid === 16'h0123, "03 VLID=0 falls back to port map");

        // ---------- 04 出向：端口不匹配也交付，并带回 VLID ----------
        acnt = 0;
        r_p = 4'd3;                       // 故意与应用侧端口 0 不匹配
        r_vlid = 16'h0A01;
        @(negedge clk); r_v=1; r_d=8'hD1; r_sop=1; r_eop=0; @(posedge clk);
        @(negedge clk);        r_d=8'hD2; r_sop=0; r_eop=1; @(posedge clk);
        @(negedge clk); r_v=0; r_sop=0; r_eop=0;
        repeat (3) @(posedge clk);
        @(negedge clk); r_commit=1; @(posedge clk); @(negedge clk); r_commit=0;
        repeat (60) @(posedge clk);
        $display("  [dbg] 04 acnt=%0d vlid=%04x", acnt, g_tvl);
        chk(acnt === 2 && ag[0] === 8'hD1 && ag[1] === 8'hD2, "04a delivered despite port mismatch");
        chk(g_tvl === 16'h0A01, "04b delivered VLID tagged");

        // ---------- 05 入向 FIFO 深度 ----------
        m_ready = 0;
        for (k = 0; k < 8; k = k + 1) push_msg(4, 8'h40 + k[7:0], 16'h0300 + k[15:0]);
        repeat (20) @(posedge clk);
        $display("  [dbg] 05 wcnt=%0d err=%b", dut.wcnt, err);
        chk(dut.wcnt === 4'd8, "05a 8 msgs queued");
        push_msg(4, 8'h5F, 16'h03FF);
        repeat (20) @(posedge clk);
        chk(err === 1'b1 && dut.wcnt === 4'd8, "05b 9th dropped+err");
        m_ready = 1;
        repeat (400) @(posedge clk);
        chk(dut.wcnt === 4'd0, "05c FIFO drained");

        $display("==================================");
        if (errs == 0) $display(">> PASS: 应用侧桥 VLID 模式（注入/交付/FIFO）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #20000000; $display(">> TIMEOUT"); $finish; end
endmodule
