// tb_afdx_top_app.v —— 赛题模板 AFDX_top 的应用侧默认配置端到端冒烟
`timescale 1ns/1ps

module tb_afdx_top_app #(
    parameter integer SOC_DLY   = 125000,   // 1 ms（BYPASS 下不起作用）
    parameter integer WAIT_MS   = 400000    // 每条消息等 3.2 ms
);

    reg clk = 0, reset = 1;
    always #4 clk = ~clk;                   // 125 MHz

    // ---- 模板 8 信号 + 旁带 ----
    reg  [7:0] rx_data  = 8'h00;
    reg        rx_valid = 1'b0;
    reg        rx_ready = 1'b1;
    reg  [15:0] app_vlid = 16'h0789;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;
    wire [3:0] tx_port;
    wire [15:0] tx_vlid_w;
    wire       err;

    // ---- 其它们 ----
    wire [7:0] gmii_a_txd, gmii_b_txd;
    wire       gmii_a_txen, gmii_a_txer, gmii_a_gtxc;
    wire       gmii_b_txen, gmii_b_txer, gmii_b_gtxc;
    wire       mdc_a, mdc_b, phy_rstb_a, phy_rstb_b;
    wire       mdio_a, mdio_b;
    wire       fl_sclk, fl_mosi, fl_cs_n;
    wire       fl_miso = 1'b1;              // 无 flash 模型 → 读回全 1（无魔数，保留预设表）
    wire [3:0] debug;
    wire       mac_cfg_done, soc_boot_done;
    pullup (mdio_a);
    pullup (mdio_b);

    AFDX_top #(.PORTS(2), .SOC_BOOT_DLY(SOC_DLY), .BYPASS(1), .APP_SIDE(1), .VLID_MODE(1)) u_dut (
        .app_vlid(app_vlid), .tx_port(tx_port), .tx_vlid(tx_vlid_w), .err(err),
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_ready), .rx_ready(rx_ready),
        .gmii_a_txd(gmii_a_txd), .gmii_a_txen(gmii_a_txen), .gmii_a_txer(gmii_a_txer),
        .gmii_a_gtxc(gmii_a_gtxc), .gmii_a_rxd(8'h00), .gmii_a_rxdv(1'b0),
        .gmii_a_rxer(1'b0), .gmii_a_rxc(clk),
        .gmii_b_txd(gmii_b_txd), .gmii_b_txen(gmii_b_txen), .gmii_b_txer(gmii_b_txer),
        .gmii_b_gtxc(gmii_b_gtxc), .gmii_b_rxd(8'h00), .gmii_b_rxdv(1'b0),
        .gmii_b_rxer(1'b0), .gmii_b_rxc(clk),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(fl_miso), .flash_cs_n(fl_cs_n),
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done)
    );

    // ================= 应用口输出：按 tx_valid 包络切"消息" =================
    reg  [7:0] segbuf [0:4095];
    integer    segstart [0:15];
    integer    seglen  [0:15];
    reg  [3:0] segport [0:15];

    integer widx = 0, nseg = 0;
    reg     in_seg = 0;


    reg seg_ours = 0;
    always @(posedge clk) begin
        if (tx_valid) begin
            if (!in_seg) begin
                in_seg      <= 1'b1;
                seg_ours    <= (tx_vlid_w === 16'h0789);
                if (tx_vlid_w === 16'h0789) begin
                    segstart[nseg] <= widx;
                    segport[nseg]  <= tx_port;
                end
            end
            segbuf[widx] <= tx_data;
            widx <= widx + 1;
        end else if (in_seg) begin
            if (seg_ours) begin
                seglen[nseg] <= widx - segstart[nseg];
                nseg   <= nseg + 1;
            end
            in_seg <= 1'b0;
        end
    end

    // ================= 应用口输入：一条连续 valid 段 = 一条消息 =================
    task send_app;
        input [7:0] b0;
        input [7:0] b1;
        input [7:0] b2;
        input [7:0] b3;
        input [7:0] b4;
        input integer n;
        begin
            @(posedge clk); @(negedge clk);
            rx_data = b0; rx_valid = 1'b1;
            @(posedge clk); @(negedge clk);
            if (n > 1) begin rx_data = b1; @(posedge clk); @(negedge clk); end
            if (n > 2) begin rx_data = b2; @(posedge clk); @(negedge clk); end
            if (n > 3) begin rx_data = b3; @(posedge clk); @(negedge clk); end
            if (n > 4) begin rx_data = b4; @(posedge clk); @(negedge clk); end
            rx_valid = 1'b0;
        end
    endtask

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    integer tmo;
    task wait_seg; input integer want;   // 等到至少 want 条完整消息
        begin
            tmo = 0;
            while ((nseg < want) && (tmo < WAIT_MS)) begin
                @(posedge clk); tmo = tmo + 1;
            end
        end
    endtask

    // ---- 诊断计数器（只在本台用层次引用，不改 RTL）----
    integer n_txf = 0, n_acc = 0, n_cmt = 0, n_abr_in = 0;
    always @(posedge clk) if (u_dut.u_es.fr_valid && u_dut.u_es.fr_sop) n_txf = n_txf + 1;
    always @(posedge clk) if (u_dut.u_es.u_soc.ir_accept) n_acc = n_acc + 1;
    always @(posedge clk) if (u_dut.u_es.u_soc.rm_commit) n_cmt = n_cmt + 1;
    always @(posedge clk)
        if (u_dut.u_es.u_soc.u_appbr.m_valid && u_dut.u_es.u_soc.u_appbr.m_sop) n_abr_in = n_abr_in + 1;

    integer k;
    initial begin
        #200 reset = 1'b0;                      // BYPASS=1：core 立即放行 SoC
        repeat (100) @(posedge clk);            // 让 CPU 起来一点点（不依赖它）

        // ---- 第 1 条："AB"（2 字节，端口 2）----
        send_app(8'h41, 8'h42, 0, 0, 0, 2);
        wait_seg(1);

        // ---- 第 2 条："HELLO"（5 字节，端口 2）----
        send_app(8'h48, 8'h45, 8'h4C, 8'h4C, 8'h4F, 5);
        wait_seg(2);

        $display("  [dbg] nseg=%0d widx=%0d err=%b txf=%0d acc=%0d cmt=%0d abr_in=%0d",
                 nseg, widx, err, n_txf, n_acc, n_cmt, n_abr_in);
        $display("  [dbg3] txp_valid=%b txp_sop=%b txp_pend=%b txp_ready=%b msg_len=%0d tbl_rdy=%b len_bad=%b go=%b cfg_is_cur=%b lmax=%0d",
                 u_dut.u_es.u_soc.u_msgmux.txp_valid, u_dut.u_es.u_soc.u_msgmux.txp_sop,
                 u_dut.u_es.u_soc.u_msgmux.txp_pend, u_dut.u_es.u_soc.u_msgmux.txp_ready,
                 u_dut.u_es.u_soc.u_msgmux.msg_len, u_dut.u_es.u_soc.u_tx.tbl_rdy,
                 u_dut.u_es.u_soc.u_tx.len_bad, u_dut.u_es.u_soc.u_tx.go_int,
                 u_dut.u_es.u_soc.u_tx.cfg_is_cur, u_dut.u_es.u_soc.u_tx.lmax_w);
        $display("  [dbg2] bst=%0d bld_idle=%b msg_rdy=%b pay_hold=%b muxcur=%0d sst=%0d ca_i=%0d vli_q=%0d vli=%0d nxtcan=%b msg_v=%b msg_sop=%b",
                 u_dut.u_es.u_soc.u_tx.u_build.st, u_dut.u_es.u_soc.u_tx.u_build.idle_o,
                 u_dut.u_es.u_soc.u_tx.u_build.msg_ready, u_dut.u_es.u_soc.u_tx.u_build.pay_hold,
                 u_dut.u_es.u_soc.u_msgmux.cur, u_dut.u_es.u_soc.u_appbr.sst, u_dut.u_es.u_soc.u_appbr.ca_i,
                 u_dut.u_es.u_soc.u_tx.vli_q, u_dut.u_es.u_soc.u_tx.vli,
                 u_dut.u_es.u_soc.u_tx.nxt_can_q,
                 u_dut.u_es.u_soc.u_msgmux.msg_valid, u_dut.u_es.u_soc.u_msgmux.msg_sop);

        chk(nseg == 2, "T1 应用口共收到 2 条回环消息（A/B 副本已被 RM 去重）");
        chk(seglen[0] == 2 && segbuf[segstart[0]+0] === 8'h41 &&
            segbuf[segstart[0]+1] === 8'h42, "T2 第 1 条 = \"AB\"（2 字节）");
        chk(segport[0] == 4'd2, "T3 第 1 条 tx_port = 2");
        chk(seglen[1] == 5 && segbuf[segstart[1]+0] === 8'h48 &&
            segbuf[segstart[1]+1] === 8'h45 && segbuf[segstart[1]+2] === 8'h4C &&
            segbuf[segstart[1]+3] === 8'h4C && segbuf[segstart[1]+4] === 8'h4F,
            "T4 第 2 条 = \"HELLO\"（5 字节）");
        chk(segport[1] == 4'd2, "T5 第 2 条 tx_port = 2");
        chk(err === 1'b0, "T6 err = 0");
        chk(soc_boot_done === 1'b1, "T7 SoC 已放行（BYPASS=1 不等 MAC）");

        $display("-----------------------------------------------------");
        if (errs == 0) $display(">> PASS: AFDX_top 默认配置（BYPASS=1/APP_SIDE=1）内部回环端到端");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #40000000;
        $display(">> TIMEOUT: nseg=%0d widx=%0d err=%b", nseg, widx, err);
        $finish;
    end

endmodule
