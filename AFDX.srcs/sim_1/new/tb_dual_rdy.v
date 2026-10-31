// tb_dual_rdy.v —— A/B 双网发送独立性
`timescale 1ns/1ps

module tb_dual_rdy #(
    parameter integer SOC_DLY = 125000,
    parameter integer WAIT_CY = 400000      // 单次等待上限（3.2 ms）
) ();

    localparam [15:0] VL_T  = 16'h0250;     // 预置数据 VL（BAG = 1 ms）
    localparam integer NMSG = 64;

    reg clk = 1'b0;
    reg reset = 1'b1;
    always #4 clk = ~clk;                   // 125 MHz

    // ---------------------------------------------------------------
    // 应用侧 / 旁带
    // ---------------------------------------------------------------
    reg         tx_v = 0;
    reg  [7:0]  tx_d = 0;
    reg  [15:0] tx_vl = 0;
    wire        rx_v;
    wire [7:0]  rx_d;
    wire [15:0] rx_vl;
    wire [3:0]  rx_p;
    wire        err;
    wire [3:0]  debug;
    wire        maccfg, boot;

    wire [7:0]  ga_txd, gb_txd;
    wire        ga_txen, ga_txer, ga_gtxc;
    wire        gb_txen, gb_txer, gb_gtxc;
    wire        mdc_a, mdc_b;
    wire        mdio_a, mdio_b;
    wire        prst_a, prst_b;
    wire        fsclk, fmosi, fmiso, fcsn;

    AFDX_top #(
        .PORTS(2), .SOC_BOOT_DLY(SOC_DLY), .BYPASS(0), .VLID_MODE(1),
        .SRC_IP(32'h0A000001)
    ) u_dut (
        .clk(clk), .reset(reset),
        .rx_valid(tx_v), .rx_data(tx_d), .app_vlid(tx_vl),
        .tx_valid(rx_v), .tx_data(rx_d),
        .tx_ready(), .rx_ready(1'b1),          // 模板 8 线：答疑明确"取 1、可用不用"
        .tx_vlid(rx_vl), .tx_port(rx_p),
        .err(err),
        .gmii_a_txd(ga_txd), .gmii_a_txen(ga_txen), .gmii_a_txer(ga_txer),
        .gmii_a_gtxc(ga_gtxc),
        .gmii_a_rxd(8'h00), .gmii_a_rxdv(1'b0), .gmii_a_rxer(1'b0), .gmii_a_rxc(clk),
        .gmii_b_txd(gb_txd), .gmii_b_txen(gb_txen), .gmii_b_txer(gb_txer),
        .gmii_b_gtxc(gb_gtxc),
        .gmii_b_rxd(8'h00), .gmii_b_rxdv(1'b0), .gmii_b_rxer(1'b0), .gmii_b_rxc(clk),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(prst_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(prst_b),
        .flash_sclk(fsclk), .flash_mosi(fmosi), .flash_miso(fmiso), .flash_cs_n(fcsn),
        .debug(debug), .mac_cfg_done(maccfg), .soc_boot_done(boot)
    );

    // ---------------------------------------------------------------
    // 模型：两路 PHY + flash
    // ---------------------------------------------------------------
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a (.mdc(mdc_a), .mdio(mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b (.mdc(mdc_b), .mdio(mdio_b));
    pullup (mdio_a); pullup (mdio_b);

    spi_flash_model #(.AW(13), .USE_IMG(1), .IMG("flash.hex")) u_flash (
        .sclk(fsclk), .cs_n(fcsn), .mosi(fmosi), .miso(fmiso));

    // ---------------------------------------------------------------
    // A 网线侧抓帧（SFD 0xD5 之后取 64 字节；按 VL 计数）
    // ---------------------------------------------------------------
    reg [7:0] cap [0:63];
    integer   wp;
    reg       fcap;
    integer   n_vl;                          // 收到的"本用例 VL"帧数
    reg [7:0] cap_da [0:5];

    always @(posedge clk) begin
        if (reset) begin
            fcap <= 1'b0; wp <= 0; n_vl <= 0;
        end else if (ga_txen) begin
            if (!fcap) begin
                if (ga_txd == 8'hD5) begin fcap <= 1'b1; wp <= 0; end
            end else begin
                if (wp < 64) cap[wp] <= ga_txd;
                wp <= wp + 1;
            end
        end else if (fcap) begin
            fcap <= 1'b0;
            if (cap[4] === VL_T[15:8] && cap[5] === VL_T[7:0]) begin
                n_vl <= n_vl + 1;
                cap_da[0] <= cap[0]; cap_da[1] <= cap[1]; cap_da[2] <= cap[2];
                cap_da[3] <= cap[3]; cap_da[4] <= cap[4]; cap_da[5] <= cap[5];
            end
        end
    end

    // ---------------------------------------------------------------
    // 发送任务（一条连续 valid 段 = 一条消息；VLID 在段首拍给出）
    // ---------------------------------------------------------------
    integer k;
    task send_msg; input integer n; input [7:0] base; input [15:0] vl;
        begin
            @(negedge clk); tx_vl = vl;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); tx_v = 1'b1; tx_d = base + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); tx_v = 1'b0;
        end
    endtask

    task wait_nvl; input integer n; input integer tmax; output ok;
        integer w;
        begin
            w = 0; while ((n_vl < n) && (w < tmax)) begin @(posedge clk); w = w + 1; end
            ok = (n_vl >= n);
        end
    endtask

    // ---------------------------------------------------------------
    // 判定
    // ---------------------------------------------------------------
    integer errs = 0, nchk = 0;
    task chk; input cond; input [255:0] name;
        begin
            nchk = nchk + 1;
            if (cond) $display("  [ok]   T%0d %0s", nchk, name);
            else begin errs = errs + 1; $display("  [FAIL] T%0d %0s", nchk, name); end
        end
    endtask

    integer i, okw, n0;
    initial begin
        #200 reset = 0;
        $display("");
        $display("======== A/B 双网发送独立性（B 网 MAC 未就绪时 A 网必须照发）========");

        // ---- T1：启动完成 ----
        okw = 0;
        while ((!boot) && okw < 5000000) begin @(posedge clk); okw = okw + 1; end
        chk(okw < 5000000 && boot === 1'b1, "boot: both MAC configured");
        repeat (20000) @(posedge clk);          // 固件把端口/服务配好

        // ---- T2：基线（不 force）----
        n0 = n_vl;
        send_msg(NMSG, 8'hA0, VL_T);
        wait_nvl(n0 + 1, WAIT_CY, okw);
        $display("  [dbg] baseline   : n_vl=%0d (was %0d)", n_vl, n0);
        chk(okw, "baseline A-net frame");

        // ---- T3：把 B 壳内部的 ff_tx_rdy 按死成 0 → A 网仍须出帧 ----
        repeat (200000) @(posedge clk);          // 等 BAG(1 ms) 过完 + 让在途帧落地
        force u_dut.u_es.g_dual.u_mac_b.ff_tx_rdy = 1'b0;
        $display("  [dbg] forced B-shell ff_tx_rdy = 0");
        n0 = n_vl;
        send_msg(NMSG, 8'hB0, VL_T);
        wait_nvl(n0 + 1, WAIT_CY, okw);
        $display("  [dbg] B-stuck    : n_vl=%0d (was %0d) waited=%0d", n_vl, n0, okw);
        chk(okw, "A sends while B never rdy");

        // ---- T4：T3 那帧的 DA 仍是 03:00:00:00:02:50（没被"半发"截断）----
        $display("  [dbg] DA = %02x:%02x:%02x:%02x:%02x:%02x",
                 cap_da[0], cap_da[1], cap_da[2], cap_da[3], cap_da[4], cap_da[5]);
        chk(cap_da[0] === 8'h03 && cap_da[1] === 8'h00 && cap_da[2] === 8'h00 &&
            cap_da[3] === 8'h00 && cap_da[4] === VL_T[15:8] && cap_da[5] === VL_T[7:0],
            "frame DA 03:00:00:00:02:50");

        // ---- T5：释放 force → 双网整条通路恢复（还能再出帧）----
        release u_dut.u_es.g_dual.u_mac_b.ff_tx_rdy;
        $display("  [dbg] released B-shell ff_tx_rdy");
        repeat (200000) @(posedge clk);
        n0 = n_vl;
        send_msg(NMSG, 8'hC0, VL_T);
        wait_nvl(n0 + 1, WAIT_CY, okw);
        $display("  [dbg] recovered  : n_vl=%0d (was %0d)", n_vl, n0);
        chk(okw, "TX resumes after B rdy back");

        $display("==================================");
        if (errs == 0) $display(">> PASS: A/B 双网发送独立（B 未就绪不影响 A）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #40000000; $display(">> TIMEOUT"); $finish; end

endmodule
