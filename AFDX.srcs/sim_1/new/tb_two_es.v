// tb_two_es.v —— 两个端系统互发
`timescale 1ns/1ps

module tb_two_es #(
    parameter integer SOC_DLY = 125000,
    parameter integer WAIT_MS = 400000      // 单条消息等待上限（3.2 ms）
) ();

    localparam [15:0] VL_AB = 16'h0200;     // A → B
    localparam [15:0] VL_BA = 16'h0201;     // B → A
    localparam integer NMSG  = 64;          // 消息长度

    reg clk = 1'b0;
    reg reset = 1'b1;
    always #4 clk = ~clk;                   // 125 MHz


    reg [7:0] ga_to_b, gb_to_b, ga_to_a, gb_to_a;
    reg       ga_to_b_v, gb_to_b_v, ga_to_a_v, gb_to_a_v;

    // ---------------------------------------------------------------
    // 实例 A
    // ---------------------------------------------------------------
    reg         a_tx_v = 0;
    reg  [7:0]  a_tx_d = 0;
    reg  [15:0] a_tx_vl = 0;
    wire        a_rx_v;
    wire [7:0]  a_rx_d;
    wire [15:0] a_rx_vl;
    wire [3:0]  a_rx_p;
    wire        a_err;
    wire [3:0]  a_debug;
    wire        a_maccfg, a_boot;

    wire [7:0]  a_ga_txd, a_gb_txd;
    wire        a_ga_txen, a_ga_txer, a_ga_gtxc;
    wire        a_gb_txen, a_gb_txer, a_gb_gtxc;
    wire        a_mdc_a, a_mdc_b;
    wire        a_mdio_a, a_mdio_b;
    wire        a_prst_a, a_prst_b;
    wire        a_fsclk, a_fmosi, a_fmiso, a_fcsn;

    AFDX_top #(
        .PORTS(2), .SOC_BOOT_DLY(SOC_DLY), .BYPASS(0), .VLID_MODE(1),
        .SRC_IP(32'h0A000001)
    ) ua (
        .clk(clk), .reset(reset),
        .rx_valid(a_tx_v), .rx_data(a_tx_d), .app_vlid(a_tx_vl),
        .tx_valid(a_rx_v), .tx_data(a_rx_d),
        .tx_vlid(a_rx_vl), .tx_port(a_rx_p),
        .err(a_err),
        .gmii_a_txd(a_ga_txd), .gmii_a_txen(a_ga_txen), .gmii_a_txer(a_ga_txer),
        .gmii_a_gtxc(a_ga_gtxc),
        .gmii_a_rxd(ga_to_a), .gmii_a_rxdv(ga_to_a_v), .gmii_a_rxer(1'b0), .gmii_a_rxc(clk),
        .gmii_b_txd(a_gb_txd), .gmii_b_txen(a_gb_txen), .gmii_b_txer(a_gb_txer),
        .gmii_b_gtxc(a_gb_gtxc),
        .gmii_b_rxd(gb_to_a), .gmii_b_rxdv(gb_to_a_v), .gmii_b_rxer(1'b0), .gmii_b_rxc(clk),
        .mdc_a(a_mdc_a), .mdio_a(a_mdio_a), .phy_rstb_a(a_prst_a),
        .mdc_b(a_mdc_b), .mdio_b(a_mdio_b), .phy_rstb_b(a_prst_b),
        .flash_sclk(a_fsclk), .flash_mosi(a_fmosi), .flash_miso(a_fmiso), .flash_cs_n(a_fcsn),
        .debug(a_debug), .mac_cfg_done(a_maccfg), .soc_boot_done(a_boot)
    );

    // ---------------------------------------------------------------
    // 实例 B（除源 IP / flash 镜像外与 A 完全对称）
    // ---------------------------------------------------------------
    reg         b_tx_v = 0;
    reg  [7:0]  b_tx_d = 0;
    reg  [15:0] b_tx_vl = 0;
    wire        b_rx_v;
    wire [7:0]  b_rx_d;
    wire [15:0] b_rx_vl;
    wire [3:0]  b_rx_p;
    wire        b_err;
    wire [3:0]  b_debug;
    wire        b_maccfg, b_boot;

    wire [7:0]  b_ga_txd, b_gb_txd;
    wire        b_ga_txen, b_ga_txer, b_ga_gtxc;
    wire        b_gb_txen, b_gb_txer, b_gb_gtxc;
    wire        b_mdc_a, b_mdc_b;
    wire        b_mdio_a, b_mdio_b;
    wire        b_prst_a, b_prst_b;
    wire        b_fsclk, b_fmosi, b_fmiso, b_fcsn;

    AFDX_top #(
        .PORTS(2), .SOC_BOOT_DLY(SOC_DLY), .BYPASS(0), .VLID_MODE(1),
        .SRC_IP(32'h0A000002)
    ) ub (
        .clk(clk), .reset(reset),
        .rx_valid(b_tx_v), .rx_data(b_tx_d), .app_vlid(b_tx_vl),
        .tx_valid(b_rx_v), .tx_data(b_rx_d),
        .tx_vlid(b_rx_vl), .tx_port(b_rx_p),
        .err(b_err),
        .gmii_a_txd(b_ga_txd), .gmii_a_txen(b_ga_txen), .gmii_a_txer(b_ga_txer),
        .gmii_a_gtxc(b_ga_gtxc),
        .gmii_a_rxd(ga_to_b), .gmii_a_rxdv(ga_to_b_v), .gmii_a_rxer(1'b0), .gmii_a_rxc(clk),
        .gmii_b_txd(b_gb_txd), .gmii_b_txen(b_gb_txen), .gmii_b_txer(b_gb_txer),
        .gmii_b_gtxc(b_gb_gtxc),
        .gmii_b_rxd(gb_to_b), .gmii_b_rxdv(gb_to_b_v), .gmii_b_rxer(1'b0), .gmii_b_rxc(clk),
        .mdc_a(b_mdc_a), .mdio_a(b_mdio_a), .phy_rstb_a(b_prst_a),
        .mdc_b(b_mdc_b), .mdio_b(b_mdio_b), .phy_rstb_b(b_prst_b),
        .flash_sclk(b_fsclk), .flash_mosi(b_fmosi), .flash_miso(b_fmiso), .flash_cs_n(b_fcsn),
        .debug(b_debug), .mac_cfg_done(b_maccfg), .soc_boot_done(b_boot)
    );

    // ---- 线侧交叉连线（各打一拍，等价于 PHY 的收发延迟）----
    always @(posedge clk) begin
        ga_to_b <= a_ga_txd;  ga_to_b_v <= a_ga_txen;   // A 的 A 网 → B
        gb_to_b <= a_gb_txd;  gb_to_b_v <= a_gb_txen;   // A 的 B 网 → B
        ga_to_a <= b_ga_txd;  ga_to_a_v <= b_ga_txen;   // B 的 A 网 → A
        gb_to_a <= b_gb_txd;  gb_to_a_v <= b_gb_txen;   // B 的 B 网 → A
    end

    // ---------------------------------------------------------------
    // 线侧：A↔B 交叉连接（各打一拍，等价于 PHY 的收发延迟）
    // ---------------------------------------------------------------

    // ---------------------------------------------------------------
    // PHY / Flash 模型
    // ---------------------------------------------------------------
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a1 (.mdc(a_mdc_a), .mdio(a_mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a2 (.mdc(a_mdc_b), .mdio(a_mdio_b));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b1 (.mdc(b_mdc_a), .mdio(b_mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b2 (.mdc(b_mdc_b), .mdio(b_mdio_b));
    pullup (a_mdio_a); pullup (a_mdio_b); pullup (b_mdio_a); pullup (b_mdio_b);


    spi_flash_model #(.AW(13), .USE_IMG(1), .IMG("flash.hex"))   u_flash_a (
        .sclk(a_fsclk), .cs_n(a_fcsn), .mosi(a_fmosi), .miso(a_fmiso));
    spi_flash_model #(.AW(13), .USE_IMG(1), .IMG("flash_b.hex")) u_flash_b (
        .sclk(b_fsclk), .cs_n(b_fcsn), .mosi(b_fmosi), .miso(b_fmiso));

    // ---------------------------------------------------------------
    // 线侧抓帧（找 SFD 0xD5 之后的前 128 字节；用于核对源 IP 与 FCS）
    // ---------------------------------------------------------------
    reg [7:0] cap_ab [0:127];
    reg [7:0] cap_ba [0:127];
    integer   wab, wba;
    reg       fab, fba, ab_done, ba_done;

    always @(posedge clk) begin
        // A → B（A 网）
        if (!reset) begin
            if (ga_to_b_v) begin
                if (!fab) begin
                    if (ga_to_b == 8'hD5) begin fab <= 1'b1; wab <= 0; ab_done <= 1'b0; end
                end else begin
                    if (wab < 128) begin cap_ab[wab] <= ga_to_b; wab <= wab + 1; end
                end
            end else if (fab) begin
                fab <= 1'b0; ab_done <= 1'b1;
            end
            // B → A（A 网）
            if (ga_to_a_v) begin
                if (!fba) begin
                    if (ga_to_a == 8'hD5) begin fba <= 1'b1; wba <= 0; ba_done <= 1'b0; end
                end else begin
                    if (wba < 128) begin cap_ba[wba] <= ga_to_a; wba <= wba + 1; end
                end
            end else if (fba) begin
                fba <= 1'b0; ba_done <= 1'b1;
            end
        end
    end

    reg ab_done_q = 0;
    integer n777 = 0, n778 = 0, nf_ab = 0;
    always @(posedge clk) begin
        if (ab_done && !ab_done_q) begin
            if (cap_ab[4] === 8'h07 && cap_ab[5] === 8'h77) n777 = n777 + 1;
            if (cap_ab[4] === 8'h07 && cap_ab[5] === 8'h78) n778 = n778 + 1;
            nf_ab = nf_ab + 1;
        end
        ab_done_q <= ab_done;
    end

    initial begin wab = 0; wba = 0; fab = 0; fba = 0; ab_done = 0; ba_done = 0; end

    // ---------------------------------------------------------------
    // 应用口监视（按 VLID 过滤：两个实例的固件自己也会发周期帧）
    // ---------------------------------------------------------------
    reg [15:0] ra_vl, rb_vl;
    reg        ra_seg, rb_seg;
    integer    na, nb;                    // 目标 VL 上收到的字节数
    reg [7:0]  ra_buf [0:255];
    reg [7:0]  rb_buf [0:255];

    always @(posedge clk) begin
        if (reset) begin
            ra_seg <= 0; rb_seg <= 0; na <= 0; nb <= 0; ra_vl <= 0; rb_vl <= 0;
        end else begin
            // A 侧
            if (a_rx_v) begin
                if (!ra_seg) begin
                    ra_vl <= a_rx_vl; ra_seg <= 1'b1;
                    if (a_rx_vl == VL_BA) begin na <= 1; ra_buf[0] <= a_rx_d; end
                end else if (ra_vl == VL_BA) begin
                    if (na < 256) ra_buf[na] <= a_rx_d;
                    na <= na + 1;
                end
            end else ra_seg <= 1'b0;
            // B 侧
            if (b_rx_v) begin
                if (!rb_seg) begin
                    rb_vl <= b_rx_vl; rb_seg <= 1'b1;
                    if (b_rx_vl == VL_AB) begin nb <= 1; rb_buf[0] <= b_rx_d; end
                end else if (rb_vl == VL_AB) begin
                    if (nb < 256) rb_buf[nb] <= b_rx_d;
                    nb <= nb + 1;
                end
            end else rb_seg <= 1'b0;
        end
    end

    // ---------------------------------------------------------------
    // 发送任务（一条连续 valid 段 = 一条消息；VLID 在段首拍给出）
    // ---------------------------------------------------------------
    integer k;
    task send_a; input integer n; input [7:0] base; input [15:0] vl;
        begin
            @(negedge clk); a_tx_vl = vl;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); a_tx_v = 1; a_tx_d = base + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); a_tx_v = 0;
        end
    endtask
    task send_b; input integer n; input [7:0] base; input [15:0] vl;
        begin
            @(negedge clk); b_tx_vl = vl;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); b_tx_v = 1; b_tx_d = base + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); b_tx_v = 0;
        end
    endtask

    task wait_nb; input integer n; input integer tmax; output ok;
        integer w;
        begin w = 0; while ((nb < n) && (w < tmax)) begin @(posedge clk); w = w + 1; end ok = (nb >= n); end
    endtask
    task wait_na; input integer n; input integer tmax; output ok;
        integer w;
        begin w = 0; while ((na < n) && (w < tmax)) begin @(posedge clk); w = w + 1; end ok = (na >= n); end
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

    integer i, okw;
    reg [7:0] w0;
    initial begin
        #200 reset = 0;
        $display("");
        $display("======== 两个端系统互发（GMII 直连，各自 flash / 各自源 IP）========");

        // ---- T1：两边都启动完成 ----
        okw = 0;
        while ((!a_boot || !b_boot) && okw < 5000000) begin @(posedge clk); okw = okw + 1; end
        chk(okw < 5000000 && a_boot === 1'b1 && b_boot === 1'b1, "both ES booted");
        repeat (20000) @(posedge clk);          // 让固件把服务/端口配好

        // ---- T2/T3：A 在 VL 0x0200 上发 64 字节 → B 收到 ----
        nb = 0;
        send_a(NMSG, 8'hA0, VL_AB);
        wait_nb(NMSG, WAIT_MS, okw);
        $display("  [dbg] A->B nb=%0d vlid=%04x b0=%02x b63=%02x",
                 nb, rb_vl, rb_buf[0], rb_buf[63]);
        chk(okw && rb_vl === VL_AB, "A->B delivered with VLID 0x0200");
        okw = (nb >= NMSG);
        for (i = 0; i < NMSG; i = i + 1) if (rb_buf[i] !== 8'hA0 + i[7:0]) okw = 0;
        chk(okw, "A->B payload A0..DF");

        // 线侧帧：DA / 源 IP
        okw = 0;
        while (!ab_done && okw < WAIT_MS) begin @(posedge clk); okw = okw + 1; end
        $display("  [dbg] wire A->B: DA=%02x:%02x:%02x:%02x:%02x:%02x sip=%0d.%0d.%0d.%0d",
                 cap_ab[0], cap_ab[1], cap_ab[2], cap_ab[3], cap_ab[4], cap_ab[5],
                 cap_ab[26], cap_ab[27], cap_ab[28], cap_ab[29]);
        chk(ab_done && cap_ab[0] === 8'h03 && cap_ab[4] === VL_AB[15:8] && cap_ab[5] === VL_AB[7:0],
            "wire A->B DA = 03:00:00:00:02:00");
        chk(cap_ab[26] === 8'd10 && cap_ab[27] === 8'd0 && cap_ab[28] === 8'd0 && cap_ab[29] === 8'd1,
            "wire A->B src IP = 10.0.0.1");

        // ---- T4/T5：B 在 VL 0x0201 上发 64 字节 → A 收到 ----
        repeat (20000) @(posedge clk);
        na = 0;
        send_b(NMSG, 8'h50, VL_BA);
        wait_na(NMSG, WAIT_MS, okw);
        $display("  [dbg] B->A na=%0d vlid=%04x b0=%02x b63=%02x",
                 na, ra_vl, ra_buf[0], ra_buf[63]);
        chk(okw && ra_vl === VL_BA, "B->A delivered with VLID 0x0201");
        okw = (na >= NMSG);
        for (i = 0; i < NMSG; i = i + 1) if (ra_buf[i] !== 8'h50 + i[7:0]) okw = 0;
        chk(okw, "B->A payload 50..8F");

        okw = 0;
        while (!ba_done && okw < WAIT_MS) begin @(posedge clk); okw = okw + 1; end
        $display("  [dbg] wire B->A: DA=%02x:%02x:%02x:%02x:%02x:%02x sip=%0d.%0d.%0d.%0d",
                 cap_ba[0], cap_ba[1], cap_ba[2], cap_ba[3], cap_ba[4], cap_ba[5],
                 cap_ba[26], cap_ba[27], cap_ba[28], cap_ba[29]);
        chk(cap_ba[26] === 8'd10 && cap_ba[27] === 8'd0 && cap_ba[28] === 8'd0 && cap_ba[29] === 8'd2,
            "wire B->A src IP = 10.0.0.2");

        // ---- T6：A+B 双网只交付一次（字节数不能翻倍）----
        repeat (200000) @(posedge clk);
        chk(nb == NMSG && na == NMSG, "A/B duplicates delivered once only");

        // ---- T10：两边都没有丢帧/溢出 ----
        $display("  [dbg] err: A=%b B=%b", a_err, b_err);
        chk(a_err === 1'b0 && b_err === 1'b0, "no drop/overflow on both ES");


        begin : t11
            integer n0, w11;
            repeat (20000) @(posedge clk);
            n0 = n777;
            send_a(64, 8'h60, 16'h0777);
            w11 = 0; while ((n777 == n0) && (w11 < 300000)) begin @(posedge clk); w11 = w11 + 1; end
            $display("  [dbg] T11 A sent VL 0x777 -> wire frames n777=%0d (waited %0d)", n777, w11);
            chk(n777 == n0 + 1, "T11 A's own flash VL 0x777 goes out");
        end
        begin : t12
            integer n0, w12;
            repeat (20000) @(posedge clk);
            n0 = n778;
            send_a(64, 8'h70, 16'h0778);
            repeat (300000) @(posedge clk);          // 2.4 ms：足够（首帧不受 BAG 限制）
            $display("  [dbg] T12 A sent VL 0x778 -> wire frames n778=%0d (was %0d)", n778, n0);
            chk(n778 === n0, "T12 VL 0x778 (only in B's flash) never sent by A");

            n0 = nf_ab;
            send_a(64, 8'h61, 16'h0250);
            w12 = 0; while ((nf_ab == n0) && (w12 < 300000)) begin @(posedge clk); w12 = w12 + 1; end
            $display("  [dbg] T13 recovery nf_ab=%0d (was %0d, waited %0d)", nf_ab, n0, w12);
            chk(nf_ab == n0 + 1, "T13 TX path still alive after rejected VL");
        end

        $display("==================================");
        if (errs == 0) $display(">> PASS: 两个端系统互发（A->B / B->A / 各自源 IP / 冗余只交付一次）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #40000000; $display(">> TIMEOUT"); $finish; end

endmodule
