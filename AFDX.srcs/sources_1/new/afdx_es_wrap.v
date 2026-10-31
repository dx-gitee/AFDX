// afdx_es_wrap.v —— 系统核心 afdx_es_core 的薄封装
`timescale 1ns/1ps

module afdx_es_wrap #(
    parameter integer PORTS       = 2,     // 冗余网口数（1=A only）
    parameter integer SOC_BOOT_DLY= 2000000, // 上电等自协商的周期数（125MHz × 2,000,000 = 16ms）
    parameter [31:0]  SRC_IP      = 32'h0A000001
) (
    input  wire        clk,                // 125 MHz 系统/引擎/MAC 同频
    input  wire        reset,              // 高有效

    // ---- 网络 A 的 GMII ----
    output wire [7:0]  gmii_a_txd,
    output wire        gmii_a_txen,
    output wire        gmii_a_txer,
    output wire        gmii_a_gtxc,
    input  wire [7:0]  gmii_a_rxd,
    input  wire        gmii_a_rxdv,
    input  wire        gmii_a_rxer,
    input  wire        gmii_a_rxc,

    // ---- 网络 B 的 GMII（PORTS=1 时不用，接确定值即可）----
    output wire [7:0]  gmii_b_txd,
    output wire        gmii_b_txen,
    output wire        gmii_b_txer,
    output wire        gmii_b_gtxc,
    input  wire [7:0]  gmii_b_rxd,
    input  wire        gmii_b_rxdv,
    input  wire        gmii_b_rxer,
    input  wire        gmii_b_rxc,

    // ---- MDIO ----
    output wire        mdc_a,
    inout  wire        mdio_a,
    output wire        phy_rstb_a,
    output wire        mdc_b,
    inout  wire        mdio_b,
    output wire        phy_rstb_b,

    // ---- 调试 ----
    output wire [3:0]  debug,
    output wire        mac_cfg_done,
    output wire        soc_boot_done,


    output wire        flash_sclk,
    output wire        flash_mosi,
    input  wire        flash_miso,
    output wire        flash_cs_n,


    input  wire        app_rx_valid,
    input  wire [7:0]  app_rx_data,
    input  wire [3:0]  app_port,
    output wire        app_tx_valid,
    output wire [7:0]  app_tx_data,
    output wire [3:0]  app_tx_port,
    output wire        app_err,

    input  wire        bs_bypass,       // 1 = 用字节流口（默认 0 = 走 GMII）
    input  wire [7:0]  bs_rx_data,      // 外部送进来的"收到的帧"字节（不含前导/SFD）
    input  wire        bs_rx_valid,
    output wire        bs_tx_ready,     // ES 侧"可以收"（本设计 RX 无背压 → 恒 1）
    output wire [7:0]  bs_tx_data,      // ES 发出的帧字节
    output wire        bs_tx_valid,
    input  wire        bs_rx_ready,     // 外部"可以收"；按官方答疑**不参与握手**（这里只引入不反压）


    input  wire [31:0] host_awaddr,
    input  wire        host_awvalid,
    output wire        host_awready,
    input  wire [31:0] host_wdata,
    input  wire [3:0]  host_wstrb,
    input  wire        host_wvalid,
    output wire        host_wready,
    output wire [1:0]  host_bresp,
    output wire        host_bvalid,
    input  wire        host_bready,
    input  wire [31:0] host_araddr,
    input  wire        host_arvalid,
    output wire        host_arready,
    output wire [31:0] host_rdata,
    output wire [1:0]  host_rresp,
    output wire        host_rvalid,
    input  wire        host_rready,
    output wire        host_irq
);
    afdx_es_core #(
        .PORTS       (PORTS),
        .SOC_BOOT_DLY(SOC_BOOT_DLY),
        .VLID_MODE   (0),
        .SRC_IP      (SRC_IP)
    ) u_core (
        .clk(clk),
        .reset(reset),
        .gmii_a_txd(gmii_a_txd),
        .gmii_a_txen(gmii_a_txen),
        .gmii_a_txer(gmii_a_txer),
        .gmii_a_gtxc(gmii_a_gtxc),
        .gmii_a_rxd(gmii_a_rxd),
        .gmii_a_rxdv(gmii_a_rxdv),
        .gmii_a_rxer(gmii_a_rxer),
        .gmii_a_rxc(gmii_a_rxc),
        .gmii_b_txd(gmii_b_txd),
        .gmii_b_txen(gmii_b_txen),
        .gmii_b_txer(gmii_b_txer),
        .gmii_b_gtxc(gmii_b_gtxc),
        .gmii_b_rxd(gmii_b_rxd),
        .gmii_b_rxdv(gmii_b_rxdv),
        .gmii_b_rxer(gmii_b_rxer),
        .gmii_b_rxc(gmii_b_rxc),
        .mdc_a(mdc_a),
        .mdio_a(mdio_a),
        .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b),
        .mdio_b(mdio_b),
        .phy_rstb_b(phy_rstb_b),
        .debug(debug),
        .mac_cfg_done(mac_cfg_done),
        .soc_boot_done(soc_boot_done),
        .flash_sclk(flash_sclk),
        .flash_mosi(flash_mosi),
        .flash_miso(flash_miso),
        .flash_cs_n(flash_cs_n),
        .app_rx_valid(app_rx_valid),
        .app_rx_data(app_rx_data),
        .app_port(app_port),

        .app_vlid    (16'h0),
        .app_tx_vlid (),
        .app_tx_valid(app_tx_valid),
        .app_tx_data(app_tx_data),
        .app_tx_port(app_tx_port),
        .app_err(app_err),
        .bs_bypass(bs_bypass),
        .bs_rx_data(bs_rx_data),
        .bs_rx_valid(bs_rx_valid),
        .bs_tx_ready(bs_tx_ready),
        .bs_tx_data(bs_tx_data),
        .bs_tx_valid(bs_tx_valid),
        .bs_rx_ready(bs_rx_ready),
        .host_awaddr(host_awaddr),
        .host_awvalid(host_awvalid),
        .host_awready(host_awready),
        .host_wdata(host_wdata),
        .host_wstrb(host_wstrb),
        .host_wvalid(host_wvalid),
        .host_wready(host_wready),
        .host_bresp(host_bresp),
        .host_bvalid(host_bvalid),
        .host_bready(host_bready),
        .host_araddr(host_araddr),
        .host_arvalid(host_arvalid),
        .host_arready(host_arready),
        .host_rdata(host_rdata),
        .host_rresp(host_rresp),
        .host_rvalid(host_rvalid),
        .host_rready(host_rready),
        .host_irq(host_irq)
    );
endmodule
