// AFDX_host_shell.v —— 带 AXI-Lite 主机接口的系统顶层（SmartNIC/外部主机形态）
`timescale 1ns/1ps

module AFDX_host_shell #(
    parameter integer PORTS        = 2,
    parameter integer SOC_BOOT_DLY = 2000000,
    parameter integer BYPASS       = 1
) (
    // ===== 赛题模板的 8 个信号（与 AFDX_top 完全一致）=====
    input  wire        clk,
    input  wire        reset,
    output wire [7:0]  tx_data,
    input  wire [7:0]  rx_data,
    output wire        tx_valid,
    input  wire        rx_valid,
    output wire        tx_ready,
    input  wire        rx_ready,

    // ===== 网络侧 / SPI（与 AFDX_top 一致）=====
    output wire [7:0]  gmii_a_txd,
    output wire        gmii_a_txen,
    output wire        gmii_a_txer,
    output wire        gmii_a_gtxc,
    input  wire [7:0]  gmii_a_rxd,
    input  wire        gmii_a_rxdv,
    input  wire        gmii_a_rxer,
    input  wire        gmii_a_rxc,

    output wire [7:0]  gmii_b_txd,
    output wire        gmii_b_txen,
    output wire        gmii_b_txer,
    output wire        gmii_b_gtxc,
    input  wire [7:0]  gmii_b_rxd,
    input  wire        gmii_b_rxdv,
    input  wire        gmii_b_rxer,
    input  wire        gmii_b_rxc,

    output wire        mdc_a,
    inout  wire        mdio_a,
    output wire        phy_rstb_a,
    output wire        mdc_b,
    inout  wire        mdio_b,
    output wire        phy_rstb_b,

    output wire        flash_sclk,
    output wire        flash_mosi,
    input  wire        flash_miso,
    output wire        flash_cs_n,

    // ===== 调试/状态 =====
    output wire [3:0]  debug,
    output wire        mac_cfg_done,
    output wire        soc_boot_done,


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
    // `rx_ready` 按官方答疑不参与握手（与 AFDX_top 同）
    wire bs_rx_ready_unused;
    assign bs_rx_ready_unused = rx_ready;

    afdx_es_wrap #(.PORTS(PORTS), .SOC_BOOT_DLY(SOC_BOOT_DLY)) u_es (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk             (clk),
        .reset           (reset),

        .bs_bypass       (BYPASS[0]),
        .bs_rx_data      (rx_data),
        .bs_rx_valid     (rx_valid),
        .bs_tx_ready     (tx_ready),
        .bs_tx_data      (tx_data),
        .bs_tx_valid     (tx_valid),
        .bs_rx_ready     (rx_ready),


        .host_awaddr     (host_awaddr),
        .host_awvalid    (host_awvalid),
        .host_awready    (host_awready),
        .host_wdata      (host_wdata),
        .host_wstrb      (host_wstrb),
        .host_wvalid     (host_wvalid),
        .host_wready     (host_wready),
        .host_bresp      (host_bresp),
        .host_bvalid     (host_bvalid),
        .host_bready     (host_bready),
        .host_araddr     (host_araddr),
        .host_arvalid    (host_arvalid),
        .host_arready    (host_arready),
        .host_rdata      (host_rdata),
        .host_rresp      (host_rresp),
        .host_rvalid     (host_rvalid),
        .host_rready     (host_rready),
        .host_irq        (host_irq),

        .gmii_a_txd      (gmii_a_txd),   .gmii_a_txen(gmii_a_txen),
        .gmii_a_txer     (gmii_a_txer),  .gmii_a_gtxc(gmii_a_gtxc),
        .gmii_a_rxd      (gmii_a_rxd),   .gmii_a_rxdv(gmii_a_rxdv),
        .gmii_a_rxer     (gmii_a_rxer),  .gmii_a_rxc (gmii_a_rxc),
        .gmii_b_txd      (gmii_b_txd),   .gmii_b_txen(gmii_b_txen),
        .gmii_b_txer     (gmii_b_txer),  .gmii_b_gtxc(gmii_b_gtxc),
        .gmii_b_rxd      (gmii_b_rxd),   .gmii_b_rxdv(gmii_b_rxdv),
        .gmii_b_rxer     (gmii_b_rxer),  .gmii_b_rxc (gmii_b_rxc),
        .mdc_a           (mdc_a),        .mdio_a     (mdio_a),     .phy_rstb_a(phy_rstb_a),
        .mdc_b           (mdc_b),        .mdio_b     (mdio_b),     .phy_rstb_b(phy_rstb_b),

        .flash_sclk      (flash_sclk),   .flash_mosi (flash_mosi),
        .flash_miso      (flash_miso),   .flash_cs_n (flash_cs_n),

        .debug           (debug),
        .mac_cfg_done    (mac_cfg_done),
        .soc_boot_done   (soc_boot_done)
    );
endmodule
