// AFDX_axis_shell.v —— 绿色框（可综合版）：AFDX 端系统 + AXI4-Stream 主机口
`timescale 1ns/1ps

module AFDX_axis_shell #(
    parameter integer PORTS        = 2,
    parameter integer SOC_BOOT_DLY = 2000000,
    parameter integer DATA_WIDTH   = 256
) (
    input  wire        clk,                 // 125 MHz（AFDX 设计时钟，千兆 GMII）
    input  wire        reset,               // 高有效

    // ---- GMII A ----
    output wire [7:0]  gmii_a_txd,
    output wire        gmii_a_txen,
    output wire        gmii_a_txer,
    output wire        gmii_a_gtxc,
    input  wire [7:0]  gmii_a_rxd,
    input  wire        gmii_a_rxdv,
    input  wire        gmii_a_rxer,
    input  wire        gmii_a_rxc,
    // ---- GMII B ----
    output wire [7:0]  gmii_b_txd,
    output wire        gmii_b_txen,
    output wire        gmii_b_txer,
    output wire        gmii_b_gtxc,
    input  wire [7:0]  gmii_b_rxd,
    input  wire        gmii_b_rxdv,
    input  wire        gmii_b_rxer,
    input  wire        gmii_b_rxc,

    // ---- MDIO / PHY 复位（引出，由外部接 PB 模型/PHY）----
    output wire        mdc_a,
    inout  wire        mdio_a,
    output wire        phy_rstb_a,
    output wire        mdc_b,
    inout  wire        mdio_b,
    output wire        phy_rstb_b,

    // ---- SPI flash（4 根）----
    output wire        flash_sclk,
    output wire        flash_mosi,
    input  wire        flash_miso,
    output wire        flash_cs_n,

    // ---- AXI4-Stream：H2C（主机→FPGA，DUT 是 sink）----
    input  wire [DATA_WIDTH-1:0] m_axis_h2c_tdata,
    input  wire [DATA_WIDTH/8-1:0] m_axis_h2c_tkeep,
    input  wire        m_axis_h2c_tlast,
    input  wire        m_axis_h2c_tvalid,
    output wire        m_axis_h2c_tready,
    // ---- AXI4-Stream：C2H（FPGA→主机，DUT 是 source）----
    output wire [DATA_WIDTH-1:0] s_axis_c2h_tdata,
    output wire [DATA_WIDTH/8-1:0] s_axis_c2h_tkeep,
    output wire        s_axis_c2h_tlast,
    output wire        s_axis_c2h_tvalid,
    input  wire        s_axis_c2h_tready,

    // ---- 状态/中断 ----
    output wire        usr_irq_req,
    output wire        axis_error,
    output wire [3:0]  debug,
    output wire        mac_cfg_done,
    output wire        soc_boot_done
);
    // 适配器 ↔ 邮箱
    wire [31:0] host_awaddr, host_wdata, host_araddr, host_rdata;
    wire        host_awvalid, host_awready, host_wvalid, host_wready;
    wire        host_bvalid, host_bready, host_arvalid, host_arready;
    wire        host_rvalid, host_rready, host_irq;
    wire [1:0]  host_bresp, host_rresp;
    wire [3:0]  host_wstrb;

    afdx_host_axis #(.DATA_WIDTH(DATA_WIDTH)) u_axis (
        .clk(clk), .reset(reset),
        .m_axis_h2c_tdata(m_axis_h2c_tdata),
        .m_axis_h2c_tkeep(m_axis_h2c_tkeep),
        .m_axis_h2c_tlast(m_axis_h2c_tlast),
        .m_axis_h2c_tvalid(m_axis_h2c_tvalid),
        .m_axis_h2c_tready(m_axis_h2c_tready),
        .s_axis_c2h_tdata(s_axis_c2h_tdata),
        .s_axis_c2h_tkeep(s_axis_c2h_tkeep),
        .s_axis_c2h_tlast(s_axis_c2h_tlast),
        .s_axis_c2h_tvalid(s_axis_c2h_tvalid),
        .s_axis_c2h_tready(s_axis_c2h_tready),
        .m_axil_awaddr(host_awaddr), .m_axil_awvalid(host_awvalid), .m_axil_awready(host_awready),
        .m_axil_wdata(host_wdata), .m_axil_wstrb(host_wstrb), .m_axil_wvalid(host_wvalid), .m_axil_wready(host_wready),
        .m_axil_bresp(host_bresp), .m_axil_bvalid(host_bvalid), .m_axil_bready(host_bready),
        .m_axil_araddr(host_araddr), .m_axil_arvalid(host_arvalid), .m_axil_arready(host_arready),
        .m_axil_rdata(host_rdata), .m_axil_rresp(host_rresp), .m_axil_rvalid(host_rvalid), .m_axil_rready(host_rready),
        .axis_error(axis_error)
    );

    wire        bs_tx_ready, bs_tx_valid;
    wire [7:0]  bs_tx_data;

    afdx_es_wrap #(.PORTS(PORTS), .SOC_BOOT_DLY(SOC_BOOT_DLY)) u_es (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk(clk), .reset(reset),
        .gmii_a_txd(gmii_a_txd), .gmii_a_txen(gmii_a_txen), .gmii_a_txer(gmii_a_txer),
        .gmii_a_gtxc(gmii_a_gtxc), .gmii_a_rxd(gmii_a_rxd), .gmii_a_rxdv(gmii_a_rxdv),
        .gmii_a_rxer(gmii_a_rxer), .gmii_a_rxc(gmii_a_rxc),
        .gmii_b_txd(gmii_b_txd), .gmii_b_txen(gmii_b_txen), .gmii_b_txer(gmii_b_txer),
        .gmii_b_gtxc(gmii_b_gtxc), .gmii_b_rxd(gmii_b_rxd), .gmii_b_rxdv(gmii_b_rxdv),
        .gmii_b_rxer(gmii_b_rxer), .gmii_b_rxc(gmii_b_rxc),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done),
        .flash_sclk(flash_sclk), .flash_mosi(flash_mosi),
        .flash_miso(flash_miso), .flash_cs_n(flash_cs_n),
        // 模板字节流口不用（真实 MAC 路径）
        .bs_bypass(1'b0),
        .bs_rx_data(8'h00), .bs_rx_valid(1'b0), .bs_rx_ready(1'b1),
        .bs_tx_data(bs_tx_data), .bs_tx_valid(bs_tx_valid), .bs_tx_ready(bs_tx_ready),
        // 主机 AXI-Lite 邮箱
        .host_awaddr(host_awaddr), .host_awvalid(host_awvalid), .host_awready(host_awready),
        .host_wdata(host_wdata), .host_wstrb(host_wstrb), .host_wvalid(host_wvalid), .host_wready(host_wready),
        .host_bresp(host_bresp), .host_bvalid(host_bvalid), .host_bready(host_bready),
        .host_araddr(host_araddr), .host_arvalid(host_arvalid), .host_arready(host_arready),
        .host_rdata(host_rdata), .host_rresp(host_rresp), .host_rvalid(host_rvalid), .host_rready(host_rready),
        .host_irq(host_irq)
    );

    assign usr_irq_req = host_irq;

endmodule
