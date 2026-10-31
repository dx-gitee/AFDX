// AFDX_axis_sim_shell.v —— 绿色框（仿真版）：端口表与 AFDX_axis_shell 基本一致，只多 2 根线侧帧监视口（mon_frame_valid/mon_frame_data）
`timescale 1ns/1ps

module AFDX_axis_sim_shell #(
    parameter integer PORTS        = 2,
    parameter integer SOC_BOOT_DLY = 2000000,
    parameter integer DATA_WIDTH   = 256,
    parameter integer SIM_BYPASS   = 1
) (
    input  wire        clk,
    input  wire        reset,

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

    // ---- MDIO / PHY 复位 ----
    output wire        mdc_a,
    inout  wire        mdio_a,
    output wire        phy_rstb_a,
    output wire        mdc_b,
    inout  wire        mdio_b,
    output wire        phy_rstb_b,

    // ---- SPI flash ----
    output wire        flash_sclk,
    output wire        flash_mosi,
    input  wire        flash_miso,
    output wire        flash_cs_n,

    // ---- AXI4-Stream ----
    input  wire [DATA_WIDTH-1:0]   m_axis_h2c_tdata,
    input  wire [DATA_WIDTH/8-1:0] m_axis_h2c_tkeep,
    input  wire        m_axis_h2c_tlast,
    input  wire        m_axis_h2c_tvalid,
    output wire        m_axis_h2c_tready,
    output wire [DATA_WIDTH-1:0]   s_axis_c2h_tdata,
    output wire [DATA_WIDTH/8-1:0] s_axis_c2h_tkeep,
    output wire        s_axis_c2h_tlast,
    output wire        s_axis_c2h_tvalid,
    input  wire        s_axis_c2h_tready,

    // ---- 状态/中断/调试 ----
    output wire        usr_irq_req,
    output wire        axis_error,
    output wire [3:0]  debug,
    output wire        mac_cfg_done,
    output wire        soc_boot_done,
    // AFDX 线侧帧监视（SIM_BYPASS=1 时 = 引擎发出的帧；连续 valid 段=一帧）
    output wire        mon_frame_valid,
    output wire [7:0]  mon_frame_data
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

    // ---- SIM_BYPASS=1：内部帧回环；=0：交给真实 MAC/GMII ----
    wire        bs_tx_valid, bs_tx_ready;
    wire [7:0]  bs_tx_data;

    reg         loop_valid;
    reg  [7:0]  loop_data;
    always @(posedge clk) begin
        if (reset) begin
            loop_valid <= 1'b0;
            loop_data  <= 8'h00;
        end
        else begin
            loop_valid <= bs_tx_valid;
            loop_data  <= bs_tx_data;
        end
    end

    wire        bs_rx_valid = SIM_BYPASS ? loop_valid : 1'b0;
    wire [7:0]  bs_rx_data  = SIM_BYPASS ? loop_data  : 8'h00;

    // SIM_BYPASS=1 时 GMII 输入不参与（接确定值，避免 X 进 MAC）
    wire [7:0]  ga_rxd  = SIM_BYPASS ? 8'h00 : gmii_a_rxd;
    wire        ga_rxdv = SIM_BYPASS ? 1'b0  : gmii_a_rxdv;
    wire        ga_rxer = SIM_BYPASS ? 1'b0  : gmii_a_rxer;
    wire        ga_rxc  = SIM_BYPASS ? clk   : gmii_a_rxc;
    wire [7:0]  gb_rxd  = SIM_BYPASS ? 8'h00 : gmii_b_rxd;
    wire        gb_rxdv = SIM_BYPASS ? 1'b0  : gmii_b_rxdv;
    wire        gb_rxer = SIM_BYPASS ? 1'b0  : gmii_b_rxer;
    wire        gb_rxc  = SIM_BYPASS ? clk   : gmii_b_rxc;

    afdx_es_wrap #(.PORTS(PORTS), .SOC_BOOT_DLY(SOC_BOOT_DLY)) u_es (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk(clk), .reset(reset),
        .gmii_a_txd(gmii_a_txd), .gmii_a_txen(gmii_a_txen), .gmii_a_txer(gmii_a_txer),
        .gmii_a_gtxc(gmii_a_gtxc), .gmii_a_rxd(ga_rxd), .gmii_a_rxdv(ga_rxdv),
        .gmii_a_rxer(ga_rxer), .gmii_a_rxc(ga_rxc),
        .gmii_b_txd(gmii_b_txd), .gmii_b_txen(gmii_b_txen), .gmii_b_txer(gmii_b_txer),
        .gmii_b_gtxc(gmii_b_gtxc), .gmii_b_rxd(gb_rxd), .gmii_b_rxdv(gb_rxdv),
        .gmii_b_rxer(gb_rxer), .gmii_b_rxc(gb_rxc),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done),
        .flash_sclk(flash_sclk), .flash_mosi(flash_mosi),
        .flash_miso(flash_miso), .flash_cs_n(flash_cs_n),
        .bs_bypass(SIM_BYPASS != 0),
        .bs_rx_data(bs_rx_data), .bs_rx_valid(bs_rx_valid), .bs_rx_ready(1'b1),
        .bs_tx_data(bs_tx_data), .bs_tx_valid(bs_tx_valid), .bs_tx_ready(bs_tx_ready),
        .host_awaddr(host_awaddr), .host_awvalid(host_awvalid), .host_awready(host_awready),
        .host_wdata(host_wdata), .host_wstrb(host_wstrb), .host_wvalid(host_wvalid), .host_wready(host_wready),
        .host_bresp(host_bresp), .host_bvalid(host_bvalid), .host_bready(host_bready),
        .host_araddr(host_araddr), .host_arvalid(host_arvalid), .host_arready(host_arready),
        .host_rdata(host_rdata), .host_rresp(host_rresp), .host_rvalid(host_rvalid), .host_rready(host_rready),
        .host_irq(host_irq)
    );

    assign usr_irq_req   = host_irq;
    assign mon_frame_valid = bs_tx_valid;
    assign mon_frame_data  = bs_tx_data;

endmodule
