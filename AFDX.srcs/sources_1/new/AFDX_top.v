// AFDX_top.v —— 赛题模板顶层（赛题原文"顶层模块统一采用下面的模板"），也是评测 TOP
`timescale 1ns/1ps

module AFDX_top #(
    parameter integer PORTS        = 2,        // 冗余网口数（1 = 只做 A 网）
    parameter integer SOC_BOOT_DLY = 2000000,   // 上电等 MAC 就绪的周期数（125 MHz × 2,000,000 = 16 ms）
    parameter integer BYPASS       = 1,

    parameter integer APP_SIDE     = 1,

    parameter integer APP_PORT_DEF = 0,

    parameter integer VLID_MODE    = 1,

    parameter [31:0]  SRC_IP       = 32'h0A000001
) (
    // ===== 赛题模板的 8 个信号（名字/方向/顺序与原文一致）=====
    input  wire        clk,          // 时钟信号
    input  wire        reset,        // 复位信号
    output wire [7:0]  tx_data,      // 发送数据
    input  wire [7:0]  rx_data,      // 接收数据
    output wire        tx_valid,     // 发送数据有效指示
    input  wire        rx_valid,     // 接收数据有效指示
    output wire        tx_ready,     // 发送数据准备好指示
    input  wire        rx_ready,     // 接收数据准备好指示


    input  wire [15:0] app_vlid,     // 本条待发消息的 VLID（段首拍采样）
    output wire [3:0]  tx_port,      // 本条交付消息来自哪个端口（段首拍有效）
    output wire [15:0] tx_vlid,      // 本条交付消息的 VLID（段首拍有效）
    output wire        err,          // 粘滞：入向被拒 / 端口溢出（reset 清）

    // ===== 额外接口（官方答疑允许"酌情增加"）：网络侧 GMII / MDIO / SPI flash =====
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

    // ===== 额外：调试/状态 =====
    output wire [3:0]  debug,
    output wire        mac_cfg_done,
    output wire        soc_boot_done
);

    wire bs_rx_ready_unused;
    assign bs_rx_ready_unused = rx_ready;

    // ---- APP_SIDE 选择：同一组模板信号，接到"应用侧桥"或"旧字节流口" ----
    wire        app_side = (APP_SIDE != 0);

    wire        es_app_tx_valid;
    wire [7:0]  es_app_tx_data;
    wire [3:0]  es_app_tx_port;
    wire        es_app_err;
    wire        es_bs_tx_ready;
    wire [7:0]  es_bs_tx_data;
    wire        es_bs_tx_valid;


    reg        bs_loop_v;
    reg  [7:0] bs_loop_d;
    always @(posedge clk) begin
        if (reset) begin
            bs_loop_v <= 1'b0;
            bs_loop_d <= 8'h00;
        end else begin
            bs_loop_v <= es_bs_tx_valid;
            bs_loop_d <= es_bs_tx_data;
        end
    end

    wire        app_loop = app_side && BYPASS[0];   // 应用侧 + 旁路 ⇒ 内部回环

    wire        es_app_rx_valid = app_side ? rx_valid : 1'b0;
    wire [7:0]  es_app_rx_data  = rx_data;
    wire [3:0]  es_app_port     = APP_PORT_DEF[3:0];
    wire        es_bs_rx_valid  = app_side ? (app_loop ? bs_loop_v : 1'b0) : rx_valid;
    wire [7:0]  es_bs_rx_data   = app_loop ? bs_loop_d : rx_data;

    assign tx_data  = app_side ? es_app_tx_data  : es_bs_tx_data;
    assign tx_valid = app_side ? es_app_tx_valid : es_bs_tx_valid;
    assign tx_ready = app_side ? 1'b1           : es_bs_tx_ready;   // 答疑：tx_ready 取值不做要求
    assign tx_port  = es_app_tx_port;
    assign err      = es_app_err;

    afdx_es_core #(
        .PORTS        (PORTS),
        .SOC_BOOT_DLY (SOC_BOOT_DLY),
        .VLID_MODE    (VLID_MODE),
        .SRC_IP       (SRC_IP)
    ) u_es (
        .clk             (clk),
        .reset           (reset),

        // ---- 赛题模板的字节流口 ----
        .bs_bypass       (BYPASS[0]),
        .bs_rx_data      (es_bs_rx_data),
        .bs_rx_valid     (es_bs_rx_valid),
        .bs_tx_ready     (es_bs_tx_ready),
        .bs_tx_data      (es_bs_tx_data),
        .bs_tx_valid     (es_bs_tx_valid),
        .bs_rx_ready     (rx_ready),

        // ---- 应用侧（APP_SIDE=1 时模板 8 根走这里）----
        .app_rx_valid    (es_app_rx_valid),
        .app_rx_data     (es_app_rx_data),
        .app_vlid        (app_vlid),
        .app_port        (es_app_port),
        .app_tx_valid    (es_app_tx_valid),
        .app_tx_data     (es_app_tx_data),
        .app_tx_port     (es_app_tx_port),
        .app_tx_vlid     (tx_vlid),
        .app_err         (es_app_err),


        .host_awaddr(32'h0), .host_awvalid(1'b0), .host_awready(),
        .host_wdata (32'h0), .host_wstrb(4'h0),  .host_wvalid(1'b0), .host_wready(),
        .host_bresp (), .host_bvalid(), .host_bready(1'b0),
        .host_araddr(32'h0), .host_arvalid(1'b0), .host_arready(),
        .host_rdata (), .host_rresp(), .host_rvalid(), .host_rready(1'b0),
        .host_irq   (),

        // ---- 额外：网络侧（BYPASS=0 时才真正使用）----
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

        // ---- 额外：SPI flash（上电加载 VL 配置表）----
        .flash_sclk      (flash_sclk),   .flash_mosi (flash_mosi),
        .flash_miso      (flash_miso),   .flash_cs_n (flash_cs_n),

        // ---- 额外：调试 ----
        .debug           (debug),
        .mac_cfg_done    (mac_cfg_done),
        .soc_boot_done   (soc_boot_done)
    );
endmodule
