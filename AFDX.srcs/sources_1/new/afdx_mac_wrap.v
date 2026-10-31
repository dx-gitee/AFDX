// afdx_mac_wrap.v —— 以太网 MAC 对接壳（eLinx 三速以太网 MAC IP + 上电配置）
`timescale 1ns/1ps

module afdx_mac_wrap #(
    parameter NET_SEL = 1'b0               // 本壳负责哪张网：0=A, 1=B
) (
    input  wire        clk,                // 125 MHz：MAC clk_sys / 用户 FIFO / 引擎 同频
    input  wire        reset,              // 高有效

    // ---- 线侧 GMII（接外部 PHY；100M MII 半字节线时 p0_gtxc=25MHz）----
    output wire [7:0]  gmii_txd,
    output wire        gmii_txen,
    output wire        gmii_txer,
    output wire        gmii_gtxc,
    input  wire [7:0]  gmii_rxd,
    input  wire        gmii_rxdv,
    input  wire        gmii_rxer,
    input  wire        gmii_rxc,

    // ---- MDIO PHY 管理 ----
    output wire        mdc,
    inout  wire        mdio,
    output wire        phy_rstb0,

    // ---- 引擎侧：要发的帧流（一次一个字节，sop/eop 与 data 同拍）----
    input  wire        fr_valid,
    output wire        fr_ready,
    input  wire        fr_sop,
    input  wire        fr_eop,
    input  wire [7:0]  fr_data,
    input  wire        fr_net,             // 这一帧属于哪张网

    // ---- 引擎侧：收下来的帧流（SFD 已剥，从目的 MAC 第一字节开始）----
    output wire        rx_valid,
    output wire        rx_sop,
    output wire        rx_eop,
    output wire [7:0]  rx_data,
    input  wire        rx_ready,           // 下游(双网仲裁器)的接收许可 → 反压 MAC 的 ff_rx

    output wire        mac_cfg_done        // 上电配置完成（可观察）
);
    wire        rst_b = ~reset;
    wire [7:0]  ff_tx_data;
    wire        ff_tx_sop, ff_tx_eop, ff_tx_wren, ff_tx_rdy, ff_tx_al_full;
    wire [7:0]  ff_rx_data;
    wire        ff_rx_data_vld, ff_rx_sop, ff_rx_eop, ff_rx_err, ff_rx_rdy;
    wire [31:0] reg_data_out;
    wire        reg_acc_bsy;
    wire        reg_wr, reg_rd;
    wire [3:0]  reg_addr;
    wire [31:0] reg_data_in;

    wire        this_net = (fr_net == NET_SEL);

    // ---- 发送方向：引擎 → MAC ff_tx ----
    //   非本网的帧也要**照常吃掉**（fr_ready 一直有效），只是不写进 MAC ——
    //   否则双网配置下主机接口会一直挂着等 B 帧被取走。
    assign ff_tx_data = fr_data;
    assign ff_tx_wren = fr_valid && this_net && ff_tx_rdy;
    assign ff_tx_sop  = fr_sop   && this_net && ff_tx_rdy;
    assign ff_tx_eop  = fr_eop   && this_net && ff_tx_rdy;

    assign fr_ready   = this_net ? ff_tx_rdy : 1'b1;


    wire [11:0] skd_used;              // 只读观察（探针/调试用；综合时被优化掉）
    wire        skd_push;              // 只读观察：本拍 MAC 送来一个有效字节（供 tb 探针）
    wire        skd_is_sfd_obs = ff_rx_data_vld && ff_rx_sop && (ff_rx_data == 8'hD5);
    assign skd_push = ff_rx_data_vld && !skd_is_sfd_obs;

    afdx_mac_rxwfifo #(.DEPTH(2048), .AW(11)) u_rxfifo (
        .clk(clk), .reset(reset),
        .in_vld(ff_rx_data_vld), .in_sop(ff_rx_sop), .in_eop(ff_rx_eop), .in_data(ff_rx_data),
        .in_rdy(ff_rx_rdy),
        .out_vld(rx_valid), .out_sop(rx_sop), .out_eop(rx_eop), .out_data(rx_data),
        .out_rdy(rx_ready),
        .used(skd_used)
    );

    // ---- MAC 上电配置 ----
    afdx_mac_cfg u_mac_cfg (
        .clk         (clk),
        .reset       (reset),
        .reg_wr      (reg_wr),
        .reg_rd      (reg_rd),
        .reg_addr    (reg_addr),
        .reg_data_in (reg_data_in),
        .reg_data_out(reg_data_out),
        .reg_acc_bsy (reg_acc_bsy),
        .done        (mac_cfg_done)
    );

    // ---- MAC IP 核 ----
    triplespeedethernet_1 u_mac (
        .mdc            (mdc),
        .phy_rstb0      (phy_rstb0),
        .p0_gtxc        (gmii_gtxc),
        .p0_txd         (gmii_txd),
        .p0_txen        (gmii_txen),
        .p0_txer        (gmii_txer),
        .ff_tx_rdy      (ff_tx_rdy),
        .ff_tx_al_full  (ff_tx_al_full),
        .ff_rx_data_vld (ff_rx_data_vld),
        .ff_rx_sop      (ff_rx_sop),
        .ff_rx_eop      (ff_rx_eop),
        .ff_rx_data     (ff_rx_data),
        .ff_rx_err      (ff_rx_err),
        .reg_data_out   (reg_data_out),
        .reg_acc_bsy    (reg_acc_bsy),

        .clk_sys        (clk),
        .rst_b          (rst_b),
        // MII 专用的三根输入：GMII 全双工下本 IP 网表里没有逻辑依赖
        .p0_txc         (1'b0),
        .p0_col         (1'b0),
        .p0_crs         (1'b0),
        .p0_rxc         (gmii_rxc),
        .p0_rxd         (gmii_rxd),
        .p0_rxdv        (gmii_rxdv),
        .p0_rxer        (gmii_rxer),
        .ff_tx_clk      (clk),
        .ff_tx_data     (ff_tx_data),
        .ff_tx_sop      (ff_tx_sop),
        .ff_tx_eop      (ff_tx_eop),
        .ff_tx_err      (1'b0),
        .ff_tx_wren     (ff_tx_wren),
        .ff_rx_clk      (clk),
        .ff_rx_rdy      (ff_rx_rdy),
        .reg_wr         (reg_wr),
        .reg_rd         (reg_rd),
        .reg_addr       (reg_addr),
        .reg_data_in    (reg_data_in),
        .mdio           (mdio)
    );

endmodule
