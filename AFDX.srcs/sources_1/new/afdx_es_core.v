// afdx_es_core.v —— 系统核心
`timescale 1ns/1ps

module afdx_es_core #(
    parameter integer PORTS       = 2,     // 冗余网口数（1=A only）
    parameter integer SOC_BOOT_DLY= 2000000, // 上电等自协商的周期数（125MHz × 2,000,000 = 16ms）
    parameter integer VLID_MODE   = 0,
    parameter [31:0]  SRC_IP     = 32'h0A000001
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
    input  wire [15:0] app_vlid,
    input  wire [3:0]  app_port,
    output wire        app_tx_valid,
    output wire [7:0]  app_tx_data,
    output wire [3:0]  app_tx_port,
    output wire [15:0] app_tx_vlid,
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
    // ---- 上电时序：MAC 配置完成 + 自协商等待之后，才放分区跑 ----
    reg  [31:0] boot_cnt;
    always @(posedge clk) begin
        if (reset)              boot_cnt <= 32'd0;
        else if (!mac_cfg_done) boot_cnt <= 32'd0;
        else if (boot_cnt < SOC_BOOT_DLY) boot_cnt <= boot_cnt + 32'd1;
    end

    wire boot_ok = bs_bypass ? 1'b1 : (boot_cnt >= SOC_BOOT_DLY);
    wire soc_rst = reset | ~boot_ok;
    assign soc_boot_done = boot_ok;

    // 引擎 ↔ MAC 的帧流
    wire        fr_valid, fr_ready, fr_sop, fr_eop, fr_net;
    wire [7:0]  fr_data;


    wire        tx_rdy_a, tx_rdy_b;

    assign fr_ready = bs_bypass ? 1'b1 :
                      ((PORTS >= 2) ? (tx_rdy_a & tx_rdy_b) : tx_rdy_a);

    // A 网壳的输出
    wire        ra_valid, ra_sop, ra_eop, ra_ready;
    wire [7:0]  ra_data;
    wire        done_a;
    // B 网壳的输出
    wire        rb_valid, rb_sop, rb_eop, rb_ready;
    wire [7:0]  rb_data;
    wire        done_b;

    // 送进引擎的接收流
    wire        rx_valid, rx_sop, rx_eop, rx_net;
    wire [7:0]  rx_data;


    reg  [7:0] bs_d_r;
    reg        bs_v_r, bs_s_r, bs_pv_r;
    always @(posedge clk) begin
        bs_d_r  <= bs_rx_data;
        bs_v_r  <= bs_rx_valid;
        bs_s_r  <= bs_rx_valid & ~bs_pv_r;      // 段首（寄存：落到"延迟流的第一个字节"那一拍 ✓）
        bs_pv_r <= bs_rx_valid;
    end

    wire       bs_e_r_w = bs_v_r & ~bs_rx_valid;

    wire        bs_rx_net = (bs_d_r == 8'h40);

    // 送进引擎的接收流：默认取仲裁器输出，`bs_bypass` 时取字节流
    wire        rx_valid_m = bs_bypass ? bs_v_r : rx_valid;
    wire        rx_sop_m   = bs_bypass ? bs_s_r : rx_sop;
    wire        rx_eop_m   = bs_bypass ? bs_e_r_w : rx_eop;
    wire [7:0]  rx_data_m  = bs_bypass ? bs_d_r : rx_data;
    wire        rx_net_m   = bs_bypass ? bs_rx_net : rx_net;

    // TX 方向：默认去 MAC 壳，`bs_bypass` 时去字节流口（两路 gated，避免同时发）
    assign bs_tx_data  = fr_data;
    assign bs_tx_valid = fr_valid & bs_bypass;
    assign bs_tx_ready = 1'b1;                  // RX 路径无背压（解析器 out_ready 钉 1）
    wire   fr_valid_mac = fr_valid & ~bs_bypass;

    assign mac_cfg_done = (PORTS >= 2) ? (done_a & done_b) : done_a;


    wire [31:0] ts_fine;
    wire [63:0] ts_rx_ts, ts_tx_ts;
    wire [15:0] ts_rx_seq, ts_tx_seq;

    wire [31:0] ts_tx_vl_ts;
    wire [12:0] ts_tx_vl_sel;
    wire [31:0] ts_rx_vl_ts;
    wire [12:0] ts_rx_vl_sel;

    afdx_ts_capture #(.NETS(2)) u_ts (
        .clk(clk), .reset(reset), .ts_now(ts_fine),
        .rx_dv({ (PORTS >= 2) ? gmii_b_rxdv : 1'b0, gmii_a_rxdv }),
        .rx_d ({ (PORTS >= 2) ? gmii_b_rxd  : 8'h00, gmii_a_rxd  }),

        .tx_en({ (PORTS >= 2) ? gmii_b_txen : 1'b0, gmii_a_txen }),
        .tx_d ({ (PORTS >= 2) ? gmii_b_txd  : 8'h00, gmii_a_txd  }),
        .rx_ts(ts_rx_ts), .rx_seq(ts_rx_seq),
        .tx_ts(ts_tx_ts), .tx_seq(ts_tx_seq)
    );


    afdx_ts_txtable #(.NETS(2), .VLW(12)) u_tstx (
        .clk(clk), .reset(reset), .ts_now(ts_fine),
        .gmii_en({ (PORTS >= 2) ? gmii_b_txen : 1'b0, gmii_a_txen }),
        .gmii_d ({ (PORTS >= 2) ? gmii_b_txd  : 8'h00, gmii_a_txd }),
        .vl_sel(ts_tx_vl_sel), .vl_ts(ts_tx_vl_ts), .tbl_rdy()
    );


    afdx_ts_txtable #(.NETS(2), .VLW(12)) u_tsrx (
        .clk(clk), .reset(reset), .ts_now(ts_fine),
        .gmii_en({ (PORTS >= 2) ? gmii_b_rxdv : 1'b0, gmii_a_rxdv }),
        .gmii_d ({ (PORTS >= 2) ? gmii_b_rxd  : 8'h00, gmii_a_rxd }),
        .vl_sel(ts_rx_vl_sel), .vl_ts(ts_rx_vl_ts), .tbl_rdy()
    );

    darkriscv_soc_top #(.VLID_MODE(VLID_MODE), .SRC_IP(SRC_IP)) u_soc (
        .clk            (clk),
        .reset          (soc_rst),
        .debug          (debug),
        .flash_sclk     (flash_sclk), .flash_mosi(flash_mosi),
        .flash_miso     (flash_miso), .flash_cs_n(flash_cs_n),
        .dp_rxin_valid  (rx_valid_m),
        .dp_rxin_sop    (rx_sop_m),
        .dp_rxin_eop    (rx_eop_m),
        .dp_rxin_data   (rx_data_m),
        .dp_rxin_net    (rx_net_m),
        .dp_fr_valid    (fr_valid),
        .dp_fr_ready    (fr_ready),
        .dp_fr_sop      (fr_sop),
        .dp_fr_eop      (fr_eop),
        .dp_fr_data     (fr_data),
        .dp_fr_net      (fr_net),
        .app_rx_valid   (app_rx_valid),
        .app_rx_data    (app_rx_data),
        .app_vlid       (app_vlid),
        .app_port       (app_port),
        .app_tx_valid   (app_tx_valid),
        .app_tx_data    (app_tx_data),
        .app_tx_port    (app_tx_port),
        .app_tx_vlid    (app_tx_vlid),
        .app_err        (app_err),
        .cfg_hit_o      (),
        .cfg_cfg_o      (),
        .ts_rx_ts       (ts_rx_ts),
        .ts_rx_seq      (ts_rx_seq),
        .ts_tx_ts       (ts_tx_ts),
        .ts_tx_seq      (ts_tx_seq),
        .ts_fine_o      (ts_fine),
        .ts_tx_vl_ts    (ts_tx_vl_ts),
        .ts_tx_vl_sel   (ts_tx_vl_sel),
        .ts_rx_vl_ts    (ts_rx_vl_ts),
        .ts_rx_vl_sel   (ts_rx_vl_sel),

        .host_awaddr    (host_awaddr),
        .host_awvalid   (host_awvalid),
        .host_awready   (host_awready),
        .host_wdata     (host_wdata),
        .host_wstrb     (host_wstrb),
        .host_wvalid    (host_wvalid),
        .host_wready    (host_wready),
        .host_bresp     (host_bresp),
        .host_bvalid    (host_bvalid),
        .host_bready    (host_bready),
        .host_araddr    (host_araddr),
        .host_arvalid   (host_arvalid),
        .host_arready   (host_arready),
        .host_rdata     (host_rdata),
        .host_rresp     (host_rresp),
        .host_rvalid    (host_rvalid),
        .host_rready    (host_rready),
        .host_irq       (host_irq)
    );


    reg [15:0] por_cnt;
    always @(posedge clk) begin
        if (reset)                    por_cnt <= 16'd0;
        else if (por_cnt != 16'hFFFF) por_cnt <= por_cnt + 16'd1;
    end
    wire por_ok = (por_cnt == 16'hFFFF);      // 上电约 5.2 ms 后放行 PHY（真实自协商本就要几百 ms）
    wire phy_rstb0_a, phy_rstb0_b;
    assign phy_rstb_a = phy_rstb0_a & por_ok;
    assign phy_rstb_b = (PORTS >= 2) ? (phy_rstb0_b & por_ok) : 1'b0;

    // ---------------- 网络 A ----------------
    afdx_mac_wrap #(.NET_SEL(1'b0)) u_mac_a (
        .clk(clk), .reset(reset),
        .gmii_txd(gmii_a_txd), .gmii_txen(gmii_a_txen), .gmii_txer(gmii_a_txer),
        .gmii_gtxc(gmii_a_gtxc),
        .gmii_rxd(gmii_a_rxd), .gmii_rxdv(gmii_a_rxdv), .gmii_rxer(gmii_a_rxer),
        .gmii_rxc(gmii_a_rxc),
        .mdc(mdc_a), .mdio(mdio_a), .phy_rstb0(phy_rstb0_a),
        .fr_valid(fr_valid_mac), .fr_ready(tx_rdy_a), .fr_sop(fr_sop),
        .fr_eop(fr_eop), .fr_data(fr_data), .fr_net(fr_net),
        .rx_valid(ra_valid), .rx_sop(ra_sop), .rx_eop(ra_eop), .rx_data(ra_data),
        .rx_ready(ra_ready), .mac_cfg_done(done_a)
    );

    // ---------------- 网络 B + 仲裁器（仅 PORTS>=2）----------------
    generate
    if (PORTS >= 2) begin : g_dual
        afdx_mac_wrap #(.NET_SEL(1'b1)) u_mac_b (
            .clk(clk), .reset(reset),
            .gmii_txd(gmii_b_txd), .gmii_txen(gmii_b_txen), .gmii_txer(gmii_b_txer),
            .gmii_gtxc(gmii_b_gtxc),
            .gmii_rxd(gmii_b_rxd), .gmii_rxdv(gmii_b_rxdv), .gmii_rxer(gmii_b_rxer),
            .gmii_rxc(gmii_b_rxc),
            .mdc(mdc_b), .mdio(mdio_b), .phy_rstb0(phy_rstb0_b),
            .fr_valid(fr_valid_mac), .fr_ready(tx_rdy_b), .fr_sop(fr_sop),
            .fr_eop(fr_eop), .fr_data(fr_data), .fr_net(fr_net),
            .rx_valid(rb_valid), .rx_sop(rb_sop), .rx_eop(rb_eop), .rx_data(rb_data),
            .rx_ready(rb_ready), .mac_cfg_done(done_b)
        );

        afdx_rx_arb u_arb (
            .clk(clk), .reset(reset),
            .a_valid(ra_valid), .a_sop(ra_sop), .a_eop(ra_eop),
            .a_data(ra_data),   .a_ready(ra_ready),
            .b_valid(rb_valid), .b_sop(rb_sop), .b_eop(rb_eop),
            .b_data(rb_data),   .b_ready(rb_ready),
            .m_valid(rx_valid), .m_sop(rx_sop), .m_eop(rx_eop),
            .m_data(rx_data),   .m_net(rx_net)
        );
    end
    else begin : g_single
        // 单网：A 路直连引擎；B 路全部接确定值
        assign rx_valid = ra_valid;
        assign rx_sop   = ra_sop;
        assign rx_eop   = ra_eop;
        assign rx_data  = ra_data;
        assign rx_net   = 1'b0;
        assign ra_ready = 1'b1;             // 单网不做反压（解析器永远能收）
        assign done_b   = 1'b1;
        assign tx_rdy_b = 1'b1;             // 没有 B 壳，占位
        assign gmii_b_txd = 8'h00; assign gmii_b_txen = 1'b0; assign gmii_b_txer = 1'b0;
        assign gmii_b_gtxc= 1'b0;
        assign mdc_b = 1'b0;       // phy_rstb_b 已由上面的 `PORTS>=2` 条件式给出（单网恒 0）
        assign mdio_b = 1'bz;
                                            //    否则该 inout 无任何驱动（Quartus 会报 no-driver）
    end
    endgenerate

endmodule
