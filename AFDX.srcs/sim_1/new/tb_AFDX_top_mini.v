// tb_AFDX_top.v —— AFDX_top 统一顶层 TB
`timescale 1ns/1ps

module tb_AFDX_top #(
    parameter integer MODE       = 5,        // 5=门级最短冒烟（默认）；0=两个 ES 应用侧互发；1=单 ES GMII→应用接收
    parameter integer PORTS      = 2,
    parameter integer SOC_DLY    = 30000,    // 0.24 ms @125 MHz（8h 方案）
    parameter integer BOOT_QUIET = 5000,     // 0.04 ms（POST_SYN 兜底）
    parameter integer WAIT_MSG   = 600000,   // 4.8 ms，等对端交付
    parameter integer MAXAV      = 512,
    parameter APP_FILE = "AFDX_ES.srcs/sim_1/new/app_side.hex"
) (
    // 顶层 debug 输出端口：vopt 会保留顶层端口，波形不需要 +acc。
    output wire        dbg_a_tx_valid,
    output wire [7:0]  dbg_a_tx_data,
    output wire [15:0] dbg_a_tx_vlid,
    output wire [3:0]  dbg_a_tx_port,
    output wire        dbg_b_tx_valid,
    output wire [7:0]  dbg_b_tx_data,
    output wire [15:0] dbg_b_tx_vlid,
    output wire [3:0]  dbg_b_tx_port,
    output wire        dbg_a_maccfg,
    output wire        dbg_a_boot,
    output wire        dbg_b_maccfg,
    output wire        dbg_b_boot,
    output wire        dbg_a_err,
    output wire        dbg_b_err,
    output wire        dbg_a_ga_txen,
    output wire [7:0]  dbg_a_ga_txd,
    output wire        dbg_a_gb_txen,
    output wire [7:0]  dbg_a_gb_txd,
    output wire        dbg_b_ga_txen,
    output wire [7:0]  dbg_b_ga_txd,
    output wire        dbg_b_gb_txen,
    output wire [7:0]  dbg_b_gb_txd
);

`ifdef POST_SYN
    // 综合后网表没有 parameter 列表；由 eLinx 综合时已固定
    `define AFDX_A_PARAMS
    `define AFDX_B_PARAMS
`else
    `define AFDX_A_PARAMS #(.PORTS(PORTS), .SOC_BOOT_DLY(DLY_A), .BYPASS(0), .APP_SIDE(1), .APP_PORT_DEF(0), .VLID_MODE(1), .SRC_IP(32'h0A000001))
    `define AFDX_B_PARAMS #(.PORTS(PORTS), .SOC_BOOT_DLY(DLY_A), .BYPASS(0), .APP_SIDE(1), .APP_PORT_DEF(0), .VLID_MODE(1), .SRC_IP(32'h0A000002))
`endif

    localparam integer MAXAVB = 131072;
    // GMII 模式用短 boot（老 GMII 台口径）；应用侧两 ES 模式用长 boot
    localparam integer DLY_A   = (MODE == 1) ? 2000 : SOC_DLY;
    localparam integer QUIET_A = (MODE == 1) ? 2000 : BOOT_QUIET;

    reg clk = 1'b0;
    always #4 clk = ~clk;                    // 125 MHz

    reg reset_a = 1'b1;
    reg reset_b = 1'b1;
    reg gmii_inject = 1'b0;                  // 1 = A 的 GMII 输入由本 TB 文件激励驱动

    // ---------------------------------------------------------------
    // A 的应用侧
    // ---------------------------------------------------------------
    reg  [7:0]  a_tx_d  = 8'h00;
    reg         a_tx_v  = 1'b0;
    reg  [15:0] a_tx_vl = 16'h0000;
    wire        a_rx_v;
    wire [7:0]  a_rx_d;
    wire [15:0] a_rx_vl;
    wire [3:0]  a_rx_p;
    wire        a_err;

    // ---------------------------------------------------------------
    // B 的应用侧
    // ---------------------------------------------------------------
    reg  [7:0]  b_tx_d  = 8'h00;
    reg         b_tx_v  = 1'b0;
    reg  [15:0] b_tx_vl = 16'h0000;
    wire        b_rx_v;
    wire [7:0]  b_rx_d;
    wire [15:0] b_rx_vl;
    wire [3:0]  b_rx_p;
    wire        b_err;

    // ---------------------------------------------------------------
    // 线侧交叉连接信号（必须在 DUT 例化之前声明）
    // ---------------------------------------------------------------
    reg [7:0] b_to_a_a, b_to_a_b, a_to_b_a, a_to_b_b;
    reg       b_to_a_a_v, b_to_a_b_v, a_to_b_a_v, a_to_b_b_v;

    // GMII 模式：TB 直接驱动 A 的 A/B 两网输入（B 保持复位）
    reg [7:0] tb_gmii_a_rxd = 8'h00;
    reg       tb_gmii_a_rxdv = 1'b0;
    reg       tb_gmii_a_rxer = 1'b0;
    reg [7:0] tb_gmii_b_rxd = 8'h00;
    reg       tb_gmii_b_rxdv = 1'b0;
    reg       tb_gmii_b_rxer = 1'b0;

    wire [7:0] a_gmii_a_rxd = gmii_inject ? tb_gmii_a_rxd : b_to_a_a;
    wire       a_gmii_a_rxdv = gmii_inject ? tb_gmii_a_rxdv : b_to_a_a_v;
    wire       a_gmii_a_rxer = gmii_inject ? tb_gmii_a_rxer : 1'b0;
    wire [7:0] a_gmii_b_rxd = gmii_inject ? tb_gmii_b_rxd : b_to_a_b;
    wire       a_gmii_b_rxdv = gmii_inject ? tb_gmii_b_rxdv : b_to_a_b_v;
    wire       a_gmii_b_rxer = gmii_inject ? tb_gmii_b_rxer : 1'b0;

    // A 的 GMII / MDIO / Flash / 状态
    wire [7:0] a_ga_txd, a_gb_txd;
    wire       a_ga_txen, a_ga_txer, a_ga_gtxc;
    wire       a_gb_txen, a_gb_txer, a_gb_gtxc;
    wire       a_mdc_a, a_mdc_b;
    wire       a_mdio_a, a_mdio_b;
    wire       a_prst_a, a_prst_b;
    wire       a_fsclk, a_fmosi, a_fmiso, a_fcsn;
    wire [3:0] a_debug;
    wire       a_maccfg, a_boot;

    // B 的 GMII / MDIO / Flash / 状态
    wire [7:0] b_ga_txd, b_gb_txd;
    wire       b_ga_txen, b_ga_txer, b_ga_gtxc;
    wire       b_gb_txen, b_gb_txer, b_gb_gtxc;
    wire       b_mdc_a, b_mdc_b;
    wire       b_mdio_a, b_mdio_b;
    wire       b_prst_a, b_prst_b;
    wire       b_fsclk, b_fmosi, b_fmiso, b_fcsn;
    wire [3:0] b_debug;
    wire       b_maccfg, b_boot;

    // 顶层 debug 端口（波形用；不影响 DUT）
    assign dbg_a_tx_valid = a_rx_v;
    assign dbg_a_tx_data  = a_rx_d;
    assign dbg_a_tx_vlid  = a_rx_vl;
    assign dbg_a_tx_port  = a_rx_p;
    assign dbg_b_tx_valid = b_rx_v;
    assign dbg_b_tx_data  = b_rx_d;
    assign dbg_b_tx_vlid  = b_rx_vl;
    assign dbg_b_tx_port  = b_rx_p;
    assign dbg_a_maccfg   = a_maccfg;
    assign dbg_a_boot     = a_boot;
    assign dbg_b_maccfg   = b_maccfg;
    assign dbg_b_boot     = b_boot;
    assign dbg_a_err      = a_err;
    assign dbg_b_err      = b_err;
    assign dbg_a_ga_txen  = a_ga_txen;
    assign dbg_a_ga_txd   = a_ga_txd;
    assign dbg_a_gb_txen  = a_gb_txen;
    assign dbg_a_gb_txd   = a_gb_txd;
    assign dbg_b_ga_txen  = b_ga_txen;
    assign dbg_b_ga_txd   = b_ga_txd;
    assign dbg_b_gb_txen  = b_gb_txen;
    assign dbg_b_gb_txd   = b_gb_txd;

    // ---------------------------------------------------------------
    // 实例 A：源 IP = 10.0.0.1，flash = flash.hex
    // ---------------------------------------------------------------
    AFDX_top `AFDX_A_PARAMS u_a (
        .clk        (clk),
        .reset      (reset_a),

        .tx_data    (a_rx_d),
        .rx_data    (a_tx_d),
        .tx_valid   (a_rx_v),
        .rx_valid   (a_tx_v),
        .tx_ready   (),
        .rx_ready   (1'b1),

        .app_vlid   (a_tx_vl),
        .tx_port    (a_rx_p),
        .tx_vlid    (a_rx_vl),
        .err        (a_err),

        .gmii_a_txd (a_ga_txd), .gmii_a_txen(a_ga_txen),
        .gmii_a_txer(a_ga_txer), .gmii_a_gtxc(a_ga_gtxc),
        .gmii_a_rxd (a_gmii_a_rxd), .gmii_a_rxdv(a_gmii_a_rxdv),
        .gmii_a_rxer(a_gmii_a_rxer), .gmii_a_rxc (clk),

        .gmii_b_txd (a_gb_txd), .gmii_b_txen(a_gb_txen),
        .gmii_b_txer(a_gb_txer), .gmii_b_gtxc(a_gb_gtxc),
        .gmii_b_rxd (a_gmii_b_rxd), .gmii_b_rxdv(a_gmii_b_rxdv),
        .gmii_b_rxer(a_gmii_b_rxer), .gmii_b_rxc (clk),

        .mdc_a      (a_mdc_a),  .mdio_a     (a_mdio_a),  .phy_rstb_a(a_prst_a),
        .mdc_b      (a_mdc_b),  .mdio_b     (a_mdio_b),  .phy_rstb_b(a_prst_b),

        .flash_sclk (a_fsclk),  .flash_mosi (a_fmosi),
        .flash_miso (a_fmiso),  .flash_cs_n (a_fcsn),

        .debug      (a_debug),  .mac_cfg_done(a_maccfg), .soc_boot_done(a_boot)
    );

    // ---------------------------------------------------------------
    // 实例 B：源 IP = 10.0.0.2，flash = flash_b.hex
    // ---------------------------------------------------------------
    AFDX_top `AFDX_B_PARAMS u_b (
        .clk        (clk),
        .reset      (reset_b),

        .tx_data    (b_rx_d),
        .rx_data    (b_tx_d),
        .tx_valid   (b_rx_v),
        .rx_valid   (b_tx_v),
        .tx_ready   (),
        .rx_ready   (1'b1),

        .app_vlid   (b_tx_vl),
        .tx_port    (b_rx_p),
        .tx_vlid    (b_rx_vl),
        .err        (b_err),

        .gmii_a_txd (b_ga_txd), .gmii_a_txen(b_ga_txen),
        .gmii_a_txer(b_ga_txer), .gmii_a_gtxc(b_ga_gtxc),
        .gmii_a_rxd (a_to_b_a), .gmii_a_rxdv(a_to_b_a_v),
        .gmii_a_rxer(1'b0),     .gmii_a_rxc (clk),

        .gmii_b_txd (b_gb_txd), .gmii_b_txen(b_gb_txen),
        .gmii_b_txer(b_gb_txer), .gmii_b_gtxc(b_gb_gtxc),
        .gmii_b_rxd (a_to_b_b), .gmii_b_rxdv(a_to_b_b_v),
        .gmii_b_rxer(1'b0),     .gmii_b_rxc (clk),

        .mdc_a      (b_mdc_a),  .mdio_a     (b_mdio_a),  .phy_rstb_a(b_prst_a),
        .mdc_b      (b_mdc_b),  .mdio_b     (b_mdio_b),  .phy_rstb_b(b_prst_b),

        .flash_sclk (b_fsclk),  .flash_mosi (b_fmosi),
        .flash_miso (b_fmiso),  .flash_cs_n (b_fcsn),

        .debug      (b_debug),  .mac_cfg_done(b_maccfg), .soc_boot_done(b_boot)
    );

    // ---------------------------------------------------------------
    // 线侧交叉：各打一拍，等价 PHY 收发延迟
    //   A 的 A 网 / B 网 → B；B 的 A 网 / B 网 → A
    // ---------------------------------------------------------------
    always @(posedge clk) begin
        if (reset_a || reset_b) begin
            b_to_a_a <= 8'h00; b_to_a_a_v <= 1'b0;
            b_to_a_b <= 8'h00; b_to_a_b_v <= 1'b0;
            a_to_b_a <= 8'h00; a_to_b_a_v <= 1'b0;
            a_to_b_b <= 8'h00; a_to_b_b_v <= 1'b0;
        end else begin
            b_to_a_a <= b_ga_txd; b_to_a_a_v <= b_ga_txen;
            b_to_a_b <= b_gb_txd; b_to_a_b_v <= b_gb_txen;
            a_to_b_a <= a_ga_txd; a_to_b_a_v <= a_ga_txen;
            a_to_b_b <= a_gb_txd; a_to_b_b_v <= a_gb_txen;
        end
    end

    // ---------------------------------------------------------------
    // PHY / Flash 模型
    // ---------------------------------------------------------------
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a1 (.mdc(a_mdc_a), .mdio(a_mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a2 (.mdc(a_mdc_b), .mdio(a_mdio_b));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b1 (.mdc(b_mdc_a), .mdio(b_mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b2 (.mdc(b_mdc_b), .mdio(b_mdio_b));
    pullup (a_mdio_a); pullup (a_mdio_b); pullup (b_mdio_a); pullup (b_mdio_b);

    // 每个实例各一份镜像：A = flash.hex，B = flash_b.hex（多一条 VL 0x778）
    spi_flash_model_top #(.AW(13), .USE_IMG(1), .IMG("flash.hex"))   u_flash_a (
        .sclk(a_fsclk), .cs_n(a_fcsn), .mosi(a_fmosi), .miso(a_fmiso));
    spi_flash_model_top #(.AW(13), .USE_IMG(1), .IMG("flash_b.hex")) u_flash_b (
        .sclk(b_fsclk), .cs_n(b_fcsn), .mosi(b_fmosi), .miso(b_fmiso));

    // =================================================================
    // 固件自检完成标志（RTL 仿真用：轮询 dRAM 0x3F0）
    //   fw/main.c 自检最后写 RES(0xF0)=0xA5A5C0DE；比固定等 0.8 ms 快。
    //   综合后网表内部层次不可见，POST_SYN 下退回 BOOT_QUIET 兜底。
    // =================================================================
`ifndef POST_SYN
    wire [31:0] a_fw_flag = {u_a.u_es.u_soc.dram3[8'hFC],
                             u_a.u_es.u_soc.dram2[8'hFC],
                             u_a.u_es.u_soc.dram1[8'hFC],
                             u_a.u_es.u_soc.dram0[8'hFC]};
    wire [31:0] b_fw_flag = {u_b.u_es.u_soc.dram3[8'hFC],
                             u_b.u_es.u_soc.dram2[8'hFC],
                             u_b.u_es.u_soc.dram1[8'hFC],
                             u_b.u_es.u_soc.dram0[8'hFC]};

    task wait_fw_ready;
        integer w;
        begin
            w = 0;
            while (((a_fw_flag !== 32'hA5A5C0DE) || (b_fw_flag !== 32'hA5A5C0DE))
                   && (w < 2000000)) begin
                @(posedge clk); w = w + 1;
            end
            $display("  [dbg] firmware flag wait = %0d cycles (A=%08x B=%08x)",
                     w, a_fw_flag, b_fw_flag);
        end
    endtask
`endif

    // =================================================================
    // 应用侧向量解析
    // =================================================================
    reg [31:0] avmem [0:MAXAVB-1];
    integer av_n_total;
    integer av_base;
    integer av_n;
    integer av_dir  [0:MAXAV-1];
    integer av_kind [0:MAXAV-1];
    integer av_vlid [0:MAXAV-1];
    integer av_len  [0:MAXAV-1];
    integer av_pst  [0:MAXAV-1];

    task load_app_vec;
        input [1023:0] fname;
        input integer base;
        input integer count;
        integer i, off, nw;
        begin
            $readmemh(fname, avmem);
            av_n_total = avmem[0];
            if (av_n_total > MAXAV) av_n_total = MAXAV;
            off = 1;
            for (i = 0; i < av_n_total; i = i + 1) begin
                av_dir [i] = avmem[off + 0];
                av_kind[i] = avmem[off + 1];
                av_vlid[i] = avmem[off + 2];
                av_len [i] = avmem[off + 3];
                av_pst [i] = off + 4;
                nw = (av_len[i] + 3) / 4;
                off = off + 4 + nw;
            end
            av_base = base;
            av_n = (count == 0) ? (av_n_total - base) : count;
            if (av_n < 0) av_n = 0;
            if ((av_base + av_n) > av_n_total) av_n = av_n_total - av_base;
            $display("  [vec] %s: base=%0d count=%0d / total=%0d",
                     fname, av_base, av_n, av_n_total);
        end
    endtask

    // 期望：A 口应收到 B→A（dir=1）；B 口应收到 A→B（dir=0）
    // 只有 kind==0 的正常记录才允许交付；kind 1/2/3 是故意注入的错误。
    integer exp_a_n, exp_b_n;
    integer exp_a_vl [0:MAXAV-1];
    integer exp_b_vl [0:MAXAV-1];

    task build_exp;
        integer i;
        begin
            exp_a_n = 0; exp_b_n = 0;
            for (i = 0; i < av_n; i = i + 1) begin
                if (av_kind[av_base+i] == 0) begin
                    if (av_dir[av_base+i] == 1) begin
                        exp_a_vl[exp_a_n] = av_vlid[av_base+i];
                        exp_a_n = exp_a_n + 1;
                    end else begin
                        exp_b_vl[exp_b_n] = av_vlid[av_base+i];
                        exp_b_n = exp_b_n + 1;
                    end
                end
            end
        end
    endtask

    function integer is_exp_a;
        input [15:0] v;
        integer j;
        begin
            is_exp_a = 0;
            for (j = 0; j < exp_a_n; j = j + 1)
                if (exp_a_vl[j] == v) is_exp_a = 1;
        end
    endfunction

    function integer is_exp_b;
        input [15:0] v;
        integer j;
        begin
            is_exp_b = 0;
            for (j = 0; j < exp_b_n; j = j + 1)
                if (exp_b_vl[j] == v) is_exp_b = 1;
        end
    endfunction

    // =================================================================
    // 应用口监视：连续 tx_valid 段 = 一条消息；只收期望 VLID
    // =================================================================
    reg [7:0] a_msgbuf [0:8191];
    integer   a_msg_start [0:MAXAV-1];
    integer   a_msg_len   [0:MAXAV-1];
    integer   a_msg_vl    [0:MAXAV-1];
    integer   a_msg_n = 0, a_wp = 0;
    integer   a_bad_vlid_n = 0;
    reg       a_seg = 0, a_seg_ours = 0;
    reg [15:0] a_seg_vl = 16'h0000;

    wire a_is_exp = is_exp_a(a_rx_vl);

    always @(posedge clk) begin
        if (reset_a) begin
            a_seg <= 1'b0; a_seg_ours <= 1'b0;
            a_msg_n <= 0; a_wp <= 0; a_bad_vlid_n <= 0;
            a_seg_vl <= 16'h0000;
        end else begin
            if (a_rx_v) begin
                if (!a_seg) begin
                    a_seg <= 1'b1;
                    a_seg_ours <= a_is_exp;
                    a_seg_vl <= a_rx_vl;
                    if (a_is_exp && (a_msg_n < MAXAV)) begin
                        a_msg_start[a_msg_n] <= a_wp;
                        a_msg_vl[a_msg_n]    <= a_rx_vl;
                        a_msg_len[a_msg_n]   <= 1;       // 本拍就是第 1 个字节
                        if (a_wp < 8191) a_msgbuf[a_wp] <= a_rx_d;
                        a_wp <= a_wp + 1;
                    end
                end else if (a_is_exp) begin
                    if (a_wp < 8191) a_msgbuf[a_wp] <= a_rx_d;
                    a_wp <= a_wp + 1;
                    if (a_msg_n < MAXAV)
                        a_msg_len[a_msg_n] <= a_msg_len[a_msg_n] + 1;
                end
            end else if (a_seg) begin
                a_seg <= 1'b0;
                if (a_seg_ours && (a_msg_n < MAXAV))
                    a_msg_n <= a_msg_n + 1;
                if ((a_seg_vl == 16'h0ABC) || (a_seg_vl == 16'h0778))
                    a_bad_vlid_n <= a_bad_vlid_n + 1;
                a_seg_ours <= 1'b0;
            end
        end
    end

    reg [7:0] b_msgbuf [0:8191];
    integer   b_msg_start [0:MAXAV-1];
    integer   b_msg_len   [0:MAXAV-1];
    integer   b_msg_vl    [0:MAXAV-1];
    integer   b_msg_n = 0, b_wp = 0;
    integer   b_bad_vlid_n = 0;
    reg       b_seg = 0, b_seg_ours = 0;
    reg [15:0] b_seg_vl = 16'h0000;

    wire b_is_exp = is_exp_b(b_rx_vl);

    always @(posedge clk) begin
        if (reset_b) begin
            b_seg <= 1'b0; b_seg_ours <= 1'b0;
            b_msg_n <= 0; b_wp <= 0; b_bad_vlid_n <= 0;
            b_seg_vl <= 16'h0000;
        end else begin
            if (b_rx_v) begin
                if (!b_seg) begin
                    b_seg <= 1'b1;
                    b_seg_ours <= b_is_exp;
                    b_seg_vl <= b_rx_vl;
                    if (b_is_exp && (b_msg_n < MAXAV)) begin
                        b_msg_start[b_msg_n] <= b_wp;
                        b_msg_vl[b_msg_n]    <= b_rx_vl;
                        b_msg_len[b_msg_n]   <= 1;       // 本拍就是第 1 个字节
                        if (b_wp < 8191) b_msgbuf[b_wp] <= b_rx_d;
                        b_wp <= b_wp + 1;
                    end
                end else if (b_is_exp) begin
                    if (b_wp < 8191) b_msgbuf[b_wp] <= b_rx_d;
                    b_wp <= b_wp + 1;
                    if (b_msg_n < MAXAV)
                        b_msg_len[b_msg_n] <= b_msg_len[b_msg_n] + 1;
                end
            end else if (b_seg) begin
                b_seg <= 1'b0;
                if (b_seg_ours && (b_msg_n < MAXAV))
                    b_msg_n <= b_msg_n + 1;
                if ((b_seg_vl == 16'h0ABC) || (b_seg_vl == 16'h0778))
                    b_bad_vlid_n <= b_bad_vlid_n + 1;
                b_seg_ours <= 1'b0;
            end
        end
    end

    // =================================================================
    // MODE=2 性能测试专用：B 口全消息捕获 + 时延时间戳 + 线侧帧格式
    // =================================================================
    localparam integer PERF_N      = 128;
    localparam integer PERF_ROUNDS = 2;
    localparam integer PERF_GAP    = 13242;   // 1471 + 13242 = 14713 拍/条 = 100 Mbps
    localparam integer PERF_TOTAL  = PERF_N + 9 * PERF_ROUNDS; // 128 + 18 = 146
    localparam integer PERF_BYTES  = PERF_N * 1471;
    localparam integer PERF_CONFIG_N   = 9;
    localparam integer PERF_CONFIG_LEN = 5;
    localparam integer PERF_CONFIG_GAP = 1000;
    localparam integer ERR_WAIT        = 20000;
    localparam integer FAST_TOTAL      = PERF_CONFIG_N + 9 * PERF_ROUNDS; // 9+18=27
    // MODE=5 门级冒烟专用上限（不跑长仿真）
    localparam integer SMOKE_BOOT_WAIT = 100;       // 0.8 µs，只看一眼 boot
    localparam integer SMOKE_MSG_WAIT  = 1000;      // 8 µs，等 2 条交付

    reg [7:0] p_buf [0:524287];
    integer   p_start [0:MAXAV-1];
    integer   p_len   [0:MAXAV-1];
    integer   p_vl    [0:MAXAV-1];
    integer   p_port  [0:MAXAV-1];
    time      p_t     [0:MAXAV-1];
    integer   p_n = 0, p_wp = 0;
    reg       p_seg = 0, p_ours = 0;
    integer   svc_bad = 0;
    reg       svc_seg = 0, svc_check = 0;

    wire p_is_ours = (b_rx_vl >= 16'h0200) && (b_rx_vl <= 16'h027F);

    always @(posedge clk) begin
        if (reset_b) begin
            p_n <= 0; p_wp <= 0; p_seg <= 0; p_ours <= 0;
        end else if (b_rx_v) begin
            if (!p_seg) begin
                p_seg <= 1'b1;
                p_ours <= p_is_ours;
                if (p_is_ours && (p_n < MAXAV)) begin
                    p_start[p_n] <= p_wp;
                    p_len  [p_n] <= 1;
                    p_vl   [p_n] <= b_rx_vl;
                    p_port [p_n] <= b_rx_p;
                    p_t    [p_n] <= $time;
                    if (p_wp < 524287) p_buf[p_wp] <= b_rx_d;
                    p_wp <= p_wp + 1;
                end
            end else if (p_ours) begin
                if (p_wp < 524287) p_buf[p_wp] <= b_rx_d;
                p_wp <= p_wp + 1;
                if (p_n < MAXAV) p_len[p_n] <= p_len[p_n] + 1;
            end
        end else if (p_seg) begin
            p_seg <= 1'b0;
            if (p_ours && (p_n < MAXAV)) begin
                if (((MODE == 0) || (MODE == 3) || (MODE == 5)))
                    $display("[MARK] %0t B-RX vlid=0x%04X len=%0d",
                             $time, p_vl[p_n], p_len[p_n]);
                p_n <= p_n + 1;
            end
            p_ours <= 1'b0;
        end
    end

    // 主发送时间戳：每条消息的第一个 tx_data 拍
    time    p_tx_t [0:MAXAV-1];
    integer p_tx_n = 0;

    // 服务帧不得出现在应用侧交付口（VLID 模式的核心验收）
    always @(posedge clk) begin
        if (reset_b) begin
            svc_bad <= 0; svc_seg <= 0; svc_check <= 0;
        end else if (b_rx_v) begin
            if (!svc_seg) begin
                svc_seg <= 1'b1;
                if (svc_check && (b_rx_vl !== 16'h0000) &&
                    !((b_rx_vl >= 16'h0200) && (b_rx_vl <= 16'h027F)))
                    svc_bad <= svc_bad + 1;
            end
        end else begin
            svc_seg <= 1'b0;
        end
    end

    // 线侧 A 网帧捕获：格式 / SN / FCS / 长度 / VLID
    reg [7:0] wbuf [0:2047];
    integer   wlen = 0;
    reg       wact = 0;
    integer   wframes = 0;
    integer   wframes_afdx = 0;
    integer   wframes_data = 0;
    integer   w_our_frames = 0;
    integer   wbad_len = 0, wbad_fmt = 0, wbad_sn = 0, wbad_fcs = 0, wbad_etype = 0;
    integer   wbad_saif = 0, wbad_sip = 0, wbag_bad = 0, wbad_preamble = 0;
    time      w_start;
    time      wlast_t [0:4095];
    reg       wbag_seen [0:4095];
    reg       wseen [0:4095];
    integer   wseen_n = 0;
    integer   wvl, wsn, wk, woff;
    integer   b_net_frames = 0;
    reg       b_net_act = 0;

    function [31:0] wcrc32b;
        input [31:0] c;
        input [7:0]  d;
        integer k;
        reg [31:0] t;
        begin
            t = c ^ {24'h0, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            wcrc32b = t;
        end
    endfunction

    function [31:0] wfcs;
        input dummy;
        integer k;
        reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = woff; k < wlen; k = k + 1)
                c = wcrc32b(c, wbuf[k]);
            wfcs = c;
        end
    endfunction

    always @(posedge clk) begin
        if (reset_a) begin
            wact <= 0; wlen <= 0; wframes <= 0; wframes_afdx <= 0; wframes_data <= 0;
            w_our_frames <= 0;
            wbad_len <= 0; wbad_fmt <= 0; wbad_sn <= 0; wbad_fcs <= 0; wbad_etype <= 0;
            wbad_saif <= 0; wbad_sip <= 0; wbag_bad <= 0; wbad_preamble <= 0;
            wseen_n <= 0;
        end else if (a_ga_txen) begin
            if (!wact) begin
                wact <= 1;
                wlen <= 1;
                w_start <= $time;
                wbuf[0] <= a_ga_txd;      // 首字节单独写入，不能被 wlen 覆盖
            end else begin
                if (wlen < 2048) wbuf[wlen] <= a_ga_txd;
                wlen <= wlen + 1;
            end
        end else if (wact) begin
            wact <= 0;
            wframes <= wframes + 1;

            // 兼容两种 GMII 捕获：带前导 8 B，或 txen 只包住 DA..FCS
            if ((wlen >= 8) && (wbuf[0] === 8'h55) && (wbuf[7] === 8'hD5))
                woff = 8;
            else if ((wlen >= 14) && (wbuf[0] === 8'h03) &&
                     (wbuf[1] === 8'h00) && (wbuf[2] === 8'h00) && (wbuf[3] === 8'h00))
                woff = 0;
            else
                woff = 8;

            if ((woff == 8) && ((wbuf[0] !== 8'h55) || (wbuf[7] !== 8'hD5))) begin
                wbad_preamble <= wbad_preamble + 1;
            end else begin
                wframes_afdx <= wframes_afdx + 1;
                if ((wlen < woff + 14) || (wlen > 1526) || ((wlen - woff) > 1518))
                    wbad_len <= wbad_len + 1;

                if ((wbuf[woff] !== 8'h03) || (wbuf[woff+1] !== 8'h00) ||
                    (wbuf[woff+2] !== 8'h00) || (wbuf[woff+3] !== 8'h00))
                    wbad_fmt <= wbad_fmt + 1;

                if (wlen >= woff + 14)
                    wvl = {wbuf[woff+4], wbuf[woff+5]};

                if (wlen >= woff + 14) begin
                    if ((wbuf[woff+12] !== 8'h08) || (wbuf[woff+13] !== 8'h00))
                        wbad_etype <= wbad_etype + 1;
                end

                if ((wvl >= 16'h0200) && (wvl <= 16'h027F)) begin
                    wframes_data <= wframes_data + 1;
                    w_our_frames <= w_our_frames + 1;
                    if (wlen >= woff + 12) begin
                        if (wbuf[woff+11] !== 8'h20) wbad_saif <= wbad_saif + 1;
                    end
                    if (wlen >= woff + 30) begin
                        if ({wbuf[woff+26], wbuf[woff+27], wbuf[woff+28], wbuf[woff+29]}
                            !== 32'h0A000001) wbad_sip <= wbad_sip + 1;
                    end
                    if (wbag_seen[wvl] && ((w_start - wlast_t[wvl]) < 1_000_000))
                        wbag_bad <= wbag_bad + 1;
                    wlast_t[wvl] = w_start;
                    wbag_seen[wvl] = 1'b1;
                    if (!wseen[wvl]) begin
                        wseen[wvl] = 1'b1;
                        wseen_n <= wseen_n + 1;
                    end
                end else if (wvl == 16'h0ABC) begin
                    w_our_frames <= w_our_frames + 1;
                end

                if (wlen >= 5) begin
                    wsn = wbuf[wlen-5];
                    if (wsn == 0) wbad_sn <= wbad_sn + 1;
                end
                if (wfcs(1'b0) !== 32'hDEBB20E3) wbad_fcs <= wbad_fcs + 1;
            end
        end
    end

    // A 侧 B 网帧计数（证明一条应用消息确实生成 A/B 两帧）
    always @(posedge clk) begin
        if (reset_a) begin
            b_net_act <= 0; b_net_frames <= 0;
        end else if (a_gb_txen) begin
            b_net_act <= 1;
        end else if (b_net_act) begin
            b_net_act <= 0;
            b_net_frames <= b_net_frames + 1;
        end
    end

    // =================================================================
    // 发送任务：一条连续 valid 段 = 一条消息
    // =================================================================
    task send_a_idx;
        input integer mi;
        integer k;
        reg [31:0] w;
        begin
            @(negedge clk); a_tx_vl = av_vlid[mi];
            for (k = 0; k < av_len[mi]; k = k + 1) begin
                w = avmem[av_pst[mi] + (k >> 2)];
                @(negedge clk);
                a_tx_v  = 1'b1;
                a_tx_d  = w[(k & 3) * 8 +: 8];
                if (k == 0) begin
                    if (p_tx_n < MAXAV) p_tx_t[p_tx_n] = $time;
                    p_tx_n = p_tx_n + 1;
                    if (((MODE == 0) || (MODE == 3) || (MODE == 5)))
                        $display("[MARK] %0t A-TX vlid=0x%04X len=%0d",
                                 $time, av_vlid[mi], av_len[mi]);
                end
                @(posedge clk);
            end
            @(negedge clk); a_tx_v = 1'b0; a_tx_d = 8'h00;
        end
    endtask

    // 只发一条消息的前 n 字节：MODE=4 的 9 条短配置（5 B）
    task send_a_idx_n;
        input integer mi;
        input integer n;
        integer k;
        reg [31:0] w;
        begin
            @(negedge clk); a_tx_vl = av_vlid[mi];
            for (k = 0; k < n; k = k + 1) begin
                w = avmem[av_pst[mi] + (k >> 2)];
                @(negedge clk);
                a_tx_v  = 1'b1;
                a_tx_d  = w[(k & 3) * 8 +: 8];
                if (k == 0) begin
                    if (p_tx_n < MAXAV) p_tx_t[p_tx_n] = $time;
                    p_tx_n = p_tx_n + 1;
                    if (MODE == 4)
                        $display("[MARK] %0t A-CFG-TX vlid=0x%04X len=%0d",
                                 $time, av_vlid[mi], n);
                end
                @(posedge clk);
            end
            @(negedge clk); a_tx_v = 1'b0; a_tx_d = 8'h00;
        end
    endtask

    task send_b_idx;
        input integer mi;
        integer k;
        reg [31:0] w;
        begin
            @(negedge clk); b_tx_vl = av_vlid[mi];
            for (k = 0; k < av_len[mi]; k = k + 1) begin
                w = avmem[av_pst[mi] + (k >> 2)];
                @(negedge clk);
                b_tx_v  = 1'b1;
                b_tx_d  = w[(k & 3) * 8 +: 8];
                if (k == 0) begin
                    if (((MODE == 0) || (MODE == 3) || (MODE == 5)))
                        $display("[MARK] %0t B-TX vlid=0x%04X len=%0d",
                                 $time, av_vlid[mi], av_len[mi]);
                end
                @(posedge clk);
            end
            @(negedge clk); b_tx_v = 1'b0; b_tx_d = 8'h00;
        end
    endtask

    task send_a_all;
        integer i;
        begin
            for (i = 0; i < av_n; i = i + 1)
                if (av_dir[av_base+i] == 0) send_a_idx(av_base+i);
        end
    endtask

    task send_b_all;
        integer i;
        begin
            for (i = 0; i < av_n; i = i + 1)
                if (av_dir[av_base+i] == 1) send_b_idx(av_base+i);
        end
    endtask

    task wait_done;
        integer w;
        begin
            w = 0;
            while (((a_msg_n < exp_a_n) || (b_msg_n < exp_b_n)) && (w < WAIT_MSG)) begin
                @(posedge clk);
                w = w + 1;
            end
        end
    endtask

    // =================================================================
    // 判定
    // =================================================================
    integer errs = 0;
    task chk;
        input c;
        input [255:0] name;
        begin
            if (c) $display("  [ok]   %0s", name);
            else begin errs = errs + 1; $display("  [FAIL] %0s", name); end
        end
    endtask

    task check_a_idx;
        input integer mi;
        input integer idx;
        input [255:0] tname;
        integer k;
        reg ok;
        begin
            ok = (idx < a_msg_n);
            if (ok) begin
                if (a_msg_vl[idx] !== av_vlid[mi]) ok = 0;
                if (a_msg_len[idx] !== av_len[mi]) ok = 0;
            end
            if (ok) begin
                for (k = 0; k < av_len[mi]; k = k + 1)
                    if (a_msgbuf[a_msg_start[idx] + k] !==
                        avmem[av_pst[mi] + (k >> 2)][(k & 3) * 8 +: 8]) ok = 0;
            end
            if (ok)
                chk(1'b1, tname);
            else begin
                chk(1'b0, tname);
                $display("         [dbg] B->A record mi=%0d idx=%0d dir=%0d",
                         mi, idx, av_dir[mi]);
                $display("               exp vlid=%04x len=%0d, got vlid=%04x len=%0d",
                         av_vlid[mi], av_len[mi],
                         (idx < a_msg_n) ? a_msg_vl[idx] : 16'hXXXX,
                         (idx < a_msg_n) ? a_msg_len[idx] : -1);
            end
        end
    endtask

    task check_b_idx;
        input integer mi;
        input integer idx;
        input [255:0] tname;
        integer k;
        reg ok;
        begin
            ok = (idx < b_msg_n);
            if (ok) begin
                if (b_msg_vl[idx] !== av_vlid[mi]) ok = 0;
                if (b_msg_len[idx] !== av_len[mi]) ok = 0;
            end
            if (ok) begin
                for (k = 0; k < av_len[mi]; k = k + 1)
                    if (b_msgbuf[b_msg_start[idx] + k] !==
                        avmem[av_pst[mi] + (k >> 2)][(k & 3) * 8 +: 8]) ok = 0;
            end
            if (ok)
                chk(1'b1, tname);
            else begin
                chk(1'b0, tname);
                $display("         [dbg] A->B record mi=%0d idx=%0d dir=%0d",
                         mi, idx, av_dir[mi]);
                $display("               exp vlid=%04x len=%0d, got vlid=%04x len=%0d",
                         av_vlid[mi], av_len[mi],
                         (idx < b_msg_n) ? b_msg_vl[idx] : 16'hXXXX,
                         (idx < b_msg_n) ? b_msg_len[idx] : -1);
            end
        end
    endtask

    // =================================================================
    // GMII 模式（MODE=1）：单 ES 文件激励接收（原 GMII 单实例台的 RX 部分）
    //   A 的 GMII 输入由本 TB 驱动，B 保持复位；文件格式不变。
    // =================================================================
    localparam integer MAXF   = 32;
    localparam integer MAXA   = 256;
    localparam integer MAXV   = 32768;
    localparam integer MAXGB  = 262144;
    localparam integer WAIT_ALL = 100000;
    localparam GMII_A_FILE  = "AFDX_ES.srcs/sim_1/new/gmii_a_frames.hex";
    localparam GMII_B_FILE  = "AFDX_ES.srcs/sim_1/new/gmii_b_frames.hex";
    localparam APP_EXP_FILE = "AFDX_ES.srcs/sim_1/new/app_expect.hex";

    reg [31:0] rxamem [0:MAXV-1];
    reg [31:0] rxbmem [0:MAXV-1];
    reg [31:0] rxemem [0:MAXV-1];

    integer rx_a_n = 0, rx_b_n = 0, rx_e_n = 0;
    integer ra_vlid [0:MAXF-1];
    integer ra_port [0:MAXF-1];
    integer ra_wlen [0:MAXF-1];
    integer ra_off  [0:MAXF-1];
    integer ra_wst  [0:MAXF-1];
    integer rb_vlid [0:MAXF-1];
    integer rb_port [0:MAXF-1];
    integer rb_wlen [0:MAXF-1];
    integer rb_off  [0:MAXF-1];
    integer rb_wst  [0:MAXF-1];
    integer re_port [0:MAXA-1];
    integer re_vlid [0:MAXA-1];
    integer re_len  [0:MAXA-1];
    integer re_pst  [0:MAXA-1];
    integer gm_off;

    task load_gmii_vectors;
        integer i, off;
        begin
            $display("  [load] GMII A ...");
            $readmemh(GMII_A_FILE, rxamem);
            $readmemh(GMII_B_FILE, rxbmem);
            $readmemh(APP_EXP_FILE, rxemem);
            rx_a_n = rxamem[0]; if (rx_a_n > MAXF) rx_a_n = MAXF;
            rx_b_n = rxbmem[0]; if (rx_b_n > MAXF) rx_b_n = MAXF;
            rx_e_n = rxemem[0]; if (rx_e_n > MAXA) rx_e_n = MAXA;

            off = 1;
            for (i = 0; i < rx_a_n; i = i + 1) begin
                ra_vlid[i] = rxamem[off+2];
                ra_port[i] = rxamem[off+4];
                ra_wlen[i] = rxamem[off+5];
                ra_off [i] = rxamem[off+6];
                ra_wst [i] = off + 7;
                off = off + 7 + ra_wlen[i];
            end
            off = 1;
            for (i = 0; i < rx_b_n; i = i + 1) begin
                rb_vlid[i] = rxbmem[off+2];
                rb_port[i] = rxbmem[off+4];
                rb_wlen[i] = rxbmem[off+5];
                rb_off [i] = rxbmem[off+6];
                rb_wst [i] = off + 7;
                off = off + 7 + rb_wlen[i];
            end
            off = 1;
            for (i = 0; i < rx_e_n; i = i + 1) begin
                re_port[i] = rxemem[off+1];
                re_vlid[i] = rxemem[off+2];
                re_len [i] = rxemem[off+4];
                re_pst [i] = off + 5;
                off = off + 5 + re_len[i];
            end
            $display("  [vec] GMII A=%0d B=%0d frames, expected=%0d messages",
                     rx_a_n, rx_b_n, rx_e_n);
        end
    endtask

    // GMII 模式的应用口捕获（所有消息；不按 VLID 过滤）
    reg [7:0] ga_buf [0:MAXGB-1];
    integer   ga_start [0:MAXA-1];
    integer   ga_len   [0:MAXA-1];
    reg [3:0] ga_port  [0:MAXA-1];
    integer   ga_n = 0, ga_wp = 0;
    reg       ga_seg = 0;

    always @(posedge clk) begin
        if (reset_a) begin
            ga_n <= 0; ga_wp <= 0; ga_seg <= 0;
        end else if (a_rx_v) begin
            if (!ga_seg) begin
                ga_seg <= 1'b1;
                if (ga_n < MAXA) begin
                    ga_start[ga_n] <= ga_wp;
                    ga_len  [ga_n] <= 1;
                    ga_port [ga_n] <= a_rx_p;
                end
                if (ga_wp < MAXGB-1) begin
                    ga_buf[ga_wp] <= a_rx_d;
                    ga_wp <= ga_wp + 1;
                end
            end else begin
                if (ga_wp < MAXGB-1) begin
                    ga_buf[ga_wp] <= a_rx_d;
                    ga_wp <= ga_wp + 1;
                end
                if (ga_n < MAXA) ga_len[ga_n] <= ga_len[ga_n] + 1;
            end
        end else if (ga_seg) begin
            ga_seg <= 1'b0;
            if (ga_n < MAXA) ga_n <= ga_n + 1;
        end
    end

    integer cyc = 0;
    always @(posedge clk) cyc <= cyc + 1;
    integer run_start = 0;

    task gmii_send_a;
        integer fi, k;
        begin
            for (fi = 0; fi < rx_a_n; fi = fi + 1) begin
                while ((cyc - run_start) < ra_off[fi]) @(posedge clk);
                for (k = 0; k < ra_wlen[fi]; k = k + 1) begin
                    @(posedge clk);
                    tb_gmii_a_rxd  <= rxamem[ra_wst[fi]+k][7:0];
                    tb_gmii_a_rxdv <= 1'b1;
                    tb_gmii_a_rxer <= 1'b0;
                end
                @(posedge clk);
                tb_gmii_a_rxdv <= 1'b0;
                tb_gmii_a_rxd  <= 8'h00;
                repeat (12) @(posedge clk);
            end
        end
    endtask

    task gmii_send_b;
        integer fi, k;
        begin
            for (fi = 0; fi < rx_b_n; fi = fi + 1) begin
                while ((cyc - run_start) < rb_off[fi]) @(posedge clk);
                for (k = 0; k < rb_wlen[fi]; k = k + 1) begin
                    @(posedge clk);
                    tb_gmii_b_rxd  <= rxbmem[rb_wst[fi]+k][7:0];
                    tb_gmii_b_rxdv <= 1'b1;
                    tb_gmii_b_rxer <= 1'b0;
                end
                @(posedge clk);
                tb_gmii_b_rxdv <= 1'b0;
                tb_gmii_b_rxd  <= 8'h00;
                repeat (12) @(posedge clk);
            end
        end
    endtask

    task gmii_check_one;
        input integer mm;
        output ok;
        integer kk;
        reg ok;
        begin
            ok = (mm < ga_n);
            if (ok) begin
                if (ga_port[mm] !== re_port[mm]) ok = 0;
                if (ga_len [mm] !== re_len [mm]) ok = 0;
                if (ok) begin
                    for (kk = 0; kk < re_len[mm]; kk = kk + 1)
                        if (ga_buf[ga_start[mm]+kk] !== rxemem[re_pst[mm]+kk][7:0])
                            ok = 0;
                end
            end
            if (!ok) begin
                $display("  [FAIL] G4 GMII msg %0d: exp port=%0d len=%0d, got port=%0d len=%0d",
                         mm, re_port[mm], re_len[mm],
                         (mm < ga_n) ? ga_port[mm] : 4'hX,
                         (mm < ga_n) ? ga_len[mm] : -1);
            end
        end
    endtask

    // =================================================================
    // MODE=2 性能测试任务
    // =================================================================
    task send_perf_config;
        integer i;
        begin
            for (i = 0; i < av_n; i = i + 1) begin
                if ((av_kind[av_base+i] == 0) && (av_dir[av_base+i] == 0)) begin
                    send_a_idx(av_base+i);
                    repeat (2000) @(posedge clk);   // 给 app_bridge/tx_path 排空
                end
            end
        end
    endtask

    task send_perf_rounds;
        integer r, i;
        begin
            for (r = 0; r < PERF_ROUNDS; r = r + 1) begin
                for (i = 0; i < 9; i = i + 1) begin
                    send_a_idx(av_base+i);              // 0x0200..0x0208
                    repeat (PERF_GAP) @(posedge clk); // 1471 + 13242 = 100 Mbps
                end
            end
        end
    endtask

    task send_cfg_short;
        integer i, w, base_n;
        begin
            base_n = p_n;
            for (i = 0; i < PERF_CONFIG_N; i = i + 1) begin
                send_a_idx_n(av_base+i, PERF_CONFIG_LEN);
                w = 0;
                while ((p_n < base_n + i + 1) && (w < 100000)) begin
                    @(posedge clk); w = w + 1;
                end
                repeat (PERF_CONFIG_GAP) @(posedge clk);
            end
        end
    endtask

    // =================================================================
    // 主流程
    // =================================================================
    integer i, ia, ib, tmo, bad_len, g_ok, m;
    time    perf_t0, perf_t1, lat, maxlat, jit_max;
    time    lat_lo [0:4095];
    time    lat_hi [0:4095];
    reg     lat_seen [0:4095];
    integer perf_bytes, vi, j;
    real    perf_rate;
    reg g_one_ok;
    reg smoke_booted;

    // 0 时刻检查：BYPASS=1 时 soc_boot_done 是 vcc，不允许开始长仿真
    initial begin
        #1;
        if ((MODE != 5) && ((a_boot === 1'b1) || (b_boot === 1'b1))) begin
            $display(">> FAIL: BYPASS=1 (soc_boot_done=vcc); need BYPASS=0 netlist");
            $finish;
        end
    end

    // MODE=5 诊断：1 µs 时打印 boot / MAC 配置状态
    initial begin
        if (MODE == 5) begin
            #1000;
            $display("  [smoke] t=1us: maccfg A=%b B=%b, boot A=%b B=%b",
                     a_maccfg, b_maccfg, a_boot, b_boot);
        end
    end

    initial begin
        gmii_inject = (MODE == 1);
        if (MODE == 0) begin
        load_app_vec(APP_FILE, 0, 2);
        build_exp;

        $display("");
        $display("======== two-ES app-side exchange (A/B own flash / own source IP) ========");
        $display("  [vec] total %0d, good A->B %0d, B->A %0d", av_n, exp_b_n, exp_a_n);

        #200;
        reset_a = 1'b0;
        reset_b = 1'b0;
        tmo = 0;
        while (((a_boot !== 1'b1) || (b_boot !== 1'b1)) && (tmo < 3000000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        chk(a_boot === 1'b1, "T1 A boot");
        chk(b_boot === 1'b1, "T2 B boot");
`ifdef POST_SYN
        repeat (BOOT_QUIET) @(posedge clk);
`else
        wait_fw_ready;
`endif

        // 清监视器（启动期间可能有固件自己的服务帧）
        a_msg_n = 0; a_wp = 0; a_seg = 1'b0; a_seg_ours = 1'b0; a_bad_vlid_n = 0;
        b_msg_n = 0; b_wp = 0; b_seg = 1'b0; b_seg_ours = 1'b0; b_bad_vlid_n = 0;

        // 两个方向同时注入
        fork
            send_a_all;
            send_b_all;
        join

        wait_done;

        $display("  [dbg] A got %0d/%0d msgs, B got %0d/%0d msgs, err A=%b B=%b",
                 a_msg_n, exp_a_n, b_msg_n, exp_b_n, a_err, b_err);

        chk(a_msg_n == exp_a_n, "T3 A rx count");
        chk(b_msg_n == exp_b_n, "T4 B rx count");

        ia = 0;
        for (i = 0; i < av_n; i = i + 1) begin
            if ((av_kind[av_base+i] == 0) && (av_dir[av_base+i] == 1)) begin
                case (ia)
                    0: check_a_idx(av_base+i, ia, "T5 B->A #1 5B");
                    1: check_a_idx(av_base+i, ia, "T6 B->A #2 64B");
                    default: check_a_idx(av_base+i, ia, "T6x B->A extra");
                endcase
                ia = ia + 1;
            end
        end
        ib = 0;
        for (i = 0; i < av_n; i = i + 1) begin
            if ((av_kind[av_base+i] == 0) && (av_dir[av_base+i] == 0)) begin
                case (ib)
                    0: check_b_idx(av_base+i, ib, "T7 A->B #1 5B");
                    1: check_b_idx(av_base+i, ib, "T8 A->B #2 64B");
                    default: check_b_idx(av_base+i, ib, "T8x A->B extra");
                endcase
                ib = ib + 1;
            end
        end
        chk((ia == exp_a_n) && (ib == exp_b_n), "T9 all records");
        $display("  [SCREENSHOT] MODE0: A->B=%0d B->A=%0d errA=%b errB=%b framesA=%0d framesB=%0d",
                 b_msg_n, a_msg_n, a_err, b_err, wframes, b_net_frames);

        // 快速向量只有 2 条正常消息，没有错误记录；错误注入在完整 app_side 的其它记录中。
        bad_len = 0;
        for (i = 0; i < av_n; i = i + 1)
            if (av_kind[av_base+i] != 0) bad_len = 1;
        if (bad_len) begin
            chk((a_bad_vlid_n == 0) && (b_bad_vlid_n == 0), "T10 bad VLID");
            bad_len = 0;
            for (i = 0; i < a_msg_n; i = i + 1)
                if (a_msg_len[i] > 1471) bad_len = 1;
            for (i = 0; i < b_msg_n; i = i + 1)
                if (b_msg_len[i] > 1471) bad_len = 1;
            chk(bad_len == 0, "T11 bad length");
            chk((a_err === 1'b1) && (b_err === 1'b1), "T12 errors rejected");
            $display("  [dbg] bad counters: A_vlid=%0d B_vlid=%0d bad_len=%0d",
                     a_bad_vlid_n, b_bad_vlid_n, bad_len);
        end else begin
            $display("  [dbg] fast vector: no error records, skip T10-T12");
        end

        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: two-ES app-side exchange, payloads match");
        else
            $display(">> FAIL: %0d checks failed", errs);
        $finish;
    end else if (MODE == 1) begin
        // ---------------- MODE=1：单 ES GMII → 应用口 ----------------
        load_gmii_vectors;
        reset_b = 1'b1;                       // B 不参与本模式
        #200;
        reset_a = 1'b0;

        tmo = 0;
        while ((a_maccfg !== 1'b1) && (tmo < 2000000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        chk(a_maccfg === 1'b1, "G1 GMII mac cfg");

        tmo = 0;
        while ((a_boot !== 1'b1) && (tmo < 2000000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        chk(a_boot === 1'b1, "G2 GMII boot");
        repeat (QUIET_A) @(posedge clk);

        ga_n = 0; ga_wp = 0; ga_seg = 1'b0;
        run_start = cyc;
        fork
            gmii_send_a;
            gmii_send_b;
        join

        tmo = 0;
        while ((ga_n < rx_e_n) && (tmo < WAIT_ALL)) begin
            @(posedge clk); tmo = tmo + 1;
        end

        $display("  [dbg] GMII got %0d/%0d messages, err=%b", ga_n, rx_e_n, a_err);
        chk(ga_n == rx_e_n, "G3 GMII count");

        g_ok = (ga_n == rx_e_n);
        for (m = 0; (m < rx_e_n) && (m < ga_n); m = m + 1) begin
            gmii_check_one(m, g_one_ok);
            if (!g_one_ok) g_ok = 0;
        end
        chk(g_ok, "G4 GMII payloads");

        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: GMII -> app receive all correct");
        else
            $display(">> FAIL: %0d checks failed", errs);
        $finish;
    end else if (MODE == 2) begin
        // ---------------- MODE=2：128 VL / 100 Mbps / 时延 / 帧格式 ----------------
        load_app_vec(APP_FILE, 2, PERF_N);

        #200;
        reset_a = 1'b0;
        reset_b = 1'b0;
        tmo = 0;
        while (((a_boot !== 1'b1) || (b_boot !== 1'b1)) && (tmo < 3000000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        chk(a_boot === 1'b1, "P1 A boot");
        chk(b_boot === 1'b1, "P2 B boot");
`ifdef POST_SYN
        repeat (BOOT_QUIET) @(posedge clk);
`else
        wait_fw_ready;
`endif

        // 清性能计数（启动期间的服务帧不纳入）
        p_n = 0; p_wp = 0; p_tx_n = 0; p_seg = 0; p_ours = 0;
        wframes = 0; w_our_frames = 0; wbad_len = 0; wbad_fmt = 0; wbad_sn = 0; wbad_fcs = 0; wbad_etype = 0;
        wbad_saif = 0; wbad_sip = 0;
        wframes_afdx = 0; wframes_data = 0; wbad_preamble = 0;
        wseen_n = 0;
        for (i = 0; i < 4096; i = i + 1) wseen[i] = 1'b0;
        svc_bad = 0; svc_check = 1'b1;

        // ---- 128 条 VL 各一条 1471 B，逐条注入 ----
        send_perf_config;
        tmo = 0;
        while ((p_n < PERF_N) && (tmo < 3000000)) begin @(posedge clk); tmo = tmo + 1; end
        chk(p_n == PERF_N, "P3 128 VL delivered");

        // 128 条 VL 的线侧 DA/VLID、长度、载荷抽样
        g_ok = 1'b1;
        for (i = 0; i < PERF_N; i = i + 1) begin
            if (!wseen[16'h0200 + i]) g_ok = 0;
            if (p_len[i] !== 1471) g_ok = 0;
            if (p_port[i] !== 4'd5) g_ok = 0;
            for (j = 0; j < 8; j = j + 1)
                if (p_buf[p_start[i] + j] !==
                    avmem[av_pst[av_base+i] + (j >> 2)][(j & 3) * 8 +: 8]) g_ok = 0;
        end
        chk(g_ok, "P4 128 VL payload");

        // ---- 100 Mbps 持续：9 条 VL × 2 轮，每条 1471 B ----
        send_perf_rounds;
        perf_t0 = p_tx_t[PERF_N];   // 必须在首批 perf 写入后再采样
        tmo = 0;
        while ((p_n < PERF_TOTAL) && (tmo < 6000000)) begin @(posedge clk); tmo = tmo + 1; end
        chk(p_n == PERF_TOTAL, "P5 no loss perf total");
        chk((a_err === 1'b0) && (b_err === 1'b0), "P6 err=0");

        perf_t1 = p_tx_t[PERF_TOTAL-1];
        perf_bytes = (PERF_TOTAL - PERF_N) * 1471 + 2 * (PERF_TOTAL - PERF_N);
        if (perf_t1 > perf_t0)
            perf_rate = perf_bytes * 8.0 * 1000.0 / (perf_t1 - perf_t0);
        else
            perf_rate = 0.0;
        $display("  [dbg] perf: %0d B (payload+VLID) / %0d ns = %0.1f Mbps",
                 perf_bytes, perf_t1 - perf_t0, perf_rate);
        chk(perf_rate >= 100000.0, "P7 100Mbps injection");

        // ---- 任意 VL 首字节 tx_data → rx_data 时延 ----
        maxlat = 0;
        for (i = 0; i < PERF_TOTAL; i = i + 1) begin
            lat = p_t[i] - p_tx_t[i];
            if (lat > maxlat) maxlat = lat;
        end
        $display("  [dbg] max latency = %0d ns", maxlat);
        chk(maxlat < 2000000, "P8 latency <2ms");

        // ---- 同 VL 抖动：9 条 perf VL 的 max-min ----
        for (i = 0; i < 4096; i = i + 1) begin
            lat_lo[i] = 0; lat_hi[i] = 0; lat_seen[i] = 1'b0;
        end
        for (i = PERF_N; i < PERF_TOTAL; i = i + 1) begin
            vi = p_vl[i];
            if (vi < 4096) begin
                lat = p_t[i] - p_tx_t[i];
                if (!lat_seen[vi]) begin
                    lat_lo[vi] = lat; lat_hi[vi] = lat; lat_seen[vi] = 1'b1;
                end else begin
                    if (lat < lat_lo[vi]) lat_lo[vi] = lat;
                    if (lat > lat_hi[vi]) lat_hi[vi] = lat;
                end
            end
        end
        jit_max = 0;
        for (i = 0; i < 4096; i = i + 1)
            if (lat_seen[i] && ((lat_hi[i] - lat_lo[i]) > jit_max))
                jit_max = lat_hi[i] - lat_lo[i];
        $display("  [dbg] max jitter = %0d ns", jit_max);
        chk(jit_max < 40000, "P9 jitter <40us");

        // ---- 帧格式：长度 / DA / EtherType / SN / FCS ----
        chk((wbad_len === 0) && (wbad_fmt === 0) &&
            (wbad_etype === 0) && (wbad_sn === 0) && (wbad_fcs === 0) &&
            (wbad_saif === 0) && (wbad_sip === 0) && (wbad_preamble === 0) &&
            (svc_bad === 0),
            "P10 frame format");
        chk(wseen_n >= PERF_N, "P11 128 VL wire");
        chk((w_our_frames >= PERF_TOTAL) && (b_net_frames >= PERF_TOTAL),
            "P12 A/B frames");
        $display("  [SCREENSHOT] MODE2: ourFramesA=%0d framesA=%0d framesB=%0d bad_len=%0d bad_fmt=%0d bad_etype=%0d bad_sn=%0d bad_fcs=%0d",
                 w_our_frames, wframes, b_net_frames, wbad_len, wbad_fmt, wbad_etype, wbad_sn, wbad_fcs);

        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: 128 VL / 100 Mbps / latency / frame format");
        else
            $display(">> FAIL: %0d checks failed", errs);
        $finish;
    end else if (MODE == 3) begin
        // ---------------- MODE=3：错误注入 / 不交付 / err 粘滞 ----------------
        load_app_vec(APP_FILE, 130, 4);

        #200;
        reset_a = 1'b0;
        reset_b = 1'b0;
        tmo = 0;
        while (((a_boot !== 1'b1) || (b_boot !== 1'b1)) && (tmo < 3000000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        chk(a_boot === 1'b1, "E1 A boot");
        chk(b_boot === 1'b1, "E2 B boot");
`ifdef POST_SYN
        repeat (BOOT_QUIET) @(posedge clk);
`else
        wait_fw_ready;
`endif

        p_n = 0; p_wp = 0; p_tx_n = 0; p_seg = 0; p_ours = 0;
        wframes = 0; w_our_frames = 0; b_net_frames = 0;
        wbad_len = 0; wbad_fmt = 0; wbad_sn = 0; wbad_fcs = 0; wbad_etype = 0;
        wseen_n = 0;

        fork
            send_a_all;   // 未知 VLID 0x0ABC + 1472 B 超长
            send_b_all;   // B→A 同样两类错误
        join

        repeat (ERR_WAIT) @(posedge clk);
        $display("  [dbg] error test: p_n=%0d a_err=%b b_err=%b framesA=%0d framesB=%0d",
                 p_n, a_err, b_err, wframes, b_net_frames);
        chk(p_n == 0, "E3 no app delivery");
        chk((a_err === 1'b1) && (b_err === 1'b1), "E4 err sticky");
        chk(w_our_frames == 0, "E5 no wire frames");
        $display("  [SCREENSHOT] MODE3: app_delivered=%0d errA=%b errB=%b ourFramesA=%0d framesB=%0d",
                 p_n, a_err, b_err, w_our_frames, b_net_frames);

        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: error injection rejected (no delivery, err sticky)");
        else
            $display(">> FAIL: %0d checks failed", errs);
        $finish;
    end else if (MODE == 5) begin
        // ---------------- MODE=5：门级最短冒烟（只发 2 条） ----------------
        // 只验证：网表能装载 + 时钟/复位 + 应用侧能注入 2 条。
        // BYPASS=0 网表的 MAC 配置要等很久，本模式只等 0.8 µs 就结束，
        // 打印 boot 状态后退出，不会又跑到 3 ms 以上。
        load_app_vec(APP_FILE, 0, 2);
        build_exp;

        $display("");
        $display("======== MODE=5 gate smoke: 2 messages (HELLO/WORLD) ========");
        $display("  [smoke] A->B vlid=0x%04X len=%0d / B->A vlid=0x%04X len=%0d",
                 av_vlid[0], av_len[0], av_vlid[1], av_len[1]);

        #200;
        reset_a = 1'b0;
        reset_b = 1'b0;

        tmo = 0;
        while (((a_boot !== 1'b1) || (b_boot !== 1'b1)) && (tmo < SMOKE_BOOT_WAIT)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        smoke_booted = (a_boot === 1'b1) && (b_boot === 1'b1);
        $display("  [smoke] t=%0t boot A=%b B=%b, mac_cfg A=%b B=%b (%0d cycles)",
                 $time, a_boot, b_boot, a_maccfg, b_maccfg, tmo);

        // BYPASS=0：boot 没到，直接发 2 条做注入冒烟，约 10 µs 结束。
        // 清启动期间监视器
        a_msg_n = 0; a_wp = 0; a_seg = 1'b0; a_seg_ours = 1'b0; a_bad_vlid_n = 0;
        b_msg_n = 0; b_wp = 0; b_seg = 1'b0; b_seg_ours = 1'b0; b_bad_vlid_n = 0;
        p_n = 0; p_wp = 0; p_seg = 1'b0; p_ours = 1'b0;

        fork
            send_a_idx(0);    // A→B HELLO
            send_b_idx(1);    // B→A WORLD
        join

        tmo = 0;
        while (((a_msg_n < 1) || (b_msg_n < 1)) && (tmo < SMOKE_MSG_WAIT)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        $display("  [smoke] delivered A=%0d B=%0d after %0d cycles, err A=%b B=%b, t=%0t",
                 a_msg_n, b_msg_n, tmo, a_err, b_err, $time);

        if (!smoke_booted) begin
            $display("======================================================");
            $display(">> SMOKE DONE: netlist loaded, clock/reset OK, 2 frames injected.");
            $display("   boot not reached in 0.8 us (mac_cfg A=%b B=%b).", a_maccfg, b_maccfg);
            $display("   This is a LOAD smoke, not a data-path pass; use MODE=0/full test for data.");
            $finish;
        end

        chk(a_msg_n == 1, "S1 B->A delivered");
        chk(b_msg_n == 1, "S2 A->B delivered");
        check_a_idx(1, 0, "S3 B->A payload");
        check_b_idx(0, 0, "S4 A->B payload");
        chk((a_err === 1'b0) && (b_err === 1'b0), "S5 err=0");

        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: MODE=5 gate smoke (2 messages)");
        else
            $display(">> FAIL: %0d checks failed (see boot/maccfg and delivered counts)", errs);
        $finish;
    end else begin
        // ---------------- MODE=4：8 小时合并验收 ----------------
        // fast 2 条 -> 9 条短配置 -> 9 条 VL × 2 轮 100 Mbps -> 4 条错误
        load_app_vec(APP_FILE, 0, 2);
        build_exp;

        #200;
        reset_a = 1'b0;
        reset_b = 1'b0;
        tmo = 0;
        while (((a_boot !== 1'b1) || (b_boot !== 1'b1)) && (tmo < 3000000)) begin
            @(posedge clk); tmo = tmo + 1;
        end
        chk(a_boot === 1'b1, "V1 A boot");
        chk(b_boot === 1'b1, "V2 B boot");
`ifdef POST_SYN
        repeat (BOOT_QUIET) @(posedge clk);
`else
        wait_fw_ready;
`endif

        p_n = 0; p_wp = 0; p_tx_n = 0; p_seg = 0; p_ours = 0;
        wframes = 0; w_our_frames = 0; b_net_frames = 0;
        wbad_len = 0; wbad_fmt = 0; wbad_sn = 0; wbad_fcs = 0; wbad_etype = 0;
        wseen_n = 0;

        // ---- 1) 2 条快速互发 ----
        fork
            send_a_idx(0);    // A→B HELLO
            send_b_idx(1);    // B→A WORLD
        join
        wait_done;
        chk(a_msg_n == 1, "V3 fast B->A count");
        chk(b_msg_n == 1, "V4 fast A->B count");
        check_a_idx(1, 0, "V3 fast B->A payload");
        check_b_idx(0, 0, "V4 fast A->B payload");
        chk(p_port[0] === 4'd5, "V4b tx_port=5");

        // ---- 1 B 边界 ----
        p_n = 0; p_wp = 0; p_tx_n = 0; p_seg = 0; p_ours = 0;
        send_a_idx_n(0, 1);
        tmo = 0;
        while ((p_n < 1) && (tmo < 100000)) begin @(posedge clk); tmo = tmo + 1; end
        chk((p_n == 1) && (p_len[0] == 1), "V17 1B boundary");

        // ---- 2) 9 条 VL 短配置（5 B） ----
        load_app_vec(APP_FILE, 2, PERF_N);
        p_n = 0; p_wp = 0; p_tx_n = 0; p_seg = 0; p_ours = 0;
        wframes = 0; w_our_frames = 0; b_net_frames = 0;
        wbad_len = 0; wbad_fmt = 0; wbad_sn = 0; wbad_fcs = 0; wbad_etype = 0;
        wbad_saif = 0; wbad_sip = 0; wbag_bad = 0;
        wframes_afdx = 0; wframes_data = 0; wbad_preamble = 0;
        wseen_n = 0;
        for (i = 0; i < 4096; i = i + 1) begin
            wseen[i] = 1'b0; wbag_seen[i] = 1'b0; wlast_t[i] = 0;
        end
        svc_bad = 0; svc_check = 1'b1;

        send_cfg_short;
        tmo = 0;
        while ((p_n < PERF_CONFIG_N) && (tmo < 1000000)) begin @(posedge clk); tmo = tmo + 1; end
        chk(p_n == PERF_CONFIG_N, "V5 9VL config delivered");
        g_ok = 1'b1;
        for (i = 0; i < PERF_CONFIG_N; i = i + 1) begin
            if (p_len[i] !== PERF_CONFIG_LEN) g_ok = 0;
            if (p_port[i] !== 4'd5) g_ok = 0;
            if (!wseen[16'h0200 + i]) g_ok = 0;
            for (j = 0; j < PERF_CONFIG_LEN; j = j + 1)
                if (p_buf[p_start[i] + j] !==
                    avmem[av_pst[av_base+i] + (j >> 2)][(j & 3) * 8 +: 8]) g_ok = 0;
        end
        chk(g_ok, "V6 config payload/len");

        // ---- 3) 9 条 VL × 2 轮 100 Mbps ----
        send_perf_rounds;
        perf_t0 = p_tx_t[PERF_CONFIG_N];   // 必须在首批 perf 写入后再采样
        tmo = 0;
        while ((p_n < FAST_TOTAL) && (tmo < 2000000)) begin @(posedge clk); tmo = tmo + 1; end
        chk(p_n == FAST_TOTAL, "V7 no loss 27");
        chk((a_err === 1'b0) && (b_err === 1'b0), "V8 err=0");

        perf_t1 = p_tx_t[FAST_TOTAL-1];
        perf_bytes = (FAST_TOTAL - PERF_CONFIG_N) * 1471
                   + 2 * (FAST_TOTAL - PERF_CONFIG_N);
        if (perf_t1 > perf_t0)
            perf_rate = perf_bytes * 8.0 * 1000.0 / (perf_t1 - perf_t0);
        else
            perf_rate = 0.0;
        $display("  [dbg] MODE4 perf: %0d B / %0d ns = %0.1f Mbps",
                 perf_bytes, perf_t1 - perf_t0, perf_rate);
        chk(perf_rate >= 100000.0, "V9 100Mbps injection");

        maxlat = 0;
        for (i = 0; i < FAST_TOTAL; i = i + 1) begin
            lat = p_t[i] - p_tx_t[i];
            if (lat > maxlat) maxlat = lat;
        end
        chk(maxlat < 2000000, "V10 latency <2ms");

        for (i = 0; i < 4096; i = i + 1) begin
            lat_lo[i] = 0; lat_hi[i] = 0; lat_seen[i] = 1'b0;
        end
        for (i = PERF_CONFIG_N; i < FAST_TOTAL; i = i + 1) begin
            vi = p_vl[i];
            if (vi < 4096) begin
                lat = p_t[i] - p_tx_t[i];
                if (!lat_seen[vi]) begin
                    lat_lo[vi] = lat; lat_hi[vi] = lat; lat_seen[vi] = 1'b1;
                end else begin
                    if (lat < lat_lo[vi]) lat_lo[vi] = lat;
                    if (lat > lat_hi[vi]) lat_hi[vi] = lat;
                end
            end
        end
        jit_max = 0;
        for (i = 0; i < 4096; i = i + 1)
            if (lat_seen[i] && ((lat_hi[i] - lat_lo[i]) > jit_max))
                jit_max = lat_hi[i] - lat_lo[i];
        chk(jit_max < 40000, "V11 jitter <40us");

        chk((wbad_len === 0) && (wbad_fmt === 0) &&
            (wbad_etype === 0) && (wbad_sn === 0) && (wbad_fcs === 0) &&
            (wbad_saif === 0) && (wbad_sip === 0) && (wbag_bad === 0) &&
            (wbad_preamble === 0) && (svc_bad === 0),
            "V12 frame format");
        chk(wframes_data >= FAST_TOTAL, "V13 data frames");
        chk(w_our_frames >= FAST_TOTAL, "V13 A/B frames");

        // ---- 5) GAP=0 背靠背 8 条 5 B 消息，放在正常流量之后，避免污染交付统计 ----
        for (i = 0; i < 8; i = i + 1)
            send_a_idx_n(av_base + 17 + i, 5); // 0x0211..0x0218：未在 perf 段使用，避免 BAG 等待
        tmo = 0;
        while ((p_n < FAST_TOTAL + 8) && (tmo < 200000)) begin @(posedge clk); tmo = tmo + 1; end
        chk((p_n == FAST_TOTAL + 8) && (a_err === 1'b0), "V18 GAP0 8-msg FIFO");

        // ---- 6) 错误注入放最后，err 粘滞后不再发正常消息 ----
        send_a_idx(130);   // 未知 VLID
        send_b_idx(132);
        send_a_idx(131);   // 1472 B 超长
        send_b_idx(133);
        repeat (ERR_WAIT) @(posedge clk);
        chk(p_n == FAST_TOTAL + 8, "V14 errors no delivery");
        chk((a_err === 1'b1) && (b_err === 1'b1), "V15 errors sticky");
        chk(w_our_frames == FAST_TOTAL + 8, "V16 no error frames");

        $display("  [SCREENSHOT] MODE4: delivered=%0d errA=%b errB=%b ourFramesA=%0d framesB=%0d",
                 p_n, a_err, b_err, w_our_frames, b_net_frames);
        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: MODE4 8h acceptance (fast + 9VL + 2 rounds + errors)");
        else
            $display(">> FAIL: %0d checks failed", errs);
        $finish;
    end
end

    initial begin
        #80000000;
        $display(">> TIMEOUT: A=%0d/%0d B=%0d/%0d", a_msg_n, exp_a_n, b_msg_n, exp_b_n);
        $finish;
    end

    // =================================================================
    // 内嵌 SPI NOR flash 行为模型（自包含；不依赖 sim_1/new/spi_flash_model.v）
    //   与原 spi_flash_model.v 功能一致，仅改模块名避免与共用源重复定义。
    // =================================================================
endmodule

// ---------------------------------------------------------------------
// 说明：Verilog-2001 不允许模块嵌套，模型放在本文件末尾、顶层模块之外。
//       名字用 `spi_flash_model_top`，这样即使 GUI 同时编译共用的
//       `spi_flash_model.v` 也不会重复定义。
// ---------------------------------------------------------------------
module spi_flash_model_top #(
    parameter AW      = 13,
    parameter USE_IMG = 1,
    parameter IMG     = "flash.hex"
) (
    input  wire sclk,
    input  wire cs_n,
    input  wire mosi,
    output reg  miso
);
    localparam SIZE = (1 << AW);
    reg [7:0] mem [0:SIZE-1];
    integer i, sbase;

    initial begin
        if (USE_IMG) $readmemh(IMG, mem);
        else begin
            for (i = 0; i < SIZE; i = i + 1)
                mem[i] = (i * 7 + 3 + (i >> 8) * 101) & 8'hFF;
        end
    end

    localparam S_IDLE = 0, S_CMD = 1, S_ADDR = 2, S_DATA = 3,
               S_RDSR = 4, S_WDATA = 5;
    integer      st;
    reg [31:0]   shin;
    reg [4:0]    bitn;
    reg [7:0]    cmd;
    reg [AW-1:0] faddr;
    reg [7:0]    shout;
    reg [3:0]    dbc;
    reg          wel;
    reg          wr_cmd;
    reg [AW-1:0] waddr;
    reg [7:0]    wshift;
    reg [3:0]    wbit;

    always @(negedge cs_n) begin
        st <= S_CMD; bitn <= 5'd0; shin <= 32'h0; dbc <= 4'd0; wbit <= 4'd0;
        miso <= 1'b0;
    end

    always @(posedge cs_n) begin
        st <= S_IDLE;
        if (wr_cmd) wel <= 1'b0;
        wr_cmd <= 1'b0;
    end

    always @(posedge sclk) if (!cs_n) begin
        if (st == S_CMD || st == S_ADDR) begin
            shin <= {shin[30:0], mosi};
            bitn <= bitn + 5'd1;
            if (bitn == 5'd7) begin
                cmd <= {shin[6:0], mosi};
                case ({shin[6:0], mosi})
                    8'h03: st <= S_ADDR;
                    8'h06: begin wel <= 1'b1; st <= S_IDLE; end
                    8'h05: begin shout <= {6'h0, wel, 1'b0}; dbc <= 4'd0; st <= S_RDSR; end
                    8'h02: begin wr_cmd <= 1'b1; st <= S_ADDR; end
                    8'h20: begin wr_cmd <= 1'b1; st <= S_ADDR; end
                    8'hD8: begin wr_cmd <= 1'b1; st <= S_ADDR; end
                    default: $display("  [FAIL] flash model: unsupported cmd 0x%02x", {shin[6:0], mosi});
                endcase
            end
            if (bitn == 5'd31) begin
                case (cmd)
                    8'h03: begin
                        faddr <= {shin[22:0], mosi};
                        shout <= mem[{shin[22:0], mosi}];
                        dbc   <= 4'd0;
                        st    <= S_DATA;
                    end
                    8'h02: begin
                        waddr <= {shin[22:0], mosi};
                        wbit  <= 4'd0;
                        st    <= S_WDATA;
                    end
                    8'h20: begin
                        if (wel) begin
                            sbase = ({shin[22:0], mosi} & 24'hFFF000);
                            for (i = sbase; i < sbase + 4096 && i < SIZE; i = i + 1)
                                mem[i] <= 8'hFF;
                        end
                        wr_cmd <= 1'b0; wel <= 1'b0;
                        st <= S_IDLE;
                    end
                    8'hD8: begin
                        if (wel) for (i = 0; i < SIZE; i = i + 1) mem[i] <= 8'hFF;
                        wr_cmd <= 1'b0; wel <= 1'b0;
                        st <= S_IDLE;
                    end
                    default: ;
                endcase
            end
        end
        else if (st == S_WDATA) begin
            wshift <= {wshift[6:0], mosi};
            wbit   <= wbit + 4'd1;
            if (wbit == 4'd7) begin
                if (wel) mem[waddr] <= mem[waddr] & {wshift[6:0], mosi};
                waddr <= {waddr[AW-1:8], waddr[7:0] + 8'd1};
                wbit  <= 4'd0;
            end
        end
    end

    always @(negedge sclk) if (!cs_n) begin
        if (st == S_DATA) begin
            miso  <= shout[7];
            shout <= {shout[6:0], 1'b0};
            if (dbc == 4'd7) begin
                dbc   <= 4'd0;
                shout <= mem[faddr + 1'b1];
                faddr <= faddr + 1'b1;
            end
            else dbc <= dbc + 4'd1;
        end
        else if (st == S_RDSR) begin
            miso  <= shout[7];
            shout <= {shout[6:0], 1'b0};
            if (dbc == 4'd7) begin
                dbc   <= 4'd0;
                shout <= {6'h0, wel, 1'b0};
            end
            else dbc <= dbc + 4'd1;
        end
    end
endmodule
