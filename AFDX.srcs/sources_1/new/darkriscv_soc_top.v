// darkriscv_soc_top.v —— AFDX 端系统整合顶层
`timescale 1ns/1ps

module darkriscv_soc_top #(

    parameter integer TICK_DIV = 1000,     // 8 µs 时基（clk 125 MHz ÷ 1000）
    parameter integer VLID_MODE = 0,

    parameter [31:0]  SRC_IP    = 32'h0A000001
) (
    input  wire        clk,
    input  wire        reset,
    output wire [3:0]  debug,

    // ---- 来自 MAC 的接收帧流 ----
    input  wire        dp_rxin_valid,
    input  wire        dp_rxin_sop,
    input  wire        dp_rxin_eop,
    input  wire [7:0]  dp_rxin_data,
    input  wire        dp_rxin_net,

    // ---- 送给 MAC 的帧流 ----
    output wire        dp_fr_valid,
    input  wire        dp_fr_ready,
    output wire        dp_fr_sop,
    output wire        dp_fr_eop,
    output wire [7:0]  dp_fr_data,
    output wire        dp_fr_net,


    input  wire        app_rx_valid,
    input  wire [7:0]  app_rx_data,

    input  wire [15:0] app_vlid,
    input  wire [3:0]  app_port,        // 段首拍采样；越界/未驱动 → 默认端口 0
    output wire        app_tx_valid,
    output wire [7:0]  app_tx_data,
    output wire [3:0]  app_tx_port,
    output wire [15:0] app_tx_vlid,
    output wire        app_err,         // 粘滞：入向被拒 / 端口溢出


    output wire         flash_sclk,
    output wire         flash_mosi,
    input  wire         flash_miso,
    output wire         flash_cs_n,

    // ---- 观察口 ----
    output wire         cfg_hit_o,
    output wire [127:0] cfg_cfg_o,


    input  wire [63:0]  ts_rx_ts,      // {B[63:32], A[31:0]} 收帧 SFD 时间戳
    input  wire [15:0]  ts_rx_seq,     // {B[15:8],  A[7:0]}  收帧锁存序号（seqlock）
    input  wire [63:0]  ts_tx_ts,      // {B[63:32], A[31:0]} 发帧 SFD 时间戳
    input  wire [15:0]  ts_tx_seq,     // {B[15:8],  A[7:0]}  发帧锁存序号
    output wire [31:0]  ts_fine_o,

    input  wire [31:0]  ts_tx_vl_ts,   // 选中的 VL 最近一次发送的 SFD 时间戳
    output wire [12:0]  ts_tx_vl_sel,  // {net, vlid[11:0]}

    input  wire [31:0]  ts_rx_vl_ts,
    output wire [12:0]  ts_rx_vl_sel,


    input  wire [31:0]  host_awaddr,
    input  wire         host_awvalid,
    output wire         host_awready,
    input  wire [31:0]  host_wdata,
    input  wire [3:0]   host_wstrb,
    input  wire         host_wvalid,
    output wire         host_wready,
    output wire [1:0]   host_bresp,
    output wire         host_bvalid,
    input  wire         host_bready,
    input  wire [31:0]  host_araddr,
    input  wire         host_arvalid,
    output wire         host_arready,
    output wire [31:0]  host_rdata,
    output wire [1:0]   host_rresp,
    output wire         host_rvalid,
    input  wire         host_rready,
    output wire         host_irq
);

    localparam integer NPT_RX = 6;
    localparam integer NPT_TX = 10;
    localparam integer NPT    = NPT_RX;


    localparam IW = 8192;                  // 指令 8192 字 = 32 KB
    localparam DW = 8192;                  // 数据 8192 字 = 32 KB（4 条字节通道）

    // ================= CPU =================
    wire         IDREQ, IDACK;
    wire [31:0]  IADDR, IDATA;
    wire         DDREQ;
    wire [31:0]  DADDR, DATAO;
    wire [2:0]   DLEN;
    wire [3:0]   DBE;
    wire         DRW, DRD, DWR;
    wire [31:0]  DATAI_D;
    wire         DDACK_D;

    wire         irq;

    darkriscv u_cpu (
        .CLK   (clk),
        .RES   (reset),
        .IRQ   (irq),
        .IDREQ (IDREQ), .IADDR (IADDR), .IDATA (IDATA), .IDACK (IDACK), .IBERR (1'b0),
        .DDREQ (DDREQ), .DADDR (DADDR), .DLEN  (DLEN),  .DBE   (DBE),
        .DRW   (DRW),   .DRD   (DRD),   .DWR   (DWR),
        .DATAO (DATAO), .DATAI (DATAI_D), .DDACK (DDACK_D), .DBERR (1'b0),
`ifdef SIMULATION
        .ESIMREQ(1'b0), .ESIMACK(),
`endif
        .DEBUG (debug)
    );

    // ================= 指令存储器（本地，直连）=================
    reg [31:0] iram [0:IW-1];
    reg [31:0] idata_q;
    reg        iack_q;
    wire       ien = IDREQ & ~iack_q;

    always @(posedge clk) begin
        iack_q  <= ien;
        idata_q <= iram[IADDR[14:2]];
    end
    assign IDACK = iack_q;
    assign IDATA = idata_q;

    // ---- 本地数据 RAM（4 条字节通道：M4K 不支持字节使能写）----
    reg [7:0] dram0 [0:DW-1], dram1 [0:DW-1], dram2 [0:DW-1], dram3 [0:DW-1];
    reg [7:0] dq0, dq1, dq2, dq3;


    wire        periph_sel = (DADDR[31:28] == 4'h1);

    wire        port_win   = periph_sel && (DADDR[15:10] == 6'd0) && (DADDR[9:8] != 2'b11);
    wire        periph_dir = periph_sel && !port_win;
    reg         dack_q;

    wire        den      = DDREQ & ~dack_q & ~port_win & ~(periph_dir & ~DWR);
    wire        pw       = den & periph_dir &  DWR & (|DBE);
    wire        pr       = den & periph_dir & ~DWR;
    wire [3:0]  pofs     = DADDR[5:2];               // 端口内字偏移（仅直接窗口用）


    wire        pstat    = (DADDR[15:8] == 8'h03);
    wire [3:0]  stat_rs  = DADDR[5:2];
    wire        stat_wr  = pw & pstat;

    // ---- VL 配置表读口（0x1000_0500，给 SNMP 的 vlTable / 以后的 flash 加载）----
    wire        pcfg     = (DADDR[15:8] == 8'h05);
    wire [3:0]  cfg_rs   = DADDR[5:2];
    wire        cfg_wr   = pw & pcfg;
    wire [31:0] cfg_rdata;
    wire [31:0] stat_rdata;
    wire [NPT-1:0] rx_ovf_set_w, rx_deliv_p_w;
    wire [2:0]     rx_deliv_pi_w;
    wire           tx_bag_hold_w;

    wire        pglob    = (DADDR[15:8] == 8'h04);   // 全局寄存器区（0x1000_0400..043F，pofs 4 位）


    wire        pflash   = (DADDR[15:6] == 10'h18);

    wire        pmbox    = (DADDR[15:8] == 8'h0A);
    wire        pmbox_wr = pw & pmbox;          // 1 拍写

    wire        pts      = (DADDR[15:8] == 8'h07);
    wire [3:0]  fl_rs    = pofs;
    wire        fl_wr    = pw & pflash;
    wire [31:0] fl_rdata;

    wire        local_den= den & ~periph_sel;


    wire [3:0]  ppi_rx4, ppi_tx;
    wire [2:0]  ppi_rx = ppi_rx4[2:0]; // 接收端口层仍只有 6 个实例（NPW=3）；越界已由 pi_ok_rx 门控
    wire [2:0]  rx_reg;      // 接收消息窗口寄存器号（AXI 从机的 px_reg）
    wire [2:0]  rx_aux_sel;  // 接收附加窗口选择（AXI 从机的 py_sel）
    wire [2:0]  tx_reg_w;    // 发送寄存器号（AXI 从机的 tx_reg）
    wire        px_wr, px_rd, py_rd, py_wr, tx_wr;       // 由 AXI 从机驱动（py_wr=端口配置写）
    wire [31:0] px_r_data, py_w_data, tx_w_data;         // 由 AXI 从机驱动
    wire        axib_ack;    // AXI 桥给出的 1 拍 ACK
    wire [31:0] axib_rdata;  // AXI 桥读回的数据
    wire        axib_pi_en;  // AXI 从机的"选端口有效"（= ~glob）


    wire [NPT-1:0] rx_has;         // 各接收端口"有数据"位图
    wire [NPT-1:0] rx_ovf;
    reg  [31:0] periph_q;          // 外设读回寄存器（数据抓取见下方 PR_CAP）
    reg  [31:0] rtc_cnt;

    reg  [31:0] freq_inc;          // 复位值在时基块里给（32'd4294967）
    reg  [31:0] freq_acc;

    reg  [31:0] ts_fine;
    assign ts_fine_o = ts_fine;
    wire [31:0] px_rdata_w;        // 端口层消息窗口回读
    wire [15:0] py_rdata_w;        // 端口层附加窗口回读
    wire [31:0] tx_rdata_w;        // 发送层状态回读
    reg  [31:0] irq_en_w;          // IRQ_EN
    reg  [12:0] ts_vl_sel;
    reg  [12:0] ts_rx_vl_sel_r;
    assign ts_tx_vl_sel    = ts_vl_sel;
    assign ts_rx_vl_sel    = ts_rx_vl_sel_r;


    localparam PR_IDLE = 2'd0, PR_W1 = 2'd1, PR_CAP = 2'd2, PR_ACK = 2'd3;
    reg [1:0] prst;
    reg [3:0] pofs_r;
    reg       pglob_r, pstat_r, pcfg_r, pflash_r, pts_r, pmbox_r;
    // 注：`txrd_lat` 已删除 —— 它是"TX_STATUS 组合输出先存一拍"用的，那条端口窗口分支
    //     移交给 AXI 之后**没有消费者了**（综合会报 assigned but never read）。

    always @(posedge clk) begin

        if (reset) begin prst <= PR_IDLE; end
        else case (prst)

            PR_IDLE: if (DDREQ && periph_dir && !DWR) prst <= PR_W1;
            PR_W1:   prst <= PR_CAP;
            PR_CAP:  prst <= PR_ACK;
            default: prst <= PR_IDLE;      // PR_ACK：正好 1 拍 ACK
        endcase
    end

    always @(posedge clk) begin
        if (prst == PR_IDLE && DDREQ && periph_dir && !DWR) begin
            pofs_r  <= pofs;
            pglob_r <= pglob;
            pstat_r <= pstat;
            pcfg_r  <= pcfg;
            pflash_r<= pflash;
            pts_r   <= pts;
            pmbox_r <= pmbox;
        end
    end

    wire prd_strobe = (prst == PR_CAP);   // 读选通/抓数据都在这一拍（与 AXI 从机的 S_EXEC 同款）
    wire fl_rd = prd_strobe & pflash_r;
    wire mb_rd = prd_strobe & pmbox_r;

    wire [2:0] mb_reg = pmbox_wr ? pofs[2:0] : pofs_r[2:0];
    wire [31:0] mb_rdata;
    wire        mb_cpu_irq;
    wire per_ack    = (prst == PR_ACK);

    always @(posedge clk) begin
        if (prd_strobe) begin

            if (pts_r)        periph_q <= (pofs_r == 4'd0) ? {16'h0, ts_rx_seq} :
                                          (pofs_r == 4'd1) ? {16'h0, ts_tx_seq} :
                                          (pofs_r == 4'd2) ? ts_rx_ts[15:0]  :
                                          (pofs_r == 4'd3) ? ts_rx_ts[31:16] :
                                          (pofs_r == 4'd4) ? ts_rx_ts[47:32] :
                                          (pofs_r == 4'd5) ? ts_rx_ts[63:48] :
                                          (pofs_r == 4'd6) ? ts_tx_ts[15:0]  :
                                          (pofs_r == 4'd7) ? ts_tx_ts[31:16] :
                                          (pofs_r == 4'd8) ? ts_tx_ts[47:32] :
                                          (pofs_r == 4'd9) ? ts_tx_ts[63:48] :
                                          (pofs_r == 4'd10) ? {19'h0, ts_vl_sel} :   // 0x728 TX 选择器回读
                                          (pofs_r == 4'd11) ? ts_tx_vl_ts :          // 0x72C TX 该 VL 的戳
                                          (pofs_r == 4'd12) ? {19'h0, ts_rx_vl_sel_r} : // 0x730 RX 选择器回读
                                          (pofs_r == 4'd13) ? ts_rx_vl_ts :          // 0x734 RX 该 VL 的戳
                                          32'h0;
            else if (pflash_r)     periph_q <= fl_rdata;
            else if (pmbox_r)      periph_q <= mb_rdata;
            else if (pcfg_r)  periph_q <= cfg_rdata;
            else if (pstat_r) periph_q <= stat_rdata;
            else if (pglob_r)
                periph_q <= (pofs_r == 4'd1) ? {{(32-NPT){1'b0}}, irq_en_w[NPT-1:0]} :  // 0x104 IRQ_EN
                            (pofs_r == 4'd2) ? rtc_cnt[15:0] :              // 0x108 RTC_LO
                            (pofs_r == 4'd3) ? rtc_cnt[31:16] :             // 0x10C RTC_HI
                            (pofs_r == 4'd4) ? freq_inc :                   // 0x410 频率加数（DDS）
                            (pofs_r == 4'd0) ? {{(32-NPT){1'b0}}, (rx_has & irq_en_w[NPT-1:0])} : // 0x100 IRQ_STATUS
                                               32'h0;

            else
                periph_q <= 32'h0;
        end
    end


    assign DDACK_D = dack_q | per_ack | axib_ack;


    always @(posedge clk) begin
        if (reset) irq_en_w <= 32'h0;
        else if (pw && pglob && (pofs == 4'd1)) irq_en_w <= DATAO;
    end

    // 注：RTC 频率加数（DDS，0x1000_0410）的写口在**时基块内部**（见上）——
    //     它和复位分支必须同块，单开一个 always 会构成多驱动。


    always @(posedge clk) begin
        if (reset) begin
            ts_vl_sel    <= 13'd0;
            ts_rx_vl_sel_r <= 13'd0;
        end
        else begin
            if (pw && pts && (pofs == 4'd10)) ts_vl_sel    <= DATAO[12:0];
            if (pw && pts && (pofs == 4'd12)) ts_rx_vl_sel_r <= DATAO[12:0];
        end
    end


    always @(posedge clk) begin
        if (reset) dack_q <= 1'b0;
        else       dack_q <= den;
    end

    // （外设读窗口状态机已上移到读选通定义之前）


    // （外设读的数据抓取已挪到上面的 PR_CAP 那一拍）


    assign DATAI_D = port_win ? axib_rdata
                              : (periph_dir ? periph_q : {dq3, dq2, dq1, dq0});


    always @(posedge clk) begin
        dq0 <= dram0[DADDR[14:2]];
        dq1 <= dram1[DADDR[14:2]];
        dq2 <= dram2[DADDR[14:2]];
        dq3 <= dram3[DADDR[14:2]];
        if (local_den && DWR) begin
            if (DBE[0]) dram0[DADDR[14:2]] <= DATAO[ 7: 0];
            if (DBE[1]) dram1[DADDR[14:2]] <= DATAO[15: 8];
            if (DBE[2]) dram2[DADDR[14:2]] <= DATAO[23:16];
            if (DBE[3]) dram3[DADDR[14:2]] <= DATAO[31:24];
        end
    end

    // AXI 桥 + AXI 从机本阶段**不接进 CPU 关键路径**（见上面注释），
    // 两个模块都已单独验证通过（sim_axi / sim_axislave），留作未来的主机接口。


    reg [31:0] tick_cnt;
    reg [15:0] ts_now;

    localparam [31:0] NOM_RTC  = 32'd4294967;
    localparam [31:0] NOM_RTC_LO = 32'd4290672;   // NOM - 4295
    localparam [31:0] NOM_RTC_HI = 32'd4299262;   // NOM + 4295

    wire [31:0] finc_c = (freq_inc > NOM_RTC_HI) ? NOM_RTC_HI :
                         (freq_inc < NOM_RTC_LO) ? NOM_RTC_LO :
                                                     freq_inc;

    wire [32:0] freq_sum = {1'b0, freq_acc} + {1'b0, finc_c};
    always @(posedge clk) begin
        if (reset) begin
            tick_cnt <= 32'd0; ts_now <= 16'd0; rtc_cnt <= 32'd0;
            freq_acc <= 32'd0; freq_inc <= NOM_RTC;        // 标称值：2^32/1000
        end
        else begin
            // ① BAG/SkewMax 的时基：**保持纯分频**（不受调频影响）
            if (tick_cnt >= TICK_DIV - 1) begin
                tick_cnt <= 32'd0;
                ts_now   <= ts_now + 16'd1;
            end
            else tick_cnt <= tick_cnt + 32'd1;
            // ② RTC 时基：DDS 调频（频率补偿）
            freq_acc <= freq_sum[31:0];
            if (freq_sum[32]) rtc_cnt <= rtc_cnt + 32'd1;
            // ③ 频率加数写口（0x1000_0410）—— **必须在同一个 always 块**里赋值，
            //    否则与复位分支构成多驱动（Quartus Error 10028；ModelSim 只是 Warning）。
            if (pw && pglob && (pofs == 4'd4)) freq_inc <= DATAO;
        end
    end


    localparam [31:0] NOM_FINE = 32'd429496730;   // 2^32/10
    wire [31:0] fdev      = finc_c - NOM_RTC;                // 补码差（标称 = 0）
    wire        fneg      = fdev[31];
    wire [31:0] fdev_abs0 = fneg ? (~fdev + 32'd1) : fdev;
    wire [31:0] fdev_abs  = (fdev_abs0 > 32'd4295) ? 32'd4295 : fdev_abs0; // 钳到 ±1000 ppm
    wire [31:0] fadj80    = fdev_abs * 32'd100;              // 1 ppm ≈ 429.5 LSB ⇒ ×100
    wire [31:0] fine_inc  = fneg ? (NOM_FINE - fadj80) : (NOM_FINE + fadj80);
    reg  [31:0] fine_acc;
    wire [32:0] fine_sum  = {1'b0, fine_acc} + {1'b0, fine_inc};

    always @(posedge clk) begin
        if (reset) begin ts_fine <= 32'd0; fine_acc <= 32'd0; end
        else begin
            fine_acc <= fine_sum[31:0];
            if (fine_sum[32]) ts_fine <= ts_fine + 32'd1;    // 每 ~10 拍一个 80 ns 刻度
        end
    end

    // ================= VL 配置表（512×128，**预设初值只读**）=================
    wire         cfg_hit_w;
    wire [127:0] cfg_w;
    wire [15:0]  tx_cfg_vlid, rx_lk_vlid;
    wire         rx_lk_req;
    wire [15:0]  cfg_res_vlid;
    reg          erl_pend;
    reg  [2:0]   port_lat;

    afdx_cfg #(.NENT(4096), .IDXW(12)) u_cfg (
        .clk(clk), .reset(reset),

        .wr(cfg_wr), .rd(1'b0), .reg_sel(cfg_rs), .wdata(DATAO), .rdata(cfg_rdata),
        .vlid_i(tx_cfg_vlid), .vlid_rx_i(rx_lk_vlid), .rx_req_i(rx_lk_req),
        .res_vlid_o(cfg_res_vlid), .hit_o(cfg_hit_w), .cfg_o(cfg_w)
    );
    assign cfg_hit_o = cfg_hit_w;
    assign cfg_cfg_o = cfg_w;

    wire        cfg_ic_en_w = cfg_w[29];
    wire        cfg_rm_en_w = cfg_w[30];
    wire [1:0]  cfg_neten_w = cfg_w[28:27];
    wire        cfg_snvld_w = cfg_w[31];
    wire [11:0] cfg_skew_w  = cfg_w[47:36];
    wire [10:0] cfg_lmax_w  = cfg_w[26:16];

    // ================= 接收通路 =================
    wire        rxp_ok, rxp_net, rxp_desc, rxp_vlid_vld;
    wire        rxp_e_eth, rxp_e_ip, rxp_e_udp, rxp_e_len, rxp_e_fcs;
    // ---- IP 分片（afdx_reasm 用）----
    wire        rxp_frag, rxp_mf;
    wire [12:0] rxp_foff;
    wire [15:0] rxp_ipid;
    wire [10:0] rxp_plen_out;
    wire        rm_v, rm_sop, rm_eop, rm_commit, rm_abort, rm_vbytes;
    wire [7:0]  rm_d;
    wire [15:0] rm_vlid, rm_dport, rm_sport;
    wire [2:0]  rm_port;
    wire [31:0] rm_sip;
    wire        rm_net;
    wire [15:0] ev_reasm_ok_w, ev_reasm_err_w;
    wire        erl_tmo_p;      // 提前查表超时那拍 = "VLID 不在表里"（统计 rxVlMiss）
    wire [15:0] rxp_vlid, rxp_sport;
    wire [31:0] rxp_sip;
    wire [31:0] rxp_dip;
    wire [7:0]  rxp_sn;
    wire        ir_lk_req, ir_accept, ir_filt, ir_ic, ir_rm;

    wire [11:0] ir_idx;
    wire        rx_commit, rx_abort, ir_dup;

    assign rx_lk_req = ir_lk_req | erl_pend;
    assign irq = |(rx_has & irq_en_w[NPT-1:0]) | mb_cpu_irq;

    assign rx_commit = ir_accept;
    assign rx_abort  = ir_filt | ir_ic | ir_rm;

    wire        rxp_ov, rxp_osop, rxp_oeop, rxp_ordy;
    wire [7:0]  rxp_od;
    wire [15:0] rxp_oport;
    wire [10:0] rxp_udplen;

    afdx_rx_parse u_rxparse (
        .clk(clk), .reset(reset),
        .in_valid(dp_rxin_valid), .in_ready(), .in_sop(dp_rxin_sop), .in_eop(dp_rxin_eop),
        .in_data(dp_rxin_data), .in_net(dp_rxin_net),
        .ok(rxp_ok), .vlid(rxp_vlid), .sip(rxp_sip), .dip(rxp_dip), .sport(rxp_sport), .dport(),
        .paylen(rxp_udplen), .sn(rxp_sn), .net(rxp_net), .framelen(),
        .err_eth(rxp_e_eth), .err_ip(rxp_e_ip), .err_udp(rxp_e_udp),
        .err_len(rxp_e_len), .err_fcs(rxp_e_fcs),
        .out_valid(rxp_ov), .out_ready(rxp_ordy), .out_sop(rxp_osop), .out_eop(rxp_oeop),
        .out_data(rxp_od), .out_port(rxp_oport),
        .desc_vld(rxp_desc), .vlid_vld(rxp_vlid_vld),
        .frag(rxp_frag), .mf(rxp_mf), .foff(rxp_foff), .ipid(rxp_ipid), .paylen_out(rxp_plen_out)
    );
    assign rxp_ordy = 1'b1;

    afdx_ic_rm #(.NVL(4096), .IDXW(12)) u_icrm (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .frame_end(rxp_desc), .f_ok(rxp_ok), .f_vlid(rxp_vlid), .f_sn(rxp_sn), .f_net(rxp_net),
        .lk_vlid_o(rx_lk_vlid), .lk_req_o(ir_lk_req), .lk_res_vlid_i(cfg_res_vlid),
        .lk_neten_i(cfg_neten_w),
        .lk_sn_vld_i(cfg_snvld_w),
        .lk_hit_i(cfg_hit_w), .lk_ic_en_i(cfg_ic_en_w), .lk_rm_en_i(cfg_rm_en_w),
        .lk_skew_i(cfg_skew_w),

        .lk_lmax_i(cfg_lmax_w), .f_dip(rxp_dip), .f_paylen(rxp_plen_out),
        .f_frag(rxp_frag),
        .accept(ir_accept), .ev_filt_drop(ir_filt), .ev_ic_drop(ir_ic), .ev_rm_drop(ir_rm),
        .rm_dup_o(ir_dup),
        .out_idx(ir_idx)
    );


    reg [3:0] erl_tmo;
    assign    erl_tmo_p = erl_pend && (erl_tmo == 4'd15);
    always @(posedge clk) begin
        if (reset) begin erl_pend <= 1'b0; port_lat <= 3'd0; erl_tmo <= 4'd0; end
        else begin
            if (rxp_vlid_vld) begin erl_pend <= 1'b1; erl_tmo <= 4'd0; end
            else if (erl_pend) begin
                if (cfg_hit_w && (cfg_res_vlid == rxp_vlid)) begin
                    erl_pend <= 1'b0;
                    port_lat <= cfg_w[82:80];       // w2[18:16] = buf_idx（2→3 位：端口 0..7）
                end
                else if (erl_tmo == 4'd15) begin
                    erl_pend <= 1'b0;               // 查不到/被抢拍 → 放手，别把 TX 饿死
                    erl_tmo  <= 4'd0;
                end
                else erl_tmo <= erl_tmo + 4'd1;
            end
        end
    end

    // ================= IP 分片重组（P7 要求；放在 IC/RM 之后、端口层之前）=================
    //   IC/RM 保持"逐帧"语义（每片有自己的 SN），本模块把它们并成**一条消息**、
    //   并且只发**一次 commit** 给端口层。
    afdx_reasm #(.AW(11)) u_reasm (
        .clk(clk), .reset(reset),
        .in_valid(rxp_ov), .in_sop(rxp_osop), .in_eop(rxp_oeop), .in_data(rxp_od),
        .in_vlid(rxp_vlid),
        .in_net(rxp_net),
        .in_dport(rxp_oport), .in_sip(rxp_sip), .in_sport(rxp_sport), .in_port(port_lat),
        .in_desc(rxp_desc), .in_frag(rxp_frag), .in_mf(rxp_mf), .in_foff(rxp_foff),
        .in_ipid(rxp_ipid), .in_ok(rxp_ok), .in_accept(ir_accept), .in_abort(rx_abort), .in_dup(ir_dup),
        .out_valid(rm_v), .out_sop(rm_sop), .out_eop(rm_eop), .out_data(rm_d),
        .out_vlid(rm_vlid), .out_dport(rm_dport), .out_sip(rm_sip), .out_sport(rm_sport),
        .out_port(rm_port), .out_net(rm_net),
        .out_commit(rm_commit), .out_abort(rm_abort), .out_vbytes(rm_vbytes),
        .ev_reasm_ok(ev_reasm_ok_w), .ev_reasm_err(ev_reasm_err_w)
    );


    afdx_flash_ctrl #(.DIV(40)) u_flash (
        .clk(clk), .reset(reset),
        .wr(fl_wr), .rd(fl_rd), .reg_sel(fl_rs), .wdata(DATAO), .rdata(fl_rdata),
        .sclk(flash_sclk), .mosi(flash_mosi), .miso(flash_miso), .cs_n(flash_cs_n)
    );


    wire [31:0] m_axi_awaddr, m_axi_araddr, m_axi_wdata, m_axi_rdata;
    wire        m_axi_awvalid, m_axi_awready, m_axi_wvalid, m_axi_wready;
    wire [3:0]  m_axi_wstrb;
    wire [1:0]  m_axi_bresp, m_axi_rresp;
    wire        m_axi_bvalid, m_axi_bready, m_axi_arvalid, m_axi_arready;
    wire        m_axi_rvalid, m_axi_rready;

    axi_lite_master_bridge u_axib (
        .clk(clk), .reset(reset),

        .DDREQ(DDREQ & port_win), .DADDR(DADDR), .DATAO(DATAO), .DBE(DBE), .DLEN(DLEN),
        .DWR(DWR), .DRD(DRD),
        .DDACK(axib_ack), .DATAI(axib_rdata),
        .m_axi_awaddr(m_axi_awaddr), .m_axi_awvalid(m_axi_awvalid), .m_axi_awready(m_axi_awready),
        .m_axi_wdata(m_axi_wdata), .m_axi_wstrb(m_axi_wstrb), .m_axi_wvalid(m_axi_wvalid),
        .m_axi_wready(m_axi_wready),
        .m_axi_bresp(m_axi_bresp), .m_axi_bvalid(m_axi_bvalid), .m_axi_bready(m_axi_bready),
        .m_axi_araddr(m_axi_araddr), .m_axi_arvalid(m_axi_arvalid), .m_axi_arready(m_axi_arready),
        .m_axi_rdata(m_axi_rdata), .m_axi_rresp(m_axi_rresp),
        .m_axi_rvalid(m_axi_rvalid), .m_axi_rready(m_axi_rready)
    );


    afdx_host_mbox #(.AW(11)) u_hmbox (
        .clk(clk), .reset(reset),
        .s_axi_awaddr(host_awaddr), .s_axi_awvalid(host_awvalid), .s_axi_awready(host_awready),
        .s_axi_wdata(host_wdata), .s_axi_wstrb(host_wstrb),
        .s_axi_wvalid(host_wvalid), .s_axi_wready(host_wready),
        .s_axi_bresp(host_bresp), .s_axi_bvalid(host_bvalid), .s_axi_bready(host_bready),
        .s_axi_araddr(host_araddr), .s_axi_arvalid(host_arvalid), .s_axi_arready(host_arready),
        .s_axi_rdata(host_rdata), .s_axi_rresp(host_rresp),
        .s_axi_rvalid(host_rvalid), .s_axi_rready(host_rready),
        .cpu_wr(pmbox_wr), .cpu_rd(mb_rd), .cpu_reg(mb_reg),
        .cpu_wdata(DATAO), .cpu_rdata(mb_rdata),
        .cpu_irq(mb_cpu_irq), .host_irq(host_irq)
    );

    afdx_axi_slave #(.NP(NPT_RX), .NPW(4), .NPTX(NPT_TX)) u_axislave (
        .clk(clk), .reset(reset),
        .s_axi_awaddr(m_axi_awaddr), .s_axi_awvalid(m_axi_awvalid), .s_axi_awready(m_axi_awready),
        .s_axi_wdata(m_axi_wdata), .s_axi_wstrb(m_axi_wstrb),
        .s_axi_wvalid(m_axi_wvalid), .s_axi_wready(m_axi_wready),
        .s_axi_bresp(m_axi_bresp), .s_axi_bvalid(m_axi_bvalid), .s_axi_bready(m_axi_bready),
        .s_axi_araddr(m_axi_araddr), .s_axi_arvalid(m_axi_arvalid), .s_axi_arready(m_axi_arready),
        .s_axi_rdata(m_axi_rdata), .s_axi_rresp(m_axi_rresp),
        .s_axi_rvalid(m_axi_rvalid), .s_axi_rready(m_axi_rready),
        // ---- 到端口层 ----
        .rx_pi(ppi_rx4), .rx_pi_en(axib_pi_en),
        .px_wr(px_wr), .px_rd(px_rd), .px_reg(rx_reg), .px_wdata(px_r_data), .px_rdata(px_rdata_w),
        .py_rd(py_rd), .py_wr(py_wr), .py_sel(rx_aux_sel), .py_wdata(py_w_data),
        .py_rdata(py_rdata_w),
        .tx_wr(tx_wr), .tx_pi(ppi_tx), .tx_reg(tx_reg_w), .tx_wdata(tx_w_data),
        .tx_rdata(tx_rdata_w),
        .rx_has(rx_has), .rx_ovf(rx_ovf), .irq()      // irq 故意不接（见上）
    );


    afdx_rx_ports #(.NP(6), .NPW(3)) u_rxports (
        .clk(clk), .reset(reset),
        .wr(px_wr), .rd(px_rd), .reg_sel(rx_reg), .wdata(px_r_data), .rdata(px_rdata_w),

        .pwr(py_wr), .prd(py_rd), .psel(rx_aux_sel), .pwdata(py_w_data), .prdata(py_rdata_w),
        .cpu_pi(ppi_rx), .cpu_sel_en(axib_pi_en),
        .in_valid(rm_v), .in_sop(rm_sop), .in_eop(rm_eop), .in_data(rm_d),
        .port_i(rm_port), .dport_i(rm_dport), .sip_i(rm_sip), .sport_i(rm_sport),
        .net_i(rm_net),
        .rx_commit(rm_commit), .rx_abort(rm_abort),
        .rx_any(), .rx_ovf_any(),
        .rx_has_o(rx_has), .rx_ovf_o(rx_ovf),
        .ovf_set_o(rx_ovf_set_w), .deliv_o(rx_deliv_p_w)
    );

    // ================= 发送通路 =================
    wire        txv, txsop, txeop, txready, tx_hold_w, tx_drop_w;
    wire [7:0]  txd;
    wire [15:0] tx_vlid, tx_dport;
    wire [31:0] tx_dip;
    wire [10:0] tx_len_w;
    wire [13:0] tx_len_x = {3'b000, tx_len_w};
    wire [NPT_TX-1:0] tx_busy_w;


    wire        txp_pend_w, txp_ready_w;
    wire [NPT_TX-1:0] tx_busy_w2;
    afdx_tx_ports #(.NP(NPT_TX), .NPW(4), .AW(14)) u_txports (
        .clk(clk), .reset(reset),
        .wr(tx_wr), .pi(ppi_tx), .reg_sel(tx_reg_w),
        .wdata(tx_w_data), .rdata(tx_rdata_w),
        .tx_valid(txv), .tx_ready(txp_ready_w), .tx_sop(txsop), .tx_eop(txeop), .tx_data(txd),
        .tx_vlid(tx_vlid), .tx_dip(tx_dip), .tx_dport(tx_dport), .tx_len(tx_len_w),
        .tx_hold(tx_hold_w), .tx_busy(tx_busy_w2),   // tx_replay 已删除（B 副本由 tx_build.pbuf 处理）
        .tx_pend(txp_pend_w),
        .tx_drop(tx_drop_w)
    );

    assign tx_busy_w = tx_busy_w2;


    wire        am_v, am_sop, am_eop, am_ready, am_busy, am_drop;
    wire [7:0]  am_data;
    wire [15:0] am_vlid;
    wire [13:0] am_len;
    wire        mux_v, mux_sop, mux_eop, mux_ready, owner_txp_w, owner_app_w;
    wire [7:0]  mux_data;
    wire [15:0] mux_vlid;
    wire [13:0] mux_len;
    wire        msg_drop_w;
    wire        tx_len_x_unused = 14'h0;

    afdx_app_bridge #(.NPW(4), .MAXLEN(1471), .APP_PORT_DEF(0), .VLID_MODE(VLID_MODE)) u_appbr (
        .clk(clk), .reset(reset),
        .app_rx_valid(app_rx_valid), .app_rx_data(app_rx_data), .app_port(app_port),
        .app_vlid(app_vlid),
        .app_tx_valid(app_tx_valid), .app_tx_data(app_tx_data), .app_tx_port(app_tx_port),
        .app_tx_vlid(app_tx_vlid),
        .err(app_err),
        .m_valid(am_v), .m_ready(am_ready), .m_sop(am_sop), .m_eop(am_eop),
        .m_data(am_data), .m_vlid(am_vlid), .m_len(am_len), .m_busy(am_busy),
        .m_drop(am_drop),
        // 旁听 reasm 输出（只读；端口号只有 3 位 → 高位补 0）
        .r_valid(rm_v), .r_sop(rm_sop), .r_eop(rm_eop), .r_data(rm_d),
        .r_port({1'b0, rm_port}), .r_commit(rm_commit), .r_abort(rm_abort),
        .r_vbytes(rm_vbytes), .r_vlid(rm_vlid)
    );

    afdx_msg_mux u_msgmux (
        .clk(clk), .reset(reset),
        .txp_valid(txv), .txp_sop(txsop), .txp_eop(txeop), .txp_data(txd),
        .txp_vlid(tx_vlid), .txp_len({3'b000, tx_len_w}), .txp_pend(txp_pend_w),
        .txp_ready(txp_ready_w),
        .app_valid(am_v), .app_sop(am_sop), .app_eop(am_eop), .app_data(am_data),
        .app_vlid(am_vlid), .app_len(am_len), .app_ready(am_ready),
        .pay_hold(tx_hold_w),
        .msg_valid(mux_v), .msg_sop(mux_sop), .msg_eop(mux_eop), .msg_data(mux_data),
        .msg_vlid(mux_vlid), .msg_len(mux_len), .msg_ready(mux_ready), .msg_drop(msg_drop_w),
        .owner_txp(owner_txp_w), .owner_app(owner_app_w)
    );

    // msg_drop 按"当前是谁在用"分流：端口层各自处理自己的丢弃
    assign tx_drop_w = msg_drop_w && owner_txp_w;
    assign am_drop   = msg_drop_w && owner_app_w;


    wire [31:0] ov_dip_w   = owner_txp_w ? tx_dip   : 32'h0;
    wire [15:0] ov_dport_w = owner_txp_w ? tx_dport : 16'h0;

    afdx_tx_path #(.NVL(4096), .NVLW(12), .SRC_IP(SRC_IP)) u_tx (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .ov_dip(ov_dip_w), .ov_dport(ov_dport_w),
        .msg_valid(mux_v), .msg_ready(mux_ready), .msg_sop(mux_sop), .msg_eop(mux_eop),
        .msg_data(mux_data), .msg_port(mux_vlid), .msg_len(mux_len),
        .tx_replay_o(),            // 已废弃（恒 0）：B 副本由 tx_build 的 pbuf 处理
        .pay_hold_o(tx_hold_w), .bag_hold_o(tx_bag_hold_w),
        .msg_drop(msg_drop_w),
        .cfg_vlid_o(tx_cfg_vlid), .cfg_hit_i(cfg_hit_w), .cfg_i(cfg_w),
        .cfg_res_vlid_i(cfg_res_vlid),
        .fr_valid(dp_fr_valid), .fr_ready(dp_fr_ready), .fr_sop(dp_fr_sop),
        .fr_eop(dp_fr_eop), .fr_data(dp_fr_data), .fr_net(dp_fr_net)
    );

    // ================= 统计计数器（SNMP 的数据源）=================
    afdx_stats #(.NVL(4096), .NVLW(12), .NP(NPT_RX), .NPW(3)) u_stats (
        .clk(clk), .reset(reset),
        .rx_desc(rxp_desc), .rx_ok(rxp_ok), .rx_vlid(rxp_vlid),
        // 同理：rxBytes 统计也要用分片感知的长度（用 udplen 会把分片帧的字节数算成垃圾）
        .rx_len(rxp_plen_out),
        .rx_e_fcs(rxp_e_fcs), .rx_e_len(rxp_e_len), .rx_e_ip(rxp_e_ip), .rx_e_udp(rxp_e_udp),
        .ev_ic(ir_ic), .ev_rm(ir_rm), .ev_filt(ir_filt), .ev_deliv(|rx_deliv_p_w),
        .ev_vlmiss(erl_tmo_p),
        .rx_ovf_set(rx_ovf_set_w), .rx_deliv_p(rx_deliv_p_w),
        .tx_frame(dp_fr_valid & dp_fr_sop),
        .tx_byte (dp_fr_valid & dp_fr_ready),
        .tx_bag_hold(tx_bag_hold_w),
        .tx_vlid(tx_vlid),
        .wr(stat_wr), .reg_sel(stat_rs), .wdata(DATAO), .rdata(stat_rdata)
    );

    initial $readmemh("iram.hex",  iram);
    initial $readmemh("dram0.hex", dram0);
    initial $readmemh("dram1.hex", dram1);
    initial $readmemh("dram2.hex", dram2);
    initial $readmemh("dram3.hex", dram3);

endmodule
