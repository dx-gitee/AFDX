// afdx_rx_ports.v —— AFDX 接收端口层（采样 / 队列 / SAP 三种端口）
`timescale 1ns/1ps

module afdx_rx_ports #(

    parameter NP  = 6,                     // 端口数（三种类型：1 采样 + 1 队列 + 4 SAP）
    parameter NPW = 3,                     // 端口号位宽 log2(NP)
    parameter PW  = 11
) (
    input  wire        clk,
    input  wire        reset,

    // ---- CPU 侧：消息窗口（AXI 从机会把它映射出去）----
    input  wire        wr,
    input  wire        rd,
    input  wire [2:0]  reg_sel,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    // ---- CPU 侧：端口配置 / 附加信息窗口 ----
    input  wire        pwr,
    input  wire        prd,
    input  wire [2:0]  psel,
    input  wire [NPW-1:0] cpu_pi,      // CPU 直接指定要访问的端口（组合，1 拍窗口用）
    input  wire           cpu_sel_en,  // 1 = 用 cpu_pi 指定端口
    input  wire [31:0] pwdata,
    output reg  [15:0] prdata,

    // ---- 接收数据面（解析器载荷字节流 + 本条帧的地址信息）----
    input  wire           in_valid,
    input  wire           in_sop,
    input  wire           in_eop,
    input  wire [7:0]     in_data,
    input  wire [NPW-1:0] port_i,          // 本帧要送进的接收端口（顶层提前查 VL 表得到）
    input  wire [15:0]    dport_i,         // 本帧 UDP 目的端口号
    input  wire [31:0]    sip_i,           // 本帧源 IP（SAP 口要交给分区）
    input  wire [15:0]    sport_i,         // 本帧源 UDP 端口（SAP 口要交给分区）
    input  wire           net_i,

    // ---- IC/RM 判决（整条消息收齐后脉冲）----
    input  wire        rx_commit,
    input  wire        rx_abort,

    output wire        rx_any,
    output wire        rx_ovf_any,
    output wire [NP-1:0] rx_has_o,       // 按端口的"有数据"位图（给 AXI 状态/中断用）
    output wire [NP-1:0] rx_ovf_o,       // 按端口的"溢出"位图
    // ---- 给统计计数器（afdx_stats）的脉冲口（单拍有效）----
    output reg  [NP-1:0] ovf_set_o,      // 某端口**新**溢出一次
    output reg  [NP-1:0] deliv_o         // 某端口**交付**了一条消息
);
    localparam SW   = 3;                   // 每端口 FIFO 槽位数（= 最大排队条数 8）
    localparam AW   = NPW + PW;
    localparam SZ   = 1 << (NPW + SW);

    localparam integer PORTSZ_W = (1 << PW);         // 每端口容量 = 2048 字 = 8 KB
    localparam integer QW       = PW + 1;            // 字计数器位宽（0..2048）
    localparam integer PWB      = PW + 1;            // 字数计数器位宽（0..2048）

    localparam T_DIS = 2'd0, T_SMP = 2'd1, T_QUE = 2'd2, T_SAP = 2'd3;

    // ================= 声明（全部排在 always 之前）=================
    reg [31:0]    dram [0:(1<<AW)-1];
    reg [AW-1:0]  waddr;
    reg [31:0]    wdata_w;
    reg           wren;
    reg [AW-1:0]  raddr;
    reg [31:0]    rd_q;


    reg [QW-1:0] queued [0:NP-1];
    reg [QW-1:0] cur_free;                      // sop 那拍量到的剩余空间（整条消息按它把关）
    reg          cur_drop;                      // 本条消息因空间不足被丢弃（不发布、只记溢出）

    reg          drop_cur;
    reg [2:0]    drop_pi;
    reg [7:0]    drop_cnt;
    reg [PW-1:0] pwptr [0:NP-1];
    reg [PW-1:0] prptr [0:NP-1];
    reg [SW:0]   pcnt  [0:NP-1];
    reg [SW-1:0] pqw   [0:NP-1];
    reg [SW-1:0] pqr   [0:NP-1];
    reg          povf  [0:NP-1];
    reg          pen   [0:NP-1];
    reg [1:0]    ptype [0:NP-1];
    reg [SW:0]   pmax  [0:NP-1];


    reg [13:0]       plen [0:SZ-1];
    reg [15:0]       pdp  [0:SZ-1];
    reg [31:0]       psip [0:SZ-1];
    reg [15:0]       pspt [0:SZ-1];

    reg              pnet [0:SZ-1];
    reg [NPW+SW-1:0] mfo_wa;
    reg [NPW+SW-1:0] mfo_ra;

    reg [NPW-1:0] cur_pi;
    reg [PW-1:0]  cur_wp, cur_ms;
    reg [1:0]     cur_bc;

    reg [7:0]     cur_b [0:3];     // 本字的 4 个字节（lane 0 在低字节）
    reg [1:0]     cur_m;           // 本字**有效字节数 − 1**（在 cur_wrp 那拍锁存）
    reg [13:0]    cur_nb;

    reg [QW-1:0]  cur_wn;
    reg [15:0]    cur_dp, cur_spt;
    reg [31:0]    cur_sip;
    reg           cur_net;
    reg           cur_wrp, cur_full;


    reg [NPW-1:0] pcfg_idx;
    integer       pi;

    reg  [NPW-1:0] sell_hold;
    reg            auto_en;
    reg  [PW-1:0]  proff [0:NP-1];      // **每端口各一份** CPU 读游标
    reg  [13:0]    plen_q;
    reg  [15:0]    pdp_q, pspt_q;
    reg  [31:0]    psip_q;
    reg            pnet_q;
    reg            smp_new;                // 采样口换了新消息 → 消费者读指针归零
    reg  [NPW-1:0] smp_pi;
    reg  [PW-1:0]  smp_off;

    // ---- 组合信号 ----


    reg  [NP-1:0]  has_vec;
    reg  [NPW-1:0] apick;
    integer        hi;
    always @(*) begin
        has_vec = {NP{1'b0}};
        apick   = {NPW{1'b0}};
        for (hi = NP - 1; hi >= 0; hi = hi - 1)
            if (pen[hi] && (pcnt[hi] != 0)) begin
                has_vec[hi] = 1'b1; apick = hi[NPW-1:0];   // 越低越优先
            end
    end

    wire           cpu_pi_ok = (cpu_pi < NP);
    wire [NPW-1:0] sell = cpu_sel_en ? (cpu_pi_ok ? cpu_pi : {NPW{1'b0}})
                                     : (auto_en ? apick : sell_hold);
    wire           sel_has = pen[sell] && (pcnt[sell] != 0);


    wire [NPW-1:0] port_i_ok = (port_i < NP) ? port_i : {NPW{1'b0}};

    wire         rx_rd     = rd && (reg_sel == 3'd4);

    wire         release_w = wr && (reg_sel == 3'd6) && wdata[0];

    wire [PW-1:0] rptr_s    = prptr[sell];
    wire [13:0]   clen      = plen_q;
    wire [PWB-1:0] nwords    = clen[13:2] + ((clen[1:0] != 2'd0) ? 1'b1 : 1'b0);

    wire [PW-1:0]  next_base = rptr_s + nwords;


    wire [13:0]   clen_now   = plen[{sell, pqr[sell]}];
    wire [PWB-1:0] nwords_now = clen_now[13:2] + ((clen_now[1:0] != 2'd0) ? 1'b1 : 1'b0);
    wire [PW-1:0]  next_base_now = rptr_s + nwords_now;


    wire rel_any = release_w && (pcnt[sell] != 0);


    wire        wr_now = cur_wrp && !cur_drop && (cur_wn < cur_free);

    wire        cur_drop_now = cur_drop || (cur_wrp && !wr_now);


    wire        drop_add = in_valid && in_sop && (cur_full || (drop_cnt != 8'd0));
    wire        drop_sub = (!cur_full) && (drop_cnt != 8'd0) && (rx_commit || rx_abort);


    wire pub_smp = cur_full && rx_commit && !cur_drop_now && pen[cur_pi] && (ptype[cur_pi] == T_SMP);
    wire pub_que = cur_full && rx_commit && !cur_drop_now && pen[cur_pi]
                   && (ptype[cur_pi] != T_DIS) && (ptype[cur_pi] != T_SMP)
                   && (pcnt[cur_pi] < pmax[cur_pi]);

    // 本拍是否正在给 `cur_pi` **置**溢出位（与 block B 里那两处置位条件同源）。
    // 用途：同拍"新溢出 + 该端口释放"时，**释放不能把这次溢出清掉** ——
    //   否则 `STATUS.bit3` / `rx_ovf_o` 看不到它，而统计脉冲 `ovf_set_o` 照样发出去
    //   → 软件看到的标志位与计数器口径不一致（分不清"新溢出"和"上次溢出"）。
    wire        ovf_now = cur_full && (rx_commit || rx_abort) &&
                          ( cur_drop_now
                            || (rx_commit && pen[cur_pi] && (ptype[cur_pi] != T_DIS)
                                && (ptype[cur_pi] != T_SMP) && (pcnt[cur_pi] >= pmax[cur_pi])) );

    wire [4:0]  pmax_in = pwdata[7:3];
    wire [3:0]  pmax_v  = (pmax_in > 5'd8) ? 4'd8 : pmax_in[3:0];


    reg  [NP-1:0] povf_vec;
    integer       qi;
    always @(*) begin
        povf_vec = {NP{1'b0}};
        for (qi = 0; qi < NP; qi = qi + 1) povf_vec[qi] = povf[qi];
    end
    wire povf_r = |povf_vec;

    assign rx_any     = (has_vec != {NP{1'b0}});
    assign rx_ovf_any = povf_r;
    assign rx_has_o   = has_vec;
    assign rx_ovf_o   = povf_vec;

    // ================= block A：数据 RAM =================
    always @(posedge clk) begin
        if (wren) dram[waddr] <= wdata_w;
        rd_q <= dram[raddr];
    end

    // ================= block B：接收状态机 + 每端口元数据（唯一驱动者）=================
    always @(posedge clk) begin
        if (reset) begin
            wren <= 1'b0; cur_wrp <= 1'b0; smp_new <= 1'b0;
            ovf_set_o <= {NP{1'b0}}; deliv_o <= {NP{1'b0}};
            cur_pi <= 0; cur_wp <= 0; cur_ms <= 0; cur_bc <= 0; cur_m <= 2'd3;
            cur_b[0] <= 8'h00; cur_b[1] <= 8'h00; cur_b[2] <= 8'h00; cur_b[3] <= 8'h00;
            cur_nb <= 0; cur_full <= 0; cur_dp <= 0; cur_sip <= 0; cur_spt <= 0;
            cur_net <= 1'b0;

            cur_drop <= 1'b0; cur_free <= PORTSZ_W[QW-1:0]; cur_wn <= {QW{1'b0}};
            drop_cur <= 1'b0; drop_pi <= 3'd0; drop_cnt <= 8'd0;
            pcfg_idx <= {NPW{1'b0}}; smp_new <= 1'b0; smp_pi <= {NPW{1'b0}}; smp_off <= {PW{1'b0}};
            mfo_wa <= {(NPW+SW){1'b0}};
            for (pi = 0; pi < NP; pi = pi + 1) begin
                pwptr[pi] <= {PW{1'b0}}; prptr[pi] <= {PW{1'b0}};
                queued[pi] <= {QW{1'b0}};
                pcnt [pi] <= {SW+1{1'b0}};
                pqw  [pi] <= {SW{1'b0}}; pqr  [pi] <= {SW{1'b0}};
                povf [pi] <= 1'b0;
                pen  [pi] <= 1'b1;
                ptype[pi] <= (pi == 0) ? T_SMP : (pi == 1) ? T_QUE : T_SAP;
                pmax [pi] <= 4'd8;
            end

            for (pi = 0; pi < SZ; pi = pi + 1) pnet[pi] <= 1'b0;   // 用本块的 pi（qi 属另一个 always 块，别跨块共用循环变量）
        end
        else begin
            wren     <= 1'b0;
            cur_wrp  <= 1'b0;
            ovf_set_o<= {NP{1'b0}};    // 统计脉冲：每拍清
            deliv_o  <= {NP{1'b0}};
            smp_new  <= 1'b0;
            mfo_wa   <= {cur_pi, pqw[cur_pi]};


            if (in_valid) begin
                if (in_sop) begin
                    if (!cur_full && (drop_cnt == 8'd0)) begin
                        drop_cur <= 1'b0;
                        cur_pi  <= port_i_ok;
                        cur_ms  <= pwptr[port_i_ok];
                        cur_wp  <= pwptr[port_i_ok];
                        cur_dp  <= dport_i;
                        cur_sip <= sip_i;
                        cur_spt <= sport_i;
                        cur_net <= net_i;
                        cur_b[0] <= in_data;
                        cur_nb   <= 14'd1;
                        cur_wn   <= {QW{1'b0}};                 // 本条还没有写出任何字

                        cur_free <= (ptype[port_i_ok] == T_SMP) ? PORTSZ_W[QW-1:0]
                                                             : (PORTSZ_W[QW-1:0] - queued[port_i_ok]);
                        cur_drop <= (ptype[port_i_ok] != T_SMP) && (queued[port_i_ok] >= PORTSZ_W[QW-1:0]);
                        if (in_eop) begin
                            cur_wrp  <= 1'b1; cur_m <= 2'd0; cur_bc <= 2'd0; cur_full <= 1'b1;
                        end
                        else cur_bc <= 2'd1;
                    end
                    else begin

                        drop_cur <= 1'b1;
                        drop_pi  <= port_i_ok;
                    end
                end
                else if (!drop_cur) begin
                    cur_b[cur_bc] <= in_data;
                    cur_nb <= cur_nb + 14'd1;
                    cur_bc <= cur_bc + 2'd1;
                    if (cur_bc == 2'd3 || in_eop) begin
                        cur_wrp <= 1'b1;
                        cur_m   <= cur_bc;      // 有效字节数−1（供下一拍组字用）
                        cur_bc  <= 2'd0;
                        if (in_eop) cur_full <= 1'b1;
                    end
                end
                // drop_cur=1：本帧的字节全部丢弃（不写 RAM、不改 cur_nb）
            end

            if (cur_wrp) begin
                if (wr_now) begin
                    waddr   <= {cur_pi, cur_wp};
                    // 只把**本字有效的 lane** 组出去，其余补 0 —— 半满的字不再带上一字的尾巴
                    wdata_w <= (cur_m == 2'd3) ? {cur_b[3], cur_b[2], cur_b[1], cur_b[0]} :
                               (cur_m == 2'd2) ? {8'h00,    cur_b[2], cur_b[1], cur_b[0]} :
                               (cur_m == 2'd1) ? {16'h0000,            cur_b[1], cur_b[0]} :
                                                 {24'h000000,                    cur_b[0]};
                    wren    <= 1'b1;
                    cur_wp  <= cur_wp + 1'b1;
                    cur_wn  <= cur_wn + 1'b1;
                end
                else cur_drop <= 1'b1;      // 这个字放不下 → 本条进不了队列（记录成可见溢出）
            end

            // ---------- IC/RM 判决 ----------
            if (cur_full && (rx_commit || rx_abort)) begin
                cur_full <= 1'b0;
                if (cur_drop_now) begin

                    povf[cur_pi]      <= 1'b1;
                    ovf_set_o[cur_pi] <= 1'b1;
                end
                else if (rx_commit && pen[cur_pi] && (ptype[cur_pi] != T_DIS)) begin
                    if (ptype[cur_pi] == T_SMP) begin
                        deliv_o[cur_pi] <= 1'b1;           // 统计：交付一条
                        // 采样口：永远只保留最新一条
                        pqw[cur_pi]  <= 3'd1; pqr[cur_pi] <= 3'd0;
                        plen[{cur_pi, 3'd0}] <= cur_nb;
                        pdp [{cur_pi, 3'd0}] <= cur_dp;
                        psip[{cur_pi, 3'd0}] <= cur_sip;
                        pspt[{cur_pi, 3'd0}] <= cur_spt;
                        pnet[{cur_pi, 3'd0}] <= cur_net;
                        pwptr[cur_pi]<= cur_wp + {10'h0, wr_now};

                        prptr[cur_pi]<= cur_ms;
                        smp_new      <= 1'b1;
                        smp_pi       <= cur_pi;
                        smp_off      <= cur_ms;
                        povf[cur_pi] <= 1'b0;
                    end
                    else if (pcnt[cur_pi] < pmax[cur_pi]) begin
                        deliv_o[cur_pi] <= 1'b1;           // 统计：交付一条
                        // 队列口 / SAP 口：追加
                        plen[mfo_wa] <= cur_nb;
                        pdp [mfo_wa] <= cur_dp;
                        psip[mfo_wa] <= cur_sip;
                        pspt[mfo_wa] <= cur_spt;
                        pnet[mfo_wa] <= cur_net;
                        pqw[cur_pi]  <= pqw[cur_pi] + 1'b1;
                        pwptr[cur_pi]<= cur_wp + {10'h0, wr_now};
                    end
                    else begin
                        povf[cur_pi]    <= 1'b1;        // 队列满：丢新消息
                        ovf_set_o[cur_pi] <= 1'b1;      // 统计：本端口新溢出一次
                    end
                end
                // rx_abort / 端口禁用：pwptr 没动 → 无需回滚
            end
            else if (!cur_full && (drop_cnt != 8'd0) && (rx_commit || rx_abort)) begin

                if (rx_commit) begin
                    povf[drop_pi]      <= 1'b1;
                    ovf_set_o[drop_pi] <= 1'b1;
                end
            end

            // ---------- CPU 释放当前消息（指针部分）----------
            if (release_w) begin

                if (rel_any && !(pub_smp && (cur_pi == sell))) begin
                    prptr[sell] <= next_base_now;
                    // 采样口只有固定槽 0，pqr 不能跟着 +1（否则元数据读到没写过的槽 → 长度读成 0）
                    if (ptype[sell] == T_SMP) pqr[sell] <= {SW{1'b0}};
                    else                      pqr[sell] <= pqr[sell] + 1'b1;
                end

                if (!(ovf_now && (cur_pi == sell))) povf[sell] <= 1'b0;
            end


            if (pub_smp) begin
                pcnt  [cur_pi] <= {{SW{1'b0}}, 1'b1};        // 采样口发布后恒为 1 条
                queued[cur_pi] <= cur_wn + wr_now;
            end
            else if (pub_que) begin
                pcnt  [cur_pi] <= pcnt[cur_pi] + 1'b1;
                queued[cur_pi] <= queued[cur_pi] + cur_wn + wr_now;
            end

            if (rel_any) begin
                if (pub_smp && (cur_pi == sell)) begin
                    // 采样口：发布 = 替换，"释放"对账目无净影响
                    pcnt  [sell] <= {{SW{1'b0}}, 1'b1};
                    queued[sell] <= cur_wn + wr_now;
                end
                else if (pub_que && (cur_pi == sell)) begin

                    pcnt  [sell] <= pcnt[sell];
                    queued[sell] <= (queued[sell] >= nwords_now)
                                    ? ((queued[sell] - nwords_now) + cur_wn + wr_now)
                                    : (cur_wn + wr_now);
                end
                else begin
                    pcnt  [sell] <= (pcnt[sell] == 0) ? {SW+1{1'b0}} : (pcnt[sell] - 1'b1);
                    queued[sell] <= (queued[sell] >= nwords_now) ? (queued[sell] - nwords_now)
                                                                 : {QW{1'b0}};
                end
            end


            if (pwr && psel == 3'd0)
                pcfg_idx <= (pwdata[NPW-1:0] < NP) ? pwdata[NPW-1:0] : {NPW{1'b0}};
            if (pwr && psel == 3'd1) begin
                pen  [pcfg_idx] <= pwdata[0];
                ptype[pcfg_idx] <= pwdata[2:1];
                pmax [pcfg_idx] <= pmax_v;
            end
            if (prd && psel == 3'd2) povf[sell] <= 1'b0;   // 读溢出位图即清

            // ---------- 被丢弃帧计数：同拍可加可减，必须合并成一次赋值 ----------
            //   数据面"新增一条被丢帧"与判决面"吸收一条被丢帧"可能同拍；
            //   同一 always 块里两次非阻塞赋值后者胜出 → 必须显式合并
            //   （本工程铁律："清"必须在所有"设"之前，或合并成一次赋值）。
            if (drop_add && !drop_sub) begin
                if (drop_cnt != 8'hFF) drop_cnt <= drop_cnt + 8'd1;
            end
            else if (!drop_add && drop_sub) begin
                drop_cnt <= drop_cnt - 8'd1;
            end
            // drop_add && drop_sub：一进一出，保持
        end
    end

    // ================= block C：CPU 读指针 / 端口选择寄存器 =================
    always @(posedge clk) begin
        if (reset) begin
            sell_hold <= {NPW{1'b0}}; auto_en <= 1'b1;
            for (pi = 0; pi < NP; pi = pi + 1) proff[pi] <= {PW{1'b0}};
            raddr <= {AW{1'b0}};
            plen_q <= 11'h0; pdp_q <= 16'h0; psip_q <= 32'h0; pspt_q <= 16'h0;
        end
        else begin
            // reg7 = RX_PORTSEL：bit3=1 自动挑端口，否则锁定到 bit[NPW-1:0]
            if (wr && reg_sel == 3'd7) begin
                auto_en   <= wdata[3];

                sell_hold <= (wdata[NPW-1:0] < NP) ? wdata[NPW-1:0] : {NPW{1'b0}};
            end
            // 采样口换了新消息 → 该端口的读游标归零（消费者重新从消息头开始）
            if (smp_new) proff[smp_pi] <= {PW{1'b0}};


            raddr <= {sell, (smp_new && (smp_pi == sell)) ? rptr_s
                          : (rel_any ? next_base_now
                                     : (rptr_s + proff[sell] + (rx_rd ? 1'b1 : 1'b0)))};

            if (release_w)      proff[sell] <= {PW{1'b0}};              // 释放后下一条从头上读
            else if (rx_rd)     proff[sell] <= proff[sell] + 1'b1;

            if (smp_new && (smp_pi == sell)) proff[sell] <= {PW{1'b0}};

            mfo_ra <= {sell, pqr[sell]};
            plen_q <= plen[mfo_ra];
            pdp_q  <= pdp [mfo_ra];
            psip_q <= psip[mfo_ra];
            pspt_q <= pspt[mfo_ra];
            pnet_q <= pnet[mfo_ra];
        end
    end

    // ================= block D/E：CPU 读选通 =================
    always @(*) begin
        case (reg_sel)

            3'd3: rdata = {28'h0, povf[sell], sel_has, 2'b00};
            3'd4: rdata = rd_q;                       // RX_DATA
            3'd5: rdata = {16'h0, pdp_q};             // RX_PORT（UDP 目的端口号）
            3'd7: rdata = {28'h0, auto_en, sell};     // RX_PORTSEL 回读
            default: rdata = 32'h0;
        endcase
    end

    always @(*) begin
        case (psel)

            3'd1: prdata = {9'h0, pmax[sell], ptype[sell], pen[sell]};

            3'd2: prdata = {{(16-2*NP){1'b0}}, povf_vec, has_vec};

            3'd3: prdata = sel_has ? {2'h0, clen}   : 16'h0;   // 本条消息长度
            3'd4: prdata = sel_has ? pspt_q         : 16'h0;   // 源 UDP 端口（SAP 用）
            3'd5: prdata = sel_has ? psip_q[15:0]   : 16'h0;   // 源 IP 低 16 位（SAP 用）
            3'd6: prdata = sel_has ? psip_q[31:16]  : 16'h0;   // 源 IP 高 16 位（SAP 用）
            3'd7: prdata = sel_has ? {15'h0, pnet_q} : 16'h0;
            default: prdata = 16'h0;
        endcase
    end

endmodule
