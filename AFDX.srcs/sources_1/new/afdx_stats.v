// afdx_stats.v —— AFDX 端系统统计计数器（SNMP 的唯一数据源）
`timescale 1ns/1ps

module afdx_stats #(
    parameter NVL  = 4096,
    parameter NVLW = 12,

    parameter NP   = 6,
    parameter NPW  = 3
) (
    input  wire         clk,
    input  wire         reset,

    // ---------------- 事件输入（单拍有效）----------------
    input  wire         rx_desc,        // 帧末描述符有效（每帧 1 拍）
    input  wire         rx_ok,
    input  wire [15:0]  rx_vlid,
    input  wire [10:0]  rx_len,
    input  wire         rx_e_fcs, rx_e_len, rx_e_ip, rx_e_udp,
    input  wire         ev_ic, ev_rm, ev_filt, ev_deliv, ev_vlmiss,
    input  wire [NP-1:0]  rx_ovf_set,
    input  wire [NP-1:0]  rx_deliv_p,
    input  wire         tx_frame,
    input  wire         tx_byte,
    input  wire         tx_bag_hold,
    input  wire [15:0]  tx_vlid,

    // ---------------- CPU 读口 ----------------
    input  wire         wr,
    input  wire [3:0]   reg_sel,
    input  wire [31:0]  wdata,
    output reg  [31:0]  rdata
);

    // ===================== 参数/常量 =====================
    localparam [3:0] R_G_IDX=4'd0, R_G_DATA=4'd1, R_P_IDX=4'd2, R_P_DATA=4'd3,
                     R_V_IDX=4'd4, R_V_LANE=4'd5, R_V_DATA=4'd6, R_BUSY=4'd7,
                     R_LOST =4'd8, R_CLR  =4'd9;
    localparam NG = 16;                 // 全局计数器个数
    localparam PS = NP * 2;             // 每端口计数器个数
    localparam NS = 4;                  // 事件来源个数
    localparam [1:0] S_IDLE=2'd0, S_RD=2'd1, S_WR=2'd2, S_DONE=2'd3;

    // ===================== 信号声明 =====================
    wire        clr;
    wire        ev_icrm;
    wire [2:0]  lane_icrm;
    wire        cpu_rd_req;
    wire        busy;

    reg  [31:0] g  [0:NG-1];
    reg  [31:0] pc [0:PS-1];
    reg  [159:0] pv [0:NVL-1];

    reg  [3:0]      g_idx, p_idx;
    reg  [NVLW-1:0] cpu_idx;
    reg  [2:0]      cpu_lane;

    reg             s_vld [0:NS-1];

    wire rxvl_ok = ~(|rx_vlid[15:NVLW]);
    wire txvl_ok = ~(|tx_vlid[15:NVLW]);
    reg  [NVLW-1:0] s_vlid[0:NS-1];
    reg  [2:0]      s_lane[0:NS-1];
    reg             s_wb  [0:NS-1];
    reg             ev_lost;

    reg  [1:0]      st;
    reg  [1:0]      cur_s;
    reg  [NVLW-1:0] eng_vlid;
    reg  [2:0]      eng_lane;
    reg             eng_wb;
    reg  [NVLW-1:0] pv_raddr, pv_waddr;
    reg  [159:0]    pv_rd, pv_nxt;
    reg             pv_we;
    reg  [31:0]     v_data;

    integer gi, pi, si;

    // ===================== 函数（须在使用前）=====================
    function [31:0] sel_lane;
        input [159:0] w;
        input [2:0]   l;
        begin
            sel_lane = (l == 3'd0) ? w[ 31:  0] :
                       (l == 3'd1) ? w[ 63: 32] :
                       (l == 3'd2) ? w[ 95: 64] :
                       (l == 3'd3) ? w[127: 96] : w[159:128];
        end
    endfunction

    // ===================== 组合 =====================
    assign clr        = wr && (reg_sel == R_CLR) && wdata[0];
    assign ev_icrm    = ev_ic | ev_rm | ev_deliv;
    assign lane_icrm  = ev_ic ? 3'd1 : (ev_rm ? 3'd2 : 3'd3);
    assign cpu_rd_req = wr && (reg_sel == R_V_LANE);   // 写 VL_LANE = 触发一次读
    assign busy       = s_vld[0] | s_vld[1] | s_vld[2] | s_vld[3] | (st != S_IDLE);


    wire s_cleared    = (st == S_DONE);                       // 本拍正在清 cur_s 槽
    wire free0 = (!s_vld[0]) || (s_cleared && (cur_s == 2'd0));
    wire free1 = (!s_vld[1]) || (s_cleared && (cur_s == 2'd1));
    wire free2 = (!s_vld[2]) || (s_cleared && (cur_s == 2'd2));
    wire free3 = (!s_vld[3]) || (s_cleared && (cur_s == 2'd3));

    // ===================== ① 全局标量 =====================
    always @(posedge clk) begin
        if (reset || clr) begin
            for (gi = 0; gi < NG; gi = gi + 1) g[gi] <= 32'd0;
        end
        else begin
            if (rx_desc) begin
                g[0] <= g[0] + 32'd1;                       // 0  rxFrames
                if (rx_e_fcs) g[2] <= g[2] + 32'd1;         // 2  rxErrFcs
                if (rx_e_len) g[3] <= g[3] + 32'd1;         // 3  rxErrLen
                if (rx_e_ip ) g[4] <= g[4] + 32'd1;         // 4  rxErrIp
                if (rx_e_udp) g[5] <= g[5] + 32'd1;         // 5  rxErrUdp
            end
            if (rx_desc && rx_ok) g[1] <= g[1] + {21'd0, rx_len};   // 1 rxBytes
            if (ev_vlmiss)   g[6]  <= g[6]  + 32'd1;        // 6  rxVlMiss
            if (ev_ic)       g[7]  <= g[7]  + 32'd1;        // 7  rxIcDrop
            if (ev_rm)       g[8]  <= g[8]  + 32'd1;        // 8  rxRmDrop
            if (ev_filt)     g[9]  <= g[9]  + 32'd1;        // 9  rxFiltDrop
            if (ev_deliv)    g[10] <= g[10] + 32'd1;        // 10 rxDelivered
            if (tx_frame)    g[11] <= g[11] + 32'd1;        // 11 txFrames
            if (tx_byte)     g[12] <= g[12] + 32'd1;        // 12 txBytes
            if (tx_bag_hold) g[13] <= g[13] + 32'd1;        // 13 txBagHold
            // 14 txPortDrop：本设计"发送口忙时挡住 CPU 写"而不是丢弃 → 恒 0（报告如实写）
            if (|rx_ovf_set) g[15] <= g[15] + 32'd1;        // 15 rxPortOvf
        end
    end

    // ===================== ② 每端口 =====================
    always @(posedge clk) begin
        if (reset || clr) begin
            for (pi = 0; pi < PS; pi = pi + 1) pc[pi] <= 32'd0;
        end
        else begin
            for (pi = 0; pi < NP; pi = pi + 1) begin
                if (rx_ovf_set[pi]) pc[pi*2]     <= pc[pi*2]     + 32'd1;
                if (rx_deliv_p[pi]) pc[pi*2 + 1] <= pc[pi*2 + 1] + 32'd1;
            end
        end
    end

    // ===================== ③ 每 VL：块 RAM =====================
    // 表项 160 bit = 5 × 32：0=rxFrames 1=icDrop 2=rmDrop 3=delivered 4=txFrames
    initial $readmemh("statzero.hex", pv);   // 必须 $readmemh（initial for 会让块 RAM 退化成触发器）

    // ---- CPU 侧锁存 ----
    always @(posedge clk) begin
        if (reset) begin
            g_idx <= 4'd0; p_idx <= 4'd0; cpu_idx <= {NVLW{1'b0}}; cpu_lane <= 3'd0;
        end
        else if (wr) begin
            if (reg_sel == R_G_IDX && wdata < NG) g_idx   <= wdata[3:0];
            if (reg_sel == R_P_IDX && wdata < PS) p_idx   <= wdata[3:0];
            if (reg_sel == R_V_IDX)               cpu_idx <= wdata[NVLW-1:0];
            if (reg_sel == R_V_LANE)              cpu_lane<= wdata[2:0];
        end
    end

    // ---- 来源槽：各自独立，谁有事谁来占 ----
    always @(posedge clk) begin
        if (reset) begin
            for (si = 0; si < NS; si = si + 1) s_vld[si] <= 1'b0;
            ev_lost <= 1'b0;
        end
        else begin

            if (st == S_DONE) s_vld[cur_s] <= 1'b0;


            if (clr) ev_lost <= 1'b0;


            if (rx_desc) begin
                if (!rxvl_ok) ;                                     // 越界 VLID：不做按 VL 计数
                else if (free0) begin s_vld[0] <= 1'b1; s_vlid[0] <= rx_vlid; s_lane[0] <= 3'd0; s_wb[0] <= 1'b1; end
                else ev_lost <= 1'b1;
            end
            if (ev_icrm) begin
                if (!rxvl_ok) ;
                else if (free1) begin s_vld[1] <= 1'b1; s_vlid[1] <= rx_vlid; s_lane[1] <= lane_icrm; s_wb[1] <= 1'b1; end
                else ev_lost <= 1'b1;
            end
            if (tx_frame) begin
                if (!txvl_ok) ;
                else if (free2) begin s_vld[2] <= 1'b1; s_vlid[2] <= tx_vlid; s_lane[2] <= 3'd4; s_wb[2] <= 1'b1; end
                else ev_lost <= 1'b1;
            end
            if (cpu_rd_req) begin

                if (free3) begin s_vld[3] <= 1'b1; s_vlid[3] <= cpu_idx; s_lane[3] <= wdata[2:0]; s_wb[3] <= 1'b0; end
                else ev_lost <= 1'b1;
            end
        end
    end

    // ---- 引擎 FSM（固定优先级：收帧 > IC/RM > 发帧 > CPU 读）----
    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; pv_we <= 1'b0;
            pv_raddr <= {NVLW{1'b0}}; pv_waddr <= {NVLW{1'b0}};
            cur_s <= 2'd0; eng_vlid <= {NVLW{1'b0}}; eng_lane <= 3'd0; eng_wb <= 1'b0; v_data <= 32'd0;
        end
        else begin
            pv_we <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (s_vld[0])      begin cur_s <= 2'd0; eng_vlid <= s_vlid[0]; eng_lane <= s_lane[0]; eng_wb <= s_wb[0]; st <= S_RD; end
                    else if (s_vld[1]) begin cur_s <= 2'd1; eng_vlid <= s_vlid[1]; eng_lane <= s_lane[1]; eng_wb <= s_wb[1]; st <= S_RD; end
                    else if (s_vld[2]) begin cur_s <= 2'd2; eng_vlid <= s_vlid[2]; eng_lane <= s_lane[2]; eng_wb <= s_wb[2]; st <= S_RD; end
                    else if (s_vld[3]) begin cur_s <= 2'd3; eng_vlid <= s_vlid[3]; eng_lane <= s_lane[3]; eng_wb <= s_wb[3]; st <= S_RD; end
                end
                S_RD: begin
                    pv_raddr <= eng_vlid;        // 寄存读：下一拍 pv_rd 有效
                    st       <= S_WR;
                end
                S_WR: st <= S_DONE;
                default: begin                   // S_DONE

                    if (eng_wb) begin pv_we <= 1'b1; pv_waddr <= eng_vlid; end
                    else              v_data <= sel_lane(pv_rd, eng_lane);   // CPU 读只取结果
                    st <= S_IDLE;
                end
            endcase
        end
    end

    // 在选中 lane 上加 1
    always @(*) begin
        pv_nxt = pv_rd;
        case (eng_lane)
            3'd0: pv_nxt[ 31:  0] = pv_rd[ 31:  0] + 32'd1;
            3'd1: pv_nxt[ 63: 32] = pv_rd[ 63: 32] + 32'd1;
            3'd2: pv_nxt[ 95: 64] = pv_rd[ 95: 64] + 32'd1;
            3'd3: pv_nxt[127: 96] = pv_rd[127: 96] + 32'd1;
            default: pv_nxt[159:128] = pv_rd[159:128] + 32'd1;
        endcase
    end


    always @(posedge clk) begin
        pv_rd <= pv[pv_raddr];
        if (pv_we) pv[pv_waddr] <= pv_nxt;
    end

    // ===================== CPU 读口 =====================
    always @(*) begin
        case (reg_sel)
            R_G_IDX : rdata = {28'h0, g_idx};        // 回读当前选中的编号（便于自检）
            R_P_IDX : rdata = {28'h0, p_idx};
            R_G_DATA: rdata = g[g_idx];
            R_P_DATA: rdata = pc[p_idx];
            R_V_DATA: rdata = v_data;
            R_BUSY  : rdata = {31'h0, busy};
            R_LOST  : rdata = {31'h0, ev_lost};
            default : rdata = 32'h0;
        endcase
    end

endmodule
