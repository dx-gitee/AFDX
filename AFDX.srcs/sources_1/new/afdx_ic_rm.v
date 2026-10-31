// afdx_ic_rm.v —— AFDX 完整性检查(IC) + 冗余管理(RM)
`timescale 1ns/1ps

module afdx_ic_rm #(
    parameter NVL  = 512,
    parameter IDXW = 9
)(
    input  wire        clk,
    input  wire        reset,

    input  wire [15:0] ts_now,               // 8 µs 时基计数（TICK_DIV=1000 @125 MHz）

    // ---- 帧末描述符（载荷结束后 1 拍有效）----
    input  wire        frame_end,
    input  wire        f_ok,                 // 解析器 ok
    input  wire [15:0] f_vlid,
    input  wire [7:0]  f_sn,
    input  wire        f_net,                // 0=A, 1=B
    input  wire [31:0] f_dip,
    input  wire [10:0] f_paylen,

    input  wire        f_frag,

    // ---- VL 配置查表口（与 TX 共享）----
    output wire [15:0] lk_vlid_o,
    output wire        lk_req_o,
    input  wire [15:0] lk_res_vlid_i,        // 当前查表结果对应的地址（自校验）
    input  wire        lk_hit_i,
    input  wire        lk_ic_en_i,
    input  wire        lk_rm_en_i,
    input  wire [11:0] lk_skew_i,   // SkewMax（单位 = 1 个 8 µs 时基刻度，12 位 → 0–32.7 ms）
    input  wire [10:0] lk_lmax_i,
    input  wire [1:0]  lk_neten_i,
    input  wire        lk_sn_vld_i,

    // ---- 判决（1 拍脉冲）----
    output reg         accept,               // 交付（First Win / 非冗余）
    output reg         ev_filt_drop,         // VL 未配置 / 帧无效
    output reg         ev_ic_drop,           // IC 丢弃
    output reg         ev_rm_drop,           // RM 丢弃（重复副本/乱序）

    output reg         rm_dup_o,
    output reg  [IDXW-1:0] out_idx           // 命中表项索引（供上层取 buf_idx）
);
    localparam S_IDLE = 2'd0, S_RD = 2'd1, S_EVAL = 2'd2;
    reg [1:0] st;

    // 每 VL 状态（RAM）
    reg [7:0]  psn_a [0:NVL-1];
    reg [7:0]  psn_b [0:NVL-1];
    reg [7:0]  rm_psn[0:NVL-1];
    reg        rm_vld[0:NVL-1];

    reg [1:0]  ic_seen[0:NVL-1];
    reg [15:0] rm_ts [0:NVL-1];


    initial $readmemh("iczero.hex",   psn_a);
    initial $readmemh("iczero.hex",   psn_b);
    initial $readmemh("iczero.hex",   rm_psn);
    initial $readmemh("ic0bit.hex",   rm_vld);
    initial $readmemh("ic0bit.hex",   ic_seen);
    initial $readmemh("iczero16.hex", rm_ts);

    // 锁存的描述符
    reg [IDXW-1:0] idx_q;
    reg [15:0]     vlid_q;
    reg [7:0]      sn_q;
    reg            net_q, ok_q, icen_q, rmen_q, hit_q;
    reg [11:0]     skew_q;
    reg [31:0]     dip_q;
    reg [10:0]     paylen_q;
    reg            frag_q;
    reg [10:0]     lmax_q;
    reg [1:0]      neten_q;
    reg            snv_q;


    reg dbg_bad_ok, dbg_bad_hit, dbg_bad_lmax, dbg_bad_dstip;

    // 读出的状态
    reg [7:0]  psn_a_q, psn_b_q, rm_psn_q;
    reg        rm_vld_q;
    reg [1:0]  ic_seen_q;
    reg [15:0] rm_ts_q;

    wire [IDXW-1:0] idx = f_vlid[IDXW-1:0];      // 直接哈希（与 VL 表同址）

    // 查表请求：S_IDLE 那拍用未锁存的 f_vlid，S_RD 用锁存值
    assign lk_vlid_o = (st == S_RD) ? vlid_q : f_vlid;
    assign lk_req_o  = (st == S_RD) || (frame_end && st == S_IDLE);

    // 循环加（1..255 空间）
    function [7:0] inc_cyc;
        input [7:0] v;
        begin
            inc_cyc = (v == 8'd255) ? 8'd1 : (v + 8'd1);
        end
    endfunction

    // 16 位循环距离（时基用）
    function [15:0] dist;
        input [15:0] a, b;                       // a - b 的循环距离
        begin
            dist = (a >= b) ? (a - b) : (a + (16'd65535 - b) + 16'd1);
        end
    endfunction

    // 8 位循环距离（序列号用，mod 256）
    function [7:0] dist8;
        input [7:0] a, b;
        begin
            dist8 = (a >= b) ? (a - b) : (a + (8'd255 - b) + 8'd1);
        end
    endfunction

    // IC 判据
    wire [7:0] psn_now  = net_q ? psn_b_q : psn_a_q;
    wire [7:0] ic_n1    = inc_cyc(psn_now);
    wire [7:0] ic_n2    = inc_cyc(inc_cyc(psn_now));
    wire       ic_seen_now = net_q ? ic_seen_q[1] : ic_seen_q[0];
    wire       ic_pass  = !icen_q || (sn_q == 8'd0) || !ic_seen_now ||
                          (sn_q == ic_n1) || (sn_q == ic_n2);

    // RM 判据
    wire       skew_ex  = (dist(ts_now, rm_ts_q) > {4'h0, skew_q});
    wire [7:0] rm_delta = dist8(sn_q, rm_psn_q);

    wire       rm_dup   = snv_q && rm_vld_q && (rm_delta == 8'd0);

    wire       rm_reset = (sn_q == 8'd0) && (rm_psn_q != 8'd0);
    wire       rm_new   = !snv_q || !rm_vld_q || (rm_delta != 8'd0 && rm_delta < 8'd128) || rm_reset;


    wire [10:0] lmax_eff = frag_q ? 11'd1479 : lmax_q;
    wire       bad_lmax = (paylen_q > lmax_eff);
    // ② 目的 IP 必须是组播地址 224.224.<VLID>（P7 §3.2.5 / Part 7：目的 IP 由 VLID 派生）。
    //    DA 的常量域与 VLID 在解析器里已查，但**目的 IP 与 VLID 是否自洽**只有这里能查。
    wire [31:0] exp_dip  = {8'hE0, 8'hE0, vlid_q[15:8], vlid_q[7:0]};
    wire       bad_dstip = (dip_q != exp_dip);

    wire       bad_net  = (neten_q == 2'd1) ? (net_q != 1'b0) :
                          (neten_q == 2'd2) ? (net_q != 1'b1) : 1'b0;

    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; accept <= 0; ev_filt_drop <= 0; ev_ic_drop <= 0;
            ev_rm_drop <= 0; rm_dup_o <= 0; out_idx <= 0;
            idx_q <= 0; vlid_q <= 0; sn_q <= 0; net_q <= 0; ok_q <= 0;
            hit_q <= 0; icen_q <= 0; rmen_q <= 0; skew_q <= 0; snv_q <= 1'b1;
            ic_seen_q <= 2'd0;
            lmax_q <= 0; neten_q <= 2'd0; frag_q <= 1'b0;
        end
        else begin
            accept      <= 1'b0;
            ev_filt_drop<= 1'b0;
            ev_ic_drop  <= 1'b0;
            ev_rm_drop  <= 1'b0;
            rm_dup_o    <= 1'b0;
            case (st)
                S_IDLE: begin
                    if (frame_end) begin
                        idx_q  <= idx;
                        vlid_q <= f_vlid;
                        sn_q   <= f_sn;
                        net_q  <= f_net;
                        ok_q   <= f_ok;
                        dip_q  <= f_dip;
                        paylen_q <= f_paylen;
                        frag_q   <= f_frag;
                        st     <= S_RD;
                    end
                end
                S_RD: begin                                   // 读状态 + 等查表结果
                    psn_a_q <= psn_a[idx_q];
                    psn_b_q <= psn_b[idx_q];
                    rm_psn_q<= rm_psn[idx_q];
                    rm_vld_q<= rm_vld[idx_q];
                    ic_seen_q<= ic_seen[idx_q];
                    rm_ts_q <= rm_ts[idx_q];
                    if (lk_res_vlid_i == vlid_q) begin        // 结果确属本帧（未被 CPU 抢走）
                        hit_q  <= lk_hit_i;
                        icen_q <= lk_ic_en_i;
                        rmen_q <= lk_rm_en_i;
                        skew_q <= lk_skew_i;
                        lmax_q <= lk_lmax_i;
                        neten_q <= lk_neten_i;
                        snv_q   <= lk_sn_vld_i;
                        st     <= S_EVAL;
                    end
                end
                S_EVAL: begin

                    dbg_bad_ok    <= !ok_q;
                    dbg_bad_hit   <= !hit_q;
                    dbg_bad_lmax  <= bad_lmax;
                    dbg_bad_dstip <= bad_dstip;

                    if (!ok_q || !hit_q || bad_lmax || bad_dstip || bad_net) begin
                        ev_filt_drop <= 1'b1;                 // 帧无效 / VL 未配置 / 超 Lmax / 目的IP不符
                        st <= S_IDLE;
                    end
                    else if (!ic_pass) begin
                        // IC 丢弃：但 PSN 仍要更新（标准：PSN 是"收到"的上一帧）
                        if (net_q) psn_b[idx_q] <= sn_q; else psn_a[idx_q] <= sn_q;

                        ic_seen[idx_q] <= ic_seen_q | (net_q ? 2'b10 : 2'b01);
                        ev_ic_drop <= 1'b1;
                        st <= S_IDLE;
                    end
                    else begin
                        // IC 通过 → 更新 IC 状态
                        if (net_q) psn_b[idx_q] <= sn_q; else psn_a[idx_q] <= sn_q;

                        ic_seen[idx_q] <= ic_seen_q | (net_q ? 2'b10 : 2'b01);
                        // RM
                        if (!rmen_q) begin
                            accept <= 1'b1;                   // 关 RM → 一律交付
                            out_idx<= idx_q;
                        end
                        else if (skew_ex) begin
                            accept <= 1'b1;                   // 超 SkewMax → 无条件接受（重同步）
                            rm_psn[idx_q] <= sn_q; rm_vld[idx_q] <= 1'b1; rm_ts[idx_q] <= ts_now;
                            out_idx<= idx_q;
                        end
                        else if (rm_dup) begin
                            ev_rm_drop <= 1'b1;               // 重复副本 → First Win
                            rm_dup_o   <= 1'b1;
                        end
                        else if (rm_new) begin
                            accept <= 1'b1;
                            rm_psn[idx_q] <= sn_q; rm_vld[idx_q] <= 1'b1; rm_ts[idx_q] <= ts_now;
                            out_idx<= idx_q;
                        end
                        else begin
                            ev_rm_drop <= 1'b1;               // 乱序（旧帧）
                            rm_dup_o   <= 1'b1;               // 旧帧同样"已收过" ⇒ 不作废拼装
                        end
                        st <= S_IDLE;
                    end
                end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
