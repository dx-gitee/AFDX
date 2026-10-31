// afdx_reasm.v —— 接收方向 IP 分片重组（ARINC 664 P7 要求端系统支持）
`timescale 1ns/1ps

module afdx_reasm #(
    parameter AW = 11,          // 每个字节通道的**字**地址位宽 → 2^AW 字 × 4 字节 = 8 KB/条

    parameter integer TMO_CYC = 64000000
) (
    input  wire        clk,
    input  wire        reset,

    // ---------------- 来自解析器的载荷流 + 随附信息 ----------------
    input  wire        in_valid,
    input  wire        in_sop,
    input  wire        in_eop,
    input  wire [7:0]  in_data,
    input  wire        in_net,
    input  wire [15:0] in_vlid,
    input  wire [15:0] in_dport,
    input  wire [31:0] in_sip,
    input  wire [15:0] in_sport,
    input  wire [2:0]  in_port,       // 目的端口号（顶层由提前查表给出）

    // ---------------- 帧末描述符（1 拍）+ 分片信息 + IC/RM 判决 ----------------
    input  wire        in_desc,     // 描述符有效（eop 后 1 拍）
    input  wire        in_frag,     // 本帧是分片
    input  wire        in_mf,       // 还有后续片
    input  wire [12:0] in_foff,     // 片偏移（8 字节单位）
    input  wire [15:0] in_ipid,     // IP 标识
    input  wire        in_ok,       // 本帧整体合法（解析 ok）
    input  wire        in_accept,   // IC/RM：接受这一片
    input  wire        in_abort,    // IC/RM：丢弃这一片

    input  wire        in_dup,

    // ---------------- 输出：一条完整消息（或原样透传）----------------
    output reg         out_valid,
    output reg         out_sop,
    output reg         out_eop,
    output reg  [7:0]  out_data,
    output reg  [15:0] out_vlid,
    output reg  [15:0] out_dport,
    output reg  [31:0] out_sip,
    output reg  [15:0] out_sport,
    output reg  [2:0]  out_port,
    output reg         out_net,
    output reg         out_commit,  // 整条消息就绪（单拍脉冲）
    output reg         out_abort,   // 整条消息作废（单拍脉冲）

    output reg         out_vbytes,

    // ---------------- 自证计数 ----------------
    output reg  [15:0] ev_reasm_ok,     // 成功重组的条数
    output reg  [15:0] ev_reasm_err     // 乱序/不匹配/被 IC-RM 丢弃导致的作废条数
);
    localparam NW = 1 << AW;                 // 每通道**字数**（×4 字节 = 8 KB）
    localparam BW = AW + 2;                  // 字节地址位宽


    reg [7:0] ba0 [0:NW-1], ba1 [0:NW-1], ba2 [0:NW-1], ba3 [0:NW-1];   // 缓冲 A
    reg [7:0] bb0 [0:NW-1], bb1 [0:NW-1], bb2 [0:NW-1], bb3 [0:NW-1];   // 缓冲 B
    reg [7:0] bc0 [0:NW-1], bc1 [0:NW-1], bc2 [0:NW-1], bc3 [0:NW-1];   // 缓冲 C
    reg [7:0] bd0 [0:NW-1], bd1 [0:NW-1], bd2 [0:NW-1], bd3 [0:NW-1];   // 缓冲 D

    initial $readmemh("reasmzero.hex", ba0);
    initial $readmemh("reasmzero.hex", ba1);
    initial $readmemh("reasmzero.hex", ba2);
    initial $readmemh("reasmzero.hex", ba3);
    initial $readmemh("reasmzero.hex", bb0);
    initial $readmemh("reasmzero.hex", bb1);
    initial $readmemh("reasmzero.hex", bb2);
    initial $readmemh("reasmzero.hex", bb3);
    initial $readmemh("reasmzero.hex", bc0);
    initial $readmemh("reasmzero.hex", bc1);
    initial $readmemh("reasmzero.hex", bc2);
    initial $readmemh("reasmzero.hex", bc3);
    initial $readmemh("reasmzero.hex", bd0);
    initial $readmemh("reasmzero.hex", bd1);
    initial $readmemh("reasmzero.hex", bd2);
    initial $readmemh("reasmzero.hex", bd3);


    reg [1:0]    acc_bank;      // 正在攒的缓冲（轮转指针）
    reg [1:0]    rpl_bank;      // 正在回放的缓冲
    reg          acc_act;       // 正在攒一条分片数据报
    reg [25:0]   acc_tmo;

    reg [BW-1:0] wnow;          // 本片当前写位置（字节地址）

    reg [BW:0] wrp;             // 已攒到的长度
    reg [15:0]   cur_vlid, cur_ipid;

    reg [15:0]   lat_dport, lat_sport;
    reg [31:0]   lat_sip;
    reg          lat_net;
    reg [2:0]    lat_port;
                                //    而顶层的 port_lat 是"当前帧"查出来的，直接用会送错端口

    reg          wr_en;
    reg [7:0]    wr_b;

    reg [BW-1:0] wa_r;


    wire [BW+4:0] wstart_full = (in_foff == 13'd0) ? {(BW+5){1'b0}}
                                                   : ({5'b0, in_foff, 3'b000} - 8'd8);
    wire [BW-1:0] wstart = wstart_full[BW-1:0];     // 只用于写地址（越界时 w_ovf 会挡写）

    reg  [BW-1:0] dw;            // 本片已写的**数据**字节数（不含本拍）
    reg  [BW:0]   dwx;

    wire [BW-1:0] dw_now  = in_sop ? {BW{1'b0}} : dw;    // 本片已写的数据字节数（不含本拍）
    wire [BW-1:0] waddr  = wstart + dw_now;

    wire [BW:0]   dwx_now = in_sop ? {(BW+1){1'b0}} : dwx;
    wire [BW+4:0] wend    = wstart_full + {1'b0, dwx_now};

    wire          w_ovf   = (wend >= (NW*4));   // NW*4 = 缓冲总字节数（8 KB）

    wire [BW+4:0] pd_end  = wstart_full + {1'b0, dwx};   // 本条已收字节数（全精度）
    wire          pd_ofl  = (pd_end > (NW*4));  // > 才是越界（== 8192 合法）

    // 回放侧
    reg [AW-1:0] raddr;         // 字地址
    reg [7:0]    rq0, rq1, rq2, rq3;

    reg [1:0]    rbyte;

    reg          wgap;

    reg          dq_wr, dq_rd;              // 1 位指针（数组只有 2 项，必须回绕）
    reg [1:0]    dq_cnt;                    // 在队条数（0..2）

    reg [6:0]    drop_cnt;

    reg [6:0]    drop_wr, drop_rd;
    reg          drop_vb_q [0:127];
    reg          pd_frag_q [0:1], pd_mf_q [0:1], pd_ok_q [0:1];
    reg [12:0]   pd_foff_q [0:1];
    reg [15:0]   pd_ipid_q [0:1], pd_vlid_q [0:1];

    reg [BW:0]   pd_len_q  [0:1];
    reg          pd_ovf_q  [0:1];
    reg          pd_lost_q [0:1];
    reg          cur_lost;          // 当前正在收的这一帧是否丢过字节
    reg [BW:0] rsp_total;       // 本条的字节数

    wire [BW:0]  rsp_last = rsp_total - 1'b1;


    reg [1:0]    st;            // 0=IDLE 1=DONE 2=REPLAY 3=PRIME
    localparam S_IDLE=2'd0, S_DONE=2'd1, S_REPLAY=2'd2, S_PRIME=2'd3;


    reg          rp_req;        // 有数据报在等回放（当前回放在跑时不许直接抢 st）

    reg [BW:0] p_len;           // 挂起那条的字节数
    reg [1:0]    p_bank;        // 挂起那条的数据在哪块缓冲


    wire [AW-1:0] rw = raddr;
    always @(posedge clk) begin
        case (rpl_bank)
            2'd0: begin rq0 <= ba0[rw]; rq1 <= ba1[rw]; rq2 <= ba2[rw]; rq3 <= ba3[rw]; end
            2'd1: begin rq0 <= bb0[rw]; rq1 <= bb1[rw]; rq2 <= bb2[rw]; rq3 <= bb3[rw]; end
            2'd2: begin rq0 <= bc0[rw]; rq1 <= bc1[rw]; rq2 <= bc2[rw]; rq3 <= bc3[rw]; end
            default: begin rq0 <= bd0[rw]; rq1 <= bd1[rw]; rq2 <= bd2[rw]; rq3 <= bd3[rw]; end
        endcase
    end

    // ---- 写口：按字节通道写 ----
    always @(posedge clk) begin
        if (wr_en) begin
            if (acc_bank == 2'd2) begin
                case (wa_r[1:0])
                    2'd0: bc0[wa_r[BW-1:2]] <= wr_b;
                    2'd1: bc1[wa_r[BW-1:2]] <= wr_b;
                    2'd2: bc2[wa_r[BW-1:2]] <= wr_b;
                    default: bc3[wa_r[BW-1:2]] <= wr_b;
                endcase
            end
            else if (acc_bank == 2'd3) begin
                case (wa_r[1:0])
                    2'd0: bd0[wa_r[BW-1:2]] <= wr_b;
                    2'd1: bd1[wa_r[BW-1:2]] <= wr_b;
                    2'd2: bd2[wa_r[BW-1:2]] <= wr_b;
                    default: bd3[wa_r[BW-1:2]] <= wr_b;
                endcase
            end
            else if (acc_bank == 1'b0) begin
                case (wa_r[1:0])
                    2'd0: ba0[wa_r[BW-1:2]] <= wr_b;
                    2'd1: ba1[wa_r[BW-1:2]] <= wr_b;
                    2'd2: ba2[wa_r[BW-1:2]] <= wr_b;
                    default: ba3[wa_r[BW-1:2]] <= wr_b;
                endcase
            end
            else begin
                case (wa_r[1:0])
                    2'd0: bb0[wa_r[BW-1:2]] <= wr_b;
                    2'd1: bb1[wa_r[BW-1:2]] <= wr_b;
                    2'd2: bb2[wa_r[BW-1:2]] <= wr_b;
                    default: bb3[wa_r[BW-1:2]] <= wr_b;
                endcase
            end
        end
    end


    // 本拍正在吃掉一条**分片**描述符（用于给重组超时复位）
    wire frag_desc_now = (dq_cnt != 2'd0) && pd_frag_q[dq_rd];

    wire frag_same_dg  = frag_desc_now && !in_dup &&
                         (pd_vlid_q[dq_rd] == cur_vlid) && (pd_ipid_q[dq_rd] == cur_ipid);
    // TMO_CYC 的安全钳位（26 位上限 67,108,863；配成 2^26 会截成 0 ⇒ 所有数据报立刻作废）
    wire [25:0] tmo_lim = (TMO_CYC < 1)        ? 26'd1      :
                          (TMO_CYC > 67108863) ? 26'h3FF_FFFF : TMO_CYC[25:0];

    integer i;
    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; acc_act <= 1'b0; acc_tmo <= 26'd0; acc_bank <= 2'd0; rpl_bank <= 2'd1;
            wnow <= {BW{1'b0}}; wrp <= {(BW+1){1'b0}}; raddr <= {AW{1'b0}};
            dw <= {BW{1'b0}}; dwx <= {(BW+1){1'b0}}; wa_r <= {BW{1'b0}}; wgap <= 1'b0;
            dq_wr <= 1'b0; dq_rd <= 1'b0; dq_cnt <= 2'd0; drop_cnt <= 7'd0; cur_lost <= 1'b0;
            drop_wr <= 7'd0; drop_rd <= 7'd0;

            for (i = 0; i < 128; i = i + 1) drop_vb_q[i] <= 1'b0;
            wr_en <= 1'b0; rbyte <= 2'd0; rsp_total <= {(BW+1){1'b0}};
            rp_req <= 1'b0; p_len <= {(BW+1){1'b0}}; p_bank <= 2'd0;
            out_valid <= 1'b0; out_sop <= 1'b0; out_eop <= 1'b0; out_data <= 8'h0;
            out_vlid <= 16'd0; out_commit <= 1'b0; out_abort <= 1'b0; out_vbytes <= 1'b0;
            ev_reasm_ok <= 16'd0; ev_reasm_err <= 16'd0;
            cur_vlid <= 16'd0; cur_ipid <= 16'd0;
            lat_dport <= 16'd0; lat_sport <= 16'd0; lat_sip <= 32'd0; lat_port <= 3'd0;
            lat_net   <= 1'b0;
            out_dport <= 16'd0; out_sport <= 16'd0; out_sip <= 32'd0; out_port <= 3'd0;
            out_net   <= 1'b0;
        end
        else begin
            wr_en      <= 1'b0;
            out_valid  <= 1'b0;
            out_commit <= 1'b0;
            out_abort  <= 1'b0;
            out_vbytes <= 1'b0;


            if (!acc_act)                       acc_tmo <= 26'd0;
            else if (frag_same_dg)              acc_tmo <= 26'd0;
            else if (acc_tmo != tmo_lim)        acc_tmo <= acc_tmo + 26'd1;

            if (acc_act && (acc_tmo == tmo_lim) && !frag_same_dg) begin
                acc_act      <= 1'b0;
                ev_reasm_err <= ev_reasm_err + 16'd1;
            end


            if (in_valid && in_sop &&
                ((!in_frag && !acc_act && (st == S_IDLE)) ||
                 ( in_frag && (in_foff == 13'd0)))) begin
                lat_dport <= in_dport; lat_sip <= in_sip; lat_sport <= in_sport;
                lat_port  <= in_port;
                lat_net   <= in_net;
            end

            // ---------------- 1) 收字节 ----------------
            if (in_valid) begin
                if (!in_frag) begin

                    if (in_sop)                       cur_lost <= (st != S_IDLE);   // 新帧：回放中就丢
                    else if ((st != S_IDLE) || cur_lost) cur_lost <= 1'b1;          // 丢过的帧保持丢
                    if ((st == S_IDLE) && (in_sop || !cur_lost)) begin
                        out_valid <= 1'b1;
                        out_data  <= in_data;
                        out_sop   <= in_sop;
                        out_eop   <= in_eop;
                        out_vlid  <= in_vlid;
                        out_dport <= in_sop ? in_dport : lat_dport;
                        out_sip   <= in_sop ? in_sip   : lat_sip;
                        out_sport <= in_sop ? in_sport : lat_sport;
                        out_port  <= in_sop ? in_port  : lat_port;
                        out_net   <= in_sop ? in_net   : lat_net;
                    end
                end
                else begin

                    if (!acc_act || (in_foff == 13'd0) ||
                        ((in_vlid == cur_vlid) && (in_ipid == cur_ipid))) begin
                        dwx <= dwx_now + 1'b1;
                        if (!w_ovf) begin
                            wr_b  <= in_data;
                            wa_r  <= waddr;             // 与 wr_b 同拍寄存
                            wr_en <= 1'b1;
                            dw    <= dw_now + 1'b1;
                        end
                    end
                end
            end


            if (in_desc) begin
                if ((dq_cnt != 2'd2) || (in_accept || in_abort)) begin
                    pd_frag_q[dq_wr] <= in_frag;
                    pd_mf_q  [dq_wr] <= in_mf;
                    pd_foff_q[dq_wr] <= in_foff;
                    pd_ipid_q[dq_wr] <= in_ipid;
                    pd_vlid_q[dq_wr] <= in_vlid;
                    pd_ok_q  [dq_wr] <= in_ok;

                    pd_len_q [dq_wr] <= pd_ofl ? {(BW+1){1'b0}} : pd_end[BW:0];
                    pd_ovf_q [dq_wr] <= pd_ofl;
                    pd_lost_q[dq_wr] <= cur_lost;
                    dq_wr <= dq_wr + 1'b1;
                end
                else begin

                    if (drop_cnt != 7'h7F) begin
                        // 记"这一帧的字节是否真的流出去过"：非分片且不在回放中 ⇒ 出去过
                        drop_vb_q[drop_wr] <= (!in_frag) && !cur_lost;
                        drop_wr  <= drop_wr + 7'd1;
                        drop_cnt <= drop_cnt + 7'd1;
                    end
                    ev_reasm_err <= ev_reasm_err + 16'd1;
                end
            end

            if (in_accept || in_abort) begin

                if (dq_cnt == 2'd0 && drop_cnt != 7'd0) begin

                    drop_cnt     <= drop_cnt - 7'd1;
                    out_abort    <= 1'b1;
                    out_vbytes   <= drop_vb_q[drop_rd];
                    drop_rd      <= drop_rd + 7'd1;
                    ev_reasm_err <= ev_reasm_err + 16'd1;
                end
                else if (dq_cnt != 2'd0) begin
                    dq_rd <= dq_rd + 1'b1;
                    if (!pd_frag_q[dq_rd]) begin
                        // 非分片：把判决原样往下传；回放期间丢过字节的不能假装 commit 成功
                        if (pd_lost_q[dq_rd]) begin
                            out_abort    <= 1'b1;
                            ev_reasm_err <= ev_reasm_err + 16'd1;
                        end
                        else if (in_accept) begin out_commit <= 1'b1; out_vbytes <= 1'b1; end
                        else                begin out_abort  <= 1'b1; out_vbytes <= 1'b1; end
                    end

                    else if (in_dup) begin
                        // 不碰 acc_act / cur_vlid / cur_ipid / wrp …
                        // …但**例外**：如果这份"旧帧"是**首片**（foff=0）且不属于当前
                        // 数据报，它的字节已经按上面的规则写进了 `acc_bank` 的 0.. 处
                        // ⇒ 正在拼装的 Q 已被**污染**。宁可**可见地作废** Q（计数 + err），
                        // 也不能把错数据交付出去（静默错数据比丢帧更糟）。
                        if ((pd_foff_q[dq_rd] == 13'd0) && acc_act &&
                            ((pd_vlid_q[dq_rd] != cur_vlid) || (pd_ipid_q[dq_rd] != cur_ipid))) begin
                            ev_reasm_err <= ev_reasm_err + 16'd1;
                            acc_act <= 1'b0;
                        end
                    end
                    else if (!in_accept || !pd_ok_q[dq_rd] || pd_ovf_q[dq_rd]) begin

                        if (acc_act || pd_ovf_q[dq_rd]) ev_reasm_err <= ev_reasm_err + 16'd1;
                        acc_act <= 1'b0;
                    end
                    else if (pd_foff_q[dq_rd] == 13'd0) begin
                        // 第一片：开一条新的
                        if (acc_act) ev_reasm_err <= ev_reasm_err + 16'd1;
                        acc_act  <= 1'b1;
                        cur_vlid <= pd_vlid_q[dq_rd];
                        cur_ipid <= pd_ipid_q[dq_rd];
                        wrp      <= pd_len_q[dq_rd];
                    end
                    else if (!acc_act || (pd_vlid_q[dq_rd] != cur_vlid) ||
                             (pd_ipid_q[dq_rd] != cur_ipid)) begin
                        ev_reasm_err <= ev_reasm_err + 16'd1;
                    end
                    else if (({5'b0, pd_foff_q[dq_rd], 3'b000} - 8'd8) != wrp) begin
                        ev_reasm_err <= ev_reasm_err + 16'd1;      // 不连续（乱序/丢中间片）
                        acc_act <= 1'b0;
                    end
                    else if (pd_mf_q[dq_rd] == 1'b0) begin

                        acc_act <= 1'b0;
                        if (rp_req) begin
                            // 双缓冲只支持"一条在回放 + 一条在等"；第三条只能丢（**计数**，不静默）
                            ev_reasm_err <= ev_reasm_err + 16'd1;
                        end
                        else begin
                            p_len    <= pd_len_q[dq_rd];
                            p_bank   <= acc_bank;
                            rp_req   <= 1'b1;
                            acc_bank <= acc_bank + 2'd1;
                        end
                    end
                    else begin
                        wrp <= pd_len_q[dq_rd];        // 中间片
                    end
                end
                else begin
                    // 没有待判描述符却来了判决（队列满丢过描述符时会这样）→ 可见地计数
                    ev_reasm_err <= ev_reasm_err + 16'd1;
                end
            end

            // 在队条数：+入队 -出队（同拍净变化）
            dq_cnt <= dq_cnt
                    + (((in_desc) && ((dq_cnt != 2'd2) || (in_accept || in_abort))) ? 2'd1 : 2'd0)
                    - (((in_accept || in_abort) && (dq_cnt != 2'd0)) ? 2'd1 : 2'd0);


            if (st == S_IDLE) begin
                if (rp_req) begin

                    if (p_len == {(BW+1){1'b0}}) begin
                        rp_req       <= 1'b0;
                        ev_reasm_err <= ev_reasm_err + 16'd1;
                    end
                    else begin
                        rpl_bank  <= p_bank;
                        rsp_total <= p_len;
                        raddr     <= {AW{1'b0}};
                        rbyte     <= 2'd0;
                        wgap      <= 1'b0;
                        rp_req    <= 1'b0;
                        st        <= S_PRIME;
                    end
                end
            end
            else if (st == S_PRIME) begin
                st <= S_REPLAY;                  // 这一拍 rq 把 buf[0] 取回来
            end
            else if (st == S_REPLAY && wgap) begin
                wgap <= 1'b0;                    // 换字空拍：这一拍不发，等 rq 更新
            end
            else if (st == S_DONE) begin
                out_commit <= 1'b1;              // 末字节的下一拍提交
                out_vbytes <= 1'b1;
                st         <= S_IDLE;            // 下一拍可能立刻接上挂起的那条
            end
            else if (st == S_REPLAY) begin
                begin
                    // 逐字节发（1 字节/拍）；rq* 是 raddr 对应的 4 个字节
                    out_valid <= 1'b1;
                    out_data  <= (rbyte == 2'd0) ? rq0 :
                                 (rbyte == 2'd1) ? rq1 :
                                 (rbyte == 2'd2) ? rq2 : rq3;
                    out_sop   <= (raddr == {AW{1'b0}}) && (rbyte == 2'd0);
                    out_eop   <= (raddr == rsp_last[AW+1:2]) &&
                                 (rbyte == rsp_last[1:0]);
                    out_vlid  <= cur_vlid;
                    out_dport <= lat_dport;
                    out_sip   <= lat_sip;
                    out_sport <= lat_sport;
                    out_port  <= lat_port;
                    out_net   <= lat_net;
                    // 推进
                    if ((raddr == rsp_last[AW+1:2]) && (rbyte == rsp_last[1:0])) begin

                        st         <= S_DONE;
                        ev_reasm_ok<= ev_reasm_ok + 16'd1;
                    end
                    else if (rbyte == 2'd3) begin
                        rbyte <= 2'd0;
                        raddr <= raddr + 1'b1;   // 下一个字
                        wgap  <= 1'b1;           // 下一拍空转，等 rq 更新
                    end
                    else begin
                        rbyte <= rbyte + 2'd1;
                    end
                end
            end
        end
    end

endmodule
