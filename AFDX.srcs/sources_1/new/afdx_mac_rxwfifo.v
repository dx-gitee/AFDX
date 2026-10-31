// afdx_mac_rxwfifo.v —— MAC ff_rx 接收侧缓冲【字宽版】（剥 SFD + 深缓冲）
`timescale 1ns/1ps

module afdx_mac_rxwfifo #(
    parameter DEPTH = 2048,           // **字节数**（与原模块同口径）
    parameter AW    = 11              // log2(DEPTH)
) (
    input  wire        clk,
    input  wire        reset,         // 高有效

    input  wire        in_vld,
    input  wire        in_sop,
    input  wire        in_eop,
    input  wire [7:0]  in_data,
    output wire        in_rdy,

    output wire        out_vld,
    output wire        out_sop,
    output wire        out_eop,
    output wire [7:0]  out_data,
    input  wire        out_rdy,

    output wire [AW:0] used           // 占用**字节数**（与原模块同口径，便于复用单元台）
);
    localparam NW  = DEPTH / 4;       // RAM 字数
    localparam WAW = AW - 2;          // 字地址位宽

    // ---------------- 寄存器声明（全部提前）----------------
    // 写侧
    reg  [7:0]   wb0, wb1, wb2, wb3;  // 装配中的字
    reg  [1:0]   wn;                  // 已装入字节数 0..3
    reg          wsop;                // 本字第 0 字节是否带 sop
    reg          pend_sop;            // 刚丢过 SFD，下一字节才是帧首
    reg  [WAW:0] wp;                  // 写字地址（纯寄存器）
    reg  [AW:0]  ub;                  // RAM 里的**字节数**
    reg  [WAW:0] rc;                  // RAM 里**尚未装进 q0 的字数**
    // 读侧
    reg  [WAW-1:0] ra;
    reg  [35:0]    qr;                // 寄存读输出 = "下一个字"的预取（= §12.3 的 q1）
    reg  [35:0]    q0;                // 当前正在送出的字
    reg  [1:0]     bi;                // q0 里下一个要送的字节
    reg            v0;                // q0 有效
    reg            rd_safe;           // 上一拍那次读是否安全
    reg            just_loaded;
    reg  [7:0]     q0_byte;
    // ---- 容量按**字**算（RAM 只有 NW 个字槽）----
    wire [WAW:0] wused = rc + (v0 ? 1'b1 : 1'b0) + ((wn != 2'd0) ? 1'b1 : 1'b0);

    (* ramstyle = "M4K" *) reg [35:0] mem [0:NW-1];

    // ---------------- 组合：写侧 ----------------
    wire is_sfd  = in_vld && in_sop && (in_data == 8'hD5);
    wire sop_eff = (in_sop && !is_sfd) || pend_sop;

    wire [1:0] q0_cnt = q0[35:34];
    wire       q0_eop = q0[33];
    wire       q0_sop = q0[32];

    wire [2:0]  q0_rem  = v0 ? ({1'b0, q0_cnt} - {1'b0, bi} + 3'd1) : 3'd0;
    wire [AW:0] q0bytes = {{(AW-2){1'b0}}, q0_rem};              // 给 `used`（要准）
    wire [AW:0] q0_occ  = v0 ? ({10'd0, q0_cnt} + 4'd1) : {(AW+1){1'b0}};  // 给满/反压（保守）
    wire [AW:0] occ     = ub + q0_occ + {10'd0, wn};

    wire        push    = in_vld && !is_sfd && ((wused + ((wn == 2'd0) ? 1'b1 : 1'b0)) <= NW);
    wire        word_end = push && (in_eop || (wn == 2'd3));

    wire [7:0] nd0 = (push && wn == 2'd0) ? in_data : wb0;
    wire [7:0] nd1 = (push && wn == 2'd1) ? in_data : wb1;
    wire [7:0] nd2 = (push && wn == 2'd2) ? in_data : wb2;
    wire [7:0] nd3 = (push && wn == 2'd3) ? in_data : wb3;
    wire       nsop = (push && wn == 2'd0) ? sop_eff : wsop;

    wire [35:0] win = {wn, (push && in_eop), nsop, nd3, nd2, nd1, nd0};
    wire [2:0]  wbytes = {1'b0, wn} + 3'd1;

    // ---------------- 组合：读侧 ----------------
    wire pop       = out_vld && out_rdy;
    wire last_byte = (bi == q0_cnt);
    wire word_done = pop && last_byte;                          // 本字取完

    wire load      = (rc != 0) && (!v0 || word_done) && rd_safe && !just_loaded; // 换字 / 起手

    always @(*) begin
        case (bi)
            2'd0:    q0_byte = q0[7:0];
            2'd1:    q0_byte = q0[15:8];
            2'd2:    q0_byte = q0[23:16];
            default: q0_byte = q0[31:24];
        endcase
    end

    assign out_vld  = v0;
    assign out_sop  = q0_sop && (bi == 2'd0);
    assign out_eop  = q0_eop && last_byte;
    assign out_data = q0_byte;
    // 反压：按字判（与上面的 push 同条件，同拍生效）
    assign in_rdy   = ((wused + ((wn == 2'd0) ? 1'b1 : 1'b0)) <= NW);

    assign used     = ub + q0bytes + {10'd0, wn};

    // ---------------- 时序 ----------------
    always @(posedge clk) begin
        if (reset) begin
            wb0 <= 8'h00; wb1 <= 8'h00; wb2 <= 8'h00; wb3 <= 8'h00;
            wn <= 2'd0; wsop <= 1'b0; pend_sop <= 1'b0;
            wp <= {(WAW+1){1'b0}}; ub <= {(AW+1){1'b0}}; rc <= {(WAW+1){1'b0}};
            ra <= {(WAW){1'b0}}; qr <= 36'd0; q0 <= 36'd0; bi <= 2'd0;
            v0 <= 1'b0; rd_safe <= 1'b0; just_loaded <= 1'b0;
        end
        else begin
            // ---- SFD ----
            if (is_sfd)        pend_sop <= 1'b1;
            else if (push)     pend_sop <= 1'b0;

            // ---- 写侧：装配 + 收尾写一个字 ----
            if (push) begin
                wb0 <= nd0; wb1 <= nd1; wb2 <= nd2; wb3 <= nd3;
                wsop <= nsop;
                wn   <= word_end ? 2'd0 : (wn + 2'd1);
            end
            if (word_end) begin
                mem[wp[WAW-1:0]] <= win;
                wp <= wp + 1'b1;
            end

            // ---- 寄存读（**地址永远是寄存器 ra**）----
            just_loaded <= load;
            qr <= mem[ra];

            rd_safe <= !(word_end && (wp[WAW-1:0] == ra));

            // ---- 换字 / 起手 / 字内前进（每拍只更新一次）----
            if (load) begin
                q0 <= qr; v0 <= 1'b1; bi <= 2'd0; ra <= ra + 1'b1;
            end
            else if (word_done) begin
                v0 <= 1'b0; bi <= 2'd0;                        // 取完了，且没有下一个字
            end
            else if (pop) begin
                bi <= bi + 2'd1;
            end

            // ---- 记账（每拍只写一次，加减合并成一个表达式）----
            ub <= ub + (word_end ? {7'd0, wbytes} : {(AW+1){1'b0}})
                     - (load ? ({10'd0, qr[35:34]} + 4'd1) : {(AW+1){1'b0}});
            rc <= rc + (word_end ? 1'b1 : 1'b0) - (load ? 1'b1 : 1'b0);
        end
    end

endmodule
