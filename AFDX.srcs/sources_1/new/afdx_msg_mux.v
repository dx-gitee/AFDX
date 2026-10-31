// afdx_msg_mux.v —— 发送消息流的源仲裁
`timescale 1ns/1ps

module afdx_msg_mux (
    input  wire        clk,
    input  wire        reset,

    // ---- 源 ①：端口层 ----
    input  wire        txp_valid,
    input  wire        txp_sop,
    input  wire        txp_eop,
    input  wire [7:0]  txp_data,
    input  wire [15:0] txp_vlid,
    input  wire [13:0] txp_len,
    input  wire        txp_pend,       // 端口层"有活"（ppend | act_vld）
    output wire        txp_ready,

    // ---- 源 ②：应用侧桥 ----
    input  wire        app_valid,
    input  wire        app_sop,
    input  wire        app_eop,
    input  wire [7:0]  app_data,
    input  wire [15:0] app_vlid,
    input  wire [13:0] app_len,
    output wire        app_ready,

    // ---- 组帧器 ----
    input  wire        pay_hold,
    output wire        msg_valid,
    output wire        msg_sop,
    output wire        msg_eop,
    output wire [7:0]  msg_data,
    output wire [15:0] msg_vlid,
    output wire [13:0] msg_len,
    input  wire        msg_ready,
    input  wire        msg_drop,

    // ---- 谁在用（给 `msg_drop` 分流）----
    output wire        owner_txp,
    output wire        owner_app
);
    localparam [1:0] S_NONE = 2'd0, S_TXP = 2'd1, S_APP = 2'd2;


    reg  [1:0] cur;
    wire       can_pick = (cur == S_NONE) && !pay_hold;


    reg  [19:0] stuck_cnt;
    wire        stuck_fire = (stuck_cnt == 20'hFFFFF);


    reg         first_taken;          // 本条消息的首字节已被接收
    reg  [11:0] pre_cnt;              // 选中后"首字节未被接收"的拍数
    wire        pre_fire = (pre_cnt == 12'hFFF);
    reg         skip_txp, skip_app;
    reg  [15:0] skip_cnt;
    // 让位窗口 = 2^16 拍 ≈ **524 µs**：端口层最多多等半个毫秒，而应用侧拿到
    // 足够长的独占窗口把自己的消息发完（33 µs 的窗口会被"配置查表 + B 副本"打断）。
    wire        skip_expire = (skip_cnt == 16'hFFFF);

    reg         txp_mask;
    reg  [19:0] mask_tmo;

    wire        mask_expire = (mask_tmo == 20'hFFFFF);
    always @(posedge clk) begin
        if (reset)              mask_tmo <= 20'd0;
        else if (!txp_mask)     mask_tmo <= 20'd0;
        else if (!mask_expire)  mask_tmo <= mask_tmo + 20'd1;
    end
    always @(posedge clk) begin
        if (reset)                             txp_mask <= 1'b0;
        else if (!txp_pend)                    txp_mask <= 1'b0;
        else if (txp_valid)                    txp_mask <= 1'b0;
        else if (mask_expire)                  txp_mask <= 1'b0;
        else if (stuck_fire && (cur == S_TXP)) txp_mask <= 1'b1;
    end
    always @(posedge clk) begin
        if (reset)                                  stuck_cnt <= 20'd0;
        else if ((cur == S_NONE) || msg_valid)      stuck_cnt <= 20'd0;
        else if (!stuck_fire)                       stuck_cnt <= stuck_cnt + 20'd1;
    end

    wire first_hs = msg_valid && msg_ready && msg_sop;
    always @(posedge clk) begin
        if (reset) first_taken <= 1'b0;
        else if (cur == S_NONE) first_taken <= 1'b0;
        else if (first_hs)      first_taken <= 1'b1;
    end

    always @(posedge clk) begin
        if (reset)                                    pre_cnt <= 12'd0;
        else if (cur == S_NONE || first_taken || !msg_valid) pre_cnt <= 12'd0;
        else if (!pre_fire)                           pre_cnt <= pre_cnt + 12'd1;
    end

    // 让位屏蔽：置位 = pre_fire 那拍的当前源；清除 = 它自己撤销 / 计时到
    always @(posedge clk) begin
        if (reset) skip_cnt <= 16'd0;
        else if (skip_txp || skip_app) begin
            if (skip_expire) skip_cnt <= 16'd0;
            else             skip_cnt <= skip_cnt + 16'd1;
        end else skip_cnt <= 16'd0;
    end

    always @(posedge clk) begin
        if (reset)                          skip_txp <= 1'b0;
        else if (!txp_pend)                 skip_txp <= 1'b0;
        else if (skip_expire)               skip_txp <= 1'b0;
        else if (first_hs && (cur == S_TXP)) skip_txp <= 1'b0;
        else if (pre_fire && (cur == S_TXP)) skip_txp <= 1'b1;
    end
    always @(posedge clk) begin
        if (reset)                          skip_app <= 1'b0;
        else if (!app_valid)                skip_app <= 1'b0;
        else if (skip_expire)               skip_app <= 1'b0;
        else if (first_hs && (cur == S_APP)) skip_app <= 1'b0;
        else if (pre_fire && (cur == S_APP)) skip_app <= 1'b1;
    end

    always @(posedge clk) begin
        if (reset) cur <= S_NONE;
        else if (can_pick) begin
            if      (txp_pend && !txp_mask && !skip_txp) cur <= S_TXP;  // 端口层优先
            else if (app_valid && !skip_app)             cur <= S_APP;
        end else if (msg_valid && msg_ready && msg_eop) begin
            cur <= S_NONE;                      // 本条收完，放回空闲
        end else if (msg_drop || stuck_fire || pre_fire) begin

            cur <= S_NONE;
        end
    end

    assign msg_valid = (cur == S_TXP) ? txp_valid :
                       (cur == S_APP) ? app_valid : 1'b0;
    assign msg_sop   = (cur == S_TXP) ? txp_sop   :
                       (cur == S_APP) ? app_sop   : 1'b0;
    assign msg_eop   = (cur == S_TXP) ? txp_eop   :
                       (cur == S_APP) ? app_eop   : 1'b0;
    assign msg_data  = (cur == S_APP) ? app_data  : txp_data;   // 默认侧接端口层（避免 X）
    assign msg_vlid  = (cur == S_APP) ? app_vlid  : txp_vlid;
    assign msg_len   = (cur == S_APP) ? app_len   : txp_len;

    assign txp_ready = (cur == S_TXP) ? msg_ready : 1'b0;
    assign app_ready = (cur == S_APP) ? msg_ready : 1'b0;

    assign owner_txp = (cur == S_TXP);
    assign owner_app = (cur == S_APP);
endmodule
