// afdx_rx_arb.v —— A/B 双网接收流仲裁（2 → 1，带网络标记）
`timescale 1ns/1ps

module afdx_rx_arb (
    input  wire        clk,
    input  wire        reset,

    // ---- A 网（来自 MAC 壳 A）----
    input  wire        a_valid,
    input  wire        a_sop,
    input  wire        a_eop,
    input  wire [7:0]  a_data,
    output wire        a_ready,

    // ---- B 网（来自 MAC 壳 B）----
    input  wire        b_valid,
    input  wire        b_sop,
    input  wire        b_eop,
    input  wire [7:0]  b_data,
    output wire        b_ready,

    // ---- 合并后的引擎侧流 ----
    output wire        m_valid,
    output wire        m_sop,
    output wire        m_eop,
    output wire [7:0]  m_data,
    output wire        m_net        // 0 = A 网, 1 = B 网
);
    reg  busy;                      // 正在转发某一帧
    reg  cur;                       // 正在转发的网络（0=A,1=B）
    wire sel;                       // 本拍放行哪张网

    reg  [15:0] wdt;

    // 仲裁：
    //   忙 → 固定放行 cur（保证一帧不被插队打断）
    //   闲 → 有 A 就选 A（A 优先），否则选 B；两者都没有时 sel 取值无关紧要
    assign sel = busy ? cur : (a_valid ? 1'b0 : 1'b1);

    always @(posedge clk) begin
        if (reset) begin
            wdt <= 16'd0;
            busy <= 1'b0;
            cur  <= 1'b0;
        end
        else if (!busy) begin

            if (a_valid) begin cur <= 1'b0; wdt <= 16'd0; busy <= ~a_eop; end
            else if (b_valid) begin cur <= 1'b1; wdt <= 16'd0; busy <= ~b_eop; end
        end
        else begin

            wdt <= wdt + 16'd1;
            if (cur == 1'b0) begin
                if ((a_valid && a_eop) || (wdt == 16'd40950)) busy <= 1'b0;
            end
            else begin
                if ((b_valid && b_eop) || (wdt == 16'd40950)) busy <= 1'b0;
            end
        end
    end

    assign m_valid = sel ? b_valid : a_valid;
    assign m_sop   = sel ? b_sop   : a_sop;
    assign m_eop   = sel ? b_eop   : a_eop;
    assign m_data  = sel ? b_data  : a_data;
    assign m_net   = sel;


    assign a_ready = busy ? (cur == 1'b0) : a_valid;
    assign b_ready = busy ? (cur == 1'b1) : (b_valid & ~a_valid);

endmodule
