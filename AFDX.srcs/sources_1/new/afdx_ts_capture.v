// afdx_ts_capture.v —— GMII 帧首（SFD）硬件时间戳锁存（A/B 双网，收发各一份）
`timescale 1ns/1ps

module afdx_ts_capture #(
    parameter NETS = 2          // 网络数（A/B）
) (
    input  wire                    clk,
    input  wire                    reset,        // 高有效

    input  wire [31:0]             ts_now,       // 时间基准（80 ns 一格，自由运行）

    input  wire [NETS-1:0]         rx_dv,        // GMII 接收有效
    input  wire [NETS*8-1:0]       rx_d,         // GMII 接收数据（每网 8 位）
    input  wire [NETS-1:0]         tx_en,        // GMII 发送有效
    input  wire [NETS*8-1:0]       tx_d,         // GMII 发送数据

    output reg  [NETS*32-1:0]      rx_ts,        // 最近一帧的接收 SFD 时间戳
    output reg  [NETS*8-1:0]       rx_seq,       // 每锁存一次 +1（seqlock）
    output reg  [NETS*32-1:0]      tx_ts,        // 最近一帧的发送 SFD 时间戳
    output reg  [NETS*8-1:0]       tx_seq
);
    reg [NETS-1:0] rx_hit_q, tx_hit_q;      // 上一拍是否已命中（去重）
    reg [NETS-1:0] rx_hit,   tx_hit;
    integer i;

    always @(*) begin
        for (i = 0; i < NETS; i = i + 1) begin
            rx_hit[i] = rx_dv[i] && (rx_d[8*i +: 8] == 8'hD5);
            tx_hit[i] = tx_en[i] && (tx_d[8*i +: 8] == 8'hD5);
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            rx_hit_q <= {NETS{1'b0}};
            tx_hit_q <= {NETS{1'b0}};
            rx_ts    <= {(NETS*32){1'b0}};
            tx_ts    <= {(NETS*32){1'b0}};
            rx_seq   <= {(NETS*8){1'b0}};
            tx_seq   <= {(NETS*8){1'b0}};
        end
        else begin
            rx_hit_q <= rx_hit;
            tx_hit_q <= tx_hit;
            for (i = 0; i < NETS; i = i + 1) begin
                // 只在"**这一拍才开始**命中"时锁存（连续 2 拍命中只记一次）
                if (rx_hit[i] && !rx_hit_q[i]) begin
                    rx_ts[32*i +: 32]  <= ts_now;
                    rx_seq[8*i +: 8]   <= rx_seq[8*i +: 8] + 8'd1;
                end
                if (tx_hit[i] && !tx_hit_q[i]) begin
                    tx_ts[32*i +: 32]  <= ts_now;
                    tx_seq[8*i +: 8]   <= tx_seq[8*i +: 8] + 8'd1;
                end
            end
        end
    end

endmodule
