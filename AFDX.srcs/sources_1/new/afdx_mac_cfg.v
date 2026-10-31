// 文件名 : afdx_mac_cfg.v
`timescale 1ns/1ps

module afdx_mac_cfg (
    input  wire        clk,
    input  wire        reset,

    // MAC IP 寄存器接口
    output reg         reg_wr,
    output reg         reg_rd,
    output reg  [3:0]  reg_addr,
    output reg  [31:0] reg_data_in,
    input  wire [31:0] reg_data_out,
    input  wire        reg_acc_bsy,

    output wire        done        // 配置完成（CFG_DONE）—— 供上层观察/握手
);


    localparam CFG_IDLE = 2'd0;
    localparam CFG_WR   = 2'd1;
    localparam CFG_DONE = 2'd2;

    reg [1:0]  st;
    reg [7:0]  wait_cnt;
    reg [19:0] bsy_cnt;      // 等 reg_acc_bsy 空闲的兜底超时计数（125 MHz 下 20 位 ≈ 8.4 ms）

    always @(posedge clk) begin
        if (reset) begin
            st          <= CFG_IDLE;
            wait_cnt    <= 8'd0;
            bsy_cnt     <= 20'd0;
            reg_wr      <= 1'b0;
            reg_rd      <= 1'b0;
            reg_addr    <= 4'd0;
            reg_data_in <= 32'h0;
        end else begin
            case (st)
                CFG_IDLE: begin
                    reg_wr <= 1'b0;
                    if (wait_cnt < 8'd16) begin
                        wait_cnt <= wait_cnt + 8'd1;
                        bsy_cnt  <= 20'd0;
                    end else if (reg_acc_bsy && (bsy_cnt < 20'hFFFFF)) begin
                        // IP 寄存器接口忙时写会被静默丢弃。而这一写是"写 0 使能
                        // tx/rx"，丢一次等于整条链路一直关闭，所以必须等它空闲。
                        // bsy_cnt 做兜底：万一 bsy 一直不落，超时后照写，避免死等。
                        bsy_cnt <= bsy_cnt + 20'd1;
                    end else begin
                        // 写控制寄存器: reg_addr=2, data=0 => tx/rx 使能等
                        reg_wr      <= 1'b1;
                        reg_addr    <= 4'd2;
                        reg_data_in <= 32'h0;
                        st          <= CFG_WR;
                    end
                end
                CFG_WR: begin
                    reg_wr <= 1'b0;     // 拉低 reg_wr，完成一次写
                    st     <= CFG_DONE;
                end
                default: begin
                    reg_wr <= 1'b0;
                    st     <= CFG_DONE;
                end
            endcase
        end
    end


    assign done = (st == 2'd2);    // CFG_DONE

endmodule
