// afdx_port_vlid.v —— 端口号 → VLID 的唯一映射源
`timescale 1ns/1ps

module afdx_port_vlid #(
    parameter integer NPW = 4
) (
    input  wire [NPW-1:0] port,
    output reg  [15:0]    vlid
);
    always @(*) begin
        case (port)
            4'd0: vlid = 16'h0123;   // 采样口（dport 1024）
            4'd1: vlid = 16'h0456;   // 队列口（dport 1025）
            4'd2: vlid = 16'h0789;   // SAP-TFTP（dport 69）
            4'd3: vlid = 16'h0161;   // SAP-SNMP（dport 161）
            4'd4: vlid = 16'h0BC5;   // SAP-RTC（dport 3001）
            4'd5: vlid = 16'h0000;   // 保留/闲置
            4'd6: vlid = 16'h0A01;   // 数据面发送专用（端口扩容新增）
            4'd7: vlid = 16'h0A02;
            4'd8: vlid = 16'h0A03;
            4'd9: vlid = 16'h0A04;
            default: vlid = 16'h0000; // 越界端口：不继承任何 VL
        endcase
    end
endmodule
