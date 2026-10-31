// 文件名 : afdx_crc32.v
`timescale 1ns/1ps

module afdx_crc32 (
    input  wire        clk,        // 时钟信号
    input  wire        reset,      // 复位信号，高有效
    input  wire        init,       // 同步初始化，将CRC寄存器装载为0xFFFFFFFF
    input  wire        en,         // 字节有效使能
    input  wire [7:0]  data_in,    // 输入字节
    output reg  [31:0] crc_out     // 当前CRC寄存器值
);

    // 单字节CRC-32组合更新函数（8次移位异或）

    function [31:0] crc32_byte;
        input [31:0] crc_in;
        input [7:0]  din;
        integer      i;
        reg   [31:0] c;
        begin
            c = crc_in ^ {24'h000000, din};
            for (i = 0; i < 8; i = i + 1) begin
                if (c[0])
                    c = (c >> 1) ^ 32'hEDB88320;
                else
                    c = (c >> 1);
            end
            crc32_byte = c;
        end
    endfunction

    // CRC寄存器
    // init与en可能同拍有效（帧首字节），此时先装载初值再折叠该字节。
   
    always @(posedge clk) begin
        if (reset)
            crc_out <= 32'hFFFFFFFF;
        else if (init) begin
            if (en) crc_out <= crc32_byte(32'hFFFFFFFF, data_in);
            else    crc_out <= 32'hFFFFFFFF;
        end
        else if (en)
            crc_out <= crc32_byte(crc_out, data_in);
    end

endmodule
