// afdx_ts_txtable.v —— 按 VL 保存"最近一次发送/接收帧"的硬件时间戳（A/B 各一张；TX/RX 各例化一次）
`timescale 1ns/1ps

module afdx_ts_txtable #(
    parameter integer NETS = 2,
    parameter integer VLW  = 12          // VLID 低 VLW 位（与 VL 配置表的直接哈希一致）
) (
    input  wire                 clk,
    input  wire                 reset,          // 高有效

    input  wire [31:0]          ts_now,         // 80 ns 自由运行计数
    input  wire [NETS-1:0]      gmii_en,          // GMII 有效（TX 或 RX）
    input  wire [NETS*8-1:0]    gmii_d,           // GMII 数据（每网 8 位；TX/RX 共用）

    input  wire [VLW:0]         vl_sel,         // {net, vlid[VLW-1:0]}
    output reg  [31:0]          vl_ts,          // 该 VL 最近一次发送的 SFD 时间戳
    output reg                  tbl_rdy         // 清表完成（1 = 可读）
);
    localparam integer NW = (1 << VLW);
    localparam integer DEPTH = NETS * NW;

    (* ramstyle = "M4K" *) reg [31:0] tbl [0:DEPTH-1];

    // ---------------- SFD / 字节计数 / DA[4..5] 解码 ----------------
    reg  [NETS-1:0] hit;              // 组合：本拍该网是 SFD
    integer i;
    always @(*) begin
        for (i = 0; i < NETS; i = i + 1)
            hit[i] = gmii_en[i] && (gmii_d[8*i +: 8] == 8'hD5);
    end

    reg  [NETS-1:0] sfd_q;            // 上一拍 SFD（去重）
    reg  [NETS-1:0] armed;            // 本帧正在数 DA 字节
    reg  [2:0]      bcnt [0:NETS-1];  // SFD 之后已过的字节数（0..5）
    reg  [7:0]      da4  [0:NETS-1];  // DA[4]（VLID 高字节）
    reg  [31:0]     sfd_ts[0:NETS-1]; // 本帧 SFD 时刻

    reg             wr_en;
    reg  [VLW:0]    wr_addr;
    reg  [31:0]     wr_ts;
    reg             clr_act;
    reg  [13:0]     clr_i;

    always @(posedge clk) begin
        if (reset) begin
            sfd_q <= {NETS{1'b0}};
            armed <= {NETS{1'b0}};
            wr_en <= 1'b0;
            for (i = 0; i < NETS; i = i + 1) begin
                bcnt[i] <= 3'd0; da4[i] <= 8'h00; sfd_ts[i] <= 32'd0;
            end
        end
        else begin
            sfd_q <= hit;
            wr_en <= 1'b0;
            for (i = 0; i < NETS; i = i + 1) begin
                if (hit[i] && !sfd_q[i]) begin
                    // 帧首：锁存时间戳，开始数 DA 字节
                    armed[i] <= 1'b1;
                    bcnt[i]  <= 3'd0;
                    sfd_ts[i]<= ts_now;
                end
                else if (armed[i] && gmii_en[i]) begin
                    if (bcnt[i] == 3'd4) da4[i] <= gmii_d[8*i +: 8];
                    if (bcnt[i] == 3'd5) begin

                        wr_en   <= (da4[i][7:4] == 4'h0);
                        wr_addr <= {i[0], da4[i][3:0], gmii_d[8*i +: 8]};
                        wr_ts   <= sfd_ts[i];
                        armed[i]<= 1'b0;
                    end
                    else bcnt[i] <= bcnt[i] + 3'd1;
                end
            end
        end
    end

    // ---------------- 表：清表 FSM（优先）/ 发帧写 / CPU 读 ----------------
    always @(posedge clk) begin
        if (reset) begin
            clr_act <= 1'b1;
            clr_i   <= 14'd0;
            tbl_rdy <= 1'b0;
            vl_ts   <= 32'd0;
        end
        else begin
            if (clr_act) begin
                tbl[clr_i] <= 32'h0;
                if (clr_i == DEPTH - 1) begin
                    clr_act <= 1'b0;
                    tbl_rdy <= 1'b1;
                end
                else clr_i <= clr_i + 14'd1;
            end
            else begin
                if (wr_en) tbl[wr_addr] <= wr_ts;       // 唯一的写口
                vl_ts <= tbl[vl_sel];                   // 唯一的读口（寄存读，地址=寄存器）
            end
        end
    end

endmodule
