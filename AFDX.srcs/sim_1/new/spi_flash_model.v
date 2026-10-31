// spi_flash_model.v —— SPI NOR flash 从机行为模型（mode 0）
`timescale 1ns/1ps

module spi_flash_model #(
    parameter AW      = 12,     // 地址位宽（12 → 4 KB）
    parameter USE_IMG = 0,      // 0 = 确定性模式；1 = $readmemh(IMG)

    parameter IMG     = "flash.hex"
) (
    input  wire sclk,
    input  wire cs_n,
    input  wire mosi,
    output reg  miso
);
    localparam SIZE = (1 << AW);
    reg [7:0] mem [0:SIZE-1];
    integer i, sbase;

    initial begin
        if (USE_IMG) begin
            $readmemh(IMG, mem);
        end
        else begin
            // 模式**必须含地址高位**（否则 0x000/0x100 同值，换地址用例验不出来）
            for (i = 0; i < SIZE; i = i + 1) mem[i] = (i*7 + 3 + (i >> 8)*101) & 8'hFF;
        end
    end

    localparam S_IDLE = 0, S_CMD = 1, S_ADDR = 2, S_DATA = 3,
               S_RDSR = 4, S_WDATA = 5;
    integer      st;
    reg [31:0]   shin;          // 命令 8 + 地址 24
    reg [4:0]    bitn;
    reg [7:0]    cmd;
    reg [AW-1:0] faddr;         // 读地址
    reg [7:0]    shout;         // 正在发的字节
    reg [3:0]    dbc;           // 数据阶段本字节已发/收几位
    reg          wel;           // 写使能锁存
    reg          wr_cmd;        // 本次是 PP/SE/BE（CS↑ 清 WEL）
    reg [AW-1:0] waddr;         // 写地址（页内回绕）
    reg [7:0]    wshift;
    reg [3:0]    wbit;

    // CS↓：开始一条命令
    always @(negedge cs_n) begin
        st <= S_CMD; bitn <= 5'd0; shin <= 32'h0; dbc <= 4'd0; wbit <= 4'd0;
        miso <= 1'b0;
    end
    // CS↑：结束；PP/SE/BE 完成后清 WEL（与真实 NOR 一致）
    always @(posedge cs_n) begin
        st <= S_IDLE;
        if (wr_cmd) wel <= 1'b0;
        wr_cmd <= 1'b0;
    end

    // ---------------- 上升沿：采 MOSI / 执行命令 ----------------
    always @(posedge sclk) if (!cs_n) begin
        if (st == S_CMD || st == S_ADDR) begin
            shin <= {shin[30:0], mosi};
            bitn <= bitn + 5'd1;
            if (bitn == 5'd7) begin
                cmd <= {shin[6:0], mosi};
                case ({shin[6:0], mosi})
                    8'h03: st <= S_ADDR;                       // READ
                    8'h06: begin wel <= 1'b1; st <= S_IDLE; end // WREN
                    8'h05: begin shout <= {6'h0, wel, 1'b0}; dbc <= 4'd0; st <= S_RDSR; end
                    8'h02: begin wr_cmd <= 1'b1; st <= S_ADDR; end   // PP
                    8'h20: begin wr_cmd <= 1'b1; st <= S_ADDR; end   // SE
                    8'hD8: begin wr_cmd <= 1'b1; st <= S_ADDR; end   // BE
                    default: $display("  [FAIL] 从机模型：不支持命令 0x%02x", {shin[6:0], mosi});
                endcase
            end
            if (bitn == 5'd31) begin
                case (cmd)
                    8'h03: begin
                        faddr <= {shin[22:0], mosi};
                        shout <= mem[{shin[22:0], mosi}];
                        dbc   <= 4'd0;
                        st    <= S_DATA;
                    end
                    8'h02: begin
                        waddr <= {shin[22:0], mosi};
                        wbit  <= 4'd0;
                        st    <= S_WDATA;
                    end
                    8'h20: begin            // SE：只擦**所在的 4 KB 扇区**

                        if (wel) begin
                            sbase = ({shin[22:0], mosi} & 24'hFFF000);
                            for (i = sbase; i < sbase + 4096 && i < SIZE; i = i + 1)
                                mem[i] <= 8'hFF;
                        end
                        wr_cmd <= 1'b0; wel <= 1'b0;
                        st <= S_IDLE;
                    end
                    8'hD8: begin            // BE：擦 64 KB 块（本模型整片 <64KB ⇒ 全擦）
                        if (wel) for (i = 0; i < SIZE; i = i + 1) mem[i] <= 8'hFF;
                        wr_cmd <= 1'b0; wel <= 1'b0;
                        st <= S_IDLE;
                    end
                    default: ;
                endcase
            end
        end
        else if (st == S_WDATA) begin
            wshift <= {wshift[6:0], mosi};
            wbit   <= wbit + 4'd1;
            if (wbit == 4'd7) begin
                // NOR：只能 1→0（mem & data）；没有 WREN 就忽略
                if (wel) mem[waddr] <= mem[waddr] & {wshift[6:0], mosi};
                // 页内回绕：低 8 位 +1（不跨 256 字节页）
                waddr <= {waddr[AW-1:8], waddr[7:0] + 8'd1};
                wbit  <= 4'd0;
            end
        end
    end

    // ---------------- 下降沿：换 MISO ----------------
    always @(negedge sclk) if (!cs_n) begin
        if (st == S_DATA) begin
            miso  <= shout[7];
            shout <= {shout[6:0], 1'b0};
            if (dbc == 4'd7) begin
                dbc   <= 4'd0;
                shout <= mem[faddr + 1'b1];
                faddr <= faddr + 1'b1;
            end
            else dbc <= dbc + 4'd1;
        end
        else if (st == S_RDSR) begin
            miso  <= shout[7];
            shout <= {shout[6:0], 1'b0};
            if (dbc == 4'd7) begin
                dbc   <= 4'd0;
                shout <= {6'h0, wel, 1'b0};       // 状态字节：bit1=WEL，bit0=WIP=0
            end
            else dbc <= dbc + 4'd1;
        end
    end

endmodule
