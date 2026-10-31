// phy_mdio_slave.v —— MDIO 从机（PHY 寄存器模型），全工程共用
module phy_mdio_slave #(
    parameter [4:0] PHY_ADDR   = 5'd1,

    parameter integer SPEED_1G = 0,
    parameter integer DEBUG_MDIO = 0
)(
    input  wire mdc,
    inout  wire mdio
);
    reg [15:0] regs[0:31];
    integer ri;
    initial begin
        for (ri = 0; ri < 32; ri = ri + 1) regs[ri] = 16'h0000;
        if (SPEED_1G) begin
            regs[0]  = 16'h1140;   // 控制: 自协商使能 + 全双工（1000M 由 reg9/10 协商）
            regs[1]  = 16'h796D;   // 状态: 100M/10M 能力 + 扩展状态 + AN 完成 + link
            regs[4]  = 16'h01E1;   // 自协商广播: 10/100 FDX+HDX
            regs[5]  = 16'h01E1;   // 对端能力
            regs[9]  = 16'h0300;   // 1000BASE-T: bit9=1000FDX, bit8=1000HDX
            regs[10] = 16'h0C00;   // 1000BASE-T 状态/能力（bit11/12）
            regs[17] = 16'h8400;
        end else begin
            regs[0] = 16'h2100;   // 控制: 100M 全双工(bit13=100M, bit8=FDX)
            regs[1] = 16'h7824;   // 状态: 100M 能力 + 自协商完成(bit5) + link up(bit2)
            regs[4] = 16'h0181;   // 自动协商广播(100M FDX)
            regs[5] = 16'h0181;   // 对端能力(100M FDX)

            regs[17] = 16'h0400;
        end
    end

    reg [5:0]  ones;
    reg        in_frame;
    reg [5:0]  bit_cnt;
    reg [31:0] frame;
    reg        op_read;
    reg [4:0]  phy_lat, reg_lat;
    reg        drive;
    reg        drive_bit;


    initial begin
        ones      = 6'd0;
        in_frame  = 1'b0;
        bit_cnt   = 6'd0;
        frame     = 32'h0;
        op_read   = 1'b0;
        phy_lat   = 5'd0;
        reg_lat   = 5'd0;
        drive     = 1'b0;
        drive_bit = 1'b0;
    end

    assign mdio = drive ? drive_bit : 1'bz;


    always @(posedge mdc) begin
        if (!in_frame) begin
            drive <= 1'b0;
            if (mdio === 1'b1) begin
                if (ones < 6'd63) ones <= ones + 6'd1;
            end else begin

                if (ones >= 6'd1) begin
                    in_frame <= 1'b1;
                    bit_cnt  <= 6'd0;
                    frame    <= {31'h00000000, 1'b0};   // ST[0]=0
                end
                ones <= 6'd0;
            end
        end else begin
            frame   <= {frame[30:0], mdio};
            bit_cnt <= bit_cnt + 6'd1;

            if (bit_cnt == 6'd13) begin
                op_read <= (frame[11:10] == 2'b10);
                phy_lat <= frame[9:5];
                reg_lat <= frame[4:0];
            end
            if (bit_cnt == 6'd31) begin
                in_frame <= 1'b0;
                if (DEBUG_MDIO)
                    $display("[mdio] frame=%h op=%b phy=%0d reg=%0d data=%h",
                             frame, frame[29:28], frame[27:23], frame[22:18], frame[15:0]);
                if (frame[29:28] == 2'b01 && frame[27:23] == PHY_ADDR)
                    regs[frame[22:18]] <= frame[15:0];
            end
        end
    end


    always @(negedge mdc) begin
        if (in_frame && op_read && phy_lat == PHY_ADDR) begin
            case (bit_cnt)
                6'd13: begin drive <= 1'b0; drive_bit <= 1'b0; end
                6'd14: begin drive <= 1'b1; drive_bit <= 1'b0; end
                6'd15: begin drive <= 1'b1; drive_bit <= regs[reg_lat][15]; end
                6'd16: begin drive <= 1'b1; drive_bit <= regs[reg_lat][14]; end
                6'd17: begin drive <= 1'b1; drive_bit <= regs[reg_lat][13]; end
                6'd18: begin drive <= 1'b1; drive_bit <= regs[reg_lat][12]; end
                6'd19: begin drive <= 1'b1; drive_bit <= regs[reg_lat][11]; end
                6'd20: begin drive <= 1'b1; drive_bit <= regs[reg_lat][10]; end
                6'd21: begin drive <= 1'b1; drive_bit <= regs[reg_lat][9];  end
                6'd22: begin drive <= 1'b1; drive_bit <= regs[reg_lat][8];  end
                6'd23: begin drive <= 1'b1; drive_bit <= regs[reg_lat][7];  end
                6'd24: begin drive <= 1'b1; drive_bit <= regs[reg_lat][6];  end
                6'd25: begin drive <= 1'b1; drive_bit <= regs[reg_lat][5];  end
                6'd26: begin drive <= 1'b1; drive_bit <= regs[reg_lat][4];  end
                6'd27: begin drive <= 1'b1; drive_bit <= regs[reg_lat][3];  end
                6'd28: begin drive <= 1'b1; drive_bit <= regs[reg_lat][2];  end
                6'd29: begin drive <= 1'b1; drive_bit <= regs[reg_lat][1];  end
                6'd30: begin drive <= 1'b1; drive_bit <= regs[reg_lat][0];  end
                default: begin drive <= 1'b0; drive_bit <= 1'b0; end
            endcase
        end else begin
            drive     <= 1'b0;
            drive_bit <= 1'b0;
        end
    end

endmodule
