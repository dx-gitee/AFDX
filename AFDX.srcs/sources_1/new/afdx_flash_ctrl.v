// afdx_flash_ctrl.v —— SPI NOR Flash 控制器（读 + 写，供配置/文件持久化）
`timescale 1ns/1ps

module afdx_flash_ctrl #(

    parameter DIV = 40         // sclk 半周期 = DIV 个 clk（125 MHz ÷ 2÷40 ≈ 1.56 MHz）
)(
    input  wire        clk,
    input  wire        reset,

    input  wire        wr,
    input  wire        rd,
    input  wire [3:0]  reg_sel,
    input  wire [31:0] wdata,
    output reg  [31:0] rdata,

    output wire        sclk,
    output wire        mosi,
    input  wire        miso,
    output reg         cs_n
);
    localparam [3:0] R_ADDR = 4'd0, R_DATA = 4'd1, R_STAT = 4'd2, R_STOP = 4'd3,
                     R_WADDR= 4'd4, R_WDATA= 4'd5, R_WLEN = 4'd6, R_WCMD = 4'd7;

    localparam [3:0] S_IDLE  = 4'd0,
                     S_CMD   = 4'd1,   // 发命令（读:0x03+addr / 写:见 phase）
                     S_RDATA = 4'd2,   // 连续接收数据（读）
                     S_WDATA = 4'd3,   // 发写缓冲的数据（PP）
                     S_RSTAT = 4'd4,   // 收状态字节（RDSR）
                     S_GAP   = 4'd5;   // CS 抬高隔离两段事务
    reg [3:0] st;

    localparam [1:0] PH_READ = 2'd0, PH_WREN = 2'd1, PH_MAIN = 2'd2, PH_RDSR = 2'd3;
    reg [1:0]  phase;

    reg [31:0] cmd_sh;
    reg [5:0]  bitn;
    reg [5:0]  cmd_nbits;
    reg [3:0]  rxbit;
    reg [7:0]  rx_sh;

    reg [7:0]  op_cmd;
    reg [23:0] op_addr;
    reg [7:0]  op_n;
    reg [2:0]  gap_next;       // 1=主命令 2=轮询 3=回 IDLE
    reg        auto_poll;
    reg [19:0] poll_tmo;

    reg [7:0]  wbuf [0:63];
    reg [6:0]  wcnt;
    reg [6:0]  wbi;
    reg [7:0]  wshift;
    reg [3:0]  wbit;
    reg        wbuf_ovf;

    reg        busy_r, done_r, wip_r, wel_r, op_err;

    reg [6:0]  hc;                 // 0..(2*DIV-1)；DIV=40 时最大 79
    wire       at_rise = (hc == DIV[6:0]);
    wire       at_fall = (hc == (2*DIV-1));

    reg [7:0]  fifo [0:15];
    reg [3:0]  fw, fr;
    reg [4:0]  fcnt;
    reg        ovf;

    reg        pend_st;
    reg [23:0] pend_ad;

    assign sclk = (hc >= DIV) && !cs_n;
    assign mosi = (st == S_WDATA) ? wshift[7] : cmd_sh[31];

    wire fempty = (fcnt == 5'd0);
    wire ffull  = (fcnt == 5'd16);
    wire pop_w  = rd && (reg_sel == R_DATA) && !fempty;
    wire push_w = (!cs_n) && at_fall && (rxbit == 4'd7) && (st == S_RDATA) && (!ffull || pop_w);
    // PP 实际要发的字节数（0 或超过已压入数 ⇒ 用已压入数）
    wire [7:0] wlen_m = (wdata[7:0] == 8'd0 || wdata[7:0] > {1'b0, wcnt}) ? {1'b0, wcnt} : wdata[7:0];

    // =====================================================================
    // 主时序块（所有 reg 只在这里赋值）
    // =====================================================================
    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; cs_n <= 1'b1; hc <= 7'd0; bitn <= 6'd0; cmd_nbits <= 6'd32;
            cmd_sh <= 32'h0; rx_sh <= 8'h0; rxbit <= 4'd0;
            phase <= PH_READ; op_cmd <= 8'h0; op_addr <= 24'h0; op_n <= 8'h0;
            gap_next <= 3'd0; auto_poll <= 1'b0; poll_tmo <= 20'h0;
            fw <= 4'd0; fr <= 4'd0; fcnt <= 5'd0; ovf <= 1'b0;
            wcnt <= 7'd0; wbi <= 7'd0; wshift <= 8'h0; wbit <= 4'd0; wbuf_ovf <= 1'b0;
            busy_r <= 1'b0; done_r <= 1'b0; wip_r <= 1'b0; wel_r <= 1'b0; op_err <= 1'b0;
            pend_st <= 1'b0; pend_ad <= 24'h0;
        end
        else begin
            hc <= at_fall ? 7'd0 : (hc + 7'd1);

            // ---------- sclk 上升沿采 MISO ----------
            if (!cs_n && at_rise) rx_sh <= {rx_sh[6:0], miso};

            // ---------- 下降沿 ----------
            if (!cs_n && at_fall) begin
                // 1) 命令/地址
                if (st == S_CMD) begin
                    cmd_sh <= {cmd_sh[30:0], 1'b0};
                    bitn   <= bitn + 6'd1;
                    if (bitn + 6'd1 == cmd_nbits) begin
                        bitn  <= 6'd0;
                        rxbit <= 4'd0;
                        case (phase)
                            PH_READ: begin
                                st <= S_RDATA;
                                fcnt <= 5'd0; fw <= 4'd0; fr <= 4'd0;
                            end
                            PH_WREN: begin
                                cs_n <= 1'b1;
                                st   <= S_GAP;
                                // 纯 WREN ⇒ 直接完成；否则去发主命令
                                gap_next <= (op_cmd == 8'h0) ? 3'd3 : 3'd1;
                                if (op_cmd == 8'h0) done_r <= 1'b1;
                            end
                            PH_MAIN: begin
                                if (op_cmd == 8'h02) begin
                                    wbi <= 7'd0; wbit <= 4'd0;
                                    wshift <= wbuf[0];
                                    st <= S_WDATA;
                                end
                                else begin
                                    cs_n <= 1'b1;
                                    st   <= S_GAP;
                                    gap_next <= 3'd2;
                                end
                            end
                            default: st <= S_RSTAT;          // PH_RDSR
                        endcase
                    end
                end
                // 2) 读数据 → FIFO
                else if (st == S_RDATA) begin
                    if (rxbit == 4'd7) begin
                        if (!ffull || pop_w) begin
                            fifo[fw] <= rx_sh; fw <= fw + 4'd1;
                            fcnt <= fcnt + 5'd1 - (pop_w ? 5'd1 : 5'd0);
                        end
                        else ovf <= 1'b1;
                        rxbit <= 4'd0;
                    end
                    else rxbit <= rxbit + 4'd1;
                end
                // 3) 发写数据
                else if (st == S_WDATA) begin
                    wshift <= {wshift[6:0], 1'b0};
                    if (wbit == 4'd7) begin
                        wbit <= 4'd0;
                        if ((wbi + 7'd1 >= op_n) || (op_n == 8'd0)) begin
                            cs_n <= 1'b1;
                            st   <= S_GAP;
                            gap_next <= 3'd2;
                        end
                        else begin
                            wbi <= wbi + 7'd1;
                            wshift <= wbuf[wbi + 7'd1];
                        end
                    end
                    else wbit <= wbit + 4'd1;
                end
                // 4) 收状态字节
                else if (st == S_RSTAT) begin
                    if (rxbit == 4'd7) begin
                        wip_r <= rx_sh[0];
                        wel_r <= rx_sh[1];
                        rxbit <= 4'd0;
                        cs_n  <= 1'b1;
                        st    <= S_GAP;
                        if (auto_poll && rx_sh[0]) begin
                            if (poll_tmo == 20'h0) begin
                                op_err <= 1'b1; auto_poll <= 1'b0; gap_next <= 3'd3;
                            end
                            else begin
                                poll_tmo <= poll_tmo - 20'd1;
                                gap_next <= 3'd2;
                            end
                        end
                        else begin
                            done_r    <= 1'b1;
                            busy_r    <= 1'b0;
                            auto_poll <= 1'b0;
                            gap_next  <= 3'd3;
                        end
                    end
                    else rxbit <= rxbit + 4'd1;
                end
            end

            else if (st == S_GAP && at_fall) begin
                case (gap_next)
                    3'd1: begin
                        phase <= PH_MAIN;
                        cmd_sh <= {op_cmd, op_addr};         // 8 位命令 + 24 位地址
                        cmd_nbits <= 6'd32;
                        bitn <= 6'd0; rxbit <= 4'd0;
                        cs_n <= 1'b0; hc <= 7'd0; st <= S_CMD;
                    end
                    3'd2: begin
                        phase <= PH_RDSR;
                        cmd_sh <= {8'h05, 24'h0};
                        cmd_nbits <= 6'd8;
                        bitn <= 6'd0; rxbit <= 4'd0;
                        auto_poll <= 1'b1;
                        // 首次轮询才装超时；后续轮询保留剩余值（否则永远超不了时）
                        if (poll_tmo == 20'h0) poll_tmo <= 20'h3FFF;
                        cs_n <= 1'b0; hc <= 7'd0; st <= S_CMD;
                    end
                    default: begin
                        st <= S_IDLE; busy_r <= 1'b0; done_r <= 1'b1;
                    end
                endcase
            end

            // ---------- CPU 读 F_DATA 即弹出 ----------
            if (pop_w) begin
                fr <= fr + 4'd1;
                if (!push_w) fcnt <= fcnt - 5'd1;
            end


            if (pend_st) begin
                pend_st <= 1'b0;
                phase   <= PH_READ;
                cmd_sh  <= {8'h03, pend_ad};
                cmd_nbits <= 6'd32;
                bitn    <= 6'd0; rxbit <= 4'd0;
                fcnt    <= 5'd0; fw <= 4'd0; fr <= 4'd0; ovf <= 1'b0;
                cs_n    <= 1'b0; hc <= 7'd0; st <= S_CMD;
            end


            if (wr) begin
                if (reg_sel == R_STOP) begin
                    cs_n <= 1'b1; st <= S_IDLE; hc <= 7'd0;
                    fcnt <= 5'd0; fw <= 4'd0; fr <= 4'd0;
                    wcnt <= 7'd0; wbi <= 7'd0;
                    busy_r <= 1'b0; auto_poll <= 1'b0; done_r <= 1'b0; op_err <= 1'b0;
                end

                else if (reg_sel == R_ADDR &&
                         (st == S_IDLE || st == S_RDATA || (st == S_CMD && phase == PH_READ))) begin
                    if (!cs_n) begin
                        cs_n <= 1'b1; st <= S_IDLE; hc <= 7'd0;
                        pend_st <= 1'b1; pend_ad <= wdata[23:0];
                    end
                    else begin
                        phase   <= PH_READ;
                        cmd_sh  <= {8'h03, wdata[23:0]};
                        cmd_nbits <= 6'd32;
                        bitn <= 6'd0; rxbit <= 4'd0;
                        fcnt <= 5'd0; fw <= 4'd0; fr <= 4'd0; ovf <= 1'b0;
                        cs_n <= 1'b0; hc <= 7'd0; st <= S_CMD;
                    end
                end
                else if (st == S_IDLE) begin
                    case (reg_sel)
                        R_WADDR: begin
                            op_addr <= wdata[23:0];
                            wcnt <= 7'd0; wbuf_ovf <= 1'b0;
                        end
                        R_WDATA: begin
                            if (wcnt < 7'd64) begin
                                wbuf[wcnt] <= wdata[7:0];
                                wcnt <= wcnt + 7'd1;
                            end
                            else wbuf_ovf <= 1'b1;
                        end
                        R_WLEN: begin
                            if (wcnt != 7'd0) begin

                                if ({1'b0, wlen_m} > (9'd256 - {1'b0, op_addr[7:0]})) begin
                                    op_err <= 1'b1; done_r <= 1'b1; busy_r <= 1'b0;
                                end
                                else begin
                                    op_cmd  <= 8'h02;
                                    op_n <= wlen_m;
                                    busy_r <= 1'b1; done_r <= 1'b0; op_err <= 1'b0; poll_tmo <= 20'h0;
                                    phase <= PH_WREN;
                                    cmd_sh <= {8'h06, 24'h0}; cmd_nbits <= 6'd8;
                                    bitn <= 6'd0; rxbit <= 4'd0;
                                    cs_n <= 1'b0; hc <= 7'd0; st <= S_CMD;
                                end
                            end
                        end
                        R_WCMD: begin
                            case (wdata[3:0])
                                4'd1, 4'd3, 4'd4: begin
                                    op_cmd <= (wdata[3:0] == 4'd1) ? 8'h00 :
                                              (wdata[3:0] == 4'd3) ? 8'h20 : 8'hD8;
                                    op_n <= 8'h0;
                                    busy_r <= 1'b1; done_r <= 1'b0; op_err <= 1'b0; poll_tmo <= 20'h0;
                                    phase <= PH_WREN;
                                    cmd_sh <= {8'h06, 24'h0}; cmd_nbits <= 6'd8;
                                    bitn <= 6'd0; rxbit <= 4'd0;
                                    cs_n <= 1'b0; hc <= 7'd0; st <= S_CMD;
                                end
                                4'd5: begin
                                    op_cmd <= 8'h05;
                                    auto_poll <= 1'b0;
                                    busy_r <= 1'b1; done_r <= 1'b0; op_err <= 1'b0; poll_tmo <= 20'h0;
                                    phase <= PH_RDSR;
                                    cmd_sh <= {8'h05, 24'h0}; cmd_nbits <= 6'd8;
                                    bitn <= 6'd0; rxbit <= 4'd0;
                                    cs_n <= 1'b0; hc <= 7'd0; st <= S_CMD;
                                end
                                default: ;
                            endcase
                        end
                        default: ;
                    endcase
                end

                else if (reg_sel >= R_WADDR) begin
                    op_err <= 1'b1;
                end
            end
        end
    end

    // ---------- CPU 读 ----------
    always @(*) begin
        case (reg_sel)
            R_DATA: rdata = fempty ? 32'h0 : {24'h0, fifo[fr]};
            R_STAT: rdata = {17'h0, op_err, wcnt, wbuf_ovf, done_r, wel_r, wip_r,
                             ovf, !fempty, (busy_r | (st != S_IDLE))};
            default: rdata = 32'h0;
        endcase
    end
endmodule
