// afdx_axi_slave.v —— AFDX 端口的 AXI4-Lite 从机接口
`timescale 1ns/1ps

module afdx_axi_slave #(

    parameter NP  = 6,       // **接收**端口数（位图 / IRQ_EN / PCFG 等接收侧寄存器）
    parameter NPW = 3,       // 端口号位宽（地址译码用 **[NPW+5:6]**：寄存器占 [5:2]；生产 4 ⇒ [9:6]）
    parameter NPTX = 6       // **发送**端口数（发送侧越界判据；生产为 10）
) (
    input  wire        clk,
    input  wire        reset,

    // ---- AXI4-Lite 从机 ----
    input  wire [31:0] s_axi_awaddr,
    input  wire        s_axi_awvalid,
    output reg         s_axi_awready,
    input  wire [31:0] s_axi_wdata,
    input  wire [3:0]  s_axi_wstrb,
    input  wire        s_axi_wvalid,
    output reg         s_axi_wready,
    output reg  [1:0]  s_axi_bresp,
    output reg         s_axi_bvalid,
    input  wire        s_axi_bready,
    input  wire [31:0] s_axi_araddr,
    input  wire        s_axi_arvalid,
    output reg         s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // ---- 到 afdx_rx_ports 的消息窗口 ----
    output wire [NPW-1:0] rx_pi,            // 直接告诉端口层访问哪个端口（组合）
    output wire           rx_pi_en,
    output reg         px_wr,
    output reg         px_rd,
    output reg  [2:0]  px_reg,
    output reg  [31:0] px_wdata,
    input  wire [31:0] px_rdata,

    // ---- 到 afdx_rx_ports 的附加信息窗口 ----
    output reg         py_rd,
    output reg         py_wr,
    output reg  [2:0]  py_sel,
    output reg  [31:0] py_wdata,
    input  wire [15:0] py_rdata,

    // ---- 到 afdx_tx_ports ----
    output reg         tx_wr,
    output reg  [NPW-1:0] tx_pi,        // 目标端口号（TX 侧按端口寻址）
    output reg  [2:0]  tx_reg,
    output reg  [31:0] tx_wdata,
    input  wire [31:0] tx_rdata,

    // ---- 端口汇总 ----
    input  wire [NP-1:0] rx_has,          // 各端口有数据
    input  wire [NP-1:0] rx_ovf,          // 各端口溢出
    output wire          irq              // 给 CPU 的中断（有数据且该端口使能）
);

    function integer clog2i;
        input integer v;
        integer i;
        begin
            clog2i = 0;
            for (i = v - 1; i > 0; i = i >> 1) clog2i = clog2i + 1;
        end
    endfunction
    localparam integer RXW = clog2i(NP);     // NP=6 → 3

    // 端口内寄存器字偏移
    localparam R_DATA=4'd0, R_LEN=4'd1, R_PORTID=4'd2, R_STATUS=4'd3,
               R_CTRL=4'd4, R_SRCPT=4'd5, R_SIPLO=4'd6, R_SIPHI=4'd7,
               R_DIP=4'd8,  R_DPT=4'd9,   R_TXST=4'd10,
               R_NET=4'd11,
               R_PCFG=4'd12;


    localparam S_IDLE=4'd0, S_GETW=4'd7, S_PSEL=4'd1, S_SELW=4'd2, S_W1=4'd3,
               S_W2=4'd4, S_EXEC=4'd5, S_RESP=4'd6;

    reg [3:0]  st;
    reg [31:0] adr_q, wd_q;
    reg [NPW-1:0] pi_q;
    reg [3:0]  rg_q;
    reg        is_wr, glob;
    reg [31:0] rd_cap;
    reg [NP-1:0] irq_en;

    assign irq = |(rx_has & irq_en);

    wire pi_ok    = (pi_q < NPTX);
    wire pi_ok_rx = (pi_q < NP);
    assign rx_pi    = pi_q;
    assign rx_pi_en = ~glob && pi_ok_rx;   // 全局寄存器区、或接收侧越界端口 → 不选端口

    // 目标寄存器号（组合算出，好在 S_SELW 就摆好）
    wire [2:0] tgt_reg  = (rg_q == R_DATA)   ? 3'd4 :
                          (rg_q == R_LEN)    ? 3'd3 :
                          (rg_q == R_PORTID) ? 3'd5 :
                          (rg_q == R_STATUS) ? 3'd3 :
                          (rg_q == R_CTRL)   ? 3'd6 : 3'd0;
    // LEN 在**附加窗口**（psel=3），不在消息窗口 —— 别混
    wire [2:0] tgt_psel = (rg_q == R_LEN)    ? 3'd3 :
                          (rg_q == R_SRCPT)  ? 3'd4 :
                          (rg_q == R_SIPLO)  ? 3'd5 :
                          (rg_q == R_SIPHI)  ? 3'd6 :
                          (rg_q == R_NET)    ? 3'd7 :
                          (rg_q == R_PCFG)   ? 3'd1 : 3'd0;
    wire       is_py    = (rg_q == R_LEN) || (rg_q == R_SRCPT)
                       || (rg_q == R_SIPLO) || (rg_q == R_SIPHI)
                       || (rg_q == R_NET) || (rg_q == R_PCFG);


    wire [31:0] glob_rdata = (rg_q == 3'd1) ? {{(32-NP){1'b0}}, irq_en}
                                            : {{(32-2*NP){1'b0}}, rx_ovf, rx_has};
    wire        is_irqen   = (rg_q == 3'd1);

    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE;
            s_axi_awready <= 1'b0; s_axi_wready <= 1'b0;
            s_axi_bvalid  <= 1'b0; s_axi_bresp  <= 2'b00;
            s_axi_arready <= 1'b0; s_axi_rvalid <= 1'b0; s_axi_rresp <= 2'b00;
            s_axi_rdata   <= 32'h0;
            px_wr <= 1'b0; px_rd <= 1'b0; px_reg <= 3'd0; px_wdata <= 32'h0;
            py_rd <= 1'b0; py_wr <= 1'b0; py_sel <= 3'd0; py_wdata <= 32'h0;
            tx_wr <= 1'b0; tx_reg <= 3'd0; tx_wdata <= 32'h0; tx_pi <= {NPW{1'b0}};
            adr_q <= 0; wd_q <= 0; pi_q <= 0; rg_q <= 0;
            is_wr <= 1'b0; glob <= 1'b0; rd_cap <= 32'h0; irq_en <= {NP{1'b0}};
        end
        else begin
            px_wr <= 1'b0; px_rd <= 1'b0; py_rd <= 1'b0; py_wr <= 1'b0;
            tx_wr <= 1'b0; tx_pi <= pi_q;

            case (st)
                S_IDLE: begin
                    s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_arready <= 1'b0;

                    if (s_axi_awvalid) begin                       // ① 先收 AW
                        s_axi_awready <= 1'b1;
                        adr_q <= s_axi_awaddr;
                        pi_q  <= s_axi_awaddr[NPW+5:6];
                        rg_q  <= s_axi_awaddr[5:2];
                        glob  <= (s_axi_awaddr[15:8] == 8'h04);
                        is_wr <= 1'b1;
                        st    <= S_GETW;
                    end
                    else if (s_axi_arvalid) begin
                        s_axi_arready <= 1'b1;
                        adr_q <= s_axi_araddr;
                        pi_q  <= s_axi_araddr[NPW+5:6];
                        rg_q  <= s_axi_araddr[5:2];
                        glob  <= (s_axi_araddr[15:8] == 8'h04);
                        is_wr <= 1'b0;
                        st    <= S_PSEL;
                    end
                end


                S_GETW: begin
                    s_axi_awready <= 1'b0;              // AW 已经收过了，别再挂
                    s_axi_wready  <= 1'b1;              // 收 W（可与 AW 不同拍）

                    if (s_axi_wready && s_axi_wvalid) begin

                        wd_q <= { {8{s_axi_wstrb[3]}} & s_axi_wdata[31:24],
                                  {8{s_axi_wstrb[2]}} & s_axi_wdata[23:16],
                                  {8{s_axi_wstrb[1]}} & s_axi_wdata[15: 8],
                                  {8{s_axi_wstrb[0]}} & s_axi_wdata[ 7: 0] };
                        s_axi_wready <= 1'b0;
                        st   <= S_PSEL;                 // 收齐了才进入执行序列
                    end
                end


                S_PSEL: begin
                    s_axi_awready <= 1'b0; s_axi_wready <= 1'b0; s_axi_arready <= 1'b0;
                    if (!glob && pi_ok_rx) begin
                        px_reg   <= 3'd7;

                        px_wdata <= {{(31-RXW){1'b0}}, 1'b0, pi_q[RXW-1:0]};
                        px_wr    <= 1'b1;                  // 与上面同拍
                    end
                    st <= S_SELW;
                end

                // 选端口写已发生（上一拍）；现在把目标寄存器号/psel 摆好留给后面几拍
                S_SELW: begin
                    if (!glob) begin
                        px_reg <= tgt_reg;
                        py_sel <= tgt_psel;

                        tx_reg <= (rg_q == R_TXST) ? 3'd5 : 3'd0;
                    end
                    st <= S_W1;
                end


                S_W1: begin
                    if (is_wr && !glob && pi_ok_rx && (rg_q == R_PCFG)) begin
                        py_sel   <= 3'd0;
                        py_wdata <= {{(32-RXW){1'b0}}, pi_q[RXW-1:0]};
                        py_wr    <= 1'b1;
                    end
                    st <= S_W2;
                end
                S_W2: begin
                    if (is_wr && !glob && pi_ok_rx && (rg_q == R_PCFG)) begin
                        py_sel   <= 3'd1;
                        py_wdata <= wd_q;
                        py_wr    <= 1'b1;      // 这一拍末尾端口层采样 → 配置真的落地
                    end
                    st <= S_EXEC;
                end

                // 执行 + **当拍抓读数据**
                S_EXEC: begin
                    if (glob) begin

                        if (is_wr && is_irqen) irq_en <= wd_q[NP-1:0];    // 0x404 IRQ_EN
                        if (!is_wr)            rd_cap <= glob_rdata;
                    end
                    else if (is_wr) begin

                        case (rg_q)
                            R_DATA: if (pi_ok) begin tx_wr <= 1'b1; tx_reg <= 3'd0; tx_wdata <= wd_q; end
                            R_LEN:  if (pi_ok) begin tx_wr <= 1'b1; tx_reg <= 3'd1; tx_wdata <= wd_q; end
                            R_CTRL: begin
                                // bit0 = 释放接收（接收侧）； bit1 = 提交发送（发送侧）
                                if (wd_q[0] && pi_ok_rx) begin px_wr <= 1'b1; px_reg <= 3'd6; px_wdata <= wd_q; end

                                if (wd_q[1] && pi_ok)    begin tx_wr <= 1'b1; tx_reg <= 3'd2; tx_wdata <= 32'h1; end
                            end
                            R_DIP:  if (pi_ok) begin tx_wr <= 1'b1; tx_reg <= 3'd3; tx_wdata <= wd_q; end
                            R_DPT:  if (pi_ok) begin tx_wr <= 1'b1; tx_reg <= 3'd4; tx_wdata <= wd_q; end

                            R_PCFG: ;
                            default: ;
                        endcase
                    end
                    else if (is_py) begin
                        if (pi_ok_rx) begin
                            py_rd  <= 1'b1;               // py_sel 已在 S_SELW 摆好
                            rd_cap <= {16'h0, py_rdata};
                        end
                        else rd_cap <= 32'h0;
                    end
                    else if (rg_q == R_TXST && pi_ok) begin

                        rd_cap <= tx_rdata;               // 发送状态（按端口索引，见上）
                    end
                    else if (rg_q == R_TXST) begin
                        rd_cap <= 32'h0;                  // 端口号越界 ⇒ 返回确定的 0
                    end
                    else begin
                        if (pi_ok_rx) begin
                            px_rd <= 1'b1;                // px_reg 已在 S_SELW 摆好
                            if (rg_q == R_STATUS)
                                // 端口层 STATUS 是 bit2=有数据 bit3=溢出 → 重排成 bit0/bit1
                                rd_cap <= {30'h0, px_rdata[3], px_rdata[2]};
                            else
                                rd_cap <= px_rdata;
                        end
                        else rd_cap <= 32'h0;
                    end
                    st <= S_RESP;
                end

                // 响应（保持到 READY）
                default: begin
                    if (is_wr) begin
                        s_axi_bvalid <= 1'b1; s_axi_bresp <= 2'b00;
                        if (s_axi_bvalid && s_axi_bready) begin
                            s_axi_bvalid <= 1'b0; st <= S_IDLE;
                        end
                    end
                    else begin
                        s_axi_rvalid <= 1'b1; s_axi_rresp <= 2'b00;
                        s_axi_rdata  <= rd_cap;
                        if (s_axi_rvalid && s_axi_rready) begin
                            s_axi_rvalid <= 1'b0; st <= S_IDLE;
                        end
                    end
                end
            endcase
        end
    end

endmodule
