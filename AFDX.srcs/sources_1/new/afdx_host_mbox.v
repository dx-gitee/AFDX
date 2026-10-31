// afdx_host_mbox.v —— 外部主机 ↔ RISC-V 的 AXI-Lite 主机邮箱
`timescale 1ns/1ps

module afdx_host_mbox #(
    parameter integer AW = 11          // 每条缓冲的字数 = 2^AW（11 → 2048 字 = 8 KB）
) (
    input  wire        clk,
    input  wire        reset,          // 高有效

    // ---------------- 主机侧：AXI-Lite 从机 ----------------
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
    output wire        s_axi_arready,
    output reg  [31:0] s_axi_rdata,
    output reg  [1:0]  s_axi_rresp,
    output reg         s_axi_rvalid,
    input  wire        s_axi_rready,

    // ---------------- CPU 侧：直连窗口 ----------------
    input  wire        cpu_wr,         // 1 拍写选通
    input  wire        cpu_rd,         // 1 拍读选通（SoC 的 PR_CAP）
    input  wire [2:0]  cpu_reg,        // DADDR[4:2]
    input  wire [31:0] cpu_wdata,
    output reg  [31:0] cpu_rdata,

    // ---------------- 中断 ----------------
    output wire        cpu_irq,        // 有主机消息（电平；固件可轮询）
    output wire        host_irq        // 有 CPU 消息（电平；主机可轮询）
);
    localparam integer NW = (1 << AW);

    // ---------------- 状态 ----------------
    reg        in_valid, out_valid;

    reg [19:0] in_info, out_info;      // {port[3:0], len[15:0]}
    reg [AW-1:0] in_widx, in_ridx;
    reg [AW-1:0] out_widx, out_ridx;
    reg        host_irq_en;
    reg        host_en;                // H_CTRL.bit2：主机数据面使能（CPU 侧可读）

    // ---------------- RAM（各一条写 + 一条寄存读） ----------------
    reg [31:0] in_mem  [0:NW-1];
    reg [31:0] out_mem [0:NW-1];
    reg [31:0] in_rdata_q, out_rdata_q;

    reg        in_we, out_we;
    reg [AW-1:0] in_waddr, out_waddr;
    reg [31:0] in_wdata, out_wdata;


    always @(posedge clk) begin
        if (in_we) in_mem[in_waddr] <= in_wdata;
        in_rdata_q <= in_mem[in_ridx];       // 读地址是纯寄存器；寄存读
    end
    always @(posedge clk) begin
        if (out_we) out_mem[out_waddr] <= out_wdata;
        out_rdata_q <= out_mem[out_ridx];
    end

    // ---------------- 主机 AXI-Lite 从机：一次一笔事务 ----------------
    localparam S_IDLE = 2'd0, S_WR = 2'd1, S_RD = 2'd2;
    reg [1:0]  st;
    reg        aw_hs, w_hs;
    reg [31:0] awaddr_q, wdata_q, araddr_q;

    // 写优先、一次一笔：AW/W 可以同拍或分开到；任一有效时就不收 AR（避免两笔交错）
    wire [31:0] aw_eff  = aw_hs ? awaddr_q : s_axi_awaddr;
    wire [31:0] w_eff   = w_hs  ? wdata_q  : s_axi_wdata;

    wire        aw_done = aw_hs | (s_axi_awvalid & s_axi_awready);
    wire        w_done  = w_hs  | (s_axi_wvalid  & s_axi_wready);
    wire        wr_go   = aw_done & w_done;


    assign s_axi_arready = ~reset && (st == S_IDLE) &&
                           ~(aw_hs | w_hs | s_axi_awvalid | s_axi_wvalid |
                             s_axi_rvalid | s_axi_bvalid);
    wire [5:0]  wreg    = aw_eff[7:2];
    wire [5:0]  rreg    = araddr_q[7:2];
    wire [31:0] host_stat = {30'h0, out_valid, in_valid};

    // CPU 侧读回（组合）
    always @(*) begin
        case (cpu_reg)
            3'd0: cpu_rdata = {30'h0, out_valid, in_valid};
            3'd1: cpu_rdata = {12'h0, in_info};   // {port[3:0], len[15:0]} = 20 位
            3'd2: cpu_rdata = in_rdata_q;              // M_IN_DATA
            3'd7: cpu_rdata = {30'h0, host_en, out_valid}; // M_OUT_STAT: bit1=host_en
            default: cpu_rdata = 32'h0;
        endcase
    end

    // 主机侧读回（组合；H_OUT_DATA 用 RAM 的寄存读）
    reg [31:0] hrdata;
    always @(*) begin
        case (rreg)
            6'd0: hrdata = 32'h484F5354;               // H_ID
            6'd1: hrdata = host_stat;                  // H_STATUS
            6'd6: hrdata = {12'h0, out_info};          // H_OUT_INFO (0x18)
            6'd7: hrdata = out_rdata_q;                // H_OUT_DATA (0x1C)
            default: hrdata = 32'h0;
        endcase
    end

    assign cpu_irq  = in_valid;
    assign host_irq = out_valid & host_irq_en;

    // ---------------- 所有状态更新（单一 always，避免多驱动） ----------------
    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE;
            s_axi_awready <= 0; s_axi_wready <= 0;
            s_axi_bvalid <= 0; s_axi_rvalid <= 0; s_axi_bresp <= 0; s_axi_rresp <= 0;
            s_axi_bvalid <= 0; s_axi_rdata <= 0;
            aw_hs <= 0; w_hs <= 0; awaddr_q <= 0; wdata_q <= 0; araddr_q <= 0;
            in_valid <= 0; out_valid <= 0;
            in_info <= 0; out_info <= 0;
            in_widx <= 0; in_ridx <= 0; out_widx <= 0; out_ridx <= 0;
            host_irq_en <= 1'b1;
            host_en <= 1'b0;
            in_we <= 0; out_we <= 0; in_waddr <= 0; out_waddr <= 0;
            in_wdata <= 0; out_wdata <= 0;
        end
        else begin
            in_we  <= 1'b0;
            out_we <= 1'b0;

            // ================= CPU 侧窗口 =================
            // 写（1 拍）
            if (cpu_wr) begin
                case (cpu_reg)
                    3'd3: begin                       // M_IN_ACK
                        in_valid <= 1'b0; in_ridx <= 0; in_widx <= 0;
                    end
                    3'd4: begin                       // M_OUT_INFO
                        out_info <= cpu_wdata[19:0]; out_widx <= 0;
                    end
                    3'd5: begin                       // M_OUT_DATA
                        out_waddr <= out_widx; out_wdata <= cpu_wdata; out_we <= 1'b1;
                        out_widx <= out_widx + 1'b1;
                    end
                    3'd6: out_valid <= 1'b1;          // M_OUT_COMMIT
                    default: ;
                endcase
            end
            // 读指针（读选通那拍 +1；数据由 in_rdata_q 提供）
            if (cpu_rd && (cpu_reg == 3'd2)) in_ridx <= in_ridx + 1'b1;

            // ================= 主机 AXI-Lite 从机 =================
            case (st)
                S_IDLE: begin
                    // 写优先；AW 与 W 可以同拍或分开到（AXI 规范要求独立握手）。
                    // 有响应未收走（rvalid/bvalid）时不再接新事务，避免覆盖。
                    s_axi_awready <= ~aw_hs & ~s_axi_rvalid & ~s_axi_bvalid;
                    s_axi_wready  <= ~w_hs  & ~s_axi_rvalid & ~s_axi_bvalid;
                    if (wr_go) begin
                        // ---- 执行写（用 aw_eff/w_eff：已锁存的或本拍输入的）----
                        case (wreg)
                            6'd2: begin                                  // H_CTRL 0x08
                                host_irq_en <= w_eff[0];
                                host_en     <= w_eff[2];
                                if (w_eff[1]) begin                      // 软复位
                                    in_valid <= 0; out_valid <= 0;
                                    in_widx <= 0; in_ridx <= 0;
                                    out_widx <= 0; out_ridx <= 0;
                                end
                            end
                            6'd3: begin                                  // H_IN_INFO 0x0C
                                in_info <= w_eff[19:0];
                                in_widx <= 0;
                            end
                            6'd4: begin                                  // H_IN_DATA 0x10
                                in_waddr <= in_widx;
                                in_wdata <= w_eff;
                                in_we    <= 1'b1;
                                in_widx  <= in_widx + 1'b1;
                            end
                            6'd5: in_valid <= 1'b1;                      // H_IN_SET 0x14
                            6'd8: begin                                  // H_OUT_ACK 0x20

                                out_valid <= 1'b0; out_ridx <= 0;
                            end
                            default: ;
                        endcase
                        aw_hs <= 1'b0; w_hs <= 1'b0;
                        s_axi_bvalid <= 1'b1; s_axi_bresp <= 2'b00;
                        st <= S_WR;
                    end
                    else begin
                        // 未凑齐：把已经到的那一边锁存
                        if (s_axi_awvalid && s_axi_awready) begin
                            aw_hs <= 1'b1; awaddr_q <= s_axi_awaddr;
                        end
                        if (s_axi_wvalid && s_axi_wready) begin
                            w_hs <= 1'b1; wdata_q <= s_axi_wdata;
                        end
                        // 读：只有在完全没有写活动时才收（写优先，避免事务交错）
                        if (s_axi_arvalid && s_axi_arready) begin
                            araddr_q <= s_axi_araddr;
                            st <= S_RD;
                        end
                    end
                end

                S_WR: begin
                    if (s_axi_bvalid && s_axi_bready) begin
                        s_axi_bvalid <= 1'b0;
                        st <= S_IDLE;
                    end
                end

                S_RD: begin
                    // 数据在地址锁存后的这一拍取（RAM 寄存读已就绪；寄存器读回是组合）
                    s_axi_rdata  <= hrdata;
                    s_axi_rresp  <= 2'b00;
                    s_axi_rvalid <= 1'b1;
                    // H_OUT_DATA (0x1C) 读一次，指针 +1
                    if (rreg == 6'd7) out_ridx <= out_ridx + 1'b1;
                    st <= S_IDLE;
                end

                default: st <= S_IDLE;
            endcase

            // RVALID 握手（与状态机分开写，避免 S_RD 与 S_IDLE 的竞争）
            if (s_axi_rvalid && s_axi_rready) s_axi_rvalid <= 1'b0;
        end
    end

endmodule
