// axi_lite_master_bridge.v —— darkriscv 数据总线 → AXI4-Lite 主机桥
`timescale 1ns/1ps

module axi_lite_master_bridge (
    input  wire        clk,
    input  wire        reset,

    // ---- darkriscv 数据总线（核侧，本模块是"从设备"）----
    input  wire        DDREQ,
    input  wire [31:0] DADDR,
    input  wire [31:0] DATAO,
    input  wire [3:0]  DBE,
    input  wire [2:0]  DLEN,
    input  wire        DWR,            // 1=写(store), 0=读(load)
    input  wire        DRD,
    output reg         DDACK,
    output reg  [31:0] DATAI,

    // ---- AXI4-Lite 主机 ----
    output wire [31:0] m_axi_awaddr,
    output wire        m_axi_awvalid,
    input  wire        m_axi_awready,
    output wire [31:0] m_axi_wdata,
    output wire [3:0]  m_axi_wstrb,
    output wire        m_axi_wvalid,
    input  wire        m_axi_wready,
    input  wire [1:0]  m_axi_bresp,
    input  wire        m_axi_bvalid,
    output wire        m_axi_bready,
    output wire [31:0] m_axi_araddr,
    output wire        m_axi_arvalid,
    input  wire        m_axi_arready,
    input  wire [31:0] m_axi_rdata,
    input  wire [1:0]  m_axi_rresp,
    input  wire        m_axi_rvalid,
    output wire        m_axi_rready
);

    localparam S_IDLE = 3'd0, S_AW = 3'd1, S_W = 3'd2, S_B = 3'd3, S_AR = 3'd4, S_R = 3'd5,
               S_GAP  = 3'd6;

    reg [2:0]  st;
    reg [31:0] addr_q, wdata_q;
    reg [3:0]  wstrb_q;
    reg        ack_q;                  // "本笔请求已处理"锁存，防止同一笔 DDREQ 触发两次

    wire den   = DDREQ & ~ack_q;       // 新事务起点
    wire is_wr = DWR & (|DBE);         // 有字节使能的写
    wire noop  = DWR & ~(|DBE);        // 无字节使能的写 → 空操作

    assign m_axi_awaddr  = addr_q;
    assign m_axi_awvalid = (st == S_AW);
    assign m_axi_wdata   = wdata_q;
    assign m_axi_wstrb   = wstrb_q;
    assign m_axi_wvalid  = (st == S_W);
    assign m_axi_bready  = (st == S_B);
    assign m_axi_araddr  = addr_q;
    assign m_axi_arvalid = (st == S_AR);
    assign m_axi_rready  = (st == S_R);

    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; DDACK <= 1'b0; DATAI <= 32'h0; ack_q <= 1'b0;
            addr_q <= 32'h0; wdata_q <= 32'h0; wstrb_q <= 4'h0;
        end
        else begin
            DDACK <= 1'b0;                            // 默认低，只在完成那一拍抬高


            case (st)
                S_IDLE: if (den) begin
                    addr_q  <= DADDR;
                    wdata_q <= DATAO;
                    wstrb_q <= DBE;
                    if (noop) begin
                        DDACK <= 1'b1; ack_q <= 1'b1; st <= S_GAP;
                    end
                    else if (is_wr) st <= S_AW;
                    else            st <= S_AR;
                end
                S_AW: if (m_axi_awready) st <= S_W;
                S_W:  if (m_axi_wready)  st <= S_B;
                S_B:  if (m_axi_bvalid) begin DDACK <= 1'b1; ack_q <= 1'b1; st <= S_GAP; end
                S_AR: if (m_axi_arready)      st <= S_R;
                S_R:  if (m_axi_rvalid) begin
                          DATAI <= m_axi_rdata;        // 与 DDACK 同拍寄存 → 同拍有效
                          DDACK <= 1'b1; ack_q <= 1'b1; st <= S_GAP;
                      end

                S_GAP: begin ack_q <= 1'b0; st <= S_IDLE; end
                default: st <= S_IDLE;
            endcase
        end
    end

endmodule
