// afdx_cfg.v —— VL 配置表 + 主机下发/查表接口
`timescale 1ns/1ps

module afdx_cfg #(
    parameter NENT = 512,                    // 表项数
    parameter IDXW = 9                       // log2(NENT)
) (
    input  wire         clk,
    input  wire         reset,

    // ---- CPU 侧 ----
    input  wire         wr,
    input  wire         rd,
    input  wire [3:0]   reg_sel,
    input  wire [31:0]  wdata,
    output reg  [31:0]  rdata,

    // ---- 数据面侧查表口（1 拍延迟，两个请求者共享一个读口）----
    //   TX 请求者：地址常驻（vlid_i 直接来自消息头），结果由 tx 侧自校验
    //   RX 请求者：由 rx_req_i 选通（IC/RM 在帧末那一拍申请），优先级高于 TX
    input  wire [15:0]  vlid_i,
    input  wire [15:0]  vlid_rx_i,
    input  wire         rx_req_i,
    output wire [15:0]  res_vlid_o,          // 当前结果对应的查询地址（供请求者自校验）
    output wire         hit_o,
    output wire [127:0] cfg_o
);
    localparam [3:0] R_IDX=4'd0, R_WORD=4'd1, R_LOOKUP=4'd2, R_HIT=4'd3,
                     R_RD0=4'd4, R_RD1=4'd5, R_RD2=4'd6, R_RD3=4'd7;

    // ================= 表存储：4 条 32 位通道 =================
    reg [31:0] tab0 [0:NENT-1];
    reg [31:0] tab1 [0:NENT-1];
    reg [31:0] tab2 [0:NENT-1];
    reg [31:0] tab3 [0:NENT-1];

    reg [IDXW-1:0] wr_idx;
    reg             wr_idx_bad;
    reg [1:0]      wr_wsel;


    initial $readmemh("cfgpreset0.hex", tab0);
    initial $readmemh("cfgpreset1.hex", tab1);
    initial $readmemh("cfgpreset2.hex", tab2);
    initial $readmemh("cfgpreset3.hex", tab3);

    always @(posedge clk) begin
        if (reset) begin

            wr_idx  <= {IDXW{1'b0}};
            wr_wsel <= 2'd0;
            wr_idx_bad <= 1'b0;
        end
        else if (wr && reg_sel == R_IDX) begin

            wr_idx_bad <= |wdata[31:IDXW];
            wr_idx  <= wdata[IDXW-1:0];
            wr_wsel <= 2'd0;
        end
        else if (wr && reg_sel == R_WORD && !wr_idx_bad) begin
            wr_wsel <= wr_wsel + 2'd1;       // 自动进位到下一个字
            case (wr_wsel)
                2'd0: tab0[wr_idx] <= wdata;
                2'd1: tab1[wr_idx] <= wdata;
                2'd2: tab2[wr_idx] <= wdata;
                default: tab3[wr_idx] <= wdata;
            endcase
        end
    end

    // ================= 查表（寄存读，1 拍延迟）=================
    // 优先级：CPU 自检(1 拍) > RX(帧末) > TX(常驻)
    wire        cpu_lk = wr && (reg_sel == R_LOOKUP);
    wire [15:0] lk_vlid = cpu_lk  ? wdata[15:0] :
                          rx_req_i ? vlid_rx_i  : vlid_i;
    wire [IDXW-1:0] lk_idx = lk_vlid[IDXW-1:0];      // 直接哈希

    reg [31:0] q0, q1, q2, q3;
    reg [15:0] lk_vlid_q;

    always @(posedge clk) begin
        q0        <= tab0[lk_idx];
        q1        <= tab1[lk_idx];
        q2        <= tab2[lk_idx];
        q3        <= tab3[lk_idx];
        lk_vlid_q <= lk_vlid;
    end

    wire lk_hit = (q0[15:0] == lk_vlid_q);
    wire [127:0] lk_cfg = {q3, q2, q1, q0};

    assign hit_o      = lk_hit;
    assign cfg_o      = lk_cfg;
    assign res_vlid_o = lk_vlid_q;

    // ================= CPU 读选通（锁存一次查表结果供软件自检）=================
    reg        hit_lat;
    reg [127:0] cfg_lat;
    reg        lk_pen;                      // 上一拍有 CPU 发起的查表

    always @(posedge clk) begin
        if (reset) begin
            lk_pen  <= 1'b0;
            hit_lat <= 1'b0;
            cfg_lat <= 128'h0;
        end
        else begin
            lk_pen <= cpu_lk;
            if (lk_pen) begin               // 查表结果就绪的那一拍锁存
                hit_lat <= lk_hit;
                cfg_lat <= lk_cfg;
            end
        end
    end

    always @(*) begin
        case (reg_sel)
            R_HIT:   rdata = {31'h0, hit_lat};
            R_RD0:   rdata = cfg_lat[31:0];
            R_RD1:   rdata = cfg_lat[63:32];
            R_RD2:   rdata = cfg_lat[95:64];
            R_RD3:   rdata = cfg_lat[127:96];
            default: rdata = 32'h0;
        endcase
    end

endmodule
