// afdx_host_axis.v —— AXI4-Stream（XDMA 用户侧）↔ 现有主机邮箱（AXI-Lite）
`timescale 1ns/1ps

module afdx_host_axis #(
    parameter integer DATA_WIDTH = 256,             // 64 / 128 / 256（XDMA 常用 256）
    parameter integer KEEP_WIDTH = DATA_WIDTH/8
) (
    input  wire                    clk,
    input  wire                    reset,           // 高有效

    // ---------------- H2C：主机 → FPGA ----------------
    input  wire [DATA_WIDTH-1:0]   m_axis_h2c_tdata,
    input  wire [KEEP_WIDTH-1:0]   m_axis_h2c_tkeep,
    input  wire                    m_axis_h2c_tlast,
    input  wire                    m_axis_h2c_tvalid,
    output wire                    m_axis_h2c_tready,

    // ---------------- C2H：FPGA → 主机 ----------------
    output reg  [DATA_WIDTH-1:0]   s_axis_c2h_tdata,
    output reg  [KEEP_WIDTH-1:0]   s_axis_c2h_tkeep,
    output reg                     s_axis_c2h_tlast,
    output reg                     s_axis_c2h_tvalid,
    input  wire                    s_axis_c2h_tready,

    // ---------------- AXI-Lite master → afdx_host_mbox ----------------
    output reg  [31:0]             m_axil_awaddr,
    output reg                     m_axil_awvalid,
    input  wire                    m_axil_awready,
    output reg  [31:0]             m_axil_wdata,
    output reg  [3:0]              m_axil_wstrb,
    output reg                     m_axil_wvalid,
    input  wire                    m_axil_wready,
    input  wire [1:0]              m_axil_bresp,
    input  wire                    m_axil_bvalid,
    output reg                     m_axil_bready,
    output reg  [31:0]             m_axil_araddr,
    output reg                     m_axil_arvalid,
    input  wire                    m_axil_arready,
    input  wire [31:0]             m_axil_rdata,
    input  wire [1:0]              m_axil_rresp,
    input  wire                    m_axil_rvalid,
    output reg                     m_axil_rready,

    // ---------------- 状态 ----------------
    output wire                    axis_error        // 非法消息/长度不符：**粘滞**（reset 清）
);
    // ================= 常量 =================
    localparam integer BYTES_PER_BEAT = DATA_WIDTH/8;
    localparam integer WPB            = DATA_WIDTH/32;   // 每拍 32 位字数
    localparam [5:0]   BEAT_BYTES     = BYTES_PER_BEAT;
    localparam integer BBL            = (DATA_WIDTH == 64) ? 3 :
                                        (DATA_WIDTH == 128) ? 4 : 5;

    localparam [5:0] REG_ID       = 6'd0,
                     REG_STATUS   = 6'd1,
                     REG_CTRL     = 6'd2,
                     REG_IN_INFO  = 6'd3,
                     REG_IN_DATA  = 6'd4,
                     REG_IN_SET   = 6'd5,
                     REG_OUT_INFO = 6'd6,
                     REG_OUT_DATA = 6'd7,
                     REG_OUT_ACK  = 6'd8;

    localparam [31:0] MAX_LEN = 32'd1471;

    // ================= AXI-Lite master（一次一笔；req/ack 电平握手） =================
    reg [2:0]  ast;
    localparam [2:0] A_IDLE=3'd0, A_WR=3'd1, A_B=3'd2, A_RD=3'd3, A_R=3'd4;

    reg        ax_req;                 // 由仲裁器给出（H2C 优先）
    reg        ax_wr;
    reg [31:0] ax_addr, ax_wdata;
    reg        ax_ack;                 // 1 拍完成脉冲
    reg [31:0] ax_rdata_r;
    reg        ax_owner_q;             // 0 = H2C，1 = C2H（事务开始时锁存）
    wire       ax_busy = (ast != A_IDLE);

    // H2C / C2H 各自需要的“命令”（组合）
    reg        h_cmd_wr;
    reg [31:0] h_cmd_addr, h_cmd_wdata;
    reg        c_cmd_wr;
    reg [31:0] c_cmd_addr, c_cmd_wdata;

    // 状态机声明（后面 always 使用）
    localparam [2:0] H_INIT=3'd0, H_IDLE=3'd1, H_POLL=3'd2, H_INFO=3'd3,
                     H_BEAT=3'd4, H_WR=3'd5, H_SET=3'd6, H_DRAIN=3'd7;
    localparam [2:0] C_POLL=3'd0, C_INFO=3'd1, C_HDR=3'd2,
                     C_READ=3'd3, C_ASM=3'd5, C_SEND=3'd4, C_ACK=3'd6,

                     C_HDRW=3'd7;

    reg [2:0]  hst, cst;
    reg        init_done;
    reg        err_h, err_c;
    reg        err_sticky;
    reg [19:0] drain_tmo;               // H_DRAIN 超时（非法头后主机不发 tlast 的兜底）

    // 命令仲裁：H2C 需要命令时优先；C2H 只在 H2C 不需要时用总线
    wire h_need = (hst == H_INIT) || (hst == H_POLL) || (hst == H_INFO) ||
                  (hst == H_WR)   || (hst == H_SET);
    wire c_need = init_done && ((cst == C_POLL) || (cst == C_INFO) ||
                                (cst == C_READ) || (cst == C_ACK));

    always @(*) begin
        ax_req   = h_need | c_need;
        ax_wr    = h_need ? h_cmd_wr    : c_cmd_wr;
        ax_addr  = h_need ? h_cmd_addr  : c_cmd_addr;
        ax_wdata = h_need ? h_cmd_wdata : c_cmd_wdata;
    end

    // ---------------- AXI-Lite master FSM ----------------
    always @(posedge clk) begin
        if (reset) begin
            ast <= A_IDLE;
            m_axil_awaddr <= 32'h0; m_axil_awvalid <= 1'b0;
            m_axil_wdata  <= 32'h0; m_axil_wstrb   <= 4'hF; m_axil_wvalid <= 1'b0;
            m_axil_bready <= 1'b0;
            m_axil_araddr <= 32'h0; m_axil_arvalid <= 1'b0;
            m_axil_rready <= 1'b0;
            ax_ack <= 1'b0; ax_rdata_r <= 32'h0; ax_owner_q <= 1'b0;
        end
        else begin
            ax_ack <= 1'b0;

            case (ast)
                A_IDLE: begin
                    if (ax_req && !ax_ack) begin
                        ax_owner_q <= h_need ? 1'b0 : 1'b1;
                        if (ax_wr) begin
                            m_axil_awaddr  <= ax_addr;
                            m_axil_awvalid <= 1'b1;
                            m_axil_wdata   <= ax_wdata;
                            m_axil_wstrb   <= 4'hF;
                            m_axil_wvalid  <= 1'b1;
                            ast <= A_WR;
                        end
                        else begin
                            m_axil_araddr  <= ax_addr;
                            m_axil_arvalid <= 1'b1;
                            ast <= A_RD;
                        end
                    end
                end

                A_WR: begin
                    if (m_axil_awready) m_axil_awvalid <= 1'b0;
                    if (m_axil_wready)  m_axil_wvalid  <= 1'b0;
                    // 两边都握手完才去等 B
                    if ((!m_axil_awvalid || m_axil_awready) &&
                        (!m_axil_wvalid  || m_axil_wready)) begin

                        m_axil_bready <= 1'b1;
                        ast <= A_B;
                    end
                end

                A_B: begin
                    if (m_axil_bvalid) begin
                        m_axil_bready <= 1'b0;
                        ax_ack <= 1'b1;
                        ast <= A_IDLE;
                    end
                end

                A_RD: begin
                    if (m_axil_arready) m_axil_arvalid <= 1'b0;
                    if (!m_axil_arvalid || m_axil_arready) begin

                        m_axil_rready <= 1'b1;
                        ast <= A_R;
                    end
                end

                A_R: begin
                    if (m_axil_rvalid) begin
                        m_axil_rready <= 1'b0;
                        ax_rdata_r <= m_axil_rdata;
                        ax_ack <= 1'b1;
                        ast <= A_IDLE;
                    end
                end

                default: ast <= A_IDLE;
            endcase
        end
    end

    wire h_ack = ax_ack && (ax_owner_q == 1'b0);
    wire c_ack = ax_ack && (ax_owner_q == 1'b1);

    // ================= 组合：H2C 命令内容 / 当前字 =================
    reg [31:0] hword;                  // 当前要写的 32 位字
    integer    kk;

    reg [DATA_WIDTH-1:0] hbeat_data;
    reg [KEEP_WIDTH-1:0] hbeat_keep;
    reg                  hbeat_last;
    reg [7:0]            hbeat_idx;      // 64 位宽时最多 184 拍，8 位够
    reg [3:0]            hword_idx;
    reg [3:0]            hport;
    reg [15:0]           hlen;

    wire [31:0] h_remain     = hlen - (hbeat_idx * BYTES_PER_BEAT);
    wire [5:0]  h_vbytes     = (h_remain > BYTES_PER_BEAT) ? BEAT_BYTES : h_remain[5:0];
    wire [3:0]  h_words_todo = (h_vbytes + 6'd3) >> 2;
    wire [7:0]  h_last_idx   = ((hlen + (BYTES_PER_BEAT-1)) >> BBL) - 1;

    // 从本拍里取第 hword_idx 个 32 位字（循环展开成固定基址的 mux）
    always @(*) begin
        hword = 32'h0;
        for (kk = 0; kk < WPB; kk = kk + 1)
            if (hword_idx == kk[3:0])
                hword = hbeat_data[kk*32 +: 32];
    end

    always @(*) begin
        h_cmd_wr    = 1'b0;
        h_cmd_addr  = 32'h0;
        h_cmd_wdata = 32'h0;
        case (hst)
            H_INIT: begin
                h_cmd_wr    = 1'b1;
                h_cmd_addr  = {24'h0, REG_CTRL, 2'b00};
                h_cmd_wdata = 32'h0000_0005;      // irq_en=1, host_en=1
            end
            H_POLL: begin
                h_cmd_addr  = {24'h0, REG_STATUS, 2'b00};
            end
            H_INFO: begin
                h_cmd_wr    = 1'b1;
                h_cmd_addr  = {24'h0, REG_IN_INFO, 2'b00};
                h_cmd_wdata = {12'h0, hport, hlen}; // {port[3:0], len[15:0]}
            end
            H_WR: begin
                h_cmd_wr    = 1'b1;
                h_cmd_addr  = {24'h0, REG_IN_DATA, 2'b00};
                h_cmd_wdata = hword;
            end
            H_SET: begin
                h_cmd_wr    = 1'b1;
                h_cmd_addr  = {24'h0, REG_IN_SET, 2'b00};
                h_cmd_wdata = 32'h0000_0001;
            end
            default: ;
        endcase
    end

    // ================= H2C FSM =================
    always @(posedge clk) begin
        if (reset) begin
            hst <= H_INIT;
            init_done <= 1'b0;
            hport <= 4'd0; hlen <= 16'd0;
            hbeat_data <= {DATA_WIDTH{1'b0}}; hbeat_keep <= {KEEP_WIDTH{1'b0}};
            hbeat_last <= 1'b0; hbeat_idx <= 8'd0; hword_idx <= 4'd0;
            err_h <= 1'b0;
            drain_tmo <= 20'd0;
        end
        else begin
            err_h <= 1'b0;

            case (hst)
                H_INIT: begin
                    if (h_ack) begin
                        init_done <= 1'b1;
                        hst <= H_IDLE;
                    end
                end

                H_IDLE: begin
                    if (m_axis_h2c_tvalid && m_axis_h2c_tready) begin
                        // header 拍：byte0=port，byte2..3=length
                        hport <= m_axis_h2c_tdata[3:0];   // byte0 的 4 位（0..9）
                        hlen  <= m_axis_h2c_tdata[31:16];

                        if ((m_axis_h2c_tdata[7:0] > 8'd9) || m_axis_h2c_tlast ||
                            (m_axis_h2c_tdata[31:16] == 16'd0) ||
                            (m_axis_h2c_tdata[31:16] > MAX_LEN[15:0])) begin
                            err_h <= 1'b1;
                            drain_tmo <= 20'd0;

                            hst <= m_axis_h2c_tlast ? H_IDLE : H_DRAIN;
                        end
                        else begin
                            hbeat_idx <= 8'd0;
                            hst <= H_POLL;
                        end
                    end
                end

                H_POLL: begin
                    if (h_ack) begin
                        if (ax_rdata_r[0] == 1'b0) hst <= H_INFO;   // IN_VALID=0
                    end
                end

                H_INFO: begin
                    if (h_ack) begin
                        hword_idx <= 4'd0;
                        hst <= H_BEAT;
                    end
                end

                H_BEAT: begin
                    if (m_axis_h2c_tvalid && m_axis_h2c_tready) begin

                        if (m_axis_h2c_tlast != (hbeat_idx == h_last_idx)) begin
                            err_h <= 1'b1;
                            if (m_axis_h2c_tlast) begin
                                // 早结束：本条消息的边界就是这一拍 ⇒ 不再有剩余拍
                                hst <= H_IDLE;
                            end else begin
                                // 该结束却没给 tlast ⇒ 吃掉剩余拍直到 tlast（有超时兜底）
                                drain_tmo <= 20'd0;
                                hst <= H_DRAIN;
                            end
                        end else begin
                            hbeat_data <= m_axis_h2c_tdata;
                            hbeat_keep <= m_axis_h2c_tkeep;
                            hbeat_last <= m_axis_h2c_tlast;
                            hword_idx <= 4'd0;
                            hst <= H_WR;
                        end
                    end
                end

                H_WR: begin
                    if (h_ack) begin
                        if ((hword_idx + 4'd1) < h_words_todo) begin
                            hword_idx <= hword_idx + 4'd1;
                        end
                        else if (hbeat_idx == h_last_idx) begin
                            hst <= H_SET;
                        end
                        else begin
                            hbeat_idx <= hbeat_idx + 8'd1;
                            hst <= H_BEAT;
                        end
                    end
                end

                H_SET: begin
                    if (h_ack) hst <= H_IDLE;
                end

                H_DRAIN: begin

                    drain_tmo <= drain_tmo + 20'd1;
                    if (m_axis_h2c_tvalid && m_axis_h2c_tlast) begin
                        hst <= H_IDLE;
                    end
                    else if (drain_tmo == 20'hA0000) begin
                        err_h <= 1'b1;
                        hst <= H_IDLE;
                    end
                end

                default: hst <= H_IDLE;
            endcase
        end
    end

    assign m_axis_h2c_tready = (hst == H_IDLE) || (hst == H_BEAT) || (hst == H_DRAIN);

    // ================= 组合：C2H =================
    reg [3:0]  cport;
    reg [15:0] clen;
    reg [7:0]  cbeat_idx;
    reg [3:0]  cword_idx;
    reg [31:0] cwords [0:WPB-1];
    reg [DATA_WIDTH-1:0] cbeat_data;
    reg [KEEP_WIDTH-1:0] ckeep;
    reg                  clast;
    reg                  clast_r;

    wire [31:0] c_remain     = clen - (cbeat_idx * BYTES_PER_BEAT);
    wire [5:0]  c_vbytes     = (c_remain > BYTES_PER_BEAT) ? BEAT_BYTES : c_remain[5:0];
    wire [3:0]  c_words_todo = (c_vbytes + 6'd3) >> 2;
    wire [7:0]  c_last_idx   = ((clen + (BYTES_PER_BEAT-1)) >> BBL) - 1;

    // 拼接本拍（未用到的字可能残留，由 tkeep 屏蔽）
    always @(*) begin
        cbeat_data = {DATA_WIDTH{1'b0}};
        for (kk = 0; kk < WPB; kk = kk + 1)
            cbeat_data[kk*32 +: 32] = cwords[kk];
    end

    // 末拍 tkeep：vbytes 个 1（vbytes==KEEP_WIDTH 时全 1）
    always @(*) begin
        ckeep = {KEEP_WIDTH{1'b1}} >> (KEEP_WIDTH - c_vbytes);
    end

    always @(*) begin
        clast = (cbeat_idx == c_last_idx);
    end

    always @(*) begin
        c_cmd_wr    = 1'b0;
        c_cmd_addr  = 32'h0;
        c_cmd_wdata = 32'h0;
        case (cst)
            C_POLL: c_cmd_addr = {24'h0, REG_STATUS, 2'b00};
            C_INFO: c_cmd_addr = {24'h0, REG_OUT_INFO, 2'b00};
            C_READ: c_cmd_addr = {24'h0, REG_OUT_DATA, 2'b00};
            C_ACK: begin
                c_cmd_wr    = 1'b1;
                c_cmd_addr  = {24'h0, REG_OUT_ACK, 2'b00};
                c_cmd_wdata = 32'h0000_0001;
            end
            default: ;
        endcase
    end

    // ================= C2H FSM =================
    always @(posedge clk) begin
        if (reset) begin
            cst <= C_POLL;
            cport <= 4'd0; clen <= 16'd0;
            cbeat_idx <= 8'd0; cword_idx <= 4'd0;
            clast_r <= 1'b0;
            s_axis_c2h_tdata <= {DATA_WIDTH{1'b0}};
            s_axis_c2h_tkeep <= {KEEP_WIDTH{1'b0}};
            s_axis_c2h_tlast <= 1'b0;
            s_axis_c2h_tvalid <= 1'b0;
            err_c <= 1'b0;
        end
        else begin
            err_c <= 1'b0;

            case (cst)
                C_POLL: begin
                    s_axis_c2h_tvalid <= 1'b0;
                    if (c_ack && ax_rdata_r[1]) cst <= C_INFO;   // OUT_VALID=1
                end

                C_INFO: begin
                    if (c_ack) begin
                        cport <= ax_rdata_r[19:16];
                        clen  <= ax_rdata_r[15:0];

                        if ((ax_rdata_r[19:16] > 4'd9) ||
                            (ax_rdata_r[15:0] == 16'd0) ||
                            (ax_rdata_r[15:0] > MAX_LEN[15:0])) begin
                            err_c <= 1'b1;
                            cst <= C_ACK;                   // 非法 → 释放 egress
                        end
                        else begin
                            cbeat_idx <= 8'd0;
                            cst <= C_HDR;
                        end
                    end
                end

                C_HDR: begin
                    // header 拍：byte0=port，byte2..3=length
                    s_axis_c2h_tdata  <= {{(DATA_WIDTH-32){1'b0}}, clen, 8'h00, {4'h0, cport}};
                    s_axis_c2h_tkeep  <= {KEEP_WIDTH{1'b1}};
                    s_axis_c2h_tlast  <= 1'b0;
                    s_axis_c2h_tvalid <= 1'b1;

                    cst <= C_HDRW;
                end
                C_HDRW: begin
                    // 等握手：tvalid 已在上一拍拉到线上
                    if (s_axis_c2h_tready) begin
                        s_axis_c2h_tvalid <= 1'b0;
                        cword_idx <= 4'd0;
                        cst <= C_READ;
                    end
                end

                C_READ: begin
                    if (c_ack) begin
                        cwords[cword_idx] <= ax_rdata_r;
                        if ((cword_idx + 4'd1) < c_words_todo) begin
                            cword_idx <= cword_idx + 4'd1;
                        end
                        else begin
                            // 等一拍让 cwords 的最后一字落定，再由 C_ASM 组装输出
                            clast_r <= clast;
                            cst <= C_ASM;
                        end
                    end
                end

                C_ASM: begin
                    s_axis_c2h_tdata  <= cbeat_data;
                    s_axis_c2h_tkeep  <= ckeep;
                    s_axis_c2h_tlast  <= clast_r;
                    s_axis_c2h_tvalid <= 1'b1;
                    cst <= C_SEND;
                end

                C_SEND: begin
                    // 数据/keep/last 必须保持到 tready
                    s_axis_c2h_tvalid <= 1'b1;
                    if (s_axis_c2h_tready) begin
                        s_axis_c2h_tvalid <= 1'b0;
                        if (clast_r) cst <= C_ACK;
                        else begin
                            cbeat_idx <= cbeat_idx + 8'd1;
                            cword_idx <= 4'd0;
                            cst <= C_READ;
                        end
                    end
                end

                C_ACK: begin
                    if (c_ack) cst <= C_POLL;
                end

                default: cst <= C_POLL;
            endcase
        end
    end


    always @(posedge clk) begin
        if (reset)                      err_sticky <= 1'b0;
        else if (err_h | err_c)         err_sticky <= 1'b1;
    end
    assign axis_error = err_sticky;

endmodule
