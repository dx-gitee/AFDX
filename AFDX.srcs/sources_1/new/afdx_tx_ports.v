// afdx_tx_ports.v —— AFDX 发送端口层（采样 / 队列 / SAP 三种端口，各 1 个）
`timescale 1ns/1ps

module afdx_tx_ports #(

    parameter NP  = 10,
    parameter NPW = 4,
    parameter AW  = 14                     // 每端口 2^(14-4)=1024 字 = 4 KB（NW=10）
) (
    input  wire        clk,
    input  wire        reset,

    // ---- CPU 侧 ----
    input  wire           wr,
    input  wire [NPW-1:0] pi,
    input  wire [2:0]     reg_sel,
    input  wire [31:0]    wdata,
    output reg  [31:0]    rdata,

    // ---- 数据面：仲裁后的一条字节流 ----
    output reg         tx_valid,
    input  wire        tx_ready,
    output reg         tx_sop,
    output reg         tx_eop,
    output wire [7:0]  tx_data,
    output reg  [15:0] tx_vlid,            // 本消息发到哪条 VL（预设映射）
    output reg  [31:0] tx_dip,             // 目的 IP（SAP 口由 CPU 写，其余为 0 = 用表里的）
    output reg  [15:0] tx_dport,           // 目的 UDP 端口（同上）
    output reg  [10:0] tx_len,             // 本消息字节数（给组帧器）

    input  wire        tx_hold,            // 组帧器："B 副本还没重放，先别起下一条消息"（见 tx_build 的 pay_hold）
    input  wire        tx_drop,

    output wire [NP-1:0] tx_busy,
    output wire          tx_pend
);
    localparam NW  = AW - NPW;             // 每端口字数位宽


    localparam [2:0] R_DATA=3'd0, R_LEN=3'd1, R_CTRL=3'd2,
                     R_DIP=3'd3,  R_DPT=3'd4, R_ST=3'd5;


    localparam RAMW = NP * (1 << NW);
    reg [31:0]   ram [0:RAMW-1];
    reg [AW-1:0] waddr;
    reg [31:0]   wdata_w;
    reg          wren;
    reg [AW-1:0] raddr;                    // 数据面读地址（纯寄存器）
    reg [31:0]   word_q;

    reg [31:0]   word_cur;

    always @(posedge clk) begin
        if (wren) ram[waddr] <= wdata_w;
        word_q <= ram[raddr];              // 一写一读
    end

    // ================= 每端口元数据 =================
    reg [NW-1:0] pwptr [0:NP-1];           // CPU 写指针
    reg [NW:0]   plen  [0:NP-1];           // 本消息字节数
    reg          ppend [0:NP-1];           // 已提交、待发送
    reg          pdone [0:NP-1];           // 发送完成（锁存，读 STATUS 清）
    reg          perr  [0:NP-1];
    reg [31:0]   pdip  [0:NP-1];           // 目的 IP（SAP 用）
    reg [15:0]   pdpt  [0:NP-1];           // 目的 UDP 端口（SAP 用）
    reg [31:0]   plen_w [0:NP-1];          // 最近一次写入 LEN 的值（等 CTRL 提交）

    // ================= 发送引擎（一次只服务一个端口）=================
    reg [NPW-1:0] act;                     // 正在发的端口
    reg           act_vld;
    reg [NW-1:0]  e_raddr;
    reg [NW:0]    e_sent, e_len;
    reg [1:0]     e_bcnt;
    reg           e_fetch, e_load, e_stream;   // e_fetch=等读结果那一拍, e_load=搬首字那一拍


    integer pi_i;


    reg  [NPW-1:0] nxt;
    reg            have_pend;

    wire [15:0] vlid_map;
    afdx_port_vlid #(.NPW(NPW)) u_vlidmap (
        .port (nxt),
        .vlid (vlid_map)
    );

    integer        ai;
    always @(*) begin
        nxt = {NPW{1'b0}}; have_pend = 1'b0;
        for (ai = NP - 1; ai >= 0; ai = ai - 1)
            if (ppend[ai]) begin nxt = ai[NPW-1:0]; have_pend = 1'b1; end
    end

    // 哪些端口是 SAP 口（带"目的地址由 CPU 给"）：本设计端口 2..NP-1 都是 SAP
    localparam [NPW-1:0] SAP_FIRST = 2'd2;


    wire abort_now = tx_drop && act_vld;


    wire [7:0]  e_byte   = word_cur[{e_bcnt, 3'b000} +: 8];
    wire        e_islast = (e_sent == e_len - 1'b1);
    assign      tx_data  = e_byte;

    // 各端口"忙"位图（通用 NP）
    genvar gi;
    generate for (gi = 0; gi < NP; gi = gi + 1) begin : g_busy
        assign tx_busy[gi] = pdone[gi] | ppend[gi];

    end endgenerate


    assign tx_pend = have_pend | act_vld;

    always @(posedge clk) begin
        if (reset) begin
            wren <= 1'b0; waddr <= 0; wdata_w <= 0; raddr <= 0;
            act <= 0; act_vld <= 0;
            e_raddr <= 0; e_sent <= 0; e_len <= 0; e_bcnt <= 0; word_cur <= 32'h0;
            e_fetch <= 0; e_load <= 0; e_stream <= 0; tx_len <= 0;
            for (pi_i = 0; pi_i < NP; pi_i = pi_i + 1) begin
                pwptr[pi_i] <= {NW{1'b0}};
                plen [pi_i] <= 0;
                ppend[pi_i] <= 1'b0;
                pdone[pi_i] <= 1'b0;
                perr [pi_i] <= 1'b0;
                pdip [pi_i] <= 32'h0;
                pdpt [pi_i] <= 16'h0;
                plen_w[pi_i] <= 32'h0;
            end
        end
        else begin
            wren <= 1'b0;


            if (wr && (pi < NP) && !(act_vld && (act == pi)) && !ppend[pi]) begin   // 越界门（与读通路的 pi_in_range 同义，此处避免前向引用）
                case (reg_sel)
                    R_DATA: begin
                        waddr   <= {pi, pwptr[pi]};
                        wdata_w <= wdata;
                        wren    <= 1'b1;
                        pwptr[pi] <= pwptr[pi] + 1'b1;
                    end
                    R_LEN:  plen_w[pi] <= wdata;
                    R_CTRL: if (wdata[0]) begin
                                if (|plen_w[pi][31:NW+1]) begin

                                    perr[pi]  <= 1'b1;
                                    pdone[pi] <= 1'b1;
                                end else if (plen_w[pi][NW:0] != 0) begin
                                    // 提交：把长度搬进引擎要用的位置，标记为待发
                                    // （0 长度直接不提交 —— 否则 e_len-1 会回绕成全 1）
                                    plen [pi] <= plen_w[pi][NW:0];
                                    ppend[pi] <= 1'b1;
                                    pdone[pi] <= 1'b0;
                                    perr [pi] <= 1'b0;
                                    pwptr[pi] <= {NW{1'b0}};      // 下一次从头发
                                end else begin

                                    perr[pi]  <= 1'b1;
                                    pdone[pi] <= 1'b1;
                                end
                            end
                    R_DIP:  pdip[pi] <= wdata;
                    R_DPT:  pdpt[pi] <= wdata[15:0];
                    default: ;
                endcase
            end


            if (abort_now) begin

                act_vld  <= 1'b0;
                e_fetch  <= 1'b0;
                e_stream <= 1'b0;
                ppend[act] <= 1'b0;
                pdone[act] <= 1'b1;
                perr [act] <= 1'b1;
            end
            // tx_hold 期间不许起新消息（否则下一条的载荷会被 B 副本吃掉、且 B 帧装上错载荷）
            else if (!act_vld && have_pend && !e_fetch && !e_stream && !tx_hold) begin
                // 起一条新消息
                act      <= nxt;
                act_vld  <= 1'b1;
                e_len    <= plen[nxt];
                e_raddr  <= {NW{1'b0}};
                raddr    <= {nxt, {NW{1'b0}}};
                e_sent   <= 0;
                e_bcnt   <= 2'd0;
                e_fetch  <= 1'b1;
                e_stream <= 1'b0;
                tx_vlid  <= vlid_map;
                tx_dip   <= (nxt >= SAP_FIRST) ? pdip[nxt] : 32'h0;   // 仅 SAP 口带动态目的
                tx_dport <= (nxt >= SAP_FIRST) ? pdpt[nxt] : 16'h0;
                tx_len   <= plen[nxt][10:0];
            end

            // ---------- 取字 / 按字节送 ----------
            if (act_vld && !abort_now) begin
                if (e_fetch) begin

                    e_fetch <= 1'b0;
                    e_load  <= 1'b1;
                end
                else if (e_load) begin
                    // 本拍 `word_q` = 第 0 个字 ✓：搬进 `word_cur`，并**预取第 1 个字**
                    word_cur <= word_q;
                    raddr    <= {act, {NW{1'b0}} + 1'b1};
                    e_bcnt   <= 2'd0;
                    e_load   <= 1'b0;
                    e_stream <= 1'b1;
                end
                else if (e_stream && tx_ready) begin
                    e_sent <= e_sent + 1'b1;
                    if (e_islast) begin
                        e_stream <= 1'b0;
                        act_vld  <= 1'b0;
                        ppend[act] <= 1'b0;
                        pdone[act] <= 1'b1;
                    end
                    else if (e_bcnt == 2'd1) begin

                        raddr  <= {act, e_raddr + 1'b1};
                        e_bcnt <= e_bcnt + 1'b1;
                    end
                    else if (e_bcnt == 2'd3) begin
                        word_cur <= word_q;
                        e_raddr  <= e_raddr + 1'b1;
                        e_bcnt   <= 2'd0;            // 下一拍接着发新字的第 0 字节 → **无空拍**
                    end
                    else e_bcnt <= e_bcnt + 1'b1;
                end
            end
        end
    end


    always @(*) begin
        tx_valid = e_stream && act_vld;
        tx_sop   = e_stream && act_vld && (e_sent == 0);
        tx_eop   = e_stream && act_vld && e_islast;
    end


    wire pi_in_range = (pi < NP);

    always @(*) begin
        if (!pi_in_range) rdata = 32'h0;
        else case (reg_sel)

            3'd5: rdata = {28'h0, perr[pi], ppend[pi], pdone[pi], (act_vld && (act == pi))};
            default: rdata = 32'h0;
        endcase
    end

endmodule
