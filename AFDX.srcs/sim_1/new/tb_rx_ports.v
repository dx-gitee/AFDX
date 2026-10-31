// tb_rx_ports.v —— 接收端口层单元测试（ARINC 664 P7 §3.3.1）
`timescale 1ns/1ps

module tb_rx_ports;

    localparam NPT = 6;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    // ---- CPU 侧 ----
    reg        wr = 0, rd = 0;
    reg  [2:0] reg_sel = 0;
    reg  [31:0] wdata = 0;
    wire [31:0] rdata;
    reg        pwr = 0, prd = 0;
    reg  [2:0] psel = 0;
    reg  [31:0] pwdata = 0;
    wire [15:0] prdata;

    // ---- 数据面 ----
    reg        in_valid = 0, in_sop = 0, in_eop = 0;
    reg  [7:0] in_data = 0;
    reg  [2:0] port_i = 0;
    reg  [15:0] dport_i = 0;
    reg  [31:0] sip_i = 0;
    reg  [15:0] sport_i = 0;
    reg         net_i = 0;
    reg        rx_commit = 0, rx_abort = 0;

    wire rx_any, rx_ovf_any;
    wire [NPT-1:0] rx_has_o, rx_ovf_o;


    afdx_rx_ports #(.NP(NPT), .NPW(3)) dut (
        .clk(clk), .reset(reset),
        .wr(wr), .rd(rd), .reg_sel(reg_sel), .wdata(wdata), .rdata(rdata),
        .pwr(pwr), .prd(prd), .psel(psel), .pwdata(pwdata), .prdata(prdata),
        .cpu_pi(3'd0), .cpu_sel_en(1'b0),
        .in_valid(in_valid), .in_sop(in_sop), .in_eop(in_eop), .in_data(in_data),
        .port_i(port_i), .dport_i(dport_i), .sip_i(sip_i), .sport_i(sport_i),
        .net_i(net_i),
        .rx_commit(rx_commit), .rx_abort(rx_abort),
        .rx_any(rx_any), .rx_ovf_any(rx_ovf_any),
        .rx_has_o(rx_has_o), .rx_ovf_o(rx_ovf_o)
    );

    integer errs = 0;

    integer nchk = 0;
    task chk;
        input cond; input [255:0] name;
        begin
            nchk = nchk + 1;
            if (cond) $display("  [ok]   #%0d %0s", nchk, name);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, name); end
        end
    endtask

    // 端口配置：type 0=禁用 1=采样 2=队列 3=SAP
    task cfg_port;
        input [2:0] idx; input en; input [1:0] ty; input [3:0] md;
        begin
            @(negedge clk); pwr = 1; psel = 3'd0; pwdata = {30'h0, idx}; @(posedge clk);
            @(negedge clk);         psel = 3'd1; pwdata = {24'h0, 3'd0, md, ty, en}; @(posedge clk);
            @(negedge clk); pwr = 0; psel = 3'd0; pwdata = 0; @(posedge clk);
        end
    endtask

    task sel_port;
        input [2:0] p;
        begin
            @(negedge clk); wr = 1; reg_sel = 3'd7; wdata = {29'h0, 1'b0, p};
            @(posedge clk);
            @(negedge clk); wr = 0;
            repeat(3) @(posedge clk);
        end
    endtask

    // 灌一条消息（只送字节流，不判决）
    task feed;
        input [2:0] pi; input [15:0] dp; input integer len; input [7:0] base;
        input [31:0] sip; input [15:0] spt;
        integer k;
        begin
            for (k = 0; k < len; k = k + 1) begin
                @(negedge clk);
                in_valid = 1; in_sop = (k == 0); in_eop = (k == len-1);
                in_data  = base + k[7:0]; port_i = pi; dport_i = dp;
                sip_i = sip; sport_i = spt;
                @(posedge clk);
            end
            @(negedge clk); in_valid = 0; in_sop = 0; in_eop = 0;
            repeat(2) @(posedge clk);
        end
    endtask

    task do_commit; begin
        @(negedge clk); rx_commit = 1; @(posedge clk);
        @(negedge clk); rx_commit = 0; @(posedge clk); end
    endtask

    task do_abort; begin
        @(negedge clk); rx_abort = 1; @(posedge clk);
        @(negedge clk); rx_abort = 0; @(posedge clk); end
    endtask


    reg [31:0] got;
    task rd_word;
        begin
            @(negedge clk); rd = 1; reg_sel = 3'd4; #1 got = rdata;
            @(posedge clk);
            @(negedge clk); rd = 0; reg_sel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask

    reg [31:0] prt;
    task rd_port;
        begin
            @(negedge clk); rd = 1; reg_sel = 3'd5; #1 prt = rdata;
            @(posedge clk);
            @(negedge clk); rd = 0; reg_sel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask

    task do_release;
        begin

            @(negedge clk); wr = 1; reg_sel = 3'd6; wdata = 32'h1; @(posedge clk);
            @(negedge clk); wr = 0; reg_sel = 3'd0;
            repeat(3) @(posedge clk);
        end
    endtask

    // 消息窗口的 STATUS（reg_sel=3）：bit2 = **本端口**有数据，bit3 = **本端口**溢出
    reg [31:0] stat_w;
    task rd_status;
        begin
            @(negedge clk); rd = 1; reg_sel = 3'd3; #1 stat_w = rdata;
            @(posedge clk);
            @(negedge clk); rd = 0; reg_sel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask

    reg [15:0] pstat;
    reg [15:0] lenr;
    task rd_len;
        begin
            @(negedge clk); prd = 1; psel = 3'd3; #1 lenr = prdata;
            @(posedge clk);
            @(negedge clk); prd = 0; psel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask

    task rd_pstat;
        begin
            @(negedge clk); prd = 1; psel = 3'd2; #1 pstat = prdata;
            @(posedge clk);
            @(negedge clk); prd = 0; psel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask


    reg [15:0] cfg_r;
    task rd_cfg;
        begin
            @(negedge clk); prd = 1; psel = 3'd1; #1 cfg_r = prdata;
            @(posedge clk);
            @(negedge clk); prd = 0; psel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask


    reg [15:0] netr;
    task rd_net;
        begin
            @(negedge clk); prd = 1; psel = 3'd7; #1 netr = prdata;
            @(posedge clk);
            @(negedge clk); prd = 0; psel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask

    reg [15:0] len_r, spt_r, sip_lo, sip_hi;
    task rd_meta;                        // 一次读出：长度 / 源端口 / 源 IP 高低 16 位
        begin
            @(negedge clk); prd = 1; psel = 3'd3; #1 len_r  = prdata; @(posedge clk);
            @(negedge clk);          psel = 3'd4; #1 spt_r  = prdata; @(posedge clk);
            @(negedge clk);          psel = 3'd5; #1 sip_lo = prdata; @(posedge clk);
            @(negedge clk);          psel = 3'd6; #1 sip_hi = prdata; @(posedge clk);
            @(negedge clk); prd = 0; psel = 3'd0;
            repeat(2) @(posedge clk);
        end
    endtask

    function [31:0] mk;                  // base 起的 4 字节小端字
        input [7:0] b;
        mk = {b+8'd3, b+8'd2, b+8'd1, b};
    endfunction

    integer i;
    reg [31:0] srcip;

    initial begin
        #200 reset = 0;
        repeat(5) @(posedge clk);
        $display("");
        $display("======== 接收端口层（采样/队列/SAP）单元测试 ========");

        // ================= 01~04：队列口（端口 1）FIFO 顺序 =================
        cfg_port(2'd1, 1'b1, 2'd2, 4'd8);

        sel_port(2'd1);
        rd_cfg;
        chk(cfg_r[7:0] === 8'h45, "00c 端口配置回读0x45");
        feed(2'd1, 16'h0BB9, 8, 8'hA0, 32'h0A000001, 16'h1000); do_commit;
        feed(2'd1, 16'h0BB9, 8, 8'hB0, 32'h0A000001, 16'h1000); do_commit;
        feed(2'd1, 16'h0BB9, 8, 8'hC0, 32'h0A000001, 16'h1000); do_commit;
        sel_port(2'd1);
        chk(dut.pcnt[1] === 4'd3, "01 队列口连灌 3 条 → 排队条数 = 3");
        rd_word; chk(got === mk(8'hA0), "02 第 1 条读出正确（FIFO 先入先出）");
        rd_word; chk(got === mk(8'hA4), "   第 1 条第 2 字正确");
        do_release;
        rd_word; chk(got === mk(8'hB0), "03 释放后第 2 条接上（顺序不乱）");
        rd_word; do_release;
        rd_word; chk(got === mk(8'hC0), "04 第 3 条正确");
        rd_word; do_release;
        rd_pstat; chk(pstat[1] === 1'b0, "   全部读完后端口 1 无数据");

        // ================= 05~10：队列深度限制 + 溢出 =================
        cfg_port(2'd1, 1'b1, 2'd2, 4'd2);
        feed(2'd1, 16'h0BB9, 8, 8'h10, 0, 0); do_commit;
        feed(2'd1, 16'h0BB9, 8, 8'h20, 0, 0); do_commit;
        feed(2'd1, 16'h0BB9, 8, 8'h30, 0, 0); do_commit;   // 第 3 条应被丢弃
        sel_port(2'd1);
        rd_pstat;
        chk(dut.pcnt[1] === 4'd2, "05 深度 2：排队条数 = 2（第 3 条被丢）");
        chk(pstat[NPT+1] === 1'b1,  "06 溢出标志置位（P7 §3.3.1 丢新到消息）");
        rd_word; chk(got === mk(8'h10), "07 队首仍是第 1 条（没被第 3 条顶掉）");
        rd_word; do_release;
        rd_word; chk(got === mk(8'h20), "08 第 2 条正确");
        rd_word; do_release;
        rd_pstat; chk(pstat[1] === 1'b0, "09 队列已空");
        chk(pstat[NPT+1] === 1'b0,         "10 溢出位图读即清");

        // ================= 11~13：采样口（端口 0）=================
        cfg_port(2'd0, 1'b1, 2'd1, 4'd8);
        feed(2'd0, 16'h0400, 8, 8'h70, 0, 0); do_commit;
        feed(2'd0, 16'h0400, 8, 8'h80, 0, 0); do_commit;
        feed(2'd0, 16'h0400, 8, 8'h90, 0, 0); do_commit;   // 新值覆盖旧值
        sel_port(2'd0);
        rd_pstat; chk(dut.pcnt[0] === 4'd1, "11 采样口连灌 3 条 → 只保留 1 条");
        rd_word;  chk(got === mk(8'h90), "12 采样口拿到的是**最新**那条（第 3 条）");
        rd_word;  do_release;
        feed(2'd0, 16'h0400, 8, 8'h50, 0, 0); do_commit;
        rd_word;  chk(got === mk(8'h50), "13 采样口释放后能拿到新值");
        rd_word;  do_release;

        // ================= 14~16：端口禁用 / abort =================
        cfg_port(2'd1, 1'b0, 2'd2, 4'd8);
        feed(2'd1, 16'h0BB9, 8, 8'hE0, 0, 0); do_commit;
        sel_port(2'd1);
        rd_pstat; chk(pstat[1] === 1'b0, "14 端口禁用 → 消息不入队");
        cfg_port(2'd1, 1'b1, 2'd2, 4'd8);
        feed(2'd1, 16'h0BB9, 8, 8'hF0, 0, 0); do_abort;    // IC/RM 判丢弃
        rd_pstat; chk(pstat[1] === 1'b0, "15 IC/RM abort → 消息对 CPU 不可见");
        feed(2'd1, 16'h0BB9, 8, 8'hF0, 0, 0); do_commit;   // 同一端口，改成接受
        rd_pstat; chk(pstat[1] === 1'b1, "16 同一端口 commit → 这次可见（对照）");
        do_release;

        // ================= 17~20：跨端口独立 + 目的端口/长度 =================
        sel_port(2'd1);
        feed(2'd1, 16'h0101, 12, 8'h01, 0, 0); do_commit;   // 12 字节，dport 0x0101
        sel_port(2'd2);
        feed(2'd2, 16'h0777,  4, 8'h02, 0, 0); do_commit;   // 4 字节，dport 0x0777
        sel_port(2'd1);
        rd_port;  chk(prt[15:0] === 16'h0101, "17 端口 1 消息的 UDP 目的端口号 = 0x0101");
        rd_meta;  chk(len_r === 16'd12,       "18 端口 1 消息长度 = 12 字节");
        rd_word; rd_word; rd_word;
        chk(got === mk(8'h01 + 8'd8),         "19 12 字节消息第 3 个字正确");
        do_release;
        sel_port(2'd2);
        rd_port;  chk(prt[15:0] === 16'h0777, "20 端口 2（SAP）消息的 UDP 目的端口号 = 0x0777");
        rd_word; do_release;


        srcip = 32'hC0A8_0164;                                // 192.168.1.100
        feed(2'd2, 16'h0045, 8, 8'h5A, srcip, 16'h1F90); do_commit;   // 源端口 8080
        sel_port(2'd2);
        rd_meta;
        chk(len_r === 16'd8,            "21 SAP 口：消息长度 = 8 字节");
        chk(spt_r === 16'h1F90,         "22 SAP 口：**源 UDP 端口**随数据入队，读回正确");
        chk({sip_hi, sip_lo} === srcip, "23 SAP 口：**源 IP** 随数据入队，读回正确");
        rd_word; chk(got === mk(8'h5A), "24 SAP 口：载荷正确");
        rd_word; do_release;
        srcip = 32'h0A00_0002;                                // 换一个源地址再验一次
        feed(2'd2, 16'h0045, 4, 8'h6B, srcip, 16'h0050); do_commit;
        sel_port(2'd2);
        rd_meta;
        chk({sip_hi, sip_lo} === srcip && spt_r === 16'h0050,
            "25 SAP 口：换一条消息，源 IP/源端口跟着变（不是残留）");


        do_release;                                    // 清掉上面 25 那条（还没释放）
        // 方向一：只有端口 2 有数据 → 端口 1 的 STATUS 必须是 0
        feed(2'd2, 16'h0045, 4, 8'h88, 0, 0); do_commit;
        sel_port(2'd1);
        rd_status; chk(stat_w[2] === 1'b0,
            "26 ★只有端口 2 有数据时，端口 1 的 STATUS.bit0 = 0（不是「任一端口有数据」）");
        sel_port(2'd2);
        rd_status; chk(stat_w[2] === 1'b1, "27 端口 2 自己读到 1（对照，证明 26 不是恒 0）");
        do_release;
        // 方向二：只有端口 1 有数据 → 端口 2 的 STATUS 必须是 0（对称，防止只修了一半）
        feed(2'd1, 16'h0102, 4, 8'h77, 0, 0); do_commit;
        sel_port(2'd2);
        rd_status; chk(stat_w[2] === 1'b0,
            "28 ★只有端口 1 有数据时，端口 2 的 STATUS.bit0 = 0（反方向也隔离）");
        sel_port(2'd1);
        rd_status; chk(stat_w[2] === 1'b1, "29 端口 1 自己读到 1（对照）");


        cfg_port(2'd1, 1'b1, 2'd2, 4'd8);        // 端口 1 = 队列口（深度 8）
        sel_port(2'd1);
        while (dut.pcnt[1] != 0) do_release;     // 排空残局
        chk(dut.pcnt[1] === 3'd0 && dut.queued[1] === 12'd0, "25 起手：队列已排空、占用归零（基线）");


        feed(2'd1, 16'h0101, 3000, 8'h00, 32'h0A000001, 16'h0BB8);
        do_commit;
        sel_port(2'd1);                          // 长度是**寄存读**，先让地址稳定两拍
        rd_len;  chk(lenr === 16'd3000, "25a ★3000 字节消息的长度读回 = 3000（11 位截断已修）");
        chk(dut.plen[{2'd1, 3'd0}] === 14'd3000, "25b 表项里存的是 3000（不是 952）");
        rd_word; chk(got === mk(8'h00), "25c 首字内容正确");
        chk(dut.queued[1] === 12'd750,  "25d 占用记账 = 750 字（= 3000 字节）");
        do_release;
        chk(dut.queued[1] === 12'd0,    "25e 释放后占用归零");


        feed(2'd1, 16'h0202, 4000, 8'h40, 32'h0A000002, 16'h0BB8); do_commit;
        feed(2'd1, 16'h0303, 4000, 8'h80, 32'h0A000003, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd2,       "26a 两条 4000 字节已入队");
        chk(dut.queued[1] === 12'd2000, "26b 占用记账 = 2000 字（= 8000 字节）");
        feed(2'd1, 16'h0404, 1000, 8'hC0, 32'h0A000004, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd2,       "26c ★★装不下的新消息被丢弃（条数仍 = 2）");
        chk(dut.queued[1] === 12'd2000, "26d ★★占用没增长（一个字都没写进去）");
        rd_pstat; chk(pstat[NPT+1] === 1'b1, "26e ★★丢弃记成**可见**的溢出（不静默）");
        begin : t26r
            integer j;
            // 每条 4000 字节 = 1000 字；末字的字节值 = (base + (3996..3999)) & 0xFF
            for (j = 0; j < 999; j = j + 1) rd_word;    // 读前 999 字
            rd_word; chk(got === mk(8'hDC), "26f 第 1 条末字完好（base 0x40 + 0x9C）");
            for (j = 0; j < 999; j = j + 1) rd_word;    // 再读 999 字（第 2 条的前 999）
            rd_word; chk(got === mk(8'h1C), "26g ★★第 2 条末字完好（base 0x80 + 0x9C，没被覆盖）");
        end

        // ---- 27 释放第 1 条腾出空间后，同样的 1000 字节消息就能进去了 ----
        do_release;
        chk(dut.queued[1] === 12'd1000, "27a 释放第 1 条后占用 = 1000 字（= 4000 字节）");
        feed(2'd1, 16'h0505, 1000, 8'hE0, 32'h0A000005, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd2,       "27b ★腾出空间后同一条消息被接受");
        chk(dut.plen[{2'd1, (dut.pqw[1] - 3'd1)}] === 14'd1000, "27c ★新消息以 1000 字节入队（最后压入的槽）");
        chk(dut.queued[1] === 12'd1250,          "27d 占用记账 = 1000 + 250 = 1250 字（= 5000 字节）");


        while (dut.pcnt[1] != 0) do_release;     // 排空残局


        feed(2'd1, 16'h0101, 4095, 8'h10, 32'h0A000001, 16'h0BB8); do_commit;
        feed(2'd1, 16'h0202, 4095, 8'h20, 32'h0A000002, 16'h0BB8); do_commit;
        chk(dut.queued[1] === 12'd2048,     "28a 两条 4095 字节 = 2048 字（正好占满整块缓冲）");
        rd_pstat; chk(pstat[NPT+1] === 1'b0,  "28b 占满但**还没**被判溢出（对照）");
        feed(2'd1, 16'h0303, 2, 8'h30, 32'h0A000003, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd2,           "28c ★★第 3 条被丢弃（剩余 0 个字，无处可写）");
        rd_pstat; chk(pstat[NPT+1] === 1'b1,  "28d ★★丢弃记成**可见**溢出");
        rd_word; chk(got === mk(8'h10),     "28e ★★未读的第 1 条首字**完好**（旧实现这里已被覆盖）");
        do_release; do_release;


        cfg_port(2'd0, 1'b1, 2'd1, 4'd8);        // 端口 0 = 采样口
        sel_port(2'd0);
        while (dut.pcnt[0] != 0) do_release;     // 排空残局
        feed(2'd0, 16'h0400, 5000, 8'h50, 32'h0A000010, 16'h0BB8); do_commit;
        chk(dut.pcnt[0] === 4'd1,           "29a 采样口先收下 5000 字节");
        feed(2'd0, 16'h0401, 4000, 8'h60, 32'h0A000011, 16'h0BB8); do_commit;
        chk(dut.pcnt[0] === 4'd1,           "29b ★★采样口换新值时**不按剩余空间判满**（旧实现会丢新值）");
        rd_len;  chk(lenr === 16'd4000,     "29c ★★读回的是**新**那条（4000 字节）");
        rd_word; chk(got === mk(8'h60),     "29d ★★新值内容正确（不是旧值）");
        rd_pstat; chk(pstat[NPT+0] === 1'b0,  "29e ★★替换不该被记成溢出");
        do_release;


        sel_port(2'd1);
        chk(dut.pcnt[1] === 4'd0,           "30a 起手：端口 1 已空");
        do_release; do_release; do_release;
        chk(dut.pqr[1] === dut.pqw[1],      "30b ★★对空端口释放不推进 pqr（旧实现 pqr 越过 pqw）");
        chk(dut.queued[1] === 12'd0,        "30c ★★占用保持 0（旧实现会被 X 毒化）");
        feed(2'd1, 16'h0707, 16, 8'h70, 32'h0A000012, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd1,           "30d ★★被「误释放」过的端口仍能正常收消息");
        rd_len;  chk(lenr === 16'd16,       "30e ★★长度正确（不是 X、也不是 0）");
        rd_word; chk(got === mk(8'h70),     "30f ★★内容正确");
        do_release;


        feed(2'd1, 16'h0808, 8192, 8'h00, 32'h0A000013, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd1,           "31a ★8192 字节（8 KB 边界）能入队");
        rd_len;  chk(lenr === 16'd8192,     "31b ★★长度读回 8192（13 位时这里会变成 0）");
        chk(dut.queued[1] === 12'd2048,     "31c ★占用 = 2048 字（整块缓冲）");
        do_release;
        chk(dut.queued[1] === 12'd0,        "31d ★释放后 2048 字全部归还");
        feed(2'd1, 16'h0909, 8, 8'h90, 32'h0A000014, 16'h0BB8); do_commit;
        chk(dut.pcnt[1] === 4'd1,           "31e ★★整块缓冲释放后仍能继续收（指针回绕正确）");
        rd_len;  chk(lenr === 16'd8,        "31f ★★新消息长度正确（读指针没跑偏）");
        rd_word; chk(got === mk(8'h90),     "31g ★★新消息内容正确");
        do_release;


        sel_port(2'd0); while (dut.pcnt[0] != 0) do_release;
        sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
        sel_port(2'd0);
        feed(3'd7, 16'h0606, 8, 8'hA0, 32'h0A000020, 16'h0BB8); do_commit;
        chk(dut.pcnt[0] === 4'd1, "32a ★★越界的端口号被夹到端口 0（不再产生 X/写错端口）");
        rd_len;  chk(lenr === 16'd8,      "32b ★★落在端口 0 且长度正确");
        rd_word; chk(got === mk(8'hA0),   "32c ★★载荷逐字节正确（没有静默丢失）");
        do_release;


        begin : t33
            integer q3;
            reg [31:0] lastw;
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            for (q3 = 0; q3 < 20; q3 = q3 + 1) begin       // 20 字节 = 5 个字
                @(negedge clk);
                in_valid = 1; in_sop = (q3 == 0); in_eop = (q3 == 19);
                in_data  = 8'h10 + q3[7:0];
                port_i = 2'd1; dport_i = 16'h0303;
                sip_i = 32'h0A000030; sport_i = 16'h0BB8;
                @(posedge clk);
            end
            @(negedge clk); in_valid = 0; in_sop = 0; in_eop = 0;
            rx_commit = 1;
            @(posedge clk);
            @(negedge clk); rx_commit = 0;
            repeat (4) @(posedge clk);
            chk(dut.queued[1] === 12'd5, "33a ★★commit 与末字写同拍：queued = 5 字（旧实现只记 4）");
            chk(dut.pcnt[1]   === 4'd1,  "33b 发布 1 条（对照）");

            feed(2'd1, 16'h0404, 4, 8'h90, 32'h0A000031, 16'h0BB8); do_commit;
            rd_len; chk(lenr === 16'd20, "33c 第 1 条长度仍是 20（对照）");
            for (q3 = 0; q3 < 4; q3 = q3 + 1) rd_word;     // 跳过前 4 个字
            rd_word; lastw = got;
            chk(lastw === {8'h23, 8'h22, 8'h21, 8'h20},
                "33d ★★第 1 条**末字**（字节 16..19 = 0x20..0x23）完好 —— 没有被第 2 条覆盖");
            do_release; do_release;
        end


        begin : t34
            sel_port(3'd4); while (dut.pcnt[4] != 0) do_release;
            sel_port(3'd5); while (dut.pcnt[5] != 0) do_release;
            feed(3'd4, 16'h00A1, 8, 8'h40, 32'h0A000041, 16'h00A1); do_commit;
            chk(dut.pcnt[4]   === 4'd1,  "34a ★端口 4（SAP-SNMP）能收下消息");
            chk(dut.pcnt[5]   === 4'd0,  "34b ★没有串到端口 5");
            chk(dut.queued[4] === 12'd2, "34c ★占用记账落在端口 4");
            chk(dut.queued[5] === 12'd0, "34d ★端口 5 记账为 0");
            sel_port(3'd4);
            rd_len;  chk(lenr === 16'd8,    "34e ★端口 4 长度正确");
            rd_word; chk(got === mk(8'h40), "34f ★端口 4 内容正确");
            rd_pstat; chk(pstat[4] === 1'b1, "34g ★位图 has_vec[4] 置位（NP=3 时这一位根本不存在）");
            do_release;
            chk(dut.pcnt[4] === 4'd0, "34h ★释放后端口 4 归零");
            feed(3'd5, 16'h2711, 8, 8'h80, 32'h0A000042, 16'h2711); do_commit;
            chk(dut.pcnt[5] === 4'd1, "34i ★端口 5（保留/闲置实例）仍能收下消息");
            sel_port(3'd5);
            rd_len;  chk(lenr === 16'd8,    "34j ★端口 5 长度正确");
            rd_word; chk(got === mk(8'h80), "34k ★端口 5 内容正确");
            rd_pstat; chk(pstat[5] === 1'b1, "34l ★位图 has_vec[5] 置位");
            do_release;
            chk(dut.pcnt[5] === 4'd0, "34m ★释放后端口 5 归零");
        end


        begin : t35
            integer q5;
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            chk(dut.queued[1] === 12'd0, "35a 起手：端口 1 已清空（对照）");
            // 先用一条 3000 字节（750 字）的消息占位，让剩余空间 = 2048-750 = 1298 字
            feed(2'd1, 16'h0101, 3000, 8'h40, 32'h0A000001, 16'h0BB8); do_commit;
            chk(dut.queued[1] === 12'd750, "35b 占位消息占 750 字");

            for (q5 = 0; q5 < 5196; q5 = q5 + 1) begin
                @(negedge clk);
                in_valid = 1; in_sop = (q5 == 0); in_eop = (q5 == 5195);
                in_data  = 8'h80 + q5[7:0];
                port_i = 2'd1; dport_i = 16'h0202;
                sip_i = 32'h0A000002; sport_i = 16'h0BB8;
                @(posedge clk);
            end
            @(negedge clk); in_valid = 0; in_sop = 0; in_eop = 0;
            rx_commit = 1;
            @(posedge clk);
            @(negedge clk); rx_commit = 0;
            repeat (4) @(posedge clk);
            chk(dut.pcnt[1] === 4'd1,       "35c ★★放不下的消息不发布：pcnt 仍是 1（只有占位那条）");
            chk(dut.queued[1] === 12'd750,  "35d ★★账目也没有按已发布更新：queued 仍是 750 字");
            rd_pstat; chk(pstat[NPT+1] === 1'b1, "35e ★★这次丢弃记成可见的溢出");
            // 对照：腾出空间后同样的消息能进去（且账目正确）
            do_release;
            chk(dut.queued[1] === 12'd0, "35f 释放占位消息后清空（对照）");
        end


        begin : t36
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            net_i = 1'b1;                                    // B 网
            feed(2'd1, 16'h0BB9, 8, 8'hE0, 32'h0A000003, 16'h1234); do_commit;
            sel_port(2'd1); rd_net;
            chk(netr === 16'h0001, "36a ★B 网消息的网络号读回 = 1");
            do_release;
            net_i = 1'b0;                                    // A 网（对照）
            feed(2'd1, 16'h0BB9, 8, 8'hE8, 32'h0A000003, 16'h1234); do_commit;
            sel_port(2'd1); rd_net;
            chk(netr === 16'h0000, "36b ★A 网消息的网络号读回 = 0（对照）");
            do_release;
        end


        begin : t37
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            net_i = 1'b0;
            // A 帧 8 字节（不判决，cur_full 保持 1）
            feed(2'd1, 16'h0456, 8, 8'hA0, 32'h0A000001, 16'h1000);
            // B 帧 4 字节（在 A 未判决时到达）
            feed(2'd1, 16'h0456, 4, 8'hB0, 32'h0A000001, 16'h1000);
            do_commit;                       // A 的判决（晚于 B 的 sop/eop）
            sel_port(2'd1); rd_len;
            chk(lenr === 16'd8, "37a A len not polluted");
            chk(dut.pcnt[1] === 4'd1, "37b only A published");
            // B 是重复副本（abort）：不能多发布一条，也不该算溢出
            do_abort;
            chk(dut.pcnt[1] === 4'd1, "37c abort(B) no extra msg");
            chk(dut.povf[1] === 1'b0, "37d abort(B) no overflow");
            do_release;

            // B 是被接受的新帧（commit）→ 它确实被丢了，必须可见（溢出）
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            feed(2'd1, 16'h0456, 8, 8'hC0, 32'h0A000001, 16'h1000);
            feed(2'd1, 16'h0456, 4, 8'hD0, 32'h0A000001, 16'h1000);
            do_commit;                       // A
            sel_port(2'd1); rd_len;
            chk(lenr === 16'd8, "37e A len still 8");
            do_commit;                       // B（被接受，但已被丢）
            chk(dut.pcnt[1] === 4'd1, "37f commit(B) no extra msg");
            chk(dut.povf[1] === 1'b1, "37g commit(B) overflow visible");
            do_release;
        end


        begin : t38
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            net_i = 1'b0;
            feed(2'd1, 16'h0456, 8, 8'h10, 32'h0A000001, 16'h1000);   // A
            feed(2'd1, 16'h0456, 4, 8'h20, 32'h0A000001, 16'h1000);   // B（丢）
            feed(2'd1, 16'h0456, 6, 8'h30, 32'h0A000001, 16'h1000);   // C（丢）
            feed(2'd1, 16'h0456, 5, 8'h40, 32'h0A000001, 16'h1000);   // D（丢）
            do_commit;                        // A
            sel_port(2'd1); rd_len;
            chk(lenr === 16'd8, "38a A len = 8");
            chk(dut.pcnt[1] === 4'd1, "38b only A published");
            do_abort;                         // B：副本 → 不报溢出
            chk(dut.povf[1] === 1'b0, "38c abort(B) no overflow");
            do_commit;                        // C：本来接受 → 可见溢出
            chk(dut.povf[1] === 1'b1, "38d commit(C) overflow");
            do_abort;                         // D
            chk(dut.pcnt[1] === 4'd1, "38e still only A");
            do_release;
        end


        begin : t39
            sel_port(2'd1); while (dut.pcnt[1] != 0) do_release;
            net_i = 1'b0;
            feed(2'd1, 16'h0456, 8, 8'h50, 32'h0A000001, 16'h1000);   // A
            feed(2'd1, 16'h0456, 4, 8'h60, 32'h0A000001, 16'h1000);   // B（丢）
            do_commit;                        // A
            // 同拍：C 的 sop + B 的 commit
            @(negedge clk);
            in_valid = 1; in_sop = 1; in_eop = 0; in_data = 8'hC0;
            port_i = 2'd1; dport_i = 16'h0456; sip_i = 32'h0A000001; sport_i = 16'h1000;
            rx_commit = 1;
            @(posedge clk);
            @(negedge clk);
            in_sop = 0; in_eop = 1; in_data = 8'hC1; rx_commit = 0;
            @(posedge clk);
            @(negedge clk); in_valid = 0; in_eop = 0;
            repeat (4) @(posedge clk);
            chk(dut.drop_cnt === 8'd1, "39d drop_cnt net = 1");
            do_commit;                        // C（被丢，但本来接受）
            sel_port(2'd1); rd_len;
            chk(lenr === 16'd8, "39a A len = 8");
            chk(dut.pcnt[1] === 4'd1, "39b only A published");
            chk(dut.povf[1] === 1'b1, "39c commit(C) overflow paired");
            do_release;
        end


        begin : t40
            integer L, w, j, wn, bad_len, bad_word;
            reg [31:0] expw, gw;
            sel_port(2'd1);
            while (dut.pcnt[1] != 0) do_release;
            bad_len = 0; bad_word = 0;
            for (L = 1; L <= 48; L = L + 1) begin
                feed(2'd1, 16'h0456, L, 8'h10, 32'h0A000001, 16'h1000);
                do_commit;
                sel_port(2'd1);
                rd_len;
                if (lenr !== L[15:0]) begin
                    bad_len = bad_len + 1;
                    $display("  [dbg] 40 len=%0d 回读=%0d", L, lenr);
                end
                wn = (L + 3) / 4;
                for (w = 0; w < wn; w = w + 1) begin
                    expw = 32'h0;
                    for (j = 0; j < 4; j = j + 1)
                        if (4*w + j < L) expw[8*j +: 8] = 8'h10 + ((4*w + j) & 8'hFF);
                    rd_word;
                    gw = got;
                    if (gw !== expw) begin
                        bad_word = bad_word + 1;
                        if (bad_word < 8)
                            $display("  [dbg] 40 len=%0d 字%0d got=%08x exp=%08x", L, w, gw, expw);
                    end
                end
                do_release;
            end
            chk(bad_len == 0,  "40a len 1..48 回读全部正确");
            chk(bad_word == 0, "40b len 1..48 每个字都已定义且内容正确（无 X）");
        end

        $display("====================================================");
        if (errs == 0) $display(">> PASS: 队列/采样/SAP源地址/深度溢出/禁用/abort/跨端口（NP=6 生产规模） 全部符合 P7 §3.3.1");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin

        #20000000;
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
