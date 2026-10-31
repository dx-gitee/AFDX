// tb_axi_slave.v —— AXI-Lite 从机 + 真实端口层 的集成测试
`timescale 1ns/1ps

module tb_axi_slave;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    integer errs = 0;
    task chk;
        input cond; input [255:0] name;
        begin
            if (cond) $display("  [ok]   %0s", name);
            else begin errs = errs + 1; $display("  [FAIL] %0s", name); end
        end
    endtask

    // ---------------- AXI 主机模型 ----------------
    reg  [31:0] awaddr = 0; reg awvalid = 0;
    wire        awready_w;
    wire        wready_w;
    reg  [31:0] wdata  = 0; reg wvalid  = 0;
    reg  [31:0] araddr = 0; reg arvalid = 0;
    wire [31:0] rdata_w;
    wire        bvalid_w, rvalid_w;
    reg         bready = 1, rready = 1;
    reg  [31:0] rval;


    integer last_gw;                       // 最近一次事务等了多少拍（给"不挂死"断言用）
    task axi_write;
        input [31:0] a; input [31:0] d;
        integer gw;
        begin
            @(negedge clk); awaddr = a; awvalid = 1; wdata = d; wvalid = 1;
            gw = 0;
            while (!bvalid_w && gw < 200) begin @(negedge clk); gw = gw + 1; end
            if (!bvalid_w) begin
                errs = errs + 1;
                $display("  [FAIL] axi_write 超时：从机 %0d 拍内没回 B（awaddr=%08x）", 200, a);
            end
            last_gw = gw;
            @(negedge clk); awvalid = 0; wvalid = 0;
            @(posedge clk);                    // bready=1，消费掉 B
            @(negedge clk);
        end
    endtask

    task axi_read;
        input [31:0] a;
        integer gw;
        begin
            @(negedge clk); araddr = a; arvalid = 1;
            gw = 0;
            while (!rvalid_w && gw < 200) begin @(negedge clk); gw = gw + 1; end
            if (!rvalid_w) begin
                errs = errs + 1;
                $display("  [FAIL] axi_read 超时：从机 %0d 拍内没回 R（araddr=%08x）", 200, a);
            end
            last_gw = gw;
            @(negedge clk); arvalid = 0;
            rval = rdata_w;                    // rvalid 高期间 rdata 稳定
            @(posedge clk);
            @(negedge clk);
        end
    endtask

    // ---------------- 端口层数据面 ----------------
    reg        in_valid = 0, in_sop = 0, in_eop = 0;
    reg  [7:0] in_data = 0;
    reg  [2:0] port_i = 0;
    reg  [15:0] dport_i = 0;
    reg  [31:0] sip_i = 0;
    reg  [15:0] sport_i = 0;
    reg        rx_commit = 0, rx_abort = 0;
    wire       rx_any, rx_ovf_any;
    wire [4:0] rx_has_o, rx_ovf_o;

    wire        px_wr, px_rd;  wire [2:0] px_reg;  wire [31:0] px_wdata;
    wire        py_rd, py_wr;  wire [2:0] py_sel;  wire [31:0] py_wdata;
    wire [2:0] rx_pi; wire rx_pi_en;
    wire [31:0] px_rdata;      wire [15:0] py_rdata;
    wire        tx_wr;  wire [2:0] tx_pi;  wire [2:0] tx_reg;  wire [31:0] tx_wdata;

    afdx_rx_ports #(.NP(5), .NPW(3), .PW(10)) u_ports (
        .clk(clk), .reset(reset),
        .wr(px_wr), .rd(px_rd), .reg_sel(px_reg), .wdata(px_wdata), .rdata(px_rdata),
        .pwr(py_wr), .prd(py_rd), .psel(py_sel), .pwdata(py_wdata), .prdata(py_rdata),
        .cpu_pi(rx_pi), .cpu_sel_en(rx_pi_en),
        .in_valid(in_valid), .in_sop(in_sop), .in_eop(in_eop), .in_data(in_data),
        .port_i(port_i), .dport_i(dport_i), .sip_i(sip_i), .sport_i(sport_i),
        .net_i(1'b0),
        .rx_commit(rx_commit), .rx_abort(rx_abort),
        .rx_any(rx_any), .rx_ovf_any(rx_ovf_any),
        .rx_has_o(rx_has_o), .rx_ovf_o(rx_ovf_o)
    );

    wire irq;
    afdx_axi_slave #(.NP(5), .NPW(3), .NPTX(5)) u_dut (
        .clk(clk), .reset(reset),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready_w),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready_w),
        .s_axi_bresp(), .s_axi_bvalid(bvalid_w), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(),
        .s_axi_rdata(rdata_w), .s_axi_rresp(), .s_axi_rvalid(rvalid_w), .s_axi_rready(rready),
        .rx_pi(rx_pi), .rx_pi_en(rx_pi_en),
        .px_wr(px_wr), .px_rd(px_rd), .px_reg(px_reg), .px_wdata(px_wdata), .px_rdata(px_rdata),
        .py_rd(py_rd), .py_wr(py_wr), .py_sel(py_sel), .py_wdata(py_wdata), .py_rdata(py_rdata),
        .tx_wr(tx_wr), .tx_pi(tx_pi), .tx_reg(tx_reg), .tx_wdata(tx_wdata), .tx_rdata(32'h0),
        .rx_has(rx_has_o), .rx_ovf(rx_ovf_o), .irq(irq)
    );


    // 地址：端口 n 基址 0x40*n；寄存器字偏移见 afdx_axi_slave 头注释
    `define PA(n,r) (32'h0 + ((n)<<6) + ((r)<<2))
    localparam R_DATA=0, R_LEN=1, R_PORTID=2, R_STATUS=3,
               R_CTRL=4, R_SRCPT=5, R_SIPLO=6, R_SIPHI=7, R_PCFG=12;

    function [31:0] mk;
        input [7:0] b;
        mk = {b+8'd3, b+8'd2, b+8'd1, b};
    endfunction

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

    integer i;

    integer n_w_hs = 0;   /* wvalid && wready 的拍数（握手必须真的发生）*/
    always @(posedge clk) if (wvalid === 1'b1 && wready_w === 1'b1) n_w_hs = n_w_hs + 1;

    initial begin
        #200 reset = 0;
        repeat(5) @(posedge clk);
        $display("");
        $display("======== AXI-Lite 从机 + 端口层 集成测试 ========");

        // 采样口（0）一条 8 字节；队列口（1）两条 8 字节；SAP（2）一条带源地址
        feed(2'd0, 16'h0400, 12, 8'hA0, 0, 0);       do_commit;   // 12 字节=3 字（好让 03b 有第 3 字可读）
        feed(2'd1, 16'h0401, 8, 8'hB0, 0, 0);        do_commit;
        feed(2'd1, 16'h0401, 8, 8'hB8, 0, 0);        do_commit;
        feed(2'd2, 16'h0045, 8, 8'hC0, 32'hC0A80164, 16'h1F90); do_commit;

        // ---------- 01 采样口（端口 0）----------
        axi_read(`PA(0,R_STATUS));
        chk(rval[0] === 1'b1, "01a 采样口 STATUS.bit0 = 有数据");
        axi_read(`PA(0,R_LEN));
        chk(rval === 32'd12, "01b 采样口 LEN = 12");
        axi_read(`PA(0,R_PORTID));
        chk(rval[15:0] === 16'h0400, "01c 采样口 PORTID = 0x0400");
        axi_read(`PA(0,R_DATA));
        chk(rval === mk(8'hA0), "01d 采样口 DATA 第 1 字正确");
        axi_read(`PA(0,R_DATA));
        chk(rval === mk(8'hA4), "01e 采样口 DATA 第 2 字正确（地址自动前移）");

        // ---------- 03 地址选端口：端口 1 不会串到端口 0 ----------
        axi_read(`PA(1,R_DATA));
        chk(rval === mk(8'hB0), "03a 切到端口 1 读到的是它自己的第 1 条（不串端口）");
        axi_read(`PA(0,R_DATA));
        chk(rval === mk(8'hA8), "03b 切回端口 0 仍从它自己的第 3 字继续（各自独立）");

        // ---------- 02 队列口（端口 1）：CTRL 释放后第二条接上 ----------
        axi_read(`PA(1,R_LEN));
        chk(rval === 32'd8, "02a 队列口 LEN = 8");
        axi_read(`PA(1,R_STATUS));
        chk(rval[1] === 1'b0, "02b 队列口尚未溢出");
        axi_write(`PA(1,R_CTRL), 32'h1);                 // 释放第 1 条
        chk(n_w_hs >= 1, "00 ★AW/W 同拍到达时 W 握手真的发生（wready 对主机可见）");
        axi_read(`PA(1,R_DATA));
        chk(rval === mk(8'hB8), "02c 释放后读到第 2 条（FIFO 顺序对）");
        axi_read(`PA(1,R_DATA));
        chk(rval === mk(8'hBC), "02d 第 2 条第 2 字正确");

        // ---------- 04 SAP 口（端口 2）：源 IP / 源端口 ----------
        axi_read(`PA(2,R_SRCPT));
        chk(rval[15:0] === 16'h1F90, "04a SAP 口：源 UDP 端口 = 0x1F90");
        axi_read(`PA(2,R_SIPLO));
        chk(rval[15:0] === 16'h0164, "04b SAP 口：源 IP 低 16 位 = 0x0164");
        axi_read(`PA(2,R_SIPHI));
        chk(rval[15:0] === 16'hC0A8, "04c SAP 口：源 IP 高 16 位 = 0xC0A8");
        axi_read(`PA(2,R_DATA));
        chk(rval === mk(8'hC0), "04d SAP 口：载荷正确");

        // ---------- 05 IRQ_EN + irq 输出 ----------
        axi_read(32'h404);
        chk(rval[4:0] === 5'b00000, "05a 复位后 IRQ_EN = 0，irq 不抬");
        chk(irq === 1'b0, "05b IRQ_EN=0 时 irq = 0");
        axi_write(32'h404, 32'h1);
        axi_read(32'h404);
        chk(rval[4:0] === 5'b00001, "05c IRQ_EN 可写会读（bit0 置起）");
        chk(irq === 1'b1, "05d 使能后 irq 真的拉高（端口 0 有数据且已使能）");

        // ---------- 06 G_STATUS 位图 ----------
        axi_read(32'h408);
        chk(rval[4:0] === rx_has_o, "06a G_STATUS 低 5 位 = 各端口有数据位图（与端口层一致）");
        chk(rval[9:5] === rx_ovf_o, "06b G_STATUS 高 5 位 = 各端口溢出位图（原来从未验过）");

        // ---------- 07 未映射地址不挂死 ----------
        axi_write(32'h3C0, 32'h1234);              // 0x3C0 的 addr[8:6]=7 → 端口 7 不存在（NP=5）
        axi_read (32'h3C0);
        $display("  [info] 未映射地址读回 %08x", rval);

        chk(rval === 32'h0 && last_gw < 200,
            "07 未映射地址读回 0 且在限定拍数内返回（总线不挂死）");


        feed(3'd4, 16'h0BC5, 8, 8'hE0, 0, 0); do_commit;
        axi_read(`PA(4,R_LEN));
        chk(rval === 32'd8, "08a ★端口 4 可寻址且 LEN=8");
        axi_read(`PA(4,R_PORTID));
        chk(rval[15:0] === 16'h0BC5,
            "08b ★端口 4 的 PORTID=0x0BC5（不是端口 0 的 0x0400 → 没有被截断/别名）");


        axi_read(`PA(0,R_PCFG));
        chk(rval[7:0] === 8'h43, "09a 端口 0 默认配置 = 采样/使能/深度 8（读回 0x43）");
        axi_read(`PA(1,R_PCFG));
        chk(rval[7:0] === 8'h45, "09b 端口 1 默认配置 = 队列/使能/深度 8（读回 0x45）");
        axi_read(`PA(3,R_PCFG));
        chk(rval[7:0] === 8'h47, "09c 端口 3 默认配置 = SAP/使能/深度 8（读回 0x47）");

        //   ① 禁用端口 3 → 送一条并 commit，不应交付
        axi_write(`PA(3,R_PCFG), 32'h00);          // en=0 type=0 depth=0
        axi_read(`PA(3,R_PCFG));
        chk(rval[7:0] === 8'h00, "09d ★写 PCFG 后读回同源（禁用生效，不是只回显）");
        feed(3'd3, 16'h7777, 8, 8'hF0, 0, 0); do_commit;
        axi_read(`PA(3,R_STATUS));
        chk(rval[0] === 1'b0, "09e ★禁用的端口不交付消息（STATUS.bit0=0）");

        //   ② 重新启用为 SAP → 同一条路径应恢复交付
        axi_write(`PA(3,R_PCFG), 32'h47);          // SAP/en/depth 8
        axi_read(`PA(3,R_PCFG));
        chk(rval[7:0] === 8'h47, "09f 重新启用后读回 0x47");
        feed(3'd3, 16'h7777, 8, 8'hF8, 0, 0); do_commit;
        axi_read(`PA(3,R_STATUS));
        chk(rval[0] === 1'b1, "09g ★重新启用后消息正常交付");
        axi_write(`PA(3,R_CTRL), 32'h1);           // 释放（队列口也必须释放才能改类型）
        @(posedge clk);

        //   ③ 改成"队列口、深度 1" → 连送两条，只应交付 1 条且溢出位置起
        axi_write(`PA(3,R_PCFG), 32'h0D);          // type=2 queue, depth=1, en=1
        axi_read(`PA(3,R_PCFG));
        chk(rval[7:0] === 8'h0D, "09h ★队列类型/深度 1 写进去读得回（0x0D）");
        feed(3'd3, 16'h7788, 8, 8'h10, 0, 0); do_commit;
        feed(3'd3, 16'h7788, 8, 8'h18, 0, 0); do_commit;
        axi_read(`PA(3,R_STATUS));
        chk(rval[0] === 1'b1 && rval[1] === 1'b1,
            "09i ★深度 1 时第二条被丢且溢出位置起（bit0=1 bit1=1）");
        axi_write(`PA(3,R_CTRL), 32'h1);           // 释放后可读下一条（这里已被丢，队列空）
        @(posedge clk);
        axi_read(`PA(3,R_STATUS));
        chk(rval[0] === 1'b0 && rval[1] === 1'b0,
            "09j 释放后队列空、溢出位随释放清除");

        //   ④ 配置一个端口不能波及别的端口
        axi_read(`PA(0,R_PCFG));
        chk(rval[7:0] === 8'h43, "09k ★配置端口 3 不影响端口 0（仍是 0x43）");

        $display("===============================================");
        if (errs == 0) $display(">> PASS: AXI 从机译码/选端口/读寄存器/释放/源地址/中断 全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #1000000;
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
