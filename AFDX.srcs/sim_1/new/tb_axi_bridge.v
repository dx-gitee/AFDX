// tb_axi_bridge.v —— AXI-Lite 主机桥单独测试（按约定：先单独测通再往上装）
`timescale 1ns/1ps

module tb_axi_bridge;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;          // 100 ns 周期，方便数拍

    integer errs = 0;
    task chk;
        input cond; input [255:0] name;
        begin
            if (cond) $display("  [ok]   %0s", name);
            else begin errs = errs + 1; $display("  [FAIL] %0s", name); end
        end
    endtask

    //=================================================================
    // 核侧主设备模型
    //=================================================================
    reg         DDREQ = 0;
    reg  [31:0] DADDR = 0, DATAO = 0;
    reg  [3:0]  DBE   = 4'hF;
    reg  [2:0]  DLEN  = 3'd2;       // word
    reg         DWR   = 0;
    wire        DDACK;
    wire [31:0] DATAI;
    integer     ack_cycles;
    integer     max_ack_run = 0;
    always @(posedge clk) if (ack_cycles > max_ack_run) max_ack_run <= ack_cycles;

    // 统计 DDACK 连续高的拍数（用例 07）
    always @(posedge clk) begin
        if (reset) ack_cycles <= 0;
        else if (DDACK) ack_cycles <= ack_cycles + 1;
        else            ack_cycles <= 0;
    end

    task bus_write;
        input [31:0] a; input [31:0] d; input [3:0] be;
        begin
            @(negedge clk);
            DDREQ = 1; DADDR = a; DATAO = d; DBE = be; DWR = 1; DLEN = 3'd2;
            @(posedge clk);
            while (!DDACK) @(posedge clk);
            @(negedge clk); DDREQ = 0; DBE = 4'hF;
            @(posedge clk);
        end
    endtask

    reg [31:0] rd_val;
    task bus_read;
        input [31:0] a;
        begin
            @(negedge clk);
            DDREQ = 1; DADDR = a; DWR = 0; DLEN = 3'd2; DBE = 4'hF;
            @(posedge clk);
            while (!DDACK) @(posedge clk);
            rd_val = DATAI;                     // 与 DDACK 同拍
            @(negedge clk); DDREQ = 0;
            @(posedge clk);
        end
    endtask


    reg [31:0] b2b_rd [0:15];
    integer    b2b_tmo;
    task bus_read_b2b;
        input integer n; input [31:0] base;
        integer k, w;
        begin
            b2b_tmo = 0;
            @(negedge clk);
            DDREQ = 1; DWR = 0; DLEN = 3'd2; DBE = 4'hF;
            for (k = 0; k < n; k = k + 1) begin
                DADDR = base + k*4;              // 每笔换地址，DDREQ **保持 1**
                w = 0;
                @(posedge clk);
                while (!DDACK && w < 300) begin @(posedge clk); w = w + 1; end
                if (w >= 300) b2b_tmo = b2b_tmo + 1;   // 这一笔没等到 ACK
                b2b_rd[k] = DATAI;
                @(negedge clk);
            end
            DDREQ = 0;
            @(posedge clk);
        end
    endtask

    //=================================================================
    // AXI-Lite 从设备模型（带可编程反压）
    //=================================================================
    wire [31:0] awaddr;  wire awvalid; reg awready;
    wire [31:0] wdata;   wire [3:0] wstrb; wire wvalid; reg wready;
    reg  [1:0]  bresp;   reg  bvalid;  wire bready;
    wire [31:0] araddr;  wire arvalid; reg arready;
    reg  [31:0] rdata;   reg  rvalid;  wire rready;

    // 反压参数：把 ready/valid 拉低 N 拍再抬起来
    integer bp_aw = 0, bp_w = 0, bp_b = 0, bp_ar = 0, bp_r = 0;

    axi_lite_master_bridge u_dut (
        .clk(clk), .reset(reset),
        .DDREQ(DDREQ), .DADDR(DADDR), .DATAO(DATAO), .DBE(DBE), .DLEN(DLEN),
        .DWR(DWR), .DRD(~DWR), .DDACK(DDACK), .DATAI(DATAI),
        .m_axi_awaddr(awaddr), .m_axi_awvalid(awvalid), .m_axi_awready(awready),
        .m_axi_wdata(wdata), .m_axi_wstrb(wstrb), .m_axi_wvalid(wvalid), .m_axi_wready(wready),
        .m_axi_bresp(bresp), .m_axi_bvalid(bvalid), .m_axi_bready(bready),
        .m_axi_araddr(araddr), .m_axi_arvalid(arvalid), .m_axi_arready(arready),
        .m_axi_rdata(rdata), .m_axi_rresp(2'b00), .m_axi_rvalid(rvalid), .m_axi_rready(rready)
    );

    // ---- 从设备存储 ----
    reg [31:0] mem [0:255];
    integer k;
    initial for (k = 0; k < 256; k = k + 1) mem[k] = 32'hDEAD_0000 + k;

    // ---- 写通道 FSM（AW → W → B，每段可编程反压）----
    reg [31:0] aw_addr_q;
    reg [1:0]  wst;
    integer    c1, c2, c3;
    localparam W_AW = 2'd0, W_W = 2'd1, W_B = 2'd2;
    always @(posedge clk) begin
        if (reset) begin
            awready <= 0; wready <= 0; bvalid <= 0;
            wst <= W_AW; c1 <= 0; c2 <= 0; c3 <= 0; aw_addr_q <= 0;
        end else begin
            case (wst)
                W_AW: begin
                    bvalid <= 1'b0;
                    if (c1 >= bp_aw) awready <= 1'b1;
                    else             c1 <= c1 + 1;
                    if (awvalid && awready) begin
                        aw_addr_q <= awaddr; awready <= 1'b0; c1 <= 0; wst <= W_W;
                    end
                end
                W_W: begin
                    if (c2 >= bp_w) wready <= 1'b1;
                    else            c2 <= c2 + 1;
                    if (wvalid && wready) begin
                        if (wstrb[0]) mem[aw_addr_q[9:2]][7:0]   <= wdata[7:0];
                        if (wstrb[1]) mem[aw_addr_q[9:2]][15:8]  <= wdata[15:8];
                        if (wstrb[2]) mem[aw_addr_q[9:2]][23:16] <= wdata[23:16];
                        if (wstrb[3]) mem[aw_addr_q[9:2]][31:24] <= wdata[31:24];
                        wready <= 1'b0; c2 <= 0; c3 <= 0; wst <= W_B;
                    end
                end
                default: begin                                   // W_B
                    if (c3 >= bp_b) bvalid <= 1'b1;
                    else            c3 <= c3 + 1;
                    if (bvalid && bready) begin bvalid <= 1'b0; wst <= W_AW; end
                end
            endcase
        end
    end


    reg [31:0] ar_addr_q;
    reg [1:0]  rstt;
    integer    c4, c5;
    always @(posedge clk) begin
        if (reset) begin
            arready <= 0; rvalid <= 0; rstt <= 2'd0; c4 <= 0; c5 <= 0; ar_addr_q <= 0;
        end else begin
            case (rstt)
                2'd0: begin                                   // AR 握手（可编程延迟）
                    if (c4 >= bp_ar) arready <= 1'b1;
                    else             c4 <= c4 + 1;
                    if (arvalid && arready) begin
                        ar_addr_q <= araddr; arready <= 1'b0; c4 <= 0; c5 <= 0; rstt <= 2'd1;
                    end
                end
                2'd1: begin                                   // 取数延迟（可编程）
                    if (c5 >= bp_r) begin
                        rdata  <= mem[ar_addr_q[9:2]];         // 与 rvalid 同拍寄存 ✓
                        rvalid <= 1'b1;
                        rstt   <= 2'd2;
                    end else c5 <= c5 + 1;
                end
                default: begin                                // R 握手
                    if (rvalid && rready) begin rvalid <= 1'b0; rstt <= 2'd0; end
                end
            endcase
        end
    end


    integer i;
    initial begin
        #200 reset = 0;
        repeat(4) @(posedge clk);
        $display("");
        $display("======== AXI-Lite 主机桥 单独测试 ========");

        // ---- 01 基本 写→读 ----
        bus_write(32'h0000_0010, 32'h1234_5678, 4'hF);
        $display("  [dbg] 写后 mem[4]=%08x  bridge.st=%0d wst=%0d rst_r=%0d rvalid=%b",
                 mem[4], u_dut.st, wst, rstt, rvalid);
        bus_read (32'h0000_0010);
        $display("  [dbg] 读后 rd_val=%08x  bridge.st=%0d", rd_val, u_dut.st);
        chk(rd_val === 32'h1234_5678, "01 写一个字再读回，值正确");

        // ---- 02 字节使能 ----
        bus_write(32'h0000_0020, 32'hFFFF_FFFF, 4'hF);   // 先铺满
        bus_write(32'h0000_0020, 32'h0000_ABCD, 4'b0011); // 只写低 2 字节
        bus_read (32'h0000_0020);
        chk(rd_val === 32'hFFFF_ABCD, "02 DBE=0011 只改低 2 字节，高 2 字节不变");

        // ---- 03 背靠背连续读写 ----
        for (i = 0; i < 8; i = i + 1) bus_write(32'h20 + i*4, 32'hA000_0000 + i, 4'hF);
        for (i = 0; i < 8; i = i + 1) begin
            bus_read(32'h20 + i*4);
            if (rd_val !== (32'hA000_0000 + i)) begin
                $display("       [i=%0d] 读到 %08x", i, rd_val);
                errs = errs + 1;
            end
        end
        chk(errs == 0, "03 连续 8 笔写 + 8 笔读全部正确（每笔之间撤销 DDREQ）");


        for (i = 0; i < 8; i = i + 1) bus_write(32'h40 + i*4, 32'hB000_0000 + i, 4'hF);
        bus_read_b2b(8, 32'h40);
        chk(b2b_tmo == 0, "03b ★DDREQ 全程高时 8 笔读全部拿到 ACK（不卡死）");
        begin : b2b_data
            integer bad;
            bad = 0;
            for (i = 0; i < 8; i = i + 1)
                if (b2b_rd[i] !== (32'hB000_0000 + i)) begin
                    $display("       [i=%0d] 读到 %08x 期望 %08x", i, b2b_rd[i], 32'hB000_0000+i);
                    bad = bad + 1;
                end
            chk(bad == 0, "03c ★背靠背的数据也全对（不是只 ACK 了就完事）");
        end

        // ---- 04 无字节使能的写：不产生 AXI 事务，直接 ACK ----
        bus_read(32'h0000_0030);
        begin : noop_test
            reg [31:0] before;
            before = rd_val;
            bus_write(32'h0000_0030, 32'hBADD_BADD, 4'h0);
            bus_read(32'h0000_0030);
            chk(rd_val === before, "04 DBE=0 的写直接 ACK，内存未被改动（没发 AXI 事务）");
        end

        // ---- 05 反压下的读 ----
        bp_ar = 3; bp_r = 5;
        bus_read(32'h0000_0010);
        chk(rd_val === 32'h1234_5678, "05 从设备把 ARREADY/RVALID 各拖几拍，读仍正确");
        bp_ar = 0; bp_r = 0;

        // ---- 06 反压下的写 ----
        bp_aw = 2; bp_w = 4;
        bus_write(32'h0000_0040, 32'hCAFE_BABE, 4'hF);
        bp_aw = 0; bp_w = 0;
        bus_read (32'h0000_0040);
        chk(rd_val === 32'hCAFE_BABE, "06 从设备把 AWREADY/WREADY 各拖几拍，写仍正确");

        // ---- 07 DDACK 正好 1 拍（一直在统计）----
        for (i = 0; i < 6; i = i + 1) bus_write(32'h50 + i*4, 32'h100 + i, 4'hF);
        for (i = 0; i < 6; i = i + 1) bus_read (32'h50 + i*4);
        chk(max_ack_run <= 1, "07 DDACK 全程不超过 1 拍（防核多发访问）");

        // ---- 08 地址互不串扰 ----
        bus_write(32'h0000_0060, 32'h1111_1111, 4'hF);
        bus_write(32'h0000_0064, 32'h2222_2222, 4'hF);
        bus_write(32'h0000_0068, 32'h3333_3333, 4'hF);
        bus_read (32'h0000_0068); chk(rd_val === 32'h3333_3333, "08a 地址 0x68 正确");
        bus_read (32'h0000_0060); chk(rd_val === 32'h1111_1111, "08b 地址 0x60 未被串扰");
        bus_read (32'h0000_0064); chk(rd_val === 32'h2222_2222, "08c 地址 0x64 未被串扰");

        $display("========================================");
        if (errs == 0) $display(">> PASS: AXI-Lite 主机桥（基本读写/字节使能/背靠背/反压/ACK 宽度）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #400000;
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
