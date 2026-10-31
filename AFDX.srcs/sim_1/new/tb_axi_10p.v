// tb_axi_10p.v —— AXI 从机在生产规模（RX=6 / TX=10）下的端口语义
`timescale 1ns/1ps

module tb_axi_10p;

    reg clk = 1'b0, reset = 1'b1;
    always #5 clk = ~clk;                  // 100 MHz（本台只测协议/译码，与时基无关）
    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    // ---- AXI 主机侧 ----
    reg  [31:0] awaddr = 0; reg awvalid = 0; wire awready;
    reg  [31:0] wdata  = 0; reg wvalid  = 0; wire wready;
    wire        bvalid; reg bready = 1;
    reg  [31:0] araddr = 0; reg arvalid = 0; wire arready;
    wire [31:0] rdata; wire rvalid; reg rready = 1;

    // ---- 从机输出（接收侧 / 发送侧）----
    wire        px_wr, px_rd, py_wr, py_rd, tx_wr;
    wire [3:0]  rx_pi, tx_pi;
    wire        rx_pi_en;
    wire [2:0]  px_reg, py_sel, tx_reg;
    wire [31:0] px_wdata, py_wdata, tx_wdata;

    // px_wr 全程监视：端口 8/9 的"释放接收"绝不允许出现
    integer px_wr_cnt = 0;
    always @(posedge clk) if (px_wr) px_wr_cnt = px_wr_cnt + 1;

    afdx_axi_slave #(.NP(6), .NPW(4), .NPTX(10)) u_dut (
        .clk(clk), .reset(reset),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .rx_pi(rx_pi), .rx_pi_en(rx_pi_en),
        .px_wr(px_wr), .px_rd(px_rd), .px_reg(px_reg), .px_wdata(px_wdata), .px_rdata(32'hA5A5A5A5),
        .py_rd(py_rd), .py_wr(py_wr), .py_sel(py_sel), .py_wdata(py_wdata), .py_rdata(16'h5A5A),
        .tx_wr(tx_wr), .tx_pi(tx_pi), .tx_reg(tx_reg), .tx_wdata(tx_wdata),
        .tx_rdata({8'h5A, 8'h5A, 8'h00, tx_pi[3:0], 4'h0}),
        .rx_has(6'h3F), .rx_ovf(6'h00), .irq()
    );

    // ---- 简易 AXI 主机（串行发起，与本工程固件同款）----
    task axi_wr;
        input [31:0] a; input [31:0] d;
        begin
            @(negedge clk); awaddr = a; awvalid = 1;
            @(posedge clk); while (!awready) @(posedge clk);
            @(negedge clk); awvalid = 0;
            @(negedge clk); wdata = d; wvalid = 1;
            @(posedge clk); while (!wready) @(posedge clk);
            @(negedge clk); wvalid = 0;
            @(posedge clk); while (!bvalid) @(posedge clk);
            repeat (2) @(posedge clk);
        end
    endtask

    task axi_rd;
        input [31:0] a; output [31:0] d;
        begin
            @(negedge clk); araddr = a; arvalid = 1;
            @(posedge clk); while (!arready) @(posedge clk);
            @(negedge clk); arvalid = 0;
            @(posedge clk); while (!rvalid) @(posedge clk);
            d = rdata;
            repeat (2) @(posedge clk);
        end
    endtask

    // 端口 n 基址 = n*0x40（端口号 = addr[9:6]，寄存器 = addr[5:2]）
    `define PA(n,r) (((n)<<6) | ((r)<<2))
    localparam R_DATA=0, R_LEN=1, R_PORTID=2, R_STATUS=3, R_CTRL=4,
                   R_DIP=8, R_DPT=9, R_TXST=10;

    reg [31:0] d;

    // 监视 tx_wr / tx_pi，供"写到底有没有落到发送侧"判据
    reg        saw_tx = 0;
    reg [3:0]  saw_pi = 0;
    reg [2:0]  saw_reg = 0;
    always @(posedge clk) if (tx_wr) begin saw_tx = 1; saw_pi = tx_pi; saw_reg = tx_reg; end

    initial begin
        #100 reset = 1'b0;
        repeat (10) @(posedge clk);
        $display("");
        $display("======== AXI 从机：生产规模 RX=6 / TX=10 的端口语义 ========");

        // ---- 1. 端口 8（只发不收）：写 DATA 必须进**发送**侧 ----
        saw_tx = 0;
        axi_wr(`PA(8, R_DATA), 32'h1234_5678);
        repeat (6) @(posedge clk);
        chk(saw_tx && (saw_pi == 4'd8) && (saw_reg == 3'd0),
            "01 ★端口 8 写 DATA → 落到发送侧（tx_pi=8, reg=DATA）");

        // ---- 2. 端口 8 写 CTRL.bit0（释放接收）→ **不许**有任何 px_wr ----
        px_wr_cnt = 0;
        axi_wr(`PA(8, R_CTRL), 32'h1);
        repeat (10) @(posedge clk);
        chk(px_wr_cnt == 0, "02 ★端口 8 的『释放接收』不产生任何接收选通（只发不收）");

        // ---- 3. 端口 8 写 CTRL.bit1（提交发送）→ 必须进发送侧 ----
        saw_tx = 0;
        axi_wr(`PA(8, R_CTRL), 32'h2);
        repeat (10) @(posedge clk);
        chk(saw_tx && (saw_pi == 4'd8) && (saw_reg == 3'd2),
            "03 ★端口 8 的『提交发送』落到发送侧（tx_reg=CTRL）");

        // ---- 4. 端口 8 读 STATUS → 接收侧越界 ⇒ 返回确定 0（不是 0xA5A5A5A5）----
        axi_rd(`PA(8, R_STATUS), d);
        chk(d === 32'h0, "04 ★端口 8 读 STATUS 返回 0（不串到接收端口层）");

        // ---- 5. 端口 5 读 STATUS → 走接收侧（返回桩值 0xA5A5A5A5）----
        axi_rd(`PA(5, R_STATUS), d);
        // STATUS 在从机里会被**重排**成 {bit1=溢出, bit0=有数据}；
        // 桩值 0xA5A5A5A5 的 [3:2] = 2'b01 ⇒ 重排后 d = 1（这本身就证明"走了接收路径"）
        chk(d === 32'h1, "05 端口 5 读 STATUS 走接收路径（桩值重排后 = 1）");

        // ---- 6. 端口 8 读 TX_STATUS → 走**发送**状态（tx_rdata）----
        axi_rd(`PA(8, R_TXST), d);
        chk(d[31:24] === 8'h5A && d[23:16] === 8'h5A,
            "06 ★端口 8 读 TX_STATUS 走发送状态路径（tx_rdata）");


        saw_tx = 0;
        axi_wr(`PA(10, R_LEN), 32'd64);
        repeat (8) @(posedge clk);
        chk(!saw_tx, "06b ★越界端口 10 写 LEN 不产生发送选通");

        saw_tx = 0;
        axi_wr(`PA(10, R_DIP), 32'hC0A80101);           // 偏移 8
        repeat (8) @(posedge clk);
        chk(!saw_tx, "06c ★越界端口 10 写 DIP 不产生发送选通");

        saw_tx = 0;
        axi_wr(`PA(10, R_DPT), 32'h0000_1234);          // 偏移 9
        repeat (8) @(posedge clk);
        chk(!saw_tx, "06d ★越界端口 10 写 DPT 不产生发送选通");

        saw_tx = 0;
        axi_wr(`PA(10, R_CTRL), 32'h2);                // CTRL.bit1 = 提交发送
        repeat (8) @(posedge clk);
        chk(!saw_tx, "06e ★越界端口 10 的『提交发送』不产生发送选通");

        // ---- 6f. 端口 8（只发不收）读 LEN（接收侧附加窗口）必须返回 0 ----
        axi_rd(`PA(8, R_LEN), d);
        chk(d === 32'h0, "06f ★端口 8 读 LEN 返回 0（接收侧越界）");

        // ---- 7. 端口 10（越界）：写 DATA 不许产生 tx_wr，读回 0 ----
        saw_tx = 0;
        axi_wr(`PA(10, R_DATA), 32'hDEAD_BEEF);
        repeat (10) @(posedge clk);
        chk(!saw_tx, "07 ★越界端口 10 写 DATA 不产生发送选通");
        axi_rd(`PA(10, R_STATUS), d);
        chk(d === 32'h0, "08 越界端口 10 读回确定的 0");

        // ---- 9. 全局窗口（0x400）仍可写 IRQ_EN ----
        saw_tx = 0;
        axi_wr(32'h400, 32'h3F);
        repeat (10) @(posedge clk);
        chk(!saw_tx, "09 ★全局窗口 0x400 不被当成端口（不发任何选通）");


        saw_tx = 0;
        axi_wr(`PA(4, R_DATA), 32'h0BAD_F00D);
        repeat (6) @(posedge clk);
        chk(saw_tx && (saw_pi == 4'd4), "10 端口 4 写 DATA 落发送侧（对照）");
        axi_rd(`PA(4, R_STATUS), d);
        chk(d === 32'h1, "11 端口 4 读 STATUS 走接收路径（对照）");

        $display("===========================================================");
        if (errs == 0) $display(">> PASS: RX=6/TX=10 的端口语义正确（只发不收 / 越界 / 全局窗口）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #200000; $display(">> TIMEOUT"); $finish; end
endmodule
