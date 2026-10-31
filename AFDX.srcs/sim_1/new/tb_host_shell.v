// tb_host_shell.v —— 系统顶层 AFDX_host_shell 的冒烟测试
`timescale 1ns/1ps

module tb_host_shell #(
    parameter integer SOC_DLY = 400000,      // 3.2 ms @125 MHz（等 MAC 配置完成）
    parameter integer OBS     = 200000       // 放行后再观察 16 ms（等固件自检发帧）
) ();

    reg clk   = 1'b0;
    reg reset = 1'b1;
    always #4  clk = ~clk;                   // 125 MHz

    // ---- 模板 8 信号（本台不做回环，只看发送） ----
    reg  [7:0] rx_data  = 8'h00;
    reg        rx_valid = 1'b0;
    reg        rx_ready = 1'b1;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;

    // ---- 网络侧 / SPI ----
    wire [7:0] gmii_a_txd, gmii_b_txd;
    wire       gmii_a_txen, gmii_a_txer, gmii_a_gtxc;
    wire       gmii_b_txen, gmii_b_txer, gmii_b_gtxc;
    wire       mdc_a, mdc_b, phy_rstb_a, phy_rstb_b;
    wire       mdio_a, mdio_b;
    wire       fl_miso, fl_sclk, fl_mosi, fl_cs_n;
    wire [3:0] debug;
    wire       mac_cfg_done, soc_boot_done;
    pullup (fl_miso);                        // 无 flash 模型 → 读回全 1 → 无魔数 → 用预设表
    pullup (mdio_a);  pullup (mdio_b);

    // ---- 主机 AXI-Lite ----
    reg  [31:0] awaddr = 0; reg awvalid = 0;
    wire        awready;
    reg  [31:0] wdata = 0; reg wvalid = 0; reg [3:0] wstrb = 4'hF;
    wire        wready;
    wire [1:0]  bresp; wire bvalid; reg bready = 0;
    reg  [31:0] araddr = 0; reg arvalid = 0;
    wire        arready;
    wire [31:0] rdata; wire [1:0] rresp; wire rvalid; reg rready = 0;
    wire        host_irq;

    AFDX_host_shell #(.PORTS(2), .SOC_BOOT_DLY(SOC_DLY), .BYPASS(1)) u_dut (
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_ready), .rx_ready(rx_ready),
        .gmii_a_txd(gmii_a_txd), .gmii_a_txen(gmii_a_txen), .gmii_a_txer(gmii_a_txer),
        .gmii_a_gtxc(gmii_a_gtxc), .gmii_a_rxd(8'h00), .gmii_a_rxdv(1'b0),
        .gmii_a_rxer(1'b0), .gmii_a_rxc(clk),
        .gmii_b_txd(gmii_b_txd), .gmii_b_txen(gmii_b_txen), .gmii_b_txer(gmii_b_txer),
        .gmii_b_gtxc(gmii_b_gtxc), .gmii_b_rxd(8'h00), .gmii_b_rxdv(1'b0),
        .gmii_b_rxer(1'b0), .gmii_b_rxc(clk),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(fl_miso), .flash_cs_n(fl_cs_n),
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done),
        .host_awaddr(awaddr), .host_awvalid(awvalid), .host_awready(awready),
        .host_wdata(wdata), .host_wstrb(wstrb), .host_wvalid(wvalid), .host_wready(wready),
        .host_bresp(bresp), .host_bvalid(bvalid), .host_bready(bready),
        .host_araddr(araddr), .host_arvalid(arvalid), .host_arready(arready),
        .host_rdata(rdata), .host_rresp(rresp), .host_rvalid(rvalid), .host_rready(rready),
        .host_irq(host_irq)
    );

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin nchk = nchk + 1;
              if (c) $display("  [ok]   #%0d %0s", nchk, n);
              else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end end
    endtask

    // ---------------- AXI-Lite 主机任务 ----------------
    reg [31:0] rdv;
    reg [1:0]  rresp_v;

    task axi_write;
        input [31:0] a, d;
        begin
            @(negedge clk); awvalid = 1; awaddr = a; wvalid = 1; wdata = d; wstrb = 4'hF;
            while (!(awready && wready)) @(posedge clk);
            @(negedge clk); awvalid = 0; wvalid = 0;
            while (!bvalid) @(posedge clk);
            @(negedge clk); bready = 1;
            @(posedge clk); @(negedge clk); bready = 0;
        end
    endtask

    task axi_read;
        input [31:0] a;
        begin
            @(negedge clk); arvalid = 1; araddr = a;
            while (!arready) @(posedge clk);
            @(negedge clk); arvalid = 0;
            while (!rvalid) @(posedge clk);
            rdv = rdata; rresp_v = rresp;
            @(negedge clk); rready = 1;
            @(posedge clk); @(negedge clk); rready = 0;
        end
    endtask

    // 观察模板口上有没有帧（固件自检会发一条）
    integer ftx = 0;
    always @(posedge clk) if (tx_valid === 1'b1 && ftx < 3) ftx = ftx + 1;

    integer w;
    initial begin
        #200 reset = 0;
        repeat (4) @(posedge clk);
        $display("");
        $display("========== 系统顶层 AFDX_host_shell（模板口 + 主机 AXI，BYPASS=1）==========");

        // T1/T2 启动标志
        w = 0;
        while (mac_cfg_done !== 1'b1 && w < 400000) begin @(posedge clk); w = w + 1; end
        chk(mac_cfg_done === 1'b1, "T1 mac_cfg_done 拉起（MAC 上电配置完成）");
        w = 0;
        while (soc_boot_done !== 1'b1 && w < 400000) begin @(posedge clk); w = w + 1; end
        chk(soc_boot_done === 1'b1, "T2 soc_boot_done 拉起（BYPASS 模式不等 MAC）");

        // T3 主机 AXI：读 H_ID
        axi_read(32'h0000_0000);
        chk(rdv === 32'h484F5354, "T3 ★主机 AXI 读 H_ID = 0x484F5354（HOST）—— 顶层把 AXI 接到了邮箱");
        chk(rresp_v === 2'b00, "T3b 读响应 OKAY");

        // T4 写 H_CTRL（host_en=1）→ 再读 H_OUT_INFO / H_STATUS，事务都要完成
        axi_write(32'h0000_0008, 32'h0000_0004);
        chk(bresp === 2'b00, "T4a 主机 AXI 写 H_CTRL(host_en) 的 B 响应 OKAY");
        axi_read(32'h0000_0018);
        chk(rresp_v === 2'b00 && rdv[15:0] === 16'h0000, "T4b 读 H_OUT_INFO 正常（暂无消息 → 长度 0）");
        axi_read(32'h0000_0004);
        chk(rresp_v === 2'b00 && rdv[1:0] === 2'b00, "T4c 读 H_STATUS 正常（IN/OUT_VALID 都为 0）");

        // T5 模板口上出现帧（固件自检 → 发送通路 → 模板字节流）
        w = 0;
        while (ftx == 0 && w < OBS) begin @(posedge clk); w = w + 1; end
        chk(ftx > 0, "T5 ★模板字节流口出现 tx_valid 帧（数据通路活着，不是只「能编译」）");

        $display("===============================================");
        if (errs == 0) $display(">> PASS: AFDX_host_shell 冒烟（启动/主机 AXI/模板口）全部通过");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #30000000; $display(">> TIMEOUT errs=%0d ftx=%0d", errs, ftx); $finish; end
endmodule
