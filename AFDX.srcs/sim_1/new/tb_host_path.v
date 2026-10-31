// tb_host_path.v —— AXI 主机数据面端到端（SoC + 固件 + 邮箱）
`timescale 1ns/1ps

module tb_host_path;
    reg clk = 0, reset = 1;
    always #4  clk = ~clk;   // 125 MHz

    // ---- SoC 帧流 ----
    reg        rx_fv = 0, rx_fsop = 0, rx_feop = 0, rx_fnet = 0;
    reg  [7:0] rx_fd = 0;
    reg        dp_fr_ready = 1;
    wire       dp_fr_valid, dp_fr_sop, dp_fr_eop, dp_fr_net;
    wire [7:0] dp_fr_data;
    wire       cfg_hit; wire [127:0] cfg_word;
    wire       debug_0, debug_1;
    wire [3:0] debug;
    wire       fl_sclk, fl_mosi, fl_miso, fl_cs_n;

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

    // 回环：SoC 发出的帧流送回接收（与 tb_darkriscv_soc 同款，延迟一拍）
    always @(posedge clk) begin
        rx_fd   <= dp_fr_data;
        rx_fv   <= dp_fr_valid;
        rx_fsop <= dp_fr_sop;
        rx_feop <= dp_fr_eop;
        rx_fnet <= dp_fr_net;
    end

    // SPI flash 模型（配置表镜像）
    spi_flash_model #(.AW(13), .USE_IMG(1)) u_flash (
        .sclk(fl_sclk), .cs_n(fl_cs_n), .mosi(fl_mosi), .miso(fl_miso)
    );

    darkriscv_soc_top u_dut (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk(clk), .reset(reset), .debug(debug),
        .dp_rxin_valid(rx_fv), .dp_rxin_sop(rx_fsop), .dp_rxin_eop(rx_feop),
        .dp_rxin_data(rx_fd), .dp_rxin_net(rx_fnet),
        .dp_fr_valid(dp_fr_valid), .dp_fr_ready(dp_fr_ready), .dp_fr_sop(dp_fr_sop),
        .dp_fr_eop(dp_fr_eop), .dp_fr_data(dp_fr_data), .dp_fr_net(dp_fr_net),
        .cfg_hit_o(cfg_hit), .cfg_cfg_o(cfg_word),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(fl_miso), .flash_cs_n(fl_cs_n),
        .ts_rx_ts(64'h0), .ts_rx_seq(16'h0), .ts_tx_ts(64'h0), .ts_tx_seq(16'h0),
        .ts_fine_o(),
        .ts_tx_vl_ts(32'h0), .ts_tx_vl_sel(),
        .host_awaddr(awaddr), .host_awvalid(awvalid), .host_awready(awready),
        .host_wdata(wdata), .host_wstrb(wstrb), .host_wvalid(wvalid), .host_wready(wready),
        .host_bresp(bresp), .host_bvalid(bvalid), .host_bready(bready),
        .host_araddr(araddr), .host_arvalid(arvalid), .host_arready(arready),
        .host_rdata(rdata), .host_rresp(rresp), .host_rvalid(rvalid), .host_rready(rready),
        .host_irq(host_irq)
    );

    // ---- 抓发送帧（SoC 的帧流） ----
    reg [7:0] fr [0:16383];
    integer nfr = 0;
    always @(posedge clk) if (dp_fr_valid && dp_fr_ready && nfr < 16383) begin
        fr[nfr] <= dp_fr_data; nfr <= nfr + 1;
    end

    function [31:0] dres; input [11:0] a;
        begin dres = {u_dut.dram3[a[11:2]], u_dut.dram2[a[11:2]],
                      u_dut.dram1[a[11:2]], u_dut.dram0[a[11:2]]}; end
    endfunction

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin nchk = nchk + 1;
              if (c) $display("  [ok]   #%0d %0s", nchk, n);
              else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end end
    endtask

    // ---------------- AXI 主机任务 ----------------
    reg [31:0] rdv;
    task axi_write; input [31:0] a, d;
        begin
            @(negedge clk); awvalid = 1; awaddr = a; wvalid = 1; wdata = d; wstrb = 4'hF;
            while (!(awready && wready)) @(posedge clk);
            @(negedge clk); awvalid = 0; wvalid = 0;
            while (!bvalid) @(posedge clk);
            chk(bresp === 2'b00, "AXI 写响应 OKAY");
            @(negedge clk); bready = 1; @(posedge clk); @(negedge clk); bready = 0;
        end
    endtask

    task axi_read; input [31:0] a;
        begin
            @(negedge clk); arvalid = 1; araddr = a;
            while (!arready) @(posedge clk);
            @(negedge clk); arvalid = 0;
            while (!rvalid) @(posedge clk);
            rdv = rdata;
            @(negedge clk); rready = 1; @(posedge clk); @(negedge clk); rready = 0;
        end
    endtask

    // 投递一条 ingress：{port, len, 4 个字（16 字节）}
    task host_send16; input [2:0] port; input [31:0] w0,w1,w2,w3;
        begin
            axi_write(32'h0C, {13'h0, port, 16'd16});
            axi_write(32'h10, w0); axi_write(32'h10, w1);
            axi_write(32'h10, w2); axi_write(32'h10, w3);
            axi_write(32'h14, 32'h1);
        end
    endtask

    // 等 egress（OUT_VALID），读 INFO + 4 个字，ACK
    task host_recv16; output [2:0] port; output [31:0] w0,w1,w2,w3;
        integer w;
        begin
            w = 0;
            axi_read(32'h04);
            while ((rdv[1] !== 1'b1) && w < 1000000) begin axi_read(32'h04); w = w + 1; end
            chk(rdv[1] === 1'b1, "主机看到 OUT_VALID = 1");
            axi_read(32'h18); port = rdv[18:16];
            chk((rdv[15:0] === 16'd16), "OUT_INFO 长度 = 16");
            axi_read(32'h1C); w0 = rdv;
            axi_read(32'h1C); w1 = rdv;
            axi_read(32'h1C); w2 = rdv;
            axi_read(32'h1C); w3 = rdv;
            axi_write(32'h20, 32'h1);
        end
    endtask

    integer k, w, base;
    reg [2:0] rport;
    reg [31:0] r0,r1,r2,r3;

    initial begin
        #200 reset = 0;
        $display("");
        $display("======== AXI 主机数据面端到端（SoC + 固件）========");

        // T1 等固件自检完成
        w = 0;
        while (dres(12'h3F0) !== 32'hA5A5C0DE && w < 2500000) begin @(posedge clk); w = w + 1; end
        chk(dres(12'h3F0) === 32'hA5A5C0DE, "T1 固件自检完成");

        // T2 主机使能数据面 + IRQ
        axi_write(32'h08, 32'h5);                      // bit0 IRQ, bit2 host_en
        axi_read(32'h04); chk(rdv[0] === 1'b0, "T2a 起手 IN_VALID = 0");

        // T3 投递到端口 1（16 字节）
        base = nfr;
        host_send16(3'd1, 32'h484F5354, 32'h11111111, 32'hDEADBEEF, 32'hCAFEBABE);

        axi_read(32'h04);
        // 固件取走后 IN_VALID 会清；这里等它清（说明 host_poll 处理了）
        w = 0;
        while ((rdv[0] !== 1'b0) && w < 500000) begin axi_read(32'h04); w = w + 1; end
        chk(rdv[0] === 1'b0, "T3b 固件取走 ingress（IN_VALID 清）");


        w = 0;
        while (nfr < base + 64 && w < 5000000) begin @(posedge clk); w = w + 1; end
        chk(nfr >= base + 64, "T4a 线侧出现了帧");
        chk(fr[base+0]===8'h03 && fr[base+1]===8'h00 && fr[base+2]===8'h00 && fr[base+3]===8'h00 &&
            fr[base+4]===8'h04 && fr[base+5]===8'h56, "T4b DA/VLID = 03:00:00:00:04:56（端口 1 的 VL）");
        chk(fr[base+42]===8'h54 && fr[base+43]===8'h53 && fr[base+44]===8'h4F && fr[base+45]===8'h48,
            "T4c 载荷前 4 字节 = 'HOST'（小端：54 53 4F 48）");

        // T5 主机收 egress（回环 → 端口 1 → 固件 → 邮箱）
        host_recv16(rport, r0, r1, r2, r3);
        chk(rport === 3'd1, "T5a egress 的端口号 = 1");
        chk(r0 === 32'h484F5354 && r1 === 32'h11111111 &&
            r2 === 32'hDEADBEEF && r3 === 32'hCAFEBABE, "T5b egress 载荷逐字节一致");

        // T6 host_en=0 时不推 egress
        axi_write(32'h08, 32'h1);                      // 关 host_en
        host_send16(3'd1, 32'h00000002, 32'h33333333, 32'h44444444, 32'h55555555);
        repeat (375000) @(posedge clk);
        axi_read(32'h04); chk(rdv[1] === 1'b0, "T6a host_en=0 时没有推 egress");
        axi_write(32'h08, 32'h5);                      // 重新使能
        repeat (250000) @(posedge clk);
        host_recv16(rport, r0, r1, r2, r3);
        chk(rport === 3'd1 && r0 === 32'h00000002 && r1 === 32'h33333333,
            "T6b 重新使能后 egress 推出（端口 1 / 0x00000002）");

        $display("=============================================");
        if (errs == 0) $display(">> PASS: AXI 主机数据面端到端，%0d 项", nchk);
        else           $display(">> FAIL: %0d 项不符（共 %0d 项）", errs, nchk);
        $finish;
    end

    initial begin #250000000; $display(">> TIMEOUT nfr=%0d", nfr); $finish; end
endmodule
