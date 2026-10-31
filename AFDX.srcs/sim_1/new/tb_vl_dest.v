// tb_vl_dest.v —— 数据 VL 的接收归属
`timescale 1ns/1ps

module tb_vl_dest;
    reg clk = 1'b0, reset = 1'b1;
    always #4 clk = ~clk;                  // 125 MHz

    // ---- SoC 接收字节流 ----
    reg        rx_fv = 0, rx_fsop = 0, rx_feop = 0, rx_fnet = 0;
    reg  [7:0] rx_fd = 8'h00;

    wire       dp_fr_valid, dp_fr_sop, dp_fr_eop, dp_fr_net;
    wire [7:0] dp_fr_data;
    wire       cfg_hit; wire [127:0] cfg_word;
    wire       fl_sclk, fl_mosi, fl_cs_n;
    wire       h_irq;

    darkriscv_soc_top u_dut (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk(clk), .reset(reset), .debug(),
        .dp_rxin_valid(rx_fv), .dp_rxin_sop(rx_fsop), .dp_rxin_eop(rx_feop),
        .dp_rxin_data(rx_fd),  .dp_rxin_net(rx_fnet),
        .dp_fr_valid(dp_fr_valid), .dp_fr_ready(1'b1), .dp_fr_sop(dp_fr_sop),
        .dp_fr_eop(dp_fr_eop), .dp_fr_data(dp_fr_data), .dp_fr_net(dp_fr_net),
        .cfg_hit_o(cfg_hit), .cfg_cfg_o(cfg_word),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(1'b1), .flash_cs_n(fl_cs_n),
        .ts_rx_ts(64'h0), .ts_rx_seq(16'h0), .ts_tx_ts(64'h0), .ts_tx_seq(16'h0),
        .ts_fine_o(),
        .ts_tx_vl_ts(32'h0), .ts_tx_vl_sel(),
        .ts_rx_vl_ts(32'h0), .ts_rx_vl_sel(),
        .host_awaddr(32'h0), .host_awvalid(1'b0), .host_awready(),
        .host_wdata(32'h0), .host_wstrb(4'h0), .host_wvalid(1'b0), .host_wready(),
        .host_bresp(), .host_bvalid(), .host_bready(1'b0),
        .host_araddr(32'h0), .host_arvalid(1'b0), .host_arready(),
        .host_rdata(), .host_rresp(), .host_rvalid(), .host_rready(1'b0),
        .host_irq(h_irq)
    );

    // ---- 每端口交付计数（直接探端口层的 deliv_o）----
    integer dv0 = 0, dv1 = 0, dv2 = 0, dv3 = 0, dv4 = 0, dv5 = 0;
    always @(posedge clk) begin
        if (u_dut.rx_deliv_p_w[0]) dv0 = dv0 + 1;
        if (u_dut.rx_deliv_p_w[1]) dv1 = dv1 + 1;
        if (u_dut.rx_deliv_p_w[2]) dv2 = dv2 + 1;
        if (u_dut.rx_deliv_p_w[3]) dv3 = dv3 + 1;
        if (u_dut.rx_deliv_p_w[4]) dv4 = dv4 + 1;
        if (u_dut.rx_deliv_p_w[5]) dv5 = dv5 + 1;
    end

    // ---- CRC32 ----
    function [31:0] rcrc32b;
        input [31:0] c; input [7:0] d;
        integer k; reg [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            rcrc32b = t;
        end
    endfunction

    reg [7:0] gf [0:2047];
    integer gf_len, b, plen, iplen, snpos, total;
    reg [31:0] sum, cs, c;

    task build_frame;
        input [15:0] vlid; input [15:0] dport; input integer pl; input [7:0] sn;
        begin
            plen = pl;
            iplen = 20 + 8 + plen;
            snpos = 14 + iplen;
            if (snpos < 59) snpos = 59;
            total = snpos + 1;
            for (b = 0; b < 2048; b = b + 1) gf[b] = 8'h00;
            gf[0]=8'h03; gf[4]=vlid[15:8]; gf[5]=vlid[7:0];
            gf[6]=8'h02; gf[10]=8'h01; gf[11]=8'h20;      // A 网
            gf[12]=8'h08; gf[13]=8'h00;
            gf[14]=8'h45; gf[16]=iplen[15:8]; gf[17]=iplen[7:0];
            gf[20]=8'h40; gf[22]=8'h01; gf[23]=8'h11;
            gf[26]=8'h0A; gf[29]=8'h09;
            gf[30]=8'hE0; gf[31]=8'hE0; gf[32]=vlid[15:8]; gf[33]=vlid[7:0];
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {gf[b], gf[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            gf[24]=cs[15:8]; gf[25]=cs[7:0];
            gf[34]=vlid[15:8]; gf[35]=vlid[7:0];          // sport = vlid
            gf[36]=dport[15:8]; gf[37]=dport[7:0];
            gf[38]=(8+plen)>>8; gf[39]=(8+plen)&8'hFF;
            gf[40]=0; gf[41]=0;
            for (b = 0; b < plen; b = b + 1) gf[42+b] = (b*3 + sn) & 8'hFF;
            gf[snpos] = sn;
            c = 32'hFFFFFFFF;
            for (b = 0; b < total; b = b + 1) c = rcrc32b(c, gf[b]);
            c = ~c;
            gf[total+0]=c[7:0];  gf[total+1]=c[15:8];
            gf[total+2]=c[23:16]; gf[total+3]=c[31:24];
            gf_len = total + 4;
        end
    endtask

    task send_frame;
        begin
            for (b = 0; b < gf_len; b = b + 1) begin
                @(negedge clk);
                rx_fv = 1; rx_fd = gf[b]; rx_fsop = (b == 0); rx_feop = (b == gf_len-1); rx_fnet = 0;
                @(posedge clk);
            end
            @(negedge clk); rx_fv = 0; rx_fsop = 0; rx_feop = 0;
            repeat (20) @(posedge clk);
        end
    endtask

    integer i, errs = 0, w, base5, base0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    initial begin
        #200 reset = 0;
        #3000;                                   // 等 SoC 上电稳定
        $display("");
        $display("======== 数据 VL 的接收归属（0x0A01..0x0A04 → 端口 5）========");
        base5 = dv5; base0 = dv0;

        // ---- 1. 4 条数据 VL 各注入 1 帧 ----
        for (i = 0; i < 4; i = i + 1) begin
            build_frame(16'h0A01 + i, 16'd2001 + i, 32, 8'd1);
            send_frame;
        end
        // 等端口 5 交付（带超时）
        w = 0;
        while ((dv5 - base5) < 4 && w < 200000) begin @(posedge clk); w = w + 1; end
        $display("  [dbg] 数据 VL：dv5=%0d (base %0d)  dv0=%0d (base %0d)", dv5, base5, dv0, base0);
        chk((dv5 - base5) == 4, "T1 ★4 条数据 VL 的帧全部交付到端口 5");
        chk(dv0 == base0,       "T2 ★★端口 0（采样口）一次都没被数据 VL 污染");

        // ---- 2. 对照：采样 VL 0x0123 的帧必须进端口 0 ----
        base0 = dv0;
        build_frame(16'h0123, 16'd1024, 32, 8'd1);
        send_frame;
        build_frame(16'h0123, 16'd1024, 32, 8'd2);
        send_frame;
        w = 0;
        while ((dv0 - base0) < 2 && w < 200000) begin @(posedge clk); w = w + 1; end
        $display("  [dbg] 采样 VL：dv0=%0d (base %0d)", dv0, base0);
        chk((dv0 - base0) == 2, "T3 采样 VL 的 2 帧交付到端口 0（对照，证明监控有效）");

        // ---- 3. 端口 5 的交付不能算到别的端口上 ----
        chk(dv1 == 0 && dv2 == 0 && dv3 == 0 && dv4 == 0,
            "T4 ★其它端口（1..4）交付保持 0（数据 VL 没有串到服务口）");

        $display("==============================================================");
        if (errs == 0) $display(">> PASS: 数据 VL 只落在保留端口 5，不污染采样口");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #5000000; $display(">> TIMEOUT dv0=%0d dv5=%0d", dv0, dv5); $finish; end
endmodule
