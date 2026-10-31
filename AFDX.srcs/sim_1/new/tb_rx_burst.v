// tb_rx_burst.v —— P7 §5.1 口径的接收吞吐/突发处理能力（RTL，无 MAC 网表）
`timescale 1ns/1ps

module tb_rx_burst #(
    parameter integer NFRAMES = 82,        // 82×1518 ≈ 124.5k 拍 ≈ 0.996 ms @125 MHz
    parameter integer PLEN    = 1471       // AFDX 最大载荷
) ();
    reg clk = 1'b0;
    reg reset = 1'b1;
    always #4 clk = ~clk;                  // 125 MHz

    // ---- 接收字节流（本台直接驱动 SoC 的 dp_rxin_*）----
    reg        rx_fv = 1'b0, rx_fsop = 1'b0, rx_feop = 1'b0, rx_fnet = 1'b0;
    reg  [7:0] rx_fd = 8'h00;

    // ---- SoC 的发送流（本台不关心，接 ready=1）----
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

    // ---- CRC32（与 tb_darkriscv_soc 同款）----
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
    integer gf_len;
    integer b, f, total, iplen, snpos;
    reg [31:0] sum, cs, c;

    task build_frame(input [7:0] sn);
        begin
            iplen  = 20 + 8 + PLEN;
            snpos  = 14 + iplen;                 // SN 紧贴 FCS
            total  = snpos + 1;                  // 帧体（DA..SN）
            for (b = 0; b < 1600; b = b + 1) gf[b] = 8'h00;
            gf[0]=8'h03; gf[4]=8'h01; gf[5]=8'h23;          // DA = 03:00:00:00:01:23
            gf[6]=8'h02; gf[10]=8'h01; gf[11]=8'h20;        // SA，IFID=1（A 网）
            gf[12]=8'h08; gf[13]=8'h00;
            gf[14]=8'h45; gf[16]=iplen[15:8]; gf[17]=iplen[7:0];
            gf[20]=8'h40; gf[22]=8'h01; gf[23]=8'h11;       // TTL=1, UDP
            gf[26]=8'h0A; gf[27]=8'h00; gf[28]=8'h00; gf[29]=8'h09;
            gf[30]=8'hE0; gf[31]=8'hE0; gf[32]=8'h01; gf[33]=8'h23;
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {gf[b], gf[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            gf[24]=cs[15:8]; gf[25]=cs[7:0];
            gf[34]=8'h01; gf[35]=8'h23;                      // sport 0x0123
            gf[36]=8'h04; gf[37]=8'h00;                      // dport 1024（采样口）
            gf[38]=(8+PLEN)>>8; gf[39]=(8+PLEN)&8'hFF;
            gf[40]=0; gf[41]=0;                              // UDP 校验和 = 0
            for (b = 0; b < PLEN; b = b + 1) gf[42+b] = (b*7 + sn) & 8'hFF;
            gf[snpos] = sn;                                  // SN
            c = 32'hFFFFFFFF;
            for (b = 0; b < total; b = b + 1) c = rcrc32b(c, gf[b]);
            c = ~c;
            gf[total+0]=c[7:0];  gf[total+1]=c[15:8];
            gf[total+2]=c[23:16]; gf[total+3]=c[31:24];
            gf_len = total + 4;                              // 1518
        end
    endtask

    task send_frame;
        begin
            for (b = 0; b < gf_len; b = b + 1) begin
                @(negedge clk);
                rx_fv = 1'b1; rx_fd = gf[b];
                rx_fsop = (b == 0); rx_feop = (b == gf_len-1);
                rx_fnet = 1'b0;                              // A 网
                @(posedge clk);
            end
            @(negedge clk); rx_fv = 1'b0; rx_fsop = 1'b0; rx_feop = 1'b0;
        end
    endtask

    // ---- 统计 ----
    integer cyc = 0, n_inj = 0, n_acc = 0;
    integer c_first = 0, c_last = 0;
    always @(posedge clk) begin
        cyc <= cyc + 1;
        if (u_dut.rm_commit) begin
            n_acc <= n_acc + 1;
            c_last <= cyc;
        end
    end

    integer t0, waitc;
    real mbps;
    initial begin
        #200 reset = 1'b0;
        #1000;                                // 等配置表/复位稳定
        t0 = cyc;
        for (f = 0; f < NFRAMES; f = f + 1) begin
            build_frame(f + 8'd1);            // SN = 1..NFRAMES
            send_frame;
            n_inj = n_inj + 1;
        end
        c_first = t0;
        // 等 IC/RM 把最后一帧提交（给 200k 拍余量）
        waitc = 0;
        while ((n_acc < n_inj) && (waitc < 200000)) begin @(posedge clk); waitc = waitc + 1; end
        $display("[burst] injected=%0d accepted=%0d  cycles=%0d (%.3f us)  payload=%0d B",
                 n_inj, n_acc, c_last - c_first, (c_last - c_first) * 8.0 / 1000.0,
                 n_acc * PLEN);
        mbps = (n_acc * PLEN * 8.0) / ((c_last - c_first) * 8.0 / 1000.0);   // bits / us = Mbps
        $display("[burst] payload throughput = %.1f Mbps  (125 MHz, 1 Byte/cycle injection)", mbps);
        if (n_acc == n_inj) $display("  [ok]   burst: %0d/%0d frames accepted by IC/RM", n_acc, n_inj);
        else                $display("  [FAIL] burst: only %0d/%0d frames accepted", n_acc, n_inj);
        if (mbps >= 100.0)  $display("  [ok]   payload throughput %.1f Mbps  (>= 100 Mbps)", mbps);
        else                $display("  [FAIL] payload throughput %.1f Mbps  (< 100 Mbps)", mbps);
        if ((n_acc == n_inj) && (mbps >= 100.0))
            $display(">> PASS: RX burst throughput %.1f Mbps  (>= 100 Mbps payload)", mbps);
        else
            $display(">> FAIL: RX burst throughput %.1f Mbps  (injected=%0d accepted=%0d)", mbps, n_inj, n_acc);
        $finish;
    end

    initial begin #10000000; $display(">> TIMEOUT"); $finish; end   // 10 ms 兜底
endmodule
