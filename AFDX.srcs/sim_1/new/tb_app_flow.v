// tb_app_flow.v —— 应用数据口（VLID 注入/交付）的端到端测量台
`timescale 1ns/1ps

module tb_app_flow;
    reg clk = 0, reset = 1;
    always #4 clk = ~clk;                     // 125 MHz

    // ---- 应用侧（VLID 注入/交付）----
    reg        a_v = 0; reg [7:0] a_d = 0; reg [15:0] a_vl = 0; reg [3:0] a_p = 0;
    wire       t_v; wire [7:0] t_d; wire [3:0] t_p; wire [15:0] t_vl; wire err;

    // ---- 内部回环：SoC 发帧流 → 延迟 1 拍 → SoC 收帧流 ----
    wire       fr_v, fr_sop, fr_eop, fr_net;
    wire [7:0] fr_d;
    reg        fr_ready = 1;
    reg        rx_v = 0, rx_sop = 0, rx_eop = 0, rx_net = 0; reg [7:0] rx_d = 0;
    always @(posedge clk) begin
        rx_v <= fr_v; rx_sop <= fr_sop; rx_eop <= fr_eop; rx_d <= fr_d; rx_net <= fr_net;
    end

    darkriscv_soc_top #(.VLID_MODE(1)) u_dut (
        .clk(clk), .reset(reset), .debug(),
        .dp_rxin_valid(rx_v), .dp_rxin_sop(rx_sop), .dp_rxin_eop(rx_eop),
        .dp_rxin_data(rx_d),  .dp_rxin_net(rx_net),
        .dp_fr_valid(fr_v), .dp_fr_ready(fr_ready), .dp_fr_sop(fr_sop),
        .dp_fr_eop(fr_eop), .dp_fr_data(fr_d), .dp_fr_net(fr_net),
        .app_rx_valid(a_v), .app_rx_data(a_d), .app_port(a_p), .app_vlid(a_vl),
        .app_tx_valid(t_v), .app_tx_data(t_d), .app_tx_port(t_p), .app_tx_vlid(t_vl),
        .app_err(err),
        .cfg_hit_o(), .cfg_cfg_o(),
        .flash_sclk(), .flash_mosi(), .flash_miso(1'b1), .flash_cs_n(),
        .ts_rx_ts(64'h0), .ts_rx_seq(16'h0), .ts_tx_ts(64'h0), .ts_tx_seq(16'h0),
        .ts_fine_o(),
        .ts_tx_vl_ts(32'h0), .ts_tx_vl_sel(),
        .ts_rx_vl_ts(32'h0), .ts_rx_vl_sel(),
        .host_awaddr(32'h0), .host_awvalid(1'b0), .host_awready(),
        .host_wdata(32'h0), .host_wstrb(4'h0), .host_wvalid(1'b0), .host_wready(),
        .host_bresp(), .host_bvalid(), .host_bready(1'b0),
        .host_araddr(32'h0), .host_arvalid(1'b0), .host_arready(),
        .host_rdata(), .host_rresp(), .host_rvalid(), .host_rready(1'b0),
        .host_irq()
    );


    reg  [7:0]  rbuf [0:2047];
    integer     rcnt = 0, rmsgs = 0, dcount = 0;
    reg  [15:0] g_rvl, exp_vl = 16'h0;
    time        t_rx0;
    reg         rseg = 0;
    always @(posedge clk) begin
        if (t_v) begin
            if (!rseg) begin
                rseg <= 1'b1;
                if (t_vl === exp_vl) begin
                    rcnt = 0; rmsgs = rmsgs + 1; dcount = dcount + 1;
                    g_rvl = t_vl; t_rx0 = $time;
                end
            end
            if (t_vl === exp_vl) begin rbuf[rcnt] = t_d; rcnt = rcnt + 1; end
        end else rseg <= 1'b0;
    end

    integer intm = 0;
    always @(posedge clk) if (u_dut.am_v && u_dut.am_ready && u_dut.am_sop) intm = intm + 1;
    integer dcount_all = 0;
    reg t_seg = 0;
    always @(posedge clk) begin
        t_seg <= t_v;
        if (t_v && !t_seg && t_vl >= 16'h0230 && t_vl <= 16'h0238) dcount_all = dcount_all + 1;
    end


    integer d_sus = 0; reg t_sus_seg = 0;
    integer b_sus = 0;
    integer wmpeak = 0;
    always @(posedge clk) begin
        t_sus_seg <= t_v;
        if (t_v && !t_sus_seg && t_vl >= 16'h0250 && t_vl <= 16'h025F) d_sus = d_sus + 1;
        if (t_v && t_vl >= 16'h0250 && t_vl <= 16'h025F) b_sus = b_sus + 1;
        if (u_dut.u_appbr.wcnt > wmpeak) wmpeak = u_dut.u_appbr.wcnt;
    end

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask

    time t_tx0;
    task send_msg; input integer n; input [7:0] base; input [15:0] vl;
        integer k;
        begin
            a_vl = vl; exp_vl = vl;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); a_v = 1; a_d = base + k[7:0];
                if (k == 0) t_tx0 = $time;
                @(posedge clk);
            end
            @(negedge clk); a_v = 0;
        end
    endtask


    task send_msg_p; input integer n; input [7:0] base; input [15:0] vl; input integer pace;
        integer k;
        begin
            a_vl = vl; exp_vl = vl;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); a_v = 1; a_d = base + k[7:0];
                if (k == 0) t_tx0 = $time;
                @(posedge clk);
                if (pace > 1) repeat (pace - 1) @(posedge clk);
            end
            @(negedge clk); a_v = 0;
        end
    endtask

    task wait_msg; input integer n; input integer tmax; output ok;
        integer w;
        begin
            w = 0;
            while ((rcnt < n) && (w < tmax)) begin @(posedge clk); w = w + 1; end
            ok = (rcnt >= n);
        end
    endtask

    integer k, vi, ok, lat, lat_max, lat_min, nbytes, w;
    time t_win0, t_win1;

    initial begin
        #200 reset = 0;
        repeat (4000) @(posedge clk);           // 等固件把端口/配置准备好
        $display("");
        $display("======== App data port (VLID inject -> loopback -> deliver) ========");

        // ---------- T1 按 VLID 注入 ----------
        rmsgs = 0; rcnt = 0;
        send_msg(5, 8'h48, 16'h0456);           // 0x48,49,4A,4B,4C
        wait_msg(5, 500000, ok);
        $display("  [dbg] T1 rmsgs=%0d rcnt=%0d vlid=%04x b=%02x %02x", rmsgs, rcnt, g_rvl, rbuf[0], rbuf[4]);
        chk(ok && g_rvl === 16'h0456, "T1 delivered with VLID 0x0456");
        chk(rbuf[0] === 8'h48 && rbuf[4] === 8'h4C, "T2 payload correct");

        // ---------- T3 变长帧（1 / 64 / 512 / 1471，各自一条 VL）----------
        begin : t3
            integer L[0:3];
            L[0] = 1; L[1] = 64; L[2] = 512; L[3] = 1471;
            for (vi = 0; vi < 4; vi = vi + 1) begin
                rcnt = 0;
                send_msg(L[vi], 8'h10, 16'h0200 + vi[15:0]);
                wait_msg(L[vi], 600000, ok);
                $display("  [dbg] T3 len=%0d got=%0d vlid=%04x", L[vi], rcnt, g_rvl);
                chk(ok && rcnt == L[vi], "T3 variable length");
                repeat (3000) @(posedge clk);
            end
        end

        // ---------- T4 时延（首个 tx 字节 → 首个 rx 字节）----------
        rcnt = 0;
        send_msg(64, 8'h20, 16'h0210);
        wait_msg(64, 600000, ok);
        lat = (t_rx0 - t_tx0) / 8;
        $display("  [dbg] T4 latency=%0d cycles = %0d us", lat, (lat*8)/1000);
        chk(ok && (t_rx0 > t_tx0) && ((t_rx0 - t_tx0) < 2000000),
            "T4 latency < 2ms (app in -> app out)");

        // ---------- T5 抖动（同 VL 同帧长 ×3）----------
        lat_max = 0; lat_min = 1000000000;
        for (k = 0; k < 3; k = k + 1) begin
            rcnt = 0;
            send_msg(128, 8'h30, 16'h0220);
            wait_msg(128, 600000, ok);
            lat = (t_rx0 - t_tx0) / 8;
            $display("  [dbg] T5 iter%0d lat=%0d rcnt=%0d intm=%0d wcnt=%0d sst=%0d mrdy=%b muxcur=%0d txp_pend=%b err=%b",
                     k, lat, rcnt, intm, u_dut.u_appbr.wcnt, u_dut.u_appbr.sst,
                     u_dut.am_ready, u_dut.u_msgmux.cur, u_dut.txp_pend_w, err);
            $display("  [dbg] T5 iter%0d pre_cnt=%0d first=%b skip_txp=%b skip_app=%b txp_v=%b mrdy_real=%b",
                     k, u_dut.u_msgmux.pre_cnt, u_dut.u_msgmux.first_taken,
                     u_dut.u_msgmux.skip_txp, u_dut.u_msgmux.skip_app,
                     u_dut.txv, u_dut.u_msgmux.msg_ready);
            $display("  [dbg]   tx: vli_q=%04x port_lat=%04x cfg_cur=%b ts=%04x nxt=%04x app_v=%b txp_v=%b",
                     u_dut.u_tx.vli_q, u_dut.u_tx.port_lat, u_dut.u_tx.cfg_is_cur,
                     u_dut.ts_now, u_dut.u_tx.nxt_q, u_dut.am_v, u_dut.txv);
            if (lat > lat_max) lat_max = lat;
            if (lat < lat_min) lat_min = lat;
            repeat (130000) @(posedge clk);      // 1.04 ms
        end
        $display("  [dbg] T5 jitter max=%0d min=%0d diff=%0d cycles = %0d us",
                 lat_max, lat_min, lat_max-lat_min, ((lat_max-lat_min)*8)/1000);
        chk((lat_max - lat_min) < 5000, "T5 jitter < 40us");

        // ---------- T6 多 VL 吞吐：**背靠背注入 9 条 VL，看多久交付完** ----------
        //   评测口径是"单位时间注入的有效载荷达到 100 Mbps 且不丢/不阻塞"⇒
        //   这里一次性灌 9 条（各 1471 B，不同 VL，首次发送不受 BAG 限制），
        //   量"首字节注入 → 第 9 条交付完"的时间窗。
        begin : t6
            integer n_ok;
            nbytes = 0;
            dcount = 0; dcount_all = 0;
            exp_vl = 16'h0230;                     // 先设成第一条，之后每条自己设

            t_win0 = $time;
            for (vi = 0; vi < 9; vi = vi + 1) begin
                send_msg(1471, 8'h40, 16'h0230 + vi[15:0]);
            end
            repeat (110000) @(posedge clk);        // ≈0.88 ms：一轮 9×1471B/1ms ≈ 106 Mbps
            // 等 9 条都交付（dcount 按 VLID 计数；不同 VL 各一条）
            w = 0; n_ok = 0;
            while ((n_ok < 9) && (w < 2000000)) begin
                @(posedge clk); w = w + 1;
                n_ok = dcount_all;
            end
            t_win1 = $time;
            nbytes = n_ok * 1471;
            $display("  [dbg] T6 delivered=%0d msgs (%0d B) over %0d ns => %0d kbps (err=%b)",
                     n_ok, nbytes, t_win1-t_win0, (nbytes*8*1000)/((t_win1-t_win0)/1000), err);
            chk(err === 1'b0, "T6a no drop/overflow (err=0)");
            chk(n_ok == 9, "T6b all 9 VLs delivered");
            chk(((nbytes*8*1000)/((t_win1-t_win0)/1000)) >= 100000, "T6c >= 100 Mbps payload");
        end


        begin : t7
            integer i7, n7; time u0, u1;
            d_sus = 0; b_sus = 0; wmpeak = 0; n7 = 160;

            for (i7 = 0; i7 < 8; i7 = i7 + 1)
                send_msg(1471, 8'h30 + i7[7:0], 16'h0250 + i7[15:0]);
            u0 = $time;

            for (i7 = 0; i7 < n7; i7 = i7 + 1) begin
                send_msg(1471, 8'h40 + i7[7:0], 16'h0250 + (i7 % 16));
                repeat (13242) @(posedge clk);      // 1471+13242 = 14713 拍/条 = 117.7 µs = 100 Mbps
            end
            u1 = $time;
            repeat (400000) @(posedge clk);          // 3.2 ms 收尾
            $display("  [dbg] T7 sent=%0d msgs, inject %0d us, delivered=%0d, err=%b",
                     n7, (u1-u0)/1000, d_sus, err);
            chk(err === 1'b0, "T7a sustained 16VL no drop");
            chk(d_sus >= n7, "T7b sustained 16VL all delivered");
            chk(b_sus == (n7 + 8) * 1471, "T7c delivered byte count exact");
            chk(wmpeak >= 2, "T7d inbound FIFO buffered (wcnt>=2)");
        end


        begin : t8
            integer i8, n8; time v0, v1;
            d_sus = 0; b_sus = 0; n8 = 120;
            v0 = $time;
            for (i8 = 0; i8 < n8; i8 = i8 + 1) begin
                send_msg(1471, 8'h50 + i8[7:0], 16'h0250 + (i8 % 9));
                repeat (13242) @(posedge clk);
            end
            v1 = $time;
            repeat (400000) @(posedge clk);
            $display("  [dbg] T8 sent=%0d msgs (9 VL), inject %0d us, delivered=%0d, err=%b",
                     n8, (v1-v0)/1000, d_sus, err);
            chk(d_sus >= n8, "T8 sustained 9VL all delivered");
            chk(b_sus == n8 * 1471, "T8b delivered byte count exact");
        end

        $display("==================================");
        if (errs == 0) $display(">> PASS: app data port (VLID / variable len / latency / jitter / multi-VL throughput)");
        else           $display(">> FAIL: %0d item(s) failed", errs);
        $finish;
    end

    initial begin #150000000; $display(">> TIMEOUT"); $finish; end
endmodule
