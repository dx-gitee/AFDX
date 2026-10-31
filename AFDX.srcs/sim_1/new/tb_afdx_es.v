// tb_afdx_es.v —— AFDX 端系统全链路测试（RISC-V 软核 + 协议层 + MAC IP + PHY 模型）
`timescale 1ns/1ps

module tb_afdx_es #(

    parameter integer SOC_DLY = 125000,   // 1 ms @125MHz（MAC 实际 ~300 us 就绪）
    parameter integer OBS     = 1000000,  // 放行后观察窗口 1 ms（# 是 ns，不是拍）
    // 冗余网口数：2 = A/B 双网（默认，标准形态）；1 = 只做 A 网（单 PHY 板子/先跑通）。
    // 命令行可 -gPORTS=1 覆盖；单网时 B 网相关检查自动跳过。
    parameter integer PORTS   = 2
) ();

    reg clk    = 1'b0;
    reg reset  = 1'b1;
    reg p0_rxc = 1'b0;
    always #4  clk    = ~clk;

    always #4  p0_rxc = ~p0_rxc;     // 125 MHz 线侧（GMII 千兆）

    wire [7:0] gmii_a_txd, gmii_b_txd;
    wire       gmii_a_txen, gmii_a_txer, gmii_a_gtxc;
    wire       gmii_b_txen, gmii_b_txer, gmii_b_gtxc;
    reg  [7:0] gmii_a_rxd, gmii_b_rxd;
    reg        gmii_a_rxdv, gmii_a_rxer, gmii_b_rxdv, gmii_b_rxer;
    wire       mdc_a, mdio_a, phy_rstb_a, mdc_b, mdio_b, phy_rstb_b;
    wire [3:0] debug;
    wire       mac_cfg_done, soc_boot_done;

    // ---- 两张网各自的 GMII 回环（p0_rxc 域打一拍）----
    reg [7:0] lb_a_d = 0, lb_b_d = 0;
    reg       lb_a_v = 0, lb_b_v = 0;
    always @(posedge p0_rxc) begin
        lb_a_d <= gmii_a_txd;  lb_a_v <= gmii_a_txen;
        lb_b_d <= gmii_b_txd;  lb_b_v <= gmii_b_txen;
    end
    initial begin
        gmii_a_rxd = 0; gmii_a_rxdv = 0; gmii_a_rxer = 0;
        gmii_b_rxd = 0; gmii_b_rxdv = 0; gmii_b_rxer = 0;
    end
    always @(*) begin
        gmii_a_rxd = lb_a_d; gmii_a_rxdv = lb_a_v; gmii_a_rxer = 1'b0;
        gmii_b_rxd = lb_b_d; gmii_b_rxdv = lb_b_v; gmii_b_rxer = 1'b0;
    end

    // ---- 两个 MDIO PHY 模型 ----
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a (.mdc(mdc_a), .mdio(mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b (.mdc(mdc_b), .mdio(mdio_b));
    pullup (mdio_a); pullup (mdio_b);


    wire fl_miso;
    pullup (fl_miso);


    integer mdc_a_edges = 0, mdc_b_edges = 0;
    integer mdio_a_lo = 0, mdio_a_hi = 0, mdio_a_z = 0, mdio_a_x = 0;
    reg [7:0] mdio_bits [0:255];
    integer   mdio_bit_n = 0;
    integer   kk;

    always @(posedge mdc_a) mdc_a_edges = mdc_a_edges + 1;
    always @(posedge mdc_b) mdc_b_edges = mdc_b_edges + 1;

    always @(posedge mdc_a) begin
        if      (mdio_a === 1'b0) mdio_a_lo = mdio_a_lo + 1;
        else if (mdio_a === 1'b1) mdio_a_hi = mdio_a_hi + 1;
        else if (mdio_a === 1'bz) mdio_a_z  = mdio_a_z  + 1;
        else                      mdio_a_x  = mdio_a_x  + 1;
        if (mdio_bit_n < 256) begin
            mdio_bits[mdio_bit_n] = (mdio_a === 1'b1) ? "1" :
                                    (mdio_a === 1'b0) ? "0" :
                                    (mdio_a === 1'bz) ? "z" : "x";
            mdio_bit_n = mdio_bit_n + 1;
        end
    end

    // ---- 线侧抓帧（A/B 各一份）----
    reg [7:0] capA [0:2047]; integer nA = 0; reg doneA = 0, prevA = 0;
    reg [7:0] capB [0:2047]; integer nB = 0; reg doneB = 0, prevB = 0;
    always @(posedge p0_rxc) begin
        prevA <= gmii_a_txen;
        if (gmii_a_txen) begin if (!doneA) begin capA[nA] <= gmii_a_txd; nA <= nA + 1; end end
        else if (prevA) doneA <= 1'b1;
        prevB <= gmii_b_txen;
        if (gmii_b_txen) begin if (!doneB) begin capB[nB] <= gmii_b_txd; nB <= nB + 1; end end
        else if (prevB) doneB <= 1'b1;
    end

    // ---- DUT ----
    afdx_es_wrap #(.SOC_BOOT_DLY(SOC_DLY), .PORTS(PORTS)) u_dut (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),
        .clk(clk), .reset(reset),
        .gmii_a_txd(gmii_a_txd), .gmii_a_txen(gmii_a_txen), .gmii_a_txer(gmii_a_txer),
        .gmii_a_gtxc(gmii_a_gtxc),
        .gmii_a_rxd(gmii_a_rxd), .gmii_a_rxdv(gmii_a_rxdv), .gmii_a_rxer(gmii_a_rxer),
        .gmii_a_rxc(p0_rxc),
        .gmii_b_txd(gmii_b_txd), .gmii_b_txen(gmii_b_txen), .gmii_b_txer(gmii_b_txer),
        .gmii_b_gtxc(gmii_b_gtxc),
        .gmii_b_rxd(gmii_b_rxd), .gmii_b_rxdv(gmii_b_rxdv), .gmii_b_rxer(gmii_b_rxer),
        .gmii_b_rxc(p0_rxc),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done),
        .flash_sclk(), .flash_mosi(), .flash_miso(fl_miso), .flash_cs_n(),

        .bs_bypass(1'b0), .bs_rx_data(8'h00), .bs_rx_valid(1'b0),
        .bs_tx_ready(), .bs_tx_data(), .bs_tx_valid(), .bs_rx_ready(1'b1),

        .host_awaddr(32'h0), .host_awvalid(1'b0), .host_awready(),
        .host_wdata(32'h0), .host_wstrb(4'h0), .host_wvalid(1'b0), .host_wready(),
        .host_bresp(), .host_bvalid(), .host_bready(1'b0),
        .host_araddr(32'h0), .host_arvalid(1'b0), .host_arready(),
        .host_rdata(), .host_rresp(), .host_rvalid(), .host_rready(1'b0),
        .host_irq()
    );

    integer errs = 0;
    wire    dual = (PORTS >= 2);       // 单网时 B 网相关检查自动跳过
    integer n_acc = 0;

    time    t_maccfg = 0, t_release = 0;
    integer n_erl = 0;                 // 提前查表（路由查找）触发过多少拍
    always @(posedge clk) if (u_dut.u_core.u_soc.erl_pend) n_erl <= n_erl + 1;
    always @(posedge clk) if (u_dut.u_core.u_soc.ir_accept) n_acc <= n_acc + 1;
    // 引擎侧按网络分别数收到的帧（在 SoC 输入端，即仲裁器输出）
    integer frx_a = 0, frx_b = 0, byt_a = 0, byt_b = 0;
    integer macv_a = 0, macv_b = 0, rdya = 0, rdyb = 0;
    always @(posedge clk) begin
        if (u_dut.u_core.u_mac_a.ff_rx_data_vld) macv_a <= macv_a + 1;
        if (u_dut.u_core.ra_ready) rdya <= rdya + 1;
        if (u_dut.u_core.rb_ready) rdyb <= rdyb + 1;
    end

    reg arb_busy = 0, arb_cur = 0;
    generate
    if (PORTS >= 2) begin : g_probe_b
        always @(posedge clk) begin
            if (u_dut.u_core.g_dual.u_mac_b.ff_rx_data_vld) macv_b <= macv_b + 1;
            arb_busy <= u_dut.u_core.g_dual.u_arb.busy;
            arb_cur  <= u_dut.u_core.g_dual.u_arb.cur;
        end
    end
    endgenerate
    always @(posedge clk) if (u_dut.u_core.rx_valid) begin
        if (u_dut.u_core.rx_net) byt_b <= byt_b + 1; else byt_a <= byt_a + 1;
    end
    always @(posedge clk) if (u_dut.u_core.rx_valid && u_dut.u_core.rx_eop) begin
        if (u_dut.u_core.rx_net) frx_b <= frx_b + 1; else frx_a <= frx_a + 1;
    end
    integer n_ic = 0, n_rm = 0;
    always @(posedge clk) begin
        if (u_dut.u_core.u_soc.ir_ic) n_ic <= n_ic + 1;
        if (u_dut.u_core.u_soc.ir_rm) n_rm <= n_rm + 1;
    end
    // 诊断用：帧收齐次数 / 收齐且判决同拍的次数
    integer n_full = 0, n_fullcmt = 0, n_sop = 0, n_eop = 0, n_rel = 0;
    always @(posedge clk) begin
        if (u_dut.u_core.u_soc.dp_rxin_sop)   n_sop  <= n_sop + 1;
        if (u_dut.u_core.u_soc.dp_rxin_eop)   n_eop  <= n_eop + 1;
        if (u_dut.u_core.u_soc.u_rxports.cur_full) n_full <= n_full + 1;
        if (u_dut.u_core.u_soc.u_rxports.cur_full && u_dut.u_core.u_soc.rx_commit)
            n_fullcmt <= n_fullcmt + 1;
        if (u_dut.u_core.u_soc.u_rxports.release_w) n_rel <= n_rel + 1;
    end
    task chk;
        input cond; input [255:0] name;
        begin
            if (cond) $display("  [ok]   %0s", name);
            else begin errs = errs + 1; $display("  [FAIL] %0s", name); end
        end
    endtask

    // CRC-32 单字节（独立实现，用来验证线侧帧的 FCS 残差）
    function [31:0] crc32b;
        input [31:0] c; input [7:0] d;
        integer k; reg [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            crc32b = t;
        end
    endfunction

    // 线侧帧（cap[0..7] = 前导 55×7 + SFD D5）的 FCS 残差
    function [31:0] A_resid;
        input integer dummy;
        integer k; reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = 8; k < nA; k = k + 1) c = crc32b(c, capA[k]);
            A_resid = c;
        end
    endfunction
    function [31:0] B_resid;
        input integer dummy;
        integer k; reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = 8; k < nB; k = k + 1) c = crc32b(c, capB[k]);
            B_resid = c;
        end
    endfunction

    integer i;
    initial begin
        $display("");
        $display("======== AFDX 端系统全链路（软核+协议层+MAC%s）========",
                 dual ? "×2 A/B 双网" : " 单网(A)");
        #200; reset = 1'b0;
        wait (mac_cfg_done);  t_maccfg  = $time;
        $display("  [info] t=%0t 两张网的 MAC 都配置完成", $time);
        wait (soc_boot_done); t_release = $time;
        $display("  [info] t=%0t 分区放行，CPU 开始跑（比配置完成晚 %0t）",
                 $time, t_release - t_maccfg);
        #OBS;                           // 放行后观察窗口（默认 3 ms，让固件发完并自环收完）
        $display("  [info] t=%0t 观察结束；线侧抓到 A=%0d B=%0d 字节",
                 $time, nA, nB);
        $display("  [info] 引擎侧: A网帧=%0d(%0d字节) B网帧=%0d(%0d字节)  IC/RM接受=%0d IC丢弃=%0d RM丢弃=%0d 目的端口=%0d",
                 frx_a, byt_a, frx_b, byt_b, n_acc, n_ic, n_rm, u_dut.u_core.u_soc.port_lat);
        $display("  [info] 仲裁器: busy=%b cur=%b（单网时恒 0）", arb_busy, arb_cur);
        $display("  [info] MAC 输出 vld: A=%0d B=%0d ; 仲裁器 ready 有效拍: A=%0d B=%0d",
                 macv_a, macv_b, rdya, rdyb);


        chk(mac_cfg_done === 1'b1 && t_maccfg > 0 && t_maccfg < 5000000,
            "T1 MAC 在 5 ms 内配置完成（超时或慢一个数量级都会红）");
        chk(soc_boot_done === 1'b1 && (t_release - t_maccfg) >= SOC_DLY*8,
            "T2 放行确实等足了 SOC_BOOT_DLY（少等/不等会红）");

        // ---------- 线侧：A 网帧 ----------
        chk(nA === 72 && (!dual || nB === 72), "T3 线侧帧长精确 = 72 字节（7 前导 + SFD + 64 帧）");
        if (nA >= 22) begin
            $display("  [info] A 线: %02x %02x %02x %02x %02x %02x %02x %02x | DA=%02x:%02x:%02x:%02x:%02x:%02x SA=%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x",
                     capA[0],capA[1],capA[2],capA[3],capA[4],capA[5],capA[6],capA[7],
                     capA[8],capA[9],capA[10],capA[11],capA[12],capA[13],
                     capA[14],capA[15],capA[16],capA[17],capA[18],capA[19],
                     capA[20],capA[21]);
        end
        if (nB >= 22) begin
            $display("  [info] B 线: %02x %02x %02x %02x %02x %02x %02x %02x | DA=%02x:%02x:%02x:%02x:%02x:%02x SA=%02x:%02x:%02x:%02x:%02x:%02x type=%02x%02x",
                     capB[0],capB[1],capB[2],capB[3],capB[4],capB[5],capB[6],capB[7],
                     capB[8],capB[9],capB[10],capB[11],capB[12],capB[13],
                     capB[14],capB[15],capB[16],capB[17],capB[18],capB[19],
                     capB[20],capB[21]);
        end
        chk(capA[0]==8'h55 && capA[6]==8'h55 && capA[7]==8'hD5 &&
            (!dual || (capB[0]==8'h55 && capB[6]==8'h55 && capB[7]==8'hD5)),
            "T4 线侧有前导 55×7 + SFD D5（双网时两张网都要有）");
        chk(capA[8]==8'h03 && capA[12]==8'h01 && capA[13]==8'h23,
            "T5a A 网 DA = 03:00:00:00:01:23（组播常量域 + VLID 0x0123）");
        if (dual)
            chk(capB[8]==8'h03 && capB[12]==8'h01 && capB[13]==8'h23,
                "T5b B 网 DA 与 A 相同（同一 VL 的冗余副本）");
        chk(capA[19]==8'h20, "T6 A 网 SA 的 Interface_ID = 1（0x20）");
        if (dual) chk(capB[19]==8'h40, "T7 B 网 SA 的 Interface_ID = 2（0x40）");
        chk(A_resid(0) === 32'hDEBB20E3, "T8a A 网帧 FCS 残差 = DEBB20E3");
        if (dual) chk(B_resid(0) === 32'hDEBB20E3, "T8b B 网帧 FCS 残差 = DEBB20E3");


        chk(frx_a == 2 && (!dual || frx_b == 2), "T9 两条消息的帧都进了引擎（双网时每张网各 2 帧）");
        chk(n_acc == 2, "T10 IC/RM 接受 2 条（两条消息各有一条先到）");
        if (dual) chk(n_rm == 2, "T11 两个 B 网同 SN 副本被 RM 判重复丢弃（First Valid Wins）");

        chk(n_erl >= 1 && u_dut.u_core.u_soc.erl_pend === 1'b0,
            "T12 收帧触发过提前查表且已完成（路由查找没挂起）");

        chk(u_dut.u_core.u_soc.port_lat === 2'd1,
            "T12b ★最后一条消息路由到端口 1（VL 0x0456 的 buf_idx=1，非 0 路由生效）");

        // ---------- CPU 读回 ----------
        chk(u_dut.u_core.u_soc.dram0[8'hC0] === 8'h41 && u_dut.u_core.u_soc.dram1[8'hC0] === 8'h46 &&
            u_dut.u_core.u_soc.dram2[8'hC0] === 8'h44 && u_dut.u_core.u_soc.dram3[8'hC0] === 8'h58,
            "T13 CPU 读回载荷 = AFDX");
        chk({u_dut.u_core.u_soc.dram3[8'hC1],u_dut.u_core.u_soc.dram2[8'hC1],
             u_dut.u_core.u_soc.dram1[8'hC1],u_dut.u_core.u_soc.dram0[8'hC1]} === 32'h00003231,
            "T14 CPU 读回载荷第 2 字 = 00003231");
        chk({u_dut.u_core.u_soc.dram3[8'hC2],u_dut.u_core.u_soc.dram2[8'hC2],
             u_dut.u_core.u_soc.dram1[8'hC2],u_dut.u_core.u_soc.dram0[8'hC2]} === 32'h00000400,
            "T15 CPU 读回消息的 UDP 目的端口 = 1024（VL 0x0123 预设 dport）");

        // ---- MDIO 诊断报告 ----
        $display("[MDIO] MDC 边沿: A=%0d B=%0d | MDIO-A 电平分布: 0=%0d 1=%0d z=%0d x=%0d",
                 mdc_a_edges, mdc_b_edges, mdio_a_lo, mdio_a_hi, mdio_a_z, mdio_a_x);
        if (mdio_bit_n == 0)
            $display("[MDIO] A 网 MDC 一次都没跳 —— MAC 根本没启动 MDIO（或无 PHY 应答）");
        else begin
            $write("[MDIO] A 网前 %0d 个 MDC 上升沿采到的位流: ", mdio_bit_n);
            for (kk = 0; kk < mdio_bit_n && kk < 256; kk = kk + 1) begin
                $write("%c", mdio_bits[kk]);
                if ((kk % 64) == 63) $write("\n      ");
            end
            $write("\n");
        end

        $display("======================================================");
        if (errs == 0) $display(">> PASS: A/B 双网冗余端到端打通（线侧↔MAC×2↔仲裁器↔协议层↔分区）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #40000000;                       // 40 ms 兜底
        $display(">> TIMEOUT: mac_cfg_done=%b soc_boot=%b nA=%0d nB=%0d",
                 mac_cfg_done, soc_boot_done, nA, nB);
        $finish;
    end

endmodule

