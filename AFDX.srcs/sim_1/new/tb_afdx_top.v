// tb_afdx_top.v —— 赛题顶层模板 AFDX_top 的测试台（走字节流口，BYPASS=1）
`timescale 1ns/1ps

module tb_afdx_top #(
    parameter integer SOC_DLY = 400000,    // 3.2 ms @125MHz（等 MAC 配置完成）
    parameter integer OBS     = 1000000    // 放行后观察 1 ms
) ();

    reg clk   = 1'b0;
    reg reset = 1'b1;
    always #4  clk = ~clk;                 // 125 MHz

    // ---- 模板那 8 个信号 ----
    reg  [7:0] rx_data  = 8'h00;
    reg        rx_valid = 1'b0;
    reg        rx_ready = 1'b1;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;

    // ---- 额外接口：全部接确定值 ----
    wire [7:0] gmii_a_txd, gmii_b_txd;
    wire       gmii_a_txen, gmii_a_txer, gmii_a_gtxc;
    wire       gmii_b_txen, gmii_b_txer, gmii_b_gtxc;
    wire       mdc_a, mdc_b, phy_rstb_a, phy_rstb_b;
    wire       mdio_a, mdio_b;
    wire [3:0] debug;
    wire       mac_cfg_done, soc_boot_done, fl_miso, fl_sclk, fl_mosi, fl_cs_n;
    pullup (fl_miso);                      // 不装 flash 模型 → 读回全 0xFF = 无魔数 → 保留预设初值
    pullup (mdio_a);  pullup (mdio_b);     // MDIO 空闲高（BYPASS 模式下 MAC 不参与数据）


    AFDX_top #(.PORTS(2), .SOC_BOOT_DLY(SOC_DLY), .BYPASS(1), .APP_SIDE(0), .VLID_MODE(0)) u_dut (
        .tx_port(), .err(),
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
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done)
    );


    always @(posedge clk) begin
        rx_data  <= tx_data;
        rx_valid <= tx_valid;
    end

    // =====================================================================
    // 帧捕获（按 valid 包络）+ FCS 残差
    // =====================================================================
    reg [7:0] fr [0:8191];
    integer   nfr = 0;
    always @(posedge clk) if (tx_valid && nfr < 8191) begin
        fr[nfr] <= tx_data; nfr <= nfr + 1;
    end

    integer n_desc = 0;                      // 诊断：解析器报了几次帧末
    wire    rxp_desc_w = u_dut.u_es.u_soc.rxp_desc;
    always @(posedge clk) if (rxp_desc_w) n_desc = n_desc + 1;

    reg     saw_b = 1'b0;
    wire    rxp_net_w = u_dut.u_es.u_soc.rxp_net;
    always @(posedge clk) if (rxp_net_w) saw_b <= 1'b1;

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask

    // FCS 残差（与 tb_afdx_es / tb_afdx_core 同一约定：不做最终异或）
    function [31:0] rcrc32b; input [31:0] c; input [7:0] d;
        integer k; reg [31:0] t;
        begin t = c ^ {24'h0, d}; for (k = 0; k < 8; k = k + 1) t = (t[0]) ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
              rcrc32b = t; end
    endfunction
    function [31:0] resid; input integer base;   // 从 base 起算到一帧结尾（含 FCS）
        integer b; reg [31:0] c;
        begin c = 32'hFFFFFFFF;
              for (b = 0; b < flen(base); b = b + 1) c = rcrc32b(c, fr[base+b]);
              resid = c; end
    endfunction
    // 一帧的长度：从 base 起，找下一个"DA 常量域开头"或缓冲结尾（简化：按 EtherType+IP 长度算）
    function integer flen; input integer base;
        integer ipl;
        begin ipl = {fr[base+16], fr[base+17]};        // IP Total Length
              flen = 14 + ipl + 4;                     // ETH + IP总长 + FCS
              if (flen < 64) flen = 64;                // 最小帧
        end
    endfunction

    // 固件结果区（dRAM 0x300 起；字索引 = 0x300>>2 = 192）
    function [31:0] dres; input integer wordidx;
        begin dres = {u_dut.u_es.u_soc.dram3[wordidx], u_dut.u_es.u_soc.dram2[wordidx],
                      u_dut.u_es.u_soc.dram1[wordidx], u_dut.u_es.u_soc.dram0[wordidx]}; end
    endfunction

    integer k, m, w;
    reg [7:0] v0;

    initial begin
        #200 reset = 1'b0;
        $display("");
        $display("========== 赛题顶层模板 AFDX_top（字节流口，BYPASS=1）==========");

        // ---------- T1/T2 上电 ----------
        w = 0;
        while (!mac_cfg_done && w < SOC_DLY*4) begin @(posedge clk); w = w + 1; end
        chk(mac_cfg_done === 1'b1, "T1 MAC 配置完成（mac_cfg_done）");
        w = 0;
        while (!soc_boot_done && w < SOC_DLY*4) begin @(posedge clk); w = w + 1; end
        chk(soc_boot_done === 1'b1, "T2 SoC 放行（soc_boot_done）");

        // ---------- 等固件跑完自检（轮询它的完成标志，不写死延时）----------
        w = 0;
        while (dres(192 + 60) !== 32'hA5A5C0DE && w < 600000) begin @(posedge clk); w = w + 1; end
        $display("  [信息] 自检标志=%08x 等=%0d 拍 | 模板口上收到的字节数=%0d（直接环回，无整形）",
                 dres(192+60), w, nfr);
        $display("  [信息] 解析器报的帧末次数=%0d", n_desc);
        chk(dres(192 + 60) === 32'hA5A5C0DE, "T3 固件自检完成标志置起（RES 0xF0）");

        // ---------- T4 模板口上真的出现了帧，且格式正确 ----------
        chk(nfr > 64, "T4a ★模板口 tx_valid/tx_data 上出现了帧（>64 字节）");
        chk(fr[0] === 8'h03 && fr[1] === 8'h00 && fr[2] === 8'h00 && fr[3] === 8'h00,
            "T4b ★DA 常量域 = 03:00:00:00（ARINC 664 的域常量）");
        chk(fr[6] === 8'h02 && fr[7] === 8'h00 && fr[8] === 8'h00,
            "T4c SA 常量域 = 02:00:00");
        chk(fr[11] === 8'h20, "T4d ★A 副本 SA 的 Interface_ID = 0x20");
        chk(fr[12] === 8'h08 && fr[13] === 8'h00, "T4e EtherType = 0x0800");
        chk(resid(0) === 32'hDEBB20E3, "T4f ★★FCS 残差 = DEBB20E3（帧在模板口上格式完整）");
        chk(saw_b === 1'b1,
            "T4g ★模板口从帧内 IFID 识别出了 B 网帧（net=1，不再写死 A）");

        // ---------- T5 固件自检结果：回环回来的帧被收下、固件读回 "AFDX" ----------
        //   RES(0x00) = 固件从采样口读回的第 1 个字，期望 'A','F','D','X' = 0x58444641
        chk(dres(192 + 0) === 32'h58444641,
            "T5a ★★固件读回 AFDX（回环→解析→IC/RM→端口层→固件 全程经过模板接口）");
        chk(dres(192 + 1) === 32'h00003231, "T5b ★固件读回第 2 个字 = \"12\"");
        chk(dres(192 + 2) === 32'h00000400, "T5c ★随附的 UDP 目的端口 = 1024（采样口）");
        //   固件那 6 条软工具链自检也要对（对数是软的：__udivsi3/__umodsi3/__mulsi3）
        chk(dres(192 + 25) === 32'd142, "T5d 软除法自检 1000/7 = 142");
        chk(dres(192 + 27) === 32'd143, "T5e 软乘法自检 13*11 = 143");

        $display("================================================================");
        if (errs == 0) $display(">> PASS: AFDX_top 模板接口（字节流）端到端可用");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #60000000; $display(">> TIMEOUT errs=%0d nfr=%0d", errs, nfr); $finish; end
endmodule
