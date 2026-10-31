// tb_afdx_top_err.v —— AFDX_top 错误帧 + 正确帧混合测试（GMII 侧，文件激励）
`timescale 1ns/1ps

module tb_afdx_top_err #(
    parameter integer PORTS      = 1,
    parameter integer SOC_DLY    = 125000,   // 1 ms
    parameter integer BOOT_QUIET = 125000,   // 1 ms
    parameter integer ERR_QUIET  = 30000,    // 每个错误帧后等 0.24 ms
    parameter integer WAIT_GOOD  = 300000,   // 正确帧等 2.4 ms
    parameter integer N_ERR      = 9,
    parameter VEC_FILE = "AFDX_ES.srcs/sim_1/new/gmii_mixed_frames.hex"
) ();

    localparam integer MAXB = 32768;
    localparam integer MAXA = 8;

    reg clk = 1'b0, reset = 1'b1;
    always #4 clk = ~clk;

    reg gmii_a_rxc = 1'b0;
    reg gmii_b_rxc = 1'b0;
    always #4 gmii_a_rxc = ~gmii_a_rxc;
    always #4 gmii_b_rxc = ~gmii_b_rxc;

    reg  [7:0] gmii_a_rxd = 0, gmii_b_rxd = 0;
    reg        gmii_a_rxdv = 0, gmii_b_rxdv = 0;
    reg        gmii_a_rxer = 0, gmii_b_rxer = 0;

    wire [7:0] gmii_a_txd, gmii_b_txd;
    wire       gmii_a_txen, gmii_a_txer, gmii_a_gtxc;
    wire       gmii_b_txen, gmii_b_txer, gmii_b_gtxc;
    wire       mdc_a, mdio_a, phy_rstb_a, mdc_b, mdio_b, phy_rstb_b;
    wire       fl_sclk, fl_mosi, fl_cs_n;
    wire [3:0] debug;
    wire       mac_cfg_done, soc_boot_done;

    reg  [7:0] rx_data = 0;
    reg        rx_valid = 0;
    reg        rx_ready = 1;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;
    reg  [15:0] app_vlid = 16'h0789;
    wire [3:0] tx_port;
    wire       err;

    pullup (mdio_a);
    pullup (mdio_b);

    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a (.mdc(mdc_a), .mdio(mdio_a));
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b (.mdc(mdc_b), .mdio(mdio_b));

    AFDX_top #(
        .PORTS       (PORTS),
        .SOC_BOOT_DLY(SOC_DLY),
        .BYPASS      (0),
        .APP_SIDE    (1),
        .APP_PORT_DEF(0),
        .VLID_MODE   (1)
    ) u_dut (
        .clk(clk), .reset(reset),
        .tx_data(tx_data), .rx_data(rx_data),
        .tx_valid(tx_valid), .rx_valid(rx_valid),
        .tx_ready(tx_ready), .rx_ready(rx_ready),
        .app_vlid(app_vlid), .tx_port(tx_port), .err(err),

        .gmii_a_txd(gmii_a_txd), .gmii_a_txen(gmii_a_txen),
        .gmii_a_txer(gmii_a_txer), .gmii_a_gtxc(gmii_a_gtxc),
        .gmii_a_rxd(gmii_a_rxd), .gmii_a_rxdv(gmii_a_rxdv),
        .gmii_a_rxer(gmii_a_rxer), .gmii_a_rxc(gmii_a_rxc),

        .gmii_b_txd(gmii_b_txd), .gmii_b_txen(gmii_b_txen),
        .gmii_b_txer(gmii_b_txer), .gmii_b_gtxc(gmii_b_gtxc),
        .gmii_b_rxd(gmii_b_rxd), .gmii_b_rxdv(gmii_b_rxdv),
        .gmii_b_rxer(gmii_b_rxer), .gmii_b_rxc(gmii_b_rxc),

        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),

        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi),
        .flash_miso(1'b1), .flash_cs_n(fl_cs_n),

        .debug(debug), .mac_cfg_done(mac_cfg_done),
        .soc_boot_done(soc_boot_done)
    );

    // =================================================================
    // 文件向量
    // =================================================================
    reg [7:0] gmem [0:MAXB-1];
    integer gnfr = 0;
    integer g_wlen [0:15], g_port [0:15], g_elen [0:15];
    integer g_pstart [0:15], g_wstart [0:15];
    integer i, off;

    // =================================================================
    // 应用口抓消息
    // =================================================================
    reg [7:0] abuf [0:1023];
    integer astart [0:MAXA-1], alen [0:MAXA-1];
    reg [3:0] aport [0:MAXA-1];
    integer aidx = 0, awp = 0;
    reg txv_d = 0;

    always @(posedge clk) begin
        txv_d <= tx_valid;
        if (tx_valid) begin
            if (!txv_d && (aidx < MAXA)) begin
                astart[aidx] = awp;
                alen[aidx] = 0;
                aport[aidx] = tx_port;
            end
            if ((aidx < MAXA) && (awp < 1024)) begin
                abuf[awp] <= tx_data;
                awp <= awp + 1;
                alen[aidx] <= alen[aidx] + 1;
            end
        end else if (txv_d) begin
            if (aidx < MAXA) aidx <= aidx + 1;
        end
    end

    // 找应用口是否出现“第 fi 帧的 exp_port/exp_len/payload”
    function integer find_app_file;
        input integer fi;
        integer m, k;
        reg hit;
        begin
            find_app_file = -1;
            for (m = 0; m < aidx; m = m + 1) begin
                hit = (aport[m] === g_port[fi]) && (alen[m] === g_elen[fi]);
                if (hit) begin
                    for (k = 0; k < g_elen[fi]; k = k + 1)
                        if (abuf[astart[m]+k] !== gmem[g_pstart[fi]+k]) hit = 0;
                end
                if (hit && (find_app_file < 0)) find_app_file = m;
            end
        end
    endfunction

    // =================================================================
    // 检查 / 激励
    // =================================================================
    integer errs = 0;
    task chk;
        input c;
        input [255:0] name;
        begin
            if (c) $display("  [ok]   %0s", name);
            else begin errs = errs + 1; $display("  [FAIL] %0s", name); end
        end
    endtask

    task gmii_send_idx;
        input integer fi;
        integer k;
        begin
            for (k = 0; k < g_wlen[fi]; k = k + 1) begin
                @(posedge gmii_a_rxc);
                gmii_a_rxd  <= gmem[g_wstart[fi]+k];
                gmii_a_rxdv <= 1'b1;
                gmii_a_rxer <= 1'b0;
            end
            @(posedge gmii_a_rxc);
            gmii_a_rxdv <= 1'b0;
            gmii_a_rxd  <= 8'h00;
            repeat (1000) @(posedge gmii_a_rxc);
        end
    endtask

    task show_name;
        input integer fi;
        begin
            case (fi)
                0: $display("  [info] 正确帧 \"GOOD\"");
                1: $display("  [info] 错误帧 0：坏 FCS");
                2: $display("  [info] 错误帧 1：短帧");
                3: $display("  [info] 错误帧 2：DA 常量域错");
                4: $display("  [info] 错误帧 3：未知 VLID");
                5: $display("  [info] 错误帧 4：EtherType 错");
                6: $display("  [info] 错误帧 5：IP 校验和错");
                7: $display("  [info] 错误帧 6：IP 协议错（TCP）");
                8: $display("  [info] 错误帧 7：UDP Length 错");
                9: $display("  [info] 错误帧 8：SA IFID 错");
                default: $display("  [info] 未知帧");
            endcase
        end
    endtask

    integer tmo, m, fi, aidx_before, good_ok;

    initial begin
        $readmemh(VEC_FILE, gmem);
        gnfr = gmem[0];
        if (gnfr > 16) gnfr = 16;

        off = 1;
        for (i = 0; i < gnfr; i = i + 1) begin
            g_wlen[i]   = gmem[off];
            g_port[i]   = gmem[off+1];
            g_elen[i]   = gmem[off+2];
            g_pstart[i] = off + 3;
            off = off + 3 + g_elen[i];
            g_wstart[i] = off;
            off = off + g_wlen[i];
        end

        $display("");
        $display("======== AFDX_top 错误帧 + 正确帧混合 TB ========");
        $display("  [info] 文件 %0s：%0d 帧（%0d 错误 + %0d 正确）",
                 VEC_FILE, gnfr, N_ERR, gnfr-N_ERR);
        chk(gnfr === (N_ERR + 1), "T0 文件帧数 = 错误帧数 + 1 条正确帧");

        #200 reset = 1'b0;
        tmo = 0;
        while (!mac_cfg_done && tmo < 2000000) begin @(posedge clk); tmo = tmo + 1; end
        chk(mac_cfg_done === 1'b1, "T1 MAC 配置完成");
        tmo = 0;
        while (!soc_boot_done && tmo < 2000000) begin @(posedge clk); tmo = tmo + 1; end
        chk(soc_boot_done === 1'b1, "T2 SoC 放行");

        repeat (BOOT_QUIET) @(posedge clk);
        aidx = 0; awp = 0; txv_d = 0;

        good_ok = 0;
        for (fi = 0; fi < gnfr; fi = fi + 1) begin
            show_name(fi);
            aidx_before = aidx;
            gmii_send_idx(fi);

            if (fi != 0) begin
                repeat (ERR_QUIET) @(posedge clk);
                if ((aidx !== aidx_before) || (find_app_file(fi) >= 0)) begin
                    $display("  [FAIL] 错误帧 %0d 被错误交付到应用口（aidx %0d→%0d）",
                             fi-1, aidx_before, aidx);
                    errs = errs + 1;
                end else begin
                    $display("  [ok]   错误帧 %0d 未被交付", fi-1);
                end
            end else begin
                tmo = 0;
                while ((find_app_file(fi) < 0) && (tmo < WAIT_GOOD)) begin
                    repeat (100) @(posedge clk);
                    tmo = tmo + 100;
                end
                m = find_app_file(fi);
                if (m < 0) begin
                    $display("  [FAIL] 正确帧没有被交付到应用口");
                    errs = errs + 1;
                end else begin
                    $display("  [info] 应用口收到正确帧：port=%0d len=%0d", aport[m], alen[m]);
                    good_ok = 1;
                end
            end
        end

        chk(good_ok === 1'b1, "T3 正确帧 \"GOOD\" 正常交付");
        chk(aidx === 1, "T4 应用口只收到 1 条消息（错误帧一条都没混进来）");

        $display("  [info] err=%b（app_bridge 粘滞位，错误帧用例不作为判据）", err);

        $display("==================================================");
        if (errs == 0) $display(">> PASS: 错误帧全被丢弃，正确帧正常交付");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #20000000;
        $display(">> TIMEOUT: cfg=%b boot=%b aidx=%0d", mac_cfg_done, soc_boot_done, aidx);
        $finish;
    end

endmodule
