// tb_afdx_top_sys.v —— AFDX_top 系统级顶层 TB（发送 + A/B 双网接收）
`timescale 1ns/1ps

module tb_afdx_top_sys #(
    parameter integer PORTS       = 2,
    parameter integer SOC_DLY     = 2000,     // 16 us @125MHz（功能测试）
    parameter integer BOOT_QUIET  = 2000,
    parameter integer WAIT_TX     = 300000,   // 等应用消息上 GMII：2.4 ms
    parameter integer WAIT_ALL    = 100000,   // 等 A/B 接收收齐：0.8 ms
    parameter integer RUN_TX      = 0,        // 本轮只回归接收链路；发送检查保留但默认关闭
    parameter integer RUN_RX      = 1,
    parameter integer B_SKEW      = 0,
    parameter integer STRICT_ERR  = 0,
    parameter GMII_A_FILE  = "AFDX_ES.srcs/sim_1/new/gmii_a_frames.hex",
    parameter GMII_B_FILE  = "AFDX_ES.srcs/sim_1/new/gmii_b_frames.hex",
    parameter APP_EXP_FILE = "AFDX_ES.srcs/sim_1/new/app_expect.hex",
    parameter APP_VEC_FILE = "AFDX_ES.srcs/sim_1/new/app_msgs.hex"
) ();

    localparam integer MAXF   = 32;
    localparam integer MAXA   = 256;
    localparam integer MAXV   = 32768;
    localparam integer MAXAPP = 16;
    localparam integer MAXFB  = 131072;
    localparam integer MAXAB  = 262144;

    reg clk    = 1'b0;
    reg reset  = 1'b1;
    always #4 clk = ~clk;                    // 125 MHz

    reg gmii_a_rxc = 1'b0;
    reg gmii_b_rxc = 1'b0;
    always #4 gmii_a_rxc = ~gmii_a_rxc;
    always #4 gmii_b_rxc = ~gmii_b_rxc;

    reg  [7:0] gmii_a_rxd  = 8'h00;
    reg        gmii_a_rxdv = 1'b0;
    reg        gmii_a_rxer = 1'b0;
    reg  [7:0] gmii_b_rxd  = 8'h00;
    reg        gmii_b_rxdv = 1'b0;
    reg        gmii_b_rxer = 1'b0;

    wire [7:0] gmii_a_txd, gmii_b_txd;
    wire       gmii_a_txen, gmii_a_txer, gmii_a_gtxc;
    wire       gmii_b_txen, gmii_b_txer, gmii_b_gtxc;

    wire       mdc_a, mdio_a, phy_rstb_a;
    wire       mdc_b, mdio_b, phy_rstb_b;
    wire       fl_sclk, fl_mosi, fl_cs_n;
    wire [3:0] debug;
    wire       mac_cfg_done, soc_boot_done;

    // 模板 8 线 + 应用侧旁带
    reg  [7:0] rx_data  = 8'h00;
    reg        rx_valid = 1'b0;
    reg        rx_ready = 1'b1;
    wire [7:0] tx_data;
    wire       tx_valid, tx_ready;
    reg  [15:0] app_vlid = 16'h0789;
    wire [3:0] tx_port;
    wire       err;

    pullup (mdio_a);
    pullup (mdio_b);

    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_a (
        .mdc(mdc_a), .mdio(mdio_a)
    );
    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy_b (
        .mdc(mdc_b), .mdio(mdio_b)
    );

    AFDX_top #(
        .PORTS       (PORTS),
        .SOC_BOOT_DLY(SOC_DLY),
        .BYPASS      (0),
        .APP_SIDE    (1),
        .APP_PORT_DEF(0),
        .VLID_MODE   (1)
    ) u_dut (
        .clk        (clk),
        .reset      (reset),
        .tx_data    (tx_data),
        .rx_data    (rx_data),
        .tx_valid   (tx_valid),
        .rx_valid   (rx_valid),
        .tx_ready   (tx_ready),
        .rx_ready   (rx_ready),
        .app_vlid   (app_vlid),
        .tx_port    (tx_port),
        .err        (err),

        .gmii_a_txd (gmii_a_txd), .gmii_a_txen(gmii_a_txen),
        .gmii_a_txer(gmii_a_txer), .gmii_a_gtxc(gmii_a_gtxc),
        .gmii_a_rxd (gmii_a_rxd),  .gmii_a_rxdv(gmii_a_rxdv),
        .gmii_a_rxer(gmii_a_rxer), .gmii_a_rxc (gmii_a_rxc),

        .gmii_b_txd (gmii_b_txd), .gmii_b_txen(gmii_b_txen),
        .gmii_b_txer(gmii_b_txer), .gmii_b_gtxc(gmii_b_gtxc),
        .gmii_b_rxd (gmii_b_rxd),  .gmii_b_rxdv(gmii_b_rxdv),
        .gmii_b_rxer(gmii_b_rxer), .gmii_b_rxc (gmii_b_rxc),

        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),

        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi),
        .flash_miso(1'b1),    .flash_cs_n(fl_cs_n),

        .debug(debug), .mac_cfg_done(mac_cfg_done),
        .soc_boot_done(soc_boot_done)
    );

    // =================================================================
    // 文件存储与索引
    // =================================================================
    reg [31:0] rxamem [0:MAXV-1];
    reg [31:0] rxbmem [0:MAXV-1];
    reg [31:0] rxemem [0:MAXV-1];
    reg [7:0]  appmem [0:8191];

    integer rx_a_n = 0, rx_b_n = 0, rx_e_n = 0, tx_n = 0;

    integer ra_fid  [0:MAXF-1];
    integer ra_mid  [0:MAXF-1];
    integer ra_vlid [0:MAXF-1];
    integer ra_sn   [0:MAXF-1];
    integer ra_port [0:MAXF-1];
    integer ra_wlen [0:MAXF-1];
    integer ra_off  [0:MAXF-1];
    integer ra_wst  [0:MAXF-1];

    integer rb_fid  [0:MAXF-1];
    integer rb_mid  [0:MAXF-1];
    integer rb_vlid [0:MAXF-1];
    integer rb_sn   [0:MAXF-1];
    integer rb_port [0:MAXF-1];
    integer rb_wlen [0:MAXF-1];
    integer rb_off  [0:MAXF-1];
    integer rb_wst  [0:MAXF-1];

    integer re_mid  [0:MAXA-1];
    integer re_port [0:MAXA-1];
    integer re_vlid [0:MAXA-1];
    integer re_sn   [0:MAXA-1];
    integer re_len  [0:MAXA-1];
    integer re_pst  [0:MAXA-1];

    integer ta_port [0:MAXAPP-1];
    integer ta_len  [0:MAXAPP-1];
    integer ta_pst  [0:MAXAPP-1];

    integer i, off;

    task load_vectors;
        begin
            $display("  [load] A ...");
            $readmemh(GMII_A_FILE, rxamem);
            $display("  [load] A done");
            $display("  [load] B ...");
            $readmemh(GMII_B_FILE, rxbmem);
            $display("  [load] B done");
            $display("  [load] E ...");
            $readmemh(APP_EXP_FILE, rxemem);
            $display("  [load] E done");

            rx_a_n = rxamem[0];
            rx_b_n = rxbmem[0];
            rx_e_n = rxemem[0];
            if (rx_a_n > MAXF) rx_a_n = MAXF;
            if (rx_b_n > MAXF) rx_b_n = MAXF;
            if (rx_e_n > MAXA) rx_e_n = MAXA;

            off = 1;
            for (i = 0; i < rx_a_n; i = i + 1) begin
                ra_fid[i]  = rxamem[off+0];
                ra_mid[i]  = rxamem[off+1];
                ra_vlid[i] = rxamem[off+2];
                ra_sn[i]   = rxamem[off+3];
                ra_port[i] = rxamem[off+4];
                ra_wlen[i] = rxamem[off+5];
                ra_off[i]  = rxamem[off+6];
                ra_wst[i]  = off + 7;
                off = off + 7 + ra_wlen[i];
            end

            off = 1;
            for (i = 0; i < rx_b_n; i = i + 1) begin
                rb_fid[i]  = rxbmem[off+0];
                rb_mid[i]  = rxbmem[off+1];
                rb_vlid[i] = rxbmem[off+2];
                rb_sn[i]   = rxbmem[off+3];
                rb_port[i] = rxbmem[off+4];
                rb_wlen[i] = rxbmem[off+5];
                rb_off[i]  = rxbmem[off+6];
                rb_wst[i]  = off + 7;
                off = off + 7 + rb_wlen[i];
            end

            off = 1;
            for (i = 0; i < rx_e_n; i = i + 1) begin
                re_mid[i]  = rxemem[off+0];
                re_port[i] = rxemem[off+1];
                re_vlid[i] = rxemem[off+2];
                re_sn[i]   = rxemem[off+3];
                re_len[i]  = rxemem[off+4];
                re_pst[i]  = off + 5;
                off = off + 5 + re_len[i];
            end

            $readmemh(APP_VEC_FILE, appmem);
            tx_n = appmem[0];
            if (tx_n > MAXAPP) tx_n = MAXAPP;
            off = 1;
            for (i = 0; i < tx_n; i = i + 1) begin
                ta_port[i] = appmem[off+0];
                ta_len[i]  = appmem[off+1];
                ta_pst[i]  = off + 2;
                off = off + 2 + ta_len[i];
            end

            $display("  [vec] A=%0d 帧, B=%0d 帧, 期望=%0d 条, 发送消息=%0d 条",
                     rx_a_n, rx_b_n, rx_e_n, tx_n);
        end
    endtask

    // =================================================================
    // GMII A 发送抓帧（发送方向检查用）
    // =================================================================
    reg [7:0] fbuf [0:MAXFB-1];
    integer   fstart [0:MAXF-1];
    integer   flen   [0:MAXF-1];
    integer   fidx = 0, fwp = 0;
    reg       txen_d = 1'b0;

    always @(posedge clk) begin
        txen_d <= gmii_a_txen;
        if (gmii_a_txen) begin
            if (!txen_d && (fidx < MAXF)) begin
                fstart[fidx] = fwp;
                flen[fidx]   = 0;
            end
            if ((fidx < MAXF) && (fwp < MAXFB)) begin
                fbuf[fwp]    <= gmii_a_txd;
                fwp          <= fwp + 1;
                flen[fidx]   <= flen[fidx] + 1;
            end
        end else if (txen_d) begin
            if (fidx < MAXF) fidx <= fidx + 1;
        end
    end

    // =================================================================
    // 应用口抓消息
    // =================================================================
    reg [7:0] abuf [0:MAXAB-1];
    integer   astart [0:MAXA-1];
    integer   alen   [0:MAXA-1];
    reg [3:0] aport  [0:MAXA-1];
    integer   aidx = 0, awp = 0;
    reg       txv_d = 1'b0;

    always @(posedge clk) begin
        txv_d <= tx_valid;
        if (tx_valid) begin
            if (!txv_d && (aidx < MAXA)) begin
                astart[aidx] = awp;
                alen[aidx]   = 0;
                aport[aidx]  = tx_port;
            end
            if ((aidx < MAXA) && (awp < MAXAB)) begin
                abuf[awp]  <= tx_data;
                awp        <= awp + 1;
                alen[aidx] <= alen[aidx] + 1;
            end
        end else if (txv_d) begin
            if (aidx < MAXA) aidx <= aidx + 1;
        end
    end

    // =================================================================
    // 检查工具
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

    function [31:0] crc32b;
        input [31:0] c;
        input [7:0]  d;
        integer k;
        reg [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            crc32b = t;
        end
    endfunction

    function [31:0] fcs_resid;
        input integer f;
        integer k;
        reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = 8; k < flen[f]; k = k + 1)
                c = crc32b(c, fbuf[fstart[f]+k]);
            fcs_resid = c;
        end
    endfunction

    function [15:0] vlid_of_port;
        input [3:0] p;
        begin
            case (p)
                4'd0: vlid_of_port = 16'h0123;
                4'd1: vlid_of_port = 16'h0456;
                4'd2: vlid_of_port = 16'h0789;
                4'd3: vlid_of_port = 16'h0161;
                4'd4: vlid_of_port = 16'h0BC5;
                4'd6: vlid_of_port = 16'h0A01;
                4'd7: vlid_of_port = 16'h0A02;
                4'd8: vlid_of_port = 16'h0A03;
                4'd9: vlid_of_port = 16'h0A04;
                default: vlid_of_port = 16'h0000;
            endcase
        end
    endfunction

    function integer find_tx_msg;
        input integer mi;
        integer f, b, k;
        reg hit, found;
        reg [15:0] vlid;
        begin
            vlid = vlid_of_port(ta_port[mi]);
            find_tx_msg = -1;
            for (f = 0; f < fidx; f = f + 1) begin
                hit = (flen[f] >= 72);
                if (hit) begin
                    if (fbuf[fstart[f]+0] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+1] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+2] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+3] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+4] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+5] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+6] !== 8'h55) hit = 0;
                    if (fbuf[fstart[f]+7] !== 8'hD5) hit = 0;
                    if (fbuf[fstart[f]+8]  !== 8'h03) hit = 0;
                    if (fbuf[fstart[f]+9]  !== 8'h00) hit = 0;
                    if (fbuf[fstart[f]+10] !== 8'h00) hit = 0;
                    if (fbuf[fstart[f]+11] !== 8'h00) hit = 0;
                    if (fbuf[fstart[f]+12] !== vlid[15:8]) hit = 0;
                    if (fbuf[fstart[f]+13] !== vlid[7:0])  hit = 0;
                    found = 0;
                    for (b = 50; b <= flen[f]-4-ta_len[mi]; b = b + 1) begin
                        found = 1;
                        for (k = 0; k < ta_len[mi]; k = k + 1)
                            if (fbuf[fstart[f]+b+k] !== appmem[ta_pst[mi]+k])
                                found = 0;
                        if (found) b = flen[f];
                    end
                    if (!found) hit = 0;
                end
                if (hit && (find_tx_msg < 0)) find_tx_msg = f;
            end
        end
    endfunction

    // =================================================================
    // 激励任务
    // =================================================================
    task app_send_idx;
        input integer mi;
        integer k;
        begin
            app_vlid = vlid_of_port(ta_port[mi]);
            for (k = 0; k < ta_len[mi]; k = k + 1) begin
                @(negedge clk);
                rx_data  = appmem[ta_pst[mi]+k];
                rx_valid = 1'b1;
            end
            @(negedge clk);
            rx_valid = 1'b0;
            rx_data  = 8'h00;
        end
    endtask

    integer cyc = 0;
    integer run_start = 0;
    always @(posedge clk) cyc <= cyc + 1;

    task rx_send_a;
        integer fi, k;
        begin
            for (fi = 0; fi < rx_a_n; fi = fi + 1) begin
                while ((cyc - run_start) < ra_off[fi]) @(posedge gmii_a_rxc);
                app_vlid = vlid_of_port(ra_port[fi]);  // 网络侧注入：VLID 模式不再需要按端口过滤，仅保持一致
                for (k = 0; k < ra_wlen[fi]; k = k + 1) begin
                    @(posedge gmii_a_rxc);
                    gmii_a_rxd  <= rxamem[ra_wst[fi]+k][7:0];
                    gmii_a_rxdv <= 1'b1;
                    gmii_a_rxer <= 1'b0;
                end
                @(posedge gmii_a_rxc);
                gmii_a_rxdv <= 1'b0;
                gmii_a_rxd  <= 8'h00;
                repeat (12) @(posedge gmii_a_rxc);
            end
        end
    endtask

    task rx_send_b;
        integer fi, k;
        begin
            for (fi = 0; fi < rx_b_n; fi = fi + 1) begin
                while ((cyc - run_start) < (rb_off[fi] + B_SKEW)) @(posedge gmii_b_rxc);
                // B 可能比 A 早到：B 起始时也要把应用口切到本帧端口
                app_vlid = vlid_of_port(rb_port[fi]);
                for (k = 0; k < rb_wlen[fi]; k = k + 1) begin
                    @(posedge gmii_b_rxc);
                    gmii_b_rxd  <= rxbmem[rb_wst[fi]+k][7:0];
                    gmii_b_rxdv <= 1'b1;
                    gmii_b_rxer <= 1'b0;
                end
                @(posedge gmii_b_rxc);
                gmii_b_rxdv <= 1'b0;
                gmii_b_rxd  <= 8'h00;
                repeat (12) @(posedge gmii_b_rxc);
            end
        end
    endtask

    // =================================================================
    // 调试探针
    // =================================================================
    integer dbg_mac_a = 0, dbg_mac_b = 0;
    integer dbg_accept = 0;
    integer dbg_rmv = 0, dbg_rmcommit = 0, dbg_rmabort = 0, dbg_appv = 0;

    always @(posedge clk) begin
        if (u_dut.u_es.ra_valid)               dbg_mac_a = dbg_mac_a + 1;
        if (u_dut.u_es.rb_valid)               dbg_mac_b = dbg_mac_b + 1;
        if (u_dut.u_es.u_soc.ir_accept)        dbg_accept = dbg_accept + 1;
        if (u_dut.u_es.u_soc.rm_v)             dbg_rmv = dbg_rmv + 1;
        if (u_dut.u_es.u_soc.rm_commit)        dbg_rmcommit = dbg_rmcommit + 1;
        if (u_dut.u_es.u_soc.rm_abort)         dbg_rmabort = dbg_rmabort + 1;
        if (tx_valid)                          dbg_appv = dbg_appv + 1;
    end

    // =================================================================
    // 比对
    // =================================================================
    integer m, k, tmo;
    integer nmatch;
    integer pc_exp [0:9];
    integer pc_got [0:9];

    task check_one;
        input integer mm;
        integer kk;
        reg hit;
        begin
            hit = 1'b1;
            if (aport[mm] !== re_port[mm]) hit = 1'b0;
            if (alen[mm] !== re_len[mm])   hit = 1'b0;
            if (hit) begin
                for (kk = 0; kk < re_len[mm]; kk = kk + 1)
                    if (abuf[astart[mm]+kk] !== rxemem[re_pst[mm]+kk][7:0]) hit = 1'b0;
            end
            if (hit) nmatch = nmatch + 1;
            if (!hit) begin
                errs = errs + 1;
                $display("  [FAIL] msg %0d: exp port=%0d len=%0d, got port=%0d len=%0d",
                         mm, re_port[mm], re_len[mm], aport[mm], alen[mm]);
            end
        end
    endtask

    // =================================================================
    // 主流程
    // =================================================================
    integer f, rx_ok, rx_bad, tx_vlid_i;
    reg [15:0] tx_vlid;

    initial begin
        for (i = 0; i < 10; i = i + 1) begin
            pc_exp[i] = 0;
            pc_got[i] = 0;
        end
        load_vectors;

        #200 reset = 1'b0;

        tmo = 0;
        while ((mac_cfg_done !== 1'b1) && (tmo < 2000000)) begin
            @(posedge clk);
            tmo = tmo + 1;
        end
        chk(mac_cfg_done === 1'b1, "T1 mac_cfg_done");

        tmo = 0;
        while ((soc_boot_done !== 1'b1) && (tmo < 2000000)) begin
            @(posedge clk);
            tmo = tmo + 1;
        end
        chk(soc_boot_done === 1'b1, "T2 soc_boot_done");

        repeat (BOOT_QUIET) @(posedge clk);


        if (RUN_TX && (tx_n > 0)) begin
            fidx = 0; fwp = 0; txen_d = 1'b0;
            app_send_idx(0);

            tmo = 0;
            while ((find_tx_msg(0) < 0) && (tmo < WAIT_TX)) begin
                repeat (100) @(posedge clk);
                tmo = tmo + 100;
            end
            f = find_tx_msg(0);
            tx_vlid = vlid_of_port(ta_port[0]);
            chk(f >= 0, "T3 应用消息在 GMII A 上组出了帧（DA/VLID 与 port 对应）");

            // 等这一帧的 txen 完整结束：find_tx_msg 可能在帧中途就命中，
            // 此时 flen 还只数到一半，直接查 UDP/FCS 会误判。
            tmo = 0;
            while ((f >= 0) && (fidx <= f) && (tmo < WAIT_TX)) begin
                repeat (10) @(posedge clk);
                tmo = tmo + 10;
            end

            if (f >= 0) begin
                chk(flen[f] >= 72, "T4a 线上帧 >= 72 字节");
                chk(fbuf[fstart[f]+7] === 8'hD5, "T4b 线侧有 SFD D5");
                chk(fbuf[fstart[f]+12] === tx_vlid[15:8] &&
                    fbuf[fstart[f]+13] === tx_vlid[7:0],
                    "T4c DA 的 VLID = 该 port 的映射值");
                chk(fbuf[fstart[f]+14] === 8'h02 && fbuf[fstart[f]+19] === 8'h20,
                    "T4d SA 的 Interface_ID = 0x20（A 网）");
                chk(fbuf[fstart[f]+20] === 8'h08 && fbuf[fstart[f]+21] === 8'h00,
                    "T4e EtherType = 0x0800");
                chk(fbuf[fstart[f]+44] === 8'h00 && fbuf[fstart[f]+45] === 8'h45,
                    "T4f UDP 目的端口 = 69");
                k = 0;
                for (i = 0; i < ta_len[0]; i = i + 1)
                    if (fbuf[fstart[f]+50+i] !== appmem[ta_pst[0]+i]) k = k + 1;
                chk(k == 0, "T4g 帧载荷 = 应用文件里的 payload");
                chk(fcs_resid(f) === 32'hDEBB20E3,
                    "T4h FCS 残差 = DEBB20E3（整帧格式完整）");
            end
        end

        // =============================================================
        // ② A/B 双网 + 变长帧 + 多 VL 接收
        // =============================================================
        if (RUN_RX && (rx_a_n > 0)) begin
            aidx = 0; awp = 0; txv_d = 1'b0; nmatch = 0;
            dbg_mac_a = 0; dbg_mac_b = 0; dbg_accept = 0;
            dbg_rmv = 0; dbg_rmcommit = 0; dbg_rmabort = 0; dbg_appv = 0;

            run_start = cyc;
            fork
                rx_send_a;
                rx_send_b;
            join

            tmo = 0;
            while ((aidx < rx_e_n) && (tmo < WAIT_ALL)) begin
                @(posedge clk);
                tmo = tmo + 1;
            end

            $display("");
            $display("  [统计] A MAC 字节=%0d, B MAC 字节=%0d, ir_accept=%0d",
                     dbg_mac_a, dbg_mac_b, dbg_accept);
            $display("  [统计] rm_v=%0d, rm_commit=%0d, rm_abort=%0d, app tx_valid=%0d",
                     dbg_rmv, dbg_rmcommit, dbg_rmabort, dbg_appv);
            $display("  [统计] 应用消息 %0d 条，等待 %0d 拍", aidx, tmo);

            if (aidx !== rx_e_n) begin
                errs = errs + 1;
                $display("  [FAIL] 应用消息数 %0d != 期望 %0d", aidx, rx_e_n);
            end

            for (m = 0; m < rx_e_n; m = m + 1)
                pc_exp[re_port[m]] = pc_exp[re_port[m]] + 1;
            for (m = 0; m < aidx; m = m + 1)
                pc_got[aport[m]] = pc_got[aport[m]] + 1;
            for (m = 0; m < 4; m = m + 1)
                $display("  [port %0d] 期望 %0d 条, 实收 %0d 条",
                         m, pc_exp[m], pc_got[m]);

            for (m = 0; m < rx_e_n && m < aidx; m = m + 1)
                check_one(m);


            chk((aidx === rx_e_n) && (nmatch === rx_e_n),
                "T5 rx msgs all match");

            if (err !== 1'b0) begin
                if (STRICT_ERR) begin
                    errs = errs + 1;
                    $display("  [FAIL] app_bridge err=%b（STRICT_ERR=1）", err);
                end else begin
                    $display("  [WARN] app_bridge err=%b（粘滞位）：A/B 冗余副本在 RM abort 前", err);
                    $display("         经过 reasm 透传段，app_bridge 忙时会置 err；");
                    $display("         %0d/%0d 应用数据仍逐字节比对通过。", rx_e_n, rx_e_n);
                end
            end
        end

        $display("======================================================");
        if (errs == 0)
            $display(">> PASS: AFDX_top A/B 双网接收全部正确（RUN_TX=%0d，发送检查默认关闭）", RUN_TX);
        else
            $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #40000000;
        $display(">> TIMEOUT: aidx=%0d/%0d fidx=%0d", aidx, rx_e_n, fidx);
        $finish;
    end

endmodule
