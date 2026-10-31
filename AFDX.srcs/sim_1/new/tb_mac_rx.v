// tb_mac_rx.v —— afdx_mac_wrap 接收侧专项测试（走真实 MAC IP 门级网表）
`timescale 1ns/1ps

module tb_mac_rx #(

    parameter integer LONGLEN   = 1200
) ();

    reg clk    = 1'b0;
    reg p0_rxc = 1'b0;
    reg reset  = 1'b1;
    always #40 clk    = ~clk;        // 12.5 MHz 系统/MAC clk_sys

    always #40 p0_rxc = ~p0_rxc;     // 12.5 MHz 线侧（与 clk 同频）

    // ---- 线侧：GMII 自回环（与 tb_afdx_es 同款，AFDXMACIP 验证过）----
    wire [7:0] gmii_txd;  wire gmii_txen, gmii_txer, gmii_gtxc;
    reg  [7:0] gmii_rxd = 8'h00;
    reg        gmii_rxdv = 1'b0;
    reg        gmii_rxer = 1'b0;
    reg  [7:0] lb_d = 8'h00;
    reg        lb_v = 1'b0;

    always @(negedge p0_rxc) begin lb_d <= gmii_txd; lb_v <= gmii_txen; end
    always @(*) begin gmii_rxd = lb_d; gmii_rxdv = lb_v; gmii_rxer = 1'b0; end

    wire mdc, mdio, phy_rstb0;

    // ---- 帧源（喂 ff_tx）----
    reg        fs_valid = 1'b0;
    reg        fs_sop = 1'b0, fs_eop = 1'b0;
    reg  [7:0] fs_data = 8'h00;
    wire       fs_ready;

    // ---- 客户端（收 ff_rx 出来的字节）----
    wire       rx_valid, rx_sop, rx_eop;
    wire [7:0] rx_data;
    reg        rx_ready = 1'b1;
    wire       mac_cfg_done;

    afdx_mac_wrap #(.NET_SEL(1'b0)) dut (
        .clk(clk), .reset(reset),
        .gmii_txd(gmii_txd), .gmii_txen(gmii_txen), .gmii_txer(gmii_txer),
        .gmii_gtxc(gmii_gtxc),
        .gmii_rxd(gmii_rxd), .gmii_rxdv(gmii_rxdv), .gmii_rxer(gmii_rxer),
        .gmii_rxc(p0_rxc),
        .mdc(mdc), .mdio(mdio), .phy_rstb0(phy_rstb0),
        .fr_valid(fs_valid), .fr_ready(fs_ready), .fr_sop(fs_sop), .fr_eop(fs_eop),
        .fr_data(fs_data), .fr_net(1'b0),
        .rx_valid(rx_valid), .rx_sop(rx_sop), .rx_eop(rx_eop), .rx_data(rx_data),
        .rx_ready(rx_ready), .mac_cfg_done(mac_cfg_done)
    );


    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(1)) u_phy (.mdc(mdc), .mdio(mdio));
    pullup (mdio);

    // ---- 反压发生器：en_bp=1 时 rx_ready 每 10 拍只高 7 拍；hold_rdy=1 时**全程停读** ----
    reg        en_bp = 1'b0;
    reg        hold_rdy = 1'b0;
    reg  [3:0] bpc   = 4'd0;
    always @(posedge clk) bpc <= bpc + 4'd1;
    always @(posedge clk) rx_ready <= !hold_rdy && (en_bp ? (bpc < 4'd7) : 1'b1);


    integer max_used = 0;
    always @(posedge clk) if (dut.skd_used > max_used) max_used <= dut.skd_used;


    reg acc_q;
    always @(posedge clk) acc_q <= fs_valid && fs_ready;


    // ---- 线侧抓帧（看帧到底有没有从 MAC 发出去）----
    reg [7:0] line [0:255]; integer nline = 0; reg line_prev = 0;
    always @(posedge p0_rxc) begin
        line_prev <= gmii_txen;
        if (gmii_txen && nline < 256) begin line[nline] <= gmii_txd; nline <= nline + 1; end
    end

    // ---- 客户端抓字节 ----
    reg       cap_en = 1'b0;            // 只在正式两帧期间记账（探测期不记）
    reg [7:0] cap [0:2047];
    integer   ncap = 0;
    integer   sop_pos [0:7];
    integer   eop_pos [0:7];
    integer   nsop = 0, neop = 0;
    always @(posedge clk) begin
        if (cap_en && rx_valid && rx_ready && ncap < 2048) begin
            cap[ncap] <= rx_data;

            if (rx_sop && nsop < 8) begin sop_pos[nsop] <= ncap; nsop <= nsop + 1; end
            if (rx_eop && neop < 8) begin eop_pos[neop] <= ncap; neop <= neop + 1; end
            ncap <= ncap + 1;
        end
    end

    // ---- 调试探针 ----
    integer d_vld = 0, d_sop = 0, d_push = 0, d_rdy = 0, d_wren = 0, d_trdy = 0;
    always @(posedge clk) begin
        if (dut.ff_rx_data_vld) d_vld <= d_vld + 1;
        if (dut.ff_rx_sop)      d_sop <= d_sop + 1;
        if (dut.skd_push)       d_push <= d_push + 1;
        if (dut.ff_rx_rdy)      d_rdy  <= d_rdy  + 1;
        if (dut.ff_tx_wren)     d_wren <= d_wren + 1;   // 写进 MAC 发送 FIFO 的字节数
        if (dut.ff_tx_rdy)      d_trdy <= d_trdy + 1;
    end

    // ---- CRC-32（与 afdx_crc32.v 完全同约定：反射式，初值 FFFFFFFF，不做最终异或）----
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

    // ---- 造一帧：60 字节帧体 + 4 字节 FCS = 64 字节 ----
    reg [7:0] body [0:59];
    reg [7:0] frm  [0:63];
    integer   i, k;
    reg [31:0] c0;

    task build;
        input integer fid;                    // 0 / 1：两帧载荷故意不同
        begin
            body[0]=8'h03; body[1]=8'h00; body[2]=8'h00; body[3]=8'h00;
            body[4]=8'h01; body[5]=8'h23;                       // DA = 03:00:00:00:01:23
            body[6]=8'h02; body[7]=8'h00; body[8]=8'h00; body[9]=8'h00;
            body[10]=8'h01; body[11]=8'h20;                     // SA，IFID=1（A 网）
            body[12]=8'h08; body[13]=8'h00;                     // EtherType
            for (i = 14; i < 60; i = i + 1)
                body[i] = fid[7:0] + i[7:0];                    // 载荷：能逐字节对出来
            c0 = 32'hFFFFFFFF;
            for (i = 0; i < 60; i = i + 1) c0 = crc32b(c0, body[i]);
            c0 = ~c0;                                           // FCS = CRC 取反，低字节先发
            for (i = 0; i < 60; i = i + 1) frm[i] = body[i];
            frm[60] = c0[7:0];  frm[61] = c0[15:8];
            frm[62] = c0[23:16]; frm[63] = c0[31:24];
        end
    endtask

    // ---- 把 frm[] 逐字节喂进 ff_tx（sop/eop 与 data 同拍）----
    task send_frame;
        integer m;
        begin
            for (m = 0; m < 64; m = m + 1) begin
                @(negedge clk);
                fs_valid = 1'b1;
                fs_data  = frm[m];
                fs_sop   = (m == 0);
                fs_eop   = (m == 63);
                while (!fs_ready) @(negedge clk);       // MAC 反压就等
            end
            @(negedge clk); fs_valid = 1'b0; fs_sop = 1'b0; fs_eop = 1'b0;
        end
    endtask


    reg [7:0] lbody [0:LONGLEN-1];
    reg [7:0] lfrm  [0:LONGLEN-1];

    task build_long;
        integer m; reg [31:0] c1;
        begin
            lbody[0]=8'h03; lbody[1]=8'h00; lbody[2]=8'h00; lbody[3]=8'h00;
            lbody[4]=8'h01; lbody[5]=8'h23;                     // DA = 03:00:00:00:01:23
            lbody[6]=8'h02; lbody[7]=8'h00; lbody[8]=8'h00; lbody[9]=8'h00;
            lbody[10]=8'h01; lbody[11]=8'h20;                   // SA，IFID=1（A 网）
            lbody[12]=8'h08; lbody[13]=8'h00;                   // EtherType
            for (m = 14; m < LONGLEN-4; m = m + 1) lbody[m] = 8'hA0 + m[7:0];
            c1 = 32'hFFFFFFFF;
            for (m = 0; m < LONGLEN-4; m = m + 1) c1 = crc32b(c1, lbody[m]);
            c1 = ~c1;
            for (m = 0; m < LONGLEN-4; m = m + 1) lfrm[m] = lbody[m];
            lfrm[LONGLEN-4]=c1[7:0];   lfrm[LONGLEN-3]=c1[15:8];
            lfrm[LONGLEN-2]=c1[23:16]; lfrm[LONGLEN-1]=c1[31:24];
        end
    endtask


    task send_long_frame;
        integer m;
        begin
            for (m = 0; m < LONGLEN; m = m + 1) begin
                @(negedge clk);
                fs_valid = 1'b1; fs_data = lfrm[m];
                fs_sop   = (m == 0);
                fs_eop   = (m == LONGLEN-1);

                while (!(fs_valid && fs_ready)) @(negedge clk);
                @(posedge clk);
            end
            @(negedge clk); fs_valid = 1'b0; fs_sop = 1'b0; fs_eop = 1'b0;
        end
    endtask

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    task wait_cap;
        input integer tgt;
        integer guard;
        begin
            guard = 0;
            while (ncap < tgt && guard < 400000) begin @(posedge clk); guard = guard + 1; end
        end
    endtask

    integer nbad;
    integer probe;
    time    t_cfg = 0;
    integer idle_run;
    integer dv0 = 0, dw0 = 0, dw1 = 0, dpush0 = 0;
    initial begin
        $display("");
        $display("======== afdx_mac_wrap 接收侧专项（MAC IP 门级网表）========");
        #200; reset = 1'b0;
        wait (mac_cfg_done); t_cfg = $time;
        $display("  [info] t=%0t MAC 配置完成（MDIO 自协商）", $time);


        probe = 0;
        while (nline == 0 && probe < 4000) begin
            build(2);                       // 探测帧：载荷 = 2 + i
            send_frame;
            repeat (100) @(posedge clk);
            probe = probe + 1;
        end
        $display("  [info] MAC 上线：第 %0d 次发帧后线侧才有字节（t=%0t，发帧用了 %0d 拍）",
                 probe, $time, probe*164);
        if (nline == 0) begin
            $display(">> FAIL: MAC 始终没有输出（probe=%0d 次）", probe);
            $finish;
        end

        // 等线侧彻底静下来（FIFO 里可能还压着几帧），再排空接收侧
        idle_run = 0;
        while (idle_run < 20000) begin
            @(posedge p0_rxc);
            if (gmii_txen) idle_run = 0; else idle_run = idle_run + 1;
        end
        repeat (2000) @(posedge clk);       // 把接收侧残留全部排空（cap_en=0 不记账）

        // ---------- 帧 1：无反压 ----------
        cap_en = 1'b1;
        ncap = 0; nsop = 0; neop = 0;
        build(0);
        send_frame;
        $display("  [info] 帧 1 已喂完：线侧字节=%0d 写MAC的wren=%0d(应 %0d) tx_rdy有效=%0d",
                 nline, d_wren, probe*64+64, d_trdy);
        wait_cap(64);
        $display("  [info] 帧 1：收到 %0d 字节  sop@%0d eop@%0d", ncap, sop_pos[0], eop_pos[0]);


        chk(mac_cfg_done === 1'b1 && t_cfg > 0 && t_cfg < 5000000,
            "T1 MAC 在 5 ms 内配置完成（超时/慢一个数量级都会红）");
        chk(ncap === 64, "T2 帧 1 收到 64 字节（帧体+FCS，前导/SFD 都没混进来）");
        nbad = 0;
        for (i = 0; i < 64; i = i + 1) if (cap[i] !== frm[i]) nbad = nbad + 1;
        chk(nbad === 0, "T3 帧 1 逐字节一致");
        chk(nsop === 1 && sop_pos[0] === 0 && neop === 1 && eop_pos[0] === 63,
            "T4 帧 1 的 sop 在第 1 字节、eop 在最后 1 字节");

        // ---------- 帧 2：开反压 ----------
        en_bp = 1'b1;                       // 每 10 拍停 3 拍
        repeat (100) @(posedge clk);
        build(1);
        send_frame;
        wait_cap(128);
        $display("  [info] 帧 2：收到 %0d 字节  sop@%0d eop@%0d（带反压）",
                 ncap, sop_pos[1], eop_pos[1]);

        chk(ncap === 128, "T5-1 帧 2 在反压下仍收满 64 字节（FIFO 没丢字节）");
        nbad = 0;
        for (i = 0; i < 64; i = i + 1) if (cap[64+i] !== frm[i]) nbad = nbad + 1;
        chk(nbad === 0, "T5-2 帧 2 逐字节一致（没有错位/串帧）");
        chk(nsop === 2 && sop_pos[1] === 64 && neop === 2 && eop_pos[1] === 127,
            "T6 两帧的 sop/eop 位置正确 → 剥 SFD 逻辑可重入");


        begin : t8
            integer g, sp, ncmp;
            cap_en = 1'b0; en_bp = 1'b0; hold_rdy = 1'b0;

            idle_run = 0;
            while (idle_run < 20000) begin
                @(posedge p0_rxc);
                if (gmii_txen) idle_run = 0; else idle_run = idle_run + 1;
            end
            repeat (2000) @(posedge clk);

            g = 0;
            while (g < 200) begin
                @(posedge clk);
                if (dut.skd_used == 0) g = g + 1; else g = 0;
            end
            dw0 = d_wren; dv0 = d_vld; dpush0 = d_push;
            chk(dut.skd_used === 0, "T8-0 起手：接收 FIFO 已排空（对照）");
            max_used = 0; ncap = 0; nsop = 0; neop = 0;
            dv0 = d_vld;
            hold_rdy = 1'b1;
            repeat (50) @(posedge clk);
            build_long;
            send_long_frame;
            dw1 = d_wren;
            repeat (400) @(posedge clk);                       // 等帧尾从 MAC 挪进 FIFO
            $display("T8SWEEP LONGLEN=%0d mac_in=%0d mac_out=%0d pushed=%0d max_used=%0d",
                     LONGLEN, dw1 - dw0, d_vld - dv0, d_push - dpush0, max_used);

            $display("T8NOTE 帧长=%0d：引擎喂入 %0d 字节、MAC 吐出 %0d 字节（差 %0d）",
                     LONGLEN, dw1 - dw0, d_vld - dv0, (dw1 - dw0) - (d_vld - dv0));

            chk((d_push - dpush0) >= (LONGLEN - 4),
                "T8-1a 用例非空：MAC 把这一帧的字节基本都吐出来了");
            chk(max_used >= (d_push - dpush0),
                "T8-1 ★★停读期间 FIFO 把 MAC 吐的全部字节都装下了（旧 528 字节实现会丢）");
            chk(max_used <= 2048,
                "T8-1b 占用没有超过 FIFO 容量（2 KB）");

            hold_rdy = 1'b0; cap_en = 1'b1;
            repeat (LONGLEN + 600) @(posedge clk);
            cap_en = 1'b0;
            sp = (nsop > 0) ? sop_pos[nsop-1] : -1;

            nbad = 0; ncmp = 0;
            for (g = 0; g < LONGLEN; g = g + 1)
                if ((sp >= 0) && ((sp+g) < 2048)) begin
                    ncmp = ncmp + 1;
                    if (cap[sp+g] !== lfrm[g]) nbad = nbad + 1;
                end
            $display("T8CHK sp=%0d ncap=%0d nsop=%0d neop=%0d eop0=%0d nbad=%0d ncmp=%0d",
                     sp, ncap, nsop, neop, eop_pos[0], nbad, ncmp);
            chk(sp >= 0,          "T8-2a 抓到了长帧的 sop（否则下面的比对是空的）");
            chk(ncmp === LONGLEN, "T8-2b 真的比了 LONGLEN 个字节（不是零比较/被截断）");
            chk(nbad === 0,       "T8-2 ★★长帧逐字节一致（时钟对齐后应成立）");
        end

        $display("  [info] 探针: MAC侧 vld=%0d sop=%0d push=%0d rdy有效=%0d",
                 d_vld, d_sop, d_push, d_rdy);
        $display("======================================================");
        if (errs == 0) $display(">> PASS: MAC 接收侧（剥 SFD + 反压 + 长帧深 FIFO）完全正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #40000000;                         // 40 ms 兜底（MAC 唤醒最长 16 ms + 收发）
        $display(">> TIMEOUT: mac_cfg_done=%b ncap=%0d", mac_cfg_done, ncap);
        $display("   探针: MAC侧 vld=%0d sop=%0d push=%0d FIFO占=%0d rdy有效=%0d",
                 d_vld, d_sop, d_push, dut.skd_used, d_rdy);
        $finish;
    end

endmodule
