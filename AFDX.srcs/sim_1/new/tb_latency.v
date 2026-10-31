// tb_latency.v —— R40 口径：端到端时延 / 同 VL 抖动（变长帧 + 多 VL）
`timescale 1ns/1ps

module tb_latency;

    reg clk = 1'b0;
    reg reset = 1'b1;
    always #4 clk = ~clk;                  // 125 MHz，1 拍 = 8 ns

    localparam integer NV   = 3;           // VL 个数（采样口 / 队列口 / SAP 口）
    localparam integer NL   = 5;           // 帧长档数
    localparam integer NREP   = 8;         // 采样口 VL 每个帧长的样本数（量抖动用）
    localparam integer NREP_L = 1;         // 另两条 VL 每个帧长的样本数（只验"多 VL 公平性"）


    localparam integer JIT_LIM  = 8;
    localparam integer PROC_LIM = 40;      // 内部处理分量上限（拍）

    // ---- 接收字节流（直接驱动 SoC 的 dp_rxin_*）----
    reg        rx_fv = 1'b0, rx_fsop = 1'b0, rx_feop = 1'b0, rx_fnet = 1'b0;
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

    // =====================================================================
    // 计时与计数（**事件捕获与 cyc 自增放在同一个 always 块**，避免两个块的次序竞争）
    // =====================================================================
    integer cyc = 0;
    integer n_deliv = 0, n_ovf = 0, n_commit = 0;
    integer t_deliv = -1, t_ovf = -1, t_commit = -1;

    always @(posedge clk) begin
        cyc = cyc + 1;
        if (|u_dut.rx_deliv_p_w) begin n_deliv = n_deliv + 1; t_deliv = cyc; end
        if (|u_dut.rx_ovf_set_w) begin n_ovf   = n_ovf   + 1; t_ovf   = cyc; end
        // 中间点：IC/RM 判定通过（reasm 输出 commit）。用来把"帧末→交付"拆成可解释的两段
        if (u_dut.rm_commit)     begin n_commit = n_commit + 1; t_commit = cyc; end
    end

    // =====================================================================
    // 帧构造（格式与 tb_rx_burst / tb_darkriscv_soc.inj_udp 一致）
    //   帧体 = [ETH14][IP20][UDP8][payload PLEN][SN 1]，FCS 4 ⇒ 帧长 = 47 + PLEN
    //   SN 紧贴 FCS（P7 §3.2.6.1 / Wireshark 解析器口径），**不计入 IP Total Length**
    // =====================================================================
    reg  [7:0] gf [0:2047];
    integer    gf_len;
    integer    flen_tab [0:NL-1];
    integer    v_vlid [0:NV-1];
    integer    v_dport [0:NV-1];
    integer    sn_v [0:NV-1];

    integer b, plen, iplen, snpos, total;
    reg [31:0] sum, cs, c;
    reg [15:0] vl, dp;                  // 任务内的 16 位副本（integer 的位选取不可移植）

    function [31:0] rcrc32b;
        input [31:0] cc; input [7:0] dd;
        integer k; reg [31:0] t;
        begin
            t = cc ^ {24'h000000, dd};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            rcrc32b = t;
        end
    endfunction

    task build_frame;
        input [15:0] vlid; input [15:0] dport; input integer flen; input [7:0] sn;
        begin
            vl = vlid; dp = dport;
            plen  = flen - 47;
            iplen = 20 + 8 + plen;
            snpos = 14 + iplen;                    // SN 紧贴 FCS
            total = snpos + 1;
            for (b = 0; b < 2048; b = b + 1) gf[b] = 8'h00;
            gf[0]=8'h03; gf[4]=vl[15:8]; gf[5]=vl[7:0];            // DA = 03:00:00:00:VLID
            gf[6]=8'h02; gf[10]=8'h01; gf[11]=8'h20;               // SA，IFID=1（A 网）
            gf[12]=8'h08; gf[13]=8'h00;
            gf[14]=8'h45; gf[15]=8'h00;
            gf[16]=iplen[15:8]; gf[17]=iplen[7:0];
            gf[20]=8'h40; gf[22]=8'h01; gf[23]=8'h11;              // TTL=1, UDP
            gf[26]=8'h0A; gf[27]=8'h00; gf[28]=8'h00; gf[29]=8'h09; // 10.0.0.9
            gf[30]=8'hE0; gf[31]=8'hE0; gf[32]=vl[15:8]; gf[33]=vl[7:0];
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {gf[b], gf[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            gf[24]=cs[15:8]; gf[25]=cs[7:0];
            gf[34]=vl[15:8]; gf[35]=vl[7:0];                       // sport = VLID
            gf[36]=dp[15:8]; gf[37]=dp[7:0];
            gf[38]=(8+plen)>>8; gf[39]=(8+plen)&8'hFF;
            gf[40]=8'h00; gf[41]=8'h00;                            // UDP 校验和 = 0
            for (b = 0; b < plen; b = b + 1) gf[42+b] = (b*7 + sn) & 8'hFF;
            gf[snpos] = sn;                                        // SN
            c = 32'hFFFFFFFF;
            for (b = 0; b < total; b = b + 1) c = rcrc32b(c, gf[b]);
            c = ~c;
            gf[total+0]=c[7:0];  gf[total+1]=c[15:8];
            gf[total+2]=c[23:16]; gf[total+3]=c[31:24];
            gf_len = total + 4;                                    // = 47 + plen
        end
    endtask

    integer t_sop = -1;
    task inject;
        begin
            for (b = 0; b < gf_len; b = b + 1) begin
                @(negedge clk);
                rx_fv = 1'b1; rx_fd = gf[b];
                rx_fsop = (b == 0); rx_feop = (b == gf_len-1);
                rx_fnet = 1'b0;                                    // 只走 A 网（B 副本会被 RM 去重）
                if (b == 0) t_sop = cyc + 1;                       // 下一拍 posedge 被采
                @(posedge clk);
            end
            @(negedge clk); rx_fv = 1'b0; rx_fsop = 1'b0; rx_feop = 1'b0;
        end
    endtask

    // =====================================================================
    // 统计
    // =====================================================================
    integer lat_min [0:NV*NL-1], lat_max [0:NV*NL-1], lat_sum [0:NV*NL-1];
    integer proc_min[0:NV*NL-1], proc_max[0:NV*NL-1];
    integer n_ok[0:NV*NL-1];
    integer n_drop = 0;
    integer d_cm_min = 0, d_cm_max = 0, d_cd_min = 0, d_cd_max = 0;   // 帧末→commit / commit→交付
    integer lat_all_min = 0, lat_all_max = 0;
    integer kidx, n0d, n0o, wt, lat, proc;
    reg [15:0] pvlid;                       // 打印用（integer 的 %h 会打成 32 位）

    task run_case;
        input integer vi; input integer li;
        begin
            pvlid = v_vlid[vi];
            sn_v[vi] = sn_v[vi] + 1;
            build_frame(v_vlid[vi], v_dport[vi], flen_tab[li], sn_v[vi]);
            n0d = n_deliv; n0o = n_ovf;
            inject;
            // 等这一条被交付（或溢出）—— 带超时，别裸 wait
            wt = 0;
            while ((n_deliv == n0d) && (n_ovf == n0o) && (wt < 50000)) begin
                @(posedge clk); wt = wt + 1;
            end
            kidx = vi*NL + li;
            if (n_deliv > n0d) begin
                lat  = t_deliv - t_sop;
                proc = lat - gf_len;
                // 分解：帧末字节（t_sop+gf_len-1）→commit→交付
                if (t_commit >= 0) begin
                    if (d_cm_min == 0 || (t_commit - (t_sop+gf_len-1)) < d_cm_min) d_cm_min = t_commit - (t_sop+gf_len-1);
                    if ((t_commit - (t_sop+gf_len-1)) > d_cm_max) d_cm_max = t_commit - (t_sop+gf_len-1);
                    if (d_cd_min == 0 || (t_deliv - t_commit) < d_cd_min) d_cd_min = t_deliv - t_commit;
                    if ((t_deliv - t_commit) > d_cd_max) d_cd_max = t_deliv - t_commit;
                end
                if (n_ok[kidx] == 0 || lat < lat_min[kidx]) lat_min[kidx] = lat;
                if (lat > lat_max[kidx]) lat_max[kidx] = lat;
                lat_sum[kidx] = lat_sum[kidx] + lat;
                if (n_ok[kidx] == 0 || proc < proc_min[kidx]) proc_min[kidx] = proc;
                if (proc > proc_max[kidx]) proc_max[kidx] = proc;
                n_ok[kidx] = n_ok[kidx] + 1;
                if (lat_all_min == 0 || lat < lat_all_min) lat_all_min = lat;
                if (lat > lat_all_max) lat_all_max = lat;
                $display("  [vl=%04h len=%4d] latency %4d cyc = %6.2f us, processing %3d cyc",
                         pvlid, gf_len, lat, lat*8.0/1000.0, proc);
            end else if (n_ovf > n0o) begin
                n_drop = n_drop + 1;
                $display("  [vl=%04h len=%4d] [DROP-OVF] port queue full, message not delivered", pvlid, gf_len);
            end else begin
                n_drop = n_drop + 1;
                $display("  [vl=%04h len=%4d] [DROP-TIMEOUT] neither delivered nor overflow (IC/RM drop?)",
                         pvlid, gf_len);
            end
            repeat (6) @(posedge clk);          // 帧间隔（真实网络也不是背靠背）
        end
    endtask


    integer i, j, v, l, errs = 0, ok_all = 0, drop_all = 0;
    integer jit_worst = 0, proc_worst = 0, proc_vl_worst = 0;
    integer v_proc_max [0:NV-1], v_proc_min [0:NV-1];
    integer jit, vlmax, vlmin;
    real    mbps;

    task chk;
        input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask

    initial begin
        flen_tab[0] = 64;   flen_tab[1] = 256;  flen_tab[2] = 512;
        flen_tab[3] = 1024; flen_tab[4] = 1518;
        v_vlid[0] = 16'h0123; v_dport[0] = 1024;   // 采样口
        v_vlid[1] = 16'h0456; v_dport[1] = 1025;   // 队列口
        v_vlid[2] = 16'h0789; v_dport[2] = 69;     // SAP 口（TFTP）
        for (i = 0; i < NV; i = i + 1) begin
            sn_v[i] = 0;
            v_proc_max[i] = 0; v_proc_min[i] = 0;
        end
        for (i = 0; i < NV*NL; i = i + 1) begin
            lat_min[i] = 0; lat_max[i] = 0; lat_sum[i] = 0;
            proc_min[i] = 0; proc_max[i] = 0; n_ok[i] = 0;
        end

        #200 reset = 1'b0;
        #2000;                                  // 等上电稳定（配置表 $readmemh 在 t=0 就绪）

        $display("");
        $display("======== R40: end-to-end latency / same-VL jitter (125 MHz, 1 cyc = 8 ns) ========");
        $display("  Scope: dp_rxin first byte -> port-layer deliv (message to partition)");
        $display("  Frame sizes: 64 / 256 / 512 / 1024 / 1518 B; %0d VLs round-robin", NV);

        // ---- 主循环：帧长在外；同一帧长内**三条 VL 交错**（多 VL 并发）----
        for (l = 0; l < NL; l = l + 1) begin
            for (i = 0; i < NREP; i = i + 1) begin
                run_case(0, l);                         // 采样口：量抖动
                if (i == 0) run_case(1, l);             // 队列口：验多 VL
                if (i == 1) run_case(2, l);             // SAP 口：验多 VL
            end
        end

        $display("");
        $display("  ---- detail (jitter = max-min, same VL & same frame size) ----");
        for (v = 0; v < NV; v = v + 1) begin
            vlmax = 0; vlmin = 0;
            for (l = 0; l < NL; l = l + 1) begin
                kidx = v*NL + l;
                if (n_ok[kidx] > 0) begin
                    jit = lat_max[kidx] - lat_min[kidx];
                    if (jit > jit_worst) jit_worst = jit;
                    if (proc_max[kidx] > proc_worst) proc_worst = proc_max[kidx];
                    if (proc_min[kidx] < v_proc_min[v] || v_proc_min[v] == 0) v_proc_min[v] = proc_min[kidx];
                    if (proc_max[kidx] > v_proc_max[v]) v_proc_max[v] = proc_max[kidx];
                    $display("  vl=%04h len=%4d  n=%0d  lat min=%4d max=%4d avg=%6.1f cyc  (jitter %2d cyc = %4.1f ns)  proc %2d..%2d cyc",
                             v_vlid[v], flen_tab[l], n_ok[kidx], lat_min[kidx], lat_max[kidx],
                             lat_sum[kidx]*1.0/n_ok[kidx], jit, jit*8.0, proc_min[kidx], proc_max[kidx]);
                end else begin
                    $display("  vl=%04h len=%4d  n=0 (**no samples**)", v_vlid[v], flen_tab[l]);
                end
            end
            $display("  => VL %04h processing %0d..%0d cyc over all frames (spread %0d cyc)",
                     v_vlid[v], v_proc_min[v], v_proc_max[v], v_proc_max[v]-v_proc_min[v]);
            if (v_proc_max[v] - v_proc_min[v] > proc_vl_worst)
                proc_vl_worst = v_proc_max[v] - v_proc_min[v];
            if (v_proc_max[v] > vlmax) vlmax = v_proc_max[v];
            ok_all = ok_all + 1;
        end

        $display("");
        // [ok] 计数：每条断言一个
        drop_all = n_drop;
        chk(drop_all == 0, "T1 all frames delivered");
        // 样本数核对：VL0 每档 NREP 条、另两条各 NREP_L 条 —— 防止"某档一条都没跑到"被当成全绿
        ok_all = 0;
        for (kidx = 0; kidx < NV*NL; kidx = kidx + 1) begin
            if ((kidx / NL) == 0) begin
                if (n_ok[kidx] != NREP)   ok_all = ok_all + 1;
            end else begin
                if (n_ok[kidx] != NREP_L) ok_all = ok_all + 1;
            end
        end
        chk(ok_all == 0, "T1b sample counts as expected");
        chk(jit_worst <= JIT_LIM, "T2 same-VL jitter <= 8 cyc");
        chk(proc_worst <= PROC_LIM, "T3 proc <= 40 cyc (320ns)");
        chk(proc_vl_worst <= PROC_LIM, "T4 proc spread <= 40 cyc");
        chk(lat_all_max <= 1518 + PROC_LIM, "T5 max lat <= 1558 cyc");

        $display("");
        $display("  [breakdown] last byte->IC/RM commit: %0d..%0d cyc; commit->delivery: %0d..%0d cyc",
                 d_cm_min, d_cm_max, d_cd_min, d_cd_max);
        chk(d_cm_max <= 16 && d_cd_max <= 8, "T6 breakdown stable");
        $display("  [summary] end-to-end (RX side, inside ES):");
        $display("    latency %0d..%0d cyc (%.2f..%.2f us, incl. whole frame on wire)",
                 lat_all_min, lat_all_max, lat_all_min*8.0/1000.0, lat_all_max*8.0/1000.0);
        $display("    internal processing %0d..%0d cyc = %.0f..%.0f ns; max jitter %0d cyc = %0d ns",
                 v_proc_min[0], proc_worst,
                 v_proc_min[0]*8.0, proc_worst*8.0, jit_worst, jit_worst*8);
        $display("    Note: this is the ES itself; switch (network) store-and-forward is NOT included,");
        $display("          it must be reported separately from R40 SFD->SFD total latency.");

        $display("================================================================");
        if (errs == 0) $display(">> PASS: R40 latency/jitter all within budget");
        else           $display(">> FAIL: %0d item(s) over budget", errs);
        $finish;
    end

    initial begin #40000000; $display(">> TIMEOUT n_deliv=%0d n_ovf=%0d", n_deliv, n_ovf); $finish; end

endmodule
