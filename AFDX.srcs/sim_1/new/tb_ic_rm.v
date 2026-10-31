// tb_ic_rm.v —— IC/RM 单元测试（逐条对照 ARINC 664 P7 §3.2.6.2）
`timescale 1ns/1ps
module tb_ic_rm;
    reg clk=0, reset=1;
    reg [15:0] ts_now=0;
    reg fe=0, f_ok=1, f_net=0; reg [15:0] f_vlid=16'h0123; reg [7:0] f_sn=0;
    reg cfg_hit=1, cfg_ic_en=1, cfg_rm_en=1; reg [11:0] cfg_skew=50;

    reg [10:0] cfg_lmax   = 11'd1471;
    reg [1:0]  cfg_neten  = 2'd0;           // 0 = A+B 都收（与预设表一致）；1=仅A 2=仅B
    reg        cfg_sn_vld = 1'b1;
    reg [10:0] f_paylen   = 11'd8;
    reg        f_frag     = 1'b0;
    reg [31:0] f_dip_ovr  = 32'h0;
    wire [31:0] f_dip = (f_dip_ovr != 32'h0) ? f_dip_ovr
                                             : {8'hE0, 8'hE0, f_vlid[15:8], f_vlid[7:0]};

    wire accept, ev_filt, ev_ic, ev_rm; wire [11:0] out_idx;
    wire [15:0] lk_vlid; wire lk_req;
    reg  [15:0] lk_res_vlid;                 // 迷你 afdx_cfg：地址回显（1 拍延迟）
    integer nacc=0, nic=0, nrm=0, nfilt=0, errs=0;
    integer i, b_a, b_i, b_r, b_f;

    afdx_ic_rm #(.NVL(4096), .IDXW(12)) u_dut (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .frame_end(fe), .f_ok(f_ok), .f_vlid(f_vlid), .f_sn(f_sn), .f_net(f_net),
        .lk_vlid_o(lk_vlid), .lk_req_o(lk_req), .lk_res_vlid_i(lk_res_vlid),
        .lk_hit_i(cfg_hit), .lk_ic_en_i(cfg_ic_en), .lk_rm_en_i(cfg_rm_en), .lk_skew_i(cfg_skew),
        .lk_lmax_i(cfg_lmax), .f_dip(f_dip), .f_paylen(f_paylen), .f_frag(f_frag),

        .lk_neten_i(cfg_neten),
        .lk_sn_vld_i(cfg_sn_vld),
        .accept(accept), .ev_filt_drop(ev_filt), .ev_ic_drop(ev_ic), .ev_rm_drop(ev_rm),
        .out_idx(out_idx)
    );

    // ---- 迷你 afdx_cfg：1 拍寄存读 + 地址回显 ----
    always @(posedge clk) lk_res_vlid <= lk_vlid;

    always #50 clk = ~clk;

    always @(posedge clk) begin
        if (accept)  nacc <= nacc+1;
        if (ev_ic)   nic  <= nic+1;
        if (ev_rm)   nrm  <= nrm+1;
        if (ev_filt) nfilt<= nfilt+1;
    end

    task send;
        input [7:0] sn; input net;
        begin
            @(posedge clk);
            f_sn <= sn; f_net <= net; fe <= 1'b1;
            @(posedge clk);
            fe <= 1'b0;
            repeat(8) @(posedge clk);
        end
    endtask

    task check;
        input cond;
        input [255:0] name;
        begin
            if (cond) $display("  [ok]   %0s", name);
            else begin
                errs = errs + 1;
                $display("  [FAIL] %0s  acc=%0d ic=%0d rm=%0d filt=%0d",
                         name, nacc-b_a, nic-b_i, nrm-b_r, nfilt-b_f);
            end
        end
    endtask

    task expect;
        input integer ea, eic, erm, efl;
        input [255:0] name;
        begin
            if (nacc!==ea || nic!==eic || nrm!==erm || nfilt!==efl) begin
                errs = errs + 1;
                $display("  [FAIL] %0s  acc=%0d ic=%0d rm=%0d filt=%0d", name, nacc, nic, nrm, nfilt);
            end else
                $display("  [ok]   %0s  acc=%0d ic=%0d rm=%0d filt=%0d", name, nacc, nic, nrm, nfilt);
        end
    endtask

    initial begin
        #200 reset = 0;
        #200;
        $display("");
        $display("======== IC/RM unit test ========");
        send(8'd0,  1'b0); expect(1,0,0,0, "01 A sn=0  accept (RSN=0)");
        send(8'd1,  1'b0); expect(2,0,0,0, "02 A sn=1  accept (window)");
        send(8'd1,  1'b1); expect(2,0,1,0, "03 B sn=1  RM dup drop (First Win)");
        send(8'd2,  1'b0); expect(3,0,1,0, "04 A sn=2  accept");
        send(8'd9,  1'b0); expect(3,1,1,0, "05 A sn=9  IC drop (window {3,4})");
        send(8'd10, 1'b0); expect(4,1,1,0, "06 A sn=10 accept (PSN updated by IC drop)");
        send(8'd5,  1'b0); expect(4,2,1,0, "07 A sn=5  IC drop (window {11,12})");
        cfg_ic_en <= 1'b0; repeat(4) @(posedge clk);
        send(8'd11, 1'b0); expect(5,2,1,0, "08 IC off: A sn=11 accept (RM new)");
        send(8'd11, 1'b0); expect(5,2,2,0, "09 same SN   RM drop (dup, no skew)");
        ts_now <= 16'd200; repeat(4) @(posedge clk);
        send(8'd11, 1'b0); expect(6,2,2,0, "10 skew exceeded: same SN accept (resync)");
        // --- 未配置 VL / 无效帧 ---
        cfg_hit <= 1'b0; repeat(4) @(posedge clk);
        send(8'd12, 1'b0); expect(6,2,2,1, "11 unconfigured VL -> filt drop");
        cfg_hit <= 1'b1; f_ok <= 1'b0; repeat(4) @(posedge clk);
        send(8'd12, 1'b0); expect(6,2,2,2, "12 bad frame (ok=0) -> filt drop");
        f_ok <= 1'b1; repeat(4) @(posedge clk);
        // --- 乱序旧帧 ---
        ts_now <= 16'd300; repeat(4) @(posedge clk);
        send(8'd50, 1'b0); expect(7,2,2,2, "13 A sn=50 accept (new)");
        send(8'd20, 1'b0); expect(7,2,3,2, "14 A sn=20 RM drop (out of order)");

        cfg_ic_en <= 1'b1; ts_now <= 16'd400; repeat(4) @(posedge clk);
        send(8'd21, 1'b0); expect(8,2,3,2, "15 skew-resync: A sn=21 accept (RM 相位对齐)");
        b_a = nacc; b_i = nic; b_r = nrm; b_f = nfilt;
        for (i = 22; i <= 255; i = i + 1) send(i[7:0], 1'b0);   // 234 帧
        send(8'd1,  1'b0);                                      // 回绕点 255 → 1
        send(8'd2,  1'b0);
        send(8'd100,1'b0);                                      // 窗口 {3,4} → IC 丢弃
        check((nacc-b_a)==236 && (nic-b_i)==1 && (nrm-b_r)==0 && (nfilt-b_f)==0,
              "16 SN 21→255→1→2 全接受(236) + sn=100 IC丢弃");


        cfg_lmax <= 11'd16;  f_paylen <= 11'd100;      // 载荷 100 > Lmax 16
        ts_now <= 16'd900; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd5, 1'b0);
        check((nfilt-b_f)==1, "17 载荷 > Lmax → 被过滤（不交付给分区）");
        cfg_lmax <= 11'd1471; f_paylen <= 11'd8;       // 恢复
        ts_now <= 16'd1000; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd6, 1'b0);
        check((nfilt-b_f)==0, "18 Lmax 恢复后同一条件不再被过滤（对照）");


        cfg_lmax <= 11'd1471; f_paylen <= 11'd1472; f_frag <= 1'b1;
        ts_now <= 16'd1050; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd7, 1'b0);
        check((nfilt-b_f)==0, "17b frag 1472 bytes not filtered");
        f_frag   <= 1'b0;                              // 同样的长度、非分片 ⇒ 必须过滤
        ts_now <= 16'd1060; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd8, 1'b0);
        check((nfilt-b_f)==1, "17c non-frag 1472 still filtered");
        f_frag   <= 1'b0; f_paylen <= 11'd8; cfg_lmax <= 11'd1471;


        cfg_lmax <= 11'd1471; f_paylen <= 11'd8;
        ts_now <= 16'd2000; repeat(4) @(posedge clk);
        send(8'd100, 1'b0);                     // 随便建立一个新的 psn（它是不是 IC 丢弃都不影响下面）
        send(8'd0,   1'b0);                     // SN=0：复位标志例外，IC 必过 → psn 被写成 0
        b_i = nic;
        send(8'd87,  1'b0);                     // 87 远在窗口 {1,2} 之外 → **必须 IC 丢弃**
        check((nic-b_i)==1,
              "19 ★★SN=0 之后，窗口外的 SN 仍被 IC 丢弃（不再被当『首帧』放行）");


        f_dip_ovr <= 32'hE0E09999;                     // 故意与 VLID 0x0123 不符
        ts_now <= 16'd1100; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd7, 1'b0);
        check((nfilt-b_f)==1, "19 目的 IP ≠ 224.224.<VLID> → 被过滤");
        f_dip_ovr <= 32'h0;                            // 恢复自洽（=224.224.0x0123）
        ts_now <= 16'd1200; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd8, 1'b0);
        check((nfilt-b_f)==0, "20 目的 IP 自洽后不再被过滤（对照）");


        cfg_neten <= 2'd1;                              // 仅 A
        ts_now <= 16'd1300; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd9, 1'b1);                               // 从 B 网来 → 应被过滤
        check((nfilt-b_f)==1, "21 net_en=仅A 时，B 网来的帧被过滤");
        ts_now <= 16'd1400; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd10, 1'b0);                              // 从 A 网来 → 放行
        check((nfilt-b_f)==0, "22 net_en=仅A 时，A 网来的帧照常收（对照）");

        cfg_neten <= 2'd2;                              // 仅 B
        ts_now <= 16'd1500; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd11, 1'b0);                              // 从 A 网来 → 应被过滤
        check((nfilt-b_f)==1, "23 net_en=仅B 时，A 网来的帧被过滤");
        ts_now <= 16'd1600; repeat(4) @(posedge clk);
        b_f = nfilt;
        send(8'd12, 1'b1);                              // 从 B 网来 → 放行
        check((nfilt-b_f)==0, "24 net_en=仅B 时，B 网来的帧照常收（对照）");
        cfg_neten <= 2'd0;                              // 恢复 A+B


        f_vlid <= 16'h0456; repeat(4) @(posedge clk);
        ts_now <= 16'd1700; repeat(4) @(posedge clk);
        b_a = nacc; b_i = nic;
        send(8'd10, 1'b0);                              // A 首帧：IC 放行、RM 接受
        send(8'd77, 1'b1);                              // B 首帧、SN 远离 A 的窗口：必须 IC 放行
        check((nacc-b_a)==2 && (nic-b_i)==0,
              "25 ★★A 网已初始化后，B 网首帧仍被放行（per-net ic_seen）");
        f_vlid <= 16'h0123; repeat(4) @(posedge clk);


        ts_now <= 16'd1700; repeat(4) @(posedge clk);
        cfg_sn_vld <= 1'b0;                             // 该 VL 不带 SN
        repeat(4) @(posedge clk);
        b_a = nacc; b_r = nrm;
        send(8'd0, 1'b0);                               // 三帧 SN 全为 0
        send(8'd0, 1'b0);
        send(8'd0, 1'b0);
        check((nacc-b_a)==3 && (nrm-b_r)==0,
              "25 ★★sn_vld=0：连续三帧 SN=0 **全部交付**（旧实现第 2 帧起被判重复丢弃）");

        ts_now <= 16'd1800; repeat(4) @(posedge clk);
        cfg_sn_vld <= 1'b1;                             // 对照：带 SN 的 VL（正常 AFDX）
        repeat(4) @(posedge clk);
        b_a = nacc; b_r = nrm;
        send(8'd0, 1'b0);                               // 同样三帧 SN=0
        send(8'd0, 1'b0);
        send(8'd0, 1'b0);
        check((nacc-b_a)==1 && (nrm-b_r)==2,
              "26 对照：sn_vld=1 时同样的三帧只会交付 1 帧、另 2 帧被 RM 判重复（原有行为不变）");

        $display("=================================");
        if (errs==0) $display(">> PASS: IC 窗口/回绕/RSN=0/首帧/重复/乱序/SkewMax 重同步/未配置 全部符合标准");
        else         $display(">> FAIL: %0d cases mismatch", errs);
        $finish;
    end
endmodule
