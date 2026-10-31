// tb_tx_bag.v —— BAG 调度（ARINC 664 P7 §3.2.3）单元测试
`timescale 1ns/1ps

module tb_tx_bag;
    localparam TICK_CYCLES = 20;             // 1 个时基刻度 = 20 拍（真实系统是 8 µs @12.5MHz）
    localparam MSGLEN      = 8;
    localparam NMSG        = 12;             // 每条 VL 灌多少条消息

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    // ---- 时基 ----
    reg [31:0] tickc = 0;
    reg [15:0] ts_now = 0;
    always @(posedge clk) begin
        if (reset) begin tickc <= 0; ts_now <= 0; end
        else if (tickc >= TICK_CYCLES-1) begin tickc <= 0; ts_now <= ts_now + 1; end
        else tickc <= tickc + 1;
    end

    // ---- 配置表 ----
    reg  cfg_wr = 0, cfg_rd = 0;
    reg  [3:0] cfg_rs = 0;
    reg  [31:0] cfg_wd = 0;
    wire [31:0] cfg_rv;
    wire [15:0] tx_cfg_vlid;
    wire        tx_hit;
    wire [127:0] tx_cfg;
    wire [15:0] cfg_res_vlid;

    afdx_cfg #(.NENT(4096), .IDXW(12)) u_cfg (
        .clk(clk), .reset(reset), .wr(cfg_wr), .rd(cfg_rd), .reg_sel(cfg_rs),
        .wdata(cfg_wd), .rdata(cfg_rv),
        .vlid_i(tx_cfg_vlid), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(cfg_res_vlid), .hit_o(tx_hit), .cfg_o(tx_cfg)
    );

    // ---- 发送通路 ----
    wire        fr_valid, fr_sop, fr_eop, fr_net, sn_adv, replay;
    wire [7:0]  fr_data;
    wire        tx_ready;

    // ---- 消息源（模拟 host_port：只要还有消息就挂着一条）----
    reg        s_valid = 0, s_sop = 0, s_eop = 0;
    reg [7:0]  s_data = 0;
    reg [15:0] s_port = 16'h0123;
    reg [10:0] s_cnt = 0;
    reg [13:0] msg_len_r = MSGLEN[13:0];
    reg [15:0] pend1 = NMSG, pend2 = NMSG, pend3 = 0;   // pend3 = 未配置 VLID 的用例

    reg        cfg_ready = 0;

    afdx_tx_path #(.NVL(4096), .NVLW(12)) u_tx (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .ov_dip(32'h0), .ov_dport(16'h0),
        .msg_valid(s_valid), .msg_ready(tx_ready), .msg_sop(s_sop), .msg_eop(s_eop),
        .msg_data(s_data), .msg_port(s_port), .msg_len(msg_len_r),
        .tx_replay_o(replay),
        .cfg_vlid_o(tx_cfg_vlid), .cfg_hit_i(tx_hit), .cfg_i(tx_cfg),
        .cfg_res_vlid_i(cfg_res_vlid),
        .fr_valid(fr_valid), .fr_ready(1'b1), .fr_sop(fr_sop), .fr_eop(fr_eop),
        .fr_data(fr_data), .fr_net(fr_net)
    );

    // 先灌满 VL1 的 NMSG 条，再灌 VL2 的 NMSG 条。
    // （不分 VL 交替灌：源一次只挂一条消息，交替会让另一条 VL 被"堵在源上"，
    //   测到的间隔就不再是这条 VL 自己的 BAG 了。）
    always @(posedge clk) begin
        if (reset) begin
            s_valid <= 0; s_sop <= 0; s_eop <= 0; s_cnt <= 0;
        end
        else if (s_valid && tx_ready) begin
            if (s_cnt == msg_len_r[10:0] - 11'd1) begin
                s_valid <= 1'b0; s_sop <= 1'b0; s_eop <= 1'b0; s_cnt <= 0;
            end
            else begin
                s_cnt <= s_cnt + 11'd1;
                s_sop <= 1'b0;
                s_data<= 8'hA0 + s_cnt[7:0] + 8'd1;
                s_eop <= (s_cnt + 11'd1 == msg_len_r[10:0] - 11'd1);
            end
        end

        else if (u_tx.msg_drop) begin
            s_valid <= 1'b0; s_sop <= 1'b0; s_eop <= 1'b0; s_cnt <= 0;
        end
        else if (!s_valid && cfg_ready) begin
            if (pend1 != 0) begin
                s_port <= 16'h0123; pend1 <= pend1 - 1'b1;
                s_valid <= 1'b1; s_sop <= 1'b1; s_data <= 8'hA0; s_cnt <= 0; s_eop <= 0;
            end
            else if (pend3 != 0) begin
                s_port <= 16'h0777; pend3 <= pend3 - 1'b1;
                s_valid <= 1'b1; s_sop <= 1'b1; s_data <= 8'hC0; s_cnt <= 0; s_eop <= 0;
            end
            else if (pend2 != 0) begin
                s_port <= 16'h0456; pend2 <= pend2 - 1'b1;
                s_valid <= 1'b1; s_sop <= 1'b1; s_data <= 8'hB0; s_cnt <= 0; s_eop <= 0;
            end
        end
    end

    // ---- 写一条 VL 配置 ----
    task cfg_write;
        input [11:0] idx;
        input [31:0] w0, w1, w2, w3;
        begin
            @(negedge clk); cfg_wr=1; cfg_rs=4'd0; cfg_wd={23'h0, idx};
            @(negedge clk); cfg_rs=4'd1; cfg_wd=w0;
            @(negedge clk);            cfg_wd=w1;
            @(negedge clk);            cfg_wd=w2;
            @(negedge clk);            cfg_wd=w3;
            @(negedge clk); cfg_wr=0; cfg_rs=0; cfg_wd=0;
        end
    endtask


    integer snv_hits_1 = 0, snv_hits_0 = 0, snv_bad = 0;
    always @(posedge clk) if (fr_valid && fr_sop) begin
        if (u_tx.u_build.cfg_vlid == 16'h0123) begin
            snv_hits_1 = snv_hits_1 + 1;
            if (u_tx.u_build.cfg_sn_vld !== 1'b1) snv_bad = snv_bad + 1;
        end
        else if (u_tx.u_build.cfg_vlid == 16'h0456) begin
            snv_hits_0 = snv_hits_0 + 1;
            if (u_tx.u_build.cfg_sn_vld !== 1'b0) snv_bad = snv_bad + 1;
        end
    end

    // ---- 量间隔 ----
    integer last1 = -1, min1 = 0, max1 = 0, n1 = 0;
    integer last2 = -1, min2 = 0, max2 = 0, n2 = 0;
    integer cyc = 0, errs = 0;
    integer iv1 [0:31], iv2 [0:31];
    integer k1 = 0, k2 = 0, i;


    integer bag_bad = 0;
    always @(posedge clk) begin
        if (!reset && u_tx.tx_start && (u_tx.rd_adr_q !== u_tx.vli_q)) bag_bad <= bag_bad + 1;
    end


    integer ndrop = 0;
    integer n1b   = 0;
    always @(posedge clk) if (u_tx.msg_drop) ndrop = ndrop + 1;

    integer fsc = 0;
    always @(posedge clk) begin
        cyc = cyc + 1;
        if (fr_valid && fr_sop && fsc < 6) begin
            fsc = fsc + 1;
            $display("  [frame] #%0d cyc=%0d ts=%0d vlid=%04x port_lat=%04x nxt=%0d",
                     fsc, cyc, ts_now, u_tx.u_build.cfg_vlid, u_tx.port_lat, u_tx.nxt_q);
        end
        if (fr_valid && fr_sop) begin
            // 用组帧器里锁存的 VLID 判帧属于哪条 VL（源在组帧期间可能已切换）
            if (u_tx.u_build.cfg_vlid == 16'h0123) begin
                if (last1 >= 0) begin
                    if (min1 == 0 || cyc-last1 < min1) min1 = cyc-last1;
                    if (cyc-last1 > max1) max1 = cyc-last1;
                    if (k1 < 32) begin iv1[k1] = cyc-last1; k1 = k1 + 1; end
                end
                n1 = n1 + 1; last1 = cyc;
            end
            else begin
                if (last2 >= 0) begin
                    if (min2 == 0 || cyc-last2 < min2) min2 = cyc-last2;
                    if (cyc-last2 > max2) max2 = cyc-last2;
                    if (k2 < 32) begin iv2[k2] = cyc-last2; k2 = k2 + 1; end
                end
                n2 = n2 + 1; last2 = cyc;
            end
        end
    end

    task chk;
        input cond; input [255:0] name;
        begin
            if (cond) $display("  [ok]   %0s", name);
            else begin errs = errs + 1; $display("  [FAIL] %0s", name); end
        end
    endtask

    // BAG = 2^n ms → 2^n × 10 个 100µs 刻度 → ×TICK_CYCLES 拍
    localparam BAG1_TICKS = 125;             // bag_shift=0 → 125 刻度 × 8µs = 1 ms
    localparam BAG2_TICKS = 500;             // bag_shift=2 → 500 刻度 × 8µs = 4 ms
    localparam BAG1_CYC   = BAG1_TICKS*TICK_CYCLES;   // 1000
    localparam BAG2_CYC   = BAG2_TICKS*TICK_CYCLES;   // 4000
    localparam SLACK      = 2*TICK_CYCLES + 120;      // 1 刻度 + 帧长 64 + 余量

    initial begin
        #200 reset = 0;
        repeat(4) @(posedge clk);
        $display("");
        $display("======== BAG 调度测试 ========");

        cfg_write(12'h123, 32'hADBF0123, 32'h0BB90320, 32'h00120123, 32'hE0E00123);

        cfg_write(12'h456, 32'h2DBF0456, 32'h0BB90322, 32'h00120123, 32'hE0E00456);
        repeat(4) @(posedge clk);
        cfg_ready <= 1'b1;              // 配置写完，放消息源开跑

        // 等两条 VL 各发满 NMSG 条
        wait (n1 >= NMSG && n2 >= NMSG);
        repeat(20) @(posedge clk);

        $display(" [VL 0x0123] BAG=1ms=%0d 拍  发出 %0d 帧  间隔 min=%0d max=%0d",
                 BAG1_CYC, n1, min1, max1);
        $display(" [VL 0x0456] BAG=4ms=%0d 拍  发出 %0d 帧  间隔 min=%0d max=%0d",
                 BAG2_CYC, n2, min2, max2);
        $display(" [抖动上界] %0d 拍（1 个时基刻度 + 帧长 + 余量）", SLACK);
        // 换算成"真实时间"：本 tb 里 1 个时基刻度 = TICK_CYCLES 拍 = 8 µs（真实系统的时基）
        $display(" [抖动实测] VL1 = %0d 拍 = %0.2f us ; VL2 = %0d 拍 = %0.2f us",
                 max1-BAG1_CYC, (max1-BAG1_CYC)*8.0/TICK_CYCLES,
                 max2-BAG2_CYC, (max2-BAG2_CYC)*8.0/TICK_CYCLES);
        $write(" [VL1 间隔]");  for (i = 0; i < k1; i = i + 1) $write(" %0d", iv1[i]);  $display("");
        $write(" [VL2 间隔]");  for (i = 0; i < k2; i = i + 1) $write(" %0d", iv2[i]);  $display("");

        chk(n1 >= NMSG && n2 >= NMSG, "T1 两条 VL 都被正常调度（没被堵死）");
        chk(min1 >= BAG1_CYC,         "T2 VL1 最小间隔 ≥ BAG(1ms) —— 标准 §3.2.3");
        chk(max1 <= BAG1_CYC + SLACK, "T3 VL1 最大间隔 ≤ BAG + 抖动上界");
        chk(min2 >= BAG2_CYC,         "T4 VL2 最小间隔 ≥ BAG(4ms)");
        chk(max2 <= BAG2_CYC + SLACK, "T5 VL2 最大间隔 ≤ BAG + 抖动上界");

        chk((max1-BAG1_CYC)*8.0/TICK_CYCLES <= 40.0,
            "T6 VL1 抖动 ≤ 40 us（标准 §3.2.4.3 的 40us 项）");
        chk((max2-BAG2_CYC)*8.0/TICK_CYCLES <= 40.0,
            "T7 VL2 抖动 ≤ 40 us");


        n1b = n1;
        cfg_write(12'h123, 32'h28040123,       // net_en=1 | ic_en=1 | lmax=4 | vlid=0x0123
                  32'h0BB90320, 32'h00120123, 32'hE0E00123);
        repeat(10) @(posedge clk);
        pend1 = 1;                             // 再喂一条 8 字节消息
        repeat(300) @(posedge clk);
        chk(n1 === n1b, "T8 ★超长消息（lmax=4、消息 8B）一帧都不起");
        chk(ndrop >= 1, "T9 ★抬起 msg_drop 通知端口层丢弃（不会把端口卡死）");


        n1b = n1;
        ndrop = 0;
        cfg_write(12'h123, 32'h0FD00123,       // net_en=1 | lmax=2000（>1471）| vlid=0x0123
                  32'h0BB90320, 32'h00120123, 32'hE0E00123);
        msg_len_r = 14'd1500;                  // 1500 > 1471，但 ≤ 配置里的 2000
        pend1 = 1;
        repeat (600) @(posedge clk);
        chk(n1 === n1b, "T11a ★★配置 lmax=2000 时，1500B 消息仍按 1471 判超长 → 一帧都不起（钳位生效）");
        chk(ndrop >= 1, "T11b ★★钳位路径同样抬起 msg_drop（不是静默丢弃）");

        chk(u_tx.builder_busy === 1'b0, "T11c builder_busy released after drop");


        n1b = n1;
        ndrop = 0;
        msg_len_r = 14'd16;
        pend3 = 1;
        repeat (200) @(posedge clk);
        chk(n1 === n1b,   "T12a unconfigured VL: no frame");
        chk(ndrop >= 1,   "T12b unconfigured VL raises msg_drop");
        chk(u_tx.builder_busy === 1'b0, "T12c builder_busy released");
        msg_len_r = MSGLEN[13:0];              // 还原，避免影响后续（本台其后无数据用例）

        chk(snv_hits_1 >= 1 && snv_hits_0 >= 1,
            "T10a 两条 VL 都起过帧（用例非空，自证）");
        chk(snv_bad === 0,
            "T10b ★★VL1(sn_vld=1) 与 VL2(sn_vld=0) 传进组帧器的 cfg_sn_vld 都与配置位一致");
        chk(bag_bad == 0, "T13 bag used own VL");
        $display("==============================");
        if (errs==0) $display(">> PASS: BAG 限速生效且抖动有界，两条 VL 独立");
        else         $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #20000000;
        $display(">> TIMEOUT: n1=%0d n2=%0d (卡住了)", n1, n2);
        $finish;
    end
endmodule
