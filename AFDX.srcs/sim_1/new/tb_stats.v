// tb_stats.v —— afdx_stats 单元测试
`timescale 1ns/1ps

module tb_stats;
    reg clk = 0, reset = 1;
    always #50 clk = ~clk;          // 10 MHz

    // ---- 事件 ----
    reg         rx_desc, rx_ok;
    reg  [15:0] rx_vlid;
    reg  [10:0] rx_len;
    reg         rx_e_fcs, rx_e_len, rx_e_ip, rx_e_udp;
    reg         ev_ic, ev_rm, ev_filt, ev_deliv, ev_vlmiss;
    reg  [4:0]  rx_ovf_set, rx_deliv_p;
    reg  [2:0]  rx_deliv_pi;
    reg         tx_frame, tx_byte, tx_bag_hold;
    reg  [15:0] tx_vlid;

    // ---- CPU ----
    reg         wr = 0;
    reg  [3:0]  reg_sel = 0;
    reg  [31:0] wdata = 0;
    wire [31:0] rdata;


    reg  inj_en   = 1'b0;
    reg  inj_done = 1'b0;
    wire inj_hit  = inj_en && !inj_done && (dut.st == 2'd3) && (dut.cur_s == 2'd0);
    always @(posedge clk) if (inj_hit) inj_done <= 1'b1;

    afdx_stats #(.NVL(4096), .NVLW(12), .NP(5), .NPW(3)) dut (
        .clk(clk), .reset(reset),
        .rx_desc(rx_desc | inj_hit), .rx_ok(rx_ok), .rx_vlid(rx_vlid), .rx_len(rx_len),
        .rx_e_fcs(rx_e_fcs), .rx_e_len(rx_e_len), .rx_e_ip(rx_e_ip), .rx_e_udp(rx_e_udp),
        .ev_ic(ev_ic), .ev_rm(ev_rm), .ev_filt(ev_filt), .ev_deliv(ev_deliv),
        .ev_vlmiss(ev_vlmiss),
        .rx_ovf_set(rx_ovf_set), .rx_deliv_p(rx_deliv_p),
        .tx_frame(tx_frame), .tx_byte(tx_byte), .tx_bag_hold(tx_bag_hold), .tx_vlid(tx_vlid),
        .wr(wr), .reg_sel(reg_sel), .wdata(wdata), .rdata(rdata)
    );

    // 寄存器编号
    localparam [3:0] R_G_IDX=4'd0, R_G_DATA=4'd1, R_P_IDX=4'd2, R_P_DATA=4'd3,
                     R_V_IDX=4'd4, R_V_LANE=4'd5, R_V_DATA=4'd6, R_BUSY=4'd7,
                     R_LOST =4'd8, R_CLR  =4'd9;

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    reg [31:0] got;

    task wr_reg; input [3:0] rs; input [31:0] d;
        begin @(negedge clk); wr=1; reg_sel=rs; wdata=d; @(posedge clk);
              @(negedge clk); wr=0; reg_sel=0; end
    endtask

    task rd_reg; input [3:0] rs;      // 组合读，当拍采
        begin @(negedge clk); reg_sel=rs; #1 got=rdata; @(posedge clk);
              @(negedge clk); reg_sel=0; end
    endtask

    // 读一个每 VL 计数器：写 VL_IDX → 写 VL_LANE（触发）→ 等 BUSY 落 → 读 VL_DATA
    task rd_vl; input [11:0] vi; input [2:0] lane; input [31:0] exp; input [255:0] n;
        integer w;
        reg     busy_ok;
        begin
            wr_reg(R_V_IDX,  {20'h0, vi});
            wr_reg(R_V_LANE, {29'h0, lane});
            w = 0; busy_ok = 1'b0;
            while (w < 40) begin
                rd_reg(R_BUSY);
                if (got[0] == 1'b0) begin busy_ok = 1'b1; w = 40; end
                else w = w + 1;
            end

            if (!busy_ok) begin
                errs = errs + 1;
                $display("  [FAIL] BUSY 40 拍内没落下就读了 V_DATA（VL 0x%03x lane %0d）—— 该值无效", vi, lane);
            end
            rd_reg(R_V_DATA);
            chk(got === exp, n);
        end
    endtask

    task rd_pulse;   // 打一拍事件
        begin @(negedge clk); end
    endtask

    integer i;
    integer base_f = 0;
    initial begin
        rx_desc=0; rx_ok=0; rx_vlid=0; rx_len=0;
        rx_e_fcs=0; rx_e_len=0; rx_e_ip=0; rx_e_udp=0;
        ev_ic=0; ev_rm=0; ev_filt=0; ev_deliv=0; ev_vlmiss=0;
        rx_ovf_set=0; rx_deliv_p=0; rx_deliv_pi=0;
        tx_frame=0; tx_byte=0; tx_bag_hold=0; tx_vlid=0;

        #200 reset = 1'b0; repeat (4) @(posedge clk);

        $display("");
        $display("========== afdx_stats 单元测试 ==========");


        @(negedge clk);
        rx_desc=1; rx_ok=1; rx_vlid=16'h0123; rx_len=11'd32;
        ev_deliv=1; rx_deliv_p=5'b00001; rx_deliv_pi=3'd0;
        @(posedge clk); @(negedge clk);
        rx_desc=0; rx_ok=0; ev_deliv=0; rx_deliv_p=0;
        repeat (12) @(posedge clk);

        // ②
        @(negedge clk);
        rx_desc=1; rx_ok=1; rx_vlid=16'h0123; rx_len=11'd32; ev_ic=1;
        @(posedge clk); @(negedge clk);
        rx_desc=0; rx_ok=0; ev_ic=0;
        repeat (12) @(posedge clk);

        // ③（FCS 错）
        @(negedge clk);
        rx_desc=1; rx_ok=0; rx_vlid=16'h0123; rx_len=11'd16; rx_e_fcs=1;
        @(posedge clk); @(negedge clk);
        rx_desc=0; rx_e_fcs=0;
        repeat (12) @(posedge clk);

        // ④ VL_0x456 收 1 帧 8 字节，交付到端口 1
        @(negedge clk);
        rx_desc=1; rx_ok=1; rx_vlid=16'h0456; rx_len=11'd8; ev_deliv=1;
        rx_deliv_p=5'b00010; rx_deliv_pi=3'd1;
        @(posedge clk); @(negedge clk);
        rx_desc=0; rx_ok=0; ev_deliv=0; rx_deliv_p=0;
        repeat (12) @(posedge clk);

        // ④b 单独一次"端口禁用过滤"（不对应某帧的交付）
        @(negedge clk); ev_filt=1; @(posedge clk); @(negedge clk); ev_filt=0;
        repeat (12) @(posedge clk);

        // ⑤ 未命中
        @(negedge clk); ev_vlmiss=1; @(posedge clk); @(negedge clk); ev_vlmiss=0;
        repeat (12) @(posedge clk);

        // ⑥ 发帧 + 发字节（VL_A）
        @(negedge clk); tx_frame=1; tx_vlid=16'h0123;
        @(posedge clk); @(negedge clk); tx_frame=0;
        for (i = 0; i < 64; i = i + 1) begin @(negedge clk); tx_byte=1; @(posedge clk); end
        @(negedge clk); tx_byte=0;
        repeat (12) @(posedge clk);

        // ⑦ 端口 0 溢出一次
        @(negedge clk); rx_ovf_set=5'b00001; @(posedge clk); @(negedge clk); rx_ovf_set=0;
        repeat (12) @(posedge clk);

        // ---------------------------------------------------------------
        // T1 全局标量
        // ---------------------------------------------------------------
        wr_reg(R_G_IDX, 0); rd_reg(R_G_DATA); chk(got === 32'd4, "T1a rxFrames = 4");
        wr_reg(R_G_IDX, 1); rd_reg(R_G_DATA); chk(got === 32'd32+32+8, "T1b rxBytes = 合法帧载荷之和 = 72（第 3 帧 FCS 错，不计）");
        wr_reg(R_G_IDX, 2); rd_reg(R_G_DATA); chk(got === 32'd1, "T1c rxErrFcs = 1");
        wr_reg(R_G_IDX, 3); rd_reg(R_G_DATA); chk(got === 32'd0, "T1d rxErrLen = 0");
        wr_reg(R_G_IDX, 4); rd_reg(R_G_DATA); chk(got === 32'd0, "T1e rxErrIp = 0");
        wr_reg(R_G_IDX, 5); rd_reg(R_G_DATA); chk(got === 32'd0, "T1f rxErrUdp = 0");
        wr_reg(R_G_IDX, 6); rd_reg(R_G_DATA); chk(got === 32'd1, "T1g rxVlMiss = 1");
        wr_reg(R_G_IDX, 7); rd_reg(R_G_DATA); chk(got === 32'd1, "T1h rxIcDrop = 1");
        wr_reg(R_G_IDX, 8); rd_reg(R_G_DATA); chk(got === 32'd0, "T1i rxRmDrop = 0");
        wr_reg(R_G_IDX, 9); rd_reg(R_G_DATA); chk(got === 32'd1, "T1j rxFiltDrop = 1");
        wr_reg(R_G_IDX,10); rd_reg(R_G_DATA); chk(got === 32'd2, "T1k rxDelivered = 2（帧①与帧④各交付一条）");
        wr_reg(R_G_IDX,11); rd_reg(R_G_DATA); chk(got === 32'd1, "T1l txFrames = 1");
        wr_reg(R_G_IDX,12); rd_reg(R_G_DATA); chk(got === 32'd64, "T1m txBytes = 64");
        wr_reg(R_G_IDX,13); rd_reg(R_G_DATA); chk(got === 32'd0, "T1n txBagHold = 0（没给过）");
        wr_reg(R_G_IDX,15); rd_reg(R_G_DATA); chk(got === 32'd1, "T1o rxPortOvf = 1");

        // ---------------------------------------------------------------
        // T2 每端口
        // ---------------------------------------------------------------
        wr_reg(R_P_IDX, 0);  rd_reg(R_P_DATA); chk(got === 32'd1, "T2a 端口 0 溢出次数 = 1");
        wr_reg(R_P_IDX, 1);  rd_reg(R_P_DATA); chk(got === 32'd1, "T2b 端口 0 交付条数 = 1");
        wr_reg(R_P_IDX, 2);  rd_reg(R_P_DATA); chk(got === 32'd0, "T2c 端口 1 溢出 = 0");
        wr_reg(R_P_IDX, 3);  rd_reg(R_P_DATA); chk(got === 32'd1, "T2d 端口 1 交付条数 = 1");

        // ---------------------------------------------------------------
        // T3/T4 每 VL（走引擎 + BUSY 握手）
        // ---------------------------------------------------------------
        rd_vl(12'h0123, 3'd0, 32'd3, "T3a VL 0x123 rxFrames = 3");
        rd_vl(12'h0123, 3'd1, 32'd1, "T3b VL 0x123 icDrop = 1");
        rd_vl(12'h0123, 3'd2, 32'd0, "T3c VL 0x123 rmDrop = 0");
        rd_vl(12'h0123, 3'd3, 32'd1, "T3d VL 0x123 delivered = 1");
        rd_vl(12'h0123, 3'd4, 32'd1, "T3e VL 0x123 txFrames = 1");
        rd_vl(12'h0456, 3'd0, 32'd1, "T3f VL 0x456 rxFrames = 1");
        rd_vl(12'h0456, 3'd3, 32'd1, "T3g VL 0x456 delivered = 1（端口 1 那条）");
        rd_vl(12'h0456, 3'd4, 32'd0, "T3h VL 0x456 txFrames = 0");

        // ---------------------------------------------------------------
        // T5 三来源同拍不丢
        // ---------------------------------------------------------------
        @(negedge clk);
        rx_desc=1; rx_ok=1; rx_vlid=16'h0789; rx_len=11'd4; ev_deliv=1;
        tx_frame=1; tx_vlid=16'h0789;
        @(posedge clk); @(negedge clk);
        rx_desc=0; rx_ok=0; ev_deliv=0; tx_frame=0;
        repeat (40) @(posedge clk);
        rd_vl(12'h0789, 3'd0, 32'd1, "T5a 同拍 3 来源：rxFrames 计到 1");
        rd_vl(12'h0789, 3'd3, 32'd1, "T5b 同拍 3 来源：delivered 计到 1");
        rd_vl(12'h0789, 3'd4, 32'd1, "T5c 同拍 3 来源：txFrames 计到 1");

        // ---------------------------------------------------------------
        // T6 LOST 恒 0（自证）
        // ---------------------------------------------------------------
        rd_reg(R_LOST); chk(got === 32'd0, "T6 LOST = 0（从未因队列满丢事件）");
        rd_reg(R_BUSY); chk(got === 32'd0, "T7a 空闲时 BUSY = 0");

        // ---------------------------------------------------------------
        // T7 CLR 的边界
        // ---------------------------------------------------------------
        wr_reg(R_CLR, 32'h1);
        repeat (4) @(posedge clk);
        wr_reg(R_G_IDX, 0); rd_reg(R_G_DATA); chk(got === 32'd0, "T7b CLR 后全局 rxFrames = 0");
        wr_reg(R_P_IDX, 0); rd_reg(R_P_DATA); chk(got === 32'd0, "T7c CLR 后端口计数 = 0");
        rd_vl(12'h0123, 3'd0, 32'd3, "T7d ★CLR **不影响**每 VL 计数器（Counter32 单调递增）");


        wr_reg(R_CLR, 32'h1);
        repeat (4) @(posedge clk);
        for (i = 0; i < 4; i = i + 1) begin      // 4 拍内连打 4 个收帧事件
            @(negedge clk); rx_desc=1; rx_ok=1; rx_vlid=16'h0AAA; rx_len=11'd8;
            @(posedge clk);
        end
        @(negedge clk); rx_desc=0; rx_ok=0;
        repeat (40) @(posedge clk);
        rd_reg(R_LOST); chk(got === 32'd1, "T8a ★来源挤爆时会置 LOST（自证位有效，不是静默丢弃）");
        rd_vl(12'h0AAA, 3'd0, 32'd1, "T8b 被挤爆的那几条里，第一条仍然正确计到 1");


        wr_reg(R_CLR, 32'h1);
        repeat (6) @(posedge clk);
        rd_reg(R_LOST); chk(got === 32'd0, "T9a 起手：LOST 已清（对照）");

        base_f = 0; inj_en = 1'b1;              // 打开组合注入
        @(negedge clk); rx_desc=1; rx_ok=1; rx_vlid=16'h0BBB; rx_len=11'd8;
        @(posedge clk); @(negedge clk); rx_desc=0; rx_ok=0;
        repeat (80) @(posedge clk);              // 等两帧都服务完
        chk(inj_done === 1'b1, "T9b 注入确实打出去了（S_DONE 拍命中 ⇒ 用例非空）");
        rd_vl(12'h0BBB, 3'd0, 32'd2, "T9c ★★清/设同拍不丢事件：该 VL 的收帧计数 = 2");
        rd_reg(R_LOST); chk(got === 32'd0, "T9d ★★这条路径不该置 LOST（原实现是静默丢的）");


        @(negedge clk); rx_desc=1; rx_ok=1; rx_vlid=16'h1BC5; rx_len=11'd8;
        @(posedge clk); @(negedge clk); rx_desc=0; rx_ok=0;
        repeat (30) @(posedge clk);
        rd_vl(12'h0BC5, 3'd0, 32'd0, "T4 VLID>=0x1000 no alias");

        $display("==========================================");
        if (errs == 0) $display(">> PASS: 统计计数器（全局/每端口/每 VL 引擎/BUSY 握手/突发自证/清设同拍）全部符合");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #2000000; $display(">> TIMEOUT"); $finish; end
endmodule
