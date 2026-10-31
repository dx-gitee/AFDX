// tb_app_side.v —— 应用侧接口端到端
`timescale 1ns/1ps

module tb_app_side;
    reg clk = 0, reset = 1;
    always #4 clk = ~clk;                  // 125 MHz

    // ---- 应用侧 ----
    reg        a_v = 0; reg [7:0] a_d = 0; reg [3:0] a_p = 4'd1;
    wire       t_v; wire [7:0] t_d; wire [3:0] t_p; wire err;

    // ---- SoC 帧流 + 回环 ----
    wire        fr_v, fr_sop, fr_eop, fr_net;
    wire [7:0]  fr_d;
    reg         fr_ready = 1;
    reg         rx_v = 0, rx_sop = 0, rx_eop = 0, rx_net = 0;
    reg  [7:0]  rx_d = 0;
    always @(posedge clk) begin
        rx_v   <= fr_v;
        rx_sop <= fr_sop;
        rx_eop <= fr_eop;
        rx_d   <= fr_d;
        rx_net <= fr_net;
    end

    wire cfg_hit; wire [127:0] cfg_word;
    wire fl_sclk, fl_mosi, fl_cs_n, h_irq;

    darkriscv_soc_top u_dut (
        .clk(clk), .reset(reset), .debug(),
        .dp_rxin_valid(rx_v), .dp_rxin_sop(rx_sop), .dp_rxin_eop(rx_eop),
        .dp_rxin_data(rx_d),  .dp_rxin_net(rx_net),
        .dp_fr_valid(fr_v), .dp_fr_ready(fr_ready), .dp_fr_sop(fr_sop),
        .dp_fr_eop(fr_eop), .dp_fr_data(fr_d), .dp_fr_net(fr_net),
        .app_rx_valid(a_v), .app_rx_data(a_d), .app_port(a_p),
        .app_tx_valid(t_v), .app_tx_data(t_d), .app_tx_port(t_p), .app_err(err),
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

    // ---- 抓线上帧（只看 A 副本）----
    reg [7:0] fbuf [0:2047];
    integer   fc = 0, fcnt = 0;
    reg [47:0] g_da;
    always @(posedge clk) begin
        if (fr_v && fr_ready && !fr_net) begin
            if (fr_sop) begin fc = 0; fcnt = fcnt + 1; end
            fbuf[fc] = fr_d; fc = fc + 1;
            if (fc == 7) g_da = {fbuf[0],fbuf[1],fbuf[2],fbuf[3],fbuf[4],fbuf[5]};
        end
    end

    // ---- 抓应用侧收到的字节 ----
    reg [7:0] abuf [0:2047];
    integer   ac = 0, acnt = 0;
    reg [3:0] g_tp;
    always @(posedge clk) begin
        if (t_v) begin
            if (ac == 0 || acnt == 0) g_tp = t_p;
            abuf[ac] = t_d; ac = ac + 1;
            acnt = acnt + 1;
            if (acnt > 0 && t_d === 8'h4F) begin end
        end
    end

    // 调试：打印**被 tx_path 接受**的字节序列（看入向流有没有重复）
    always @(posedge clk)
        if (u_dut.u_msgmux.msg_valid && u_dut.u_msgmux.msg_ready)
            $display("  [acc] data=%02x sop=%b eop=%b vlid=%04x len=%0d",
                     u_dut.u_msgmux.msg_data, u_dut.u_msgmux.msg_sop, u_dut.u_msgmux.msg_eop,
                     u_dut.u_msgmux.msg_vlid, u_dut.u_msgmux.msg_len);

    task push_msg; input integer n; input [7:0] b0; input [3:0] port;
        integer k;
        begin
            a_p = port;
            for (k = 0; k < n; k = k + 1) begin
                @(negedge clk); a_v = 1; a_d = b0 + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); a_v = 0;
        end
    endtask

    integer errs = 0, w, k, n0;
    reg [7:0] hello [0:4];
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    initial begin
        #200 reset = 0;
        #4000;                              // 等配置表/固件稳定（配置表 t=0 就绪）
        $display("");
        $display("======== 应用侧接口端到端（应用消息 → 帧 → 回环 → 交付）========");

        // ---- 推一条真正的 "HELLO"（48 45 4C 4C 4F）到端口 1 ----
        hello[0]=8'h48; hello[1]=8'h45; hello[2]=8'h4C; hello[3]=8'h4C; hello[4]=8'h4F;
        a_p = 4'd1;
        for (k = 0; k < 5; k = k + 1) begin
            @(negedge clk); a_v = 1; a_d = hello[k];
            @(posedge clk);
        end
        @(negedge clk); a_v = 0;

        // 等线上出现 A 副本帧
        w = 0;
        while (fcnt == 0 && w < 200000) begin @(posedge clk); w = w + 1; end
        repeat (200) @(posedge clk);
        $display("  [dbg] 帧数=%0d DA=%012x 帧长=%0d", fcnt, g_da, fc);
        $display("  [dbg] 帧前 60 字节: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 fbuf[0],fbuf[1],fbuf[2],fbuf[3],fbuf[4],fbuf[5],fbuf[6],fbuf[7],fbuf[8],fbuf[9],
                 fbuf[10],fbuf[11],fbuf[12],fbuf[13],fbuf[14],fbuf[15],fbuf[16],fbuf[17],fbuf[18],fbuf[19]);
        $display("  [dbg] 帧 40..59: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                 fbuf[40],fbuf[41],fbuf[42],fbuf[43],fbuf[44],fbuf[45],fbuf[46],fbuf[47],fbuf[48],fbuf[49],
                 fbuf[50],fbuf[51],fbuf[52],fbuf[53],fbuf[54],fbuf[55],fbuf[56],fbuf[57],fbuf[58],fbuf[59]);
        chk(fcnt >= 1, "T1 线上出现 A 副本帧");
        chk(g_da === 48'h030000000456, "T2 DA = 03:00:00:00:04:56（端口 1 → VL 0x0456）");
        chk(fbuf[42] === 8'h48 && fbuf[43] === 8'h45 && fbuf[44] === 8'h4C &&
            fbuf[45] === 8'h4C && fbuf[46] === 8'h4F, "T3 帧载荷 = HELLO（字节 42..46）");

        // 等应用侧收到（回环 → 解析 → IC/RM → 桥）
        w = 0;
        while (acnt < 5 && w < 400000) begin @(posedge clk); w = w + 1; end
        $display("  [dbg] 应用收到 %0d 字节: %02x %02x %02x %02x %02x %02x，tx_port=%0d，err=%b",
                 acnt, abuf[0],abuf[1],abuf[2],abuf[3],abuf[4],abuf[5], g_tp, err);
        chk(acnt >= 5, "T4 应用侧收到 ≥5 字节");
        chk(abuf[0] === 8'h48 && abuf[1] === 8'h45 && abuf[4] === 8'h4F, "T5 收到的字节与发出的逐一致（HELLO）");
        chk(g_tp === 4'd1, "T6 app_tx_port = 1（交付来自端口 1）");
        chk(err === 1'b0, "T7 正常路径没有置 err");


        force u_dut.tx_dip   = 32'hC0A80105;
        force u_dut.tx_dport = 16'd40000;
        n0 = fcnt;
        a_p = 4'd2;                        // 端口 2 → VL 0x0789（BAG=1 ms，且本用例里还没用过）
        for (k = 0; k < 3; k = k + 1) begin
            @(negedge clk); a_v = 1; a_d = 8'h58 + k[7:0];      // X/Y/Z
            @(posedge clk);
        end
        @(negedge clk); a_v = 0;
        w = 0;
        while (fcnt == n0 && w < 200000) begin @(posedge clk); w = w + 1; end
        repeat (200) @(posedge clk);
        $display("  [dbg] T8 dip=%02x%02x%02x%02x dport=%02x%02x",
                 fbuf[30],fbuf[31],fbuf[32],fbuf[33],fbuf[36],fbuf[37]);
        chk(fbuf[30]===8'hE0 && fbuf[31]===8'hE0 && fbuf[32]===8'h07 && fbuf[33]===8'h89,
            "T8 app msg keeps cfg dst IP");
        chk(fbuf[36]===8'h00 && fbuf[37]===8'h45, "T9 app msg keeps cfg dst port");
        release u_dut.tx_dip;
        release u_dut.tx_dport;

        $display("==============================================================");
        if (errs == 0) $display(">> PASS: 应用侧端到端（消息→帧→回环→交付）成立");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #8000000; $display(">> TIMEOUT fcnt=%0d acnt=%0d", fcnt, acnt); $finish; end
endmodule
