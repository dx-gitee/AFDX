// tb_ts_capture.v —— afdx_ts_capture 单元测试（纯 RTL，不依赖 MAC 网表）
`timescale 1ns/1ps

module tb_ts_capture;

    reg clk = 0, reset = 1;
    always #40 clk = ~clk;                       // 12.5 MHz（1 拍 = 80 ns）

    // 时间基准：每拍 +1（模拟 80 ns 自由运行计数）
    reg [31:0] ts_now = 32'd1000;
    always @(posedge clk) ts_now <= ts_now + 32'd1;

    reg  [1:0]   rx_dv = 2'b00, tx_en = 2'b00;
    reg  [15:0]  rx_d  = 16'h0000, tx_d = 16'h0000;
    wire [63:0]  rx_ts, tx_ts;
    wire [15:0]  rx_seq, tx_seq;

    afdx_ts_capture #(.NETS(2)) dut (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .rx_dv(rx_dv), .rx_d(rx_d), .tx_en(tx_en), .tx_d(tx_d),
        .rx_ts(rx_ts), .rx_seq(rx_seq), .tx_ts(tx_ts), .tx_seq(tx_seq)
    );

    integer errs = 0, nchk = 0;
    task chk;
        input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    // 在 negedge 采样（组合输出已稳定）
    reg [31:0] ts_at_d5 [0:3];                   // 每次 D5 那一拍的 ts_now
    integer    nd5 = 0;
    reg [31:0] ts_d5_cur;

    // 送一帧：7×0x55 + D5 + 若干个数据字节（net 选择 0/1）
    task send_byte;
        input [1:0] net; input is_tx; input [7:0] b;
        begin
            @(negedge clk);
            if (is_tx) begin tx_en[net] = 1'b1; tx_d[8*net +: 8] = b; end
            else       begin rx_dv[net] = 1'b1; rx_d[8*net +: 8] = b; end
            if (b == 8'hD5) begin ts_at_d5[nd5] = ts_now; nd5 = nd5 + 1; end
            @(posedge clk);
        end
    endtask

    task end_frame;
        input [1:0] net; input is_tx;
        begin
            @(negedge clk);
            if (is_tx) begin tx_en[net] = 1'b0; tx_d[8*net +: 8] = 8'h00; end
            else       begin rx_dv[net] = 1'b0; rx_d[8*net +: 8] = 8'h00; end
            @(posedge clk); @(negedge clk);
        end
    endtask

    integer k;
    reg [31:0] s0, t0;

    initial begin
        #200; reset = 1'b0;
        repeat (4) @(posedge clk);
        $display("");
        $display("======== afdx_ts_capture 单元测试 ========");

        // ---------- T1：前导码 0x55 不触发 ----------
        s0 = tx_seq;
        for (k = 0; k < 7; k = k + 1) send_byte(2'd0, 1'b1, 8'h55);
        chk(tx_seq[7:0] === 8'd0, "T1 前导码 0x55 不触发（只有 0xD5 才算帧首）");

        // ---------- T2：SFD 触发，且时间戳 = D5 那一拍 ----------
        send_byte(2'd0, 1'b1, 8'hD5);
        @(negedge clk);
        chk(tx_seq[7:0] === 8'd1, "T2a ★SFD 触发一次（A 网 tx seq = 1）");
        chk(tx_ts[31:0] === ts_at_d5[nd5-1], "T2b ★★时间戳 = SFD(0xD5) 那一拍的 ts_now（逐拍精确）");
        chk(rx_seq[7:0] === 8'd0, "T2c 发送不触发接收（tx/rx 独立）");
        chk(rx_seq[15:8] === 8'd0 && tx_seq[15:8] === 8'd0, "T2d B 网完全没动（A/B 独立）");

        // ---------- T3：连续 2 拍 D5 只记 1 次（去重） ----------
        send_byte(2'd0, 1'b1, 8'hD5);            // 紧接着再来一拍 D5
        @(negedge clk);
        chk(tx_seq[7:0] === 8'd1, "T3 ★同一个 SFD 连续 2 拍只记 1 次（去重）");
        for (k = 0; k < 3; k = k + 1) send_byte(2'd0, 1'b1, 8'hA0 + k[7:0]);
        end_frame(2'd0, 1'b1);

        // ---------- T4：第二帧 → seq=2，ts 跟到第二帧 ----------
        for (k = 0; k < 7; k = k + 1) send_byte(2'd0, 1'b1, 8'h55);
        send_byte(2'd0, 1'b1, 8'hD5);
        send_byte(2'd0, 1'b1, 8'h11);
        end_frame(2'd0, 1'b1);
        @(negedge clk);
        chk(tx_seq[7:0] === 8'd2, "T4a 第二帧 → tx seq = 2");
        chk(tx_ts[31:0] === ts_at_d5[nd5-1], "T4b ★时间戳跟到第二帧的 SFD");

        // ---------- T5：B 网接收：独立计时 ----------
        for (k = 0; k < 7; k = k + 1) send_byte(2'd1, 1'b0, 8'h55);
        send_byte(2'd1, 1'b0, 8'hD5);
        send_byte(2'd1, 1'b0, 8'h22);
        end_frame(2'd1, 1'b0);
        @(negedge clk);
        chk(rx_seq[15:8] === 8'd1, "T5a ★B 网接收：rx seq = 1（A/B 各记各的）");
        chk(rx_ts[63:32] === ts_at_d5[nd5-1], "T5b ★B 网时间戳 = 它自己的 SFD 那拍");
        chk(tx_seq[15:8] === 8'd0, "T5c B 网发送仍未动（方向独立）");
        chk(rx_seq[7:0] === 8'd0, "T5d A 网接收仍未动（网络独立）");

        // ---------- T6：复位清空 ----------
        @(negedge clk); reset = 1'b1;
        @(posedge clk); @(negedge clk); reset = 1'b0;
        @(posedge clk); @(negedge clk);
        chk(rx_seq === 16'h0000 && tx_seq === 16'h0000, "T6a ★复位后 seq 全 0");
        chk(rx_ts === 64'h0 && tx_ts === 64'h0, "T6b ★复位后 ts 全 0");

        $display("==========================================");
        if (errs == 0) $display(">> PASS: SFD 硬件时间戳（逐拍精确/前导不触发/去重/双网独立/复位）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #200000;
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
