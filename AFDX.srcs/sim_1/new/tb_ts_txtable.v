// tb_ts_txtable.v —— per-VL TX 时间戳表单元台（纯 RTL）
`timescale 1ns/1ps

module tb_ts_txtable;
    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    reg  [31:0] ts_now = 0;
    reg  [1:0]  tx_en = 0;
    reg  [15:0] tx_d = 0;
    reg  [12:0] vl_sel = 0;
    wire [31:0] vl_ts;
    wire        tbl_rdy;

    afdx_ts_txtable #(.NETS(2), .VLW(12)) u_dut (
        .clk(clk), .reset(reset), .ts_now(ts_now),
        .gmii_en(tx_en), .gmii_d(tx_d),
        .vl_sel(vl_sel), .vl_ts(vl_ts), .tbl_rdy(tbl_rdy)
    );

    always @(posedge clk) ts_now <= ts_now + 32'd1;

    // 记录每张网 SFD 那拍的时间戳（与 DUT 同一拍、同一个 ts_now 旧值）
    reg [31:0] sfd_a = 0, sfd_b = 0;
    always @(posedge clk) begin
        if (tx_en[0] && tx_d[7:0]   == 8'hD5) sfd_a <= ts_now;
        if (tx_en[1] && tx_d[15:8]  == 8'hD5) sfd_b <= ts_now;
    end

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin nchk = nchk + 1;
              if (c) $display("  [ok]   #%0d %0s", nchk, n);
              else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end end
    endtask

    // 发一帧：前导 7×55 + SFD + DA=03:00:00:00:<vlid> + 2 个数据字节
    task send_frame;
        input integer net;
        input [15:0]  vlid;
        integer k;
        reg [7:0] b;
        begin
            // 前导 + SFD
            for (k = 0; k < 8; k = k + 1) begin
                @(negedge clk);
                if (net == 0) begin tx_en = 2'b01; tx_d[7:0]  = (k == 7) ? 8'hD5 : 8'h55; end
                else          begin tx_en = 2'b10; tx_d[15:8] = (k == 7) ? 8'hD5 : 8'h55; end
                @(posedge clk);
            end
            // DA[0..5]
            for (k = 0; k < 6; k = k + 1) begin
                case (k)
                    0: b = 8'h03;
                    1: b = 8'h00;
                    2: b = 8'h00;
                    3: b = 8'h00;
                    4: b = vlid[15:8];
                    5: b = vlid[7:0];
                endcase
                @(negedge clk);
                if (net == 0) tx_d[7:0]  = b; else tx_d[15:8] = b;
                @(posedge clk);
            end
            // 两个数据字节（让帧长一点；不影响表）
            for (k = 0; k < 2; k = k + 1) begin
                @(negedge clk);
                if (net == 0) tx_d[7:0]  = 8'hA0 + k[7:0]; else tx_d[15:8] = 8'hA0 + k[7:0];
                @(posedge clk);
            end
            @(negedge clk); tx_en = 2'b00; tx_d = 16'h0000;
            @(posedge clk);
        end
    endtask

    reg [31:0] got;
    reg [31:0] ts_bc5_a, ts_789, ts_bc5_b, ts_bc5_a2, ts_bc5_a3;
    task read_vl; input integer net; input [11:0] vlid;
        begin
            @(negedge clk); vl_sel = {net ? 1'b1 : 1'b0, vlid};
            repeat (3) @(posedge clk);
            got = vl_ts;
        end
    endtask

    integer w;

    initial begin
        #200 reset = 0;
        // 等清表完成（8192 拍）
        w = 0;
        while (!tbl_rdy && w < 20000) begin @(posedge clk); w = w + 1; end
        $display("");
        $display("======== per-VL TX 时间戳表单元台 ========");
        chk(tbl_rdy === 1'b1, "T1a 清表完成（tbl_rdy）");
        read_vl(0, 12'h000); chk(got === 32'd0, "T1b 未写过的 VL 读 0");
        // 复位释放后多等几拍，确保清表已结束
        repeat (4) @(posedge clk);

        // T2 A 网 0x0BC5
        send_frame(0, 16'h0BC5);
        ts_bc5_a = sfd_a;
        repeat (3) @(posedge clk);
        read_vl(0, 12'hBC5);
        chk(got === ts_bc5_a, "T2 ★A 网 VL 0x0BC5 的戳 = 该帧 SFD 时间戳");
        // T3 A 网别的 VL 不影响 0x0BC5
        send_frame(0, 16'h0789);
        ts_789 = sfd_a;
        repeat (3) @(posedge clk);
        read_vl(0, 12'h789);
        chk(got === ts_789, "T3a 0x0789 自己的条目被写入");
        read_vl(0, 12'hBC5);
        chk(got === ts_bc5_a, "T3b ★★0x0789 的发送**没有覆盖** 0x0BC5（t3 根治）");
        // T4 B 网同 VL 与 A 分开
        send_frame(1, 16'h0BC5);
        ts_bc5_b = sfd_b;
        repeat (3) @(posedge clk);
        read_vl(1, 12'hBC5);
        chk(got === ts_bc5_b, "T4a B 网 0x0BC5 的戳 = B 帧 SFD");
        read_vl(0, 12'hBC5);
        chk(got === ts_bc5_a, "T4b ★A/B 同 VL 的条目分开存（A 仍是 A 的）");
        // T5 同一 VL 再发一帧 → 更新
        send_frame(0, 16'h0BC5);
        ts_bc5_a2 = sfd_a;
        repeat (3) @(posedge clk);
        read_vl(0, 12'hBC5);
        chk(got === ts_bc5_a2 && ts_bc5_a2 > ts_bc5_a,
            "T5 ★同一 VL 再发一帧：条目更新为最新 SFD");


        send_frame(0, 16'h0BC5);                 // 先给 0x0BC5 一个可识别的戳
        ts_bc5_a3 = sfd_a;
        repeat (3) @(posedge clk);
        read_vl(0, 12'hBC5);
        chk(got === ts_bc5_a3, "T6a 0x0BC5 已写入基线戳");
        send_frame(0, 16'h1BC5);                 // 高位非 0 的 VLID（低 12 位同为 BC5）
        repeat (5) @(posedge clk);
        read_vl(0, 12'hBC5);
        chk(got === ts_bc5_a3, "T6b VLID>=0x1000 not written");

        $display("=============================================");
        if (errs == 0) $display(">> PASS: per-VL TX 时间戳表，%0d 项", nchk);
        else           $display(">> FAIL: %0d 项不符（共 %0d 项）", errs, nchk);
        $finish;
    end

    initial begin #2000000; $display(">> TIMEOUT"); $finish; end
endmodule
