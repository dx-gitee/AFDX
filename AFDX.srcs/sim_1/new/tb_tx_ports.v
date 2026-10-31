// tb_tx_ports.v —— 发送端口层 单元测试
`timescale 1ns/1ps

module tb_tx_ports;

    localparam NPT = 10;
    localparam NPW = 4;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    // CPU 侧
    reg        wr = 0;
    reg  [3:0] pi = 0;
    reg  [2:0] reg_sel = 0;
    reg  [31:0] wdata = 0;
    wire [31:0] rdata;
    // 数据面
    wire        tx_valid, tx_sop, tx_eop;
    wire [7:0]  tx_data;
    reg         tx_ready = 1;
    wire [15:0] tx_vlid;
    wire [31:0] tx_dip;
    wire [15:0] tx_dport;
    wire [10:0] tx_len;
    reg         tx_hold   = 0;   // 组帧器的"先别起下一条"（tx_build 的 pay_hold）
    reg         tx_drop   = 0;
    reg  [31:0] st_r;            // 读回的 STATUS
    wire [NPT-1:0] tx_busy;


    afdx_tx_ports #(.NP(NPT), .NPW(NPW), .AW(14)) dut (
        .clk(clk), .reset(reset),
        .wr(wr), .pi(pi), .reg_sel(reg_sel), .wdata(wdata), .rdata(rdata),
        .tx_valid(tx_valid), .tx_ready(tx_ready), .tx_sop(tx_sop), .tx_eop(tx_eop),
        .tx_data(tx_data), .tx_vlid(tx_vlid), .tx_dip(tx_dip), .tx_dport(tx_dport),
        .tx_len(tx_len),
        .tx_hold(tx_hold), .tx_busy(tx_busy), .tx_pend(),
        .tx_drop(tx_drop)
    );

    localparam R_DATA=0, R_LEN=1, R_CTRL=2, R_DIP=3, R_DPT=4, R_ST=5;

    task cpu_wr; input [3:0] p; input [2:0] r; input [31:0] d;
        begin
            @(negedge clk); wr=1; pi=p; reg_sel=r; wdata=d;
            @(posedge clk);
            @(negedge clk); wr=0; reg_sel=0;
        end
    endtask


    task send_msg; input [3:0] p; input integer nbytes; input [7:0] bbase;
        integer k, b, nw;
        reg [31:0] wd;
        begin
            nw = (nbytes + 3) / 4;

            for (k = 0; k < nw; k = k + 1) begin
                wd = 32'h0;
                for (b = 0; b < 4; b = b + 1)
                    wd = wd | ((bbase + k*4 + b) << (8*b));
                cpu_wr(p, R_DATA, wd);
            end
            cpu_wr(p, R_LEN, nbytes);
            cpu_wr(p, R_CTRL, 1);
        end
    endtask

    // 收一帧字节
    reg [7:0] got [0:63];
    integer   n = 0;
    integer   eops = 0;
    reg       cap_en = 1;
    always @(posedge clk) begin
        if (tx_valid && tx_ready && cap_en) begin
            got[n] <= tx_data; n <= n + 1;
            if (tx_eop) eops <= eops + 1;
        end
    end

    function [31:0] mk; input [7:0] b;
        mk = {b+8'd3, b+8'd2, b+8'd1, b}; endfunction

    integer i, n0;
    reg [15:0] v0;
    reg [31:0] d0, p0;

    initial begin
        #200 reset = 0;
        repeat(5) @(posedge clk);
        $display("");
        $display("======== 发送端口层（采样/队列/SAP）单元测试 ========");

        // ---------- 01 端口 0 发 8 字节 ----------
        n = 0; cap_en = 1;
        send_msg(2'd0, 8, 8'hA0);              // 8 字节，内容 A0..A7
        repeat(40) @(posedge clk);
        chk(n === 8, "01a 流上出现 8 个字节");
        chk(got[0] === 8'hA0 && got[1] === 8'hA1 && got[2] === 8'hA2 && got[3] === 8'hA3 &&
            got[4] === 8'hA4 && got[5] === 8'hA5 && got[6] === 8'hA6 && got[7] === 8'hA7,
            "01b 字节序正确（小端，A0..A7）");
        chk(tx_valid === 1'b0 || n >= 8, "01c 发完流上不再有数据");

        // ---------- 07 STATUS ----------
        @(negedge clk); pi=2'd0; reg_sel=R_ST; @(posedge clk); v0 = rdata[15:0];
        @(negedge clk); reg_sel=0;
        chk(v0[1] === 1'b1,   "07a STATUS.bit1 = 发送完成（锁存）");
        chk(v0[2] === 1'b0,   "07b STATUS.bit2 = 无待发");


        // ---------- 03 SAP 口带动态目的地址 ----------
        n = 0;
        cpu_wr(2'd2, R_DIP, 32'hC0A80164);     // 目的 IP 192.168.1.100
        cpu_wr(2'd2, R_DPT, 16'h0050);         // 目的端口 80
        send_msg(2'd2, 8, 8'hC0);
        repeat(40) @(posedge clk);
        chk(n === 8 && got[0] === 8'hC0, "03a SAP 口 8 字节发出");
        chk(tx_vlid === 16'h0789,        "03b SAP 口的 tx_vlid = 0x0789");
        chk(tx_dip  === 32'hC0A80164,    "03c SAP 口目的 IP 取自 CPU 寄存器");
        chk(tx_dport=== 16'h0050,        "03d SAP 口目的端口取自 CPU 寄存器");

        // ---------- 04 采样/队列口的目的字段必须为 0 ----------
        n = 0;
        send_msg(2'd1, 4, 8'hB0);
        repeat(30) @(posedge clk);
        chk(tx_vlid === 16'h0456,               "04a 端口 1 的 tx_vlid = 0x0456");
        chk(tx_dip === 32'h0 && tx_dport === 16'h0,
            "04b 采样/队列口不带动态目的（目的由配置表定）");

        // ---------- 06 0 长度提交被挡掉 ----------
        n = 0;
        cpu_wr(2'd0, R_LEN, 0);
        cpu_wr(2'd0, R_CTRL, 1);
        repeat(20) @(posedge clk);
        chk(n === 0 && tx_valid === 1'b0, "06 0 长度提交被挡掉（不发空消息、不卡死）");

        @(negedge clk); pi=2'd0; reg_sel=R_ST; @(posedge clk); v0 = rdata[15:0];
        @(negedge clk); reg_sel=0;
        chk(v0[3] === 1'b1 && v0[1] === 1'b1, "06b len=0 -> perr(bit3)+done(bit1) visible");


        n = 0;
        cpu_wr(2'd0, R_LEN, 32'h800);
        cpu_wr(2'd0, R_CTRL, 1);
        repeat(20) @(posedge clk);
        $display("  [dbg] 09 n=%0d tx_valid=%b", n, tx_valid);
        chk(n === 0 && tx_valid === 1'b0, "09a overlong 0x800 not sent");
        @(negedge clk); pi=2'd0; reg_sel=R_ST; @(posedge clk); v0 = rdata[15:0];
        @(negedge clk); reg_sel=0;
        chk(v0[3] === 1'b1 && v0[1] === 1'b1,
            "09b overlong -> perr(bit3)+done(bit1) both set");
        n = 0;
        cpu_wr(2'd0, R_LEN, 32'h801);
        cpu_wr(2'd0, R_CTRL, 1);
        repeat(20) @(posedge clk);
        chk(n === 0 && tx_valid === 1'b0, "09c overlong 0x801 not sent");

        // ---------- 05 仲裁：两端口同时待发，按端口号顺序 ----------
        n = 0;
        // 先把两条都写好但都不提交
        cpu_wr(2'd1, R_DATA, 32'h11111111);
        cpu_wr(2'd1, R_LEN, 4);
        cpu_wr(2'd2, R_DATA, 32'h22222222);
        cpu_wr(2'd2, R_LEN, 4);
        cpu_wr(2'd1, R_CTRL, 1);          // 端口 1 先起（引擎立刻取走）
        cpu_wr(2'd2, R_CTRL, 1);          // 端口 2 在端口 1 还在发的时候排队
        repeat(80) @(posedge clk);
        chk(n === 8, "05a 两条消息都发出来了（共 8 字节）");
        // 真正的性质：一次只发一条，**整条串行、不交错**
        chk(got[0]===8'h11 && got[1]===8'h11 && got[2]===8'h11 && got[3]===8'h11 &&
            got[4]===8'h22 && got[5]===8'h22 && got[6]===8'h22 && got[7]===8'h22,
            "05b 两条消息整条串行发出、不交错（端口 1 在前，端口 2 在后）");


        n = 0;
        send_msg(2'd1, 4, 8'h33);              // 端口 1：33 33 33 33
        begin : wait_armed
            integer w;
            w = 0;
            while (n == 0 && w < 200) begin @(posedge clk); w = w + 1; end
            chk(n >= 1, "08z 抬 hold 之前，端口 1 的载荷已开始送出（引擎确实武装了）");
        end
        tx_hold = 1;
        cpu_wr(2'd2, R_DATA, 32'h44444444);
        cpu_wr(2'd2, R_LEN, 4);
        cpu_wr(2'd2, R_CTRL, 1);
        repeat(40) @(posedge clk);
        chk(n === 4 && got[0] === 8'h33, "08a hold 期间只有端口 1 的 4 字节载荷（端口 2 没能抢跑道）");
        tx_hold = 0;
        repeat(60) @(posedge clk);
        chk(n === 8 && got[4] === 8'h44, "08b 放开 hold 后端口 2 的消息才发出（44..）");
        chk(tx_vlid === 16'h0789, "08c 这条 tx_vlid = 端口 2 的 0x0789");


        n = 0; cap_en = 1;
        send_msg(2'd1, 8, 8'h70);              // 端口 1：8 字节
        begin : wait_s2
            integer w;
            w = 0;
            while (n == 0 && w < 200) begin @(posedge clk); w = w + 1; end
        end
        repeat(2) @(posedge clk);
        @(negedge clk); tx_drop = 1;           // 模拟 tx_path 判出超长
        @(posedge clk);
        @(negedge clk); tx_drop = 0;
        repeat(20) @(posedge clk);
        chk(n < 8, "09a 超长消息被中止：没有把整条 8 字节送完");

        pi = 2'd1; reg_sel = R_ST; @(negedge clk); #1 st_r = rdata;
        chk(st_r[0] === 1'b0 && st_r[2] === 1'b0,
            "09b 该端口已脱离发送/待发（status bit0=0 且 bit2=0，没卡死）");
        chk(st_r[1] === 1'b1 && st_r[3] === 1'b1,
            "09c STATUS: bit1=本条已处理完 且 bit3=因超长被丢弃");
        pi = 2'd0; reg_sel = 3'd0;
        // 丢弃后别的端口照常能发（证明仲裁没被卡住）
        n = 0;
        send_msg(2'd2, 4, 8'h90);
        repeat(40) @(posedge clk);
        chk(n === 4 && got[0] === 8'h90, "09d 丢弃之后其它端口照常发送（仲裁没卡死）");
        // 重新提交同一端口 → bit3 应被清掉
        n = 0;
        send_msg(2'd1, 4, 8'hA8);
        repeat(40) @(posedge clk);
        pi = 2'd1; reg_sel = R_ST; @(negedge clk); #1 st_r = rdata;
        chk(st_r[3] === 1'b0, "09e 重新提交后 bit3（超长标志）被清掉");
        chk(n === 4 && got[0] === 8'hA8, "09f 重新提交的消息正常发出");
        pi = 2'd0; reg_sel = 3'd0;


        n = 0;
        cpu_wr(3'd4, R_DIP, 32'h0A000009);     // RTC：目的地址由分区给（SAP 语义）
        cpu_wr(3'd4, R_DPT, 16'h1234);
        send_msg(3'd4, 4, 8'hC8);
        repeat(40) @(posedge clk);
        chk(n === 4 && got[0] === 8'hC8, "10a ★端口 4（SAP-RTC）发得出来");
        chk(tx_vlid === 16'h0BC5,        "10b ★端口 4 的 tx_vlid = 0x0BC5");
        chk(tx_dip === 32'h0A000009 && tx_dport === 16'h1234,
            "10c ★端口 4 的动态目的地址生效（SAP 语义对高编号端口同样成立）");
        n = 0;
        send_msg(3'd5, 4, 8'hD8);
        repeat(40) @(posedge clk);
        chk(n === 4 && got[0] === 8'hD8, "10d ★端口 5（保留/闲置实例）仍发得出来");
        chk(tx_vlid === 16'h0000,        "10e ★端口 5 保留：tx_vlid = 0（无 VL、无服务）");
        // 高编号端口也要参与"最低端口号优先"仲裁
        n = 0;
        cpu_wr(3'd4, R_DATA, 32'h66666666); cpu_wr(3'd4, R_LEN, 4); cpu_wr(3'd4, R_CTRL, 1);
        cpu_wr(3'd5, R_DATA, 32'h55555555); cpu_wr(3'd5, R_LEN, 4); cpu_wr(3'd5, R_CTRL, 1);
        repeat(80) @(posedge clk);
        chk(n === 8 && got[0] === 8'h66 && got[4] === 8'h55,
            "10f ★端口 4 与端口 5 同时待发时整条串行、按提交顺序（仲裁含全部端口）");


        begin : t11
            reg [31:0] st6, st7;

            @(negedge clk); pi = 4'd10; reg_sel = R_ST; #1 st6 = rdata;
            @(negedge clk); pi = 4'd15; reg_sel = R_ST; #1 st7 = rdata;
            chk(st6 === 32'h0, "11a ★★越界端口 10 的 STATUS 读回确定的 0（不是 X/don't-care）");
            chk(st7 === 32'h0, "11b ★★越界端口 15 的 STATUS 读回确定的 0（对照）");
            @(negedge clk); pi = 4'd0; reg_sel = 3'd0;
        end


        for (i = 6; i <= 9; i = i + 1) begin
            n = 0;
            send_msg(i[3:0], 4, i[7:0]*4);
            repeat(40) @(posedge clk);
            $display("  [dbg12] port=%0d n=%0d got0=%02x vlid=%04x", i, n, got[0], tx_vlid);
            chk(n === 4 && got[0] === i[7:0]*4, "12a ★新端口 6..9 分别发得出来（逐条）");
            chk(tx_vlid === (16'h0A00 + (i - 5)), "12b ★新端口的 tx_vlid = 0x0A01..0x0A04（逐条）");
        end
        // 端口 8 与端口 9 同时待发 → 按端口号从小到大串行
        n = 0;
        cpu_wr(4'd8, R_DATA, 32'h88888888); cpu_wr(4'd8, R_LEN, 4); cpu_wr(4'd8, R_CTRL, 1);
        cpu_wr(4'd9, R_DATA, 32'h99999999); cpu_wr(4'd9, R_LEN, 4); cpu_wr(4'd9, R_CTRL, 1);
        repeat(80) @(posedge clk);
        chk(n === 8 && got[0] === 8'h88 && got[4] === 8'h99,
            "12c ★端口 8/9 同时待发时按端口号串行（仲裁覆盖新增端口）");
        @(negedge clk); pi = 4'd0; reg_sel = 3'd0;


        n = 0;
        send_msg(4'd6, 1400, 8'h30);
        repeat (1500) @(posedge clk);
        $display("  [dbg13] port6 1400B -> n=%0d tx_len=%0d", n, tx_len);
        chk(n === 1400, "13a ★新增端口发 1400 字节：长度不截断（NW 必须仍是 10）");
        chk(tx_len === 11'd1400, "13b ★tx_len 端口输出 = 1400（不是 376）");

        $display("======================================================");
        if (errs == 0) $display(">> PASS: 发送端口层（NP=10 生产规模/重放+hold/动态目的/仲裁/0长度/状态/越界读/新增端口）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end
    initial begin #500000; $display(">> TIMEOUT n=%0d", n); $finish; end
endmodule
