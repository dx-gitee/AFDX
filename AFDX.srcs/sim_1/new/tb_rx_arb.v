// tb_rx_arb.v —— A/B 双网接收仲裁（afdx_rx_arb）单元测试
`timescale 1ns/1ps

module tb_rx_arb;
    reg clk = 0, reset = 1;
    always #50 clk = ~clk;                    // 10 MHz

    reg         a_valid = 0, a_sop = 0, a_eop = 0;
    reg  [7:0]  a_data  = 0;
    reg         b_valid = 0, b_sop = 0, b_eop = 0;
    reg  [7:0]  b_data  = 0;
    wire        a_ready, b_ready;
    wire        m_valid, m_sop, m_eop;
    wire [7:0]  m_data;
    wire        m_net;

    afdx_rx_arb u_arb (
        .clk(clk), .reset(reset),
        .a_valid(a_valid), .a_sop(a_sop), .a_eop(a_eop), .a_data(a_data), .a_ready(a_ready),
        .b_valid(b_valid), .b_sop(b_sop), .b_eop(b_eop), .b_data(b_data), .b_ready(b_ready),
        .m_valid(m_valid), .m_sop(m_sop), .m_eop(m_eop), .m_data(m_data), .m_net(m_net)
    );

    // ---------------- 合并流捕获 ----------------
    reg [7:0] cap_d [0:511];
    reg       cap_s [0:511], cap_e [0:511], cap_n [0:511];
    integer   ncap = 0;
    always @(posedge clk) if (m_valid) begin
        cap_d[ncap] = m_data; cap_s[ncap] = m_sop;
        cap_e[ncap] = m_eop;  cap_n[ncap] = m_net;
        ncap = ncap + 1;
    end

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask
    task mark; begin ncap = 0; end endtask          // 清捕获游标


    task do_reset;
        begin
            a_valid = 0; a_sop = 0; a_eop = 0; a_data = 0;
            b_valid = 0; b_sop = 0; b_eop = 0; b_data = 0;
            reset = 1;
            repeat (4) @(posedge clk);
            reset = 0;
            repeat (2) @(posedge clk);
        end
    endtask


    task put_byte;
        input net; input [7:0] d; input s; input e;
        begin
            @(negedge clk);
            if (net == 0) begin a_data = d; a_sop = s; a_eop = e; a_valid = 1; end
            else          begin b_data = d; b_sop = s; b_eop = e; b_valid = 1; end
            #1;
            // 等到本网被放行（ready=1）的那一拍：数据就在这一拍的上升沿被接收
            if (net == 0) while (!a_ready) begin @(negedge clk); #1; end
            else          while (!b_ready) begin @(negedge clk); #1; end
            @(posedge clk);
            @(negedge clk);
            if (net == 0) a_valid = 0; else b_valid = 0;
        end
    endtask

    // 发一整帧（每字节都能被接收；忙时自动等反压）
    task put_frame;
        input net; input integer len; input [7:0] base;
        integer k;
        begin
            for (k = 0; k < len; k = k + 1)
                put_byte(net, base + k[7:0], (k == 0), (k == len - 1));
        end
    endtask

    integer k, bad;

    initial begin
        #200 reset = 0;
        repeat (4) @(posedge clk);
        $display("");
        $display("======== A/B 双网接收仲裁（afdx_rx_arb）========");

        // ---------------- T1 A 网普通帧 ----------------
        do_reset;
        mark; put_frame(0, 8, 8'h10);
        chk(ncap == 8, "T1a A 网 8 字节帧完整上行（8 字节）");
        bad = 0;
        for (k = 0; k < 8; k = k + 1)
            if (cap_d[k] !== (8'h10 + k[7:0]) || cap_n[k] !== 1'b0 ||
                cap_s[k] !== (k == 0) || cap_e[k] !== (k == 7)) bad = bad + 1;
        chk(bad == 0, "T1b ★逐字节正确（数据/net=A/sop 仅首字节/eop 仅末字节）");

        // ---------------- T2 B 网普通帧 ----------------
        do_reset;
        mark; put_frame(1, 6, 8'h20);
        chk(ncap == 6 && cap_n[0] === 1'b1 && cap_e[5] === 1'b1,
            "T2 B 网 6 字节帧完整上行、net 标记 = 1");


        do_reset;
        mark; put_byte(0, 8'hAA, 1'b1, 1'b1);
        chk(u_arb.busy === 1'b0, "T3a ★单字节(sop&eop)帧之后 busy 必须落回 0（不许锁死）");
        mark; put_frame(1, 4, 8'h30);
        chk(ncap == 4 && cap_e[3] === 1'b1 && cap_n[0] === 1'b1,
            "T3b ★之后 B 网仍能收帧（b_ready 没有被永久拉死）");

        // ---------------- T4 忙时另一张网被反压，字节不丢 ----------------
        //   A 忙期间 B 先摆一个字节：b_ready 必须是 0（B 的 MAC 会保持该字节）
        do_reset;
        @(negedge clk); a_data = 8'h40; a_sop = 1'b1; a_eop = 1'b0; a_valid = 1'b1; b_valid = 0;
        @(posedge clk);                       // A 的首字节被接收，busy=1
        @(negedge clk); b_data = 8'hBB; b_sop = 1'b0; b_eop = 1'b0; b_valid = 1'b1; #1;
        chk(b_ready === 1'b0, "T4a 忙(A)时 B 被反压（b_ready=0，字节不会被吞）");
        chk(a_ready === 1'b1, "T4b 同时 A 仍被放行（不误伤正在转发的那张网）");
        //   A 帧收尾 → 释放
        @(posedge clk); a_valid = 0; @(negedge clk); a_data = 8'h41; a_sop = 0; a_eop = 1'b1; a_valid = 1'b1;
        @(posedge clk); @(negedge clk); a_valid = 0;
        @(posedge clk);
        chk(b_ready === 1'b1, "T4c A 帧 eop 之后 B 立刻被放行（下一拍就能上行）");
        //   B 那一个字节接着走完
        mark;
        @(posedge clk); @(negedge clk); b_valid = 0;
        chk(ncap >= 1 && cap_d[0] === 8'hBB, "T4d B 被压住的那个字节原样上行（没丢）");

        // ---------------- T5 回归：eop 但 valid=0 不许提前释放 busy ----------------
        //   MAC 小 FIFO 队首残留会让 eop 在 valid=0 时仍为 1；只判 eop 会把帧腰斩。
        do_reset;
        mark;
        put_byte(0, 8'h50, 1'b1, 1'b0);       // 首字节
        //   人为制造"eop=1 而 valid=0"的一拍
        @(negedge clk); a_eop = 1'b1; a_valid = 1'b0; #1;
        chk(u_arb.busy === 1'b1, "T5a eop 但 valid=0 时 busy 保持（队首残留不腰斩本帧）");
        @(negedge clk); a_eop = 1'b0;
        put_byte(0, 8'h51, 1'b0, 1'b1);       // 末字节
        chk(ncap == 2 && cap_s[0] === 1'b1 && cap_e[1] === 1'b1,
            "T5b ★整帧仍是 2 字节、sop/eop 位置正确（没被拆成两帧）");


        a_valid = 0; b_valid = 0;
        do_reset;
        @(negedge clk); a_data = 8'h60; a_sop = 1'b1; a_eop = 1'b0; a_valid = 1'b1;
        for (k = 0; k < 4096; k = k + 1) begin   // 全程不给 eop
            @(negedge clk); #1;
            while (!a_ready) begin @(negedge clk); #1; end
            @(posedge clk);
        end
        @(negedge clk); a_valid = 0;
        repeat (45000) @(posedge clk);   // 125 MHz 迁移后看门狗阈值 40950 拍
        chk(u_arb.busy === 1'b0, "T6 ★连续 4096 字节无 eop → busy 被兜底强制释放（不会永久卡住）");

        $display("==============================================");

        do_reset;
        mark;
        a_valid = 1; a_sop = 1; a_eop = 0; a_data = 8'h11; @(posedge clk);
        a_sop = 0;
        for (k = 0; k < 100; k = k + 1) begin a_data = k[7:0]; @(posedge clk); end
        a_valid = 0; a_sop = 0; a_eop = 0;
        repeat (45000) @(posedge clk);                  // > 40950 拍（125 MHz 迁移后的看门狗阈值）
        chk(u_arb.busy === 1'b0,
            "T7 ★★被选中的网在帧中间彻底停住时，看门狗仍能兜底释放（旧实现会永久 busy）");

        mark;
        b_valid = 1; b_sop = 1; b_eop = 0; b_data = 8'hB0; @(posedge clk);
        b_sop = 0;
        for (k = 1; k < 9; k = k + 1) begin b_data = 8'hB0 + k[7:0]; @(posedge clk); end
        b_eop = 1; b_data = 8'hB9; @(posedge clk);
        b_valid = 0; b_sop = 0; b_eop = 0; @(posedge clk);
        repeat (10) @(posedge clk);
        chk(ncap >= 10 && cap_n[0] === 1'b1 && cap_e[9] === 1'b1,
            "T7b ★★兜底之后 B 网仍能正常收帧（没有被永久反压）");

        if (errs == 0) $display(">> PASS: 双网仲裁（分帧/反压/net 标记/退化帧/队首残留/兜底）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #40000000; $display(">> TIMEOUT errs=%0d ncap=%0d", errs, ncap); $finish; end
endmodule
