// tb_phy_mdio.v —— PHY 模型（phy_mdio_slave）单元台
`timescale 1ns/1ps

module tb_phy_mdio;

    // ---- MDIO 总线：DUT 驱动 + 上拉（与真实板一致）----
    wire mdio;
    reg  mdc = 0;
    always #20 mdc = ~mdc;          // MDC 25 MHz（仿真用；真实上限 2.5 MHz，模型只看边沿）
    pullup (mdio);

    phy_mdio_slave #(.PHY_ADDR(5'd1), .SPEED_1G(0)) dut (.mdc(mdc), .mdio(mdio));

    // ---- 主机侧（带三态驱动）----
    reg  mo = 0, mo_en = 0;
    assign mdio = mo_en ? mo : 1'bz;

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    // one MDC bit: 主机在下降沿换值、PHY 在上升沿采样（与真实时序一致）
    task bit_wr; input b;
        begin
            @(negedge mdc); mo = b; mo_en = 1;
            @(posedge mdc); #1;
        end
    endtask


    reg rb;
    task bit_rd;
        begin
            @(negedge mdc); mo_en = 0;
            @(posedge mdc); #1 rb = mdio;
        end
    endtask

    // 完整 MDIO 事务：op=2'b01 写 / 2'b10 读；pre_one=1 表示前导码只发 1 个 1
    integer i, k;
    reg [15:0] rdv;
    task mdio_xact; input [1:0] op; input [4:0] pa; input [4:0] ra;
                     input [15:0] wd; input pre_one; output [15:0] rd;
        begin
            mo_en = 1; mo = 1;
            if (pre_one) begin @(negedge mdc); @(posedge mdc); #1; end
            else begin for (i = 0; i < 32; i = i + 1) bit_wr(1'b1); end
            // ST = 01
            bit_wr(1'b0); bit_wr(1'b1);
            // OP
            bit_wr(op[1]); bit_wr(op[0]);
            // PHYAD / REGAD
            for (i = 4; i >= 0; i = i - 1) bit_wr(pa[i]);
            for (i = 4; i >= 0; i = i - 1) bit_wr(ra[i]);
            if (op == 2'b01) begin
                // TA = 10，然后 16 位数据（MSB 先）
                bit_wr(1'b1); bit_wr(1'b0);
                for (i = 15; i >= 0; i = i - 1) bit_wr(wd[i]);
                rd = 16'h0000;
            end else begin
                // TA：第 1 位主机放开（PHY 高阻）、第 2 位 PHY 驱动 0，然后 16 位数据（标准时序）
                bit_rd; bit_rd;
                for (i = 15; i >= 0; i = i - 1) begin bit_rd; rdv[i] = rb; end
                rd = rdv;
            end
            @(negedge mdc); mo_en = 0; mo = 1;
            for (k = 0; k < 4; k = k + 1) begin @(negedge mdc); @(posedge mdc); end
        end
    endtask

    reg [15:0] r1, r2, r3;
    initial begin
        #100;
        $display("======== PHY MDIO 模型单元台 ========");

        // T1：写 reg20 = 0xA55A 再读回（前导码按标准 32 个 1）
        mdio_xact(2'b01, 5'd1, 5'd20, 16'hA55A, 1'b0, r1);
        mdio_xact(2'b10, 5'd1, 5'd20, 16'h0000, 1'b0, r1);
        $display("  [dbg] T1 read reg20 = %04x（期望 a55a）", r1);
        chk(r1 === 16'hA55A, "T1 write/read-back reg20 = 0xA55A");

        // T2：**连读两个不同寄存器** —— 先 reg9（0x0C00 在 1G 模式下才有值，
        //     这里 100M 模式全 0），所以改用"写两个不同的值再分别读"：
        mdio_xact(2'b01, 5'd1, 5'd21, 16'h1234, 1'b0, r1);
        mdio_xact(2'b01, 5'd1, 5'd22, 16'h5678, 1'b0, r1);
        mdio_xact(2'b10, 5'd1, 5'd21, 16'h0000, 1'b0, r2);   // 读 21
        mdio_xact(2'b10, 5'd1, 5'd22, 16'h0000, 1'b0, r3);   // 紧接着读 22
        $display("  [dbg] T2 reg21=%04x reg22=%04x（期望 1234 / 5678）", r2, r3);
        chk(r2 === 16'h1234, "T2a read reg21 = 0x1234");
        chk(r3 === 16'h5678, "T2b read reg22 = 0x5678（换寄存器不许读成上一个）");

        // T3：前导码只有 1 个 1（本 IP 的实际形态）
        mdio_xact(2'b01, 5'd1, 5'd23, 16'hBEEF, 1'b1, r1);
        mdio_xact(2'b10, 5'd1, 5'd23, 16'h0000, 1'b1, r1);
        $display("  [dbg] T3 pre=1 read reg23 = %04x（期望 beef）", r1);
        chk(r1 === 16'hBEEF, "T3 1-bit preamble still works");

        // T4：非本 PHY 地址的读 —— PHY 不得驱动（总线被上拉为 1 ⇒ 读回 ffff）
        mdio_xact(2'b10, 5'd7, 5'd20, 16'h0000, 1'b0, r1);
        $display("  [dbg] T4 read phy7 = %04x（期望 ffff：没人驱动）", r1);
        chk(r1 === 16'hFFFF, "T4 other PHY addr not driven");

        $display("==================================");
        if (errs == 0) $display(">> PASS: PHY 模型（写/读回、换寄存器、1 位前导、地址过滤）");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end
    initial begin #2000000; $display(">> TIMEOUT"); $finish; end
endmodule
