// tb_host_mbox.v —— AXI-Lite 主机邮箱单元测试（纯 RTL，不依赖固件/SoC）
`timescale 1ns/1ps

module tb_host_mbox;
    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    // ---- 主机 AXI-Lite ----
    reg  [31:0] awaddr = 0; reg awvalid = 0;
    wire        awready;
    reg  [31:0] wdata = 0; reg wvalid = 0; reg [3:0] wstrb = 4'hF;
    wire        wready;
    wire [1:0]  bresp; wire bvalid; reg bready = 0;
    reg  [31:0] araddr = 0; reg arvalid = 0;
    wire        arready;
    wire [31:0] rdata; wire [1:0] rresp; wire rvalid; reg rready = 0;

    // ---- CPU 侧 ----
    reg        cpu_wr = 0, cpu_rd = 0;
    reg  [2:0] cpu_reg = 0;
    reg  [31:0] cpu_wdata = 0;
    wire [31:0] cpu_rdata;

    wire cpu_irq, host_irq;

    afdx_host_mbox #(.AW(11)) dut (
        .clk(clk), .reset(reset),
        .s_axi_awaddr(awaddr), .s_axi_awvalid(awvalid), .s_axi_awready(awready),
        .s_axi_wdata(wdata), .s_axi_wstrb(wstrb), .s_axi_wvalid(wvalid), .s_axi_wready(wready),
        .s_axi_bresp(bresp), .s_axi_bvalid(bvalid), .s_axi_bready(bready),
        .s_axi_araddr(araddr), .s_axi_arvalid(arvalid), .s_axi_arready(arready),
        .s_axi_rdata(rdata), .s_axi_rresp(rresp), .s_axi_rvalid(rvalid), .s_axi_rready(rready),
        .cpu_wr(cpu_wr), .cpu_rd(cpu_rd), .cpu_reg(cpu_reg),
        .cpu_wdata(cpu_wdata), .cpu_rdata(cpu_rdata),
        .cpu_irq(cpu_irq), .host_irq(host_irq)
    );

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin nchk = nchk + 1;
              if (c) $display("  [ok]   #%0d %0s", nchk, n);
              else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end end
    endtask

    // ---------------- AXI 主机任务 ----------------
    reg [31:0] rdv;

    task axi_write;
        input [31:0] a, d;
        begin
            @(negedge clk); awvalid = 1; awaddr = a; wvalid = 1; wdata = d; wstrb = 4'hF;
            // 等两边都握手（可能不同拍）
            while (!(awready && wready)) @(posedge clk);
            @(negedge clk); awvalid = 0; wvalid = 0;
            while (!bvalid) @(posedge clk);
            chk(bresp === 2'b00, "AXI 写响应 OKAY");
            @(negedge clk); bready = 1;
            @(posedge clk); @(negedge clk); bready = 0;
        end
    endtask

    // AW/W 分开到：AW 先一拍，W 下一拍
    task axi_write_split;
        input [31:0] a, d;
        begin
            @(negedge clk); awvalid = 1; awaddr = a;
            while (!awready) @(posedge clk);
            @(negedge clk); awvalid = 0;
            @(negedge clk); wvalid = 1; wdata = d; wstrb = 4'hF;
            while (!wready) @(posedge clk);
            @(negedge clk); wvalid = 0;
            while (!bvalid) @(posedge clk);
            @(negedge clk); bready = 1;
            @(posedge clk); @(negedge clk); bready = 0;
        end
    endtask

    task axi_read;
        input [31:0] a;
        begin
            @(negedge clk); arvalid = 1; araddr = a;
            while (!arready) @(posedge clk);
            @(negedge clk); arvalid = 0;
            while (!rvalid) @(posedge clk);
            rdv = rdata;
            @(negedge clk); rready = 1;
            @(posedge clk); @(negedge clk); rready = 0;
        end
    endtask

    // ---------------- CPU 侧任务 ----------------
    task cpu_write;
        input [2:0] r; input [31:0] d;
        begin
            @(negedge clk); cpu_wr = 1; cpu_reg = r; cpu_wdata = d;
            @(posedge clk);
            @(negedge clk); cpu_wr = 0;
        end
    endtask

    task cpu_read;
        input [2:0] r;
        begin
            @(negedge clk); cpu_rd = 1; cpu_reg = r; #1 rdv = cpu_rdata;
            @(posedge clk);
            @(negedge clk); cpu_rd = 0;
        end
    endtask

    integer i;
    reg [31:0] info;

    initial begin
        #200 reset = 0;
        repeat (4) @(posedge clk);
        $display("");
        $display("======== AXI-Lite 主机邮箱单元测试 ========");

        // ---------- T1 复位状态 ----------
        axi_read(32'h00); chk(rdv === 32'h484F5354, "T1a H_ID = HOST");
        axi_read(32'h04); chk(rdv[0] === 1'b0 && rdv[1] === 1'b0, "T1b 复位后 IN/OUT valid = 0");
        chk(cpu_irq === 1'b0 && host_irq === 1'b0, "T1c 复位后两个 IRQ = 0");

        // ---------- T2 主机 ingress ----------
        axi_write(32'h0C, {13'h0, 3'd1, 16'd13});          // INFO: port 1, len 13
        axi_write(32'h10, 32'h44332211);
        axi_write(32'h10, 32'h88776655);
        axi_write(32'h10, 32'hCCBBAA99);
        axi_write(32'h10, 32'h000000DD);
        axi_write(32'h14, 32'h1);                           // SET
        axi_read (32'h04); chk(rdv[0] === 1'b1, "T2a 投递后 IN_VALID = 1");
        chk(cpu_irq === 1'b1, "T2b cpu_irq = 1");

        // ---------- T3 CPU 读 ingress ----------
        cpu_read(3'd0); chk(rdv[0] === 1'b1, "T3a M_STAT.IN_VALID = 1");
        cpu_read(3'd1); info = rdv;
        chk(info === {13'h0, 3'd1, 16'd13}, "T3b M_IN_INFO = {port=1, len=13}");
        cpu_read(3'd2); chk(rdv === 32'h44332211, "T3c 第 1 字");
        cpu_read(3'd2); chk(rdv === 32'h88776655, "T3d 第 2 字");
        cpu_read(3'd2); chk(rdv === 32'hCCBBAA99, "T3e 第 3 字");
        cpu_read(3'd2); chk(rdv === 32'h000000DD, "T3f 第 4 字（末字高 3 字节无意义）");
        cpu_write(3'd3, 32'h1);                             // M_IN_ACK
        repeat (3) @(posedge clk);
        chk(cpu_irq === 1'b0, "T3g ACK 后 cpu_irq = 0");
        axi_read(32'h04); chk(rdv[0] === 1'b0, "T3h ACK 后主机看到 IN_VALID = 0");


        axi_write(32'h0C, {12'h0, 4'd9, 16'd4});            // INFO: port 9, len 4
        axi_write(32'h10, 32'hA5A5A5A5);
        axi_write(32'h14, 32'h1);                           // SET
        repeat (2) @(posedge clk);
        cpu_read(3'd0); chk(rdv[0] === 1'b1, "T3i port=9 delivers");
        cpu_read(3'd1); chk(rdv === {12'h0, 4'd9, 16'd4}, "T3j M_IN_INFO = {port=9, len=4}");
        cpu_write(3'd3, 32'h1);                             // M_IN_ACK
        repeat (3) @(posedge clk);

        // ---------- T4 CPU egress ----------
        cpu_write(3'd4, {13'h0, 3'd0, 16'd8});              // OUT_INFO: port 0, len 8
        cpu_write(3'd5, 32'h11111111);
        cpu_write(3'd5, 32'h22222222);
        cpu_write(3'd6, 32'h1);                             // COMMIT
        repeat (3) @(posedge clk);
        cpu_read(3'd7); chk(rdv[0] === 1'b1, "T4a M_OUT_STAT = 1");
        chk(host_irq === 1'b1, "T4b host_irq = 1");
        axi_read(32'h04); chk(rdv[1] === 1'b1, "T4c H_STATUS.OUT_VALID = 1");

        // ---------- T5 主机读 egress ----------
        axi_read(32'h18); chk(rdv === {13'h0, 3'd0, 16'd8}, "T5a H_OUT_INFO = {port=0, len=8}");
        axi_read(32'h1C); chk(rdv === 32'h11111111, "T5b 第 1 字");
        axi_read(32'h1C); chk(rdv === 32'h22222222, "T5c 第 2 字");
        axi_write(32'h20, 32'h1);                           // OUT_ACK
        repeat (3) @(posedge clk);
        chk(host_irq === 1'b0, "T5d ACK 后 host_irq = 0");
        axi_read(32'h04); chk(rdv[1] === 1'b0, "T5e H_STATUS.OUT_VALID = 0");

        // ---------- T6 AW/W 分开到 ----------
        axi_write_split(32'h0C, {13'h0, 3'd2, 16'd4});
        axi_write_split(32'h10, 32'hA5A5A5A5);
        axi_write_split(32'h14, 32'h1);
        axi_read(32'h04); chk(rdv[0] === 1'b1, "T6a 分开握手也能投递");
        cpu_read(3'd1); chk(rdv === {13'h0, 3'd2, 16'd4}, "T6b INFO 正确");
        cpu_read(3'd2); chk(rdv === 32'hA5A5A5A5, "T6c 数据正确");
        cpu_write(3'd3, 32'h1); repeat (3) @(posedge clk);

        // ---------- T7 B/R 反压 ----------
        // 写：bready 先不拉，确认 bvalid 保持
        @(negedge clk); awvalid = 1; awaddr = 32'h08; wvalid = 1; wdata = 32'h1; wstrb = 4'hF;
        while (!(awready && wready)) @(posedge clk);
        @(negedge clk); awvalid = 0; wvalid = 0;
        repeat (5) @(posedge clk);
        chk(bvalid === 1'b1, "T7a bvalid 在 bready 拖 5 拍时保持");
        @(negedge clk); bready = 1; @(posedge clk); @(negedge clk); bready = 0;
        // 读：rready 先不拉
        @(negedge clk); arvalid = 1; araddr = 32'h00;
        while (!arready) @(posedge clk);
        @(negedge clk); arvalid = 0;
        repeat (5) @(posedge clk);
        chk(rvalid === 1'b1 && rdata === 32'h484F5354, "T7b rvalid/rdata 在 rready 拖 5 拍时保持");
        @(negedge clk); rready = 1; @(posedge clk); @(negedge clk); rready = 0;

        // ---------- T8 软复位 ----------
        axi_write(32'h0C, {13'h0, 3'd1, 16'd4});
        axi_write(32'h10, 32'hDEADBEEF);
        axi_write(32'h14, 32'h1);
        axi_read(32'h04); chk(rdv[0] === 1'b1, "T8a 软复位前 IN_VALID = 1");
        axi_write(32'h08, 32'h2);                           // CTRL.bit1
        repeat (3) @(posedge clk);
        axi_read(32'h04); chk(rdv[0] === 1'b0 && rdv[1] === 1'b0, "T8b 软复位后 valid 全 0");

        // ---------- T9 主机中断使能 ----------
        axi_write(32'h08, 32'h0);                           // IRQ disable
        cpu_write(3'd4, {13'h0, 3'd0, 16'd4});
        cpu_write(3'd5, 32'h12345678);
        cpu_write(3'd6, 32'h1);
        repeat (3) @(posedge clk);
        chk(host_irq === 1'b0, "T9a IRQ 关闭时 host_irq = 0");
        axi_read(32'h04); chk(rdv[1] === 1'b1, "T9b 但 OUT_VALID 仍然是 1（状态不受使能影响）");
        axi_write(32'h08, 32'h1);                           // IRQ enable
        repeat (2) @(posedge clk);
        chk(host_irq === 1'b1, "T9c 重新使能后 host_irq = 1");
        axi_write(32'h20, 32'h1); repeat (2) @(posedge clk);

        // ---------- T10 ingress/egress 独立交错 ----------
        // 主机写 ingress 的同时 CPU 写 egress（不同 RAM，互不干扰）
        @(negedge clk); awvalid = 1; awaddr = 32'h0C; wvalid = 1;
        wdata = {13'h0, 3'd1, 16'd4}; wstrb = 4'hF;
        while (!(awready && wready)) @(posedge clk);
        @(negedge clk); awvalid = 0; wvalid = 0;

        while (!bvalid) @(posedge clk);
        @(negedge clk); bready = 1; @(posedge clk); @(negedge clk); bready = 0;
        cpu_write(3'd4, {13'h0, 3'd0, 16'd4});
        cpu_write(3'd5, 32'hCAFEBABE);
        cpu_write(3'd6, 32'h1);
        axi_write(32'h10, 32'h0BADF00D);
        axi_write(32'h14, 32'h1);
        repeat (3) @(posedge clk);
        cpu_read(3'd1); chk(rdv === {13'h0, 3'd1, 16'd4}, "T10a ingress INFO 未受 egress 影响");
        cpu_read(3'd2); chk(rdv === 32'h0BADF00D, "T10b ingress 数据正确");
        axi_read(32'h18); chk(rdv === {13'h0, 3'd0, 16'd4}, "T10c egress INFO 未受 ingress 影响");
        axi_read(32'h1C); chk(rdv === 32'hCAFEBABE, "T10d egress 数据正确");
        cpu_write(3'd3, 32'h1); axi_write(32'h20, 32'h1);
        repeat (2) @(posedge clk);


        begin : t11
            integer w0;
            @(negedge clk); araddr = 32'h08; arvalid = 1;      // 读 H_CTRL（长度无关）
            while (!arready) @(posedge clk);
            @(negedge clk); arvalid = 0;
            while (!rvalid) @(posedge clk);                    // 故意**不收** R
            w0 = dut.in_widx;
            @(negedge clk); awaddr = 32'h10; awvalid = 1;
                            wdata  = 32'hA5A5_5A5A; wvalid = 1; wstrb = 4'hF;
            repeat (10) @(posedge clk);
            chk(dut.in_widx === w0 && bvalid === 1'b0,
                "T11a no wr while R outstanding");
            @(negedge clk); rready = 1; @(posedge clk); @(negedge clk); rready = 0;
            repeat (10) @(posedge clk);                        // 握手补上后写才生效
            chk(dut.in_widx === w0 + 1 && bvalid === 1'b1,
                "T11b wr done after handshake ok");
            @(negedge clk); awvalid = 0; wvalid = 0; bready = 1;
            @(posedge clk); @(negedge clk); bready = 0;
            repeat (4) @(posedge clk);
        end

        $display("=============================================");
        if (errs == 0) $display(">> PASS: 主机邮箱（AXI 从机 + 双口 RAM + CPU 窗口）全部通过，%0d 项", nchk);
        else           $display(">> FAIL: %0d 项不符（共 %0d 项）", errs, nchk);
        $finish;
    end

    initial begin #2000000; $display(">> TIMEOUT"); $finish; end
endmodule
