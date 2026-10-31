// tb_flash.v —— SPI Flash 读取控制器单元测试（T5）
`timescale 1ns/1ps


// ---------------- 主测试台 ----------------
module tb_flash;
    reg clk = 0, reset = 1;
    always #4  clk = ~clk;   // 125 MHz

    // CPU 窗口
    reg        wr = 0, rd = 0;
    reg  [3:0] reg_sel = 0;
    reg  [31:0] wdata = 0;
    wire [31:0] rdata;
    localparam R_ADDR = 4'd0, R_DATA = 4'd1, R_STAT = 4'd2, R_STOP = 4'd3;
    localparam R_WADDR= 4'd4, R_WDATA= 4'd5, R_WLEN = 4'd6, R_WCMD = 4'd7;

    wire sclk, mosi, miso;
    wire cs_n;

    afdx_flash_ctrl #(.DIV(40)) u_dut (
        .clk(clk), .reset(reset),
        .wr(wr), .rd(rd), .reg_sel(reg_sel), .wdata(wdata), .rdata(rdata),
        .sclk(sclk), .mosi(mosi), .miso(miso), .cs_n(cs_n)
    );

    spi_flash_model #(.AW(13), .USE_IMG(0)) u_flash (
        .sclk(sclk), .cs_n(cs_n), .mosi(mosi), .miso(miso)
    );

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs = errs + 1; $display("  [FAIL] %0s", n); end end
    endtask

    // ---- CPU 写 ----
    task cpu_wr; input [3:0] r; input [31:0] d;
        begin @(negedge clk); wr=1; reg_sel=r; wdata=d;
              @(posedge clk); @(negedge clk); wr=0; reg_sel=0; wdata=0; end
    endtask
    // ---- CPU 读（组合 rdata，读选通 1 拍）----
    reg [31:0] rv;
    task cpu_rd; input [3:0] r;
        begin @(negedge clk); rd=1; reg_sel=r; #1 rv = rdata;
              @(posedge clk); @(negedge clk); rd=0; reg_sel=0; end
    endtask

    // ---- 等写类操作完成（轮询 F_STAT.bit5 = done，带超时）----
    task wait_done;
        integer w;
        begin
            w = 0;
            cpu_rd(R_STAT);

            while (!(rv[5] === 1'b1 && rv[0] === 1'b0) && w < 2500000) begin
                @(negedge clk); rd = 1; reg_sel = R_STAT; #1 rv = rdata;
                @(posedge clk); @(negedge clk); rd = 0; reg_sel = 0;
                w = w + 1;
            end
            if (w >= 200000) chk(1'b0, "wait_done TIMEOUT (flash op stuck)");
            else if (rv[14] === 1'b1) chk(1'b0, "wait_done op_err=1 (flash reported error)");
        end
    endtask

    // ---- 页编程 N 个字节（数据预先放在 wdat[]）----
    reg [7:0] wdat [0:63];
    task pp_bytes;
        input [23:0] addr; input integer n; input integer base;
        integer j;
        begin
            cpu_wr(R_WADDR, {8'h0, addr});
            for (j = 0; j < n; j = j + 1) cpu_wr(R_WDATA, {24'h0, wdat[base+j]});
            cpu_wr(R_WLEN, n);
            wait_done;
        end
    endtask

    // ---- 用控制器从 flash 读 n 个字节 ----
    reg [7:0] got [0:63];
    task flash_read;
        input [23:0] addr; input integer n;
        integer k, w;
        begin
            cpu_wr(R_ADDR, {8'h0, addr});
            for (k = 0; k < n; k = k + 1) begin
                w = 0;
                cpu_rd(R_STAT);
                while (!(rv[1] == 1'b1) && w < 3750) begin   // 等 FIFO 有数据
                    @(negedge clk); rd=1; reg_sel=R_STAT; #1 rv = rdata;
                    @(posedge clk); @(negedge clk); rd=0; reg_sel=0;
                    w = w + 1;
                end
                cpu_rd(R_DATA);
                got[k] = rv[7:0];
            end
            cpu_wr(R_STOP, 32'h1);
            repeat (4) @(posedge clk);
        end
    endtask

    // 与 flash 模型**同一个模式**（含地址高位）
    function [7:0] pat; input integer i;
        begin pat = (i*7 + 3 + (i/256)*101) & 8'hFF; end
    endfunction

    // ---- 取 n 个字节但**不写 F_STOP**（用于"连续读"相关的边界用例）----
    task rd_bytes; input integer n; input integer base;
        integer j, w;
        begin
            for (j = 0; j < n; j = j + 1) begin
                w = 0;
                cpu_rd(R_STAT);
                while (!(rv[1] === 1'b1) && w < 3750) begin
                    @(negedge clk); rd = 1; reg_sel = R_STAT; #1 rv = rdata;
                    @(posedge clk); @(negedge clk); rd = 0; reg_sel = 0;
                    w = w + 1;
                end
                cpu_rd(R_DATA);
                got[base+j] = rv[7:0];
            end
        end
    endtask

    integer k;
    initial begin
        #200 reset = 0;
        repeat (4) @(posedge clk);
        $display("");
        $display("======== SPI Flash 读取控制器（mode 0 / 0x03 READ）========");
        // ---- 01 从地址 0 读 8 字节，逐字节对拍 ----
        flash_read(24'h000000, 8);
        $display("  [信息] 本次读到: %02x %02x %02x %02x %02x %02x %02x %02x",
                 got[0],got[1],got[2],got[3],got[4],got[5],got[6],got[7]);
        chk(cs_n === 1'b1, "01a 读完之后 CS 已拉高（真的停下了）");
        chk(got[0] === pat(0) && got[1] === pat(1) && got[2] === pat(2) && got[3] === pat(3) &&
            got[4] === pat(4) && got[5] === pat(5) && got[6] === pat(6) && got[7] === pat(7),
            "01b ★从地址 0 读 8 字节逐字节正确（命令/地址位序 + 数据起始地址都对）");
        chk(got[0] !== 8'h00 && got[0] !== 8'hFF, "01c 读到的不是 0/FF（不是空模型）");

        // ---- 02 换一个非 0 起始地址，验证地址真的被用上 ----
        flash_read(24'h000100, 4);
        chk(got[0] === pat(256) && got[3] === pat(259),
            "02 ★从地址 0x100 读 4 字节正确（地址被真的用上了，不是恒读 0）");

        // ---- 03 连续两次读之间状态干净（再来一次 0 地址）----
        flash_read(24'h000000, 4);
        chk(got[0] === pat(0), "03 再次从 0 读仍然正确（状态复位干净）");

        // ---- 04 跨页边界（0xFF0 读到 0x1000 之后 wraps，但至少前 16 字节对）----
        flash_read(24'h000FF0, 16);
        begin : t04
            integer bad; bad = 0;
            for (k = 0; k < 16; k = k + 1)
                if (got[k] !== pat(12'hFF0 + k)) bad = bad + 1;
            chk(bad == 0, "04 ★跨页读 16 字节全对（地址逐字节递增，无错位）");
        end


        cpu_wr(R_ADDR, 32'h0);
        repeat (27500) @(posedge clk);            // 明显超过"填满"所需
        cpu_rd(R_STAT);
        chk(rv[2] === 1'b1, "05 ★FIFO 溢出时 F_STAT.bit2 置起（溢出对固件可见，不再静默丢字节）");
        chk(rv[1] === 1'b1, "05b 溢出时 FIFO 仍报有数据（bit1=1，计数确实到顶了）");
        cpu_wr(R_STOP, 32'h1);
        repeat (4) @(posedge clk);

        // ---- 06 溢出标志由"下一次启动读"清掉（每笔读独立判定）----
        cpu_wr(R_ADDR, 32'h0);                  // 重新启动 → R_ADDR 里清 ovf/fcnt/fw/fr
        cpu_rd(R_STAT);
        chk(rv[2] === 1'b0, "06 重新启动一笔读会清掉溢出标志（上一笔的溢出不粘滞）");
        chk(rv[1] === 1'b0, "06b 同时清空了 FIFO 计数（bit1=0）");
        cpu_wr(R_STOP, 32'h1);
        repeat (4) @(posedge clk);


        cpu_wr(R_ADDR, 32'h000000);          // 读地址 0，取 2 个字节
        rd_bytes(2, 0);
        cpu_wr(R_ADDR, 32'h000100);
        rd_bytes(8, 16);
        begin : t07
            integer bad; bad = 0;
            for (k = 0; k < 8; k = k + 1)
                if (got[16+k] !== pat(256 + k)) bad = bad + 1;
            chk(bad == 0, "07 ★不写 F_STOP 直接换地址 → 仍然从**新地址**读到正确的 8 字节");
        end
        cpu_wr(R_STOP, 32'h1);
        repeat (4) @(posedge clk);


        cpu_wr(R_ADDR, 32'h000000);
        repeat (7500) @(posedge clk);          // 让 SPI 往 FIFO 里灌进 ~9 个字节
        cpu_wr(R_STOP, 32'h1);
        repeat (4) @(posedge clk);
        cpu_rd(R_STAT);
        chk(rv[1] === 1'b0, "08a ★F_STOP 之后 F_STAT 不再报有数据（bit1=0）");
        cpu_rd(R_DATA);
        chk(rv === 32'h0, "08b ★F_STOP 之后读 F_DATA 返回 0（不会吐出上一笔的残留）");


        cpu_wr(R_ADDR, 32'h000000);
        k = 0;
        while (!(u_dut.fcnt == 5'd16) && k < 62500) begin @(posedge clk); k = k + 1; end
        chk(u_dut.fcnt == 5'd16, "09a FIFO 已填满 16 字节（构造成功）");
        k = 0;
        while (!((u_dut.st == 2'd2) && (u_dut.rxbit == 3'd7) && (u_dut.hc == 7'd79)) && k < 2000) begin
            @(negedge clk); k = k + 1;
        end
        chk((u_dut.st == 2'd2) && (u_dut.rxbit == 3'd7) && (u_dut.hc == 7'd79),
            "09b 找到「本拍即将入队」的时钟（push 与 pop 可同拍）");
        // 在 push 发生的那个 posedge 上同时拉 rd（cpu 读 F_DATA）
        rd = 1; reg_sel = R_DATA; #1 got[0] = rdata;
        @(posedge clk);
        @(negedge clk); rd = 0; reg_sel = 0;
        chk(u_dut.ovf === 1'b0, "09c ★★满 + 同拍弹出时**不**误报溢出");

        for (k = 0; k < 16; k = k + 1) begin
            cpu_rd(R_DATA);
            got[1+k] = rv[7:0];
        end
        begin : t09
            integer bad; bad = 0;
            for (k = 0; k < 17; k = k + 1)
                if (got[k] !== pat(k)) bad = bad + 1;
            chk(bad == 0, "09d ★★满时同拍弹出+入队：17 个字节仍逐字节连续（一个都没丢）");
        end
        cpu_wr(R_STOP, 32'h1);
        repeat (4) @(posedge clk);


        cpu_wr(R_WCMD, 32'd1);                 // WREN
        wait_done;
        cpu_wr(R_WCMD, 32'd5);                 // RDSR
        wait_done;
        cpu_rd(R_STAT);
        chk(rv[4] === 1'b1, "10a ★WREN 后 RDSR.WEL = 1");
        chk(rv[3] === 1'b0, "10b WIP = 0（本模型瞬时完成）");

        // ---- 11 SE：整片擦成 0xFF；之后 WEL 被 CS↑ 清掉 ----
        cpu_wr(R_STOP, 32'h1); repeat (4) @(posedge clk);
        cpu_wr(R_WADDR, 24'h000100);
        cpu_wr(R_WCMD, 32'd3);                 // SE
        wait_done;
        flash_read(24'h000100, 4);
        chk(got[0] === 8'hFF && got[1] === 8'hFF && got[2] === 8'hFF && got[3] === 8'hFF,
            "11a ★SE 之后 4 字节全为 0xFF");
        cpu_wr(R_WCMD, 32'd5); wait_done; cpu_rd(R_STAT);
        chk(rv[4] === 1'b0, "11b SE 完成后 WEL 被清（CS↑ 语义）");

        // ---- 12 PP：写 3 个字节并逐字节读回 ----
        wdat[0]=8'h11; wdat[1]=8'h22; wdat[2]=8'h33;
        pp_bytes(24'h000100, 3, 0);
        flash_read(24'h000100, 4);
        chk(got[0] === 8'h11 && got[1] === 8'h22 && got[2] === 8'h33 && got[3] === 8'hFF,
            "12 ★PP 3 字节后逐字节读回（0x11 0x22 0x33 0xFF）");

        // ---- 13 NOR 语义：只能 1→0，写 0xFF 不能把 0 变回 1 ----
        wdat[0]=8'h00;
        pp_bytes(24'h000100, 1, 0);
        flash_read(24'h000100, 1);
        chk(got[0] === 8'h00, "13a NOR：0x11 上写 0x00 → 0x00");
        wdat[0]=8'hFF;
        pp_bytes(24'h000100, 1, 0);
        flash_read(24'h000100, 1);
        chk(got[0] === 8'h00, "13b ★NOR：再写 0xFF 仍然是 0x00（只能擦除恢复）");

        // ---- 14 写缓冲上限：压 65 字节 → 溢出可见；实际编程 64 字节 ----
        cpu_wr(R_WADDR, 24'h000200);
        for (k = 0; k < 65; k = k + 1) cpu_wr(R_WDATA, {24'h0, 8'hA0 + k[7:0]});
        cpu_rd(R_STAT);
        chk(rv[6] === 1'b1, "14a ★写缓冲超过 64 字节 → F_STAT.bit6 溢出可见");
        cpu_wr(R_WLEN, 32'd64);
        wait_done;
        flash_read(24'h000200, 64);
        begin : t14
            integer bad; bad = 0;
            for (k = 0; k < 64; k = k + 1)
                if (got[k] !== (8'hA0 + k[7:0])) bad = bad + 1;
            chk(bad == 0, "14b ★64 字节页编程逐字节正确（从 0xFF 擦除态起写）");
        end

        // ---- 15 页内多块 + 跨页边界：固件必须按 256 字节页对齐分块 ----
        //   这里验证「页首 0x2F0 + 16 字节」不跨页，以及「0x300 再写」互不影响。
        for (k = 0; k < 16; k = k + 1) wdat[k] = 8'h10 + k[7:0];
        pp_bytes(24'h0002F0, 16, 0);
        for (k = 0; k < 16; k = k + 1) wdat[k] = 8'h30 + k[7:0];
        pp_bytes(24'h000300, 16, 0);
        flash_read(24'h0002F0, 16);
        begin : t15a
            integer bad; bad = 0;
            for (k = 0; k < 16; k = k + 1)
                if (got[k] !== (8'h10 + k[7:0])) bad = bad + 1;
            chk(bad == 0, "15a 0x2F0 的 16 字节正确（未跨页）");
        end
        flash_read(24'h000300, 16);
        begin : t15b
            integer bad; bad = 0;
            for (k = 0; k < 16; k = k + 1)
                if (got[k] !== (8'h30 + k[7:0])) bad = bad + 1;
            chk(bad == 0, "15b 0x300 的 16 字节正确（页边界另一侧未被破坏）");
        end


        cpu_wr(R_WADDR, 24'h001000);
        cpu_wr(R_WCMD, 32'd3); wait_done;
        flash_read(24'h001000, 2);
        chk(got[0] === 8'hFF && got[1] === 8'hFF, "16a SE(0x1000) -> FF");
        wdat[0]=8'h41; wdat[1]=8'h46; wdat[2]=8'h53; wdat[3]=8'h31;   // "AFS1"
        pp_bytes(24'h001000, 4, 0);
        flash_read(24'h001000, 4);
        chk(got[0] === 8'h41 && got[1] === 8'h46 && got[2] === 8'h53 && got[3] === 8'h31,
            "16b PP(0x1000) readback AFS1");

        flash_read(24'h000100, 1);
        chk(got[0] === 8'h00, "16c SE(0x1000) did not damage sector 0 (0x100 still 0x00)");


        cpu_wr(R_STOP, 32'h1); repeat (4) @(posedge clk);
        cpu_wr(R_WADDR, 24'h001000);
        cpu_wr(R_WCMD, 32'd3); wait_done;          // 擦扇区 1
        cpu_wr(R_WADDR, 24'h0010F0);               // 页内只剩 16 字节
        for (k = 0; k < 64; k = k + 1) cpu_wr(R_WDATA, {24'h0, 8'hA0 + k[7:0]});
        cpu_wr(R_WLEN, 32'd64);                    // 64 > 16 → 必须拒绝
        begin : t17
            integer w17;
            w17 = 0;
            cpu_rd(R_STAT);
            while (!(rv[5] === 1'b1 && rv[0] === 1'b0) && w17 < 250000) begin
                @(negedge clk); rd = 1; reg_sel = R_STAT; #1 rv = rdata;
                @(posedge clk); @(negedge clk); rd = 0; reg_sel = 0;
                w17 = w17 + 1;
            end
        end
        chk(rv[14] === 1'b1, "17a ★跨 256B 页的 PP 被拒绝（op_err=1）");
        flash_read(24'h0010F0, 16);
        begin : t17b
            integer bad; bad = 0;
            for (k = 0; k < 16; k = k + 1)
                if (got[k] !== 8'hFF) bad = bad + 1;
            chk(bad == 0, "17b ★被拒绝的跨页 PP 一个字节都没写");
        end


        cpu_wr(R_STOP, 32'h1); repeat (4) @(posedge clk);
        cpu_wr(R_WADDR, 24'h001200);
        for (k = 0; k < 8; k = k + 1) cpu_wr(R_WDATA, {24'h0, 8'h50 + k[7:0]});
        cpu_wr(R_WLEN, 32'd8);                     // 启动 PP
        repeat (4) @(negedge clk);
        cpu_wr(R_WCMD, 32'd1);                     // 忙的时候发 WREN → 必须置 err
        begin : t18
            integer w18;
            w18 = 0;
            cpu_rd(R_STAT);
            while (!(rv[5] === 1'b1 && rv[0] === 1'b0) && w18 < 250000) begin
                @(negedge clk); rd = 1; reg_sel = R_STAT; #1 rv = rdata;
                @(posedge clk); @(negedge clk); rd = 0; reg_sel = 0;
                w18 = w18 + 1;
            end
        end
        chk(rv[14] === 1'b1, "18a ★非 idle 的写命令被标记为错误（不再静默丢弃）");
        flash_read(24'h001200, 8);
        begin : t18b
            integer bad; bad = 0;
            for (k = 0; k < 8; k = k + 1)
                if (got[k] !== (8'h50 + k[7:0])) bad = bad + 1;
            chk(bad == 0, "18b ★原 PP 的 8 字节仍然正确（杂散命令没破坏写事务）");
        end


        cpu_wr(R_STOP, 32'h1); repeat (4) @(posedge clk);
        cpu_wr(R_WADDR, 24'h001400);
        cpu_wr(R_WCMD, 32'd3); wait_done;
        wdat[0]=8'hA5; wdat[1]=8'h5A; wdat[2]=8'h11; wdat[3]=8'h22;
        pp_bytes(24'h001400, 4, 0);
        flash_read(24'h001400, 4);
        chk(got[0] === 8'hA5 && got[3] === 8'h22, "19a 第一次 SE+PP 读回正确");
        // 第二次：擦掉已编程的扇区再写
        cpu_wr(R_WADDR, 24'h001400);
        cpu_wr(R_WCMD, 32'd3); wait_done;
        flash_read(24'h001400, 4);
        chk(got[0] === 8'hFF && got[3] === 8'hFF, "19b 第二次 SE 把已编程扇区擦净");
        wdat[0]=8'hC3; wdat[1]=8'h3C; wdat[2]=8'h55; wdat[3]=8'hAA;
        pp_bytes(24'h001400, 4, 0);
        flash_read(24'h001400, 4);
        chk(got[0] === 8'hC3 && got[3] === 8'hAA, "19c 第二次 PP 读回新数据");

        $display("==============================================");
        if (errs == 0) $display(">> PASS: SPI Flash 读取（mode0/0x03/地址递增/CS 收尾/溢出可见）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #20000000; $display(">> TIMEOUT errs=%0d", errs); $finish; end
endmodule
