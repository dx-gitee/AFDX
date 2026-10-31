// tb_flash_write.v —— SPI NOR 从机模型的写路径单元台（纯 RTL，不依赖 DUT）
`timescale 1ns/1ps

module tb_flash_write;
    reg  sclk = 0, cs_n = 1, mosi = 0;
    wire miso;

    spi_flash_model #(.AW(13), .USE_IMG(0)) u_flash (
        .sclk(sclk), .cs_n(cs_n), .mosi(mosi), .miso(miso)
    );

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin nchk = nchk + 1;
              if (c) $display("  [ok]   #%0d %0s", nchk, n);
              else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end end
    endtask

    function [7:0] pat; input integer a;
        begin pat = (a*7 + 3 + (a/256)*101) & 8'hFF; end
    endfunction

    // ---------------- SPI 主机（mode 0） ----------------
    task spi_begin; begin cs_n = 0; sclk = 0; #50; end endtask
    task spi_end;   begin sclk = 0; #10; cs_n = 1; #50; end endtask

    task spi_xfer;
        input  [7:0] o;
        output [7:0] i;
        integer k;
        begin
            i = 8'h00;
            for (k = 7; k >= 0; k = k - 1) begin
                mosi = o[k];
                #10; sclk = 1; #10;
                i[k] = miso;
                sclk = 0; #10;
            end
        end
    endtask

    reg [7:0] dummy;

    // 读 n 字节
    reg [7:0] rbuf [0:63];
    task flash_read; input [23:0] a; input integer n;
        integer k;
        begin
            spi_begin;
            spi_xfer(8'h03, dummy);
            spi_xfer(a[23:16], dummy); spi_xfer(a[15:8], dummy); spi_xfer(a[7:0], dummy);
            for (k = 0; k < n; k = k + 1) spi_xfer(8'hFF, rbuf[k]);
            spi_end;
        end
    endtask

    task flash_wren; begin spi_begin; spi_xfer(8'h06, dummy); spi_end; end endtask

    task flash_rdsr; output [7:0] s;
        begin spi_begin; spi_xfer(8'h05, dummy); spi_xfer(8'hFF, s); spi_end; end
    endtask

    task flash_pp; input [23:0] a; input integer n;
        integer k;
        begin
            spi_begin;
            spi_xfer(8'h02, dummy);
            spi_xfer(a[23:16], dummy); spi_xfer(a[15:8], dummy); spi_xfer(a[7:0], dummy);
            for (k = 0; k < n; k = k + 1) spi_xfer(k[7:0], dummy);   // 数据 = 0,1,2,...
            spi_end;
        end
    endtask

    task flash_se; input [23:0] a;
        begin
            spi_begin;
            spi_xfer(8'h20, dummy);
            spi_xfer(a[23:16], dummy); spi_xfer(a[15:8], dummy); spi_xfer(a[7:0], dummy);
            spi_end;
        end
    endtask

    reg [7:0] st;
    integer k;

    initial begin
        #100;
        $display("");
        $display("======== SPI NOR 写模型单元台 ========");

        // ---------- T1 初始读 ----------
        flash_read(24'h000100, 4);
        chk(rbuf[0] === pat(16'h0100) && rbuf[3] === pat(16'h0103), "T1 READ 初始内容（确定性模式）");

        // ---------- T2 WREN → RDSR.WEL=1 ----------
        flash_wren;
        flash_rdsr(st);
        chk(st[1] === 1'b1, "T2a WREN 后 RDSR.bit1（WEL）= 1");
        chk(st[0] === 1'b0, "T2b 模型 WIP 恒 0（瞬时完成）");

        // ---------- T3 SE → 全 0xFF；CS↑ 清 WEL ----------
        flash_se(24'h000100);                      // WEL 已置
        flash_read(24'h000100, 4);
        chk(rbuf[0] === 8'hFF && rbuf[1] === 8'hFF && rbuf[2] === 8'hFF && rbuf[3] === 8'hFF,
            "T3a SE 后读回 0xFF");
        flash_rdsr(st);
        chk(st[1] === 1'b0, "T3b SE 结束后 WEL 自动清 0");
        flash_read(24'h000800, 1);
        chk(rbuf[0] === 8'hFF, "T3c 4 KB 扇区整体擦除（0x800 也是 0xFF）");

        // ---------- T4 没有 WREN 的 PP 被忽略 ----------
        // 此时 WEL=0
        spi_begin; spi_xfer(8'h02, dummy);
        spi_xfer(8'h00, dummy); spi_xfer(8'h01, dummy); spi_xfer(8'h00, dummy);
        spi_xfer(8'h55, dummy); spi_end;
        flash_read(24'h000100, 1);
        chk(rbuf[0] === 8'hFF, "T4 ★没有 WREN 的 PP 被忽略（仍是 0xFF）");

        // ---------- T5 WREN + PP 0x55 ----------
        flash_wren;
        spi_begin; spi_xfer(8'h02, dummy);
        spi_xfer(8'h00, dummy); spi_xfer(8'h01, dummy); spi_xfer(8'h00, dummy);
        spi_xfer(8'h55, dummy); spi_end;
        flash_read(24'h000100, 1);
        chk(rbuf[0] === 8'h55, "T5 WREN + PP → 读回 0x55");
        flash_rdsr(st);
        chk(st[1] === 1'b0, "T5b PP 结束后 WEL=0");

        // ---------- T6 NOR 只能 1→0 ----------
        flash_wren;
        spi_begin; spi_xfer(8'h02, dummy);
        spi_xfer(8'h00, dummy); spi_xfer(8'h01, dummy); spi_xfer(8'h00, dummy);
        spi_xfer(8'h00, dummy); spi_end;
        flash_read(24'h000100, 1);
        chk(rbuf[0] === 8'h00, "T6 ★0x55 再 PP 0x00 → 0x00（按位与，不能变回 1）");

        // ---------- T7 页内回绕 ----------
        flash_wren; flash_se(24'h0001F0);           // 擦（整 4 KB）
        flash_wren;
        flash_pp(24'h0001F0, 32);                   // 写 0x00..0x1F，跨页边界

        flash_read(24'h0001F0, 16);
        chk(rbuf[0] === 8'h00 && rbuf[1] === 8'h01 && rbuf[15] === 8'h0F,
            "T7a ★页内回绕：0x1F0..0x1FF = 前 16 字节 0x00..0x0F");
        flash_read(24'h000100, 16);
        chk(rbuf[0] === 8'h10 && rbuf[1] === 8'h11 && rbuf[15] === 8'h1F,
            "T7b ★回绕到页首：0x100..0x10F = 后 16 字节 0x10..0x1F");
        flash_read(24'h000200, 2);
        chk(rbuf[0] === 8'hFF && rbuf[1] === 8'hFF, "T7c ★不跨页：0x200 仍是擦除态 0xFF");

        // ---------- T8 写后再擦 ----------
        flash_wren; flash_se(24'h0001F0);
        flash_rdsr(st);
        chk(st[1] === 1'b0, "T8 SE 之后 WEL=0（可以开始下一次操作）");
        flash_read(24'h0001F0, 1);
        chk(rbuf[0] === 8'hFF, "T8b 擦除后读回 0xFF");

        $display("=============================================");
        if (errs == 0) $display(">> PASS: SPI NOR 写模型（WREN/RDSR/PP/SE + NOR 语义 + 页回绕），%0d 项", nchk);
        else           $display(">> FAIL: %0d 项不符（共 %0d 项）", errs, nchk);
        $finish;
    end

    initial begin #2000000; $display(">> TIMEOUT"); $finish; end
endmodule
