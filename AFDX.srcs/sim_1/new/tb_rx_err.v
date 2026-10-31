// tb_rx_err.v —— 接收解析器的错误路径专项（ARINC 664 P7 §3.2.5/§3.2.6.1）
`timescale 1ns/1ps

module tb_rx_err;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    // ---- 喂给解析器的字节流 ----
    reg        iv = 0, isop = 0, ieop = 0;
    reg  [7:0] id = 0;
    reg        inet = 0;

    wire        ok, err_eth, err_ip, err_udp, err_len, err_fcs;
    wire [15:0] rv_vlid;
    wire [7:0]  rv_sn;
    wire [10:0] rv_framelen, rv_paylen;
    wire        ov, osop, oeop;
    wire [7:0]  od;

    afdx_rx_parse u_dut (
        .clk(clk), .reset(reset),
        .in_valid(iv), .in_sop(isop), .in_eop(ieop), .in_data(id), .in_net(inet),
        .ok(ok), .vlid(rv_vlid), .sip(), .dip(), .sport(), .dport(),
        .paylen(rv_paylen), .sn(rv_sn), .net(), .framelen(rv_framelen),
        .err_eth(err_eth), .err_ip(err_ip), .err_udp(err_udp), .err_len(err_len), .err_fcs(err_fcs),
        .out_valid(ov), .out_ready(1'b1), .out_sop(osop), .out_eop(oeop), .out_data(od),
        .out_port(), .desc_vld(), .vlid_vld()
    );

    // ---- 载荷捕获 ----
    reg [7:0] got [0:15];
    integer   np = 0;
    always @(posedge clk) if (ov) begin got[np] <= od; np <= np + 1; end

    // ---- 帧构造（64 字节，与 tx_build 同布局）----
    reg [7:0] f [0:79];
    integer   flen;

    reg [7:0] ifid_byte = 8'h20;   // IFID=1(A) → 0x20 ; IFID=2(B) → 0x40
    reg       net_sel   = 1'b0;    // 0=A 网, 1=B 网
    integer   i, k;
    reg [31:0] c0;

    function [31:0] crc32b;
        input [31:0] c; input [7:0] d;
        integer j; reg [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (j = 0; j < 8; j = j + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            crc32b = t;
        end
    endfunction

    // 造帧：pl=载荷字节数, da0=DA 首字节, snv=序列号, badip=破坏 IP 校验和, badfcs=破坏 FCS
    task build;
        input integer pl;
        input [7:0]   da0;
        input [7:0]   snv;
        input         badip;
        input         badfcs;
        integer       b, nw;
        reg [31:0]    cs, sum, c;
        begin
            for (b = 0; b < 80; b = b + 1) f[b] = 8'h00;
            // ETH
            f[0]=da0; f[1]=8'h00; f[2]=8'h00; f[3]=8'h00; f[4]=8'h01; f[5]=8'h23;
            f[6]=8'h02; f[7]=8'h00; f[8]=8'h00; f[9]=8'h00; f[10]=8'h01; f[11]=ifid_byte;
            f[12]=8'h08; f[13]=8'h00;
            // IP
            f[14]=8'h45; f[15]=8'h00;
            f[16]=(28+pl)>>8; f[17]=(28+pl)&8'hFF;
            f[18]=8'h00; f[19]=8'h00; f[20]=8'h40; f[21]=8'h00;
            f[22]=8'h01; f[23]=8'h11; f[24]=8'h00; f[25]=8'h00;
            f[26]=8'h0A; f[27]=8'h00; f[28]=8'h00; f[29]=8'h01;   // 源 IP 10.0.0.1
            f[30]=8'hE0; f[31]=8'hE0; f[32]=8'h01; f[33]=8'h23;   // 目的 224.224.1.35
            // IP 校验和
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {f[b], f[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            f[24] = badip ? 8'h5A : cs[15:8];
            f[25] = badip ? 8'h5A : cs[7:0];
            // UDP
            f[34]=8'h0B; f[35]=8'hB8; f[36]=8'h0B; f[37]=8'hB9;
            f[38]=(8+pl)>>8; f[39]=(8+pl)&8'hFF;
            f[40]=8'h00; f[41]=8'h00;
            // 载荷
            for (b = 0; b < pl; b = b + 1) f[42+b] = 8'hA0 + b[7:0];
            // padding 到 60，SN 放最后一字节
            flen = 42 + pl + 1;
            if (flen < 60) flen = 60;
            for (b = 42+pl; b < flen-1; b = b + 1) f[b] = 8'h00;   // padding
            f[flen-1] = snv;                                        // SN（帧末前 4 字节）
            // FCS
            c = 32'hFFFFFFFF;
            for (b = 0; b < flen; b = b + 1) c = crc32b(c, f[b]);
            c = ~c;
            f[flen]   = c[7:0];  f[flen+1] = c[15:8];
            f[flen+2] = c[23:16];f[flen+3] = c[31:24];
            if (badfcs) f[flen] = f[flen] ^ 8'hFF;     // 只破坏 FCS：帧内容合法、校验必错
            flen = flen + 4;
        end
    endtask

    // 把 f[0..flen-1] 一个字节一拍喂进去
    task feed;
        integer b;
        begin
            inet = net_sel;
            for (b = 0; b < flen; b = b + 1) begin
                @(negedge clk);
                iv = 1; id = f[b]; isop = (b == 0); ieop = (b == flen-1);
                @(posedge clk);
            end
            @(negedge clk); iv = 0; isop = 0; ieop = 0;
            repeat (4) @(posedge clk);
        end
    endtask

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    reg [7:0] sn_cap; reg ok_cap, fcs_cap, ip_cap, len_cap;
    initial begin
        #200 reset = 1'b0; repeat (5) @(posedge clk);
        $display("");
        $display("======== 接收解析器 错误路径专项 ========");

        // T1 好帧（对照）
        np = 0; build(8, 8'h03, 8'h00, 0, 0); feed;
        chk(ok===1'b1 && err_eth===1'b0 && err_ip===1'b0 && err_udp===1'b0 &&
            err_len===1'b0 && err_fcs===1'b0, "T1 好帧：ok=1 且所有 err=0");
        chk(rv_framelen === 11'd64 && rv_paylen === 11'd8 && rv_vlid === 16'h0123,
            "T1b 帧长 64 / 载荷长 8 / VLID 0x0123 解析正确");

        // T7 SN 位置（帧末前第 4 字节）
        build(8, 8'h03, 8'h77, 0, 0); feed;
        sn_cap = rv_sn;
        chk(rv_sn === 8'h77 && ok === 1'b1,
            "T7 ★SN 取自帧末前第 4 字节（喂 0x77 必须解析出 0x77；带 padding 的短帧）");

        // T2 坏 FCS
        build(8, 8'h03, 8'h11, 0, 1); feed;
        fcs_cap = err_fcs;
        chk(ok === 1'b0 && err_fcs === 1'b1, "T2 坏 FCS：ok=0 且 err_fcs=1");


        np = 0; build(8, 8'h03, 8'h12, 0, 0); feed;
        chk(ok === 1'b1 && err_fcs === 1'b0 && err_ip === 1'b0,
            "T3 ★坏 FCS 帧之后的好帧仍判 ok=1（err_fcs 不粘滞，接收通路没被打死）");
        chk(np === 8 && got[0] === 8'hA0, "T3b 且载荷正常交付（8 字节）");

        // T4 坏 IP 校验和
        build(8, 8'h03, 8'h13, 1, 0); feed;
        ip_cap = err_ip;
        chk(ok === 1'b0 && err_ip === 1'b1,
            "T4 ★坏 IP 校验和：本帧 ok=0（原来读的是刚清零的旧标志 → 坏帧被当好帧放行）");

        // T5 帧长不足（不留 padding：48+4 = 52 < 64）
        np = 0; build(8, 8'h03, 8'h14, 0, 0);
        flen = 42 + 8 + 1 + 4;                  // 覆盖 build 里补到 60 的长度
        feed;
        chk(ok === 1'b0 && err_len === 1'b1, "T5 帧长 < 64：ok=0 且 err_len=1");

        // T6 DA 常量域首字节错
        build(8, 8'h01, 8'h15, 0, 0); feed;
        chk(ok === 1'b0, "T6 DA 首字节不是 0x03：ok=0（原来 byte0 从未校验）");

        // 再确认一次：坏 DA 帧之后好帧仍正常
        build(8, 8'h03, 8'h16, 0, 0); feed;
        chk(ok === 1'b1, "T6b 坏 DA 帧之后的好帧仍 ok=1");


        ifid_byte = 8'h40; net_sel = 1'b0;
        build(8, 8'h03, 8'd17, 0, 0); feed;
        chk(ok === 1'b0 && err_eth === 1'b1, "T7 IFID 与物理端口不一致（IFID=B 却从 A 口来）：ok=0 且 err_eth=1");

        // T7b 对照：IFID=0x40 且确实从 B 网来 → 接受
        ifid_byte = 8'h40; net_sel = 1'b1;
        build(8, 8'h03, 8'd18, 0, 0); feed;
        chk(ok === 1'b1, "T7b 对照：IFID=0x40 且来自 B 网 → ok=1");

        // T7c 对照：IFID=0x20 且从 A 网来 → 接受
        ifid_byte = 8'h20; net_sel = 1'b0;
        build(8, 8'h03, 8'd19, 0, 0); feed;
        chk(ok === 1'b1, "T7c 对照：IFID=0x20 且来自 A 网 → ok=1");

        // T7d 低 5 位非 0（标准要求为 0）→ 也判无效
        ifid_byte = 8'h21; net_sel = 1'b0;
        build(8, 8'h03, 8'd20, 0, 0); feed;
        chk(ok === 1'b0, "T7d SA 第 6 字节低 5 位非 0：ok=0（整字节比较，顺带校验标准要求的 0）");

        ifid_byte = 8'h20; net_sel = 1'b0;      // 复原

        $display("========================================");


        begin : t8
            integer b8; reg [31:0] c8;
            build(200, 8'h03, 8'h00, 0, 0);
            flen = 64;
            c8 = 32'hFFFFFFFF;
            for (b8 = 0; b8 < 60; b8 = b8 + 1) c8 = crc32b(c8, f[b8]);
            c8 = ~c8;
            f[60]=c8[7:0]; f[61]=c8[15:8]; f[62]=c8[23:16]; f[63]=c8[31:24];
            feed;
            chk(ok === 1'b0 && err_len === 1'b1,
                "T8 ★★声明载荷 200 字节、实际帧只有 64 字节 → ok=0 且 err_len=1（不再静默丢失）");
        end


        begin : t9
            integer b9; reg [31:0] s9, c9;
            build(8, 8'h03, 8'h21, 0, 0);
            f[16] = 8'h00; f[17] = 8'd40;          // IP 总长 36 → 40（UDP 仍是 16）
            f[24] = 8'h00; f[25] = 8'h00;
            s9 = 0;
            for (b9 = 14; b9 < 34; b9 = b9 + 2) s9 = s9 + {f[b9], f[b9+1]};
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = ~s9[15:0];
            f[24] = s9[15:8]; f[25] = s9[7:0];     // IP 校验和重算 ⇒ 只剩"长度不自洽"这一处错
            c9 = 32'hFFFFFFFF;
            for (b9 = 0; b9 < flen-4; b9 = b9 + 1) c9 = crc32b(c9, f[b9]);
            c9 = ~c9;
            f[flen-4]=c9[7:0]; f[flen-3]=c9[15:8]; f[flen-2]=c9[23:16]; f[flen-1]=c9[31:24];
            feed;
            chk(ok === 1'b0 && err_len === 1'b1, "T9 IP len != UDP+20 rejected");
            build(8, 8'h03, 8'h22, 0, 0); feed;    // 对照：合法帧仍通过
            chk(ok === 1'b1, "T9b valid frame still ok");
        end


        begin : t9c
            integer b9; reg [31:0] s9, c9;
            build(24, 8'h03, 8'h24, 0, 0);         // 非分片骨架（载荷 24 字节）
            f[16] = 8'h08; f[17] = 8'h34;          // IP 总长 = 2100（>1499）
            f[20] = 8'h20;                          // MF=1, foff=0 → 首片
            f[24] = 8'h00; f[25] = 8'h00;
            s9 = 0;
            for (b9 = 14; b9 < 34; b9 = b9 + 2) s9 = s9 + {f[b9], f[b9+1]};
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = ~s9[15:0];
            f[24] = s9[15:8]; f[25] = s9[7:0];
            c9 = 32'hFFFFFFFF;
            for (b9 = 0; b9 < flen-4; b9 = b9 + 1) c9 = crc32b(c9, f[b9]);
            c9 = ~c9;
            f[flen-4]=c9[7:0]; f[flen-3]=c9[15:8]; f[flen-2]=c9[23:16]; f[flen-1]=c9[31:24];
            feed;
            $display("  [dbg] T9c ok=%b err_len=%b paylen=%0d framelen=%0d", ok, err_len, rv_paylen, rv_framelen);
            chk(ok === 1'b0, "T9c oversized IP len on fragment rejected");
            build(8, 8'h03, 8'h25, 0, 0); feed;             // 对照：合法帧仍通过
            chk(ok === 1'b1, "T9d valid frame still ok");
        end


        begin : t9e
            integer b9; reg [31:0] s9, c9;
            // (a) 坏校验和：首片，UDP csum = 0x5A00
            build(8, 8'h03, 8'h26, 0, 0);
            f[20] = 8'h20;                          // MF=1, foff=0 → 首片（UDP 头仍在）
            f[40] = 8'h5A;                          // UDP 校验和非 0
            f[24]=0; f[25]=0;
            s9 = 0;
            for (b9 = 14; b9 < 34; b9 = b9 + 2) s9 = s9 + {f[b9], f[b9+1]};
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = ~s9[15:0];
            f[24] = s9[15:8]; f[25] = s9[7:0];
            c9 = 32'hFFFFFFFF;
            for (b9 = 0; b9 < flen-4; b9 = b9 + 1) c9 = crc32b(c9, f[b9]);
            c9 = ~c9;
            f[flen-4]=c9[7:0]; f[flen-3]=c9[15:8]; f[flen-2]=c9[23:16]; f[flen-1]=c9[31:24];
            feed;
            $display("  [dbg] T9e first-frag bad UDP csum: ok=%b", ok);
            chk(ok === 1'b0, "T9e first-frag bad UDP csum rejected");
            // (b) 对照：同样的首片、校验和 0 → 必须通过
            build(8, 8'h03, 8'h27, 0, 0);
            f[20] = 8'h20; f[40] = 8'h00; f[41] = 8'h00;
            f[24]=0; f[25]=0;
            s9 = 0;
            for (b9 = 14; b9 < 34; b9 = b9 + 2) s9 = s9 + {f[b9], f[b9+1]};
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = (s9 & 32'hFFFF) + (s9 >> 16);
            s9 = ~s9[15:0];
            f[24] = s9[15:8]; f[25] = s9[7:0];
            c9 = 32'hFFFFFFFF;
            for (b9 = 0; b9 < flen-4; b9 = b9 + 1) c9 = crc32b(c9, f[b9]);
            c9 = ~c9;
            f[flen-4]=c9[7:0]; f[flen-3]=c9[15:8]; f[flen-2]=c9[23:16]; f[flen-1]=c9[31:24];
            feed;
            $display("  [dbg] T9f first-frag good UDP csum: ok=%b", ok);
            chk(ok === 1'b1, "T9f first-frag UDP csum=0 accepted");
        end

        if (errs == 0) $display(">> PASS: 解析器错误路径（坏 FCS/IP 校验和/短帧/坏 DA/SN 位置/不粘滞）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #500000; $display(">> TIMEOUT errs=%0d", errs); $finish; end

endmodule
