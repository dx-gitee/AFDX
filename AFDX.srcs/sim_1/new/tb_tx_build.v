// tb_tx_build.v —— 发送组帧器测试
`timescale 1ns/1ps
module tb_tx_build;
    reg clk=0, reset=1;
    // 配置
    reg [15:0] vlid=16'h0123, sport=16'h0BB8, dport=16'h0BB9;
    reg [31:0] dip=32'hE0E00123;
    reg [1:0]  neten=2'd0;
    reg [7:0]  sn=8'h05; reg sn_vld=1'b1; reg cfg_vld=1'b1;
    // 消息
    reg        mv=0, msop=0, meop=0; reg [7:0] mdat=0; reg [10:0] mlen=8;
    wire       mrdy;
    // 帧输出
    wire       fv, fsop, feop; wire [7:0] fd; wire fnet, snadv, preplay;
    reg        frdy=1;

    afdx_tx_build #(.SRC_IP(32'h0A000001)) u_dut (
        .clk(clk), .reset(reset),
        .cfg_vlid(vlid), .cfg_sport(sport), .cfg_dport(dport), .cfg_dip(dip),
        .cfg_neten(neten), .cfg_sn(sn), .cfg_sn_vld(sn_vld), .cfg_vld(cfg_vld),
        .msg_valid(mv), .msg_ready(mrdy), .msg_sop(msop), .msg_eop(meop),
        .msg_data(mdat), .msg_len(mlen), .go(1'b1),
        .fr_valid(fv), .fr_ready(frdy), .fr_sop(fsop), .fr_eop(feop),
        .fr_data(fd), .fr_net(fnet), .sn_adv(snadv), .pay_replay(preplay)
    );
    always #50 clk = ~clk;

    always @(posedge clk) if (fv && frdy && fnet==1'b0 && u_dut.pos > 16'd38 && u_dut.pos < 16'd54)
        $display("  BYTE pos=%0d data=%02x mrdy=%b mv=%b mdat=%02x pcnt=%0d",
                 u_dut.pos, fd, mrdy, mv, mdat, pcnt);
    // 抓帧
    reg [7:0] frameA [0:127];
    reg [7:0] frameB [0:127];
    integer nA=0, nB=0, i;
    // 阶段4 专用抓帧
    reg        capB = 1'b0;
    reg [7:0]  pb4 [0:127];
    integer    n4 = 0;

    always @(posedge clk) if (fv && frdy) begin
        if (fnet==1'b0) begin frameA[nA] <= fd; nA <= nA+1; end
        else            begin frameB[nB] <= fd; nB <= nB+1; end
    end
    always @(posedge clk) if (capB && fv && frdy && fnet) begin pb4[n4] <= fd; n4 <= n4+1; end


    reg         bp_en = 0;
    reg  [1:0]  bp_div = 2'd0;
    reg  [7:0]  frame2 [0:127];
    integer     n2 = 0;

    reg        st41_en = 0, st41_done = 0;
    always @(posedge clk) begin
        if (bp_en) begin
            bp_div <= (bp_div == 2'd2) ? 2'd0 : (bp_div + 2'd1);
            frdy   <= (bp_div != 2'd1);        // 每 3 拍里拉低 1 拍（持续反压）
        end
        else if (st41_en) begin
            bp_div <= 2'd0;
            if (!st41_done && fv && (u_dut.st == 2'd2) && (u_dut.pos == 16'd41)) begin
                frdy <= 1'b0; st41_done <= 1'b1;   // 精确定位的一拍反压
            end else frdy <= 1'b1;
        end
        else begin bp_div <= 2'd0; frdy <= 1'b1; end
    end
    always @(posedge clk) if (bp_en && fv && frdy && (fnet == 1'b0)) begin
        frame2[n2] <= fd; n2 <= n2 + 1;
    end
    function [31:0] resid2;
        input integer dummy;
        integer k; reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = 0; k < 64; k = k + 1) c = crc32b(c, frame2[k]);
            resid2 = c;
        end
    endfunction

    // 载荷源
    reg [7:0] pay [0:15];
    initial begin pay[0]="A"; pay[1]="F"; pay[2]="D"; pay[3]="X";
                  pay[4]="1"; pay[5]="2"; pay[6]="3"; pay[7]="4"; end
    reg [3:0] pcnt = 0;
    reg        ok1 = 0, ok2 = 0, ok3 = 0, ok4 = 0, ok5 = 0, ok6 = 0;
    integer    bad2 = 0, w = 0;
    reg [4:0] plen_src = 5'd8;                  // 本条消息的字节数（阶段 2 改成 12）
    always @(posedge clk) begin
        if (preplay) begin                      // 双网第二遍：重放载荷
            mv <= 1'b1; msop <= 1'b1; mdat <= pay[0]; pcnt <= 4'd0; meop <= 1'b0;
        end
        else if (mv && mrdy) begin

            if (pcnt == plen_src - 1'b1) begin mv<=0; msop<=0; meop<=0; end
            else begin
                pcnt <= pcnt + 1'b1;
                mdat <= pay[pcnt + 1'b1];
                meop <= (pcnt + 1'b1 == plen_src - 1'b1);
                msop <= 1'b0;
            end
        end
    end

    // 计算 IP 校验和（用抓到的帧重算，验证发送方算得对不对）
    function [15:0] cksum_of;
        input integer n;                 // 从第 14 字节起 20 字节
        integer k; reg [31:0] sum;
        begin
            sum = 0;
            for (k = 14; k < 34; k = k + 2) begin
                if (k == 24) sum = sum + 0;                       // 校验和字段按 0
                else sum = sum + {frameA[k], frameA[k+1]};
            end
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cksum_of = ~sum[15:0];
        end
    endfunction

    // 独立实现的 CRC-32（验证组帧器补的 FCS 是否正确）
    function [31:0] crc32b;
        input [31:0] c; input [7:0] d;
        integer k; reg [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            crc32b = t;
        end
    endfunction
    function [31:0] residA;
        input integer dummy;
        integer k; reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = 0; k < 64; k = k + 1) c = crc32b(c, frameA[k]);
            residA = c;
        end
    endfunction

    function [31:0] residB;
        input integer dummy;
        integer k; reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = 0; k < 64; k = k + 1) c = crc32b(c, frameB[k]);
            residB = c;
        end
    endfunction

    initial begin
        #200 reset = 0;
        #100;                                   // 发消息
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        #20000;
        $display("");
        $display("  total=%0d  pos=%0d   (expect total=60)", u_dut.total, u_dut.pos);
        $display("======== TX frame builder ========");
        $display("  frames: A=%0d bytes, B=%0d bytes (expect 64/64，含 4 字节 FCS)", nA, nB);
        $display("  A FCS 残差 = %08x (expect DEBB20E3)", residA(0));
        $display("  A[0..13] = %02x %02x %02x %02x %02x %02x | %02x %02x %02x %02x %02x %02x | %02x %02x",
                 frameA[0],frameA[1],frameA[2],frameA[3],frameA[4],frameA[5],
                 frameA[6],frameA[7],frameA[8],frameA[9],frameA[10],frameA[11],frameA[12],frameA[13]);
        $display("  A IP len = %02x%02x (expect 0024 = 36)", frameA[16],frameA[17]);
        $display("  A IP cks  = %02x%02x  重算 = %04x", frameA[24],frameA[25], cksum_of(0));
        $display("  A UDP     = sport %02x%02x dport %02x%02x len %02x%02x cks %02x%02x",
                 frameA[34],frameA[35],frameA[36],frameA[37],frameA[38],frameA[39],frameA[40],frameA[41]);
        $display("  A payload = %c%c%c%c%c%c%c%c", frameA[42],frameA[43],frameA[44],frameA[45],
                 frameA[46],frameA[47],frameA[48],frameA[49]);

        $display("  A SN      = %02x @59 (expect 05)", frameA[59]);
        $display("  A pad[50..58] = %02x %02x %02x %02x %02x %02x %02x %02x %02x (expect 全 00)",
                 frameA[50],frameA[51],frameA[52],frameA[53],frameA[54],
                 frameA[55],frameA[56],frameA[57],frameA[58]);
        $display("  B SA[11]  = %02x (expect 40)", frameB[11]);
        $display("=================================");
        ok1 = (nA>=63 && nB>=63 && u_dut.total==16'd60 && residA(0)==32'hDEBB20E3 &&
            frameA[0]==8'h03 && frameA[4]==8'h01 && frameA[5]==8'h23 &&
            frameA[6]==8'h02 && frameA[11]==8'h20 && frameA[12]==8'h08 && frameA[13]==8'h00 &&
            frameA[14]==8'h45 && frameA[16]==8'h00 && frameA[17]==8'h24 &&
            {frameA[24],frameA[25]} == cksum_of(0) &&
            frameA[34]==8'h0B && frameA[35]==8'hB8 && frameA[36]==8'h0B && frameA[37]==8'hB9 &&
            frameA[59]==8'h05 &&                        // SN 在帧末前 4 字节（协议要求）
            frameA[50]==8'h00 && frameA[58]==8'h00 &&   // 50..58 是 padding
            frameB[11]==8'h40);
        $display("  [阶段1] 帧格式（MAC/IP/UDP/SN/padding/FCS/双网）: %s", ok1 ? "PASS" : "FAIL");

        // ================= 阶段 2：MAC 反压下的载荷完整性 =================
        pay[0]=8'h10; pay[1]=8'h11; pay[2]=8'h12; pay[3]=8'h13;
        pay[4]=8'h14; pay[5]=8'h15; pay[6]=8'h16; pay[7]=8'h17;
        pay[8]=8'h18; pay[9]=8'h19; pay[10]=8'h1A; pay[11]=8'h1B;
        plen_src = 5'd12;
        mlen = 11'd12;
        n2 = 0; bp_en = 1;
        w = 0;
        while ((fv || preplay) && w < 100000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        w = 0;
        while (n2 < 64 && w < 200000) begin @(posedge clk); w = w + 1; end
        bp_en = 0;
        repeat(4) @(posedge clk);
        bad2 = 0;
        for (i = 0; i < 12; i = i + 1) if (frame2[42+i] !== (8'h10 + i[7:0])) bad2 = bad2 + 1;
        ok2 = (n2 >= 64) && (bad2 == 0) && (resid2(0) == 32'hDEBB20E3) &&
              (frame2[59] === 8'h05) && (frame2[54] === 8'h00);
        $display("  [阶段2] 反压下的 12 字节载荷: n2=%0d 错字节=%0d FCS 残差=%08x SN=%02x",
                 n2, bad2, resid2(0), frame2[59]);
        $display("  [阶段2] MAC 反压（fr_ready 每 3 拍拉低 1 拍）下的载荷完整性: %s",
                 ok2 ? "PASS" : "FAIL");


        sn_vld = 1'b0;
        pay[0]=8'h20; pay[1]=8'h21;
        plen_src = 5'd2;  mlen = 11'd2;
        n2 = 0;
        bp_en = 1;

        w = 0;
        while ((fv || preplay) && w < 100000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        w = 0;
        while (n2 < 64 && w < 200000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        ok3 = (n2 >= 64) && (resid2(0) == 32'hDEBB20E3) &&
              (frame2[59] === 8'h00) && (frame2[42] === 8'h20);
        $display("  [阶段3] sn_vld=0：n2=%0d SN 字节@59=%02x（期望 00，对照阶段2 的 05）FCS 残差=%08x",
                 n2, frame2[59], resid2(0));
        $display("  [阶段3] 不带 SN 的帧（SN 字节恒 0）：%s", ok3 ? "PASS" : "FAIL");


        vlid=16'h0123; sport=16'h0BB8; dport=16'h0BB9; dip=32'hE0E00123;
        sn_vld=1'b1; neten=2'd0; plen_src=5'd8; mlen=11'd8;
        pay[0]=8'h41; pay[1]=8'h42; pay[2]=8'h43; pay[3]=8'h44;
        pay[4]=8'h45; pay[5]=8'h46; pay[6]=8'h47; pay[7]=8'h48;
        // 真正空闲（上一阶段的 B 副本发完）再发
        w = 0;
        while (!u_dut.idle_o && w < 200000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        capB = 1'b0; n4 = 0;
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        // 等 A 副本末字节（fr_eop && !fr_net），**当拍**切到"下一条消息"的配置
        w = 0;
        while (!(fv && feop && (fnet == 1'b0)) && w < 200000) begin @(posedge clk); w = w + 1; end
        $display("  [dbg4] A-eop wait w=%0d fv=%b feop=%b fnet=%b", w, fv, feop, fnet);
        n4 = 0; capB = 1'b1;
        vlid=16'h0BAD; sport=16'h1111; dport=16'h2222; dip=32'hE0E00BAD;
        w = 0;
        while (n4 < 64 && w < 200000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        capB = 1'b0;
        $display("  [dbg4] after B wait w=%0d n4=%0d", w, n4);
        ok4 = (pb4[4] === 8'h01) && (pb4[5] === 8'h23) &&
              (pb4[30] === 8'hE0) && (pb4[31] === 8'hE0) &&
              (pb4[32] === 8'h01) && (pb4[33] === 8'h23) &&
              (pb4[34] === 8'h0B) && (pb4[35] === 8'hB8) &&
              (pb4[36] === 8'h0B) && (pb4[37] === 8'hB9);
        $display("  [阶段4] B 头 = VLID %02x%02x DIP %02x%02x%02x%02x sport %02x%02x dport %02x%02x（期望 0123/E0E00123/0BB8/0BB9）",
                 pb4[4],pb4[5],pb4[30],pb4[31],pb4[32],pb4[33],pb4[34],pb4[35],pb4[36],pb4[37]);
        $display("  [阶段4] B 副本帧头用了本条消息的配置（不被下一条改写）: %s", ok4 ? "PASS" : "FAIL");


        vlid=16'h0123; sport=16'h0BB8; dport=16'h0BB9; dip=32'hE0E00123;
        sn_vld=1'b1; neten=2'd0; sn=8'h11; plen_src=5'd8; mlen=11'd8;
        pay[0]=8'h60; pay[1]=8'h61; pay[2]=8'h62; pay[3]=8'h63;
        pay[4]=8'h64; pay[5]=8'h65; pay[6]=8'h66; pay[7]=8'h67;
        w = 0;
        while (fv && w < 100000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        nA = 0; nB = 0; bp_en = 1;
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        w = 0;
        while (nB < 64 && w < 300000) begin @(posedge clk); w = w + 1; end
        bp_en = 0;
        repeat(4) @(posedge clk);
        begin : chk5
            integer i5; reg [31:0] okb;
            okb = 1;
            for (i5 = 0; i5 < 8; i5 = i5 + 1)
                if (frameA[42+i5] !== (8'h60 + i5[7:0]) ||
                    frameB[42+i5] !== (8'h60 + i5[7:0]) ||
                    frameB[42+i5] !== frameA[42+i5]) okb = 0;
            ok5 = (nA >= 64) && (nB >= 64) && (okb == 1) &&
                  (residA(0) == 32'hDEBB20E3) && (residB(0) == 32'hDEBB20E3);
            $display("  [阶段5] 反压下 B 载荷: nA=%0d nB=%0d A_FCS=%08x B_FCS=%08x 逐字节一致=%0d",
                     nA, nB, residA(0), residB(0), okb);
            $display("  [阶段5] MAC 反压下的 B 副本载荷逐字节（A/B 相等且 FCS 合法）: %s",
                     ok5 ? "PASS" : "FAIL");
        end


        vlid=16'h0123; sport=16'h0BB8; dport=16'h0BB9; dip=32'hE0E00123;
        sn_vld=1'b1; neten=2'd0; sn=8'h22; plen_src=5'd8; mlen=11'd8;
        pay[0]=8'h70; pay[1]=8'h71; pay[2]=8'h72; pay[3]=8'h73;
        pay[4]=8'h74; pay[5]=8'h75; pay[6]=8'h76; pay[7]=8'h77;
        w = 0;
        while (fv && w < 100000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        nA = 0; nB = 0; st41_done = 0; st41_en = 1;
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        w = 0;
        while (nB < 64 && w < 300000) begin @(posedge clk); w = w + 1; end
        st41_en = 0;
        repeat(4) @(posedge clk);
        begin : chk7
            integer i7; reg [31:0] ok7;
            ok7 = 1;
            for (i7 = 0; i7 < 8; i7 = i7 + 1)
                if (frameA[42+i7] !== (8'h70 + i7[7:0]) ||
                    frameB[42+i7] !== (8'h70 + i7[7:0])) ok7 = 0;
            ok7 = ok7 && (st41_done === 1'b1) &&
                  (nA >= 64) && (nB >= 64) &&
                  (residA(0) == 32'hDEBB20E3) && (residB(0) == 32'hDEBB20E3);
            $display("  [阶段7] pos=41 单拍反压: done=%b nA=%0d nB=%0d b42=%02x/%02x b49=%02x/%02x",
                     st41_done, nA, nB, frameA[42], frameB[42], frameA[49], frameB[49]);
            $display("  [阶段7] B 副本 pos==41 单拍反压下的载荷完整性: %s", ok7 ? "PASS" : "FAIL");
        end


        vlid=16'h0123; sport=16'h0BB8; dport=16'h0BB9; dip=32'hE0E00123;
        sn_vld=1'b1; sn=8'h22; neten=2'd3; plen_src=5'd8; mlen=11'd8;
        pay[0]=8'h70; pay[1]=8'h71; pay[2]=8'h72; pay[3]=8'h73;
        pay[4]=8'h74; pay[5]=8'h75; pay[6]=8'h76; pay[7]=8'h77;
        w = 0;
        while (!u_dut.idle_o && w < 200000) begin @(posedge clk); w = w + 1; end
        repeat(4) @(posedge clk);
        nA = 0; nB = 0;
        mv <= 1; msop <= 1; mdat <= pay[0]; meop <= 0; pcnt <= 0;
        w = 0;
        while ((nB < 64 || !u_dut.idle_o) && w < 300000) begin @(posedge clk); w = w + 1; end
        repeat(8) @(posedge clk);
        begin : chk6
            integer i6; reg okb6;
            okb6 = 1;
            for (i6 = 0; i6 < 8; i6 = i6 + 1)
                if (frameA[42+i6] !== (8'h70 + i6[7:0]) ||
                    frameB[42+i6] !== (8'h70 + i6[7:0])) okb6 = 0;
            ok6 = (nA >= 64) && (nB >= 64) && (okb6 == 1) && u_dut.idle_o &&
                  (residA(0) == 32'hDEBB20E3) && (residB(0) == 32'hDEBB20E3);
            $display("  [阶段6] net_en=3: nA=%0d nB=%0d idle=%b A_FCS=%08x B_FCS=%08x okb=%0d",
                     nA, nB, u_dut.idle_o, residA(0), residB(0), okb6);
            $display("  [阶段6] net_en=3 视同 0（B 从 pbuf 重放、不锁死）: %s", ok6 ? "PASS" : "FAIL");
        end

        $display("=================================");
        if (ok1 && ok2 && ok3 && ok4 && ok5 && ok6) $display(">> PASS: 帧格式 + MAC 反压下的字节完整性 + sn_vld=0 + net_en=3 全部正确");
        else            $display(">> FAIL: 阶段1=%0d 阶段2=%0d 阶段3=%0d 阶段4=%0d 阶段5=%0d 阶段6=%0d", ok1, ok2, ok3, ok4, ok5, ok6);
        $finish;
    end
endmodule
