// tb_mac_rxwfifo.v —— afdx_mac_rxwfifo 单元测试（纯 RTL，不依赖 MAC 门级网表）
`timescale 1ns/1ps

module tb_mac_rxwfifo;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    integer errs = 0, nchk = 0;
    task chk;
        input cond; input [255:0] name;
        begin
            nchk = nchk + 1;                       // 断言编号：控制台是 GBK，中文名会变 `?`
            if (cond) $display("  [ok]   #%0d %0s", nchk, name);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, name); end
        end
    endtask

    // ---------------- 生产深度实例（2048）----------------
    reg         in_vld = 0, in_sop = 0, in_eop = 0;
    reg  [7:0]  in_data = 0;
    wire        in_rdy;
    wire        out_vld, out_sop, out_eop;
    wire [7:0]  out_data;
    reg         out_rdy = 0;
    wire [11:0] used_big;

    afdx_mac_rxwfifo #(.DEPTH(2048), .AW(11)) u_big (
        .clk(clk), .reset(reset),
        .in_vld(in_vld), .in_sop(in_sop), .in_eop(in_eop), .in_data(in_data), .in_rdy(in_rdy),
        .out_vld(out_vld), .out_sop(out_sop), .out_eop(out_eop), .out_data(out_data),
        .out_rdy(out_rdy), .used(used_big)
    );

    // ---------------- 小深度实例（16，验证"满就反压、绝不丢"）----------------
    reg         s_vld = 0, s_sop = 0, s_eop = 0;
    reg  [7:0]  s_data = 0;
    wire        s_rdy;
    wire        s_out_vld;
    wire        s_out_sop, s_out_eop;
    wire [7:0]  s_out_data;
    reg         s_out_rdy = 0;
    wire [4:0]  used_small;

    afdx_mac_rxwfifo #(.DEPTH(16), .AW(4)) u_small (
        .clk(clk), .reset(reset),
        .in_vld(s_vld), .in_sop(s_sop), .in_eop(s_eop), .in_data(s_data), .in_rdy(s_rdy),
        .out_vld(s_out_vld), .out_sop(s_out_sop), .out_eop(s_out_eop), .out_data(s_out_data),
        .out_rdy(s_out_rdy), .used(used_small)
    );

    // 抓输出字节
    reg [7:0] cap [0:4095];
    integer   ncap = 0;
    reg [7:0] sc  [0:255];
    integer   nsc  = 0;

    function [7:0] pat; input integer k;
        pat = 8'h20 + k[7:0];
    endfunction

    integer k;

    // ---- 计数探针（case 08 用；用 always 块 + 使能，别用 fork/join）----
    reg       cnt8_en = 0;
    integer   nps8 = 0, npop8 = 0, nld8 = 0;
    always @(posedge clk) if (cnt8_en) begin
        if (u_big.push) nps8  = nps8  + 1;
        if (u_big.pop)  npop8 = npop8 + 1;
        if (u_big.load) nld8  = nld8  + 1;
    end


    task do_reset;
        begin
            reset = 1'b1;
            repeat (4) @(posedge clk);
            reset = 1'b0;
            repeat (4) @(posedge clk);
        end
    endtask

    initial begin
        #200 reset = 0;
        repeat(5) @(posedge clk);
        $display("");
        $display("======== afdx_mac_rxwfifo 单元测试（纯 RTL）========");

        // ================= 01 连续流：600 字节 1 字节/拍 =================
        ncap = 0;
        for (k = 0; k < 600; k = k + 1) begin
            @(negedge clk);
            out_rdy = 1'b1;                               // 本拍就取
            if (out_vld) begin
                cap[ncap] = out_data;
                if (out_sop) cap[ncap] = cap[ncap];        // sop/eop 位置另判（见下）
                ncap = ncap + 1;
            end
            in_vld = 1'b1; in_data = pat(k);
            in_sop = (k == 0); in_eop = (k == 599);
            @(posedge clk);
        end
        // 排空
        for (k = 0; k < 8; k = k + 1) begin
            @(negedge clk);
            out_rdy = 1'b1;
            if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
            in_vld = 1'b0; in_sop = 1'b0; in_eop = 1'b0;
            @(posedge clk);
        end
        chk(ncap === 600, "01a ★连续流：出来正好 600 个字节（不重复、不丢）");
        begin : c01
            integer q; integer bad;
            bad = 0;
            for (q = 0; q < 600; q = q + 1) if (cap[q] !== pat(q)) bad = bad + 1;
            chk(bad === 0, "01b ★连续流：逐字节一致（值 = 0x20+k）");
        end

        // ================= 02 深缓冲：消费者全程停读，灌 1500 字节 =================
        begin : c02
            integer mx, q;
            // 排空
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (5) @(posedge clk);
            mx = 0;
            for (q = 0; q < 1500; q = q + 1) begin
                @(negedge clk);
                out_rdy = 1'b0;
                in_vld = 1'b1; in_data = pat(q);
                in_sop = (q == 0); in_eop = (q == 1499);
                if (used_big > mx) mx = used_big;
                @(posedge clk);
            end
            @(negedge clk); in_vld = 1'b0; in_sop = 1'b0; in_eop = 1'b0;
            @(posedge clk);
            if (used_big > mx) mx = used_big;
            chk(mx >= 1499, "02a ★★停读期间 FIFO 攒下整整 1500 字节（旧 16 深实现只能到 16）");
            // 放开，逐字节校验
            ncap = 0;
            for (q = 0; q < 1520; q = q + 1) begin
                @(negedge clk);
                out_rdy = 1'b1;
                if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
                @(posedge clk);
            end
            chk(ncap === 1500, "02b ★★放开后正好取出 1500 个字节");
            begin : c02b
                integer r; integer bad2;
                bad2 = 0;
                for (r = 0; r < 1500; r = r + 1) if (cap[r] !== pat(r)) bad2 = bad2 + 1;
                chk(bad2 === 0, "02c ★★深缓冲后逐字节一致（没有重复/错位——这正是 2026-10-09 那次失败的症状）");
            end
        end

        // ================= 03 危险窗口①：空 FIFO + 本拍 push 且立刻取 =================
        begin : c03
            integer q; integer bad3;
            do_reset;
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (6) @(posedge clk);
            chk(used_big === 0, "03a 起手：FIFO 已空（对照）");
            ncap = 0;
            for (q = 0; q < 40; q = q + 1) begin
                @(negedge clk);
                out_rdy = 1'b1;                           // 边进边出（占用在 0/1 之间反复）
                if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
                in_vld = 1'b1; in_data = pat(q); in_sop = (q == 0); in_eop = (q == 39);
                @(posedge clk);
            end

            repeat (12) begin
                @(negedge clk); out_rdy = 1'b1; in_vld = 1'b0;
                if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
                @(posedge clk);
            end
            $display("WD 03 ncap=%0d used=%0d", ncap, used_big);
            chk(ncap === 40, "03b 危险窗口①：40 进 40 出");
            bad3 = 0;
            for (q = 0; q < 40; q = q + 1) if (cap[q] !== pat(q)) bad3 = bad3 + 1;
            chk(bad3 === 0, "03c ★★危险窗口①（读地址==写地址）不产生重复/旧值");
        end

        // ================= 04 危险窗口②：占用=1 时同拍 pop+push，反复 200 拍 =================
        begin : c04
            integer q; integer bad4;
            do_reset;
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (6) @(posedge clk);

            for (q = 0; q < 4; q = q + 1) begin
                @(negedge clk); in_vld = 1'b1; in_data = pat(q);
                in_sop = (q == 0); in_eop = 1'b0;
                @(posedge clk);
            end
            @(negedge clk); in_vld = 1'b0;
            repeat (6) @(posedge clk);
            $display("WD 04 start used=%0d", used_big);
            chk(used_big === 4, "04a 起手：占用正好 = 4（一个字）");
            ncap = 0; k = 0;
            // 流内容 = pat(0..203) 连续（起手 4 个 + 后面 200 个）
            for (q = 0; q < 200; q = q + 1) begin
                @(negedge clk);
                out_rdy = 1'b1;                           // 同拍：取一个
                if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
                in_vld = 1'b1; in_data = pat(q+4);        // 同拍：再灌一个（接在起手 4 个之后）
                in_sop = 1'b0; in_eop = (q == 199);
                                                          //   否则尾部那个"不满的字"会留在装配寄存器里
                @(posedge clk);
            end
            $display("WD 04 preDrain ncap=%0d used=%0d v0=%0d vld=%0d rdy=%0d bi=%0d cnt=%0d",
                     ncap, used_big, u_big.v0, out_vld, out_rdy, u_big.bi, u_big.q0[35:34]);

            repeat (12) begin
                @(negedge clk); in_vld = 1'b0; out_rdy = 1'b1;
                $display("WD 04 drain: vld=%0d rdy=%0d data=%02x used=%0d bi=%0d",
                         out_vld, out_rdy, out_data, used_big, u_big.bi);
                if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
                @(posedge clk);
            end
            $display("  [dbg] 04 ncap=%0d used=%0d", ncap, used_big);
            $display("WD 04 ncap=%0d used=%0d ub=%0d rc=%0d wn=%0d v0=%0d bi=%0d",
                     ncap, used_big, u_big.ub, u_big.rc, u_big.wn, u_big.v0, u_big.bi);
            chk(ncap === 204, "04b 危险窗口②：灌了 204 个、正好取出 204 个");
            bad4 = 0;
            for (q = 0; q < 204; q = q + 1) if (cap[q] !== pat(q)) bad4 = bad4 + 1;
            chk(bad4 === 0, "04c ★★危险窗口②（读地址==写地址）不产生重复/旧值");
        end

        // ================= 05 SFD 剥离 =================
        begin : c05
            integer q;
            do_reset;
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (6) @(posedge clk);
            // 送一"帧"：0xD5（SFD）→ 03 00 00 …（真帧首）
            @(negedge clk); in_vld = 1'b1; in_data = 8'hD5; in_sop = 1'b1; in_eop = 1'b0;
            @(posedge clk);
            @(negedge clk); in_data = 8'h03; in_sop = 1'b1; in_eop = 1'b0;   // MAC 的 sop 在整个帧首拍
            @(posedge clk);
            for (q = 0; q < 4; q = q + 1) begin
                @(negedge clk); in_data = 8'h10 + q[7:0]; in_sop = 1'b0;
                in_eop = (q == 3);
                @(posedge clk);
            end
            @(negedge clk); in_vld = 1'b0; in_sop = 1'b0; in_eop = 1'b0;
            @(posedge clk);
            $display("WD 05 used=%0d", used_big);
            chk(used_big === 5, "05a SFD 被丢掉：5 个进来 → 只剩 5 个（不含 0xD5）");
            // 取出来看边界
            ncap = 0;
            for (q = 0; q < 8; q = q + 1) begin
                @(negedge clk); out_rdy = 1'b1;
                if (out_vld) begin
                    cap[ncap] = out_data;
                    if (ncap == 0) chk(out_sop === 1'b1, "05b ★★SFD 之后那一个字节带 sop");
                    if (ncap == 4) chk(out_eop === 1'b1, "05c 帧末字节带 eop");
                    ncap = ncap + 1;
                end
                @(posedge clk);
            end
            $display("WD 05 ncap=%0d", ncap);
            chk(ncap === 5, "05d 取出 5 个字节");
            chk(cap[0] === 8'h03, "05e 第一个字节是真帧首 0x03（不是 0xD5）");
        end

        // ================= 06 小深度（16）：满了反压，一个字节都不能丢 =================
        begin : c06
            integer q; integer nsc_l;
            s_out_rdy = 1'b0; s_vld = 1'b0;
            repeat (6) @(posedge clk);
            nsc = 0;
            // 灌到满：in_rdy 应在第 16 个之后落下
            for (q = 0; q < 16; q = q + 1) begin
                @(negedge clk);
                s_vld = 1'b1; s_data = pat(q); s_sop = (q == 0); s_eop = 1'b0;
                @(posedge clk);
            end
            @(negedge clk); s_vld = 1'b0;
            @(posedge clk);
            chk(used_small === 16, "06a 小深度实例：正好装满 16 个");
            chk(s_rdy === 1'b0,    "06b ★满了必须拉低 in_rdy（反压 MAC，而不是丢字节）");
            // 再"想灌"10 次但 in_rdy=0 ⇒ 一个都不该进去
            for (q = 0; q < 10; q = q + 1) begin
                @(negedge clk); s_vld = 1'b1; s_data = 8'hEE; @(posedge clk);
            end
            @(negedge clk); s_vld = 1'b0;
            @(posedge clk);
            chk(used_small === 16, "06c ★满了以后灌不进去（占用仍是 16，没有覆盖/溢出）");
            // 放开全取出来
            for (q = 0; q < 40; q = q + 1) begin
                @(negedge clk); s_out_rdy = 1'b1;
                if (s_out_vld) begin sc[nsc] = s_out_data; nsc = nsc + 1; end
                @(posedge clk);
            end
            chk(nsc === 16, "06d 取出正好 16 个");
            begin : c06b
                integer r; integer bad6;
                bad6 = 0;
                for (r = 0; r < 16; r = r + 1) if (sc[r] !== pat(r)) bad6 = bad6 + 1;
                chk(bad6 === 0, "06e ★小深度下也逐字节一致（满 + 反压不会打乱顺序）");
            end
            nsc_l = 0;   // 占位（保持 VB 的"先声明后用"整洁）
        end


        begin : c07
            integer q; integer bad7;
            do_reset;
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (6) @(posedge clk);
            // 帧 A = 5 字节（尾字 1 字节），帧 B = 8 字节
            for (q = 0; q < 5; q = q + 1) begin
                @(negedge clk); in_vld = 1'b1; in_data = pat(q);
                in_sop = (q == 0); in_eop = (q == 4);
                @(posedge clk);
            end
            for (q = 0; q < 8; q = q + 1) begin
                @(negedge clk); in_vld = 1'b1; in_data = pat(100 + q);
                in_sop = (q == 0); in_eop = (q == 7);
                @(posedge clk);
            end
            @(negedge clk); in_vld = 1'b0; in_sop = 1'b0; in_eop = 1'b0;
            @(posedge clk);
            // 放开、快速排空
            ncap = 0;
            for (q = 0; q < 64; q = q + 1) begin
                @(negedge clk); out_rdy = 1'b1;
                if (out_vld) begin cap[ncap] = out_data; ncap = ncap + 1; end
                @(posedge clk);
            end
            chk(ncap === 13, "07a ★1 字节尾字 + 后继字：正好 13 个字节出来（不重复、不丢）");
            bad7 = 0;
            for (q = 0; q < 5;  q = q + 1) if (cap[q]     !== pat(q))       bad7 = bad7 + 1;
            for (q = 0; q < 8;  q = q + 1) if (cap[5+q]   !== pat(100 + q)) bad7 = bad7 + 1;
            chk(bad7 === 0, "07b ★★逐字节一致（两帧边界上不重复、不丢字）");
        end


        begin : c08
            integer q; integer bad8; integer ndrop8;
            do_reset;
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (6) @(posedge clk);
            ncap = 0; bad8 = 0; ndrop8 = 0; nps8 = 0; npop8 = 0; nld8 = 0;
            cnt8_en = 1;

            for (q = 0; q < 3000; q = q + 1) begin
                @(negedge clk);
                out_rdy = 1'b1;
                if (out_vld) begin
                    if (out_data !== pat(ncap)) begin
                        bad8 = bad8 + 1;
                    end
                    cap[ncap] = out_data;
                    ncap = ncap + 1;
                end
                in_vld = 1'b1; in_data = pat(q);
                in_sop = (q == 0); in_eop = (q == 2999);
                // 探针：FIFO 说满（in_rdy=0）而我们还在灌 ⇒ 这一拍会被丢掉
                if (!in_rdy) ndrop8 = ndrop8 + 1;
                @(posedge clk);
            end

            @(negedge clk); in_vld = 1'b0; in_sop = 1'b0; in_eop = 1'b0; out_rdy = 1'b1;
            if (out_vld) begin
                if (out_data !== pat(ncap)) bad8 = bad8 + 1;
                cap[ncap] = out_data; ncap = ncap + 1;
            end
            // 排空：**排到空为止**（延迟会变，不能用固定拍数）
            q = 0;
            while ((used_big != 0) && (q < 8000)) begin
                @(negedge clk); out_rdy = 1'b1;
                if (out_vld) begin
                    if (out_data !== pat(ncap)) begin
                        bad8 = bad8 + 1;
                    end
                    cap[ncap] = out_data; ncap = ncap + 1;
                end
                @(posedge clk);
                q = q + 1;
            end
            $display("WD08 ndrop8=%0d push=%0d pop=%0d load=%0d", ndrop8, nps8, npop8, nld8);
            $display("WD08 ncap=%0d bad8=%0d used=%0d ub=%0d rc=%0d v0=%0d wn=%0d wp=%0d ra=%0d",
                     ncap, bad8, used_big, u_big.ub, u_big.rc, u_big.v0, u_big.wn, u_big.wp, u_big.ra);
            chk(ncap === 3000, "08a ★★回绕压力：3000 进 3000 出（指针回绕多圈不丢字节）");
            chk(bad8 === 0,    "08b ★★回绕压力：逐字节与源序列一致（没有读到上一圈的旧字）");
            chk(used_big === 0, "08c 排空后占用归零（ub 没有下溢/漂移）");
        end


        begin : c09
            integer q; integer nacc9;
            do_reset;
            s_out_rdy = 1'b0; s_vld = 1'b0;
            repeat (6) @(posedge clk);
            nacc9 = 0;
            for (q = 0; q < 4; q = q + 1) begin        // 4 个 2 字节帧 = 4 个字 = 正好装满
                @(negedge clk); s_vld = 1'b1; s_data = pat(2*q);
                s_sop = 1'b1; s_eop = 1'b0; @(posedge clk);
                @(negedge clk); s_vld = 1'b1; s_data = pat(2*q+1);
                s_sop = 1'b0; s_eop = 1'b1; @(posedge clk);
                nacc9 = nacc9 + 2;
            end
            @(negedge clk); s_vld = 1'b0;
            @(posedge clk);
            chk(used_small === 8, "09a 4 个 2 字节帧 = 8 字节（只占 8/16 字节）");
            chk(s_rdy === 1'b0,   "09b ★★按**字**判满：4 个字装满后必须反压（按字节判会以为还空一半）");
            // 再想灌一帧：必须被挡住
            @(negedge clk); s_vld = 1'b1; s_data = 8'hEE; s_sop = 1'b1; s_eop = 1'b0;
            @(posedge clk);
            @(negedge clk); s_vld = 1'b0; @(posedge clk);
            chk(used_small === 8, "09c ★★装满后灌不进去（没有覆盖未读数据）");
            nsc = 0;
            for (q = 0; q < 16; q = q + 1) begin
                @(negedge clk); s_out_rdy = 1'b1;
                if (s_out_vld) begin sc[nsc] = s_out_data; nsc = nsc + 1; end
                @(posedge clk);
            end
            chk(nsc === 8, "09d 取出正好 8 个字节");
        end


        begin : c10
            integer q;
            do_reset;
            out_rdy = 1'b0; in_vld = 1'b0;
            repeat (6) @(posedge clk);
            for (q = 0; q < 3; q = q + 1) begin       // 3 个字节、**不带 eop**（停在装配寄存器里）
                @(negedge clk); in_vld = 1'b1; in_data = pat(q);
                in_sop = (q == 0); in_eop = 1'b0; @(posedge clk);
            end
            @(negedge clk); in_vld = 1'b0;
            @(posedge clk);
            chk(used_big === 3, "10 ★装配寄存器里的 3 个字节也要计入 used（原来报 0）");
        end


        begin : c11
            integer j, it, sop11, eop11, bad11;
            do_reset;
            nsc = 0; s_out_rdy = 1'b0; s_vld = 1'b0; s_sop = 1'b0; s_eop = 1'b0;
            // --- 前半段：消费者停读，灌 16 字节（= 4 个字 = 满）---
            for (j = 0; j < 16; j = j + 1) begin
                @(negedge clk);
                s_vld = 1'b1; s_data = 8'h80 + j[7:0];
                s_sop = (j == 0); s_eop = 1'b0;
                @(posedge clk);
            end

            @(negedge clk); s_vld = 1'b0;
            chk(s_rdy === 1'b0,     "11a ★帧中途撞满：4 个字灌满后必须反压");
            chk(used_small === 16,  "11b ★帧中途撞满：占用=16（没有覆盖未读数据）");
            // --- 后半段：边取边灌，直到帧尾 30 字节全部送完、也全部取完 ---
            j = 16; it = 0;
            while (((j < 30) || (nsc < 30)) && (it < 400)) begin
                @(negedge clk);
                s_out_rdy = 1'b1;
                if (s_out_vld) begin sc[nsc] = s_out_data; nsc = nsc + 1; end
                s_vld = (j < 30); s_data = 8'h80 + j[7:0];
                s_sop = (j == 0); s_eop = (j == 29);
                if (s_vld && s_rdy) j = j + 1;
                @(posedge clk);
                it = it + 1;
            end
            // --- 收尾：把余下的取干净 ---
            for (it = 0; it < 40; it = it + 1) begin
                @(negedge clk);
                s_vld = 1'b0; s_out_rdy = 1'b1;
                if (s_out_vld) begin sc[nsc] = s_out_data; nsc = nsc + 1; end
                @(posedge clk);
            end
            @(negedge clk); s_out_rdy = 1'b0;
            chk(nsc === 30, "11c ★帧中途撞满：30 进 30 出（不丢字节）");
            bad11 = 0;
            for (it = 0; it < 30; it = it + 1) if (sc[it] !== 8'h80 + it[7:0]) bad11 = bad11 + 1;
            chk(bad11 === 0, "11d ★★逐字节一致（帧中途停摆再续，没有错位/重复）");
            chk(used_small === 0, "11e 排空后占用归零（ub 没有下溢/漂移）");
        end


        begin : c12
            integer j, it, eop12, n_eop;
            do_reset;
            nsc = 0; s_out_rdy = 1'b0; s_vld = 1'b0; s_sop = 1'b0; s_eop = 1'b0;
            for (j = 0; j < 16; j = j + 1) begin
                @(negedge clk);
                s_vld = 1'b1; s_data = 8'h40 + j[7:0];
                s_sop = (j == 0); s_eop = (j == 15);
                @(posedge clk);
            end
            @(negedge clk); s_vld = 1'b0;                  // 同上：negedge 才能读到本拍结果
            chk(s_rdy === 1'b0 && used_small === 16,
                "12a ★带 eop 的那一字节正好把 FIFO 灌满");
            // 取干净，逐字节判 + 数 eop/最后一个字节
            n_eop = 0;
            for (it = 0; it < 60; it = it + 1) begin
                @(negedge clk);
                s_vld = 1'b0; s_out_rdy = 1'b1;
                if (s_out_vld) begin sc[nsc] = s_out_data; nsc = nsc + 1;
                                    if (s_out_eop) n_eop = n_eop + 1; end
                @(posedge clk);
            end
            @(negedge clk); s_out_rdy = 1'b0;
            chk(nsc === 16, "12b ★满着收尾的帧：16 进 16 出");
            chk(n_eop === 1 && sc[15] === 8'h4F, "12c ★★帧尾 eop 没有丢（正好在最后一个字节上）");
            chk(used_small === 0, "12d 排空后占用归零");
        end

        $display("====================================================");
        if (errs == 0) $display(">> PASS: 接收 FIFO（深缓冲/SFD/危险窗口/满反压/1字节尾字/回绕/按字判满/帧中途撞满）全部正确");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #2000000;                                       // 兜底：2 ms 内必须跑完
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
