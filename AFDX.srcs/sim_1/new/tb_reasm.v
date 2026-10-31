// tb_reasm.v —— afdx_reasm（IP 分片重组）单元测试
`timescale 1ns/1ps

module tb_reasm;
    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    reg         in_valid = 0, in_sop = 0, in_eop = 0;
    reg  [7:0]  in_data = 0;
    reg  [15:0] in_vlid = 0;
    reg         in_desc = 0, in_frag = 0, in_mf = 0;
    reg  [12:0] in_foff = 0;
    reg  [15:0] in_ipid = 0;
    reg         in_ok = 1, in_accept = 0, in_abort = 0, in_dup = 0;
    reg         in_net = 0;

    reg  [2:0]  in_port = 3'd0;
    reg  [15:0] in_dport = 16'd0;
    reg  [31:0] in_sip = 32'd0;
    reg  [15:0] in_sport = 16'd0;

    wire        out_valid, out_sop, out_eop, out_commit, out_abort;
    wire [7:0]  out_data;
    wire [15:0] out_vlid;
    wire [15:0] ev_reasm_ok, ev_reasm_err;
    wire [2:0]  out_port;
    wire [15:0] out_dport, out_sport;
    wire [31:0] out_sip;
    wire        out_net;
    wire        out_vbytes;


    afdx_reasm #(.AW(11), .TMO_CYC(20000)) dut (
        .clk(clk), .reset(reset),
        .in_valid(in_valid), .in_sop(in_sop), .in_eop(in_eop), .in_data(in_data),
        .in_vlid(in_vlid), .in_net(in_net),
        .in_desc(in_desc), .in_frag(in_frag), .in_mf(in_mf), .in_foff(in_foff),
        .in_ipid(in_ipid), .in_ok(in_ok), .in_accept(in_accept), .in_abort(in_abort), .in_dup(in_dup),
        .in_port(in_port), .in_dport(in_dport), .in_sip(in_sip), .in_sport(in_sport),
        .out_valid(out_valid), .out_sop(out_sop), .out_eop(out_eop), .out_data(out_data),
        .out_vlid(out_vlid), .out_commit(out_commit), .out_abort(out_abort),
        .out_vbytes(out_vbytes),
        .out_port(out_port), .out_net(out_net),
        .out_dport(out_dport), .out_sip(out_sip), .out_sport(out_sport),
        .ev_reasm_ok(ev_reasm_ok), .ev_reasm_err(ev_reasm_err)
    );


    reg eop_wo_sop = 0, oseg = 0;
    always @(posedge clk) begin
        if (out_valid) begin
            if (out_sop) oseg <= 1'b1;
            if (out_eop && !out_sop && !oseg) eop_wo_sop <= 1'b1;
            if (out_eop) oseg <= 1'b0;
        end
    end

    // 输出抓取
    reg [7:0] cap [0:16383];
    reg [2:0] cap_p [0:16383];
    integer   ncap = 0, ncommit = 0, nsop = 0, neop = 0, nabort = 0;
    integer   nabort_vb = 0;
    reg       net_sop = 1'b0;
    always @(posedge clk) begin
        if (out_valid) begin
            cap[ncap] = out_data; cap_p[ncap] = out_port; ncap = ncap + 1;
            if (out_sop) begin nsop = nsop + 1; net_sop = out_net; end
            if (out_eop)  neop  = neop  + 1;
        end
        if (out_commit) ncommit = ncommit + 1;
        if (out_abort)  nabort  = nabort  + 1;
        if (out_abort && out_vbytes) nabort_vb = nabort_vb + 1;
    end

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    // 载荷模式：第 block 块的载荷第 i 字节
    function [7:0] pat; input integer block; input integer i;
        begin pat = (block * 37 + i) & 8'hFF; end
    endfunction

    // 发一"帧"：n 字节载荷 + 帧末描述符 —— **发完立即返回，不等回放跑完**
    //   （并发/重叠类用例必须用它；顺序用例用下面的 send_frame 包装）
    task send_frame_x;
        input        frag, mf;
        input [12:0] foff;
        input [15:0] ipid;
        input [15:0] vlid;
        input        acc;
        input integer n;
        input integer block;
        integer i;
        begin
            in_frag = frag; in_mf = mf; in_foff = foff; in_ipid = ipid; in_vlid = vlid;
            in_ok = 1;
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk);
                in_valid = 1; in_sop = (i == 0); in_eop = (i == n-1);
                in_data  = pat(block, i);
                @(posedge clk);
            end
            @(negedge clk); in_valid = 0; in_sop = 0; in_eop = 0;
            // 帧末描述符
            @(negedge clk); in_desc = 1; in_ok = 1;
            @(posedge clk); @(negedge clk); in_desc = 0;

            repeat (3) @(posedge clk);
            @(negedge clk); in_accept = acc; in_abort = ~acc;
            @(posedge clk); @(negedge clk); in_accept = 0; in_abort = 0;
        end
    endtask


    task send_dup_frame;
        input        frag, mf;
        input [12:0] foff;
        input [15:0] ipid, vlid;
        input integer n, block;
        integer i;
        begin
            in_frag = frag; in_mf = mf; in_foff = foff; in_ipid = ipid; in_vlid = vlid;
            in_ok = 1;
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk);
                in_valid = 1; in_sop = (i == 0); in_eop = (i == n-1);
                in_data  = pat(block, i);
                @(posedge clk);
            end
            @(negedge clk); in_valid = 0; in_sop = 0; in_eop = 0;
            @(negedge clk); in_desc = 1;
            @(posedge clk); @(negedge clk); in_desc = 0;
            repeat (3) @(posedge clk);
            @(negedge clk); in_abort = 1; in_dup = 1;
            @(posedge clk); @(negedge clk); in_abort = 0; in_dup = 0;
            repeat (8) @(posedge clk);
        end
    endtask


    task send_frame;
        input        frag, mf;
        input [12:0] foff;
        input [15:0] ipid;
        input [15:0] vlid;
        input        acc;
        input integer n;
        input integer block;
        begin
            send_frame_x(frag, mf, foff, ipid, vlid, acc, n, block);
            repeat (128) @(posedge clk);
        end
    endtask

    task clear_cap; begin ncap = 0; ncommit = 0; nsop = 0; neop = 0; nabort = 0; end endtask


    task send_bytes_desc;
        input        frag, mf;
        input [12:0] foff;
        input [15:0] ipid, vlid;
        input integer n, block;
        integer i;
        begin
            in_frag = frag; in_mf = mf; in_foff = foff; in_ipid = ipid; in_vlid = vlid;
            in_ok = 1;
            for (i = 0; i < n; i = i + 1) begin
                @(negedge clk);
                in_valid = 1; in_sop = (i == 0); in_eop = (i == n-1);
                in_data  = pat(block, i);
                @(posedge clk);
            end
            @(negedge clk); in_valid = 0; in_sop = 0; in_eop = 0;
            @(negedge clk); in_desc = 1; in_ok = 1;
            @(posedge clk); @(negedge clk); in_desc = 0;
        end
    endtask

    task do_judge;
        input acc;
        begin
            repeat (3) @(posedge clk);
            @(negedge clk); in_accept = acc; in_abort = ~acc;
            @(posedge clk); @(negedge clk); in_accept = 0; in_abort = 0;
        end
    endtask

    // 检查抓到的内容是否等于"若干块按顺序拼起来"
    //   这里做简化：逐字节对照期望函数
    integer k, m, okn;
    reg cok;
    task check_concat;
        input integer nblk;
        input [4095:0] what;     // 占位（Verilog 没有好办法传数组，桩）
        begin end
    endtask

    integer i;
    initial begin
        #200 reset = 1'b0; repeat (4) @(posedge clk);
        $display("");
        $display("========== afdx_reasm（IP 分片重组）==========");

        // ---------------------------------------------------------------
        // T1 非分片：原样透传
        // ---------------------------------------------------------------
        clear_cap;
        send_frame(1'b0, 1'b0, 13'd0, 16'hAAAA, 16'h0123, 1'b1, 8, 1);
        chk(ncap === 8,        "T1a 非分片：8 字节原样输出");
        chk(ncommit === 1 && nsop === 1 && neop === 1, "T1b 非分片：一次 commit / 一个 sop / 一个 eop");
        chk(cap[0] === pat(1,0) && cap[7] === pat(1,7), "T1c 非分片：内容逐字节一致");
        chk(net_sop === 1'b0, "T1d ★非分片透传：网络号随帧传递（A=0）");


        clear_cap;
        in_net = 1'b1;                                            // 首片来自 B 网
        send_frame(1'b1, 1'b1, 13'd0, 16'h1234, 16'h0456, 1'b1,  8, 2);   // 片1（8 字节数据）
        in_net = 1'b0;                                            // 末片故意给 A：必须用**首片**的 net
        send_frame(1'b1, 1'b0, 13'd2, 16'h1234, 16'h0456, 1'b1, 12, 3);   // 片2（末片，12 字节）
        chk(ncap === 8+12,     "T2a ★2 片重组：输出 20 字节（8 + 12）");
        chk(ncommit === 1,     "T2b ★只发**一次** commit（不是每片一次）");
        chk(nsop === 1 && neop === 1, "T2c 一个 sop / 一个 eop（整条数据报一条消息）");
        chk(cap[0] === pat(2,0) && cap[7] === pat(2,7),
            "T2d ★前 8 字节 = 片1 的 8 个数据字节（UDP 头在上游已被剥掉）");
        chk(cap[8] === pat(3,0) && cap[19] === pat(3,11), "T2e 后 12 字节来自片2（接在片1 之后）");
        chk(ev_reasm_ok === 16'd1, "T2f 重组成功计数 = 1");
        chk(net_sop === 1'b1, "T2g ★重组消息的网络号 = **首片**的 B 网（不是末片的 A）");


        clear_cap;
        send_frame_x(1'b1, 1'b1, 13'd0, 16'h7777, 16'h0AA1, 1'b1,  8, 20);  // 片1 (A) accept
        repeat (8) @(posedge clk);
        send_dup_frame(1'b1, 1'b1, 13'd0, 16'h7777, 16'h0AA1, 8, 20);       // 片1 (B) dup
        send_frame_x(1'b1, 1'b0, 13'd2, 16'h7777, 16'h0AA1, 1'b1,  6, 21);  // 片2 (A) 末片 accept
        repeat (400) @(posedge clk);
        $display("  [dbg] T2b ncap=%0d ncommit=%0d ev_ok=%0d", ncap, ncommit, ev_reasm_ok);
        chk(ncap === 14, "T2b dup copy keeps reassembly");
        chk(ncommit === 1 && nsop === 1 && neop === 1, "T2c dup: one commit");
        chk(cap[0] === pat(20,0) && cap[8] === pat(21,0), "T2d dup: payload correct");


        clear_cap;
        send_frame_x(1'b1, 1'b1, 13'd0, 16'h9001, 16'h0AA2, 1'b1, 8, 30);   // Q 片1
        repeat (8) @(posedge clk);

        send_dup_frame(1'b1, 1'b0, 13'd1, 16'h9002, 16'h0AA2, 8, 31);       // 旧 P 的片（dup，foff=1）
        send_frame_x(1'b1, 1'b0, 13'd2, 16'h9001, 16'h0AA2, 1'b1, 6, 32);   // Q 片2（末片）
        repeat (400) @(posedge clk);
        $display("  [dbg] T2e ncap=%0d cap8=%02x exp=%02x", ncap, cap[8], pat(32,0));
        chk(ncap === 14, "T2e old frag does not kill assembly");
        chk(cap[0] === pat(30,0) && cap[8] === pat(32,0) && cap[13] === pat(32,5),
            "T2f Q bytes intact");


        clear_cap;
        begin : t2g
            integer e0, n0;
            e0 = ev_reasm_err; n0 = ncommit;
            send_frame_x(1'b1, 1'b1, 13'd0, 16'hA001, 16'h0AA3, 1'b1, 8, 40);  // Q 片1
            repeat (8) @(posedge clk);
            send_dup_frame(1'b1, 1'b1, 13'd0, 16'hA002, 16'h0AA3, 8, 41);      // 旧 P 的首片（dup）
            send_frame_x(1'b1, 1'b0, 13'd2, 16'hA001, 16'h0AA3, 1'b1, 6, 42);  // Q 片2
            repeat (400) @(posedge clk);
            $display("  [dbg] T2g ncap=%0d err=%0d", ncap, ev_reasm_err-e0);
            chk(ncap === 0 && ncommit === n0, "T2g stale first frag kills asm");
            chk(ev_reasm_err > e0, "T2h stale first frag is visible");
        end


        clear_cap;
        send_frame(1'b1, 1'b1, 13'd0, 16'h2222, 16'h0789, 1'b1, 16, 4);
        send_frame(1'b1, 1'b1, 13'd3, 16'h2222, 16'h0789, 1'b1, 16, 5);
        send_frame(1'b1, 1'b0, 13'd5, 16'h2222, 16'h0789, 1'b1,  6, 6);
        chk(ncap === 16+16+6,  "T3a ★3 片重组：输出 38 字节（16+16+6）");
        chk(ncommit === 1,     "T3b 仍然只发一次 commit");
        chk(cap[0] === pat(4,0) && cap[15] === pat(4,15), "T3c 第 1 段来自片1（16 字节数据）");
        chk(cap[16] === pat(5,0) && cap[31] === pat(5,15), "T3d 第 2 段接在后面");
        chk(cap[32] === pat(6,0) && cap[37] === pat(6,5),  "T3e 末段（短片）也对");
        chk(ev_reasm_ok === 16'd4, "T3f 重组成功计数累到 4");   // T2f=1 → T2b=2 → T2e=3 → 本用例=4

        // ---------------------------------------------------------------
        // T4 乱序：片2 的偏移跳空 → 整条作废
        // ---------------------------------------------------------------
        clear_cap;
        begin : t4
            integer e0;
            e0 = ev_reasm_err;
            send_frame(1'b1, 1'b1, 13'd0, 16'h3333, 16'h0456, 1'b1, 16, 7);
            send_frame(1'b1, 1'b0, 13'd5, 16'h3333, 16'h0456, 1'b1,  8, 8);  // 偏移跳空
            chk(ncap === 0 && ncommit === 0, "T4a ★乱序片：一个字节都不输出、不发 commit");
            chk(ev_reasm_err === e0 + 1,     "T4b 计入 ev_reasm_err");
        end

        // ---------------------------------------------------------------
        // T5 ipid 不匹配 → 整条作废
        // ---------------------------------------------------------------
        clear_cap;
        begin : t5
            integer e0;
            e0 = ev_reasm_err;
            send_frame(1'b1, 1'b1, 13'd0, 16'h4444, 16'h0456, 1'b1, 16, 9);
            send_frame(1'b1, 1'b0, 13'd2, 16'h5555, 16'h0456, 1'b1,  8, 10);  // ipid 不同
            chk(ncap === 0 && ncommit === 0, "T5a ★ipid 不匹配：整条作废");
            chk(ev_reasm_err === e0 + 1,     "T5b 计入 ev_reasm_err");
        end

        // ---------------------------------------------------------------
        // T6 中间片被 IC/RM 丢弃 → 整条作废（不能交出半条）
        // ---------------------------------------------------------------
        clear_cap;
        begin : t6
            integer e0;
            e0 = ev_reasm_err;
            send_frame(1'b1, 1'b1, 13'd0, 16'h6666, 16'h0789, 1'b1, 16, 11);
            send_frame(1'b1, 1'b1, 13'd2, 16'h6666, 16'h0789, 1'b0, 16, 12);  // 被丢
            send_frame(1'b1, 1'b0, 13'd4, 16'h6666, 16'h0789, 1'b1,  8, 13);
            chk(ncap === 0 && ncommit === 0, "T6a ★中间片被丢：整条作废");
            chk(ev_reasm_err >= e0 + 1,      "T6b 计入 ev_reasm_err");
        end

        // ---------------------------------------------------------------
        // T7 下一条数据报能接着做（缓冲轮换正常）
        // ---------------------------------------------------------------
        clear_cap;
        send_frame(1'b1, 1'b1, 13'd0, 16'h7777, 16'h0123, 1'b1,  8, 14);   // 8 数据字节 → 占缓冲 0..7
        send_frame(1'b1, 1'b0, 13'd2, 16'h7777, 16'h0123, 1'b1, 16, 15);   // 起点 16-8=8 ✓ 接上
        chk(ncap === 8+16 && ncommit === 1, "T7a 作废之后的下一条照样能重组（8 + 16 = 24 字节）");
        chk(cap[0] === pat(14,0) && cap[8] === pat(15,0), "T7b 内容正确（片1 的 8 字节在前，片2 接上）");

        // ---------------------------------------------------------------
        // T8 非分片与分片混着来
        // ---------------------------------------------------------------
        clear_cap;
        send_frame(1'b0, 1'b0, 13'd0, 16'h8888, 16'h0456, 1'b1, 4, 16);      // 非分片
        send_frame(1'b1, 1'b1, 13'd0, 16'h9999, 16'h0456, 1'b1,  8, 17);     // 片1：8 数据字节
        send_frame(1'b0, 1'b0, 13'd0, 16'hAAAA, 16'h0456, 1'b1, 4, 18);      // 非分片
        send_frame(1'b1, 1'b0, 13'd2, 16'h9999, 16'h0456, 1'b1, 12, 19);     // 片2：起点 16-8=8 ✓
        chk(ncap === 4+4+8+12, "T8a 混着来：4 + 4 + (8+12) 字节");
        chk(ncommit === 3,     "T8b 三次 commit（两条非分片 + 一条重组）");

        // ---------------------------------------------------------------
        // T9 非分片被 IC/RM 丢弃 → 原样透传 abort
        // ---------------------------------------------------------------
        clear_cap;
        send_frame(1'b0, 1'b0, 13'd0, 16'hBBBB, 16'h0456, 1'b0, 4, 20);
        chk(nabort === 1 && ncommit === 0, "T9 非分片被丢 → 透传 abort");


        clear_cap;
        begin : t11
            integer e0;
            e0 = ev_reasm_err;
            send_frame(1'b1, 1'b1, 13'd0, 16'hC0DE, 16'h0456, 1'b1, 8200, 20);
            chk(ncap === 0 && ncommit === 0, "T11a ★超 8 KB 的片：一个字节都不输出（没有回绕覆写）");
            chk(ev_reasm_err >= e0 + 1, "T11b 计入 ev_reasm_err");
        end
        // T11c 对照：8 KB 以内仍然正常
        clear_cap;
        send_frame(1'b1, 1'b1, 13'd0, 16'hC0DF, 16'h0456, 1'b1, 1000, 21);
        send_frame(1'b1, 1'b0, 13'd126, 16'hC0DF, 16'h0456, 1'b1, 1000, 22);

        repeat(2500) @(posedge clk);
        chk(ncap === 2000 && ncommit === 1, "T11c 对照：2 KB 的两片仍能正常重组（2000 字节）");


        clear_cap;
        begin : t12
            integer e0, a0;
            e0 = ev_reasm_err; a0 = nabort;
            // 造一条 200 字节的重组消息（片1 = 96B@off0，片2 = 104B@off13 → 13*8-8 = 96 ✓）
            send_frame (1'b1, 1'b1, 13'd0,  16'hD001, 16'h0456, 1'b1, 96,  21);
            send_frame_x(1'b1, 1'b0, 13'd13, 16'hD001, 16'h0456, 1'b1, 104, 22);
            // 此刻回放刚开始（200 字节 ≈ 50 字 × 5 拍 = 250 拍）→ 立刻塞一条普通帧进去
            send_frame_x(1'b0, 1'b0, 13'd0, 16'hD002, 16'h0456, 1'b1, 8, 30);
            repeat (500) @(posedge clk);          // 等回放跑完
            chk(ncap === 200,          "T12a 重组消息本身完整（200 字节，没有被插进来的帧打断）");
            chk(ncommit === 1,         "T12b 重组消息提交恰好一次");
            chk(ev_reasm_err >= e0 + 1,"T12c ★回放期间丢的普通帧**计入 ev_reasm_err**（不再静默）");
            chk(nabort >= a0 + 1,      "T12d ★该帧以 abort 上报（不是假装 commit 成功）");
        end


        clear_cap;
        begin : t17
            integer e0, a0;
            e0 = ev_reasm_err; a0 = nabort;
            // 造一条 200 字节的重组消息（片1 96B + 片2 104B）→ 回放启动
            send_frame (1'b1, 1'b1, 13'd0,  16'hE001, 16'h0456, 1'b1, 96,  41);
            send_frame_x(1'b1, 1'b0, 13'd13, 16'hE001, 16'h0456, 1'b1, 104, 42);
            // 回放期间连发 **4 条**普通帧，只记 desc、先不发判决
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hE002, 16'h0456, 8, 43);
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hE003, 16'h0456, 8, 44);
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hE004, 16'h0456, 8, 45);  // 队满 → 丢 desc
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hE005, 16'h0456, 8, 46);  // 队满 → 丢 desc
            do_judge(1'b1);                 // 第 1 条判决（出队）
            do_judge(1'b1);                 // 第 2 条判决（出队）
            do_judge(1'b1);                 // 第 3 条判决（属于被丢的 desc → 跳过）
            do_judge(1'b1);                 // 第 4 条判决（同上）
            repeat (500) @(posedge clk);    // 等回放跑完
            chk(ev_reasm_err >= e0 + 4,
                "T17a ★★回放期间丢的**四条**普通帧都计入 ev_reasm_err（描述符不再互相覆盖/错位）");
            chk(nabort >= a0 + 4,
                "T17b ★★四条都以 abort 上报（被丢描述的判决不会与后续帧错位）");
        end


        clear_cap;
        begin : t_dropvb
            integer e0, a0, v0;
            e0 = ev_reasm_err; a0 = nabort; v0 = nabort_vb;
            // 两条非分片：字节透传 + desc 入队，判决先压着 → dq_cnt = 2
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hF001, 16'h0456, 8, 51);
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hF002, 16'h0456, 8, 52);
            // 第三条非分片：空闲态，字节照样透传出去；但 desc 队满 → 被丢
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hF003, 16'h0456, 8, 53);
            do_judge(1'b1);                 // 出队
            do_judge(1'b1);                 // 出队
            do_judge(1'b1);                 // 属于被丢的 desc
            repeat (20) @(posedge clk);
            chk(nabort >= a0 + 1, "T20a abort still reported");
            chk(nabort_vb === v0 + 1,
                "T20b vbytes=1 nonfrag drop");
            chk(ev_reasm_err >= e0 + 1, "T20c err counted");
        end

        // T20d：**分片帧**被丢描述符时相反 —— 这帧字节确实没出去，不能带 vbytes
        clear_cap;
        begin : t_dropvb_frag
            integer v0;
            v0 = nabort_vb;
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hF011, 16'h0456, 8, 61);
            send_bytes_desc(1'b0, 1'b0, 13'd0, 16'hF012, 16'h0456, 8, 62);
            // 第三条是**分片首片**：desc 队满被丢；分片帧不往外吐字节
            send_bytes_desc(1'b1, 1'b1, 13'd0, 16'hF013, 16'h0456, 16, 63);
            do_judge(1'b1); do_judge(1'b1); do_judge(1'b1);
            repeat (20) @(posedge clk);
            chk(nabort_vb === v0,
                "T20d vbytes=0 frag drop");
        end


        clear_cap;
        begin : t13
            integer bad;
            send_frame (1'b1, 1'b1, 13'd0,  16'hE001, 16'h0456, 1'b1, 96,  21);
            send_frame_x(1'b1, 1'b0, 13'd13, 16'hE001, 16'h0456, 1'b1, 104, 22);  // A 回放开始
            // 紧接着送第二条数据报（8B + 8B，off2 → 2*8-8 = 8 ✓，紧接在 8 字节之后）
            send_frame_x(1'b1, 1'b1, 13'd0, 16'hE002, 16'h0456, 1'b1, 8, 23);
            send_frame_x(1'b1, 1'b0, 13'd2, 16'hE002, 16'h0456, 1'b1, 8, 24);
            repeat (700) @(posedge clk);
            chk(ncap === 216 && ncommit === 2,
                "T13a ★两条背靠背的分片数据报都交付（216 字节 / 2 次 commit）");
            bad = 0;
            for (k = 0;   k < 96;  k = k + 1) if (cap[k]     !== pat(21, k)) bad = bad + 1;
            for (k = 0;   k < 104; k = k + 1) if (cap[96+k]  !== pat(22, k)) bad = bad + 1;
            for (k = 0;   k < 8;   k = k + 1) if (cap[200+k] !== pat(23, k)) bad = bad + 1;
            for (k = 0;   k < 8;   k = k + 1) if (cap[208+k] !== pat(24, k)) bad = bad + 1;
            chk(bad === 0, "T13b ★逐字节对照：第一条没被劫持、第二条内容正确（216 字节全对）");
        end


        clear_cap;
        begin : t14
            integer bad, e1;
            e1 = ev_reasm_err;
            send_frame (1'b1, 1'b1, 13'd0,  16'hE011, 16'h0456, 1'b1, 496, 31);
            send_frame_x(1'b1, 1'b0, 13'd63, 16'hE011, 16'h0456, 1'b1, 504, 32);  // 1000B，回放开始（~1250 拍）
            // 回放期间：第二条（16B）先完成并在等；第三条（16B）的分片正落在回放窗口里
            send_frame_x(1'b1, 1'b1, 13'd0, 16'hE012, 16'h0456, 1'b1, 8, 33);
            send_frame_x(1'b1, 1'b0, 13'd2, 16'hE012, 16'h0456, 1'b1, 8, 34);

            send_frame_x(1'b1, 1'b1, 13'd0, 16'hE013, 16'h0456, 1'b1, 8, 35);
            send_frame_x(1'b1, 1'b0, 13'd2, 16'hE013, 16'h0456, 1'b1, 400, 36);
            repeat (4000) @(posedge clk);
            bad = 0;
            for (k = 0; k < 496; k = k + 1) if (cap[k]      !== pat(31, k)) bad = bad + 1;
            for (k = 0; k < 504; k = k + 1) if (cap[496+k]  !== pat(32, k)) bad = bad + 1;
            for (k = 0; k < 8;   k = k + 1) if (cap[1000+k] !== pat(33, k)) bad = bad + 1;
            for (k = 0; k < 8;   k = k + 1) if (cap[1008+k] !== pat(34, k)) bad = bad + 1;
            $display("  [info] T14: ncap=%0d ncommit=%0d 首条错字节=%0d ev_err %0d→%0d",
                     ncap, ncommit, bad, e1, ev_reasm_err);
            chk(bad === 0,
                "T14a ★★回放途中到达的第三条数据报**没有污染正在回放的第一条**（1000 字节逐字节对照）");
            chk(ncommit === 2, "T14b 交付 2 条（第一条 + 等待的那条），第三条被拒");
            chk(ev_reasm_err > e1, "T14c ★第三条被拒是**可见**的（计入 ev_reasm_err）");
        end


        clear_cap;
        begin : t15
            integer bad15;
            in_port = 3'd4; in_dport = 16'd3001;
            send_frame (1'b1, 1'b1, 13'd0,  16'hE031, 16'h0456, 1'b1, 4096, 51);
            send_frame (1'b1, 1'b0, 13'd513, 16'hE031, 16'h0456, 1'b1, 4096, 52);  // 513*8-8 = 4096
            repeat (14000) @(posedge clk);
            bad15 = 0;
            for (k = 0; k < 4096; k = k + 1) if (cap[k]      !== pat(51, k)) bad15 = bad15 + 1;
            for (k = 0; k < 4096; k = k + 1) if (cap[4096+k] !== pat(52, k)) bad15 = bad15 + 1;
            $display("  [info] T15: 8192 字节数据报 ncap=%0d ncommit=%0d 错字节=%0d", ncap, ncommit, bad15);
            chk(ncap === 8192 && ncommit === 1,
                "T15a ★★恰好 8192 字节（标准单实例上限）的数据报被接受并交付（不再被误判越界）");
            chk(bad15 === 0, "T15b ★★而且 8192 字节逐字节正确");
        end


        clear_cap;
        begin : t16
            integer bad16;
            in_port = 3'd3; in_dport = 16'd3001;
            send_frame_x(1'b1, 1'b1, 13'd0, 16'hE041, 16'h0456, 1'b1, 96, 61);   // 片1（端口 3）
            in_port = 3'd2; in_dport = 16'd69;                                     // 插一个非分片帧
            send_frame_x(1'b0, 1'b0, 13'd0, 16'hE042, 16'h0456, 1'b1, 8, 62);
            in_port = 3'd3; in_dport = 16'd3001;
            send_frame_x(1'b1, 1'b0, 13'd13, 16'hE041, 16'h0456, 1'b1, 104, 63);  // 片2 → 回放
            repeat (900) @(posedge clk);
            bad16 = 0;

            for (k = (ncap >= 200 ? ncap - 200 : 0); k < ncap; k = k + 1)
                if (cap_p[k] !== 3'd3) bad16 = bad16 + 1;
            $display("  [info] T16: ncap=%0d，其中最后 200 字节带错端口的 = %0d", ncap, bad16);
            chk(bad16 === 0,
                "T16 ★★攒积期间插入的非分片帧**没有改掉**端口信息（重组消息仍交付到端口 3）");
        end


        clear_cap;
        begin : t19
            integer e19;
            e19 = ev_reasm_err;
            in_port = 3'd0; in_dport = 16'd1024;
            // 片1：一整条 8192 字节（MF=1，先攒着）
            send_frame(1'b1, 1'b1, 13'd0,    16'hE051, 16'h0456, 1'b1, 8192, 71);
            // 片2：偏移 1025 → 全精度起点 1025*8-8 = 8192（正好接上），但截断后 wstart=0
            send_frame(1'b1, 1'b0, 13'd1025, 16'hE051, 16'h0456, 1'b1,    8, 72);
            repeat (800) @(posedge clk);
            chk(ncap === 0 && ncommit === 0,
                "T19a 大偏移片被拒绝");
            chk(ev_reasm_err >= e19 + 1, "T19b 计入 ev_reasm_err");
        end

        // ---------- T20 8192 的另一种切分：8000 + 192（foff=1001 → 起点 8000）----------
        clear_cap;
        begin : t20
            integer bad20;
            in_port = 3'd0; in_dport = 16'd1024;
            send_frame(1'b1, 1'b1, 13'd0,    16'hE061, 16'h0456, 1'b1, 8000, 81);
            send_frame(1'b1, 1'b0, 13'd1001, 16'hE061, 16'h0456, 1'b1,  192, 82);
            repeat (14000) @(posedge clk);
            bad20 = 0;
            for (k = 0; k < 8000; k = k + 1) if (cap[k] !== pat(81, k)) bad20 = bad20 + 1;
            for (k = 0; k < 192;  k = k + 1) if (cap[8000+k] !== pat(82, k)) bad20 = bad20 + 1;
            $display("  [info] T20 ncap=%0d ncommit=%0d bad=%0d", ncap, ncommit, bad20);
            chk(ncap === 8192 && ncommit === 1 && bad20 === 0,
                "T20 8000+192 恰好 8192");
        end

        // ---------- T21 8184 + 9 = 8193 → 必须拒绝（边界外 1 字节）----------
        clear_cap;
        begin : t21
            integer e21;
            e21 = ev_reasm_err;
            in_port = 3'd0; in_dport = 16'd1024;
            send_frame(1'b1, 1'b1, 13'd0,    16'hE062, 16'h0456, 1'b1, 8184, 83);
            send_frame(1'b1, 1'b0, 13'd1024, 16'hE062, 16'h0456, 1'b1,    9, 84);
            repeat (2000) @(posedge clk);
            $display("  [info] T21 ncap=%0d ncommit=%0d err=%0d", ncap, ncommit, ev_reasm_err - e21);
            chk(ncap === 0 && ncommit === 0, "T21 8193 被拒绝");
            chk(ev_reasm_err >= e21 + 1, "T21 计入 ev_reasm_err");
        end


        clear_cap;
        begin : t22
            integer e22;
            e22 = ev_reasm_err;
            in_port = 3'd0; in_dport = 16'd1024;
            send_frame(1'b1, 1'b1, 13'd0, 16'hE071, 16'h0456, 1'b1, 0, 0);   // 空首片（MF=1）
            send_frame(1'b1, 1'b0, 13'd0, 16'hE071, 16'h0456, 1'b1, 0, 0);   // 空末片（MF=0）
            repeat (200) @(posedge clk);
            $display("  [info] T22 ncap=%0d ncommit=%0d err=%0d", ncap, ncommit, ev_reasm_err - e22);
            chk(ncap === 0 && ncommit === 0, "T22a 零长度数据报不输出、不提交");
            chk(ev_reasm_err >= e22 + 1,     "T22b 计入 ev_reasm_err（可见地丢）");
            // 活性：随后一条正常数据报照常交付
            clear_cap;
            send_frame(1'b0, 1'b0, 13'd0, 16'hE072, 16'h0456, 1'b1, 8, 3);
            chk(ncommit === 1, "T22c 零长度之后仍能交付正常数据报（未卡死）");

        end


        begin : t23
            integer e0, w0;
            send_frame_x(1'b1, 1'b1, 13'd0, 16'h00A1, 16'h0123, 1'b1, 20, 8'h11);
            repeat (10) @(posedge clk);
            chk(dut.acc_act === 1'b1, "T23a first fragment starts assembly");
            e0 = ev_reasm_err;
            w0 = 0;
            while ((dut.acc_act === 1'b1) && (w0 < 60000)) begin @(posedge clk); w0 = w0 + 1; end
            $display("  [dbg] T23 waited %0d cycles (TMO_CYC=20000)", w0);
            chk(dut.acc_act === 1'b0, "T23b reassembly timeout discards partial");
            chk(ev_reasm_err === (e0 + 1), "T23c timeout counted once");
        end


        chk(eop_wo_sop === 1'b0, "T24 no eop without sop (replay suppression is frame-sticky)");

        $display("==============================================");
        if (errs == 0) $display(">> PASS: IP 分片重组（透传/2片/3片/乱序/ipid不匹配/中间片被丢/混流）全部符合");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin #16000000; $display(">> TIMEOUT ncap=%0d ncommit=%0d", ncap, ncommit); $finish; end
endmodule
