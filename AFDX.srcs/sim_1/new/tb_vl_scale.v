// tb_vl_scale.v —— VL 配置表的规模与容量验证（对应赛题技术指标）
`timescale 1ns/1ps
module tb_vl_scale;

    reg clk = 0, reset = 1;
    always #50 clk = ~clk;

    // ---- CPU 侧窗口 ----
    reg         wr = 0, rd = 0;
    reg  [3:0]  rs = 0;
    reg  [31:0] wd = 0;
    wire [31:0] rv;

    // ---- 数据面查表口 ----
    reg  [15:0] vlid_i = 16'h0;
    wire [15:0] res_vlid;
    wire        hit;
    wire [127:0] cfg;

    afdx_cfg #(.NENT(4096), .IDXW(12)) u (
        .clk(clk), .reset(reset),
        .wr(wr), .rd(rd), .reg_sel(rs), .wdata(wd), .rdata(rv),
        .vlid_i(vlid_i), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(res_vlid), .hit_o(hit), .cfg_o(cfg)
    );

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    localparam R_IDX = 4'd0, R_W = 4'd1, R_LK = 4'd2,
               R_HIT = 4'd3, R_R0 = 4'd4, R_R1 = 4'd5, R_R2 = 4'd6, R_R3 = 4'd7;

    // 写一条表项（索引 + 4 个字；字指针在模块内自动 0→1→2→3）
    task ent_wr;
        input [11:0] idx; input [31:0] w0, w1, w2, w3;
        begin
            @(negedge clk); wr = 1; rs = R_IDX; wd = {20'h0, idx}; @(posedge clk);
            @(negedge clk);         rs = R_W;   wd = w0;           @(posedge clk);
            @(negedge clk);                     wd = w1;           @(posedge clk);
            @(negedge clk);                     wd = w2;           @(posedge clk);
            @(negedge clk);                     wd = w3;           @(posedge clk);
            @(negedge clk); wr = 0; rs = R_IDX; wd = 0;            @(posedge clk);
        end
    endtask

    // 查一条（用数据面口；等 4 拍让寄存读出数）—— 与 tx_path/ic_rm 用的是同一个口
    task lk;
        input [15:0] v;
        begin
            vlid_i = v; repeat (4) @(posedge clk);
        end
    endtask

    // 第 i 项期望的 4 个字（**唯一的、可复现的标记**；4 条 RAM 通道各自不同）
    function [31:0] ew0; input [11:0] i;
        //  w0 = rm_en<<30 | ic_en<<29 | net_en<<27 | lmax<<16 | vlid
        ew0 = (32'd1 << 30) | (32'd1 << 29) | ({30'h0, i} % 3) << 27
            | ((i[0] ? 32'd1471 : 32'd64) << 16) | {20'h0, i};
    endfunction
    function [31:0] ew1; input [11:0] i;
        //  w1 = udp_dport<<16 | skewmax<<4 | bag_shift
        ew1 = ((32'd1000 + i) << 16)
            | ((32'd100 + {20'h0, i} % 400) << 4)
            | ({28'h0, i} % 8);
    endfunction
    function [31:0] ew2; input [11:0] i;
        //  w2 = port_type<<20 | buf_idx<<16 | udp_sport
        ew2 = (({28'h0, i} % 4) << 20) | (({28'h0, i} % 5) << 16) | {20'h0, i};
    endfunction
    function [31:0] ew3; input [11:0] i;
        ew3 = 32'hE0E00000 | {20'h0, i};        // 目的 IP = 224.224.<低 12 位>
    endfunction

    integer i, k, bad;
    reg [31:0] g0, g1, g2, g3;

    initial begin
        #200 reset = 0;
        repeat (6) @(posedge clk);
        $display("");
        $display("======== VL table scale & capacity (128 TX VL / 4096 RX VL) ========");

        // ---------------- T1：全表 4096 项写入 ----------------
        for (i = 0; i < 4096; i = i + 1)
            ent_wr(i[11:0], ew0(i[11:0]), ew1(i[11:0]), ew2(i[11:0]), ew3(i[11:0]));
        chk(1'b1, "T1 full 4096 entries written");

        // ---------------- T2：全表 4096 项逐项查回（容量 + 无别名 + 字段完整）----------------
        bad = 0;
        for (i = 0; i < 4096; i = i + 1) begin
            lk(i[15:0]);
            if (hit !== 1'b1) bad = bad + 1;
            else if (cfg !== {ew3(i[11:0]), ew2(i[11:0]), ew1(i[11:0]), ew0(i[11:0])})
                bad = bad + 1;
        end
        $display("  [info] 4096 entries read back: %0d mismatch", bad);
        chk(bad == 0, "T2 4096 readback all match");

        // ---------------- T3：128 条"发送 VL" 的关键字段 ----------------
        //  赛题口径「最多 128 个发送 VL」—— 逐条核对发送侧真正会用到的字段
        bad = 0;
        for (i = 0; i < 128; i = i + 1) begin
            lk(i[15:0]);
            if (hit !== 1'b1) bad = bad + 1;
            else if (cfg[26:16] !== (i[0] ? 11'd1471 : 11'd64)) bad = bad + 1;   // lmax
            else if (cfg[35:32] !== ({28'h0, i[11:0]} % 8))     bad = bad + 1;   // bag_shift
            else if (cfg[28:27] !== ({30'h0, i[11:0]} % 3))     bad = bad + 1;   // net_en
            else if (cfg[29]    !== 1'b1 || cfg[30] !== 1'b1)   bad = bad + 1;   // ic/rm 使能
            else if (cfg[15:0]  !== i[15:0])                    bad = bad + 1;   // vlid
        end
        chk(bad == 0, "T3 128 TX VLs fields match");

        // ---------------- T4：高索引（0x800 / 0xFFF）也要通 ----------------
        lk(16'h0800);
        chk(hit === 1'b1 && cfg === {ew3(12'h800), ew2(12'h800), ew1(12'h800), ew0(12'h800)},
            "T4a index 0x800 hit & correct");
        chk(res_vlid === 16'h0800, "T4b lookup echo = 0x0800");
        lk(16'h0FFF);
        chk(hit === 1'b1 && cfg === {ew3(12'hFFF), ew2(12'hFFF), ew1(12'hFFF), ew0(12'hFFF)},
            "T4c index 0xFFF hit & correct");

        // ---------------- T5：命中判据是"存的 vlid == 查的 vlid" ----------------
        //   同一个槽（索引 0x800）用一个**不同的** vlid 去查 → 必须不命中
        lk(16'h1800);                                  // 低 12 位仍是 0x800，但 vlid 不同
        chk(hit === 1'b0, "T5 0x1800 miss (real hit)");

        // ---------------- T6：4 条 RAM 通道逐条独立 ----------------
        //   T2 的整字比对已经覆盖了这一点；这里再做一个"只改 w2 通道的一个字段"的定点对照
        ent_wr(12'h7FF, ew0(12'h7FF), ew1(12'h7FF), 32'h00A0_1234, ew3(12'h7FF));
        lk(16'h07FF);
        chk(hit === 1'b1 && cfg[31:0] === ew0(12'h7FF) && cfg[63:32] === ew1(12'h7FF) &&
            cfg[95:64] === 32'h00A0_1234 && cfg[127:96] === ew3(12'h7FF),
            "T6 4 RAM channels independent");

        $display("======================================================");
        if (errs == 0) $display(">> PASS: VL table scale/capacity/no-alias/channel-independence OK (4096 + 128 TX VL)");
        else           $display(">> FAIL: %0d item(s) failed", errs);
        $finish;
    end

    // 4096 写 + 4096 查 ≈ 6 万拍；给足余量
    initial begin #5000000; $display(">> TIMEOUT"); $finish; end

endmodule
