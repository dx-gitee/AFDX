// tb_cfg.v —— VL 配置表预设初值 单元测试
`timescale 1ns/1ps
module tb_cfg;
    reg clk=0, reset=1;
    always #50 clk = ~clk;
    reg [15:0] vlid = 0;
    reg wr = 0, rd = 0;
    reg [3:0]  reg_sel = 0;
    reg [31:0] wdata = 0;
    wire hit; wire [127:0] cfg;

    afdx_cfg #(.NENT(4096), .IDXW(12)) u (
        .clk(clk), .reset(reset),

        .wr(wr), .rd(rd), .reg_sel(reg_sel), .wdata(wdata), .rdata(),
        .vlid_i(vlid), .vlid_rx_i(16'h0), .rx_req_i(1'b0),
        .res_vlid_o(), .hit_o(hit), .cfg_o(cfg)
    );

    integer errs=0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    initial begin
        #200 reset = 0;
        repeat(6) @(posedge clk);
        $display("");
        $display("======== VL 配置表 预设初值 ========");

        vlid = 16'h0123; repeat(4) @(posedge clk);
        chk(hit===1'b1 && cfg[15:0]===16'h0123, "T1 VLID 0x0123 命中，vlid 字段对");
        chk(cfg[26:16]===11'd1471, "T2 0x0123 Lmax = 1471");
        chk(cfg[28:27]===2'd0,     "T3 0x0123 net_en = A+B(0)（标准冗余形态，双网各发一份）");
        chk(cfg[29]===1'b1 && cfg[30]===1'b1, "T4 0x0123 IC/RM 都使能");
        chk(cfg[35:32]===4'd3,     "T5 0x0123 bag_shift = 3（BAG 8ms）");
        chk(cfg[47:36]===12'd625,  "T6 0x0123 skewmax = 625 刻度 = 5ms");
        chk(cfg[63:48]===16'd1024, "T7 0x0123 UDP 目的端口 = 1024（采样口）");
        chk(cfg[81:80]===2'd0,     "T8 0x0123 buf_idx = 0（送进端口 0）");
        chk(cfg[87:84]===4'd1,     "T9 0x0123 port_type = 1（采样口）");
        chk(cfg[127:96]===32'hE0E00123, "T10 0x0123 目的 IP = 224.224.0x0123");

        vlid = 16'h0456; repeat(4) @(posedge clk);
        chk(hit===1'b1 && cfg[63:48]===16'd1025 && cfg[81:80]===2'd1 && cfg[87:84]===4'd2,
            "T11 VLID 0x0456 = 队列口（dport 1025 / buf_idx 1 / type 2）");
        vlid = 16'h0789; repeat(4) @(posedge clk);
        chk(hit===1'b1 && cfg[63:48]===16'd69 && cfg[81:80]===2'd2 && cfg[87:84]===4'd3,
            "T12 VLID 0x0789 = SAP 口（dport 69 / buf_idx 2 / type 3）");
        // BAG 32ms → 1ms：三个服务口（615A/SNMP/RTC）改用最小 BAG，理由见 gen_cfg.py 的注释
        // （服务口是"请求-应答"型，BAG 直接决定吞吐，也决定仿真跑不跑得动）
        chk(cfg[35:32]===4'd0, "T13 0x0789 bag_shift = 0（BAG 1ms，服务口）");

        vlid = 16'h0ABC; repeat(4) @(posedge clk);
        chk(hit===1'b0, "T14 未配置的 VLID 0x0ABC 不命中");


        chk(u.wr_idx === 12'd0 && u.wr_wsel === 2'd0,
            "T15 wr_idx/wr_wsel reset=0");
        // 第一条就写 R_WORD（不写 R_IDX）：复位到 0 后应确定地写 tab0[0]
        @(negedge clk); wr = 1; reg_sel = 4'd1; wdata = 32'hA5A51234; @(posedge clk);
        @(negedge clk); wr = 0; reg_sel = 4'd0;
        repeat (2) @(posedge clk);
        chk(u.wr_wsel === 2'd1, "T16 first R_WORD wsel=1");
        chk(u.tab0[0] === 32'hA5A51234, "T17 first R_WORD hits tab0[0]");

        @(negedge clk); wr=1; reg_sel=4'd0; wdata=32'h0000_0BC5; @(posedge clk);  // R_IDX 合法
        @(negedge clk);          reg_sel=4'd1; wdata=32'h11112222; @(posedge clk); // R_WORD 第 0 字
        @(negedge clk);          reg_sel=4'd0; wdata=32'h0000_1BC5; @(posedge clk); // R_IDX 越界
        @(negedge clk);          reg_sel=4'd1; wdata=32'hDEADBEEF; @(posedge clk); // 必须被忽略
        @(negedge clk); wr=0; reg_sel=4'd0;
        repeat (2) @(posedge clk);
        chk(u.tab0[12'h0BC5] === 32'h11112222, "T18 out-of-range idx no alias");

        $display("==================================");
        if (errs==0) $display(">> PASS: VL 表预设初值（三条 VL + 端口绑定 + 未命中）全部正确");
        else         $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end
    initial begin #200000; $display(">> TIMEOUT"); $finish; end
endmodule
