// tb_axis_sim_shell.v —— AFDX_axis_sim_shell 冒烟（SE + MAC 网表）
`timescale 1ns/1ps

module tb_axis_sim_shell;
    reg clk = 0, reset = 1;
    always #4  clk = ~clk;   // 125 MHz

    // GMII（SIM_BYPASS=1 时顶层内部不用，这里给确定值）
    wire [7:0] ga_txd, gb_txd;
    wire ga_txen, ga_txer, ga_gtxc, gb_txen, gb_txer, gb_gtxc;
    wire mdc_a, mdc_b, phy_rstb_a, phy_rstb_b;
    wire mdio_a, mdio_b;
    pullup (mdio_a);
    pullup (mdio_b);
    wire fl_sclk, fl_mosi, fl_cs_n;
    wire fl_miso = 1'b1;

    wire [3:0] debug;
    wire mac_cfg_done, soc_boot_done;
    wire usr_irq_req, axis_error;
    wire mon_frame_valid;
    wire [7:0] mon_frame_data;

    // AXI4-Stream
    reg  [255:0] h2c_tdata = 0;
    reg  [31:0]  h2c_tkeep = 0;
    reg          h2c_tlast = 0, h2c_tvalid = 0;
    wire         h2c_tready;
    wire [255:0] c2h_tdata;
    wire [31:0]  c2h_tkeep;
    wire         c2h_tlast, c2h_tvalid;
    reg          c2h_tready = 0;

    AFDX_axis_sim_shell #(.PORTS(2), .SIM_BYPASS(1)) u_dut (
        .clk(clk), .reset(reset),
        .gmii_a_txd(ga_txd), .gmii_a_txen(ga_txen), .gmii_a_txer(ga_txer), .gmii_a_gtxc(ga_gtxc),
        .gmii_a_rxd(8'h00), .gmii_a_rxdv(1'b0), .gmii_a_rxer(1'b0), .gmii_a_rxc(clk),
        .gmii_b_txd(gb_txd), .gmii_b_txen(gb_txen), .gmii_b_txer(gb_txer), .gmii_b_gtxc(gb_gtxc),
        .gmii_b_rxd(8'h00), .gmii_b_rxdv(1'b0), .gmii_b_rxer(1'b0), .gmii_b_rxc(clk),
        .mdc_a(mdc_a), .mdio_a(mdio_a), .phy_rstb_a(phy_rstb_a),
        .mdc_b(mdc_b), .mdio_b(mdio_b), .phy_rstb_b(phy_rstb_b),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(fl_miso), .flash_cs_n(fl_cs_n),
        .m_axis_h2c_tdata(h2c_tdata), .m_axis_h2c_tkeep(h2c_tkeep),
        .m_axis_h2c_tlast(h2c_tlast), .m_axis_h2c_tvalid(h2c_tvalid),
        .m_axis_h2c_tready(h2c_tready),
        .s_axis_c2h_tdata(c2h_tdata), .s_axis_c2h_tkeep(c2h_tkeep),
        .s_axis_c2h_tlast(c2h_tlast), .s_axis_c2h_tvalid(c2h_tvalid),
        .s_axis_c2h_tready(c2h_tready),
        .usr_irq_req(usr_irq_req), .axis_error(axis_error),
        .debug(debug), .mac_cfg_done(mac_cfg_done), .soc_boot_done(soc_boot_done),
        .mon_frame_valid(mon_frame_valid), .mon_frame_data(mon_frame_data)
    );

    function [31:0] dres; input [11:0] a;
        begin dres = {u_dut.u_es.u_core.u_soc.dram3[a[11:2]], u_dut.u_es.u_core.u_soc.dram2[a[11:2]],
                      u_dut.u_es.u_core.u_soc.dram1[a[11:2]], u_dut.u_es.u_core.u_soc.dram0[a[11:2]]}; end
    endfunction

    integer errs = 0, nchk = 0, nfr = 0;
    task chk; input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    always @(posedge clk) if (mon_frame_valid) nfr <= nfr + 1;

    // H2C 发送（header + 单拍 payload；smoke 只用 ≤32 字节）
    task h2c_send; input [2:0] port; input integer len; input [255:0] pdata;
        reg [15:0] len16;
        integer nb;
        begin
            len16 = len;
            @(negedge clk);
            h2c_tdata = {{(256-32){1'b0}}, len16, 8'h00, {5'h0, port}};
            h2c_tkeep = {32{1'b1}}; h2c_tlast = 1'b0; h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk); @(negedge clk); h2c_tvalid = 1'b0;

            nb = len; if (nb > 32) nb = 32;
            @(negedge clk);
            h2c_tdata = pdata;
            h2c_tkeep = (nb == 32) ? {32{1'b1}} : ((32'h1 << nb) - 1);
            h2c_tlast = 1'b1; h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk); @(negedge clk); h2c_tvalid = 1'b0;
        end
    endtask

    reg [255:0] hdr, pay;
    reg [31:0]  kk;
    reg         ll;
    integer     gw;
    task c2h_get; output [255:0] d; output [31:0] k; output l;
        begin
            gw = 0;
            while (!c2h_tvalid && gw < 3000000) begin @(negedge clk); gw = gw + 1; end
            d = c2h_tdata; k = c2h_tkeep; l = c2h_tlast;
            c2h_tready = 1; @(posedge clk); @(negedge clk); c2h_tready = 0;
        end
    endtask

    integer w;
    initial begin
        #200 reset = 0;
        $display("");
        $display("======== AFDX_axis_sim_shell 冒烟（SE + MAC 网表）========");

        w = 0;
        while (dres(12'h3F0) !== 32'hA5A5C0DE && w < 3000000) begin @(posedge clk); w = w + 1; end
        chk(dres(12'h3F0) === 32'hA5A5C0DE, "T1 固件自检完成");

        h2c_send(3'd1, 16, {32'hCAFEBABE, 32'hDEADBEEF, 32'h11111111, 32'h484F5354});
        c2h_get(hdr, kk, ll);
        chk(hdr[2:0] === 3'd1, "T2a C2H port = 1");
        chk(hdr[31:16] === 16'd16, "T2b C2H len = 16");
        chk(ll === 1'b0, "T2c header 拍 tlast = 0");
        c2h_get(pay, kk, ll);
        chk(pay[127:0] === {32'hCAFEBABE, 32'hDEADBEEF, 32'h11111111, 32'h484F5354},
            "T2d payload 一致");
        chk(kk === 32'h0000FFFF, "T2e tkeep = 16 字节");
        chk(ll === 1'b1, "T2f payload tlast = 1");
        chk(axis_error === 1'b0, "T2g 无 axis_error");
        chk(nfr > 0, "T3 内部环回出现了 AFDX 帧（mon_frame_valid）");

        $display("-----------------------------------------------------");
        if (errs == 0) $display(">> PASS: AFDX_axis_sim_shell 冒烟全过");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin
        #300000000;
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
