// tb_axis_host.v —— AXI4-Stream 主机口快速冒烟测试（不依赖 MAC 网表）
`timescale 1ns/1ps

module tb_axis_host;
    reg clk = 0, reset = 1;
    always #4  clk = ~clk;   // 125 MHz              // 12.5 MHz

    // ---------------- SoC 帧流 / 回环 ----------------
    reg        rx_fv = 0, rx_fsop = 0, rx_feop = 0, rx_fnet = 0;
    reg  [7:0] rx_fd = 0;
    wire       dp_fr_valid, dp_fr_sop, dp_fr_eop, dp_fr_net;
    wire [7:0] dp_fr_data;
    wire       dp_fr_ready = 1'b1;      // 引擎发送侧始终能收（与 tb_host_path 一致）
    wire       cfg_hit; wire [127:0] cfg_word;
    wire [3:0] debug;
    wire       fl_sclk, fl_mosi, fl_cs_n;
    wire       fl_miso = 1'b1;          // 无 flash → 读回 0xFF → 保留预设配置

    always @(posedge clk) begin
        rx_fd   <= dp_fr_data;
        rx_fv   <= dp_fr_valid;
        rx_fsop <= dp_fr_sop;
        rx_feop <= dp_fr_eop;
        rx_fnet <= dp_fr_net;
    end

    // ---------------- 适配器 ↔ SoC 主机 AXI-Lite ----------------
    wire [31:0] h_awaddr, h_wdata, h_araddr, h_rdata;
    wire        h_awvalid, h_awready, h_wvalid, h_wready;
    wire        h_bvalid, h_bready, h_arvalid, h_arready;
    wire        h_rvalid, h_rready, h_irq;
    wire [1:0]  h_bresp, h_rresp;
    wire [3:0]  h_wstrb;

    // ---------------- AXI4-Stream（TB 驱动） ----------------
    reg  [255:0] h2c_tdata = 0;
    reg  [31:0]  h2c_tkeep = 0;
    reg          h2c_tlast = 0;
    reg          h2c_tvalid = 0;
    wire         h2c_tready;
    wire [255:0] c2h_tdata;
    wire [31:0]  c2h_tkeep;
    wire         c2h_tlast, c2h_tvalid;
    reg          c2h_tready = 0;      // 默认不接：先看清一拍，再脉冲 tready
    wire         axis_error;

    afdx_host_axis #(.DATA_WIDTH(256)) u_axis (
        .clk(clk), .reset(reset),
        .m_axis_h2c_tdata(h2c_tdata), .m_axis_h2c_tkeep(h2c_tkeep),
        .m_axis_h2c_tlast(h2c_tlast), .m_axis_h2c_tvalid(h2c_tvalid),
        .m_axis_h2c_tready(h2c_tready),
        .s_axis_c2h_tdata(c2h_tdata), .s_axis_c2h_tkeep(c2h_tkeep),
        .s_axis_c2h_tlast(c2h_tlast), .s_axis_c2h_tvalid(c2h_tvalid),
        .s_axis_c2h_tready(c2h_tready),
        .m_axil_awaddr(h_awaddr), .m_axil_awvalid(h_awvalid), .m_axil_awready(h_awready),
        .m_axil_wdata(h_wdata), .m_axil_wstrb(h_wstrb), .m_axil_wvalid(h_wvalid), .m_axil_wready(h_wready),
        .m_axil_bresp(h_bresp), .m_axil_bvalid(h_bvalid), .m_axil_bready(h_bready),
        .m_axil_araddr(h_araddr), .m_axil_arvalid(h_arvalid), .m_axil_arready(h_arready),
        .m_axil_rdata(h_rdata), .m_axil_rresp(h_rresp), .m_axil_rvalid(h_rvalid), .m_axil_rready(h_rready),
        .axis_error(axis_error)
    );

    darkriscv_soc_top u_dut (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk(clk), .reset(reset), .debug(debug),
        .dp_rxin_valid(rx_fv), .dp_rxin_sop(rx_fsop), .dp_rxin_eop(rx_feop),
        .dp_rxin_data(rx_fd), .dp_rxin_net(rx_fnet),
        .dp_fr_valid(dp_fr_valid), .dp_fr_ready(dp_fr_ready), .dp_fr_sop(dp_fr_sop),
        .dp_fr_eop(dp_fr_eop), .dp_fr_data(dp_fr_data), .dp_fr_net(dp_fr_net),
        .cfg_hit_o(cfg_hit), .cfg_cfg_o(cfg_word),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(fl_miso), .flash_cs_n(fl_cs_n),
        .ts_rx_ts(64'h0), .ts_rx_seq(16'h0), .ts_tx_ts(64'h0), .ts_tx_seq(16'h0),
        .ts_fine_o(),
        .ts_tx_vl_ts(32'h0), .ts_tx_vl_sel(),
        .ts_rx_vl_ts(32'h0), .ts_rx_vl_sel(),
        .host_awaddr(h_awaddr), .host_awvalid(h_awvalid), .host_awready(h_awready),
        .host_wdata(h_wdata), .host_wstrb(h_wstrb), .host_wvalid(h_wvalid), .host_wready(h_wready),
        .host_bresp(h_bresp), .host_bvalid(h_bvalid), .host_bready(h_bready),
        .host_araddr(h_araddr), .host_arvalid(h_arvalid), .host_arready(h_arready),
        .host_rdata(h_rdata), .host_rresp(h_rresp), .host_rvalid(h_rvalid), .host_rready(h_rready),
        .host_irq(h_irq)
    );

    function [31:0] dres; input [11:0] a;
        begin dres = {u_dut.dram3[a[11:2]], u_dut.dram2[a[11:2]],
                      u_dut.dram1[a[11:2]], u_dut.dram0[a[11:2]]}; end
    endfunction

    integer errs = 0, nchk = 0;
    task chk; input c; input [255:0] n;
        begin
            nchk = nchk + 1;
            if (c) $display("  [ok]   #%0d %0s", nchk, n);
            else begin errs = errs + 1; $display("  [FAIL] #%0d %0s", nchk, n); end
        end
    endtask

    // 载荷测试模式（长消息用）
    function [7:0] pat; input integer block; input integer i;
        begin pat = (block * 37 + i) & 8'hFF; end
    endfunction

    // ---------------- H2C 驱动 ----------------
    task h2c_send; input [3:0] port; input integer len; input [255:0] pdata;
        integer nbeats, b, nb;
        reg [255:0] td;
        reg [15:0]  len16;
        begin
            len16 = len;
            // header
            @(negedge clk);
            h2c_tdata  = {{(256-32){1'b0}}, len16, 8'h00, {4'h0, port}};
            h2c_tkeep  = {32{1'b1}};
            h2c_tlast  = 1'b0;
            h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            h2c_tvalid = 1'b0;

            // payload beats
            nbeats = (len + 31) / 32;
            for (b = 0; b < nbeats; b = b + 1) begin
                td = pdata >> (b*32*8);         // 256 位以内；len<=32 的 smoke 用例够用
                nb = len - b*32;
                if (nb > 32) nb = 32;
                @(negedge clk);
                h2c_tdata  = td;
                h2c_tkeep  = (nb == 32) ? {32{1'b1}} : ((32'h1 << nb) - 1);
                h2c_tlast  = (b == nbeats-1);
                h2c_tvalid = 1'b1;
                while (!h2c_tready) @(negedge clk);
                @(posedge clk);
                @(negedge clk);
                h2c_tvalid = 1'b0;
            end
        end
    endtask

    // 按 pat() 生成任意长度（1..1471）的 H2C 消息
    task h2c_send_pat; input [3:0] port; input integer len; input integer block;
        integer nbeats, b, j, nb;
        reg [255:0] td;
        reg [15:0]  len16;
        begin
            len16 = len;
            @(negedge clk);
            h2c_tdata  = {{(256-32){1'b0}}, len16, 8'h00, {4'h0, port}};
            h2c_tkeep  = {32{1'b1}};
            h2c_tlast  = 1'b0;
            h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            h2c_tvalid = 1'b0;

            nbeats = (len + 31) / 32;
            for (b = 0; b < nbeats; b = b + 1) begin
                td = {256{1'b0}};
                nb = len - b*32;
                if (nb > 32) nb = 32;
                for (j = 0; j < nb; j = j + 1)
                    td[j*8 +: 8] = pat(block, b*32 + j);
                @(negedge clk);
                h2c_tdata  = td;
                h2c_tkeep  = (nb == 32) ? {32{1'b1}} : ((32'h1 << nb) - 1);
                h2c_tlast  = (b == nbeats-1);
                h2c_tvalid = 1'b1;
                while (!h2c_tready) @(negedge clk);
                @(posedge clk);
                @(negedge clk);
                h2c_tvalid = 1'b0;
            end
        end
    endtask

    // 只发 header（用于非法头 / 不发 tlast 的容错测试）
    task h2c_send_hdr; input [3:0] port; input integer len;
        reg [15:0] len16;
        begin
            len16 = len;
            @(negedge clk);
            h2c_tdata  = {{(256-32){1'b0}}, len16, 8'h00, {4'h0, port}};
            h2c_tkeep  = {32{1'b1}};
            h2c_tlast  = 1'b0;
            h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            h2c_tvalid = 1'b0;
        end
    endtask


    task h2c_send_badlast; input [3:0] port; input integer len; input [255:0] pdata;
        reg [15:0] len16;
        begin
            len16 = len;
            @(negedge clk);
            h2c_tdata  = {{(256-32){1'b0}}, len16, 8'h00, {4'h0, port}};
            h2c_tkeep  = {32{1'b1}}; h2c_tlast = 1'b0; h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk); @(negedge clk); h2c_tvalid = 1'b0;
            @(negedge clk);
            h2c_tdata = pdata; h2c_tkeep = {32{1'b1}}; h2c_tlast = 1'b1; h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk); @(negedge clk); h2c_tvalid = 1'b0; h2c_tlast = 1'b0;
            repeat (8) @(posedge clk);
        end
    endtask


    task h2c_send_hdr_last; input [3:0] port; input integer len;
        reg [15:0] len16;
        begin
            len16 = len;
            @(negedge clk);
            h2c_tdata  = {{(256-32){1'b0}}, len16, 8'h00, {4'h0, port}};
            h2c_tkeep  = {32{1'b1}};
            h2c_tlast  = 1'b1;
            h2c_tvalid = 1'b1;
            while (!h2c_tready) @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            h2c_tvalid = 1'b0;
            h2c_tlast  = 1'b0;
        end
    endtask


    reg [255:0] got_hdr, got_pay;
    reg [31:0]  got_keep;
    reg         got_last;
    reg [255:0] beat_d;
    reg [31:0]  beat_k;
    reg         beat_l;
    integer     gw;

    integer c2h_hs = 0;
    reg     dbg_hs = 0;
    always @(posedge clk) if (c2h_tvalid && c2h_tready) begin
        c2h_hs = c2h_hs + 1;
        if (dbg_hs && c2h_hs <= 6)
            $display("  [dbg] C2H hs #%0d t=%0t tlast=%b b0=%02x b1=%02x",
                     c2h_hs, $time, c2h_tlast, c2h_tdata[7:0], c2h_tdata[15:8]);
    end

    reg         bp_en = 0;
    integer     bp_dly = 0;

    task c2h_get_beat; output [255:0] d; output [31:0] k; output l;
        begin
            gw = 0;
            while (!c2h_tvalid && gw < 3000000) begin @(negedge clk); gw = gw + 1; end
            d = c2h_tdata; k = c2h_tkeep; l = c2h_tlast;
            if (bp_en && bp_dly > 0) begin
                // 保持 tready=0，DUT 必须稳住 data/keep/last
                repeat (bp_dly) @(negedge clk);
                if ((c2h_tdata !== d) || (c2h_tkeep !== k) || (c2h_tlast !== l)) begin
                    $display("  [FAIL] C2H beat changed under backpressure");
                    errs = errs + 1;
                end
            end
            c2h_tready = 1;
            @(posedge clk);
            @(negedge clk);
            c2h_tready = 0;
        end
    endtask

    task c2h_recv; output [2:0] port; output integer len; output [255:0] pdata;
        begin
            c2h_get_beat(got_hdr, got_keep, got_last);
            port = got_hdr[2:0];
            len  = got_hdr[31:16];
            $display("  [info] C2H header: port=%0d len=%0d tlast=%b", port, len, got_last);
            c2h_get_beat(got_pay, got_keep, got_last);
            $display("  [info] C2H payload: keep=%08x tlast=%b data=%064x",
                     got_keep, got_last, got_pay);
            pdata = got_pay;
        end
    endtask

    // 读取一条 C2H 消息并逐字节比对 pat()；返回 bad（0 = 全对）
    task c2h_recv_pat; output [2:0] port; output integer len;
        input integer block; output integer bad;
        integer nbeats, b, j, nb;
        reg [255:0] d;
        reg [31:0]  k;
        reg         l;
        reg [31:0]  expk;
        begin
            bad = 0;
            c2h_get_beat(d, k, l);
            port = d[2:0]; len = d[31:16];
            if (l !== 1'b0) bad = bad + 1;
            nbeats = (len + 31) / 32;
            for (b = 0; b < nbeats; b = b + 1) begin
                c2h_get_beat(d, k, l);
                nb = len - b*32;
                if (nb > 32) nb = 32;
                expk = (nb == 32) ? {32{1'b1}} : ((32'h1 << nb) - 1);
                if (k !== expk) begin
                    bad = bad + 1;
                    $display("  [info] C2H keep mismatch beat %0d: got %08x exp %08x",
                             b, k, expk);
                end
                if (l !== (b == nbeats-1)) bad = bad + 1;
                for (j = 0; j < nb; j = j + 1)
                    if (d[j*8 +: 8] !== pat(block, b*32 + j)) begin
                        bad = bad + 1;
                        $display("  [info] C2H byte mismatch beat %0d byte %0d: got %02x exp %02x",
                                 b, j, d[j*8 +: 8], pat(block, b*32+j));
                    end
            end
        end
    endtask

    integer w;
    reg [2:0] rport; integer rlen;
    reg [255:0] rpay;
    integer bad, ti, tlen;
    integer lat_c2h = 0; reg lat_run = 1'b0;
    always @(posedge clk) if (lat_run) lat_c2h = lat_c2h + 1;

    initial begin
        #200 reset = 0;
        $display("");
        $display("======== AXI4-Stream 主机口冒烟（适配器 + 邮箱 + 固件 + AFDX）========");

        // T1 等固件自检完成
        w = 0;
        while (dres(12'h3F0) !== 32'hA5A5C0DE && w < 3000000) begin @(posedge clk); w = w + 1; end
        chk(dres(12'h3F0) === 32'hA5A5C0DE, "T1 固件自检完成");

        // T2 投递 16 字节到端口 1，检查 C2H 回环
        lat_c2h = 0; lat_run = 1'b1;
        h2c_send(3'd1, 16, {32'hCAFEBABE, 32'hDEADBEEF, 32'h11111111, 32'h484F5354});
        c2h_recv(rport, rlen, rpay);
        lat_run = 1'b0;
        $display("  [lat] T2 H2C->C2H = %0d cycles", lat_c2h);
        chk(rport === 3'd1, "T2a C2H port = 1");
        chk(rlen === 16, "T2b C2H len = 16");
        chk(rpay[127:0] === {32'hCAFEBABE, 32'hDEADBEEF, 32'h11111111, 32'h484F5354},
            "T2c C2H payload 逐字节一致");
        chk(got_keep === 32'h0000FFFF, "T2d 末拍 tkeep = 16 个有效字节");
        chk(got_last === 1'b1, "T2e 末拍 tlast = 1");
        chk(axis_error === 1'b0, "T2f 无 axis_error");

        // T3 1 字节边界
        h2c_send(3'd1, 1, {248'h0, 8'hA5});
        c2h_recv(rport, rlen, rpay);
        chk(rport === 3'd1 && rlen === 1, "T3a C2H port/len = 1/1");
        chk(rpay[7:0] === 8'hA5 && got_keep === 32'h00000001,
            "T3b 1 字节 payload + tkeep=1");
        chk(got_last === 1'b1, "T3c 单拍即 tlast");

        // T4 32 字节整拍
        h2c_send(3'd1, 32, {32'h0BADF00D, 32'h9ABCDEF0, 32'h12345678,
                            32'hDEADBEEF, 32'h1100FFEE, 32'h55443322,
                            32'h99887766, 32'hDDCCBBAA});
        c2h_recv(rport, rlen, rpay);
        chk(rlen === 32, "T4a C2H len = 32");
        chk(got_keep === {32{1'b1}}, "T4b 32 字节末拍 tkeep 全 1");
        chk(got_last === 1'b1, "T4c tlast = 1");

        // T5 非法头 + 不发 tlast → H_DRAIN 超时恢复；axis_error 粘滞
        begin : t5
            h2c_send_hdr(4'd10, 1);             // port=10 非法（端口字段已扩到 4 位，0..9 合法）
            repeat (4) @(posedge clk);          // 粘滞位在 err_h 之后一拍才置起
            chk(axis_error === 1'b1, "T5a invalid header sticky error");
            // H_DRAIN 期间 tready 仍为 1（要吃非法字节）；没有状态输出，
            // 等 70000 拍（> 65536 超时）再发下一条。
            repeat (700000) @(posedge clk);   // 125 MHz: DUT 的 H_DRAIN 超时 = 655360 拍
            chk(h2c_tready === 1'b1, "T5b drain timeout recovered");
            h2c_send(3'd1, 1, {248'h0, 8'h5A});
            c2h_recv(rport, rlen, rpay);
            chk(rport === 3'd1 && rlen === 1, "T5c valid msg after recovery");
            chk(rpay[7:0] === 8'h5A, "T5d payload after recovery");
            chk(axis_error === 1'b1, "T5e error stays sticky");
        end


        begin : t5b
            h2c_send_hdr_last(4'd10, 1);        // port=10 非法 + tlast=1（单拍消息）
            repeat (8) @(posedge clk);
            h2c_send(3'd1, 1, {248'h0, 8'h7B});
            c2h_recv(rport, rlen, rpay);
            chk(rport === 3'd1 && rlen === 1, "T5b valid msg not swallowed");
            chk(rpay[7:0] === 8'h7B, "T5c payload after bad hdr");
        end


        begin : t5d
            h2c_send_badlast(3'd1, 48, {224'h0, 32'hDEADBEEF});
            repeat (40) @(posedge clk);
            chk(axis_error === 1'b1, "T5d bad tlast sets error");
            h2c_send(3'd1, 1, {248'h0, 8'h3C});
            c2h_recv(rport, rlen, rpay);
            chk(rport === 3'd1 && rlen === 1 && rpay[7:0] === 8'h3C,
                "T5e bad tlast msg dropped");
        end

        // T6 最大长度 1471（46 个 payload 拍，末拍 tkeep 31 位）
        h2c_send_pat(3'd1, 1471, 6);
        c2h_recv_pat(rport, rlen, 6, bad);
        $display("  [info] T6 port=%0d len=%0d bad=%0d", rport, rlen, bad);
        chk(rport === 3'd1 && rlen === 1471 && bad === 0,
            "T6 1471B 全拍往返");

        // T7 拍边界：31/32/33/63/64/65
        for (ti = 0; ti < 6; ti = ti + 1) begin
            case (ti)
                0: tlen = 31;
                1: tlen = 32;
                2: tlen = 33;
                3: tlen = 63;
                4: tlen = 64;
                default: tlen = 65;
            endcase
            h2c_send_pat(3'd1, tlen, 10 + ti);
            c2h_recv_pat(rport, rlen, 10 + ti, bad);
            $display("  [info] T7 ti=%0d len=%0d port=%0d bad=%0d", ti, tlen, rport, bad);
            chk(rport === 3'd1 && rlen === tlen && bad === 0,
                "T7 拍边界 tkeep/tlast");
        end

        // T8 C2H 反压：每拍保持 tready=0 3 拍，DUT 必须稳住数据
        bp_en = 1; bp_dly = 3;
        h2c_send_pat(3'd1, 1471, 30);
        c2h_recv_pat(rport, rlen, 30, bad);
        bp_en = 0; bp_dly = 0;
        $display("  [info] T8 port=%0d len=%0d bad=%0d", rport, rlen, bad);
        chk(rport === 3'd1 && rlen === 1471 && bad === 0,
            "T8 1471B under C2H backpressure");


        c2h_tready = 1;
        repeat (30) @(posedge clk);
        $display("  [info] T9 drain: tvalid=%b（应为 0）", c2h_tvalid);
        c2h_hs = 0; dbg_hs = 1;            // 排空之后才开始计数
        h2c_send(3'd1, 16, {32'h0A0B0C0D, 32'h0E0F1011, 32'h12131415, 32'h16171819});
        w = 0;
        while (!(c2h_tvalid && c2h_tlast && c2h_tready) && w < 4000000) begin
            @(posedge clk); w = w + 1;
        end
        repeat (10) @(posedge clk);
        c2h_tready = 0; dbg_hs = 0;
        $display("  [info] T9 exit w=%0d tvalid=%b tlast=%b", w, c2h_tvalid, c2h_tlast);
        $display("  [info] T9 C2H handshakes with tready-always-high = %0d (expect 2)", c2h_hs);
        chk(c2h_hs == 2, "T9 tready-high: 2 beats exactly");

        $display("-----------------------------------------------------");
        if (errs == 0) $display(">> PASS: AXI4-Stream 主机口冒烟全过");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end

    initial begin

        #400000000;
        $display(">> TIMEOUT");
        $finish;
    end

endmodule
