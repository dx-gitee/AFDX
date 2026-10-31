// tb_darkriscv_soc.v —— SoC 端到端测试
`timescale 1ns/1ps

module tb_darkriscv_soc;
    reg        clk = 1'b0;
    reg        res = 1'b1;
    wire [3:0] debug;

    wire        dp_fr_valid, dp_fr_sop, dp_fr_eop, dp_fr_net;
    wire [7:0]  dp_fr_data;
    reg         dp_fr_ready;

    wire        cfg_hit;
    wire [127:0] cfg_word;

    reg         rx_fv, rx_fsop, rx_feop, rx_fnet;
    reg  [7:0]  rx_fd;


    wire        fl_sclk, fl_mosi, fl_cs_n, fl_miso;
    spi_flash_model #(.AW(13), .USE_IMG(1)) u_flash (
        .sclk(fl_sclk), .cs_n(fl_cs_n), .mosi(fl_mosi), .miso(fl_miso)
    );


    localparam [7:0] N_VL_TABLE = 8'd139;

    wire [31:0] ts_fine_obs;

    parameter integer MST_PPM = 500;

    parameter integer FINE_DIV = 10;
    reg [31:0] m_fine = 0;
    reg [15:0] m_drift = 0;
    reg        m_drift_en = 1'b0;
    reg [3:0]  m_div = 0;
    wire       m_tick = (m_div == FINE_DIV - 1);
    localparam integer DRIFT_DIV = (MST_PPM > 0) ? (1000000 / MST_PPM) : 0;
    always @(posedge clk) begin
        if (res) begin m_fine <= 32'd0; m_drift <= 16'd0; m_div <= 4'd0; end
        else begin
            m_div <= m_tick ? 4'd0 : (m_div + 4'd1);
            if (m_tick) begin
                if (m_drift_en && MST_PPM > 0) begin
                    if (m_drift >= DRIFT_DIV - 1) begin m_drift <= 16'd0; m_fine <= m_fine + 32'd2; end
                    else begin m_drift <= m_drift + 16'd1; m_fine <= m_fine + 32'd1; end
                end
                else m_fine <= m_fine + 32'd1;
            end
        end
    end
    wire [31:0] m_ts   = m_fine + 32'd5000;
    wire [31:0] m_ts_b = m_fine + 32'd7000;
    reg  [31:0] m_tx_sop;      // A 网主时钟"收到 ES 的 DelayReq"的时刻
    reg  [31:0] m_tx_sop_b;    // B 网主时钟"收到 ES 的 DelayReq(B 副本)"的时刻
    reg  [31:0] rx_ts_w = 32'd0, tx_ts_w = 32'd0;
    reg  [31:0] rx_ts_b = 32'd0, tx_ts_b = 32'd0;
    reg  [7:0]  rx_seq_w = 8'd0, tx_seq_w = 8'd0;
    reg  [7:0]  rx_seq_b = 8'd0, tx_seq_b = 8'd0;
    reg  inj_net = 1'b0;


    wire [12:0] tb_tx_vl_sel;
    reg  [31:0] tb_tx_vl_ts;

    wire [12:0] tb_rx_vl_sel;
    reg  [31:0] tb_rx_vl_ts;

    darkriscv_soc_top u_dut (

        .app_rx_valid(1'b0), .app_rx_data(8'h00), .app_port(4'h0),
        .app_tx_valid(), .app_tx_data(), .app_tx_port(), .app_err(),

        .clk(clk), .reset(res), .debug(debug),
        .dp_rxin_valid(rx_fv), .dp_rxin_sop(rx_fsop), .dp_rxin_eop(rx_feop),
        .dp_rxin_data(rx_fd),  .dp_rxin_net(rx_fnet),
        .dp_fr_valid(dp_fr_valid), .dp_fr_ready(dp_fr_ready), .dp_fr_sop(dp_fr_sop),
        .dp_fr_eop(dp_fr_eop), .dp_fr_data(dp_fr_data), .dp_fr_net(dp_fr_net),
        .cfg_hit_o(cfg_hit), .cfg_cfg_o(cfg_word),
        .flash_sclk(fl_sclk), .flash_mosi(fl_mosi), .flash_miso(fl_miso), .flash_cs_n(fl_cs_n),

        .ts_rx_ts({rx_ts_b, rx_ts_w}), .ts_rx_seq({rx_seq_b, rx_seq_w}),
        .ts_tx_ts({tx_ts_b, tx_ts_w}), .ts_tx_seq({tx_seq_b, tx_seq_w}),
        .ts_fine_o(ts_fine_obs),
        .ts_tx_vl_ts(tb_tx_vl_ts), .ts_tx_vl_sel(tb_tx_vl_sel),
        .ts_rx_vl_ts(tb_rx_vl_ts), .ts_rx_vl_sel(tb_rx_vl_sel),

        .host_awaddr(32'h0), .host_awvalid(1'b0), .host_awready(),
        .host_wdata(32'h0), .host_wstrb(4'h0), .host_wvalid(1'b0), .host_wready(),
        .host_bresp(), .host_bvalid(), .host_bready(1'b0),
        .host_araddr(32'h0), .host_arvalid(1'b0), .host_arready(),
        .host_rdata(), .host_rresp(), .host_rvalid(), .host_rready(1'b0),
        .host_irq()
    );


    always @(posedge clk)
        if (dp_fr_valid && dp_fr_ready && dp_fr_sop) begin
            if (dp_fr_net) begin
                tx_ts_b  <= ts_fine_obs + 32'd2000;   // ES 的 B 网发帧戳（模型）
                tx_seq_b <= tx_seq_b + 8'd1;
                m_tx_sop_b <= m_ts_b;                  // B 网主时钟收到这一帧的时刻
            end
            else begin
                tx_ts_w  <= ts_fine_obs;
                tx_seq_w <= tx_seq_w + 8'd1;
                m_tx_sop <= m_ts;                      // A 网主时钟收到这一帧的时刻
            end
        end

    reg  [31:0] tb_tbl [0:8191];
    reg  [31:0] tb_tbl_i;
    reg  [3:0]  tb_bcnt;
    reg  [7:0]  tb_vh;
    reg         tb_armed;
    initial for (tb_tbl_i = 0; tb_tbl_i < 8192; tb_tbl_i = tb_tbl_i + 1) tb_tbl[tb_tbl_i] = 32'd0;
    always @(posedge clk) begin
        if (dp_fr_valid && dp_fr_ready) begin
            if (dp_fr_sop) begin
                tb_armed <= 1'b1; tb_bcnt <= 4'd1;
            end
            else if (tb_armed) begin
                if (tb_bcnt == 4'd4) tb_vh <= dp_fr_data;
                if (tb_bcnt == 4'd5) begin
                    tb_tbl[{dp_fr_net, tb_vh[3:0], dp_fr_data}] <= dp_fr_net ? tx_ts_b : tx_ts_w;
                    tb_armed <= 1'b0;
                end
                tb_bcnt <= tb_bcnt + 4'd1;
            end
        end
    end
    always @(*) tb_tx_vl_ts = tb_tbl[tb_tx_vl_sel];


    integer n_netb_desc = 0, n_netb_commit = 0, n_p4_deliv = 0;
    integer n_p3_deliv = 0, n_desc_a = 0;
    always @(posedge clk) begin
        if (u_dut.u_rxports.deliv_o[3]) n_p3_deliv = n_p3_deliv + 1;
        if (u_dut.rxp_desc && !u_dut.rxp_net) n_desc_a = n_desc_a + 1;
    end
    always @(posedge clk) begin
        if (u_dut.rxp_desc && u_dut.rxp_net) n_netb_desc   = n_netb_desc + 1;
        if (u_dut.rm_commit && u_dut.rm_net) n_netb_commit = n_netb_commit + 1;
        if (u_dut.u_rxports.deliv_o[4])      n_p4_deliv    = n_p4_deliv + 1;
    end


    always @(posedge clk)
        if (rx_fv && rx_fsop) begin
            if (rx_fnet) begin
                rx_ts_b  <= ts_fine_obs + 32'd2000;
                rx_seq_b <= rx_seq_b + 8'd1;
            end
            else begin
                rx_ts_w  <= ts_fine_obs + 32'd3000;
                rx_seq_w <= rx_seq_w + 8'd1;
            end
        end


    reg [31:0] tb_rxtbl [0:8191];
    reg [31:0] tb_rxtbl_i;
    reg [3:0]  tb_rcnt;
    reg [7:0]  tb_rvh;
    reg        tb_rarmed;
    reg [31:0] tb_rx_sop_ts;
    initial for (tb_rxtbl_i = 0; tb_rxtbl_i < 8192; tb_rxtbl_i = tb_rxtbl_i + 1)
        tb_rxtbl[tb_rxtbl_i] = 32'd0;
    always @(posedge clk) begin
        if (rx_fv && rx_fsop) begin
            tb_rarmed    <= 1'b1;
            tb_rcnt      <= 4'd1;
            tb_rx_sop_ts <= rx_fnet ? (ts_fine_obs + 32'd2000) : ts_fine_obs;
        end
        else if (rx_fv && tb_rarmed) begin
            if (tb_rcnt == 4'd4) tb_rvh <= rx_fd;
            if (tb_rcnt == 4'd5) begin
                tb_rxtbl[{rx_fnet, tb_rvh[3:0], rx_fd}] <= tb_rx_sop_ts;
                tb_rarmed <= 1'b0;
            end
            tb_rcnt <= tb_rcnt + 4'd1;
        end
    end
    always @(*) tb_rx_vl_ts = tb_rxtbl[tb_rx_vl_sel];

    always #4  clk = ~clk;   // 125 MHz


    reg [31:0] dadr_prev = 0;
    integer n_den = 0, n_stable = 0, n_port_chg = 0, n_port_chg_stable = 0;
    reg [1:0]  ppi_prev = 0;
    always @(posedge clk) begin
        dadr_prev <= u_dut.DADDR;
        ppi_prev  <= u_dut.DADDR[7:6];
        if (u_dut.den) begin
            n_den <= n_den + 1;
            if (dadr_prev === u_dut.DADDR) n_stable <= n_stable + 1;
            if (u_dut.DADDR[7:6] !== ppi_prev) begin
                n_port_chg <= n_port_chg + 1;
                if (dadr_prev[7:6] === u_dut.DADDR[7:6]) n_port_chg_stable <= n_port_chg_stable + 1;
            end
        end
    end

    initial begin
        rx_fv = 0; rx_fsop = 0; rx_feop = 0; rx_fnet = 0; rx_fd = 0;
        dp_fr_ready = 1;
    end

    reg inject_en = 0;
    always @(posedge clk) begin
        if (!inject_en) begin
            rx_fv   <= dp_fr_valid;
            rx_fsop <= dp_fr_sop;
            rx_feop <= dp_fr_eop;
            rx_fd   <= dp_fr_data;
            rx_fnet <= dp_fr_net;
        end
    end
    // RTC 请求帧的构造（合法帧：DA=03:00:00:00:0B:C5，UDP dport=3001，源 10.0.0.9:0x1234）
    reg [7:0] rf [0:79]; integer rlen;
    function [31:0] rcrc32b;
        input [31:0] c; input [7:0] d;
        integer k; reg [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            rcrc32b = t;
        end
    endfunction
    task inj_rtc;
        integer b; reg [31:0] sum, cs, c;
        begin
            for (b = 0; b < 80; b = b + 1) rf[b] = 8'h00;
            rf[0]=8'h03; rf[4]=8'h0B; rf[5]=8'hC5;                 // DA = 03:00:00:00:0B:C5
            rf[6]=8'h02; rf[10]=8'h01; rf[11]=8'h20;               // SA（IFID=1）
            rf[12]=8'h08; rf[13]=8'h00;

            rf[14]=8'h45; rf[16]=8'h00; rf[17]=8'h1E;              // IP, TotalLen=30
            rf[20]=8'h40; rf[22]=8'h01; rf[23]=8'h11;              // TTL=1, proto=UDP
            rf[26]=8'h0A; rf[27]=8'h00; rf[28]=8'h00; rf[29]=8'h09;// 源 IP 10.0.0.9
            rf[30]=8'hE0; rf[31]=8'hE0; rf[32]=8'h0B; rf[33]=8'hC5;// 目的 224.224.11.197
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {rf[b], rf[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            rf[24]=cs[15:8]; rf[25]=cs[7:0];
            rf[34]=8'h12; rf[35]=8'h34;                            // 源端口 0x1234
            rf[36]=8'h0B; rf[37]=8'hB9;                            // 目的端口 3001（RTC）
            rf[38]=8'h00; rf[39]=8'h0A; rf[40]=8'h00; rf[41]=8'h00;// UDP len=10, 校验和=0
            rf[42]=8'h51; rf[43]=8'h01;                            // 载荷 2 字节（请求内容不解析）
            rlen = 60;                                             // 补 padding 到 60
            rf[59]=8'h00;                                          // SN（帧末前 4 字节）
            c = 32'hFFFFFFFF;
            for (b = 0; b < 60; b = b + 1) c = rcrc32b(c, rf[b]);
            c = ~c;
            rf[60]=c[7:0]; rf[61]=c[15:8]; rf[62]=c[23:16]; rf[63]=c[31:24];
            rlen = 64;
            for (b = 0; b < rlen; b = b + 1) begin
                @(negedge clk);

                rx_fv = 1; rx_fd = rf[b]; rx_fsop = (b == 0); rx_feop = (b == rlen-1);
                rx_fnet = 1'b0;
                @(posedge clk);
            end
            @(negedge clk); rx_fv = 0; rx_fsop = 0; rx_feop = 0; rx_fnet = 1'b0;
        end
    endtask

    // ---- 通用 UDP 帧注入（615A/TFTP 等用例用）----
    //   调用前把载荷填进 pl[0..pl_len-1]，然后 inj_udp(VLID, 目的UDP端口, 源UDP端口)
    //   帧结构与本设计一致：DA=03:00:00:00|<VLID>，SA 的 IFID=1，SN 在**帧末前 1 字节**（紧贴 FCS）
    reg [7:0]  pl [0:511];
    // SNMP GET/GETNEXT 请求的模板（47 字节，见下面 inj_snmp 的注释）
    reg [7:0]  snmp_req [0:63];
    integer    pl_len = 0;
    reg [7:0]  gf [0:511];      // 同上（注入帧总长 42+284+1+4 = 331）
    integer    gf_len = 0;                 // 本帧总字节数（含 4 字节 FCS）
    task inj_udp;
        input [15:0] vlid;
        input [15:0] dport;
        input [15:0] sport;
        input [7:0]  sn;
        integer b, total, iplen, snpos;
        reg [31:0] sum, cs, c;
        begin
            iplen = 20 + 8 + pl_len;

            snpos = 14 + iplen;                         // 载荷结束的下一字节
            if (snpos < 59) snpos = 59;                 // 不足 60 字节先补 padding
            total = snpos + 1;                          // 帧体 = ... + SN（不含 FCS）
            for (b = 0; b < 300; b = b + 1) gf[b] = 8'h00;
            // ETH
            gf[0]=8'h03; gf[4]=vlid[15:8]; gf[5]=vlid[7:0];

            gf[6]=8'h02; gf[10]=8'h01; gf[11]= inj_net ? 8'h40 : 8'h20;
            gf[12]=8'h08; gf[13]=8'h00;
            // IP
            gf[14]=8'h45; gf[16]=iplen[15:8]; gf[17]=iplen[7:0];
            gf[20]=8'h40; gf[22]=8'h01; gf[23]=8'h11;   // TTL=1 proto=UDP
            gf[26]=8'h0A; gf[27]=8'h00; gf[28]=8'h00; gf[29]=8'h09;   // 源 IP 10.0.0.9
            gf[30]=8'hE0; gf[31]=8'hE0; gf[32]=vlid[15:8]; gf[33]=vlid[7:0];
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {gf[b], gf[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            gf[24]=cs[15:8]; gf[25]=cs[7:0];
            // UDP（校验和固定 0 —— 标准要求）
            gf[34]=sport[15:8]; gf[35]=sport[7:0];
            gf[36]=dport[15:8]; gf[37]=dport[7:0];
            gf[38]=(8+pl_len)>>8; gf[39]=(8+pl_len)&8'hFF;
            gf[40]=0; gf[41]=0;
            for (b = 0; b < pl_len; b = b + 1) gf[42+b] = pl[b];

            gf[snpos] = sn;
            c = 32'hFFFFFFFF;
            for (b = 0; b < total; b = b + 1) c = rcrc32b(c, gf[b]);
            c = ~c;
            gf[total+0]=c[7:0]; gf[total+1]=c[15:8];
            gf[total+2]=c[23:16]; gf[total+3]=c[31:24];
            gf_len = total + 4;
            for (b = 0; b < gf_len; b = b + 1) begin
                @(negedge clk);

                rx_fv = 1; rx_fd = gf[b]; rx_fsop = (b == 0); rx_feop = (b == gf_len-1); rx_fnet = inj_net;
                @(posedge clk);
            end
            @(negedge clk); rx_fv = 0; rx_fsop = 0; rx_feop = 0; rx_fnet = 1'b0;
        end
    endtask

    // 喂一帧短帧（头 6 字节足够让解析器抓出 VLID 并触发提前查表；末字节带 eop）
    task inj_frame;
        input [15:0] bad_vlid;
        begin
            @(negedge clk); rx_fv=1; rx_fsop=1; rx_feop=0; rx_fnet=0; rx_fd=8'h03;
            @(negedge clk); rx_fsop=0; rx_fd=8'h00;
            @(negedge clk); rx_fd=8'h00;
            @(negedge clk); rx_fd=8'h00;
            @(negedge clk); rx_fd=bad_vlid[15:8];
            @(negedge clk); rx_fd=bad_vlid[7:0];
            @(negedge clk); rx_fv=1; rx_feop=1; rx_fd=8'h02;
            @(negedge clk); rx_fv=0; rx_feop=0;
        end
    endtask


    reg [7:0] fr [0:32767];

    reg [31:0] fr_sop_ts [0:32767];
    integer   nfr = 0;
    integer   base_rtc = 0, n_rtc = 0;
    integer   base_tftp = 0, k = 0, m = 0, okcnt = 0;
    // T7 探针：SAP 口 4 是否真的收到过消息（pcnt[4] 由端口层维护）
    reg       saw_rtc_msg = 1'b0;
    always @(posedge clk) if (u_dut.u_rxports.pcnt[4] != 0) saw_rtc_msg <= 1'b1;
    localparam integer F2 = 64;              // 第 2 帧（B 副本）在 fr[] 里的起始下标
    always @(posedge clk) if (dp_fr_valid && dp_fr_ready) begin
        fr[nfr] <= dp_fr_data;
        if (dp_fr_sop) fr_sop_ts[nfr] <= dp_fr_net ? m_ts_b : m_ts;
        nfr <= nfr + 1;
    end


    function integer lsz;
        input integer lp;
        begin
            lsz = (fr[lp] & 8'h80) ? (1 + (fr[lp] & 8'h7F)) : 1;
        end
    endfunction
    function integer lv;
        input integer lp;
        integer q2, nb;
        begin
            lv = 0;
            if (fr[lp] & 8'h80) begin
                nb = fr[lp] & 8'h7F;
                for (q2 = 0; q2 < nb; q2 = q2 + 1) lv = (lv << 8) | fr[lp + 1 + q2];
            end
            else lv = fr[lp];
        end
    endfunction


    integer scan_pos = 0;
    integer wscan = 0;                  // P0：等扫表标志的轮询计数
    task expect_reply;
        input [15:0] vlid;          // 在哪个 VL 上等（0x0789 = 615A / 0x0161 = SNMP）
        input [31:0] exp4;          // 期望的载荷前 4 字节（小端）
        input integer need;         // 这一帧至少要收全多少字节才算数（校验要用的长度）
        input integer maxc;
        integer w;
        reg found;
        begin
            found = 0; w = 0;
            while (!found && w < maxc) begin
                k = find_frame(scan_pos, vlid);
                if (k >= 0) begin

                    if ((k + need <= nfr) &&
                        ((exp4 === 32'h0) ||
                         ({fr[k+42],fr[k+43],fr[k+44],fr[k+45]} === exp4))) found = 1;

                    else if (k + need <= nfr) scan_pos = k + 1;
                end
                @(posedge clk);
                w = w + 1;
            end
            if (!found)
                $display(" [超时] 期望载荷 %08x 没等到；起点=%0d 游标=%0d nfr=%0d 收到=%0d",
                         exp4, base_tftp, scan_pos, nfr, dres(12'h340));
        end
    endtask

    // 在 SNMP 应答里按 request-id 找（A/B 副本 + 多条请求时，光看 OID 分不开）
    task expect_snmp_reqid;
        input [15:0] vlid;
        input [7:0]  rid;
        input integer need;
        input integer maxc;
        integer w3;
        reg found3;
        begin
            found3 = 0; w3 = 0;
            while (!found3 && w3 < maxc) begin
                k = find_frame(scan_pos, vlid);
                if (k >= 0 && (k + need) <= nfr) begin
                    // GetResponse 的 PDU: [15]=02 [16]=01 [17]=reqid
                    if ({fr[k+42+15], fr[k+42+16], fr[k+42+17]} === {8'h02, 8'h01, rid})
                        found3 = 1;
                    else
                        scan_pos = k + 1;
                end
                @(posedge clk);
                w3 = w3 + 1;
            end
            if (!found3)
                $display(" [超时] 没等到 SNMP reqid=%0d 的应答（nfr=%0d）", rid, nfr);
        end
    endtask

    // 构造一条 SNMP v1 GET/GETNEXT 请求（RFC 1157）：
    //   SEQUENCE{ INTEGER version=0, OCTET STRING "public", PDU }
    //   PDU = [0xA0 GET / 0xA1 GETNEXT] SEQUENCE{ reqid, errstat, erridx, SEQUENCE{ SEQUENCE{ OID, NULL } } }
    //   OID = 1.3.6.1.4.1.13712.664.<g>.<i>.<x>
    task inj_snmp;
        input [7:0] g; input [7:0] ii; input [7:0] x;
        input [7:0] pdu;
        begin
            pl_len = 47;
            for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
            pl[0]=8'h30; pl[1]=8'h2D;                       // SEQUENCE (45)
            pl[2]=8'h02; pl[3]=8'h01; pl[4]=8'h00;          // version = 0 (v1)
            pl[5]=8'h04; pl[6]=8'h06;                       // community
            pl[7]=8'h70; pl[8]=8'h75; pl[9]=8'h62; pl[10]=8'h6C; pl[11]=8'h69; pl[12]=8'h63; // "public"
            pl[13]=pdu;  pl[14]=8'h20;                      // PDU (32)
            pl[15]=8'h02; pl[16]=8'h04; pl[17]=0; pl[18]=0; pl[19]=0; pl[20]=1; // reqid = 1
            pl[21]=8'h02; pl[22]=8'h01; pl[23]=8'h00;       // error-status
            pl[24]=8'h02; pl[25]=8'h01; pl[26]=8'h00;       // error-index
            pl[27]=8'h30; pl[28]=8'h12;                     // varbind list (18)
            pl[29]=8'h30; pl[30]=8'h10;                     // varbind (16)
            pl[31]=8'h06; pl[32]=8'h0C;                     // OID (12)
            pl[33]=8'h2B; pl[34]=8'h06; pl[35]=8'h01; pl[36]=8'h04; pl[37]=8'h01;
            pl[38]=8'hEB; pl[39]=8'h10;                     // 13712 (base128)
            pl[40]=8'h85; pl[41]=8'h18;                     // 664   (base128)
            pl[42]=g; pl[43]=ii; pl[44]=x;
            pl[45]=8'h05; pl[46]=8'h00;                     // NULL
        end
    endtask


    reg [7:0] oidb [0:7];
    reg [7:0] vbuf [0:31];
    task inj_snmp_priv;
        input [7:0] pdu;
        input integer noid;            // OID 尾字节数（g,ii,x 共几字节）
        input integer vlen;            // 值字节数（0 → 用 NULL）
        input [7:0] vtag;              // 值的 BER 标签（0x02 = INTEGER，0x04 = OCTET STRING）
        integer i2, n, olen, vbl, lst, pdu_len, seq_len;
        begin
            olen    = 9 + noid;                                  // OID 长度字段值（前缀 9 字节）
            vbl     = (2 + olen) + (vlen ? (2 + vlen) : 2);      // 一条绑定的内容长度
            lst     = 2 + vbl;                                   // 绑定列表内容长度
            pdu_len = 9 + lst;                                   // reqid+errstat+erridx 各 3
            seq_len = 14 + pdu_len;                              // ver 3 + community 9 + PDU 2
            n = 0;
            pl[n]=8'h30; pl[n+1]=seq_len[7:0];                            n=n+2;
            pl[n]=8'h02; pl[n+1]=8'h01; pl[n+2]=8'h00;                    n=n+3;  // version 0
            pl[n]=8'h04; pl[n+1]=8'h07;                                   n=n+2;  // community 7
            pl[n]=8'h70; pl[n+1]=8'h72; pl[n+2]=8'h69; pl[n+3]=8'h76;     // "priv"
            pl[n+4]=8'h61; pl[n+5]=8'h74; pl[n+6]=8'h65;                  n=n+7;  // "ate"
            pl[n]=pdu; pl[n+1]=pdu_len[7:0];                              n=n+2;
            pl[n]=8'h02; pl[n+1]=8'h01; pl[n+2]=8'h01;                    n=n+3;  // reqid = 1
            pl[n]=8'h02; pl[n+1]=8'h01; pl[n+2]=8'h00;                    n=n+3;  // error-status
            pl[n]=8'h02; pl[n+1]=8'h01; pl[n+2]=8'h00;                    n=n+3;  // error-index
            pl[n]=8'h30; pl[n+1]=lst[7:0];                                n=n+2;
            pl[n]=8'h30; pl[n+1]=vbl[7:0];                                n=n+2;
            pl[n]=8'h06; pl[n+1]=olen[7:0];                               n=n+2;
            pl[n]=8'h2B; pl[n+1]=8'h06; pl[n+2]=8'h01; pl[n+3]=8'h04;     // 1.3.6.1.4
            pl[n+4]=8'h01; pl[n+5]=8'hEB; pl[n+6]=8'h10; pl[n+7]=8'h85;   // .1 .13712(b128) .
            pl[n+8]=8'h18;                                                n=n+9;  // 664(b128)
            for (i2 = 0; i2 < noid; i2 = i2 + 1) begin pl[n] = oidb[i2]; n = n + 1; end
            if (vlen) begin
                pl[n]=vtag; pl[n+1]=vlen[7:0]; n=n+2;
                for (i2 = 0; i2 < vlen; i2 = i2 + 1) begin pl[n] = vbuf[i2]; n = n + 1; end
            end else begin
                pl[n]=8'h05; pl[n+1]=8'h00; n=n+2;                        // NULL
            end
            pl_len = n;
        end
    endtask


    integer pn;                        // 组包游标（模块级，便于 put_len 直接推进）
    task put_len;                      // 按最短形式写一个长度字段
        input integer v;
        begin
            if (v < 128)      begin pl[pn] = v[7:0];                     pn = pn + 1; end
            else if (v < 256) begin pl[pn]=8'h81; pl[pn+1]=v[7:0];        pn = pn + 2; end
            else              begin pl[pn]=8'h82; pl[pn+1]=v[15:8];
                                    pl[pn+2]=v[7:0];                      pn = pn + 3; end
        end
    endtask

    // 构造 GET 请求，含 n 条绑定；第 bad_at 条（1 起；0 = 全合法）的 OID 用**不存在**的 .1.9.0
    task inj_snmp_multi;
        input integer n;
        input integer bad_at;
        integer i3, vbl, lst, pdu_c, msg_c;
        begin
            for (i3 = 0; i3 < 2048; i3 = i3 + 1) pl[i3] = 8'h00;
            vbl   = 18 * n;                                    // 每条绑定 TLV = 2 + 14(OID) + 2(NULL)
            lst   = vbl + ((vbl < 128) ? 2 : (vbl < 256) ? 3 : 4);
            pdu_c = 9 + lst;
            msg_c = 3 + 9 + pdu_c + 2;                         // ver + community + PDU(TLV)
            pn = 0;
            pl[pn]=8'h30; pn=pn+1; put_len(msg_c);
            pl[pn]=8'h02; pl[pn+1]=8'h01; pl[pn+2]=8'h00; pn=pn+3;          // version 0
            pl[pn]=8'h04; pl[pn+1]=8'h06; pn=pn+2;                          // community "public"
            pl[pn]=8'h70; pl[pn+1]=8'h75; pl[pn+2]=8'h62;
            pl[pn+3]=8'h6C; pl[pn+4]=8'h69; pl[pn+5]=8'h63; pn=pn+6;
            pl[pn]=8'hA0; pn=pn+1; put_len(pdu_c);                          // GetRequest
            pl[pn]=8'h02; pl[pn+1]=8'h01; pl[pn+2]=8'h11; pn=pn+3;          // reqid = 0x11
            pl[pn]=8'h02; pl[pn+1]=8'h01; pl[pn+2]=8'h00; pn=pn+3;          // error-status
            pl[pn]=8'h02; pl[pn+1]=8'h01; pl[pn+2]=8'h00; pn=pn+3;          // error-index
            pl[pn]=8'h30; pn=pn+1; put_len(vbl);                            // 绑定列表
            for (i3 = 1; i3 <= n; i3 = i3 + 1) begin
                pl[pn]=8'h30; pl[pn+1]=8'd16; pn=pn+2;                      // 一条绑定（内容 16）
                pl[pn]=8'h06; pl[pn+1]=8'h0C; pn=pn+2;                      // OID TLV（12 字节）
                pl[pn]=8'h2B; pl[pn+1]=8'h06; pl[pn+2]=8'h01; pl[pn+3]=8'h04;
                pl[pn+4]=8'h01; pl[pn+5]=8'hEB; pl[pn+6]=8'h10;
                pl[pn+7]=8'h85; pl[pn+8]=8'h18;                             // 1.3.6.1.4.1.13712.664
                pl[pn+9]=8'h01;                                             // g = 1（esInfo）
                pl[pn+10]=(i3 == bad_at) ? 8'h09 : 8'h02;                   // i = 2 合法 / 9 不存在
                pl[pn+11]=8'h00; pn=pn+12;
                pl[pn]=8'h05; pl[pn+1]=8'h00; pn=pn+2;                      // NULL
            end
            pl_len = pn;
        end
    endtask


    task inj_frag;
        input [15:0] vlid, dport, sport;
        input [7:0]  sn;
        input [15:0] ipid;
        input [12:0] foff;
        input        mf;
        input integer plen;            // IP 载荷字节数（调用前填好 pl[]）
        integer b, total, iplen, snpos;
        reg [31:0] sum, cs, c;
        reg [15:0] flags_off;
        begin
            iplen = 20 + plen;
            for (b = 0; b < 300; b = b + 1) gf[b] = 8'h00;
            gf[0]=8'h03; gf[4]=vlid[15:8]; gf[5]=vlid[7:0];
            gf[6]=8'h02; gf[10]=8'h01; gf[11]=8'h20;
            gf[12]=8'h08; gf[13]=8'h00;
            // IP：总长 = 20+plen；ID；标志/片偏移
            gf[14]=8'h45; gf[16]=iplen[15:8]; gf[17]=iplen[7:0];
            gf[18]=ipid[15:8]; gf[19]=ipid[7:0];

            flags_off = {2'b0, mf, foff};
            gf[20]=flags_off[15:8]; gf[21]=flags_off[7:0];
            gf[22]=8'h01; gf[23]=8'h11;            // TTL=1 proto=UDP
            gf[26]=8'h0A; gf[27]=8'h00; gf[28]=8'h00; gf[29]=8'h09;
            gf[30]=8'hE0; gf[31]=8'hE0; gf[32]=vlid[15:8]; gf[33]=vlid[7:0];
            sum = 0;
            for (b = 14; b < 34; b = b + 2) sum = sum + {gf[b], gf[b+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cs = ~sum[15:0];
            gf[24]=cs[15:8]; gf[25]=cs[7:0];
            // IP 载荷（第一片由调用者把 UDP 头放在 pl[0..7]）
            for (b = 0; b < plen; b = b + 1) gf[34+b] = pl[b];
            snpos = 34 + plen;
            if (snpos < 59) snpos = 59;
            total = snpos + 1;
            gf[snpos] = sn;
            c = 32'hFFFFFFFF;
            for (b = 0; b < total; b = b + 1) c = rcrc32b(c, gf[b]);
            c = ~c;
            gf[total+0]=c[7:0]; gf[total+1]=c[15:8];
            gf[total+2]=c[23:16]; gf[total+3]=c[31:24];
            gf_len = total + 4;
            for (b = 0; b < gf_len; b = b + 1) begin
                @(negedge clk);
                rx_fv = 1; rx_fd = gf[b]; rx_fsop = (b == 0); rx_feop = (b == gf_len-1); rx_fnet = 1'b0;
                @(posedge clk);
            end
            @(negedge clk); rx_fv = 0; rx_fsop = 0; rx_feop = 0; rx_fnet = 1'b0;
        end
    endtask

    // 在抓到的帧 k 的载荷里找一段连续字节（返回绝对偏移，-1 = 没找到）
    function integer find_bytes;
        input integer base;
        input integer patlen;
        integer kk, jj; reg hit;
        begin
            find_bytes = -1;
            for (kk = base + 42; kk < base + 260 - patlen; kk = kk + 1) begin
                hit = 1;
                for (jj = 0; jj < patlen; jj = jj + 1)
                    if (fr[kk + jj] !== snmp_req[jj]) hit = 0;
                if (hit && find_bytes < 0) find_bytes = kk;
            end
        end
    endfunction

    function integer find_frame;
        input integer start;
        input [15:0]  vlid;
        integer k;
        begin
            find_frame = -1;
            for (k = start; k < nfr - 14; k = k + 1)
                if (find_frame < 0 &&
                    fr[k]==8'h03 && fr[k+1]==8'h00 && fr[k+2]==8'h00 && fr[k+3]==8'h00 &&
                    fr[k+4]==vlid[15:8] && fr[k+5]==vlid[7:0] &&
                    fr[k+12]==8'h08 && fr[k+13]==8'h00) find_frame = k;
        end
    endfunction


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

    function [31:0] resid;
        input integer base;              // 从哪一帧的头开始算（A 副本=0 / B 副本=F2）
        integer k; reg [31:0] c;
        begin
            c = 32'hFFFFFFFF;
            for (k = base; k < base + 64; k = k + 1) c = crc32b(c, fr[k]);
            resid = c;
        end
    endfunction

    function [15:0] cks;
        input integer dummy;
        integer k; reg [31:0] sum;
        begin
            sum = 0;
            for (k = 14; k < 34; k = k + 2)
                if (k != 24) sum = sum + {fr[k], fr[k+1]};
            sum = (sum & 32'hFFFF) + (sum >> 16);
            sum = (sum & 32'hFFFF) + (sum >> 16);
            cks = ~sum[15:0];
        end
    endfunction

    function [31:0] dres;      // 读 dRAM（4 条字节通道拼回）
        input [11:0] a;
        begin
            dres = {u_dut.dram3[a[11:2]], u_dut.dram2[a[11:2]],
                    u_dut.dram1[a[11:2]], u_dut.dram0[a[11:2]]};
        end
    endfunction

    // 615A 用例的载荷花样：第 block 块（1 起）的第 i 字节
    //   两块各不相同、块内递增，这样"块接错/字节错位/重复写"都会被抓出来
    function [7:0] pat8;
        input integer block;
        input integer i;
        begin
            pat8 = (block * 32 + i) & 8'hFF;
        end
    endfunction

    integer errs = 0;
    task chk; input c; input [255:0] n;
        begin if (c) $display("  [ok]   %0s", n);
              else begin errs=errs+1; $display("  [FAIL] %0s", n); end end
    endtask

    reg [31:0] v_d0, v_d1, v_pt, v_st, v_tx, v_len0, v_q0, v_q1, v_qpt, v_qlen, v_st2;

    initial begin
        #200 res = 1'b0;

        begin : wait_fw_selftest
            integer w;
            w = 0;
            while (dres(12'h3F0) !== 32'hA5A5C0DE && w < 1250000) begin
                @(posedge clk); w = w + 1;
            end
            chk(dres(12'h3F0) === 32'hA5A5C0DE,
                "固件开机自检在超时前跑完（轮询 RES(0xF0) 完成标志）");
            $display(" [自检] 等待 %0d 拍（原固定延时 11250 拍）", w);
        end

        $display("");
        $display("=========== SoC 端到端（三端口架构）===========");
        $display(" [帧] %0d 字节 DA=%02x%02x%02x%02x%02x%02x SA=%02x%02x%02x%02x%02x%02x type=%02x%02x",
                 nfr, fr[0],fr[1],fr[2],fr[3],fr[4],fr[5],
                 fr[6],fr[7],fr[8],fr[9],fr[10],fr[11],fr[12],fr[13]);
        $display(" [帧] IP len=%02x%02x cks=%02x%02x(重算%04x) TTL=%02x proto=%02x",
                 fr[16],fr[17],fr[24],fr[25],cks(0),fr[22],fr[23]);
        $display(" [帧] UDP %02x%02x -> %02x%02x len=%02x%02x",
                 fr[34],fr[35],fr[36],fr[37],fr[38],fr[39]);
        $display(" [帧] 载荷hex=%02x%02x%02x%02x%02x%02x%02x%02x SN=%02x FCS残差=%08x",
                 fr[42],fr[43],fr[44],fr[45],fr[46],fr[47],fr[48],fr[49],fr[50],resid(0));

        v_d0 = dres(12'h300); v_d1 = dres(12'h304); v_pt = dres(12'h308);
        v_st = dres(12'h30C); v_tx = dres(12'h310); v_len0 = dres(12'h314);
        v_q0 = dres(12'h318); v_q1 = dres(12'h31C); v_qpt = dres(12'h320);
        v_qlen = dres(12'h324); v_st2 = dres(12'h328);
        $display(" [固件] ★cfg_ld(0xD8)=%0d", dres(12'h3D8));
        $display(" [固件] d0=%08x d1=%08x portid=%04x status=%08x txstat=%08x len0=%0d",
                 v_d0, v_d1, v_pt, v_st, v_tx, v_len0);
        $display(" [固件] 队列口 q0=%08x q1=%08x portid=%04x len=%0d status=%08x",
                 v_q0, v_q1, v_qpt, v_qlen, v_st2);


        $display("=============================================");


        chk(dres(12'h3D8) === 32'd0,        "T5a ★flash 有魔数 → 配置表加载成功（cfg_ld=0）");
        chk(dres(12'h3DC) === 32'd1,        "T5b ★镜像里那条 VL 0x777 查表命中");
        chk(dres(12'h3E0) === 32'hE5BF0777, "T5c ★加载进来的 w0 与 flash 镜像逐位一致");
        chk(dres(12'h3E4) === 32'd1,        "T5d 预设 VL 0x123 仍在表里（加载没把整表清掉）");
        chk(dres(12'h3E8) === 32'hE5BF0123, "T5e ★预设表项内容未被破坏");
        //   镜像现在有**两项**：第 2 项（VLID 0xFFE，表尾附近）用来走通
        //   `for (i=0;i<n;i++) flash_read(8+i*20, ...)` 的**第 2 次迭代** ——
        //   那一轮要重算 flash 地址、重发命令、重装 FIFO，此前只测过 N=1，这条路径没走过。
        chk(dres(12'h3EC) === 32'd1,        "T5f ★镜像第 2 项（VLID 0xFFE）也加载成功（N>1 的迭代）");
        chk(dres(12'h378) === 32'hE5BF0FFE, "T5g ★第 2 项内容与镜像逐位一致");


        $display(" [诊断] txports: act_vld=%b act=%0d e_sent=%0d e_len=%0d e_stream=%b e_fetch=%b ppend0=%b pdone0=%b",
                 u_dut.u_txports.act_vld, u_dut.u_txports.act,
                 u_dut.u_txports.e_sent, u_dut.u_txports.e_len,
                 u_dut.u_txports.e_stream, u_dut.u_txports.e_fetch,
                 u_dut.u_txports.ppend[0], u_dut.u_txports.pdone[0]);
        $display(" [诊断] tx_path: txv=%b txready=%b txsop=%b txeop=%b replay=%b bcnt=%0d",
                 u_dut.txv, u_dut.txready, u_dut.txsop,
                 u_dut.txeop, 1'b0, u_dut.u_txports.e_bcnt);   // tx_replay 已废弃（恒 0）

        $display(" [诊断] CPU 访问 %0d 次：DADDR 在选通前已稳定 %0d 次；换端口 %0d 次，其中地址已提前稳定 %0d 次",
                 n_den, n_stable, n_port_chg, n_port_chg_stable);
        $display(" [诊断] CPU: PC=%08x IFPC=%08x IDPC=%08x IDREQ=%b IDACK=%b DDREQ=%b DDACK=%b HLT=%b",
                 u_dut.u_cpu.PC, u_dut.u_cpu.IFPC, u_dut.u_cpu.IDPC,
                 u_dut.IDREQ, u_dut.IDACK, u_dut.DDREQ, u_dut.DDACK_D, u_dut.u_cpu.HLT);

        $display(" [算术自检] (29+3)/4=%0d 1000/7=%0d 1000%%7=%0d 13*11=%0d 0x12345678/0x1000=%08x %%0x1000=%08x",
                 dres(12'h360), dres(12'h364), dres(12'h368), dres(12'h36C),
                 dres(12'h370), dres(12'h374));

        chk(dres(12'h360) === 32'd8,        "T-self0 ★软工具链自检：(29+3)/4 = 8");
        chk(dres(12'h364) === 32'd142,      "T-self1 ★软工具链自检：1000/7 = 142（__udivsi3）");
        chk(dres(12'h368) === 32'd6,        "T-self2 ★软工具链自检：1000%7 = 6（__umodsi3）");
        chk(dres(12'h36C) === 32'd143,      "T-self3 ★软工具链自检：13*11 = 143（__mulsi3）");
        chk(dres(12'h370) === 32'h00012345, "T-self4 ★软工具链自检：0x12345678/0x1000 = 0x12345");
        chk(dres(12'h374) === 32'h00000678, "T-self5 ★软工具链自检：0x12345678%0x1000 = 0x678");

        chk(dres(12'h380) === 32'd0,  "T24a 端口配置自检0错");
        chk(dres(12'h384) === 32'h43, "T24b 端口0读回 0x43");
        chk(dres(12'h388) === 32'h45, "T24c 端口1读回 0x45");
        chk(dres(12'h38C) === 32'h47, "T24d 端口4读回 0x47");

        chk(dres(12'h390) === 32'h15, "T24e ★改配后读回 0x15");
        chk(dres(12'h394) === 32'h47, "T24f ★恢复后读回 0x47");
        chk(nfr >= 64, "T1a 线侧组出了帧（>=64 字节，含 4 字节 FCS）");
        chk(fr[0]==8'h03 && fr[1]==8'h00 && fr[2]==8'h00 && fr[3]==8'h00 &&
            fr[4]==8'h01 && fr[5]==8'h23,
            "T1b DA = 03:00:00:00:01:23（组播常量域 + 预设 VLID 0x0123）");
        chk(fr[11]==8'h20, "T1c SA 的 Interface_ID = 1（A 副本）");
        chk(fr[12]==8'h08 && fr[13]==8'h00, "T1d EtherType = 0x0800");
        chk(fr[36]==8'h04 && fr[37]==8'h00,
            "T1e UDP 目的端口 = 1024（预设：采样口，AFDX 口 >=1024）");
        chk(fr[42]==8'h41 && fr[43]==8'h46 && fr[44]==8'h44 && fr[45]==8'h58 &&
            fr[46]==8'h31 && fr[47]==8'h32 && fr[48]==8'h00 && fr[49]==8'h00,
            "T1f 载荷 = 'AFDX' + '12'（固件写进采样口的）");

        base_rtc = nfr;                            // 记住注入前抓到的字节数（应答帧从这儿开始）
        inject_en = 1;
        repeat (4) @(posedge clk);
        inj_rtc;                                   // 源 10.0.0.9:0x1234 → 目的 3001
        repeat (75000) @(posedge clk);              // 等固件在服务循环里处理并回帧（含 BAG 与 A/B 两帧）
        inject_en = 0;
        n_rtc = nfr - base_rtc;                    // 应答帧的总字节数（应为 128 = A 64 + B 64）

        if (n_rtc >= 64) begin
            $display(" [RTC] 应答帧 %0d 字节 DA=%02x%02x%02x%02x%02x%02x IP dst=%0d.%0d.%0d.%0d UDP %04x->%04x 载荷=%c%c%c%c %02x%02x%02x%02x 残差=%08x",
                     n_rtc, fr[base_rtc+0],fr[base_rtc+1],fr[base_rtc+2],fr[base_rtc+3],fr[base_rtc+4],fr[base_rtc+5],
                     fr[base_rtc+30],fr[base_rtc+31],fr[base_rtc+32],fr[base_rtc+33],
                     {fr[base_rtc+34],fr[base_rtc+35]}, {fr[base_rtc+36],fr[base_rtc+37]},
                     fr[base_rtc+42],fr[base_rtc+43],fr[base_rtc+44],fr[base_rtc+45],
                     fr[base_rtc+46],fr[base_rtc+47],fr[base_rtc+48],fr[base_rtc+49],
                     resid(base_rtc));
        end

        chk(n_rtc >= 64,
            "T7a ★RTC 服务回了帧（SAP 口 4 收到请求 → 固件在服务循环里应答）");
        chk(fr[base_rtc+0]==8'h03 && fr[base_rtc+1]==8'h00 && fr[base_rtc+2]==8'h00 &&
            fr[base_rtc+3]==8'h00 && fr[base_rtc+4]==8'h0B && fr[base_rtc+5]==8'hC5,
            "T7b RTC 应答帧 DA = 03:00:00:00:0B:C5（VL 0x0BC5，预设给 SAP-RTC）");

        chk(fr[base_rtc+30]==8'h0A && fr[base_rtc+31]==8'h00 &&
            fr[base_rtc+32]==8'h00 && fr[base_rtc+33]==8'h09,
            "T7c ★应答的目的 IP = 10.0.0.9 = **请求方的源 IP**（SAP 语义：目的地址由分区给）");
        chk(fr[base_rtc+36]==8'h12 && fr[base_rtc+37]==8'h34,
            "T7d ★应答的目的 UDP 端口 = 0x1234 = **请求方的源端口**（不是配置表里的 69/161/3001）");
        chk(fr[base_rtc+42]==8'h52 && fr[base_rtc+43]==8'h54 &&   // 'R','T'
            fr[base_rtc+44]==8'h43 && fr[base_rtc+45]==8'h31,     // 'C','1'
            "T7e 应答载荷前 4 字节 = \"RTC1\"（RTC 服务标识）");
        chk(resid(base_rtc) === 32'hDEBB20E3, "T7f RTC 应答帧 FCS 残差 = DEBB20E3（自算 FCS 正确）");

        // ---- 消息真的路由到了端口 4，而且被服务取走了 ----
        // （按端口隔离的 STATUS 有多服务轮询的专项单元测试，在 tb_rx_ports 里）
        chk(saw_rtc_msg === 1'b1,
            "T7g 注入的 RTC 请求帧路由到了 SAP 口 4（pcnt[4] 曾非 0）");
        chk(u_dut.u_rxports.pcnt[4] === 0,
            "T7h 端口 4 的消息已被 RTC 服务取走（pcnt[4] 归 0，没有卡住）");
        chk(u_dut.u_rxports.pcnt[0] === 0 && u_dut.u_rxports.pcnt[1] === 0,
            "T7i 采样口/队列口没有被 RTC 服务误碰（仍是 0 条待读）");

        chk(fr[59]==8'h00 && fr[50]==8'h00 && fr[58]==8'h00,
            "T1g SN 在帧末前 4 字节（=59），符合 P7/Wireshark 的 SN 位置");
        chk(resid(0) === 32'hDEBB20E3, "T1h FCS 残差 = DEBB20E3");

        // ---- B 副本：AFDX 冗余的另一张网（P7 §3.2.6）----
        chk(nfr >= 2*F2, "T1i 一条消息连出 A、B 两帧（>=128 字节，标准冗余双发）");
        chk(fr[F2+4]==8'h01 && fr[F2+5]==8'h23, "T1j B 副本 DA 同 VL（VLID 0x0123）");
        chk(fr[F2+11]==8'h40, "T1k B 副本 SA 的 Interface_ID = 2（B 网）");
        chk(resid(F2) === 32'hDEBB20E3, "T1l B 副本 FCS 残差 = DEBB20E3");

        chk(v_d0 === 32'h58444641, "T2a 固件读回 RX_DATA 第 1 字 = AFDX");
        chk(v_d1 === 32'h00003231, "T2b 固件读回 RX_DATA 第 2 字 = 12");
        chk(v_pt === 32'h00000400, "T2c 固件读回 PORTID = 1024");
        chk(v_st[0] === 1'b1,      "T2d 读时 STATUS.bit0 = 有数据");

        chk(v_tx[1] === 1'b1 && v_tx[2] === 1'b0,
            "T3a TX_STATUS：完成位 bit1 已置起、无待发（释放接收没有顺带再提交发送）");
        chk(u_dut.u_txports.ppend[0] === 1'b0 && u_dut.u_txports.pdone[0] === 1'b1,
            "T3b 发送端口层终态：ppend=0 且 pdone=1（消息发完，没被重新武装）");

        chk(fr[2*F2+4]==8'h04 && fr[2*F2+5]==8'h56 && fr[2*F2+11]==8'h20,
            "T3c 第 3 帧是队列口 VL 0x0456 的 A 副本（DA=03:00:00:00:04:56，IFID=1）");
        chk(resid(2*F2) === 32'hDEBB20E3, "T3d 第 3 帧 FCS 残差 = DEBB20E3");


        chk(v_len0 === 32'd8, "T6a 端口 0 消息的 P_LEN = 8（LEN 译码原来恒 0）");
        chk(v_q0 === 32'h11223344 && v_q1 === 32'h55667788,
            "T6b ★队列口载荷读回正确（跨端口读：先读端口 0 再读端口 1）");
        chk(v_qpt === 32'h00000401, "T6c ★队列口 PORTID = 1025（VL 0x456 的 dport，buf_idx=1 路由生效）");
        chk(v_qlen === 32'd8, "T6d 队列口消息长度 = 8");
        chk(v_st2[0] === 1'b1, "T6e 读队列口时 STATUS.bit0 = 有数据");

        chk(base_rtc === 4*F2, "T6f 固件两条消息正好出 4 帧（两条 × A/B 各一份），无多余帧");


        inject_en = 1;
        repeat (4) @(posedge clk);
        inj_frame(16'h9999);                       // 表里没有这条 VL
        repeat (60) @(posedge clk);
        chk(u_dut.erl_pend === 1'b0, "T4 ★未配置 VLID 的帧之后，提前查表没有卡住（TX 不会被饿死）");
        // 固件结尾已经释放过端口 0（pcnt 归 0），所以这里 0 = 那条未配置 VLID 的帧没被投递
        chk(u_dut.u_rxports.pcnt[0] === 4'd0,
            "T5 未配置 VLID 的帧被 IC/RM 过滤（端口 0 没有多出消息）");
        inject_en = 0;
        repeat (10) @(posedge clk);


        inject_en = 1;
        repeat (4) @(posedge clk);
        base_tftp = nfr;

        // --- 1) WRQ "load.bin" / octet / blksize=128 ---
        pl_len = 29;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h02;                                  // opcode 2 = WRQ
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;    // "load"
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;    // ".bin"
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;  // "octet"
        pl[17]=8'h62; pl[18]=8'h6C; pl[19]=8'h6B; pl[20]=8'h73;    // "blksize"
        pl[21]=8'h69; pl[22]=8'h7A; pl[23]=8'h65;
        pl[25]=8'h31; pl[26]=8'h32; pl[27]=8'h38;                  // "128"
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd0);    // WRQ：SN=0（复位后的第一帧）
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h0006626C, 64, 750000);           // 期望 OACK：00 06 'b' 'l'（blksize）
        chk(k >= 0, "T8a ★615A：WRQ 得到应答帧（SAP 口 2 的 VLAN 0x0789）");
        if (k >= 0) begin
            chk({fr[k+36],fr[k+37]} === 16'h0BB8,
                "T8b 应答回给**请求方的源 UDP 端口**（0x0BB8，SAP 语义同 RTC）");
            chk(fr[k+42]==8'h00 && fr[k+43]==8'h06,
                "T8c ★OACK（opcode 6，RFC 2347 选项协商）");
            chk(fr[k+44]==8'h62 && fr[k+45]==8'h6C && fr[k+46]==8'h6B && fr[k+47]==8'h73 &&
                fr[k+48]==8'h69 && fr[k+49]==8'h7A && fr[k+50]==8'h65 && fr[k+51]==8'h00 &&
                fr[k+52]==8'h31 && fr[k+53]==8'h32 && fr[k+54]==8'h38 && fr[k+55]==8'h00,
                "T8d OACK 回显协商后的 blksize = 128");
        end

        // --- 2) DATA(1)：128 字节（正好一块，不是末块）---
        repeat (137500) @(posedge clk);               // 先等过 1 ms 的 BAG
        base_tftp = nfr;
        pl_len = 132;
        pl[0]=8'h00; pl[1]=8'h03; pl[2]=8'h00; pl[3]=8'h01;        // opcode 3, block 1
        for (m = 0; m < 128; m = m + 1) pl[4+m] = pat8(1, m);
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd1);    // DATA(1)：SN=1
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00040001, 64, 750000);           // 期望 ACK(1)：00 04 00 01
        chk(k >= 0 && fr[k+42]==8'h00 && fr[k+43]==8'h04 && fr[k+44]==8'h00 && fr[k+45]==8'h01,
            "T8e 收到 DATA(1) → 回 ACK(1)（opcode 4，块号 1）");

        // --- 3) DATA(2)：40 字节（不足一块 ⇒ 末块，传输结束）---
        base_tftp = nfr;
        pl_len = 44;
        pl[0]=8'h00; pl[1]=8'h03; pl[2]=8'h00; pl[3]=8'h02;        // opcode 3, block 2
        for (m = 0; m < 40; m = m + 1) pl[4+m] = pat8(2, m);
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd2);    // DATA(2)：SN=2
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00040002, 64, 750000);           // 期望 ACK(2)：00 04 00 02
        chk(k >= 0 && fr[k+42]==8'h00 && fr[k+43]==8'h04 && fr[k+44]==8'h00 && fr[k+45]==8'h02,
            "T8f 收到末块 DATA(2) → 回 ACK(2)");

        // --- 4) RRQ "load.bin"（无选项 ⇒ 块大小回默认 512）---
        base_tftp = nfr;
        pl_len = 17;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h01;                                  // opcode 1 = RRQ
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd3);    // RRQ：SN=3
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00030001, 220, 750000);           // 期望 DATA(1)：00 03 00 01
        chk(k >= 0 && fr[k+42]==8'h00 && fr[k+43]==8'h03 &&
            fr[k+44]==8'h00 && fr[k+45]==8'h01,
            "T8g ★RRQ → 回 DATA(1)（opcode 3，块号 1）");
        // 逐字节对：RRQ 发出的 168 字节必须**正好是**第 2/3 步写进去的内容
        okcnt = 0;
        if (k >= 0) begin
            for (m = 0; m < 168; m = m + 1)
                if (fr[k+46+m] === pat8(m < 128 ? 1 : 2, m < 128 ? m : (m - 128))) okcnt = okcnt + 1;
        end
        chk(okcnt === 168, "T8h ★★RRQ 读回的 168 字节与 WRQ 写入的**逐字节一致**（真落盘、真卸载）");

        // 固件结果区里的文件大小 / 前几个字节（证明文件区状态对固件也是可见的）
        chk(dres(12'h330) === 32'd168, "T8i 固件报的文件大小 = 168 字节");
        chk(dres(12'h334) === {pat8(1,3),pat8(1,2),pat8(1,1),pat8(1,0)},
            "T8j 固件读文件前 4 字节 = 刚写入的内容（不是上电的演示文件）");


        base_tftp = nfr;
        pl_len = 29;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h02;                                  // opcode 2 = WRQ
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        pl[17]=8'h62; pl[18]=8'h6C; pl[19]=8'h6B; pl[20]=8'h73;
        pl[21]=8'h69; pl[22]=8'h7A; pl[23]=8'h65;
        pl[25]=8'h31; pl[26]=8'h32; pl[27]=8'h38;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd4);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h0006626C, 64, 750000);
        chk(k >= 0, "T14a 第二轮 WRQ(blksize=128) 得到 OACK");

        // --- 14b) DATA(1) = **正好满块 128 字节**（不是末块）---
        base_tftp = nfr;
        pl_len = 132;
        pl[0]=8'h00; pl[1]=8'h03; pl[2]=8'h00; pl[3]=8'h01;
        for (m = 0; m < 128; m = m + 1) pl[4+m] = pat8(5, m);
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd5);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00040001, 64, 750000);
        chk(k >= 0, "T14b 满块 DATA(1) → 回 ACK(1)");

        // --- 14c) DATA(2) = **0 字节空块**（RFC 1350 的结束标志）→ ACK(2)，文件大小 = 128 ---
        base_tftp = nfr;
        pl_len = 4;
        pl[0]=8'h00; pl[1]=8'h03; pl[2]=8'h00; pl[3]=8'h02;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd6);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00040002, 64, 750000);
        chk(k >= 0, "T14c 空块 DATA(2) → 回 ACK(2)（文件落盘 128 字节）");


        base_tftp = nfr;
        pl_len = 29;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h01;                                  // opcode 1 = RRQ（**带选项**）
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        pl[17]=8'h62; pl[18]=8'h6C; pl[19]=8'h6B; pl[20]=8'h73;
        pl[21]=8'h69; pl[22]=8'h7A; pl[23]=8'h65;
        pl[25]=8'h31; pl[26]=8'h32; pl[27]=8'h38;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd7);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h0006626C, 64, 750000);
        chk(k >= 0, "T14d ★第二条**带选项**的 RRQ 得到 OACK（snd_block 已在用过的前提下）");


        base_tftp = nfr;
        pl_len = 4;
        pl[0]=8'h00; pl[1]=8'h04; pl[2]=8'h00; pl[3]=8'h00;        // opcode 4 = ACK，块号 0
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd8);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00030001, 200, 1000000);
        chk(k >= 0 && fr[k+42]==8'h00 && fr[k+43]==8'h03 &&
            fr[k+44]==8'h00 && fr[k+45]==8'h01,
            "T14e ★★OACK 之后的 ACK(0) 被受理 → 收到 DATA(1)（不再卡死）");
        okcnt = 0;
        if (k >= 0)
            for (m = 0; m < 128; m = m + 1)
                if (fr[k+46+m] === pat8(5, m)) okcnt = okcnt + 1;
        chk(okcnt === 128, "T14f ★DATA(1) 的 128 字节与写入内容逐字节一致");


        base_tftp = nfr;
        pl_len = 4;
        pl[0]=8'h00; pl[1]=8'h04; pl[2]=8'h00; pl[3]=8'h01;        // ACK，块号 1
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd9);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h00030002, 64, 1000000);
        chk(k >= 0 && fr[k+42]==8'h00 && fr[k+43]==8'h03 &&
            fr[k+44]==8'h00 && fr[k+45]==8'h02,
            "T14g ★★ACK(1) 之后补发空块 DATA(2)（整数倍长度的收尾块）");
        //   用 UDP 长度区分"空块"（8+4=12）与"满块"（8+128=136）—— 只看前 4 字节区分不出来
        chk(k >= 0 && {fr[k+38],fr[k+39]} === 16'd12,
            "T14h ★★该 DATA(2) 的 UDP 长度 = 12（载荷 4 字节 = 真正的**空块**）");
        chk(dres(12'h330) === 32'd128, "T14i 固件报的文件大小 = 128 字节（空块没把它改坏）");

        inject_en = 0;
        repeat (10) @(posedge clk);


        inject_en = 1;
        repeat (4) @(posedge clk);

        // --- GET esVlNum（.1.5.0）→ GetResponse，值应 = 已配置 VL 条数（预设 5 条）---
        inj_snmp(8'd1, 8'd5, 8'd0, 8'hA0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd4);   // VL 0x0161 / UDP 161，SN=4

        wscan = 0;
        while (!((dres(12'h398) & 32'h80000000)) && wscan < 2500000) begin
            @(posedge clk); wscan = wscan + 1;
        end
        $display(" [P0] scan-active seen after %0d cycles (flag=%08x)", wscan, dres(12'h398));
        inj_snmp(8'd1, 8'd4, 8'd0, 8'hA1);
        pl[20] = 8'd2;                                  // request-id = 2
        inj_net = 1'b0; inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd5);
        inj_net = 1'b1; inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd5);
        inj_net = 1'b0;
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 10000000);
        $display(" [SNMP诊断] 请求=%0d 应答=%0d 出错=%0d 阶段码=%0d PDU=%02x | nfr=%0d",
                 dres(12'h3C4), dres(12'h3C8), dres(12'h3CC), dres(12'h3D0), dres(12'h3D4), nfr);
        chk(k >= 0, "T9a ★SNMP：GET esVlNum 得到应答帧（SAP 口 3 / VL 0x0161）");
        chk(k >= 0 && fr[k+42+13] === 8'hA2, "T9b 应答 PDU = GetResponse(0xA2)");
        // 报文布局（GetResponse）：[13,14]=A2 len | [15..17]=reqid | [18..20]=errstat
        //   [21..23]=erridx | [24,25]=30 len(列表) | [26,27]=30 len(绑定) | [28,29]=06 0C
        //   [30..41]=OID | [42..44]=值
        chk(k >= 0 && fr[k+42+28]==8'h06 && fr[k+42+29]==8'h0C &&
            fr[k+42+39]==8'h01 && fr[k+42+40]==8'h05 && fr[k+42+41]==8'h00 &&

            fr[k+42+42]==8'h02 &&
            ((fr[k+42+43]==8'h01 && fr[k+42+44]==N_VL_TABLE) ||
             (fr[k+42+43]==8'h02 && fr[k+42+44]==8'h00 && fr[k+42+45]==N_VL_TABLE)),
            "T9c ★varbind = 原 OID 回显(.1.5.0) + INTEGER = 表里 VL 条数（预设 9 + flash 2）");
        chk(k >= 0 && fr[k+42+20] === 8'h00, "T9d error-status = 0（noError）");

        // --- 第二条请求（T9z 注入的 A/B 对）的应答：reqid=2，GETNEXT .1.4.0 → .1.5.0 ---
        repeat (137500) @(posedge clk);
        scan_pos = k + 1;                               // 跳过第一条应答（及其 B 副本）
        expect_snmp_reqid(16'h0161, 8'd2, 96, 5000000);
        chk(k >= 0 && fr[k+42+17] === 8'd2,
            "T9e2 ★扫表期间 A/B 对的第二条请求得到 reqid=2 的应答（B 副本被 RM 去重）");
        chk(k >= 0 && fr[k+42+39]==8'h01 && fr[k+42+40]==8'h05 && fr[k+42+41]==8'h00 &&

            fr[k+42+42]==8'h02 &&
            ((fr[k+42+43]==8'h01 && fr[k+42+44]==N_VL_TABLE) ||
             (fr[k+42+43]==8'h02 && fr[k+42+44]==8'h00 && fr[k+42+45]==N_VL_TABLE)),
            "T9e ★GETNEXT 从 .1.4.0 走到 .1.5.0（返回的是**下一个**对象）");

        chk(dres(12'h398) === 32'd47,
            "T9e3 ★扫表期间 A/B 对的请求长度 = 47（未被 B 副本污染）");

        // --- GET 一个不存在的对象（.9.9.0）→ noSuchName(2) ---
        repeat (137500) @(posedge clk);
        inj_snmp(8'd9, 8'd9, 8'd0, 8'hA0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd6);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+20] === 8'h02 && fr[k+42+23] === 8'h01 &&
            fr[k+42+24] === 8'h30 && fr[k+42+25] === 8'h00,
            "T9f ★不存在的对象 → error-status=noSuchName(2)、error-index=1、varbind 为空");

        // --- GET .1.1.0（esId）→ 值 = 1 ---
        repeat (137500) @(posedge clk);
        inj_snmp(8'd1, 8'd1, 8'd0, 8'hA0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd7);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+39]==8'h01 && fr[k+42+40]==8'h01 && fr[k+42+41]==8'h00 &&
            fr[k+42+42]==8'h02 && fr[k+42+43]==8'h01 && fr[k+42+44]==8'h01,
            "T9g GET esId(.1.1.0) → INTEGER 1");

        // --- 服务计数（固件侧自证）---

        inject_en = 0;
        repeat (2500) @(posedge clk);          // 等固件把服务循环跑完，结果区才是"最终值"


        begin : t9cnt
            integer w9;
            w9 = 0;
            while (((dres(12'h3C4) < 32'd4) || (dres(12'h3C8) < 32'd4)) && w9 < 2500000) begin
                @(posedge clk); w9 = w9 + 1;
            end
            chk(dres(12'h3C4) >= 32'd4, "T9h 固件收到 >= 4 条 SNMP 请求");
            chk(dres(12'h3C8) >= 32'd4, "T9i 固件回了 >= 4 条 SNMP 应答");
        end


        inject_en = 1;
        repeat (4) @(posedge clk);
        // --- 15a/b) SET .9.1.0 (esCfgIrqEn) = 7 → noError 且回显 7 ---
        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd1; oidb[2]=8'd0;
        vbuf[0]=8'd7;
        inj_snmp_priv(8'hA3, 3, 1, 8'h02);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd8);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+19]==8'h02 && fr[k+42+21]==8'h00,
            "T15a ★SET .9.1.0 = 7 → error-status = 0（noError）");
        chk(k >= 0 && fr[k+42+45]==8'h07, "T15b ★应答里回显被设置的值 = 7");

        // --- 15c) GET .9.1.0 回读 7（证明真的落地进 IRQ_EN 了）---
        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd1; oidb[2]=8'd0;
        inj_snmp_priv(8'hA0, 3, 0, 8'h05);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd9);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+45]==8'h07,
            "T15c ★★GET 回读 .9.1.0 = 7（SET 真的写进了寄存器，不是吃个 T_NULL 就走）");

        // --- 15d/e) SET .9.3.<VLID 0x123> = 40（该 VL 的 Lmax），再用**另一条路径**读回 ---
        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd3; oidb[2]=8'h82; oidb[3]=8'h23;   // 291 = 0x123（base128）
        vbuf[0]=8'd40;
        inj_snmp_priv(8'hA3, 4, 1, 8'h02);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd10);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'h00, "T15d ★SET .9.3.<0x123>（该 VL 的 Lmax）= 40 → noError");

        repeat (137500) @(posedge clk);
        oidb[0]=8'd5; oidb[1]=8'd3; oidb[2]=8'h82; oidb[3]=8'h23;   // .5.3 = vlTable.vlLmax
        inj_snmp_priv(8'hA0, 4, 0, 8'h05);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd11);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+46]==8'd40,
            "T15e ★★GET .5.3.<0x123>（vlTable，**另一条代码路径**）读到 40 → 配置表真被改写");

        // --- 15f) SET 只读对象（.1.1.0 esId）→ readOnly(4) ---
        repeat (137500) @(posedge clk);
        oidb[0]=8'd1; oidb[1]=8'd1; oidb[2]=8'd0;
        vbuf[0]=8'd5;
        inj_snmp_priv(8'hA3, 3, 1, 8'h02);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd12);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'h04 && fr[k+42+24]==8'h01,
            "T15f ★SET 只读对象 → error-status = readOnly(4)、error-index = 1");

        // --- 15g) SET 越界值（.9.1.0 = 99，合法 0..31）→ badValue(3) ---
        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd1; oidb[2]=8'd0;
        vbuf[0]=8'd99;
        inj_snmp_priv(8'hA3, 3, 1, 8'h02);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd13);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'h03, "T15g ★SET 越界值 → error-status = badValue(3)");

        // --- 15h/i) SET .9.2.0 = OCTET STRING "AFDX"（4 字节）→ 再 GET 读回 ---
        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd2; oidb[2]=8'd0;
        vbuf[0]=8'h41; vbuf[1]=8'h46; vbuf[2]=8'h44; vbuf[3]=8'h58;   // "AFDX"
        inj_snmp_priv(8'hA3, 3, 4, 8'h04);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd14);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'h00, "T15h ★SET .9.2.0 字符串 = noError");

        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd2; oidb[2]=8'd0;
        inj_snmp_priv(8'hA0, 3, 0, 8'h05);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd15);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+43]==8'h04 && fr[k+42+44]==8'h04 &&
                    fr[k+42+45]==8'h41 && fr[k+42+48]==8'h58,
            "T15i ★★GET .9.2.0 读回 OCTET STRING 长度 4、内容 AFDX（逐字节）");
        inject_en = 0;

        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd3; oidb[2]=8'h82; oidb[3]=8'h23;
        vbuf[0]=8'h05; vbuf[1]=8'hDC;                       // 1500 = 0x05DC（大端）
        inj_snmp_priv(8'hA3, 4, 2, 8'h02);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd16);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'h03,
            "T15j ★SET Lmax = 1500 → badValue(3)（上限 1471 = 帧长 1518 的载荷上限）");


        repeat (137500) @(posedge clk);
        for (m = 0; m < 64; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h30; pl[1]=8'h12;
        pl[2]=8'h02; pl[3]=8'h84; pl[4]=8'hFF; pl[5]=8'hFF; pl[6]=8'hFF; pl[7]=8'hFF;
        pl[8]=8'h04; pl[9]=8'h06;
        pl[10]=8'h70; pl[11]=8'h75; pl[12]=8'h62; pl[13]=8'h6C; pl[14]=8'h69; pl[15]=8'h63;
        pl[16]=8'hA0; pl[17]=8'h00;
        pl_len = 18;
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd17);
        repeat (50000) @(posedge clk);
        inj_snmp(8'd1, 8'd5, 8'd0, 8'hA0);                 // 合法 GET esVlNum（SN=18）
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd18);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 3750000);
        chk(k >= 0,
            "T15k ★★注入「长度字段声明 0xFFFFFFFF」的畸形报文后，固件仍在正常应答（轮询没被卡死）");


        repeat (137500) @(posedge clk);
        inj_snmp_multi(14, 0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd19);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 512, 5000000);
        begin : t15l
            integer o2, p2, cnt2, q2;
            o2 = k + 42;
            p2 = o2 + 1 + lsz(o2 + 1);                  // 跳过外层 SEQUENCE 头
            p2 = p2 + 1 + lsz(p2+1) + lv(p2+1);         // version
            p2 = p2 + 1 + lsz(p2+1) + lv(p2+1);         // community
            p2 = p2 + 1 + lsz(p2+1);                    // PDU 头（**只跳 tag+长度字段**）→ 指向 reqid
            p2 = p2 + 1 + lsz(p2+1) + lv(p2+1);         // reqid
            p2 = p2 + 1 + lsz(p2+1) + lv(p2+1);         // error-status
            p2 = p2 + 1 + lsz(p2+1) + lv(p2+1);         // error-index
            q2 = lv(p2 + 1);                            // 绑定列表声明的内容长度
            p2 = p2 + 1 + lsz(p2 + 1);                  // 列表内容起点
            cnt2 = 0;
            while ((cnt2 < 32) && (fr[p2] === 8'h30) && ((p2 - o2) < q2 + 8)) begin
                p2   = p2 + 1 + lsz(p2+1) + lv(p2+1);   // 走完一条绑定
                cnt2 = cnt2 + 1;
            end
        chk(cnt2 === 14, "T15m ★★按声明的长度能完整走完 14 条绑定（长度字段与实际一致）");
        end


        repeat (137500) @(posedge clk);
        for (m = 0; m < 64; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h30; pl[1]=8'h2D;                              // SEQUENCE (45)
        pl[2]=8'h02; pl[3]=8'h01; pl[4]=8'h00;                 // version 0
        pl[5]=8'h04; pl[6]=8'h06;                              // community "public"
        pl[7]=8'h70; pl[8]=8'h75; pl[9]=8'h62; pl[10]=8'h6C; pl[11]=8'h69; pl[12]=8'h63;
        pl[13]=8'hA0; pl[14]=8'h20;                            // GetRequest (32)
        pl[15]=8'h02; pl[16]=8'h04;                            // reqid INTEGER，长度 4
        pl[17]=8'hFF; pl[18]=8'hFF; pl[19]=8'hFF; pl[20]=8'hFF;
        pl[21]=8'h02; pl[22]=8'h01; pl[23]=8'h00;              // error-status
        pl[24]=8'h02; pl[25]=8'h01; pl[26]=8'h00;              // error-index
        pl[27]=8'h30; pl[28]=8'h12;                            // varbind list (18)
        pl[29]=8'h30; pl[30]=8'h10;                            // varbind (16)
        pl[31]=8'h06; pl[32]=8'h0C;                            // OID (12)
        pl[33]=8'h2B; pl[34]=8'h06; pl[35]=8'h01; pl[36]=8'h04; pl[37]=8'h01;
        pl[38]=8'hEB; pl[39]=8'h10; pl[40]=8'h85; pl[41]=8'h18;
        pl[42]=8'h01; pl[43]=8'h05; pl[44]=8'h00;              // .1.5.0（esVlNum）
        pl[45]=8'h05; pl[46]=8'h00;                            // NULL
        pl_len = 47;
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd20);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 3750000);
        chk(k >= 0 && fr[k+42+15]===8'h02 && fr[k+42+16]===8'h04 &&
                    fr[k+42+17]===8'hFF && fr[k+42+20]===8'hFF,
            "T15p ★★request-id = 0xFFFFFFFF 的**合法**报文也得到应答（不再被带内哨兵误杀）");


        repeat (137500) @(posedge clk);
        inj_snmp_multi(12, 12);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd21);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 512, 5000000);
        begin : t15n
            integer o3, p3;
            o3 = k + 42;
            p3 = o3 + 1 + lsz(o3 + 1);                  // 跳过外层 SEQUENCE 头
            p3 = p3 + 1 + lsz(p3+1) + lv(p3+1);         // version
            p3 = p3 + 1 + lsz(p3+1) + lv(p3+1);         // community
            p3 = p3 + 1 + lsz(p3+1);                    // PDU 头（**只跳 tag+长度字段**）→ 指向 reqid
            p3 = p3 + 1 + lsz(p3+1) + lv(p3+1);         // reqid → 现在指向 error-status
            chk(k >= 0 && fr[p3] === 8'h02 && fr[p3+1] === 8'h01 && fr[p3+2] === 8'h02,
                "T15n ★★error-status 仍落在正确的字节上 = noSuchName(2)（长度右移后指针失效过）");
            chk(k >= 0 && fr[p3+3] === 8'h02 && fr[p3+4] === 8'h01 && fr[p3+5] === 8'd12,
                "T15o ★★error-index = 12（第 12 条绑定）");
        end


        inject_en = 1; repeat (4) @(posedge clk);

        // ---- 16a) RRQ(blksize=128) → OACK ----
        base_tftp = nfr;
        pl_len = 29;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h01;                                  // RRQ
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;    // "load"
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;    // ".bin"
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        pl[17]=8'h62; pl[18]=8'h6C; pl[19]=8'h6B; pl[20]=8'h73;
        pl[21]=8'h69; pl[22]=8'h7A; pl[23]=8'h65;
        pl[25]=8'h31; pl[26]=8'h32; pl[27]=8'h38;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd0);
        scan_pos = base_tftp;
        expect_reply(16'h0789, 32'h0006626C, 64, 750000);
        $display("  [p] T16a: k=%0d nfr=%0d scan_pos=%0d | 首帧前 8 载荷=%02x %02x %02x %02x %02x %02x %02x %02x",
                 k, nfr, scan_pos,
                 fr[k+42],fr[k+43],fr[k+44],fr[k+45],fr[k+46],fr[k+47],fr[k+48],fr[k+49]);
        chk(k >= 0, "T16a RRQ(blksize=128) → OACK");

        // ---- 16b) ACK(0) → DATA(1) ----
        repeat (162500) @(posedge clk);
        pl_len = 4; pl[0]=8'h00; pl[1]=8'h04; pl[2]=8'h00; pl[3]=8'h00;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd1);
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h00030001, 200, 750000);
        chk(k >= 0, "T16b ACK(0) → DATA(1)（128 字节满块）");


        repeat (562500) @(posedge clk);              // 4.5 ms ≈ RTO(4ms) + 余量

        scan_pos = nfr;
        expect_reply(16'h0789, 32'h00030001, 200, 750000);
        chk(k >= 0, "T16c ★★DATA(1) 超时后被**重传**（停等协议必须有重传）");

        // ---- 16d) 补上 ACK(1) → 会话照常继续（128 字节是满块 → 应补发**空块** DATA(2)）----
        repeat (162500) @(posedge clk);
        pl_len = 4; pl[0]=8'h00; pl[1]=8'h04; pl[2]=8'h00; pl[3]=8'h01;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd2);
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h00030002, 64, 750000);
        chk(k >= 0 && {fr[k+38],fr[k+39]} === 16'd12,
            "T16d ★重传后会话照常继续（ACK(1) → 空块 DATA(2)，UDP 长度 = 12）");

        // ---- 16e) WRQ(blksize=128) → OACK ----
        repeat (162500) @(posedge clk);
        pl_len = 29;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h02;                                  // WRQ
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        pl[17]=8'h62; pl[18]=8'h6C; pl[19]=8'h6B; pl[20]=8'h73;
        pl[21]=8'h69; pl[22]=8'h7A; pl[23]=8'h65;
        pl[25]=8'h31; pl[26]=8'h32; pl[27]=8'h38;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd3);
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h0006626C, 64, 750000);
        chk(k >= 0, "T16e WRQ(blksize=128) → OACK");

        // ---- 16f) DATA(1)（8 字节短块 = 末块）→ ACK(1) ----
        repeat (162500) @(posedge clk);
        pl_len = 12; pl[0]=8'h00; pl[1]=8'h03; pl[2]=8'h00; pl[3]=8'h01;
        for (m = 0; m < 8; m = m + 1) pl[4+m] = 8'h77;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd4);
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h00040001, 64, 750000);
        chk(k >= 0, "T16f WRQ 的末块 → ACK(1)");


        repeat (562500) @(posedge clk);
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h00040001, 64, 750000);
        chk(k >= 0, "T16g ★★ACK(1) 超时后被**重传**（WRQ 方向也做重传）");


        repeat (1750000) @(posedge clk);              // 14 ms（> 3×4 ms）→ 重试应已耗尽并收会话
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h00040001, 64, 1500000);   // 再观察 12 ms
        chk(k < 0, "T16h ★★重传到上限后会话被收掉，不再无限重传（修前这里会一直收到 ACK）");


        inject_en = 1; repeat (4) @(posedge clk);
        // 会话 A：带选项的 RRQ
        pl_len = 29;
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h01;
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        pl[17]=8'h62; pl[18]=8'h6C; pl[19]=8'h6B; pl[20]=8'h73;
        pl[21]=8'h69; pl[22]=8'h7A; pl[23]=8'h65;
        pl[25]=8'h31; pl[26]=8'h32; pl[27]=8'h38;
        scan_pos = nfr;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd5);
        expect_reply(16'h0789, 32'h0006626C, 64, 1000000);       // OACK = 00 06 'b' 'l'
        chk(k >= 0, "T16i 会话 A：带选项的 RRQ → OACK（把 oack_len 装填起来）");
        repeat (162500) @(posedge clk);
        // 会话 B：**不带选项**的 WRQ
        pl_len = 17;
        for (m = 0; m < 64; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h00; pl[1]=8'h02;                              // WRQ
        pl[ 2]=8'h6C; pl[ 3]=8'h6F; pl[ 4]=8'h61; pl[ 5]=8'h64;
        pl[ 6]=8'h2E; pl[ 7]=8'h62; pl[ 8]=8'h69; pl[ 9]=8'h6E;
        pl[10]=8'h00;
        pl[11]=8'h6F; pl[12]=8'h63; pl[13]=8'h74; pl[14]=8'h65; pl[15]=8'h74;
        pl[16]=8'h00;
        scan_pos = nfr;
        inj_udp(16'h0789, 16'd69, 16'h0BB8, 8'd6);
        expect_reply(16'h0789, 32'h00040000, 64, 1000000);       // ACK(0) = 00 04 00 00
        chk(k >= 0, "T16i2 会话 B：不带选项的 WRQ → ACK(0)");
        // 不给 DATA(1)，等超时重传
        repeat (562500) @(posedge clk);
        scan_pos = nfr;
        expect_reply(16'h0789, 32'h0, 64, 2500000);
        chk(k >= 0 && fr[k+42]===8'h00 && fr[k+43]===8'h04 && fr[k+44]===8'h00 && fr[k+45]===8'h00,
            "T16j ★★超时重发的是 **ACK(0)**（不是上一条会话残留的 OACK）");


        inject_en = 1;
        repeat (4) @(posedge clk);
        base_rtc = nfr;                          // 复用变量存"注入前"的快照

        pl_len = 24;                             // 片1：UDP 头 + 数据
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        pl[0]=8'h04; pl[1]=8'h01;                // 源端口 1025
        pl[2]=8'h04; pl[3]=8'h01;                // 目的端口 1025
        pl[4]=8'h00; pl[5]=8'h18;                // UDP 长度 = 24（8+16）
        pl[6]=8'h00; pl[7]=8'h00;                // 校验和 = 0（标准要求）
        for (m = 0; m < 16; m = m + 1) pl[8+m] = (8'hA0 + m[7:0]);
        inj_frag(16'h0456, 16'd1025, 16'h0456, 8'd2, 16'hBEEF, 13'd0, 1'b1, 24);
        repeat (60) @(posedge clk);

        pl_len = 16;                             // 片2：纯数据
        for (m = 0; m < 256; m = m + 1) pl[m] = 8'h00;
        for (m = 0; m < 16; m = m + 1) pl[m] = (8'hB0 + m[7:0]);
        inj_frag(16'h0456, 16'd1025, 16'h0456, 8'd3, 16'hBEEF, 13'd3, 1'b0, 16);
        inject_en = 0;
        repeat (25000) @(posedge clk);            // 等重组 + 端口层发布 + 判决


        chk(u_dut.u_rxports.plen[{2'd1, u_dut.u_rxports.pqw[1] - 1'b1}] === 11'd32,
            "T10b ★重组后消息长度 = 32 字节（16 + 16，第一片的 UDP 头不计）");
        // 逐字节对照：端口 RAM 里从该消息起点开始的 32 字节
        begin : t10c
            integer q0, bad;
            reg [31:0] ww, exp;
            reg [7:0]  eb0, eb1, eb2, eb3;
            bad = 0;
            for (q0 = 0; q0 < 8; q0 = q0 + 1) begin
                ww  = u_dut.u_rxports.dram[{3'd1, (u_dut.u_rxports.prptr[1] + q0[9:0])}];

                eb0 = (q0 < 4) ? (8'hA0 + 4*q0[7:0]) : (8'hB0 + 4*(q0[7:0] - 4'd4));
                eb1 = eb0 + 8'd1; eb2 = eb0 + 8'd2; eb3 = eb0 + 8'd3;
                exp = {eb3, eb2, eb1, eb0};
                if (ww !== exp) bad = bad + 1;
            end

            chk(bad === 0, "T10c ★重组后 32 字节逐字节正确（A0..AF + B0..BF）");
        end


        inject_en = 1; repeat (4) @(posedge clk);
        inj_snmp(8'd6, 8'd2, 8'd5, 8'hA0);                  // GET .6.2.5 = 保留端口
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd22);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 3750000);
        chk(k >= 0 && fr[k+42+20]===8'h02 && fr[k+42+23]===8'h01 &&
                    fr[k+42+24]===8'h30 && fr[k+42+25]===8'h00,
            "T19a ★GET .6.2.5 → noSuchName（保留端口不再报告）");
        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd1; oidb[2]=8'h00;
        vbuf[0]=8'h20;                                       // 32 = bit5（超出 5 个通信端口）
        inj_snmp_priv(8'hA3, 3, 1, 8'h02);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd23);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 3750000);

        chk(k >= 0 && fr[k+42+21]===8'h03,
            "T19b ★SET .9.1.0 = 32 → badValue（5 个通信端口，位图 0..31）");


        repeat (137500) @(posedge clk);
        inj_snmp(8'd1, 8'd6, 8'd0, 8'hA0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd24);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 3750000);
        chk(k >= 0 && fr[k+42+20]===8'h00 && fr[k+42+42]===8'h02 &&
                    fr[k+42+43]===8'h01 && fr[k+42+44]===8'd5,
            "T19c ★GET esPortNum(.1.6.0) = 5（与 portTable 行数一致）");


        repeat (137500) @(posedge clk);
        inj_snmp(8'd7, 8'd4, 8'd0, 8'hA0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd25);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 3750000);
        chk(k >= 0 && fr[k+42+20]===8'h00 && fr[k+42+42]===8'h02 &&
                    fr[k+42+43]===8'h01 && fr[k+42+44]===8'h00,
            "T19e ★★未同步时 GET rtcSynced(.7.4.0) = 0（不谎报已同步）");
        inject_en = 0;


        begin : t20
            integer w20, k20;
            reg [31:0] ofsf;

            chk(dres(12'h3E4) === 32'd0 && dres(12'h3E0) === 32'd0,
                "T20a 同步前：sync_cnt = 0、细偏移 = 0（尚未同步）");

            inject_en = 1; repeat (4) @(posedge clk);
            scan_pos = nfr;
            // 1) Sync：'S' + t1(小端) + seq=0x21；SN=1（接 T7 的 0）
            pl_len = 6;
            pl[0]=8'h53;
            pl[1]=m_ts[7:0]; pl[2]=m_ts[15:8]; pl[3]=m_ts[23:16]; pl[4]=m_ts[31:24];
            pl[5]=8'h21;
            inj_udp(16'h0BC5, 16'd3001, 16'd4000, 8'd1);
            // 2) 等 ES 的 DelayReq（'Q' + seq + 0 + 0）；先把整帧收全再判内容
            k20 = -1; w20 = 0;
            while (k20 < 0 && w20 < 500000) begin
                k20 = find_frame(scan_pos, 16'h0BC5);
                if (k20 < 0) begin @(posedge clk); w20 = w20 + 1; end
                else scan_pos = k20 + 1;
            end
            chk(k20 >= 0, "T20b ★ES 回了 DelayReq（SAP 口 4 → 主时钟）");
            if (k20 >= 0) begin
                w20 = 0;
                while (nfr < k20 + 64 && w20 < 250000) begin @(posedge clk); w20 = w20 + 1; end
                chk({fr[k20+42],fr[k20+43],fr[k20+44],fr[k20+45]} === {8'h51,8'h21,8'h00,8'h00},
                    "T20c DelayReq 载荷 = 'Q' + seq(0x21)（4 字节）");
                // SAP 语义：目的地址 = 本次 Sync 的**源**（主时钟 10.0.0.9:4000）
                chk(fr[k20+30]===8'h0A && fr[k20+31]===8'h00 &&
                    fr[k20+32]===8'h00 && fr[k20+33]===8'h09 &&
                    fr[k20+36]===8'h0F && fr[k20+37]===8'hA0,
                    "T20d ★DelayReq 发回主时钟的源地址（10.0.0.9:4000，SAP 动态目的）");
            end
            // 3) 回 DelayResp：'R' + seq + t4(小端)；t4 = 主时钟收到 DelayReq 的时刻；SN=2
            pl_len = 6;
            pl[0]=8'h52; pl[1]=8'h21;
            pl[2]=m_tx_sop[7:0]; pl[3]=m_tx_sop[15:8];
            pl[4]=m_tx_sop[23:16]; pl[5]=m_tx_sop[31:24];
            inj_udp(16'h0BC5, 16'd3001, 16'd4000, 8'd2);
            // 4) 等固件结算（轮询 sync_cnt，不用固定延时）
            begin : ptpwait
                integer wp;
                wp = 0;
                while (dres(12'h3E4) < 32'd1 && wp < 2500000) begin @(posedge clk); wp = wp + 1; end
                chk(wp < 2500000, "T20e ★固件结算了 DelayResp（sync_cnt ≥ 1，没卡在等 t3）");
            end
            ofsf = dres(12'h3E0);
            $display(" [PTP] sync_cnt=%0d ofs_fine=%0d ofs_ticks=%0d（期望 5000 / 50）",
                     dres(12'h3E4), $signed(ofsf), $signed(dres(12'h3DC)));
            //   真值推导（本台）：t1 = m_ts 取样值、t2 = 帧首收戳、t3 = 发戳、t4 = m_tx_sop
            //   两者相差正好 5000 格；t1 到 t2 最多几个拍（1 拍 = 100 ns = 1 格，÷2 后影响 ≤ 几格）
            //   ⇒ 判定窗口取 4950..5050（±50 格 = ±4 µs，远小于 8 µs 的刻度粒度）。
            chk(ofsf >= 32'd4950 && ofsf <= 32'd5050,
                "T20f ★★PTP 细偏移 = +5000 格（ES 落后主时钟 400 µs，算法符号/量值正确）");
            chk(dres(12'h3DC) === 32'd50,
                "T20g ★8 µs 刻度偏移 = 50 格（5000/100 四舍五入）");
            inject_en = 0;
        end


        begin : t21
            integer w21, k21, k21b, q21;
            reg [7:0]  b0;
            reg [31:0] ofsf21, t1b_s, t4_used;
            inject_en = 1; inj_net = 1'b1; repeat (4) @(posedge clk);
            scan_pos = nfr;
            b0 = tx_seq_b;
            // 1) B 网 Sync：'S' + t1(小端) + seq=0x31；SN=3
            pl_len = 6;
            pl[0]=8'h53;
            t1b_s = m_ts_b;
            pl[1]=m_ts_b[7:0]; pl[2]=m_ts_b[15:8]; pl[3]=m_ts_b[23:16]; pl[4]=m_ts_b[31:24];
            pl[5]=8'h31;
            inj_udp(16'h0BC5, 16'd3001, 16'd4000, 8'd3);
            // 2) 等 A 副本（find_frame 先找到的通常是 A），再等 **B 副本的 SOP**
            k21 = -1; w21 = 0;
            while (k21 < 0 && w21 < 500000) begin
                k21 = find_frame(scan_pos, 16'h0BC5);
                if (k21 < 0) begin @(posedge clk); w21 = w21 + 1; end
                else scan_pos = k21 + 1;
            end
            chk(k21 >= 0, "T21a ★B 网 Sync 也回了 DelayReq");

            k21b = -1; w21 = 0;
            while (k21b < 0 && w21 < 500000) begin
                for (q21 = (k21 >= 0) ? k21 : 0; q21 < nfr - 14; q21 = q21 + 1)
                    if (k21b < 0 &&
                        fr[q21]==8'h03 && fr[q21+4]==8'h0B && fr[q21+5]==8'hC5 &&
                        fr[q21+11]==8'h40 && fr[q21+12]==8'h08 && fr[q21+13]==8'h00 &&
                        fr[q21+42]==8'h51 && fr[q21+43]==8'h31) k21b = q21;
                if (k21b < 0) begin @(posedge clk); w21 = w21 + 1; end
            end
            chk(k21b >= 0, "T21b ★DelayReq 的 B 副本（VL 0x0BC5 / IFID=0x40 / 'Q'）确实发出去了");
            if (k21b >= 0) begin
                w21 = 0;
                while (nfr < k21b + 64 && w21 < 250000) begin @(posedge clk); w21 = w21 + 1; end
            end
            t4_used = (k21b >= 0) ? fr_sop_ts[k21b] : m_tx_sop_b;
            if (k21 >= 0) begin
                w21 = 0;
                while (nfr < k21 + 64 && w21 < 250000) begin @(posedge clk); w21 = w21 + 1; end
                chk({fr[k21+42],fr[k21+43],fr[k21+44],fr[k21+45]} === {8'h51,8'h31,8'h00,8'h00},
                    "T21c B 网 DelayReq 载荷 = 'Q' + seq(0x31)");
            end
            // 3) B 网 DelayResp：t4 = B 网主时钟收到 DelayReq 的时刻；SN=4
            pl_len = 6;
            pl[0]=8'h52; pl[1]=8'h31;
            pl[2]=t4_used[7:0]; pl[3]=t4_used[15:8];
            pl[4]=t4_used[23:16]; pl[5]=t4_used[31:24];
            inj_udp(16'h0BC5, 16'd3001, 16'd4000, 8'd4);
            // 4) 等第二次结算
            w21 = 0;
            while (dres(12'h3E4) < 32'd2 && w21 < 2500000) begin @(posedge clk); w21 = w21 + 1; end
            $display(" [PTP-B] sync_cnt=%0d | B 网 parser 描述符=%0d 重组提交=%0d 端口4交付=%0d",
                     dres(12'h3E4), n_netb_desc, n_netb_commit, n_p4_deliv);
            $display(" [PTP-B] settle: of=%0d pre=%0d cnt=%0d | seq0_fw=%0d tb_b0=%0d | t3=%08x t4_used-5000=%08x",
                     $signed(dres(12'h378)), $signed(dres(12'h37C)), dres(12'h39C),
                     dres(12'h3FC), b0, dres(12'h3F4), t4_used - 32'd5000);
            $display(" [PTP-B] fw t1=%08x t2=%08x t3=%08x t4=%08x | tb t1=%08x t4_used=%08x | d_ms=%0d d_sm=%0d",
                     dres(12'h3E8), dres(12'h3EC), dres(12'h3F4), dres(12'h3F8),
                     t1b_s, t4_used,
                     $signed(dres(12'h3EC) - t1b_s), $signed(dres(12'h3F8) - dres(12'h3F4)));
            chk(dres(12'h3E4) >= 32'd2, "T21d ★B 网 DelayResp 被结算（sync_cnt ≥ 2）");
            ofsf21 = dres(12'h3E0);
            $display(" [PTP-B] sync_cnt=%0d ofs_fine=%0d ofs_ticks=%0d（期望仍 ≈5000 / 50）",
                     dres(12'h3E4), $signed(ofsf21), $signed(dres(12'h3DC)));
            chk(ofsf21 >= 32'd4950 && ofsf21 <= 32'd5050,
                "T21e ★★B 网时间戳被正确使用：偏移仍 ≈5000（错用 A 网会漂到 ~5500）");
            inj_net = 1'b0; inject_en = 0;
        end


        begin : t19fg
            inject_en = 1; inj_net = 1'b0; repeat (4) @(posedge clk);
            repeat (137500) @(posedge clk);
            inj_snmp(8'd7, 8'd2, 8'd0, 8'hA0);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd26);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+42]===8'h02 && fr[k+42+43]===8'h01 && fr[k+42+44]>=8'd2,
                "T19f ★PTP 同步后 rtcSyncCnt(.7.2.0) ≥ 2（同步次数真的在涨）");
            repeat (137500) @(posedge clk);
            inj_snmp(8'd7, 8'd4, 8'd0, 8'hA0);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd27);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+42]===8'h02 && fr[k+42+43]===8'h01 && fr[k+42+44]===8'h01,
                "T19g ★PTP 同步后 rtcSynced(.7.4.0) = 1");
            inject_en = 0;
        end


        begin : t22
            integer r22, w22, k22, q22;
            reg [7:0]  seq22;
            reg [31:0] t4v, true_off, inc_last, ofs_last;
            reg [31:0] t0f, t0r, dtf, dtr;
            integer    ppm_last;
            m_drift_en = 1'b1;
            inject_en = 1; inj_net = 1'b0; repeat (4) @(posedge clk);
            inc_last = dres(12'h350);
            for (r22 = 0; r22 < 6; r22 = r22 + 1) begin
                repeat (1562500) @(posedge clk);              // 10 ms
                seq22 = 8'h40 + r22[7:0];
                pl_len = 6;
                pl[0]=8'h53;
                pl[1]=m_ts[7:0]; pl[2]=m_ts[15:8]; pl[3]=m_ts[23:16]; pl[4]=m_ts[31:24];
                pl[5]=seq22;
                true_off = m_ts - ts_fine_obs;               // 注入这一拍的真实偏置（主-从）

                inj_udp(16'h0BC5, 16'd3001, 16'd4000, 8'd3 + 2*r22);
                // 找 DelayReq（DA/VL + SA IFID=0x20 + 载荷 'Q'+seq）
                k22 = -1; w22 = 0;
                while (k22 < 0 && w22 < 500000) begin
                    for (q22 = 0; q22 < nfr - 64; q22 = q22 + 1)
                        if (k22 < 0 &&
                            fr[q22]==8'h03 && fr[q22+1]==8'h00 && fr[q22+2]==8'h00 &&
                            fr[q22+3]==8'h00 && fr[q22+4]==8'h0B && fr[q22+5]==8'hC5 &&
                            fr[q22+11]==8'h20 && fr[q22+12]==8'h08 && fr[q22+13]==8'h00 &&
                            fr[q22+42]==8'h51 && fr[q22+43]==seq22) k22 = q22;
                    if (k22 < 0) begin @(posedge clk); w22 = w22 + 1; end
                end
                if (k22 < 0) begin
                    chk(1'b0, "T22a ★每轮都能找到 DelayReq（频率补偿用例）");
                    r22 = 6;                                 // 结束循环
                end
                else begin
                    w22 = 0;
                    while (nfr < k22 + 64 && w22 < 250000) begin @(posedge clk); w22 = w22 + 1; end
                    t4v = fr_sop_ts[k22];
                    pl_len = 6;
                    pl[0]=8'h52; pl[1]=seq22;
                    pl[2]=t4v[7:0]; pl[3]=t4v[15:8]; pl[4]=t4v[23:16]; pl[5]=t4v[31:24];
                    inj_udp(16'h0BC5, 16'd3001, 16'd4000, 8'd4 + 2*r22);
                    w22 = 0;
                    while (dres(12'h3E4) < (3 + r22) && w22 < 2500000) begin @(posedge clk); w22 = w22 + 1; end
                    inc_last = dres(12'h350);
                    ppm_last = $signed(dres(12'h354));
                    ofs_last = dres(12'h3E0);
                    $display(" [FREQ] r=%0d sync_cnt=%0d ppm=%0d inc=%0d ofs=%0d true=%0d",
                             r22, dres(12'h3E4), ppm_last, inc_last,
                             $signed(ofs_last), $signed(true_off));
                end
            end
            chk(dres(12'h3E4) >= 32'd8, "T22a ★6 轮全部结算（sync_cnt ≥ 8）");
            chk(inc_last >= 32'd4297114 - 32'd60 && inc_last <= 32'd4297114 + 32'd60,
                "T22b ★★DDS 加数收敛到 nominal+2147（+500 ppm，±60 LSB）");
            chk(ppm_last >= -60 && ppm_last <= 60,
                "T22c ★★估出的频率偏差收敛到 0（±60 ppm）");
            chk($signed(ofs_last - true_off) <= 32'sd200 && $signed(ofs_last - true_off) >= -32'sd200,
                "T22d ★相位修正量与真实偏置之差 ≤200 格（slew 有界）");

            // ---- T22e/f：直接量**两条 DDS 链**的速率（+500 ppm）----
            //   T22a~d 只能证明"寄存器值收敛"；如果某天接线断了（只调细时钟、没调 8 µs 时基，
            //   或反过来），前面的断言仍然全绿。这里用 force 把加数固定成 +500 ppm，
            //   直接数 400k 拍内 ts_fine / rtc_cnt 的增量。
            force u_dut.freq_inc = 32'd4297114;    // NOM + 2147
            t0f = u_dut.ts_fine; t0r = u_dut.rtc_cnt;
            repeat (5000000) @(posedge clk);        // 32 ms
            dtf = u_dut.ts_fine - t0f;
            dtr = u_dut.rtc_cnt - t0r;
            release u_dut.freq_inc;
            $display(" [FREQ] DDS rate: ts_fine +=%0d (期望 500250)  rtc_cnt +=%0d (期望 5002/5003)",
                     dtf, dtr);
            chk(dtf >= 500200 && dtf <= 500300,
                "T22e ★★DDS 80 ns 时钟速率 = +500 ppm（细时钟真的被调频）");
            chk(dtr >= 5001 && dtr <= 5003,
                "T22f ★★DDS 8 µs 时基速率 = +500 ppm（rtc_now 的时基也被调频）");

            // ---- T22g/h：**负频偏 -500 ppm** —— 覆盖"溢出时少走一拍"这条路径
            //   （正频偏只走 +2 路径；如果负方向的 fneg 逻辑写错，前面的用例全绿也发现不了）----
            force u_dut.freq_inc = 32'd4292820;    // NOM - 2147
            t0f = u_dut.ts_fine; t0r = u_dut.rtc_cnt;
            repeat (5000000) @(posedge clk);
            dtf = u_dut.ts_fine - t0f;
            dtr = u_dut.rtc_cnt - t0r;
            release u_dut.freq_inc;
            $display(" [FREQ] DDS neg: ts_fine +=%0d (期望 499750)  rtc_cnt +=%0d (期望 4997/4998)",
                     dtf, dtr);
            chk(dtf >= 499700 && dtf <= 499800,
                "T22g ★★DDS 负频偏 -500 ppm（细时钟少走一拍）");
            chk(dtr >= 4996 && dtr <= 4998,
                "T22h ★★DDS 负频偏：8 µs 时基也变慢");

            // ---- T22i：加数写 0 必须被**硬件**钳到 -1000 ppm，不能冻结时基 ----
            force u_dut.freq_inc = 32'd0;
            t0f = u_dut.ts_fine; t0r = u_dut.rtc_cnt;
            repeat (5000000) @(posedge clk);
            dtf = u_dut.ts_fine - t0f;
            dtr = u_dut.rtc_cnt - t0r;
            release u_dut.freq_inc;
            $display(" [FREQ] DDS clamp0: ts_fine +=%0d (期望 499500)  rtc_cnt +=%0d (期望 4995/4996)",
                     dtf, dtr);
            chk(dtf >= 499450 && dtf <= 499550,
                "T22i ★加数写 0 被钳到 -1000 ppm（时基不会冻结）");
            m_drift_en = 1'b0; inject_en = 0;
        end


        begin : t23
            integer w23, a23;
            reg [31:0] bc0;
            inject_en = 1; inj_net = 1'b0; repeat (4) @(posedge clk);
            bc0 = dres(12'h358);                       // 启动计数（首次应为 1）
            // 1) SET .9.2.0 = "AFDX-PERSIST"（12 字符）
            oidb[0]=8'd9; oidb[1]=8'd2; oidb[2]=8'd0;
            vbuf[0]=8'h41; vbuf[1]=8'h46; vbuf[2]=8'h44; vbuf[3]=8'h58;   // AFDX
            vbuf[4]=8'h2D; vbuf[5]=8'h50; vbuf[6]=8'h45; vbuf[7]=8'h52;   // -PER
            vbuf[8]=8'h53; vbuf[9]=8'h49; vbuf[10]=8'h53; vbuf[11]=8'h54; // SIST
            inj_snmp_priv(8'hA3, 3, 12, 8'h04);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd28);   // 接 T19g 的 27
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+21]===8'h00, "T23a SET esSysName(.9.2.0) → noError");
            $display(" [T23] SET resp tail: %02x %02x %02x %02x %02x %02x %02x %02x",
                     fr[k+42+40],fr[k+42+41],fr[k+42+42],fr[k+42+43],
                     fr[k+42+44],fr[k+42+45],fr[k+42+46],fr[k+42+47]);
            repeat (137500) @(posedge clk);
            // 1b) GET .1.3.0 → 确认 SET 真的落进了 RAM（保存前的对照）
            inj_snmp(8'd1, 8'd3, 8'd0, 8'hA0);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd29);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 120, 3750000);   // 单帧 ~104 B；不要依赖 B 副本凑字节
            chk(k >= 0 && fr[k+42+42]===8'h04 && fr[k+42+43]===8'd12 &&
                fr[k+42+44]===8'h41 && fr[k+42+45]===8'h46 && fr[k+42+46]===8'h44 &&
                fr[k+42+47]===8'h58 && fr[k+42+48]===8'h2D && fr[k+42+49]===8'h50 &&
                fr[k+42+50]===8'h45 && fr[k+42+51]===8'h52 && fr[k+42+52]===8'h53 &&
                fr[k+42+53]===8'h49 && fr[k+42+54]===8'h53 && fr[k+42+55]===8'h54,
                "T23a2 ★保存前 GET .1.3.0 = AFDX-PERSIST（SET 已落 RAM）");
            repeat (137500) @(posedge clk);
            // 2) SET .9.4.0 = 1 → 保存到 flash
            oidb[0]=8'd9; oidb[1]=8'd4; oidb[2]=8'd0;
            vbuf[0]=8'd1;
            inj_snmp_priv(8'hA3, 3, 1, 8'h02);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd30);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 10000000);     // 含擦除 + 24 字节编程
            chk(k >= 0 && fr[k+42+21]===8'h00, "T23b ★★SET .9.4.0=1 → 设置写入 flash（noError）");


            repeat (137500) @(posedge clk);
            oidb[0]=8'd9; oidb[1]=8'd2; oidb[2]=8'd0;
            vbuf[0]=8'h41; vbuf[1]=8'h46; vbuf[2]=8'h44; vbuf[3]=8'h58;   // AFDX
            vbuf[4]=8'h2D; vbuf[5]=8'h54; vbuf[6]=8'h57; vbuf[7]=8'h4F;   // -TWO
            inj_snmp_priv(8'hA3, 3, 8, 8'h04);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd31);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+21]===8'h00, "T23b2 SET 名字 = AFDX-TWO → noError");
            repeat (137500) @(posedge clk);
            oidb[0]=8'd9; oidb[1]=8'd4; oidb[2]=8'd0;
            vbuf[0]=8'd1;
            inj_snmp_priv(8'hA3, 3, 1, 8'h02);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd32);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 10000000);
            chk(k >= 0 && fr[k+42+21]===8'h00,
                "T23b3 ★★第二次保存（擦已编程扇区 + 重写）→ noError");
            // 3) 复位（模拟掉电重启）
            inject_en = 0; rx_fv = 0; rx_fsop = 0; rx_feop = 0;
            res = 1'b1; repeat (40) @(posedge clk); res = 1'b0;
            // 4) 等启动计数 +1（dRAM 不重载，所以这个计数**只能**由新的 main() 自增）
            w23 = 0;
            while (dres(12'h358) <= bc0 && w23 < 5000000) begin @(posedge clk); w23 = w23 + 1; end
            chk(w23 < 5000000 && dres(12'h358) === (bc0 + 32'd1),
                "T23c 复位后固件重新启动（boot_cnt 1→2）");

            w23 = 0;
            while (dres(12'h32C) !== 32'hA5000002 && w23 < 5000000) begin @(posedge clk); w23 = w23 + 1; end
            chk(w23 < 5000000, "T23c2 复位后的自检完成（RES 2C = 0xA5000002）");
            repeat (1500000) @(posedge clk);                // 8 ms：等主循环跑起来

            chk(dres(12'h3E4) === 32'd0 && dres(12'h3E0) === 32'd0 &&
                dres(12'h350) === 32'd4294967,
                "T23c3 ★热复位后 PTP 状态清零、DDS 加数回标称（rtc_init）");
            // 5) GET .1.3.0 —— 必须是从 flash 恢复的名字
            inject_en = 1; inj_net = 1'b0; repeat (4) @(posedge clk);
            n_p3_deliv = 0; n_desc_a = 0;
            inj_snmp(8'd1, 8'd3, 8'd0, 8'hA0);
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd33);   // 热复位：IC 的 PSN RAM 不重载，接 32 往下数
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 120, 10000000);
            $display(" [T23] GET .1.3.0 应答: %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x %02x",
                     fr[k+42+42],fr[k+42+43],fr[k+42+44],fr[k+42+45],fr[k+42+46],fr[k+42+47],
                     fr[k+42+48],fr[k+42+49],fr[k+42+50],fr[k+42+51],fr[k+42+52],fr[k+42+53],
                     fr[k+42+54],fr[k+42+55]);
            $display(" [T23] probes: descA=%0d p3deliv=%0d", n_desc_a, n_p3_deliv);
            $display(" [T23] boot_cnt=%0d load_rc=%0d | SNMP req=%0d rsp=%0d",
                     dres(12'h358), $signed(dres(12'h35C)), dres(12'h3C4), dres(12'h3C8));
            chk(k >= 0 && fr[k+42+42]===8'h04 && fr[k+42+43]===8'd8 &&
                fr[k+42+44]===8'h41 && fr[k+42+45]===8'h46 && fr[k+42+46]===8'h44 &&
                fr[k+42+47]===8'h58 && fr[k+42+48]===8'h2D && fr[k+42+49]===8'h54 &&
                fr[k+42+50]===8'h57 && fr[k+42+51]===8'h4F,
                "T23d ★★复位后 GET .1.3.0 = 第二次保存的 AFDX-TWO（重写/持久化生效）");


            repeat (137500) @(posedge clk);
            inj_snmp(8'd9, 8'd4, 8'd9, 8'hA0);            // .9.4.9（标量非 .0）
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd34);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+20]===8'h02,
                "T23e ★GET .9.4.9 → noSuchName（标量只接受 .0）");
            repeat (137500) @(posedge clk);
            inj_snmp(8'd5, 8'd1, 8'd0, 8'hA0);            // .5.1.0（VLID 0 保留）
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd35);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+20]===8'h02,
                "T23f ★GET .5.1.0 → noSuchName（VLID 0 是保留值，不会假命中）");
            repeat (137500) @(posedge clk);
            inj_snmp(8'd1, 8'd5, 8'd3, 8'hA0);            // .1.5.3（标量非 .0）
            inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd36);
            scan_pos = nfr;
            expect_reply(16'h0161, 32'h0, 96, 3750000);
            chk(k >= 0 && fr[k+42+20]===8'h02,
                "T23g ★GET .1.5.3 → noSuchName（标量实例规则）");
            inject_en = 0;
        end


        repeat (137500) @(posedge clk);
        oidb[0]=8'd5; oidb[1]=8'd3; oidb[2]=8'hB0; oidb[3]=8'h23;  // x=6179（≥4096，2 字节）
        inj_snmp_priv(8'hA0, 4, 0, 8'h05);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd0);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 128, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'd2, "T15q unknown instance -> noSuchName");
        chk(k >= 0 && fr[k+42+24]==8'd1, "T15r error-index = 1");

        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd3; oidb[2]=8'hB0; oidb[3]=8'h23;
        vbuf[0]=8'd60;                                   // 若被误改，VL 0x123 的 Lmax 会变 60
        inj_snmp_priv(8'hA3, 4, 1, 8'h02);

        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd1);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 128, 2500000);
        $display("  [dbg] T15s k=%0d p[19..26]=%02x %02x %02x %02x %02x %02x %02x %02x", k,
                 fr[k+42+19],fr[k+42+20],fr[k+42+21],fr[k+42+22],fr[k+42+23],fr[k+42+24],fr[k+42+25],fr[k+42+26]);
        chk(k >= 0 && fr[k+42+21]==8'd2, "T15s SET unknown instance rejected");

        repeat (137500) @(posedge clk);
        oidb[0]=8'd9; oidb[1]=8'd3; oidb[2]=8'h82; oidb[3]=8'h23;   // 291 = 0x123
        inj_snmp_priv(8'hA0, 4, 0, 8'h05);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd2);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 96, 2500000);
        $display("  [dbg] T15t k=%0d p[44..48]=%02x %02x %02x %02x %02x", k,
                 fr[k+42+44],fr[k+42+45],fr[k+42+46],fr[k+42+47],fr[k+42+48]);
        chk(k >= 0 && fr[k+42+46]==8'd40, "T15t VL 0x123 Lmax untouched (40)");


        repeat (137500) @(posedge clk);
        oidb[0]=8'd5; oidb[1]=8'd3; oidb[2]=8'h84; oidb[3]=8'h82; oidb[4]=8'h23;
        inj_snmp_priv(8'hA0, 5, 0, 8'h05);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd3);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 128, 2500000);
        chk(k >= 0 && fr[k+42+21]==8'd2, "T15u 3-byte subid -> noSuchName");
        chk(dres(12'h398) === 32'd47,   "T15v 3-byte subid len=47");


        repeat (137500) @(posedge clk);
        inj_snmp_multi(17, 0);
        inj_udp(16'h0161, 16'd161, 16'h0BB8, 8'd4);
        scan_pos = nfr;
        expect_reply(16'h0161, 32'h0, 128, 3500000);
        $display("  [dbg] T15w k=%0d errstat=%0d", k, (k >= 0) ? fr[k+42+20] : 8'hxx);

        chk(k >= 0 && fr[k+42+20]==8'd1, "T15w 17 varbinds -> tooBig");

        $display("=============================================");
        if (errs == 0) $display(">> PASS: 固件→CPU窗口→三端口→组帧→回环→解析→IC/RM→固件，端到端打通");
        else           $display(">> FAIL: %0d 项不符", errs);
        $finish;
    end


    initial begin #600000000; $display(">> TIMEOUT nfr=%0d", nfr); $finish; end
endmodule
