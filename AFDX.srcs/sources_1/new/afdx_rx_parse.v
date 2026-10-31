// afdx_rx_parse.v —— AFDX 接收帧解析器（字节流 → 帧描述符 + 载荷流）
`timescale 1ns/1ps

module afdx_rx_parse #(
    parameter [31:0] DA_CONST = 32'h0300_0000
)(
    input  wire        clk,
    input  wire        reset,

    input  wire        in_valid,
    output wire        in_ready,
    input  wire        in_sop,
    input  wire        in_eop,
    input  wire [7:0]  in_data,
    input  wire        in_net,

    output reg         ok,
    output reg  [15:0] vlid,
    output reg  [31:0] sip,
    output reg  [31:0] dip,
    output reg  [15:0] sport,
    output reg  [15:0] dport,

    output reg  [10:0] paylen,
    output reg  [7:0]  sn,
    output reg         net,
    output reg  [10:0] framelen,

    output reg         err_eth,
    output reg         err_ip,
    output reg         err_udp,
    output reg         err_len,
    output reg         err_fcs,      // 帧校验序列(FCS)错 —— 本 MAC 是 FCS 透传，必须 ES 自己校验

    output reg         out_valid,
    input  wire        out_ready,
    output reg         out_sop,
    output reg         out_eop,
    output reg  [7:0]  out_data,
    output reg  [15:0] out_port,     // 载荷流期间有效的端口号（= UDP 目的端口）
    // ---- IP 分片（ARINC 664 P7 要求端系统支持 IP 分片重组）----
    output wire        frag,         // 本帧属于一条**分片**数据报
    output wire        mf,           // MF 标志（还有后续分片）
    output wire [12:0] foff,         // 片偏移（单位 8 字节）
    output wire [15:0] ipid,         // IP 标识（同一条数据报的各片相同）
    output reg  [10:0] paylen_out,   // 本帧交出去的载荷字节数（分片感知）
    output reg         desc_vld,     // 帧描述符有效（**eop 后 1 拍**脉冲，供 IC/RM 判决）
    output reg         vlid_vld      // VLID 抓全（第 5 字节后 1 拍）脉冲 → 顶层提前查表取端口号
);

    reg [10:0] pos;
    // ---- IP 分片相关（字节 16..21）----
    reg [15:0] iptot_q;      // IP 总长度
    reg [15:0] ipid_q;       // IP 标识
    reg [12:0] foff_q;       // 片偏移（8 字节为单位）
    reg        mf_q;         // MF（more fragments）
    // 组合判据（必须在错误判据之前声明 —— Verilog 先声明后用）
    wire        is_frag   = (foff_q != 13'd0) || mf_q;      // 本帧是分片
    wire        is_firstf = (foff_q == 13'd0);              // 第一片（含 UDP 头）

    wire        hdr_udp   = !is_frag || is_firstf;
    reg        in_frame;
    reg [31:0] ip_sum;
    reg [7:0]  prev_byte;
    reg [15:0] udplen_q;
    reg        vlid_pend;
    reg [31:0] crc_r;

    // CRC-32 单字节更新（与 afdx_crc32.v 同一算法）
    function [31:0] crc_byte;
        input [31:0] c;
        input [7:0]  d;
        integer      k;
        reg   [31:0] t;
        begin
            t = c ^ {24'h000000, d};
            for (k = 0; k < 8; k = k + 1)
                t = t[0] ? ((t >> 1) ^ 32'hEDB88320) : (t >> 1);
            crc_byte = t;
        end
    endfunction

    assign in_ready = 1'b1;

    // 本帧的组合错误判据（eop 那拍取值即本帧结论，避免"读到清零后的旧值"）
    wire [10:0] flen_w  = pos + 11'd1;
    wire        e_len_w = (flen_w < 11'd64) || (flen_w > 11'd1518);
    wire        e_fcs_w = (crc_byte(crc_r, in_data) != 32'hDEBB20E3);
    wire        e_ip_w  = (fold(ip_sum) != 16'hFFFF);

    wire        e_udp_w = !is_frag && ((udplen_q < 16'd9) || (udplen_q > 16'd1479));

    wire        e_lenc_w = !is_frag && (iptot_q != (udplen_q + 16'd20));

    wire        e_iptot_w = (iptot_q < 16'd20) || (iptot_q > 16'd1499);

    function [15:0] fold;
        input [31:0] s;
        reg [31:0] t;
        begin
            t    = (s & 32'hFFFF) + (s >> 16);
            t    = (t & 32'hFFFF) + (t >> 16);
            fold = t[15:0];
        end
    endfunction


    wire [10:0] npay_last = udplen_q[10:0] + 11'd33;
    wire [10:0] fpay_last = iptot_q[10:0] + 11'd13;         // 34 + (IP总长-20) - 1
    wire [10:0] pay_start = is_frag ? (is_firstf ? 11'd42 : 11'd34) : 11'd42;
    wire [10:0] pay_last  = is_frag ? fpay_last : npay_last;
    wire        pay_en    = is_frag ? 1'b1 : (udplen_q >= 16'd9);
    wire        in_pay    = in_frame && (pos >= pay_start) && (pos <= pay_last) && pay_en;
    wire [10:0] paylen_w  = pay_last - pay_start + 11'd1;

    reg         pay_done;

    wire        pay_done_now = pay_done || (in_pay && (pos == pay_last));


    assign frag = is_frag;
    assign mf   = mf_q;
    assign foff = foff_q;
    assign ipid = ipid_q;


    reg  [7:0]  sn_d1, sn_d2, sn_d3, sn_d4;

    always @(posedge clk) begin
        if (reset) begin
            crc_r <= 32'hFFFFFFFF;
        end
        else if (in_valid && in_ready) begin
            crc_r <= in_sop ? crc_byte(32'hFFFFFFFF, in_data)
                            : crc_byte(crc_r, in_data);
        end
    end

    always @(posedge clk) begin
        if (reset) begin
            pos <= 0; in_frame <= 0; ip_sum <= 0; prev_byte <= 0; udplen_q <= 0; sn_d1 <= 0; sn_d2 <= 0; sn_d3 <= 0; sn_d4 <= 0;
            ok <= 0; vlid <= 0; sip <= 0; dip <= 0; sport <= 0; dport <= 0;
            iptot_q <= 16'd0; ipid_q <= 16'd0; foff_q <= 13'd0; mf_q <= 1'b0;
            paylen <= 0; sn <= 0; net <= 0; framelen <= 0;
            err_eth <= 0; err_ip <= 0; err_udp <= 0; err_len <= 0; err_fcs <= 0; pay_done <= 1'b0;
            out_valid <= 0; out_sop <= 0; out_eop <= 0; out_data <= 0; out_port <= 0;
            desc_vld <= 0; vlid_vld <= 0; vlid_pend <= 0;
        end
        else begin
            out_valid <= 1'b0; out_sop <= 1'b0; out_eop <= 1'b0;
            desc_vld  <= 1'b0;              // 默认；下面的 eop 分支会把它抬高一拍
            vlid_vld  <= vlid_pend;         // pos==5 的下一拍（vlid 已全）发一拍脉冲
            vlid_pend <= 1'b0;

            if (in_valid && in_ready) begin
                if (in_sop) begin
                    pos <= 11'd1; in_frame <= 1'b1; ip_sum <= 32'h0;
                    udplen_q <= 16'h0;

                    ok <= (in_data == DA_CONST[31:24]);
                    err_eth <= 0; err_ip <= 0; err_udp <= 0; err_len <= 0; err_fcs <= 0; pay_done <= 1'b0;

                end
                else if (in_frame) begin
                    pos <= pos + 11'd1;
                    // SN 延迟线（1 字节/拍，只在帧内有效字节上移位）
                    sn_d4 <= sn_d3; sn_d3 <= sn_d2; sn_d2 <= sn_d1; sn_d1 <= in_data;

                    // ---- ETH ----
                    if (pos < 11'd4)
                        ok <= ok && (in_data == DA_CONST[8*(3-pos) +: 8]);
                    if (pos == 11'd4) vlid[15:8] <= in_data;
                    if (pos == 11'd5) begin vlid[7:0] <= in_data; vlid_pend <= 1'b1; end
                    if (pos == 11'd12) ok <= ok && (in_data == 8'h08);
                    if (pos == 11'd13) ok <= ok && (in_data == 8'h00);

                    if (pos == 11'd11) begin

                        net <= in_net;
                        if (in_data != (in_net ? 8'h40 : 8'h20)) err_eth <= 1'b1;
                        ok <= ok && (in_data == (in_net ? 8'h40 : 8'h20));
                    end

                    // ---- IP ----
                    if (pos == 11'd14) ok <= ok && (in_data == 8'h45);
                    if (pos == 11'd16) iptot_q[15:8] <= in_data;      // IP 总长度
                    if (pos == 11'd17) iptot_q[7:0]  <= in_data;
                    if (pos == 11'd18) ipid_q[15:8]  <= in_data;      // IP 标识
                    if (pos == 11'd19) ipid_q[7:0]   <= in_data;
                    if (pos == 11'd20) begin mf_q <= in_data[5]; foff_q[12:8] <= in_data[4:0]; end
                    if (pos == 11'd21) foff_q[7:0]   <= in_data;
                    if (pos == 11'd22) ok <= ok && (in_data >= 8'h01);
                    if (pos == 11'd23) ok <= ok && (in_data == 8'h11);
                    if (pos == 11'd26) sip[31:24] <= in_data;
                    if (pos == 11'd27) sip[23:16] <= in_data;
                    if (pos == 11'd28) sip[15:8]  <= in_data;
                    if (pos == 11'd29) sip[7:0]   <= in_data;
                    if (pos == 11'd30) dip[31:24] <= in_data;
                    if (pos == 11'd31) dip[23:16] <= in_data;
                    if (pos == 11'd32) dip[15:8]  <= in_data;
                    if (pos == 11'd33) dip[7:0]   <= in_data;
                    if (pos >= 11'd15 && pos < 11'd34 && pos[0])
                        ip_sum <= ip_sum + {prev_byte, in_data};
                    prev_byte <= in_data;

                    // ---- UDP ----
                    if (pos == 11'd34 && hdr_udp) sport[15:8] <= in_data;
                    if (pos == 11'd35 && hdr_udp) sport[7:0]  <= in_data;
                    if (pos == 11'd36 && hdr_udp) begin dport[15:8] <= in_data; out_port[15:8] <= in_data; end
                    if (pos == 11'd37 && hdr_udp) begin dport[7:0]  <= in_data; out_port[7:0]  <= in_data; end
                    if (pos == 11'd38 && hdr_udp) udplen_q[15:8] <= in_data;
                    if (pos == 11'd39 && hdr_udp) udplen_q[7:0]  <= in_data;

                    if (pos == 11'd40 && hdr_udp) ok <= ok && (in_data == 8'h00);
                    if (pos == 11'd41 && hdr_udp) ok <= ok && (in_data == 8'h00);

                    // ---- 载荷流 ----
                    if (in_pay) begin
                        if (pos == pay_last) pay_done <= 1'b1;
                        out_valid <= 1'b1;
                        out_data  <= in_data;
                        out_sop   <= (pos == pay_start);
                        out_eop   <= (pos == pay_last);      // 载荷最后一个字节就收尾（不是整帧末！）
                    end
                end

                if (in_eop && in_frame) begin
                    in_frame <= 1'b0;
                    desc_vld <= 1'b1;                          // 下一拍：描述符全部定稿
                    framelen <= pos + 11'd1;

                    if (flen_w < 11'd64 || flen_w > 11'd1518) err_len <= 1'b1;
                    if (pay_en && !pay_done_now) err_len <= 1'b1;
                    if (e_lenc_w) err_len <= 1'b1;

                    if (e_iptot_w) err_len <= 1'b1;
                    if (crc_byte(crc_r, in_data) != 32'hDEBB20E3) err_fcs <= 1'b1;
                    if (fold(ip_sum) != 16'hFFFF) err_ip <= 1'b1;

                    if (!is_frag && (udplen_q < 16'd9 || udplen_q > 16'd1479)) err_udp <= 1'b1;
                    ok      <= ok && !(e_len_w | e_fcs_w | e_ip_w | e_udp_w | e_lenc_w | e_iptot_w | (pay_en && !pay_done_now));
                    sn      <= sn_d4;                          // SN = 帧末前第 4 个字节（紧贴 FCS）
                    // 载荷长度输出（UDP 长度 - 8）
                    paylen     <= udplen_q[10:0] - 11'd8;   // 兼容旧语义（非分片才是这个）
                    paylen_out <= paylen_w;
                end
            end
        end
    end

endmodule
