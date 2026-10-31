// afdx_tx_build.v —— AFDX 发送组帧器（消息 + VL 配置 → 完整以太网帧）
`timescale 1ns/1ps

module afdx_tx_build #(
    parameter [23:0] MAC_CONST_SRC = 24'h020000,
    parameter [15:0] USER_DEFINED  = 16'h0001,
    parameter [31:0] SRC_IP        = 32'h0A000001
)(
    input  wire        clk,
    input  wire        reset,

    input  wire [15:0] cfg_vlid,
    input  wire [15:0] cfg_sport,
    input  wire [15:0] cfg_dport,
    input  wire [31:0] cfg_dip,
    input  wire [1:0]  cfg_neten,
    input  wire [7:0]  cfg_sn,
    input  wire        cfg_sn_vld,
    input  wire        cfg_vld,

    input  wire        msg_valid,
    output wire        msg_ready,
    input  wire        msg_sop,
    input  wire        msg_eop,
    input  wire [7:0]  msg_data,
    input  wire [10:0] msg_len,
    input  wire        go,          // BAG 调度许可：为 0 时不许起帧（消息挂在流上等）

    output wire        idle_o,

    output wire        fr_valid,
    input  wire        fr_ready,
    output wire        fr_sop,
    output wire        fr_eop,
    output wire [7:0]  fr_data,
    output reg         fr_net,
    output reg         sn_adv,
    output reg         pay_replay,      // 已废弃：恒 0（B 副本由本模块的 pbuf 承担）
    output wire        pay_hold
);
    function [15:0] ip_cksum;
        input [15:0] a,b,c,d,e,f,g,h,i,j;
        reg [31:0] sum;
        begin
            sum = a+b+c+d+e+f+g+h+i+j;
            sum = (sum & 16'hFFFF) + (sum >> 16);
            sum = (sum & 16'hFFFF) + (sum >> 16);
            ip_cksum = ~sum[15:0];
        end
    endfunction

    reg [15:0] ip_len, udp_len, total, lat_len;
    reg [15:0] pos;
    reg [1:0]  st;
    reg        send_b;


    (* ramstyle = "M4K" *) reg [7:0] pbuf [0:2047];
    reg [10:0] pw;      // A 副本载荷写指针
    reg [10:0] pr;      // B 副本读地址（寄存器；只在 S_B 里按载荷推进）
    reg [7:0]  pbr;     // B 副本读数据（寄存读，比 pr 晚一拍）
    // 1 = 当前这次「S_B 发送」的载荷来自 pbuf（net_en=0 的第二副本）；
    // 0 = 载荷仍来自上游（net_en=2 的唯一 B 副本，没有先行的 A 副本可存）
    reg        b_from_buf;


    reg [15:0] vlid_l, sport_l, dport_l;
    reg [31:0] dip_l;
    reg        snvld_l;
    reg [7:0]  sn_l;

    localparam S_IDLE = 2'd0, S_A = 2'd1, S_B = 2'd2, S_GAP = 2'd3;


    // ---- FCS（CRC-32）：本 MAC IP 是 FCS 透传（不插不剥），必须由 AFDX 侧自己算 ----
    wire [31:0] crc_out;
    wire [31:0] fcs   = ~crc_out;                   // 以太网 FCS = CRC 结果取反，低字节先发
    wire [15:0] ftot  = total + 16'd4;              // 帧体 + 4 字节 FCS

    afdx_crc32 u_crc (
        .clk     (clk),
        .reset   (reset),
        .init    (fr_valid && fr_ready && (pos == 16'd0)),
        .en      (fr_valid && fr_ready && (pos < total)),   // 只折叠帧体，不折叠 FCS 自身
        .data_in (fr_data),
        .crc_out (crc_out)
    );

    wire [15:0] ip_cs = ip_cksum(16'h4500, ip_len, 16'h0000, 16'h4000, 16'h0111,
                                 16'h0000, SRC_IP[31:16], SRC_IP[15:0],
                                 dip_l[31:16], dip_l[15:0]);

    // 按位置取字节
    function [7:0] fbyte;
        input [15:0] p;
        input [15:0] il, ul;
        input [15:0] cs;
        reg [15:0] q;
        begin
            fbyte = 8'h00;
            if (p < 16'd14) begin
                case (p)
                    16'd0:            fbyte = 8'h03;      // DA 常量域 03:00:00:00
                    16'd1,16'd2,16'd3:fbyte = 8'h00;
                    16'd4:            fbyte = vlid_l[15:8];
                    16'd5:            fbyte = vlid_l[7:0];
                    16'd6:            fbyte = MAC_CONST_SRC[23:16];
                    16'd7:            fbyte = MAC_CONST_SRC[15:8];
                    16'd8:            fbyte = MAC_CONST_SRC[7:0];
                    16'd9:            fbyte = USER_DEFINED[15:8];
                    16'd10:           fbyte = USER_DEFINED[7:0];
                    16'd11:           fbyte = fr_net ? 8'h40 : 8'h20;  // Interface_ID A=1, B=2
                    16'd12:           fbyte = 8'h08;
                    16'd13:           fbyte = 8'h00;
                endcase
            end
            else if (p < 16'd34) begin
                q = p - 16'd14;
                case (q)
                    16'd0:  fbyte = 8'h45;
                    16'd1:  fbyte = 8'h00;
                    16'd2:  fbyte = il[15:8];
                    16'd3:  fbyte = il[7:0];
                    16'd4:  fbyte = 8'h00;
                    16'd5:  fbyte = 8'h00;
                    16'd6:  fbyte = 8'h40;
                    16'd7:  fbyte = 8'h00;
                    16'd8:  fbyte = 8'h01;                // TTL = 1
                    16'd9:  fbyte = 8'h11;                // Protocol = UDP
                    16'd10: fbyte = cs[15:8];
                    16'd11: fbyte = cs[7:0];
                    16'd12: fbyte = SRC_IP[31:24];
                    16'd13: fbyte = SRC_IP[23:16];
                    16'd14: fbyte = SRC_IP[15:8];
                    16'd15: fbyte = SRC_IP[7:0];
                    16'd16: fbyte = dip_l[31:24];
                    16'd17: fbyte = dip_l[23:16];
                    16'd18: fbyte = dip_l[15:8];
                    16'd19: fbyte = dip_l[7:0];
                endcase
            end
            else if (p < 16'd42) begin
                q = p - 16'd34;
                case (q)
                    16'd0: fbyte = sport_l[15:8];
                    16'd1: fbyte = sport_l[7:0];
                    16'd2: fbyte = dport_l[15:8];
                    16'd3: fbyte = dport_l[7:0];
                    16'd4: fbyte = ul[15:8];
                    16'd5: fbyte = ul[7:0];
                    16'd6: fbyte = 8'h00;                 // UDP 校验和 = 0（AFDX 不用）
                    16'd7: fbyte = 8'h00;
                endcase
            end

            else if (p < 16'd42 + {5'd0, lat_len}) fbyte = msg_data;
            else if (p == total - 16'd1)           fbyte = snvld_l ? sn_l : 8'h00;  // SN
            else                                   fbyte = 8'h00;                       // padding
        end
    endfunction


    wire [15:0] pay_end = 16'd41 + {5'd0, lat_len};
    wire        pay_win = (pos >= 16'd42) && (pos <= pay_end);


    assign fr_valid  = (st != S_IDLE) && (st != S_GAP);
    assign fr_sop    = fr_valid && (pos == 16'd0);
    assign fr_eop    = fr_valid && (pos == ftot - 16'd1);

    assign fr_data   = fr_valid ? (pay_win ? (((st == S_B) && b_from_buf) ? pbr : msg_data) :
                       ((pos >= total) ? fcs[{pos - total, 3'b000} +: 8] :
                                         fbyte(pos, ip_len, udp_len, ip_cs))) : 8'h00;

    assign msg_ready = fr_valid && pay_win && fr_ready && !((st == S_B) && b_from_buf);
    assign idle_o    = (st == S_IDLE);


    assign pay_hold = (st == S_A) && send_b;

    always @(posedge clk) begin
        if (reset) begin
            st <= S_IDLE; pos <= 0; send_b <= 0; fr_net <= 0;
            sn_adv <= 0; pay_replay <= 0; pw <= 0; pr <= 0; pbr <= 0; b_from_buf <= 0;
            vlid_l <= 0; sport_l <= 0; dport_l <= 0; dip_l <= 0; snvld_l <= 0; sn_l <= 0;
            ip_len <= 0; udp_len <= 0; total <= 0; lat_len <= 0;
        end
        else begin
            sn_adv     <= 1'b0;
            pay_replay <= 1'b0;

            if ((st != S_B) || (fr_valid && fr_ready)) pbr <= pbuf[pr];

            if (fr_valid && fr_ready && (pos == 16'd1)) sn_l <= cfg_sn;   // pos=1：sn_frame 已稳定（见下方注释）
            if (st != S_B)                     pr <= 11'd0;
            else if (fr_valid && fr_ready &&
                     (pos >= 16'd41) && (pos < pay_end)) pr <= pr + 11'd1;
            case (st)
                S_IDLE: begin
                    pos <= 0;
                    if (msg_valid && msg_sop && cfg_vld && go) begin
                        pw <= 11'd0; pr <= 11'd0;
                        vlid_l <= cfg_vlid; sport_l <= cfg_sport; dport_l <= cfg_dport;
                        dip_l  <= cfg_dip;  snvld_l <= cfg_sn_vld;
                        lat_len <= msg_len;                       // 锁存本帧长度
                        ip_len  <= 16'd28 + {5'd0, msg_len};
                        udp_len <= 16'd8  + {5'd0, msg_len};
                        total   <= (16'd29 + {5'd0, msg_len} < 16'd46) ? 16'd60 :
                                   (16'd43 + {5'd0, msg_len});
                        send_b  <= (cfg_neten != 2'd1);

                        b_from_buf <= (cfg_neten == 2'd0) || (cfg_neten == 2'd3);
                        fr_net  <= (cfg_neten == 2'd2);
                        st      <= (cfg_neten == 2'd2) ? S_B : S_A;
                    end
                end
                default: begin
                    // 副本间隔态：只空一拍就进 B 副本（见 S_GAP 的说明）
                    if (st == S_GAP) begin st <= S_B; pr <= 11'd0; end
                    // 载荷直接由源**组合**驱动（见上面的 fr_data）—— 不再取样/输出两拍
                    if (fr_valid && fr_ready) begin
                        // A 副本的载荷边发边存（B 副本从这里读）
                        if (pay_win && (st == S_A) && send_b) begin
                            pbuf[pw] <= msg_data;
                            pw <= pw + 11'd1;
                        end
                        if (pos == ftot - 16'd1) begin
                            pos <= 0;
                            if (st == S_A && send_b) begin
                                st <= S_GAP; fr_net <= 1'b1;   // 先空一拍，再发 B 副本（载荷从 pbuf 读，不再要上游重放）
                            end
                            else begin
                                st <= S_IDLE; sn_adv <= 1'b1;
                            end
                        end
                        else pos <= pos + 16'd1;
                    end
                end
            endcase
        end
    end

endmodule
