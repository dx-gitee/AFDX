// tb_rx_parse.v —— 发送组帧器 → 接收解析器 对拍
`timescale 1ns/1ps
module tb_rx_parse;
    reg clk=0, reset=1;
    // 配置
    reg [15:0] vlid=16'h0123, sport=16'h0BB8, dport=16'h0BB9;
    reg [31:0] dip=32'hE0E00123;
    reg [1:0]  neten=2'd2;                       // 仅网络 A（只发一帧，便于检查）
    reg [7:0]  sn=8'h05; reg sn_vld=1'b1, cfg_vld=1'b1;
    // 消息
    reg        mv=0, msop=0, meop=0; reg [7:0] mdat=0; reg [10:0] mlen=8;
    wire       mrdy;
    // 帧流（TX → RX）
    wire       fv, fsop, feop, fnet, snadv, preplay;
    wire [7:0] fd;
    // 解析输出
    wire       ok; wire [15:0] rv_vlid, rv_sport, rv_dport;
    wire [31:0] rv_sip, rv_dip;
    wire [10:0] rv_paylen, rv_framelen;
    wire [7:0]  rv_sn;
    wire        rv_net, err_eth, err_ip, err_udp, err_len, err_fcs;
    wire        ov, osop, oeop; wire [7:0] od; reg ordy=1;

    afdx_tx_build #(.SRC_IP(32'h0A000001)) u_tx (
        .clk(clk), .reset(reset),
        .cfg_vlid(vlid), .cfg_sport(sport), .cfg_dport(dport), .cfg_dip(dip),
        .cfg_neten(neten), .cfg_sn(sn), .cfg_sn_vld(sn_vld), .cfg_vld(cfg_vld),
        .msg_valid(mv), .msg_ready(mrdy), .msg_sop(msop), .msg_eop(meop),
        .msg_data(mdat), .msg_len(mlen), .go(1'b1),
        .fr_valid(fv), .fr_ready(1'b1), .fr_sop(fsop), .fr_eop(feop),
        .fr_data(fd), .fr_net(fnet), .sn_adv(snadv), .pay_replay(preplay)
    );

    afdx_rx_parse #(.DA_CONST(32'h0300_0000)) u_rx (
        .clk(clk), .reset(reset),
        .in_valid(fv), .in_ready(), .in_sop(fsop), .in_eop(feop),
        .in_data(fd), .in_net(fnet),
        .ok(ok), .vlid(rv_vlid), .sip(rv_sip), .dip(rv_dip),
        .sport(rv_sport), .dport(rv_dport), .paylen(rv_paylen), .sn(rv_sn),
        .net(rv_net), .framelen(rv_framelen),
        .err_eth(err_eth), .err_ip(err_ip), .err_udp(err_udp), .err_len(err_len),
        .err_fcs(err_fcs), .vlid_vld(),
        .out_valid(ov), .out_ready(ordy), .out_sop(osop), .out_eop(oeop), .out_data(od)
    );

    always #50 clk = ~clk;

    // 载荷源
    reg [7:0] pay [0:15];
    reg [3:0] pcnt = 0;
    initial begin
        pay[0]="A"; pay[1]="F"; pay[2]="D"; pay[3]="X";
        pay[4]="1"; pay[5]="2"; pay[6]="3"; pay[7]="4";
    end
    always @(posedge clk) begin
        if (preplay) begin mv<=1; msop<=1; mdat<=pay[0]; pcnt<=0; meop<=0; end
        else if (mv && mrdy) begin
            if (pcnt==4'd7) begin mv<=0; msop<=0; meop<=0; end
            else begin pcnt<=pcnt+1'b1; mdat<=pay[pcnt+1'b1];
                       meop<=(pcnt+1'b1==4'd7); msop<=1'b0; end
        end
    end


    // 抓解析出的载荷
    reg [7:0] got [0:15];
    integer np=0;
    always @(posedge clk) if (ov && ordy) begin got[np] <= od; np <= np+1; end

    initial begin
        #200 reset = 0;
        #100; mv<=1; msop<=1; mdat<=pay[0]; meop<=0; pcnt<=0;
        #8000;
        $display("");
        $display("======== TX → RX 对拍 ========");
        $display("  ok=%b (期望1)  错误 eth/ip/udp/len/fcs = %b/%b/%b/%b/%b", ok, err_eth, err_ip, err_udp, err_len, err_fcs);
        $display("  rv_net=%b(期望1)", rv_net);
        $display("  vlid=%04x(期望0123)", rv_vlid);
        $display("  sip=%08x(期望0a000001) dip=%08x(期望e0e00123)", rv_sip, rv_dip);
        $display("  sport=%04x(期望0bb8) dport=%04x(期望0bb9)", rv_sport, rv_dport);
        $display("  paylen=%0d(期望8) sn=%02x(期望05) framelen=%0d(期望64=60+4FCS)", rv_paylen, rv_sn, rv_framelen);
        $display("  载荷=%c%c%c%c%c%c%c%c (期望 AFDX1234)",
                 got[0],got[1],got[2],got[3],got[4],got[5],got[6],got[7]);
        $display("==============================");
        if (ok===1'b1 && rv_vlid===16'h0123 && rv_sport===16'h0BB8 && rv_dport===16'h0BB9 &&
            rv_sip===32'h0A000001 && rv_dip===32'hE0E00123 && rv_paylen===11'd8 &&
            rv_sn===8'h05 && rv_net===1'b1 && rv_framelen===11'd64 && err_fcs===1'b0 &&
            got[0]==8'h41 && got[1]==8'h46 && got[2]==8'h44 && got[3]==8'h58 &&
            got[4]==8'h31 && got[5]==8'h32 && got[6]==8'h33 && got[7]==8'h34)
            $display(">> PASS: 解析器完整还原发送端配置（VLID/IP/端口/长度/SN/载荷）+ FCS 校验通过");
        else
            $display(">> FAIL");
        $finish;
    end
endmodule
