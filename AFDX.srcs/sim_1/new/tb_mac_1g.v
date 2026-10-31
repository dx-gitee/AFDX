// tb_mac_1g.v —— 千兆冒烟：MAC IP + PHY 模型，125 MHz 线侧
`timescale 1ns/1ps

module tb_mac_1g #(
    parameter integer SPEED_1G   = 1,
    parameter integer DEBUG_MDIO = 1
) ();
    localparam integer HALF = 4;      // 125 MHz 半周期 = 4 ns

    reg clk    = 1'b0;
    reg p0_rxc = 1'b0;
    reg reset  = 1'b1;
    always #HALF clk    = ~clk;       // 125 MHz 系统/用户时钟
    always #HALF p0_rxc = ~p0_rxc;    // 125 MHz 线侧（GMII 千兆）

    // ---- GMII 自回环（发送端在 gtxc 下降沿驱动，接收端在 rxc 上升沿采样）----
    wire [7:0] gmii_txd;
    wire       gmii_txen, gmii_txer, gmii_gtxc;
    reg  [7:0] lb_d = 8'h00;
    reg        lb_v = 1'b0;
    always @(negedge p0_rxc) begin
        lb_d <= gmii_txd;
        lb_v <= gmii_txen;
    end
    wire [7:0] gmii_rxd   = lb_d;
    wire       gmii_rxdv  = lb_v;
    wire       gmii_rxer  = 1'b0;

    // ---- 用户侧 TX / RX ----
    reg        tx_wren = 1'b0;
    reg  [7:0] tx_data = 8'h00;
    reg        tx_sop  = 1'b0;
    reg        tx_eop  = 1'b0;
    wire       ff_tx_rdy, ff_tx_al_full;

    wire       rx_vld, rx_sop, rx_eop, rx_err;
    wire [7:0] rx_data;

    // ---- MAC 寄存器配置 ----
    wire        reg_wr, reg_rd;
    wire [3:0]  reg_addr;
    wire [31:0] reg_din, reg_dout;
    wire        reg_bsy, cfg_done;

    wire mdc, mdio, phy_rstb0;
    pullup (mdio);

    afdx_mac_cfg u_cfg (
        .clk(clk), .reset(reset),
        .reg_wr(reg_wr), .reg_rd(reg_rd), .reg_addr(reg_addr),
        .reg_data_in(reg_din), .reg_data_out(reg_dout),
        .reg_acc_bsy(reg_bsy), .done(cfg_done)
    );

    triplespeedethernet_1 u_mac (
        .mdc            (mdc),
        .phy_rstb0      (phy_rstb0),
        .p0_gtxc        (gmii_gtxc),
        .p0_txd         (gmii_txd),
        .p0_txen        (gmii_txen),
        .p0_txer        (gmii_txer),
        .ff_tx_rdy      (ff_tx_rdy),
        .ff_tx_al_full  (ff_tx_al_full),
        .ff_rx_data_vld (rx_vld),
        .ff_rx_sop      (rx_sop),
        .ff_rx_eop      (rx_eop),
        .ff_rx_data     (rx_data),
        .ff_rx_err      (rx_err),
        .reg_data_out   (reg_dout),
        .reg_acc_bsy    (reg_bsy),
        .clk_sys        (clk),
        .rst_b          (~reset),
        .p0_txc         (1'b0),
        .p0_rxc         (p0_rxc),
        .p0_rxd         (gmii_rxd),
        .p0_rxdv        (gmii_rxdv),
        .p0_rxer        (gmii_rxer),
        .p0_col         (1'b0),
        .p0_crs         (1'b0),
        .ff_tx_clk      (clk),
        .ff_tx_data     (tx_data),
        .ff_tx_sop      (tx_sop),
        .ff_tx_eop      (tx_eop),
        .ff_tx_err      (1'b0),
        .ff_tx_wren     (tx_wren),
        .ff_rx_clk      (clk),
        .ff_rx_rdy      (1'b1),
        .reg_wr         (reg_wr),
        .reg_rd         (reg_rd),
        .reg_addr       (reg_addr),
        .reg_data_in    (reg_din),
        .mdio           (mdio)
    );

    phy_mdio_slave #(
        .PHY_ADDR  (5'd1),
        .SPEED_1G  (SPEED_1G),
        .DEBUG_MDIO(DEBUG_MDIO)
    ) u_phy (.mdc(mdc), .mdio(mdio));

    // ---- 统计：TX 字节数、相邻字节间隔（拍） ----
    integer cyc = 0;
    integer n_tx = 0;
    integer last_cyc = -1;
    integer min_gap = 1000000;
    integer max_gap = 0;
    integer gap;

    always @(posedge clk) cyc <= cyc + 1;

    always @(posedge clk) begin
        if (gmii_txen) begin
            n_tx <= n_tx + 1;
            if (last_cyc >= 0) begin
                gap = cyc - last_cyc;
                if (gap < min_gap) min_gap = gap;
                if (gap > max_gap) max_gap = gap;
            end
            last_cyc <= cyc;
        end
    end

    // ---- 统计：RX 字节 ----
    integer n_rx = 0;
    always @(posedge clk) if (rx_vld) n_rx <= n_rx + 1;

    // ---- R40：MAC 自身时延（用户口 → GMII → 回环 → 用户口） ----
    //   R40 的时延是"发送端 GMII SFD → 接收端 GMII SFD"。本台只有一颗 MAC，
    //   能量的是它自己的贡献：**用户口提交 → GMII 上出现 SFD**（TX 方向）
    //   + **GMII SFD → 用户口首字节**（RX 方向，含 MAC 内部 FIFO）。
    integer tx_bc = 0;                                  // 本帧已上 GMII 的字节数
    integer t_tx_user = -1, t_tx_sfd = -1, t_rx_sfd = -1;
    reg [7:0] rx_first = 8'h00;
    always @(posedge clk) begin
        if (tx_wren && ff_tx_rdy && tx_sop) begin
            t_tx_user = cyc; tx_bc = 0;
        end
        if (gmii_txen) begin
            tx_bc = tx_bc + 1;
            // 前 7 个字节是前导码 0x55，第 8 个是 SFD 0xD5
            if ((tx_bc == 8) && (t_tx_sfd < 0)) t_tx_sfd = cyc;
        end
        if (rx_vld && rx_sop && (t_rx_sfd < 0)) begin
            t_rx_sfd = cyc; rx_first = rx_data;
        end
    end

    // ---- 探针：厂商 IP 内部的 speed/link（网表保留了这两个线名）----
    integer mdc_edges = 0;
    always @(posedge mdc) mdc_edges = mdc_edges + 1;

    // ---- 发送一个 64 字节帧 ----
    reg [7:0] fr [0:63];
    integer i;
    initial begin
        fr[ 0]=8'h03; fr[ 1]=8'h00; fr[ 2]=8'h00; fr[ 3]=8'h00; fr[ 4]=8'h01; fr[ 5]=8'h23;
        fr[ 6]=8'h02; fr[ 7]=8'h00; fr[ 8]=8'h00; fr[ 9]=8'h00; fr[10]=8'h01; fr[11]=8'h20;
        fr[12]=8'h08; fr[13]=8'h00;
        for (i=14; i<60; i=i+1) fr[i] = i[7:0];
        fr[60]=8'hDE; fr[61]=8'hAD; fr[62]=8'hBE; fr[63]=8'hEF;
    end

    task send_frame;
        integer k;
        begin
            for (k=0; k<64; k=k+1) begin
                @(negedge clk);
                tx_data <= fr[k];
                tx_sop  <= (k==0);
                tx_eop  <= (k==63);
                tx_wren <= 1'b1;
                // 被接受（wren && rdy 同一拍）才前进
                @(posedge clk);
                while (!ff_tx_rdy) @(posedge clk);
            end
            @(negedge clk);
            tx_wren <= 1'b0; tx_sop <= 1'b0; tx_eop <= 1'b0;
        end
    endtask

    initial begin
        reset = 1'b1;
        repeat (20) @(posedge clk);
        reset = 1'b0;

        wait (cfg_done);
        $display("[1g] MAC cfg done @%0t ns", $time);

        // 给 MDIO 自协商/轮询留时间（真实 PHY 通常几百 ms；模型里 ~100 us 足够，这里给 2 ms）
        #2000000;
        $display("[1g] after 2ms: n_tx=%0d n_rx=%0d mdc_edges=%0d",
                 n_tx, n_rx, mdc_edges);

        n_tx = 0; n_rx = 0; min_gap = 1000000; max_gap = 0; last_cyc = -1;
        send_frame;
        #200000;   // 200 us，等帧走完
        $display("[1g] RESULT n_tx=%0d n_rx=%0d min_gap=%0d max_gap=%0d  (1000M 期望 gap=1, 100M 期望 gap=10)",
                 n_tx, n_rx, min_gap, max_gap);
        // R40：MAC 自身时延
        if (t_tx_sfd > 0 && t_rx_sfd > 0)
            $display("[1g] MAC 时延：用户口提交→GMII SFD = %0d 拍（%0d ns）；GMII SFD→用户口首字节 = %0d 拍（%0d ns）；合计 %0d 拍 = %0d ns；RX 首字节=0x%02x",
                     t_tx_sfd - t_tx_user, (t_tx_sfd - t_tx_user)*8,
                     t_rx_sfd - t_tx_sfd, (t_rx_sfd - t_tx_sfd)*8,
                     t_rx_sfd - t_tx_user, (t_rx_sfd - t_tx_user)*8, rx_first);
        else
            $display("[1g] MAC 时延：**没量到**（t_tx_user=%0d t_tx_sfd=%0d t_rx_sfd=%0d）",
                     t_tx_user, t_tx_sfd, t_rx_sfd);

        if ((n_tx >= 72) && (n_rx == 64) && (min_gap == 1) && (max_gap == 1)
            && (t_tx_sfd > 0) && (t_rx_sfd > t_tx_sfd) && ((t_rx_sfd - t_tx_user) < 2000))
            $display(">> PASS: 1G MAC 冒烟（GMII 字节间隔 = 1 拍、64 B 帧完整回环）");
        else
            $display(">> FAIL: 1G MAC 冒烟（n_tx=%0d n_rx=%0d gap=%0d..%0d）",
                     n_tx, n_rx, min_gap, max_gap);
        $finish;
    end

    // 看门狗
    initial begin
        #20000000;   // 20 ms
        $display("[1g] TIMEOUT n_tx=%0d n_rx=%0d", n_tx, n_rx);
        $finish;
    end
endmodule
