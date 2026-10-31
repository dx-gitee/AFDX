// afdx_tx_path.v —— AFDX 发送路径：消息 → 查 VL 表 → 组帧 → 帧流
`timescale 1ns/1ps

module afdx_tx_path #(
    parameter NVL  = 512,                    // 与 VL 配置表同规模
    parameter NVLW = 9,

    parameter [31:0] SRC_IP = 32'h0A000001
) (
    input  wire        clk,
    input  wire        reset,

    input  wire [15:0] ts_now,               // 8 µs 时基（BAG 用）
    // ---- SAP 口用：目的地址覆盖（§3.3.1.2：SAP 的目的由分区给，不是 VL 固定的）----
    //   为 0 表示"用配置表里的"（采样/队列口走这条）
    input  wire [31:0] ov_dip,
    input  wire [15:0] ov_dport,

    // ---- 消息（来自消息通道）----
    input  wire        msg_valid,
    output wire        msg_ready,
    input  wire        msg_sop,
    input  wire        msg_eop,
    input  wire [7:0]  msg_data,
    input  wire [15:0] msg_port,          // 发送方向 = VLID
    input  wire [13:0] msg_len,

    // ---- 载荷重放（给消息通道）----
    output wire        tx_replay_o,     // 已废弃：恒 0（B 副本已移入 tx_build.pbuf）
    output wire        pay_hold_o,      // 组帧器："B 副本还没重放，先别起下一条"（给发送端口层）
    output wire        bag_hold_o,      // 统计：本拍"消息就绪、但被 BAG 挡住"（自证确定性用）
    output wire        msg_drop,

    // ---- 配置表查表口 ----
    output wire [15:0] cfg_vlid_o,
    input  wire        cfg_hit_i,
    input  wire [127:0] cfg_i,
    input  wire [15:0] cfg_res_vlid_i,    // 当前结果对应的地址（自校验：被别人抢走就不锁存）

    // ---- 帧输出（给 MAC）----
    output wire        fr_valid,
    input  wire        fr_ready,
    output wire        fr_sop,
    output wire        fr_eop,
    output wire [7:0]  fr_data,
    output wire        fr_net
);
    // 组帧器空闲（由内部 u_build 的 idle_o 驱动，见下面例化）—— 用于门控起帧
    wire        u_build_idle;

    // ================= 查表握手：消息一出现就用它的 VLID 去查 =================
    wire  [15:0] lk_vlid = msg_port;
    assign       cfg_vlid_o = lk_vlid;

    reg          cfg_vld_r;
    reg          builder_busy;
    reg  [127:0] cfg_lat;
    reg  [15:0]  port_lat;

    // 查表结果自校验：结果地址必须就是本消息的 VLID 才采用。
    // （afdx_cfg 的读口被 CPU 自检或 RX/IC-RM 抢走的那些拍，结果属于别人 → 不锁存，下一拍自然重来）
    wire cfg_match = (cfg_res_vlid_i == lk_vlid);

    wire cfg_is_cur = cfg_vld_r && (port_lat == msg_port);

    always @(posedge clk) begin
        if (reset) begin
            cfg_vld_r <= 1'b0;
            cfg_lat   <= 128'h0;
            port_lat  <= 16'h0;
        end
        else begin
            // 消息在流上期间，只要结果地址是自己就一直锁存（天然容忍被抢拍）
            if (msg_valid && cfg_match) begin
                cfg_vld_r <= cfg_hit_i;      // 命中才算配置有效
                cfg_lat   <= cfg_i;
                port_lat  <= lk_vlid;
            end
            else if (builder_busy) begin
                // 发送过程中保持
            end
            else cfg_vld_r <= 1'b0;
        end
    end


    (* ramstyle = "M4K" *) reg [7:0] snram [0:NVL-1];
    initial $readmemh("snzero.hex", snram);

    wire [NVLW-1:0] sn_idx = port_lat[NVLW-1:0];


    reg  [7:0]      sn_frame;              // 本帧用的 SN（snram[sn_idx_lat] 的寄存读）
    reg  [NVLW-1:0] sn_idx_lat;            // 本帧的 VL 索引（起帧那拍锁存）

    always @(posedge clk) sn_frame <= snram[sn_idx_lat];   // 寄存读（纯寄存器地址）

    wire        sn_adv_w;
    wire        pay_replay_w;
    assign      tx_replay_o = pay_replay_w;


    (* ramstyle = "M4K" *) reg [16:0] nxt_t [0:NVL-1];
    // 注：下面"复位后状态机逐项清零"是**功能上正确**的初始化方式（也避开了 `initial` 大数组
    //   这个已知陷阱），但**它不是推断问题的解** —— 当初改成这样数字并没有变化，别误会。
    //   清零把有效位也清掉（= 每个 VL 都还没发过帧）✓
    reg              tbl_clr;            // 1 = 还在清零
    reg [NVLW:0]     clr_i;
    reg              tbl_rdy;            // 1 = 表可用（清零完成）
    always @(posedge clk) begin
        if (reset) begin
            tbl_clr <= 1'b1; clr_i <= {(NVLW+1){1'b0}}; tbl_rdy <= 1'b0;
        end
        else if (tbl_clr) begin
            if (clr_i == (NVL + 1)) begin
                tbl_clr <= 1'b0; tbl_rdy <= 1'b1;
            end
            else clr_i <= clr_i + 1'b1;
        end
    end

    reg [NVLW-1:0] vli_q;
    reg [16:0]     nxt_raw_q;
    reg [NVLW-1:0] rd_adr_q;
    reg [NVLW-1:0] scan_i;
    wire [NVLW-1:0] vli = port_lat[NVLW-1:0];

    wire [NVLW-1:0] rd_adr = msg_valid ? vli : scan_i;
    wire [15:0]    nxt_q     = nxt_raw_q[15:0];   // 上次起帧时刻（+BAG+1）
    wire           nxt_can_q = nxt_raw_q[16];

    always @(posedge clk) begin
        nxt_raw_q <= nxt_t[rd_adr];          // 寄存读（纯寄存器地址）
        rd_adr_q  <= rd_adr;
        vli_q     <= vli;
        // 空闲时轮转扫描（有消息在流上时不扫，保证那条 VL 每拍都被检查）
        if (reset || tbl_clr)      scan_i <= {NVLW{1'b0}};
        else if (!msg_valid)       scan_i <= scan_i + 1'b1;
    end

    wire [3:0]  bsh       = cfg_lat[35:32];                        // w1[3:0] = bag_shift

    wire [15:0] bag_ticks = (bsh > 4'd7) ? 16'd16000 : (16'd125 << bsh);

    wire        bag_ok    = msg_valid && (vli_q == vli) && (rd_adr_q == vli_q) && nxt_can_q;
    // 维护用：当前读到的这一项"还不可发"、但时刻已经到 → 把标志置回 1
    //   （只在"不可发"时判，此时 `ts_now - nxt_q` 必然从 `65536-(bag+1)` 单调走到 0，
    //     全程落在 `≥0x8000 → <0x8000` 这一次翻转里，不会歧义）
    // 对"刚刚读到的这一项"（`rd_adr_q`）补置位 —— 空闲时它就是扫描到的那条 VL
    wire        bag_set   = !nxt_can_q && ((ts_now - nxt_q) < 16'h8000);


    wire [13:0] lmax_raw = {3'b000, cfg_lat[26:16]};
    wire [13:0] lmax_w   = (lmax_raw > 14'd1471) ? 14'd1471 : lmax_raw;
    wire        len_bad = cfg_is_cur && (msg_len > lmax_w);

    wire        cfg_miss = msg_valid && msg_sop && cfg_match && !cfg_hit_i && tbl_rdy;
    // 只脉冲一次：端口层收到就中止本条消息（`msg_valid` 下一拍落下）
    reg         drop_done;
    assign      msg_drop = msg_valid && msg_sop && (len_bad || cfg_miss) && !drop_done;
    always @(posedge clk) begin
        if (reset)            drop_done <= 1'b0;
        else if (!msg_valid)  drop_done <= 1'b0;   // 消息走了 → 复位，下一条重新判
        else if (msg_drop)    drop_done <= 1'b1;
    end
    // 统计用：配置已经有效（知道 BAG 了）但这一拍不许起帧 → 就是"被 BAG 挡住"
    assign      bag_hold_o = msg_valid && cfg_is_cur && !bag_ok;


    wire        bld_idle  = u_build_idle;                     // 组帧器空闲（S_IDLE）
    wire        go_int    = bag_ok && tbl_rdy && !len_bad && bld_idle;
    wire        tx_start  = msg_valid && msg_sop && cfg_is_cur && go_int;

    always @(posedge clk) begin

        if (tbl_clr && (clr_i < NVL)) nxt_t[clr_i[NVLW-1:0]] <= 17'h10000;
        else if (tx_start)   nxt_t[vli]  <= {1'b0, (ts_now + bag_ticks + 16'd1)};  // 刚发过 → 不可发
        else if (bag_set)    nxt_t[rd_adr_q] <= {1'b1, nxt_q};                     // BAG 已满 → 置可发
    end

    // ================= SN 的按帧锁存与递增（见上面 sn_frame 处的说明）=================
    always @(posedge clk) begin
        if (reset)             sn_idx_lat <= {NVLW{1'b0}};
        else if (tx_start)     sn_idx_lat <= sn_idx;   // 此刻 port_lat 必然就是本消息的 VL
    end

    // 帧末递增：写**本帧锁存下来的那个 VL**，并用**本帧实际发出去的 SN** +1（255 → 1 回绕）
    always @(posedge clk) begin
        if (sn_adv_w)
            snram[sn_idx_lat] <= (sn_frame == 8'd255) ? 8'd1 : (sn_frame + 8'd1);
    end

    // ================= 组帧器 =================
    wire [15:0] cfg_vlid_w  = cfg_lat[15:0];
    wire [10:0] cfg_lmax_w  = cfg_lat[26:16];
    wire [1:0]  cfg_neten_w = cfg_lat[28:27];

    wire        cfg_snvld_w = cfg_lat[31];
    wire [15:0] cfg_sport_w = cfg_lat[79:64];   // w2[15:0]（不是 95:80，那是 w2[31:16]）
    // 目的信息：SAP 口取 CPU 寄存器写的覆盖值（非 0 才覆盖），其余口取配置表
    wire [15:0] cfg_dport_w = (ov_dport != 16'h0) ? ov_dport : cfg_lat[63:48];
    wire [31:0] cfg_dip_w   = (ov_dip   != 32'h0) ? ov_dip   : cfg_lat[127:96];

    afdx_tx_build #(.SRC_IP(SRC_IP)) u_build (
        .clk        (clk),
        .reset      (reset),
        .cfg_vlid   (cfg_vlid_w),
        .cfg_sport  (cfg_sport_w),
        .cfg_dport  (cfg_dport_w),
        .cfg_dip    (cfg_dip_w),
        .cfg_neten  (cfg_neten_w),
        .cfg_sn     (sn_frame),
        .cfg_sn_vld (cfg_snvld_w),
        .cfg_vld    (cfg_is_cur),
        .msg_valid  (msg_valid),
        .msg_ready  (msg_ready),
        .msg_sop    (msg_sop),
        .msg_eop    (msg_eop),
        .msg_data   (msg_data),
        .msg_len    (msg_len[10:0]),

        .go         (go_int),   // BAG 未到期 / 表没就绪 / 超 Lmax / 组帧器忙 都不许起帧
        .idle_o     (u_build_idle),
        .pay_replay (pay_replay_w),
        .pay_hold   (pay_hold_o),
        .fr_valid   (fr_valid),
        .fr_ready   (fr_ready),
        .fr_sop     (fr_sop),
        .fr_eop     (fr_eop),
        .fr_data    (fr_data),
        .fr_net     (fr_net),
        .sn_adv     (sn_adv_w)
    );

    always @(posedge clk) begin
        if (reset) builder_busy <= 1'b0;
        else if (msg_valid && msg_sop) builder_busy <= 1'b1;

        else if (sn_adv_w || !msg_valid || msg_drop) builder_busy <= 1'b0;
    end

endmodule
