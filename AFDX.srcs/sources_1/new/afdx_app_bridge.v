// afdx_app_bridge.v —— 应用侧接口 ↔ AFDX 数据面 的桥
`timescale 1ns/1ps

module afdx_app_bridge #(
    parameter integer NPW          = 4,
    parameter integer MAXLEN       = 1471,   // 入向单条上限
    parameter integer OUT_MAX      = 8192,
    parameter integer APP_PORT_DEF = 0,

    parameter integer VLID_MODE    = 0
) (
    input  wire        clk,
    input  wire        reset,

    // ---------------- 应用侧 ----------------
    input  wire        app_rx_valid,
    input  wire [7:0]  app_rx_data,

    input  wire [15:0] app_vlid,
    input  wire [NPW-1:0] app_port,
    output wire        app_tx_valid,
    output wire [7:0]  app_tx_data,
    output reg  [NPW-1:0] app_tx_port,
    output reg  [15:0] app_tx_vlid,     // 交付消息的 VLID（段首拍有效）
    output reg         err,

    // ---------------- 发给 tx_path 的消息流（上级 mux 仲裁） ----------------
    output wire        m_valid,
    input  wire        m_ready,
    output wire        m_sop,
    output wire        m_eop,
    output wire [7:0]  m_data,
    output reg  [15:0] m_vlid,
    output wire [13:0] m_len,
    output wire        m_busy,
    input  wire        m_drop,

    // ---------------- 旁听 reasm 输出（只读） ----------------
    input  wire        r_valid,
    input  wire        r_sop,
    input  wire        r_eop,
    input  wire [7:0]  r_data,
    input  wire [NPW-1:0] r_port,
    input  wire        r_commit,
    input  wire        r_abort,
    input  wire        r_vbytes,
    input  wire [15:0] r_vlid
);
    // =====================================================================
    // 端口号安全化
    // =====================================================================
    reg [NPW-1:0] sel_port;
    always @(*) begin
        case (app_port)
            4'd0: sel_port = 4'd0;
            4'd1: sel_port = 4'd1;
            4'd2: sel_port = 4'd2;
            4'd3: sel_port = 4'd3;
            4'd4: sel_port = 4'd4;
            4'd5: sel_port = 4'd5;
            4'd6: sel_port = 4'd6;
            4'd7: sel_port = 4'd7;
            4'd8: sel_port = 4'd8;
            4'd9: sel_port = 4'd9;
            default: sel_port = APP_PORT_DEF[NPW-1:0];
        endcase
    end

    wire [15:0] sel_vlid;
    afdx_port_vlid #(.NPW(NPW)) u_vlidmap (.port(sel_port), .vlid(sel_vlid));
    wire        vlid_en = (VLID_MODE != 0);   // 参数（不是端口）⇒ 未连接不会产生 X


    (* ramstyle = "M4K" *) reg [31:0] ibuf [0:4095];     // 入向 8 条 × 512×32
    (* ramstyle = "M4K" *) reg [31:0] ebuf [0:8191];     // 出向 4 bank × 2048×32（每 bank 8 KB）

    // =====================================================================
    // 入向
    // =====================================================================
    reg [31:0] iwacc;
    reg [1:0]  iwb;
    reg [8:0]  iwp;
    reg [8:0]  irp;
    reg [31:0] irw;
    reg [31:0] iwcur, iwnxt;
    reg [1:0]  irb;
    reg [13:0] ilen;
    reg [13:0] ca_i;
    reg [NPW-1:0] i_port_q;
    reg [15:0] i_vlid_q;
    reg        i_use;              // 本条入向消息是否 VLID 模式
    reg [2:0]  wsl;                // 正在写的槽
    reg [2:0]  whead, wtail;       // 消息 FIFO 头/尾
    reg [3:0]  wcnt;               // 已收条数（0..8）
    reg [2:0]  phe;                // presenter 正在读的槽
    reg [13:0] plen_i;             // presenter 本条长度
    reg [13:0] mlen_q [0:7];       // 每槽元信息：字节数
    reg [15:0] mvlid_q[0:7];       // 每槽元信息：VLID
    reg [NPW-1:0] mport_q [0:7];   // 每槽元信息：端口号（旧语义用）
    reg        muse_q [0:7];       // 每槽元信息：VLID 模式
    reg        ipend, idrop;

    localparam [1:0] I_IDLE = 2'd0, I_WR = 2'd1;
    localparam [2:0] S_IDLE = 3'd0, S_LD = 3'd1, S_LD2 = 3'd2, S_LD3 = 3'd3, S_PRES = 3'd4;
    reg [1:0] ist;
    reg [2:0] sst;

    // =====================================================================
    // 出向
    // =====================================================================
    reg [1:0]  cst;
    localparam [1:0] C_IDLE = 2'd0, C_WR = 2'd1, C_DRAIN = 2'd2;


    reg  [15:0] cst_tmo;
    wire        cst_abandon = (cst_tmo == 16'hFFFF);
    always @(posedge clk) begin
        if (reset)                             cst_tmo <= 16'd0;

        else if ((cst == C_IDLE) ||
                 ((cst == C_WR) && r_valid) ||
                 ((cst == C_DRAIN) && r_valid && r_eop))  cst_tmo <= 16'd0;   // 不用 fl_eop（它在本块之后才声明）
        else if (!cst_abandon)                 cst_tmo <= cst_tmo + 16'd1;
    end
    reg [1:0]  wbank;
    reg [10:0] ewp;                    // bank 内字指针（0..2047）
    reg [1:0]  ewb;
    reg [31:0] ewacc;
    reg [13:0] elen;
    reg [NPW-1:0] wport;               // sop 那拍锁存的端口

    reg [NPW-1:0] wsel;
    reg           wuse;                // sop 那拍锁存的"VLID 模式"（1 ⇒ 交付不过滤端口）
    reg [15:0]    wvlid;               // sop 那拍锁存的本帧 VLID（分片时为该数据报的 VLID）
    wire          sel_hit = wuse ? 1'b1 : (wport == wsel);
    reg [3:0]  bank_used;

    // verdict 队列（按"帧"顺序；depth 8）：kind 0=capture 1=byte 2=tombstone
    reg [1:0]  vkind [0:7];
    reg [1:0]  vbank [0:7];
    reg [13:0] vlen  [0:7];
    reg [7:0]  vbyte [0:7];
    reg [NPW-1:0] vport [0:7];
    reg [15:0] vvli [0:7];
    reg [2:0]  vhead, vtail;
    reg [3:0]  vcnt;

    reg [7:0]  v_drop;
    reg [15:0] v_tmo;
    reg [5:0]  v_abandon;

    // delivery 队列（depth 8）
    reg [1:0]  dkind [0:7];
    reg [1:0]  dbank [0:7];
    reg [13:0] dlen  [0:7];
    reg [7:0]  dbyte [0:7];
    reg [NPW-1:0] dport [0:7];
    reg [15:0] dvli [0:7];
    reg [2:0]  dhead, dtail;
    reg [3:0]  dcnt;

    // presenter
    reg [2:0]  pst;
    localparam [2:0] P_IDLE = 3'd0, P_RD = 3'd1, P_LD = 3'd2, P_LD2 = 3'd3, P_PRES = 3'd4;
    reg        pkind;                  // 0=capture 1=byte
    reg [1:0]  pbank;
    reg [13:0] plen;
    reg [13:0] ce_i;
    reg [1:0]  erb;
    reg [10:0] erp;
    reg [12:0] era;                    // {pbank, 11-bit ptr}
    reg [31:0] erw;
    reg [31:0] ewcur, ewnxt;

    wire any_free = ~&bank_used;
    reg [1:0] free_b;
    always @(*) begin
        if      (!bank_used[0]) free_b = 2'd0;
        else if (!bank_used[1]) free_b = 2'd1;
        else if (!bank_used[2]) free_b = 2'd2;
        else                    free_b = 2'd3;
    end


    wire v_recv     = r_vbytes && (r_commit || r_abort);
    wire v_do_aband = v_recv && (v_abandon != 6'd0);
    wire v_do_pop   = v_recv && (v_abandon == 6'd0) && (vcnt != 0);
    wire v_timeout  = (vcnt != 0) && (v_tmo == 16'hFFFF) && !v_recv;

    wire v_drop_tmo = (vcnt == 0) && (v_drop != 8'd0) && (v_tmo == 16'hFFFF) && !v_recv;


    wire d_pop      = (pst == P_IDLE) && (dcnt != 0);
    wire d_has_room = (dcnt != 4'd8) || d_pop;   // 本拍有 pop ⇒ 尾指针正好复用该槽
    wire d_push     = v_do_pop && r_commit && (vkind[vhead] != 2'd2) && d_has_room;


    wire fl_sop  = r_valid && r_sop;
    wire fl_eop  = r_valid && r_eop;

    wire ovf_now = (cst == C_WR) && r_valid && !r_sop && (elen >= OUT_MAX);

    wire p_push_now = ((cst == C_IDLE ) && fl_sop && fl_eop) ||   // 1 字节完整帧
                      ((cst == C_WR   ) && fl_sop && fl_eop) ||   // C_WR 里紧跟的 1 字节帧
                      ((cst == C_WR   ) && fl_eop && !r_sop) ||   // 正常帧末
                      ((cst == C_DRAIN) && fl_eop) ||             // 排空帧的帧末

                      ((cst == C_IDLE ) && fl_eop && !r_sop);

    wire v_has_room = (vcnt != 4'd8);

    wire v_accept   = v_has_room && ((v_drop == 8'd0) || v_drop_tmo);
    wire v_push_eff = (p_push_now && v_accept)  ? 3'd1 : 3'd0;
    wire [7:0] v_drop_add = (p_push_now && !v_accept) ? 8'd1 : 8'd0;
    wire v_drop_sub = v_recv && (v_abandon == 6'd0) && (vcnt == 0) && (v_drop != 0);

    // 字节选择
    reg [7:0] m_byte, a_byte;
    always @(*) begin
        case (irb)
            2'd0: m_byte = iwcur[7:0];
            2'd1: m_byte = iwcur[15:8];
            2'd2: m_byte = iwcur[23:16];
            default: m_byte = iwcur[31:24];
        endcase
        case (erb)
            2'd0: a_byte = ewcur[7:0];
            2'd1: a_byte = ewcur[15:8];
            2'd2: a_byte = ewcur[23:16];
            default: a_byte = ewcur[31:24];
        endcase
    end


    wire w_push_now = (ist == I_WR) && (app_rx_valid === 1'b0) && (idrop === 1'b0) && (ilen != 14'd0);
    wire w_pop_now  = (sst == S_PRES) &&
                      (((m_ready === 1'b1) && (ca_i == plen_i - 1'b1)) || (m_drop === 1'b1));

    assign m_valid = (sst == S_PRES);
    assign m_data  = m_byte;
    assign m_sop   = m_valid && (ca_i == 14'd0);

    assign m_eop   = m_valid && (ca_i == plen_i - 1'b1);
    assign m_len   = plen_i;
    assign m_busy  = (sst != S_IDLE);

    assign app_tx_valid = (pst == P_PRES);
    assign app_tx_data  = a_byte;


    always @(posedge clk) begin

        irw <= ibuf[{whead, irp}];
        erw <= ebuf[era];

        if (reset) begin
            ist <= I_IDLE; sst <= S_IDLE;
            iwacc <= 0; iwb <= 0; iwp <= 0; irp <= 0; iwcur <= 0; iwnxt <= 0; irb <= 0;
            ilen <= 0; ca_i <= 0; ipend <= 0; idrop <= 0;
            i_port_q <= 0; i_vlid_q <= 0;
            m_vlid <= 0; app_tx_port <= 0; app_tx_vlid <= 0; err <= 0;
            i_use <= 0; wsl <= 0; whead <= 0; wtail <= 0; wcnt <= 0; phe <= 0; plen_i <= 0;
            wuse <= 0; wvlid <= 0;
            cst <= C_IDLE; wbank <= 0; ewp <= 0; ewb <= 0; ewacc <= 0; elen <= 0;
            wport <= 0; wsel <= 0; bank_used <= 4'b0000;
            vhead <= 0; vtail <= 0; vcnt <= 0; v_tmo <= 0; v_abandon <= 0; v_drop <= 0;
            dhead <= 0; dtail <= 0; dcnt <= 0;
            pst <= P_IDLE; pkind <= 0; pbank <= 0; plen <= 0; ce_i <= 0; erb <= 0;
            erp <= 0; era <= 0; erw <= 0; ewcur <= 0; ewnxt <= 0;
        end else begin

            wcnt <= wcnt + {{3{1'b0}}, w_push_now} - {{3{1'b0}}, w_pop_now};

            // ================= 入向 =================
            case (ist)
                I_IDLE: begin
                    if (app_rx_valid) begin
                        if (wcnt == 4'd8) begin
                            // 消息 FIFO 满 ⇒ 只丢**这一条**（可见）；正在发的/已排队的都不受影响
                            idrop <= 1'b1; err <= 1'b1;
                        end else begin
                            wsl      <= wtail;
                            i_port_q <= sel_port;
                            // VLID 模式下 `app_vlid=0` 仍回落到端口映射（对未驱动更友好）
                            i_vlid_q <= (vlid_en && (app_vlid != 16'd0)) ? app_vlid : sel_vlid;
                            i_use    <= vlid_en;
                            iwacc    <= {24'h0, app_rx_data};
                            iwb      <= 2'd1;
                            iwp      <= 0;
                            ilen     <= 14'd1;
                            idrop    <= 1'b0;
                        end
                        ist <= I_WR;
                    end
                end
                I_WR: begin
                    if (app_rx_valid) begin
                        if (!idrop) begin
                            if (ilen < MAXLEN) begin
                                ilen <= ilen + 1'b1;
                                if (iwb == 2'd3) begin
                                    ibuf[{wsl, iwp}] <= {app_rx_data, iwacc[23:0]};
                                    iwp   <= iwp + 1'b1;
                                    iwb   <= 2'd0;
                                    iwacc <= 32'h0;
                                end else begin
                                    iwacc[{iwb, 3'b000} +: 8] <= app_rx_data;
                                    iwb   <= iwb + 2'd1;
                                end
                            end else begin
                                idrop <= 1'b1; err <= 1'b1;
                            end
                        end
                    end else begin
                        if (!idrop && (ilen != 0)) begin
                            if (iwb != 2'd0) ibuf[{wsl, iwp}] <= iwacc;
                            mlen_q [wtail] <= ilen;
                            mvlid_q[wtail] <= i_vlid_q;
                            mport_q[wtail] <= i_port_q;
                            muse_q [wtail] <= i_use;
                            wtail <= wtail + 1'b1;
                        end else if (!idrop) begin
                            err <= 1'b1;
                        end
                        ist <= I_IDLE;
                    end
                end
                default: ist <= I_IDLE;
            endcase

            case (sst)
                S_IDLE: begin
                    irp <= 0;
                    if (wcnt != 4'd0) begin
                        phe     <= whead;
                        plen_i  <= mlen_q[whead];
                        m_vlid  <= mvlid_q[whead];
                        ca_i    <= 0;
                        irb     <= 0;
                        sst     <= S_LD;
                    end
                end
                S_LD: begin iwcur <= irw; irp <= 1; sst <= S_LD2; end
                S_LD2: begin irp <= 2; sst <= S_LD3; end
                S_LD3: begin iwnxt <= irw; sst <= S_PRES; end
                S_PRES: begin
                    if (m_ready) begin
                        if (ca_i == plen_i - 1'b1) begin
                            sst <= S_IDLE; ca_i <= 0; irp <= 0;
                            whead <= whead + 1'b1;
                        end else begin
                            ca_i <= ca_i + 1'b1;
                            if (irb == 2'd3) begin
                                irb <= 2'd0; iwcur <= iwnxt; iwnxt <= irw; irp <= irp + 1'b1;
                            end else irb <= irb + 2'd1;
                        end
                    end
                    if (m_drop) begin
                        sst <= S_IDLE; irp <= 0; err <= 1'b1;
                        whead <= whead + 1'b1;
                    end
                end
                default: sst <= S_IDLE;
            endcase

            // ================= 出向① capture =================
            case (cst)
                C_IDLE: begin
                    if (fl_sop) begin
                        if (fl_eop) begin
                            // 1 字节完整帧：入队（端口不匹配 → tombstone）/ 或记欠账
                            if (v_accept) begin
                                // 端口不匹配 → tombstone（占位不交付），保持判决配对
                                vkind[vtail] <= (vlid_en ? 1'b1 : (r_port == sel_port)) ? 2'd1 : 2'd2;
                                vlen [vtail] <= 14'd1;
                                vbyte[vtail] <= r_data;
                                vport[vtail] <= r_port;
                                vvli [vtail] <= r_vlid;

                                vbank[vtail] <= 2'd0;
                                vtail <= vtail + 1'b1;
                            end else begin
                                err <= 1'b1;             // 队列满：这条不给槽（v_drop_add 已记）
                            end
                        end else if (v_accept && any_free) begin
                            wbank <= free_b;
                            bank_used[free_b] <= 1'b1;
                            ewp   <= 0;
                            ewb   <= 2'd1;
                            ewacc <= {24'h0, r_data};
                            elen  <= 14'd1;
                            wport <= r_port;
                            wsel  <= sel_port;
                            wuse  <= vlid_en;
                            wvlid <= r_vlid;
                            cst   <= C_WR;
                        end else begin
                            // 没有空 bank / 队列暂时不能收 ⇒ **整帧排空**，
                            //   在它的 eop 那拍才结账（截断帧 ⇒ 不占槽、不记账）
                            err <= 1'b1;
                            cst <= C_DRAIN;
                        end
                    end else if (fl_eop) begin

                        err <= 1'b1;
                        if (v_accept) begin
                            vkind[vtail] <= 2'd2;
                            vvli [vtail] <= 16'h0;

                            vbank[vtail] <= 2'd0;
                            vlen [vtail] <= 14'd0;
                            vbyte[vtail] <= 8'h00;
                            vport[vtail] <= 4'd0;
                            vtail <= vtail + 1'b1;
                        end
                    end
                end
                C_WR: begin
                    if (r_valid && r_sop) begin

                        err <= 1'b1;
                        if (r_eop) begin
                            // 新帧 = 1 字节，**是完整帧**（有 eop ⇒ 会有判决）⇒ 占 1 个槽
                            if (v_accept) begin
                                vkind[vtail] <= (vlid_en ? 1'b1 : (r_port == sel_port)) ? 2'd1 : 2'd2;
                                vbank[vtail] <= 2'd0;
                                vlen [vtail] <= 14'd1;
                                vbyte[vtail] <= r_data;
                                vport[vtail] <= r_port;
                                vvli [vtail] <= r_vlid;
                                vtail <= vtail + 1'b1;
                            end else begin
                                err <= 1'b1;        // 条数由 v_drop_add 记
                            end
                            bank_used[wbank] <= 1'b0;
                            cst <= C_IDLE;
                        end else if (v_accept) begin
                            // 新帧继续（复用同一个 wbank，旧半条的数据被覆盖）
                            ewp   <= 0;
                            ewb   <= 2'd1;
                            ewacc <= {24'h0, r_data};
                            elen  <= 14'd1;
                            wport <= r_port;
                            wsel  <= sel_port;
                            wuse  <= vlid_en;
                            wvlid <= r_vlid;
                        end else begin
                            // 队列满 / 有欠账 ⇒ 这条新帧整条排空（eop 那拍结账）
                            err <= 1'b1;
                            bank_used[wbank] <= 1'b0;
                            cst <= C_DRAIN;
                        end
                    end else if (r_valid && r_eop) begin

                        if (ovf_now) begin
                            err <= 1'b1;
                            if (v_accept) begin
                                vkind[vtail] <= 2'd2;
                                vvli [vtail] <= wvlid;
                                vbank[vtail] <= 2'd0;
                                vlen [vtail] <= 14'd0;
                                vbyte[vtail] <= 8'h00;
                                vport[vtail] <= 4'd0;
                                vtail <= vtail + 1'b1;
                            end
                            bank_used[wbank] <= 1'b0;
                            cst <= C_IDLE;
                        end else begin
                        case (ewb)
                            2'd0: ebuf[{wbank, ewp}] <= {24'h0, r_data};
                            2'd1: ebuf[{wbank, ewp}] <= {16'h0, r_data, ewacc[7:0]};
                            2'd2: ebuf[{wbank, ewp}] <= {8'h0, r_data, ewacc[15:0]};
                            default: ebuf[{wbank, ewp}] <= {r_data, ewacc[23:0]};
                        endcase
                        if (v_accept) begin

                            vkind[vtail] <= sel_hit ? 2'd0 : 2'd2;
                            vbank[vtail] <= wbank;
                            vlen [vtail] <= elen + 1'b1;
                            vport[vtail] <= wport;
                            vvli [vtail] <= wvlid;

                            vbyte[vtail] <= 8'h00;
                            vtail <= vtail + 1'b1;
                            if (!sel_hit) bank_used[wbank] <= 1'b0;
                        end else begin
                            // 队列满 / 有欠账：这条不给槽，bank 立刻释放（判决欠账已记）
                            err <= 1'b1;
                            bank_used[wbank] <= 1'b0;
                        end
                        cst   <= C_IDLE;
                        end
                    end else if (r_valid) begin
                        if (ovf_now) begin

                            err <= 1'b1;
                            bank_used[wbank] <= 1'b0;
                            cst <= C_DRAIN;
                        end else if (ewb == 2'd3) begin
                            ebuf[{wbank, ewp}] <= {r_data, ewacc[23:0]};
                            ewp   <= ewp + 1'b1;
                            ewb   <= 2'd0;
                            ewacc <= 32'h0;
                            elen  <= elen + 1'b1;
                        end else begin
                            ewacc[{ewb, 3'b000} +: 8] <= r_data;
                            ewb   <= ewb + 2'd1;
                            elen  <= elen + 1'b1;
                        end
                    end
                end
                C_DRAIN: begin

                    if (fl_eop) begin
                        if (v_accept) begin
                            vkind[vtail] <= 2'd2;
                            vvli [vtail] <= 16'h0;

                            vbank[vtail] <= 2'd0;
                            vlen [vtail] <= 14'd0;
                            vbyte[vtail] <= 8'h00;
                            vport[vtail] <= 4'd0;
                            vtail <= vtail + 1'b1;
                        end else begin
                            err <= 1'b1;             // 条数由 v_drop_add 记
                        end
                        cst <= C_IDLE;
                    end
                end
                default: cst <= C_IDLE;
            endcase


            if (cst_abandon && (cst != C_IDLE) && !(r_valid && (r_eop || r_sop))) begin
                if (cst == C_WR) bank_used[wbank] <= 1'b0;
                cst <= C_IDLE;
                err <= 1'b1;
            end

            // ---- 队列计数（一次算净变化）----
            vcnt <= vcnt + {1'b0, v_push_eff} - {1'b0, (v_do_pop | v_timeout)};

            v_drop <= v_drop_tmo ? v_drop_add
                      : (({1'b0, v_drop} + {1'b0, v_drop_add} - {1'b0, v_drop_sub} > 9'd255)
                         ? 8'hFF
                         : (v_drop + v_drop_add - v_drop_sub));
            dcnt <= dcnt + {3'b000, d_push} - {3'b000, d_pop};

            if (v_recv) begin
                if (v_do_aband) v_abandon <= v_abandon - 1'b1;
                v_tmo <= 16'd0;
            end else if (v_drop_tmo) begin
                v_tmo <= 16'd0;
            end else if ((vcnt != 0) || (v_drop != 0)) begin
                v_tmo <= v_tmo + 16'd1;
            end else begin
                v_tmo <= 16'd0;
            end

            // ================= 出向② verdict =================
            if (v_do_pop) begin

                if (r_commit && (vkind[vhead] != 2'd2)) begin
                    if (d_has_room) begin
                        dkind[dtail] <= vkind[vhead];
                        dbank[dtail] <= vbank[vhead];
                        dlen [dtail] <= vlen [vhead];
                        dbyte[dtail] <= vbyte[vhead];
                        dport[dtail] <= vport[vhead];
                        dvli [dtail] <= vvli [vhead];
                        dtail <= dtail + 1'b1;
                    end else begin

                        err <= 1'b1;
                        if (vkind[vhead] == 2'd0) bank_used[vbank[vhead]] <= 1'b0;
                    end
                end else if (vkind[vhead] == 2'd0) begin
                    bank_used[vbank[vhead]] <= 1'b0;
                end
                vhead <= vhead + 1'b1;
            end else if (v_timeout) begin
                if (vkind[vhead] == 2'd0) bank_used[vbank[vhead]] <= 1'b0;
                vhead <= vhead + 1'b1;
                if (v_abandon != 6'h3F) v_abandon <= v_abandon + 1'b1;   // 6 位饱和
                err <= 1'b1;
            end
            // 队里已经空了，但还有"因队满未入队"的帧欠着判决 ⇒ 按序把这条判决吃掉
            else if (v_drop_sub) begin
                err <= 1'b1;                 // 这条消息确实被丢了，让软件可见
            end

            // ================= 出向③ presenter =================
            case (pst)
                P_IDLE: begin
                    if (dcnt != 0) begin
                        pkind <= (dkind[dhead] == 2'd1);
                        app_tx_port <= dport[dhead];
                        app_tx_vlid <= dvli[dhead];
                        plen  <= dlen[dhead];
                        ce_i  <= 0;
                        erb   <= 0;
                        dhead <= dhead + 1'b1;
                        if (dkind[dhead] == 2'd1) begin
                            ewcur <= {24'h0, dbyte[dhead]};
                            pst   <= P_PRES;
                        end else begin
                            pbank <= dbank[dhead];
                            erp   <= 0;
                            era   <= {dbank[dhead], 11'd0};
                            pst   <= P_RD;
                        end
                    end
                end
                P_RD: begin era <= {pbank, 11'd1}; pst <= P_LD; end
                P_LD: begin ewcur <= erw; era <= {pbank, 11'd2}; pst <= P_LD2; end
                P_LD2: begin ewnxt <= erw; erp <= 11'd2; pst <= P_PRES; end
                P_PRES: begin
                    if (ce_i == plen - 1'b1) begin
                        if (!pkind) bank_used[pbank] <= 1'b0;
                        ce_i <= 0;
                        pst  <= P_IDLE;
                    end else begin
                        ce_i <= ce_i + 1'b1;
                        if (erb == 2'd3) begin
                            erb   <= 2'd0;
                            ewcur <= ewnxt;
                            ewnxt <= erw;
                            era   <= {pbank, erp + 1'b1};
                            erp   <= erp + 1'b1;
                        end else erb <= erb + 2'd1;
                    end
                end
                default: pst <= P_IDLE;
            endcase
        end
    end
endmodule
