`timescale 1ns / 1ps
//===========================================================================
// canny_core.v  --  multi-pass Canny edge detector over BRAM frame buffers.
//
// Canny does not fit a single streaming pipeline because hysteresis propagates
// along edge chains without bound. So the whole frame lives in Block RAM and
// each stage is a separate raster pass:
//
//   P1 GAUSS   m_in   -> m_blur
//   P2 SOBEL   m_blur -> m_mag + m_dir
//   P3 NMS     m_mag  -> m_nms          (m_dir read alongside)
//   P4 THRESH  m_nms  -> m_cls
//   P5 HYST    m_cls  -> m_a -> m_b -> m_a ...  until nothing changes
//
// Every pass walks the same address generator, the same window3x3, and the
// same 2-clock operator latency, so the control logic is written once.
//
// PIPELINE (a pass is one long shift register; get this wrong and the image
// comes out shifted by a few pixels, which is the classic silent failure):
//
//   fetch presents raddr
//    +1   BRAM dout valid          -> window3x3 in_vld
//    +2   window3x3 tap stage
//    +3   window3x3 window regs    -> win_vld / win_addr / win_border
//    +4   alignment stage (q*)     -> m_dir dout lands here too
//    +5   operator stage 1
//    +6   operator stage 2         -> write, address = wa2
//
// The window centre lags its input by W+1 pixels, so each pass feeds
// N + W + 1 samples (N real, then W+1 zeros) to flush all N windows out.
//
// RESULT LOCATION: whichever of m_a / m_b the final hysteresis iteration wrote
// (tracked by final_is_a). Surviving weak pixels are mapped to 0 during
// read-out rather than in an extra pass, which keeps the pre-hysteresis
// classification readable over UART for debugging.
//===========================================================================
module canny_core #(
    parameter W        = 128,
    parameter H        = 128,
    parameter AW       = 14,
    parameter MAX_HYST = 256          // hard cap; measured worst case is 82
)(
    input  wire           clk,
    input  wire           rst_n,

    input  wire           start,      // pulse ONE clock to begin
    output reg            done,       // high from completion until the next start

    input  wire [7:0]     t_low,
    input  wire [7:0]     t_high,

    // host write port into the input frame buffer
    input  wire           in_we,
    input  wire [AW-1:0]  in_addr,
    input  wire [7:0]     in_data,

    // host read-out port (valid one clock after rd_addr; only while idle)
    input  wire [2:0]     rd_sel,
    input  wire [AW-1:0]  rd_addr,
    output wire [7:0]     rd_data,

    // status
    output reg  [8:0]     hyst_iters,
    output reg            hyst_capped
);

    localparam N = W * H;

    localparam P_GAUSS  = 3'd0,
               P_SOBEL  = 3'd1,
               P_NMS    = 3'd2,
               P_THRESH = 3'd3,
               P_HYST   = 3'd4;

    localparam S_IDLE = 2'd0,
               S_INIT = 2'd1,
               S_RUN  = 2'd2,
               S_NEXT = 2'd3;

    reg [1:0]  state;
    reg [2:0]  pass;
    reg [15:0] fetch;
    reg [15:0] wr_cnt;
    reg [8:0]  hyst_n;
    reg        changed;
    reg        dst_is_a;
    reg        final_is_a;
    reg        win_rst;
    reg        feeding;

    //=====================================================================
    // Memories
    //=====================================================================
    wire [7:0] d_in, d_blur, d_mag, d_nms, d_cls, d_a, d_b;
    wire [1:0] d_dir;

    wire proc = (state != S_IDLE);
    wire [AW-1:0] mem_raddr = proc ? fetch[AW-1:0] : rd_addr;

    //=====================================================================
    // Address generation and the BRAM read delay
    //=====================================================================
    reg [15:0] fetch_d;
    reg        feed_d;

    always @(posedge clk) begin
        fetch_d <= fetch;
        feed_d  <= (state == S_RUN) && feeding;
    end

    wire       hyst_first = (hyst_n == 9'd0);

    wire [7:0] src_dout = (pass == P_GAUSS)  ? d_in   :
                          (pass == P_SOBEL)  ? d_blur :
                          (pass == P_NMS)    ? d_mag  :
                          (pass == P_THRESH) ? d_nms  :
                          (hyst_first ? d_cls : (dst_is_a ? d_b : d_a));

    wire       win_in_vld = feed_d;
    wire [7:0] win_in_pix = (fetch_d < N) ? src_dout : 8'd0;

    //=====================================================================
    // Sliding window
    //=====================================================================
    wire          win_vld;
    wire [AW-1:0] win_addr;
    wire          win_border;
    wire [7:0]    p00, p01, p02, p10, p11, p12, p20, p21, p22;

    window3x3 #(.DW(8), .W(W), .H(H), .AW(AW)) u_win (
        .clk(clk), .rst(win_rst),
        .in_vld(win_in_vld), .in_pix(win_in_pix),
        .win_vld(win_vld), .win_addr(win_addr), .win_border(win_border),
        .p00(p00), .p01(p01), .p02(p02),
        .p10(p10), .p11(p11), .p12(p12),
        .p20(p20), .p21(p21), .p22(p22)
    );

    //---------------------------------------------------------------------
    // Alignment stage. m_dir's read is issued with win_addr, so its dout
    // arrives exactly here -- in the same cycle as the registered window.
    //---------------------------------------------------------------------
    reg           d_vld;
    reg [AW-1:0]  d_addr;
    reg           d_bord;
    reg [7:0]     q00, q01, q02, q10, q11, q12, q20, q21, q22;

    always @(posedge clk) begin
        d_vld <= win_vld;
        if (win_vld) begin
            d_addr <= win_addr;
            d_bord <= win_border;
            q00 <= p00; q01 <= p01; q02 <= p02;
            q10 <= p10; q11 <= p11; q12 <= p12;
            q20 <= p20; q21 <= p21; q22 <= p22;
        end
    end

    // carry write address and border flag alongside the 2-clock operators
    reg [AW-1:0] wa1, wa2;
    reg          wb1, wb2;
    reg          v1, v2;

    always @(posedge clk) begin
        v1  <= d_vld;   v2  <= v1;
        wa1 <= d_addr;  wa2 <= wa1;
        wb1 <= d_bord;  wb2 <= wb1;
    end

    wire op_vld = v2 && (state == S_RUN);

    //=====================================================================
    // Operators -- all instantiated in parallel, all 2-clock latency.
    // Only one pass is ever selected, so the others just compute discarded
    // values. Costs well under 1000 LUTs total and keeps the muxing trivial.
    //=====================================================================
    wire [7:0] g_dout;
    op_gauss u_gauss (.clk(clk),
        .p00(q00), .p01(q01), .p02(q02),
        .p10(q10), .p11(q11), .p12(q12),
        .p20(q20), .p21(q21), .p22(q22), .dout(g_dout));

    wire [7:0] s_mag;
    wire [1:0] s_dir;
    op_sobel u_sobel (.clk(clk),
        .p00(q00), .p01(q01), .p02(q02),
        .p10(q10), .p11(q11), .p12(q12),
        .p20(q20), .p21(q21), .p22(q22), .mag(s_mag), .dir(s_dir));

    wire [7:0] n_dout;
    op_nms u_nms (.clk(clk), .dir(d_dir),
        .p00(q00), .p01(q01), .p02(q02),
        .p10(q10), .p11(q11), .p12(q12),
        .p20(q20), .p21(q21), .p22(q22), .dout(n_dout));

    wire [7:0] t_dout;
    op_thresh u_thr (.clk(clk), .t_low(t_low), .t_high(t_high),
        .p11(q11), .dout(t_dout));

    wire [7:0] h_dout;
    wire       h_promote;
    op_hyst u_hyst (.clk(clk),
        .p00(q00), .p01(q01), .p02(q02),
        .p10(q10), .p11(q11), .p12(q12),
        .p20(q20), .p21(q21), .p22(q22),
        .dout(h_dout), .promote(h_promote));

    //=====================================================================
    // Write back. The 1-pixel border is forced to 0 on every pass, matching
    // the reference model exactly.
    //=====================================================================
    wire [7:0] op_data = (pass == P_GAUSS)  ? g_dout :
                         (pass == P_SOBEL)  ? s_mag  :
                         (pass == P_NMS)    ? n_dout :
                         (pass == P_THRESH) ? t_dout : h_dout;

    wire [7:0] wdata = wb2 ? 8'd0 : op_data;
    wire [1:0] wdir  = wb2 ? 2'd0 : s_dir;

    wire we_blur = op_vld && (pass == P_GAUSS);
    wire we_mag  = op_vld && (pass == P_SOBEL);
    wire we_dir  = op_vld && (pass == P_SOBEL);
    wire we_nms  = op_vld && (pass == P_NMS);
    wire we_cls  = op_vld && (pass == P_THRESH);
    wire we_a    = op_vld && (pass == P_HYST) &&  dst_is_a;
    wire we_b    = op_vld && (pass == P_HYST) && !dst_is_a;

    dpram #(.DW(8), .AW(AW)) m_in   (.clk(clk), .we(in_we),  .waddr(in_addr), .din(in_data),
                                     .raddr(mem_raddr), .dout(d_in));
    dpram #(.DW(8), .AW(AW)) m_blur (.clk(clk), .we(we_blur), .waddr(wa2), .din(wdata),
                                     .raddr(mem_raddr), .dout(d_blur));
    dpram #(.DW(8), .AW(AW)) m_mag  (.clk(clk), .we(we_mag),  .waddr(wa2), .din(wdata),
                                     .raddr(mem_raddr), .dout(d_mag));
    dpram #(.DW(2), .AW(AW)) m_dir  (.clk(clk), .we(we_dir),  .waddr(wa2), .din(wdir),
                                     .raddr(win_addr),  .dout(d_dir));
    dpram #(.DW(8), .AW(AW)) m_nms  (.clk(clk), .we(we_nms),  .waddr(wa2), .din(wdata),
                                     .raddr(mem_raddr), .dout(d_nms));
    dpram #(.DW(8), .AW(AW)) m_cls  (.clk(clk), .we(we_cls),  .waddr(wa2), .din(wdata),
                                     .raddr(mem_raddr), .dout(d_cls));
    dpram #(.DW(8), .AW(AW)) m_a    (.clk(clk), .we(we_a),    .waddr(wa2), .din(wdata),
                                     .raddr(mem_raddr), .dout(d_a));
    dpram #(.DW(8), .AW(AW)) m_b    (.clk(clk), .we(we_b),    .waddr(wa2), .din(wdata),
                                     .raddr(mem_raddr), .dout(d_b));

    //=====================================================================
    // Read-out mux (see the CMD table in docs/DESIGN_NOTES.md)
    //=====================================================================
    wire [7:0] final_raw = final_is_a ? d_a : d_b;
    wire [7:0] final_px  = (final_raw == 8'd255) ? 8'd255 : 8'd0;

    assign rd_data = (rd_sel == 3'd1) ? final_px :
                     (rd_sel == 3'd2) ? d_in     :
                     (rd_sel == 3'd3) ? d_blur   :
                     (rd_sel == 3'd4) ? d_mag    :
                     (rd_sel == 3'd5) ? d_nms    :
                     (rd_sel == 3'd6) ? d_cls    : 8'd0;

    //=====================================================================
    // Pass sequencer
    //=====================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state       <= S_IDLE;
            pass        <= P_GAUSS;
            fetch       <= 16'd0;
            wr_cnt      <= 16'd0;
            hyst_n      <= 9'd0;
            changed     <= 1'b0;
            dst_is_a    <= 1'b1;
            final_is_a  <= 1'b1;
            win_rst     <= 1'b1;
            feeding     <= 1'b0;
            done        <= 1'b0;
            hyst_iters  <= 9'd0;
            hyst_capped <= 1'b0;
        end else begin
            case (state)

            S_IDLE: begin
                win_rst <= 1'b1;
                feeding <= 1'b0;
                if (start) begin
                    done        <= 1'b0;
                    pass        <= P_GAUSS;
                    hyst_n      <= 9'd0;
                    hyst_iters  <= 9'd0;
                    hyst_capped <= 1'b0;
                    dst_is_a    <= 1'b1;
                    state       <= S_INIT;
                end
            end

            S_INIT: begin
                win_rst <= 1'b0;
                fetch   <= 16'd0;
                wr_cnt  <= 16'd0;
                changed <= 1'b0;
                feeding <= 1'b1;
                state   <= S_RUN;
            end

            S_RUN: begin
                if (feeding) begin
                    if (fetch == (N + W)) feeding <= 1'b0;
                    fetch <= fetch + 16'd1;
                end
                if (op_vld) begin
                    wr_cnt <= wr_cnt + 16'd1;
                    if ((pass == P_HYST) && h_promote && !wb2) changed <= 1'b1;
                    if (wr_cnt == (N - 1)) state <= S_NEXT;
                end
            end

            S_NEXT: begin
                win_rst <= 1'b1;
                case (pass)
                P_GAUSS:  begin pass <= P_SOBEL;  state <= S_INIT; end
                P_SOBEL:  begin pass <= P_NMS;    state <= S_INIT; end
                P_NMS:    begin pass <= P_THRESH; state <= S_INIT; end
                P_THRESH: begin
                    pass     <= P_HYST;
                    hyst_n   <= 9'd0;
                    dst_is_a <= 1'b1;              // iteration 0: m_cls -> m_a
                    state    <= S_INIT;
                end
                P_HYST: begin
                    hyst_iters <= hyst_n + 9'd1;
                    final_is_a <= dst_is_a;
                    if (!changed) begin
                        done  <= 1'b1;             // converged
                        state <= S_IDLE;
                    end else if (hyst_n == (MAX_HYST - 1)) begin
                        hyst_capped <= 1'b1;       // gave up; led[3] will show it
                        done        <= 1'b1;
                        state       <= S_IDLE;
                    end else begin
                        hyst_n   <= hyst_n + 9'd1;
                        dst_is_a <= ~dst_is_a;     // ping-pong m_a <-> m_b
                        state    <= S_INIT;
                    end
                end
                default: state <= S_IDLE;
                endcase
            end

            endcase
        end
    end

endmodule
