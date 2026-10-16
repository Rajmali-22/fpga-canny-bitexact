`timescale 1ns / 1ps
//===========================================================================
// window3x3.v  --  raster stream -> sliding 3x3 window.
//
// Feed pixels in raster order (row 0 left-to-right, then row 1, ...). Two
// W-deep delay lines hold the two previous rows, and three 3-deep tap
// registers form the window.
//
//      p00 p01 p02        NW  N  NE
//      p10 p11 p12   ==    W  C  E      p11 is the window CENTRE
//      p20 p21 p22        SW  S  SE
//
// The centre lags the input by W+1 pixels, so to flush all W*H windows out the
// caller must feed W*H + W + 1 pixels (pad the tail with zeros).
//
// win_border marks centres on the 1-pixel frame. Those are exactly the cases
// where the window straddles a row boundary and holds wrapped-around pixels,
// so the caller MUST force the output to 0 there -- which is also what the
// Canny reference model does.
//
// NOTE: W and H must be powers of two, and $clog2(W)+$clog2(H) must equal AW,
// because win_addr is formed by concatenating the row and column counters.
//===========================================================================
module window3x3 #(
    parameter DW = 8,
    parameter W  = 128,
    parameter H  = 128,
    parameter AW = 14
)(
    input  wire           clk,
    input  wire           rst,          // synchronous, active high: flush between passes
    input  wire           in_vld,
    input  wire [DW-1:0]  in_pix,

    output reg            win_vld,
    output wire [AW-1:0]  win_addr,     // linear address of the CENTRE pixel
    output wire           win_border,   // centre is on the 1-pixel frame
    output reg  [DW-1:0]  p00, p01, p02,
    output reg  [DW-1:0]  p10, p11, p12,
    output reg  [DW-1:0]  p20, p21, p22
);

    localparam CW = $clog2(W);
    localparam RW = $clog2(H);

    //-----------------------------------------------------------------------
    // Two W-deep delay lines.
    //   lb1[col] read gives the pixel from W samples ago  -> row y-1
    //   lb0[col] read gives the pixel from 2W samples ago -> row y-2
    // Small enough for LUT RAM; keeping them out of BRAM leaves all 50 block
    // RAMs for the frame buffers.
    //-----------------------------------------------------------------------
    (* ram_style = "distributed" *) reg [DW-1:0] lb0 [0:W-1];
    (* ram_style = "distributed" *) reg [DW-1:0] lb1 [0:W-1];

    reg [CW-1:0] col;
    reg [DW-1:0] tap0, tap1, tap2;
    reg          tap_vld;

    always @(posedge clk) begin
        if (rst) begin
            col     <= {CW{1'b0}};
            tap_vld <= 1'b0;
        end else begin
            tap_vld <= in_vld;
            if (in_vld) begin
                tap2     <= in_pix;              // row y
                tap1     <= lb1[col];            // row y-1  (non-blocking: old value)
                tap0     <= lb0[col];            // row y-2
                lb0[col] <= lb1[col];            // shift the rows down one line
                lb1[col] <= in_pix;
                col      <= (col == W-1) ? {CW{1'b0}} : col + 1'b1;
            end
        end
    end

    //-----------------------------------------------------------------------
    // 3x3 tap registers. Newest sample enters on the right (East) side.
    //-----------------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            p00 <= {DW{1'b0}}; p01 <= {DW{1'b0}}; p02 <= {DW{1'b0}};
            p10 <= {DW{1'b0}}; p11 <= {DW{1'b0}}; p12 <= {DW{1'b0}};
            p20 <= {DW{1'b0}}; p21 <= {DW{1'b0}}; p22 <= {DW{1'b0}};
        end else if (tap_vld) begin
            p00 <= p01; p01 <= p02; p02 <= tap0;
            p10 <= p11; p11 <= p12; p12 <= tap1;
            p20 <= p21; p21 <= p22; p22 <= tap2;
        end
    end

    //-----------------------------------------------------------------------
    // Validity and centre coordinates.
    // sh_cnt counts completed tap shifts. Shift s produces the window whose
    // centre index is s-2-W, so the first valid window appears at s = W+2,
    // i.e. when the pre-increment count sh_cnt reaches W+1.
    //-----------------------------------------------------------------------
    reg [AW:0]   sh_cnt;
    reg [CW-1:0] cx;
    reg [RW-1:0] cy;

    always @(posedge clk) begin
        if (rst) begin
            sh_cnt  <= {(AW+1){1'b0}};
            win_vld <= 1'b0;
            cx      <= {CW{1'b0}};
            cy      <= {RW{1'b0}};
        end else begin
            if (tap_vld) sh_cnt <= sh_cnt + 1'b1;

            win_vld <= tap_vld && (sh_cnt >= (W + 1));

            // cx/cy hold the coords of the window presented THIS cycle, and
            // advance to the next one on the same edge.
            if (win_vld) begin
                if (cx == W-1) begin
                    cx <= {CW{1'b0}};
                    cy <= cy + 1'b1;
                end else begin
                    cx <= cx + 1'b1;
                end
            end
        end
    end

    assign win_addr   = {cy, cx};
    assign win_border = (cx == 0) || (cx == W-1) || (cy == 0) || (cy == H-1);

endmodule
