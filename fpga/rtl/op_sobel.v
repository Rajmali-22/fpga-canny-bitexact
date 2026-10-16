`timescale 1ns / 1ps
//===========================================================================
// op_sobel.v  --  Canny pass 2: gradient magnitude and quantised direction.
//
//   Gx = [-1 0 1; -2 0 2; -1 0 1]     Gy = [-1 -2 -1; 0 0 0; 1 2 1]
//   mag = min(255, (|Gx| + |Gy|) >> 2)
//
// No square root: |Gx|+|Gy| is the standard FPGA approximation (under 12%
// error). Shift is 2, not 3: |Gx|+|Gy| spans 0..2040 so >>3 would never clip,
// but measurement on real images showed magnitudes only reaching ~118/255 --
// half the range wasted and edges crammed into coarse low codes. >>2 spans
// 0..510 and saturates on a handful of pixels that are far above T_HIGH
// anyway, and it halves quantisation ties in the NMS stage.
//
// Direction needs no atan and no divide. With ax=|Gx|, ay=|Gy| and the Q8
// constants tan(22.5) = 106/256 and tan(67.5) = 618/256:
//
//   ay*256 <= ax*106  -> 0 deg    gradient horizontal, compare W  / E
//   ay*256 >= ax*618  -> 90 deg   gradient vertical,   compare N  / S
//   sign(Gx)==sign(Gy)-> 135 deg  compare NW / SE
//   otherwise         -> 45 deg   compare NE / SW
//
// Flat regions give ax=ay=0, which satisfies the first test and yields dir 0.
// Harmless: magnitude there is 0, so NMS emits 0 regardless.
//
// Fixed 2-clock latency: stage 1 forms Gx/Gy, stage 2 does the magnitude and
// the direction compares. Splitting it this way keeps the combinational path
// short enough to close timing at 50 MHz.
//===========================================================================
module op_sobel (
    input  wire       clk,
    input  wire [7:0] p00, p01, p02,
    input  wire [7:0] p10, p11, p12,     // p11 unused: both kernels have 0 there
    input  wire [7:0] p20, p21, p22,
    output reg  [7:0] mag,
    output reg  [1:0] dir
);

    wire signed [11:0] s00 = {4'd0, p00};
    wire signed [11:0] s01 = {4'd0, p01};
    wire signed [11:0] s02 = {4'd0, p02};
    wire signed [11:0] s10 = {4'd0, p10};
    wire signed [11:0] s12 = {4'd0, p12};
    wire signed [11:0] s20 = {4'd0, p20};
    wire signed [11:0] s21 = {4'd0, p21};
    wire signed [11:0] s22 = {4'd0, p22};

    // range +/-1020, fits 12-bit signed
    wire signed [11:0] gx_c = (s02 - s00) + ((s12 - s10) <<< 1) + (s22 - s20);
    wire signed [11:0] gy_c = (s20 - s00) + ((s21 - s01) <<< 1) + (s22 - s02);

    reg signed [11:0] gx, gy;

    always @(posedge clk) begin
        gx <= gx_c;
        gy <= gy_c;
    end

    //---------------- stage 2 -------------------------------------------
    wire [11:0] ax = gx[11] ? (12'd0 - gx) : gx;      // 0..1020
    wire [11:0] ay = gy[11] ? (12'd0 - gy) : gy;

    wire [12:0] sum   = {1'b0, ax} + {1'b0, ay};      // 0..2040
    wire [12:0] scale = sum >> 2;                     // 0..510
    wire [7:0]  mag_c = (scale > 13'd255) ? 8'd255 : scale[7:0];

    wire [20:0] ay_256 = {ay, 8'd0};                  // ay * 256
    wire [20:0] ax_106 = ax * 21'd106;                // constant mult -> adder tree
    wire [20:0] ax_618 = ax * 21'd618;

    wire same_sign = (gx[11] == gy[11]);

    wire [1:0] dir_c = (ay_256 <= ax_106) ? 2'd0 :
                       (ay_256 >= ax_618) ? 2'd2 :
                       (same_sign        ? 2'd3 : 2'd1);

    always @(posedge clk) begin
        mag <= mag_c;
        dir <= dir_c;
    end

endmodule
