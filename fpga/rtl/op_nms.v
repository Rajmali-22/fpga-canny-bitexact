`timescale 1ns / 1ps
//===========================================================================
// op_nms.v  --  Canny pass 3: non-maximum suppression.
//
// Thins wide gradient ridges to single-pixel edges. The centre survives only
// if it is >= both of its neighbours ALONG the gradient direction; otherwise
// it is suppressed to 0.
//
//   dir 0 (0 deg)   -> W  (p10) / E  (p12)
//   dir 1 (45 deg)  -> NE (p02) / SW (p20)
//   dir 2 (90 deg)  -> N  (p01) / S  (p21)
//   dir 3 (135 deg) -> NW (p00) / SE (p22)
//
// `dir` comes from the direction buffer written by op_sobel and must be
// presented in the same cycle as the window. canny_core arranges that by
// issuing the direction-RAM read one pipeline stage early.
//
// Fixed 2-clock latency.
//===========================================================================
module op_nms (
    input  wire       clk,
    input  wire [1:0] dir,
    input  wire [7:0] p00, p01, p02,
    input  wire [7:0] p10, p11, p12,
    input  wire [7:0] p20, p21, p22,
    output reg  [7:0] dout
);

    wire keep_c = (dir == 2'd0) ? ((p11 >= p10) && (p11 >= p12)) :
                  (dir == 2'd1) ? ((p11 >= p02) && (p11 >= p20)) :
                  (dir == 2'd2) ? ((p11 >= p01) && (p11 >= p21)) :
                                  ((p11 >= p00) && (p11 >= p22));

    reg       keep;
    reg [7:0] centre;

    always @(posedge clk) begin
        keep   <= keep_c;
        centre <= p11;
        dout   <= keep ? centre : 8'd0;
    end

endmodule
