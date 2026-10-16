`timescale 1ns / 1ps
//===========================================================================
// op_thresh.v  --  Canny pass 4: double threshold (classification).
//
//   >= t_high  -> 255  STRONG, definitely an edge
//   >= t_low   -> 128  WEAK, an edge only if it connects to a strong one
//   otherwise  ->   0  discarded
//
// Pointwise: only the window centre is used. It still runs through the shared
// window path so it inherits the same border zeroing and the same 2-clock
// latency as every other pass, which keeps canny_core's control logic uniform.
//===========================================================================
module op_thresh (
    input  wire       clk,
    input  wire [7:0] t_low,
    input  wire [7:0] t_high,
    input  wire [7:0] p11,           // window centre
    output reg  [7:0] dout
);

    reg [7:0] centre;

    always @(posedge clk) begin
        centre <= p11;
        dout   <= (centre >= t_high) ? 8'd255 :
                  (centre >= t_low)  ? 8'd128 : 8'd0;
    end

endmodule
