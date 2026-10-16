`timescale 1ns / 1ps
//===========================================================================
// op_hyst.v  --  Canny pass 5: hysteresis edge tracking (one iteration).
//
// A weak pixel (128) that touches any strong pixel (255) in its 8-neighbourhood
// is promoted to strong. Everything else passes through unchanged.
//
// This is the stage that does not fit a single streaming pass: promotion
// propagates along an edge chain one pixel per iteration, so canny_core runs
// this operator repeatedly over the whole frame, ping-ponging between two
// buffers, until an iteration promotes nothing. `promote` is the per-pixel
// evidence the core ORs together into its "changed" flag.
//
// Measured convergence: 82 iterations worst case over the 24 tiles of the
// 480x720 test image. The core's cap is 256.
//
// Surviving weak pixels are mapped to 0 at read-out time, not here, so the
// intermediate classification stays inspectable over UART (CMD 0x06).
//
// Fixed 2-clock latency.
//===========================================================================
module op_hyst (
    input  wire       clk,
    input  wire [7:0] p00, p01, p02,
    input  wire [7:0] p10, p11, p12,
    input  wire [7:0] p20, p21, p22,
    output reg  [7:0] dout,
    output reg        promote       // this pixel was just promoted weak -> strong
);

    wire near_strong = (p00 == 8'd255) || (p01 == 8'd255) || (p02 == 8'd255) ||
                       (p10 == 8'd255) ||                    (p12 == 8'd255) ||
                       (p20 == 8'd255) || (p21 == 8'd255) || (p22 == 8'd255);

    wire promote_c = (p11 == 8'd128) && near_strong;

    reg       prom;
    reg [7:0] centre;

    always @(posedge clk) begin
        prom    <= promote_c;
        centre  <= p11;
        dout    <= prom ? 8'd255 : centre;
        promote <= prom;
    end

endmodule
