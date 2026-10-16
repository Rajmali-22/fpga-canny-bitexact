`timescale 1ns / 1ps
//===========================================================================
// op_gauss.v  --  Canny pass 1: 3x3 Gaussian smoothing.
//
//        1 2 1
//   1/16 2 4 2      all weights are powers of two -> shifts and adds only
//        1 2 1
//
// Fixed 2-clock latency. Pure datapath, no valid port -- canny_core tracks
// validity for every operator with one shared 2-deep delay.
//===========================================================================
module op_gauss (
    input  wire       clk,
    input  wire [7:0] p00, p01, p02,
    input  wire [7:0] p10, p11, p12,
    input  wire [7:0] p20, p21, p22,
    output reg  [7:0] dout
);

    // Zero-extend to the accumulator width BEFORE shifting. Shifting an 8-bit
    // operand would truncate back to 8 bits and silently drop the MSB.
    wire [11:0] e00 = {4'd0, p00};
    wire [11:0] e01 = {4'd0, p01};
    wire [11:0] e02 = {4'd0, p02};
    wire [11:0] e10 = {4'd0, p10};
    wire [11:0] e11 = {4'd0, p11};
    wire [11:0] e12 = {4'd0, p12};
    wire [11:0] e20 = {4'd0, p20};
    wire [11:0] e21 = {4'd0, p21};
    wire [11:0] e22 = {4'd0, p22};

    // max = 16 * 255 = 4080, fits 12 bits
    wire [11:0] acc_c = e00 + (e01 << 1) + e02
                      + (e10 << 1) + (e11 << 2) + (e12 << 1)
                      + e20 + (e21 << 1) + e22;

    reg [11:0] acc;

    always @(posedge clk) begin
        acc  <= acc_c;
        dout <= acc[11:4];          // >> 4, truncating -- matches the model
    end

endmodule
