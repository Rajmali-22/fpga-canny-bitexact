
`timescale 1ns / 1ps
//===========================================================================
// dpram.v  --  simple dual-port RAM: 1 write port, 1 read port, sync read.
//
// Infers Block RAM in Vivado. Read-during-write at the same address returns
// the OLD data (READ_FIRST); no Canny pass ever depends on that case, because
// every pass reads one buffer and writes a different one.
//
// Read latency is 1 clock: present raddr on cycle T, dout is valid on T+1.
//===========================================================================
module dpram #(
    parameter DW = 8,               // data width
    parameter AW = 14               // address width -> depth = 2**AW
)(
    input  wire           clk,
    input  wire           we,
    input  wire [AW-1:0]  waddr,
    input  wire [DW-1:0]  din,
    input  wire [AW-1:0]  raddr,
    output reg  [DW-1:0]  dout
);

    (* ram_style = "block" *) reg [DW-1:0] mem [0:(1<<AW)-1];

    always @(posedge clk) begin
        if (we) mem[waddr] <= din;
        dout <= mem[raddr];
    end

endmodule
