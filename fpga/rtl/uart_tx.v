`timescale 1ns / 1ps
//===========================================================================
// uart_tx.v  --  8N1 UART transmitter.
//
// Assert din_vld for one clock while busy is low to queue a byte. The byte is
// latched immediately and the start bit goes out on the same edge.
//===========================================================================
module uart_tx #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       din_vld,
    input  wire [7:0] din,
    output reg        tx,
    output wire       busy
);

    localparam integer DIV = CLK_HZ / BAUD;

    reg [15:0] cnt;
    reg [3:0]  bitno;
    reg [9:0]  sh;        // {stop, data[7:0], start}
    reg        active;

    assign busy = active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx     <= 1'b1;
            active <= 1'b0;
            cnt    <= 16'd0;
            bitno  <= 4'd0;
            sh     <= 10'h3FF;
        end else if (!active) begin
            tx <= 1'b1;                          // idle high
            if (din_vld) begin
                sh     <= {1'b1, din, 1'b0};
                active <= 1'b1;
                cnt    <= DIV[15:0] - 16'd1;
                bitno  <= 4'd0;
                tx     <= 1'b0;                  // start bit, right now
            end
        end else begin
            if (cnt == 16'd0) begin
                cnt <= DIV[15:0] - 16'd1;
                if (bitno == 4'd9) begin         // stop bit has been held a full period
                    active <= 1'b0;
                    tx     <= 1'b1;
                end else begin
                    bitno <= bitno + 4'd1;
                    tx    <= sh[bitno + 4'd1];   // bit 0 -> data[0] ... bit 8 -> stop
                end
            end else begin
                cnt <= cnt - 16'd1;
            end
        end
    end

endmodule
