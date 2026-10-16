`timescale 1ns / 1ps
//===========================================================================
// uart_rx.v  --  8N1 UART receiver.
//
// Samples in the middle of each bit. The start bit is re-checked at its
// midpoint so a short glitch on the line cannot fake a byte.
//
// DIV = CLK_HZ / BAUD must be >= 8 for the half-bit sampling to be sane.
// At 50 MHz: 115200 -> 434,  921600 -> 54.
//===========================================================================
module uart_rx #(
    parameter CLK_HZ = 50_000_000,
    parameter BAUD   = 115_200
)(
    input  wire       clk,
    input  wire       rst_n,
    input  wire       rx,           // asynchronous serial input
    output reg        dout_vld,     // 1-clock strobe when dout is fresh
    output reg  [7:0] dout
);

    localparam integer DIV  = CLK_HZ / BAUD;
    localparam integer HALF = DIV / 2;

    // Two-flop synchroniser: rx crosses into our clock domain.
    reg rx_m, rx_s;
    always @(posedge clk) begin
        rx_m <= rx;
        rx_s <= rx_m;
    end

    localparam S_IDLE  = 2'd0,
               S_START = 2'd1,
               S_DATA  = 2'd2,
               S_STOP  = 2'd3;

    reg [1:0]  state;
    reg [15:0] cnt;
    reg [2:0]  bitno;
    reg [7:0]  sh;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state    <= S_IDLE;
            cnt      <= 16'd0;
            bitno    <= 3'd0;
            sh       <= 8'd0;
            dout     <= 8'd0;
            dout_vld <= 1'b0;
        end else begin
            dout_vld <= 1'b0;

            case (state)

            S_IDLE: begin
                if (!rx_s) begin                 // falling edge = possible start
                    cnt   <= HALF[15:0];
                    state <= S_START;
                end
            end

            S_START: begin                       // land in the middle of the start bit
                if (cnt == 16'd0) begin
                    if (!rx_s) begin             // still low -> genuine start
                        cnt   <= DIV[15:0] - 16'd1;
                        bitno <= 3'd0;
                        state <= S_DATA;
                    end else begin
                        state <= S_IDLE;         // glitch, abandon
                    end
                end else begin
                    cnt <= cnt - 16'd1;
                end
            end

            S_DATA: begin                        // sample mid-bit, LSB first
                if (cnt == 16'd0) begin
                    cnt <= DIV[15:0] - 16'd1;
                    sh  <= {rx_s, sh[7:1]};
                    if (bitno == 3'd7) state <= S_STOP;
                    else                bitno <= bitno + 3'd1;
                end else begin
                    cnt <= cnt - 16'd1;
                end
            end

            S_STOP: begin
                if (cnt == 16'd0) begin
                    dout     <= sh;              // accept regardless of stop level;
                    dout_vld <= 1'b1;            // framing errors are not our problem
                    state    <= S_IDLE;
                end else begin
                    cnt <= cnt - 16'd1;
                end
            end

            endcase
        end
    end

endmodule
