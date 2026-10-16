`timescale 1ns / 1ps
//===========================================================================
// top.v  --  UART <-> Canny frame processor.
//
// The PC cuts a full-resolution image into overlapping 128x128 tiles and sends
// them one at a time. Each transaction is:
//
//   PC  -> FPGA :  A5 5A  CMD  W_lo W_hi  H_lo H_hi  T_LOW T_HIGH  <W*H bytes>
//   FPGA -> PC  :  5A A5  <W*H bytes>
//
// CMD selects which internal buffer comes back, which is the whole debugging
// strategy: if the result looks wrong, dump each stage and see which pass broke.
//
//   0x01  final Canny result          0x04  gradient magnitude
//   0x02  input echo (proves the link) 0x05  after non-max suppression
//   0x03  after Gaussian blur          0x06  after double threshold
//
// The magic bytes let the receiver resynchronise if a byte is ever lost, and
// W/H are checked against the synthesis parameters so a mismatched host script
// fails loudly on led[3] instead of silently producing garbage.
//
// LEDs: [0] receiving  [1] processing  [2] transmitting
//       [3] error -- size mismatch, or hysteresis hit its iteration cap
//===========================================================================
module top #(
    parameter CLK_HZ         = 50_000_000,   // oscillator on pin N11
    // 2 Mbaud divides EXACTLY on both sides: 50 MHz/25 here, 48 MHz/24 in the
    // FT2232H. Zero baud error, unlike 921600 (0.47%) or 3 M (1.96%).
    // 5_000_000 also works (50 MHz/10 exact, FTDI 0.26%) if you want it faster
    // still, but only 10 clocks per bit leaves less sampling margin.
    parameter BAUD           = 2_000_000,
    parameter W              = 128,
    parameter H              = 128,
    parameter AW             = 14,           // $clog2(W*H)
    parameter LED_ACTIVE_LOW = 0             // flip to 1 if the LEDs read inverted
)(
    input  wire       clk,
    input  wire       rst_btn,      // centre push button, ACTIVE HIGH (pin has PULLDOWN)
    input  wire       uart_rxd,     // PC -> FPGA
    output wire       uart_txd,     // FPGA -> PC
    output wire [3:0] led
);

    localparam N = W * H;

    //=====================================================================
    // Reset
    //
    // The board has no dedicated reset pin -- M6 is a slide switch and the
    // push buttons are active HIGH (they carry PULLDOWN true, so an unpressed
    // button reads 0). So reset is generated internally instead: a power-on
    // counter holds the design for ~1.3 ms after configuration, which means it
    // starts working the moment it is programmed with no button press at all.
    // The centre button is ORed in as a manual restart.
    //
    // Everything downstream still sees a conventional active-low rst_n, and it
    // is released synchronously so no flop sees a reset edge near its clock.
    //=====================================================================
    reg [15:0] por_cnt = 16'd0;
    reg        por     = 1'b1;

    always @(posedge clk) begin
        if (por_cnt == 16'hFFFF) por     <= 1'b0;
        else                     por_cnt <= por_cnt + 1'b1;
    end

    reg [1:0] rst_sync = 2'b00;
    always @(posedge clk) rst_sync <= {rst_sync[0], ~(por | rst_btn)};

    wire rst_n = rst_sync[1];

    //=====================================================================
    // UART
    //=====================================================================
    wire       rx_vld;
    wire [7:0] rx_byte;

    uart_rx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_rx (
        .clk(clk), .rst_n(rst_n), .rx(uart_rxd),
        .dout_vld(rx_vld), .dout(rx_byte)
    );

    reg        tx_vld;
    reg  [7:0] tx_byte;
    wire       tx_busy;

    uart_tx #(.CLK_HZ(CLK_HZ), .BAUD(BAUD)) u_tx (
        .clk(clk), .rst_n(rst_n),
        .din_vld(tx_vld), .din(tx_byte), .tx(uart_txd), .busy(tx_busy)
    );

    //=====================================================================
    // Canny core
    //=====================================================================
    reg           core_start;
    wire          core_done;
    reg  [7:0]    t_low_r, t_high_r;
    reg           in_we;
    reg  [AW-1:0] in_addr;
    reg  [7:0]    in_data;
    reg  [2:0]    rd_sel;
    reg  [AW-1:0] rd_addr;
    wire [7:0]    rd_data;
    wire [8:0]    hyst_iters;
    wire          hyst_capped;

    canny_core #(.W(W), .H(H), .AW(AW)) u_core (
        .clk(clk), .rst_n(rst_n),
        .start(core_start), .done(core_done),
        .t_low(t_low_r), .t_high(t_high_r),
        .in_we(in_we), .in_addr(in_addr), .in_data(in_data),
        .rd_sel(rd_sel), .rd_addr(rd_addr), .rd_data(rd_data),
        .hyst_iters(hyst_iters), .hyst_capped(hyst_capped)
    );

    //=====================================================================
    // Protocol FSM
    //=====================================================================
    localparam S_M0    = 4'd0,  S_M1    = 4'd1,  S_CMD   = 4'd2,
               S_WLO   = 4'd3,  S_WHI   = 4'd4,  S_HLO   = 4'd5,
               S_HHI   = 4'd6,  S_TLO   = 4'd7,  S_THI   = 4'd8,
               S_PIX   = 4'd9,  S_GO    = 4'd10, S_WAIT  = 4'd11,
               S_TXA   = 4'd12, S_TXB   = 4'd13, S_TXSET = 4'd14,
               S_TXSND = 4'd15;

    reg [3:0]    st;
    reg [7:0]    cmd;
    reg [15:0]   rx_w, rx_h;
    reg [AW-1:0] pix_cnt;
    reg          err;

    // A byte may only be handed to uart_tx when it is neither busy nor about to
    // become busy. tx_busy lags the tx_vld pulse by one clock, so testing
    // tx_busy alone would push a second byte into the gap and lose it.
    wire tx_ready = !tx_busy && !tx_vld;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st         <= S_M0;
            cmd        <= 8'd0;
            rx_w       <= 16'd0;
            rx_h       <= 16'd0;
            pix_cnt    <= {AW{1'b0}};
            err        <= 1'b0;
            core_start <= 1'b0;
            in_we      <= 1'b0;
            in_addr    <= {AW{1'b0}};
            in_data    <= 8'd0;
            rd_sel     <= 3'd1;
            rd_addr    <= {AW{1'b0}};
            tx_vld     <= 1'b0;
            tx_byte    <= 8'd0;
            t_low_r    <= 8'd50;
            t_high_r   <= 8'd100;
        end else begin
            core_start <= 1'b0;
            in_we      <= 1'b0;
            tx_vld     <= 1'b0;

            case (st)

            //--------------------------------------------------- header
            S_M0: if (rx_vld && rx_byte == 8'hA5) st <= S_M1;

            S_M1: if (rx_vld) begin
                      if      (rx_byte == 8'h5A) st <= S_CMD;
                      else if (rx_byte == 8'hA5) st <= S_M1;   // A5 A5 ... keep hunting
                      else                       st <= S_M0;
                  end

            S_CMD: if (rx_vld) begin cmd        <= rx_byte; st <= S_WLO; end
            S_WLO: if (rx_vld) begin rx_w[7:0]  <= rx_byte; st <= S_WHI; end
            S_WHI: if (rx_vld) begin rx_w[15:8] <= rx_byte; st <= S_HLO; end
            S_HLO: if (rx_vld) begin rx_h[7:0]  <= rx_byte; st <= S_HHI; end
            S_HHI: if (rx_vld) begin rx_h[15:8] <= rx_byte; st <= S_TLO; end
            S_TLO: if (rx_vld) begin t_low_r    <= rx_byte; st <= S_THI; end

            S_THI: if (rx_vld) begin
                       t_high_r <= rx_byte;
                       if (rx_w == W && rx_h == H) begin
                           err     <= 1'b0;
                           pix_cnt <= {AW{1'b0}};
                           st      <= S_PIX;
                       end else begin
                           err <= 1'b1;                        // host/bitstream mismatch
                           st  <= S_M0;
                       end
                   end

            //--------------------------------------------------- payload in
            S_PIX: if (rx_vld) begin
                       in_we   <= 1'b1;
                       in_addr <= pix_cnt;
                       in_data <= rx_byte;
                       if (pix_cnt == (N - 1)) st      <= S_GO;
                       else                    pix_cnt <= pix_cnt + 1'b1;
                   end

            //--------------------------------------------------- process
            S_GO: begin
                      core_start <= 1'b1;
                      st         <= S_WAIT;
                  end

            // core_done is still high from the PREVIOUS run during the cycle
            // core_start is asserted, so it must be qualified.
            S_WAIT: if (core_done && !core_start) begin
                        rd_sel  <= cmd[2:0];
                        rd_addr <= {AW{1'b0}};
                        st      <= S_TXA;
                    end

            //--------------------------------------------------- payload out
            S_TXA: if (tx_ready) begin tx_byte <= 8'h5A; tx_vld <= 1'b1; st <= S_TXB;   end
            S_TXB: if (tx_ready) begin tx_byte <= 8'hA5; tx_vld <= 1'b1; st <= S_TXSET; end

            // one dead cycle so the BRAM read at rd_addr lands before it is used
            S_TXSET: st <= S_TXSND;

            S_TXSND: if (tx_ready) begin
                         tx_byte <= rd_data;
                         tx_vld  <= 1'b1;
                         if (rd_addr == (N - 1)) begin
                             st <= S_M0;
                         end else begin
                             rd_addr <= rd_addr + 1'b1;
                             st      <= S_TXSET;
                         end
                     end

            default: st <= S_M0;
            endcase
        end
    end

    //=====================================================================
    // Status LEDs
    //=====================================================================
    wire rxing = (st == S_CMD) || (st == S_WLO) || (st == S_WHI) ||
                 (st == S_HLO) || (st == S_HHI) || (st == S_TLO) ||
                 (st == S_THI) || (st == S_PIX);
    wire procing = (st == S_GO)  || (st == S_WAIT);
    wire txing   = (st == S_TXA) || (st == S_TXB) ||
                   (st == S_TXSET) || (st == S_TXSND);

    wire [3:0] led_raw = {err | hyst_capped, txing, procing, rxing};

    assign led = LED_ACTIVE_LOW ? ~led_raw : led_raw;

endmodule
