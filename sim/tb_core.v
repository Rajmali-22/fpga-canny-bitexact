`timescale 1ns / 1ps
// Testbench: drives canny_core directly (no UART), dumps every stage to hex files.
module tb_core;
    localparam W = 128, H = 128, AW = 14, N = W*H;
    reg clk = 0; always #10 clk = ~clk;          // 50 MHz
    reg rst_n = 0, start = 0;
    reg [7:0] t_low, t_high;
    reg in_we = 0; reg [AW-1:0] in_addr = 0; reg [7:0] in_data = 0;
    reg [2:0] rd_sel = 1; reg [AW-1:0] rd_addr = 0;
    wire [7:0] rd_data; wire done; wire [8:0] hyst_iters; wire hyst_capped;

    canny_core #(.W(W), .H(H), .AW(AW)) dut (
        .clk(clk), .rst_n(rst_n), .start(start), .done(done),
        .t_low(t_low), .t_high(t_high),
        .in_we(in_we), .in_addr(in_addr), .in_data(in_data),
        .rd_sel(rd_sel), .rd_addr(rd_addr), .rd_data(rd_data),
        .hyst_iters(hyst_iters), .hyst_capped(hyst_capped));

    reg [7:0] img [0:N-1];
    reg [7:0] out [0:N-1];
    reg [1023:0] infile, outprefix, fname;
    integer i, s, tl, th;
    integer cyc_start, cyc;

    task dump(input [2:0] sel, input [8*8-1:0] tag);
        begin
            rd_sel = sel;
            for (i = 0; i < N; i = i + 1) begin
                rd_addr = i;
                @(posedge clk); #1;       // BRAM read latency: 1 clock
                out[i] = rd_data;
            end
            $sformat(fname, "%0s_%0s.hex", outprefix, tag);
            $writememh(fname, out);
        end
    endtask

    initial begin
        if (!$value$plusargs("IN=%s", infile)) begin $display("need +IN="); $finish; end
        if (!$value$plusargs("OUT=%s", outprefix)) outprefix = "sim";
        if (!$value$plusargs("TL=%d", tl)) tl = 25;
        if (!$value$plusargs("TH=%d", th)) th = 60;
        t_low = tl; t_high = th;
        $readmemh(infile, img);
        repeat (5) @(posedge clk); #1 rst_n = 1;
        repeat (2) @(posedge clk);
        // write the frame through the host port
        for (i = 0; i < N; i = i + 1) begin
            #1 in_we = 1; in_addr = i; in_data = img[i];
            @(posedge clk);
        end
        #1 in_we = 0;
        @(posedge clk);
        // one-clock start pulse
        #1 start = 1; cyc_start = $time;
        @(posedge clk); #1 start = 0;
        // wait for completion
        cyc = 0;
        while (!done) begin @(posedge clk); cyc = cyc + 1; if (cyc > 20000000) begin $display("TIMEOUT"); $finish; end end
        $display("RESULT cycles=%0d hyst_iters=%0d capped=%0d", cyc, hyst_iters, hyst_capped);
        @(posedge clk);
        dump(3'd1, "final");
        dump(3'd3, "blur");
        dump(3'd4, "mag");
        dump(3'd5, "nms");
        dump(3'd6, "cls");
        $finish;
    end
endmodule
