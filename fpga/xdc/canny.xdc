#===========================================================================
# canny.xdc  --  constraints for the UART Canny edge detector
# Board : EDGE Artix-7, XC7A35T-FTG256
#
# Pins taken from the board's master XDC (new_contrain.xdc).
#
# IMPORTANT: add ONLY this file to the Vivado project. Do NOT add
# new_contrain.xdc itself -- it contains syntax errors (an unclosed brace on
# the pin17 line, doubled braces on the sd_cd / sd_cmd lines) and its
# "20 pin expansion connector" block re-assigns the same 16 package pins that
# the ov7670_* block already uses, which Vivado rejects as duplicates.
#===========================================================================

#---------------------------------------------------------------------------
# Clock -- 50 MHz oscillator
#---------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN N11 IOSTANDARD LVCMOS33} [get_ports clk]
create_clock -period 20.000 -name sys_clk -waveform {0.000 10.000} [get_ports clk]

# The board's master XDC does not waive the dedicated-route check for N11, and
# this design has no MMCM (the 50 MHz input clocks everything directly), so the
# waiver your old project needed should not be required here.
#
# If implementation fails with a placer error about a clock-capable pin,
# uncomment the next line and re-run:
# set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets clk_IBUF]
#
# The old project also had:
#   set_property CLOCK_DEDICATED_ROUTE FALSE [get_nets -of_objects [get_pins m0/m1/clkout1_buf/O]]
# That path names the OLD design's MMCM instance. It is deliberately omitted --
# there is no such instance here and the line would fail with "no pins matched".

#---------------------------------------------------------------------------
# Reset -- centre push button, ACTIVE HIGH
#
# This board has no dedicated reset pin. The buttons are declared PULLDOWN in
# the master XDC, so unpressed reads 0 and pressed reads 1. top.v also runs a
# power-on reset, so the design comes up working without pressing anything --
# this button is only a manual restart.
#---------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN M14 IOSTANDARD LVCMOS33 PULLDOWN true} [get_ports rst_btn]

#---------------------------------------------------------------------------
# USB UART  -- shares the single USB cable with JTAG, enumerates as COM3
#
# The master XDC names these from the FPGA's point of view:
#   usb_uart_txd (C4) = FPGA drives  -> our uart_txd
#   usb_uart_rxd (D4) = FPGA listens -> our uart_rxd
#
# If the self-test times out with no bytes at all, these two are the FIRST
# thing to swap -- vendor master files are not always consistent about whose
# perspective the tx/rx names are written from.
#---------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN C4 IOSTANDARD LVCMOS33} [get_ports uart_txd]
set_property -dict {PACKAGE_PIN D4 IOSTANDARD LVCMOS33} [get_ports uart_rxd]

# uart_rxd is asynchronous and goes through a two-flop synchroniser inside
# uart_rx.v, so there is no real setup/hold relationship to constrain.
set_false_path -from [get_ports uart_rxd]

#---------------------------------------------------------------------------
# Status LEDs
#   [0] receiving   [1] processing   [2] transmitting   [3] error / hyst cap
#---------------------------------------------------------------------------
set_property -dict {PACKAGE_PIN J3 IOSTANDARD LVCMOS33} [get_ports {led[0]}]
set_property -dict {PACKAGE_PIN H3 IOSTANDARD LVCMOS33} [get_ports {led[1]}]
set_property -dict {PACKAGE_PIN J1 IOSTANDARD LVCMOS33} [get_ports {led[2]}]
set_property -dict {PACKAGE_PIN K1 IOSTANDARD LVCMOS33} [get_ports {led[3]}]

# If the LEDs read inverted on your board, do NOT edit the RTL -- set the
# top-level parameter LED_ACTIVE_LOW = 1 instead.

#---------------------------------------------------------------------------
# Configuration -- silences the standard CFGBVS / CONFIG_VOLTAGE DRC warnings
#---------------------------------------------------------------------------
set_property CFGBVS VCCO        [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]
