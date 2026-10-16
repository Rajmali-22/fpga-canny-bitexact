# FPGA sources

Everything in this folder goes into Vivado. Nothing here runs on the PC.

## What to add to the Vivado project

**Design sources** — all 11 files in `rtl/`:

| File | Role |
|---|---|
| `top.v` | **set as top module** — UART protocol FSM, power-on reset, LEDs |
| `canny_core.v` | pass sequencer + the 8 frame buffers |
| `window3x3.v` | line buffers → sliding 3×3 window |
| `op_gauss.v` | pass 1 — Gaussian blur |
| `op_sobel.v` | pass 2 — gradient magnitude + direction |
| `op_nms.v` | pass 3 — non-maximum suppression |
| `op_thresh.v` | pass 4 — double threshold |
| `op_hyst.v` | pass 5 — hysteresis, one iteration |
| `uart_rx.v` / `uart_tx.v` | 8N1 serial |
| `dpram.v` | generic dual-port BRAM |

**Constraints** — `xdc/canny.xdc` **only**.

**Part:** `xc7a35tftg256-1`

## Pinout in use

| Signal | Pin | Note |
|---|---|---|
| `clk` | N11 | 50 MHz |
| `uart_txd` | C4 | FPGA → PC |
| `uart_rxd` | D4 | PC → FPGA |
| `rst_btn` | M14 | centre button, **active high**; optional (there is a power-on reset) |
| `led[3:0]` | J3 H3 J1 K1 | rx / proc / tx / error |

## Expected results

- **Timing:** WNS positive at 50 MHz
- **BRAM:** ~29 of 50 BRAM36 (58%)
- **LUTs:** under 10%
- **Harmless warning:** `p11` unused in `op_sobel` — both Sobel kernels have a
  zero in the centre, so the window centre genuinely isn't read

## Tuning without a rebuild

Thresholds are sent in the UART header, so `--tlow` / `--thigh` need no
re-synthesis. Only these require rebuilding:

| Change | Where |
|---|---|
| Baud rate | `BAUD` in `top.v` |
| Frame size | `W` / `H` / `AW` in `top.v` (AW = log2(W*H)) |
| Hysteresis cap | `MAX_HYST` in `canny_core.v` |
| LED polarity | `LED_ACTIVE_LOW` in `top.v` |
