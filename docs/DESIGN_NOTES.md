# Design notes

The reasoning behind the design: which values were fixed, which were measured,
and where the non-obvious constraints are. For the module-level walkthrough see
[ARCHITECTURE.md](ARCHITECTURE.md).

---

## 1. Fixed design decisions

| Item | Decision | Reason |
|---|---|---|
| Frame size on FPGA | **128 × 128, 8-bit gray** | BRAM math: 8 buffers × 4 BRAM36 = 29 of 50. 160×160 would need 56 and not fit |
| Full-image strategy | **Overlapping tiles**, TILE=128, **HALO=4**, **CORE=120** | 480×720 divides exactly into 4 × 6 = **24 tiles** |
| Input path | UART RX, streamed per tile | Change image or thresholds with no re-synthesis |
| Output path | UART TX → Python stitches → PNG | A real file for the report |
| Architecture | **Multi-pass over BRAM**, not a fused stream | Hysteresis is iterative; frame buffers make it trivial and debuggable |
| Gaussian | 3×3 `[1 2 1; 2 4 2; 1 2 1]/16` | Shift-only; reuses the one window generator |
| Magnitude | `(|Gx|+|Gy|) >> 2`, saturating at 255 | See §3 — measured, not guessed |
| Direction | Integer tangent compare → 0/45/90/135° | No atan, no CORDIC, no divide |
| Borders | 1-pixel frame forced to 0 every pass | Simplest; the halo discards it anyway |
| Hysteresis cap | **256 iterations** | See §3 — measured worst case is 82 |
| Baud | **2 Mbaud** (50 MHz / 25) | Divides exactly on both the FPGA and the FT2232H side: zero baud error |
| Reset | Internal power-on reset + M14 button (active high) | Board has no reset pin; buttons are `PULLDOWN true` |

---

## 2. Tiling — why the halo is exactly 4

Every pass forces its 1-pixel border to 0, and that contamination spreads inward
one ring per pass:

```
gaussian   -> ring 0 forced to 0
sobel      -> ring 1 corrupted (reads the zeroed ring 0)
nms        -> ring 2 corrupted (reads the corrupted ring 1)
threshold  -> pointwise, spreads nothing
```

Contamination depth is exactly **3**, so **HALO ≥ 4** makes each tile's core
**bit-identical** to whole-image processing for blur / sobel / NMS / threshold.

Hysteresis is the exception — it propagates along edge chains without bound, so
a tile can never know about a strong pixel far outside it. Measured against a
whole-image reference at T=50/100:

| HALO | CORE | grid | tiles | UART bytes | diff vs whole image |
|---|---|---|---|---|---|
| **4** | **120** | **4 × 6** | **24** | **786 K** | **0.815 %** |
| 8 | 112 | 5 × 7 | 35 | 1147 K | 0.761 % |
| 16 | 96 | 5 × 8 | 40 | 1311 K | 0.719 % |

Quadrupling the halo buys 0.1 % accuracy for 67 % more traffic. **HALO=4 wins**,
and the flat curve confirms the residual is inherent to tiling hysteresis, not
something a bigger halo fixes. No visible seams in the output.

Edge tiles are padded by **replication**, not zeros — zero padding would
manufacture a huge artificial gradient and ring the image with fake edges.

---

## 3. Two design values that were measured, not assumed

**Magnitude shift = 2, not 3.** `|Gx|+|Gy|` spans 0..2040, so `>>3` never clips.
But on real images the magnitude only reached **118 of 255** — over half the
range wasted, edges crammed into coarse low codes. Measured:

| shift | max (city) | clipped px | NMS survivors |
|---|---|---|---|
| >>3 | 118 | 0 | 5697 |
| **>>2** | **236** | **0** | **5507** |
| >>1 | 255 | 56 | 5403 (saturation destroying discrimination) |

`>>2` doubles usable range *and* cuts spurious NMS ties. `>>1` clips badly.

**Hysteresis cap = 256, not 32.** Promotion moves one pixel per iteration along
a weak chain, so the count scales with the longest edge. Measured: 28 for the
city photo, **89 for the synthetic pattern**, 82 worst-case across the 24 real
tiles. A 32 cap silently lost 57 pixels. Each iteration is 16384 cycles =
0.33 ms, so 256 costs 84 ms worst case — nothing next to the UART transfer.

---

## 4. Thresholds depend on IMAGE RESOLUTION, not just brightness

Sobel is a 3×3 *local* operator, so what matters is the gradient **per pixel**,
not the total contrast of an edge. Downscaling concentrates an edge into 1–2
pixels and makes it steep; native 4K spreads the same physical edge over many
pixels and makes it shallow.

Measured on a 3840×2160 road photograph, the asphalt/grass boundary — visually an
obvious high-contrast edge:

```
profile across the edge : 26, 42, 57, 69, 76, 76, 83, 106, 122, 124, 114, 102, 95
total contrast          : ~100 levels
biggest 1-pixel step    : only 23 levels   -> Sobel magnitude ~23
```

With `T_LOW=50` that edge is invisible. Densities for three representative tiles:

| T_LOW/T_HIGH | road edge | foliage | bare asphalt |
|---|---|---|---|
| 50/100 | 1.38 % | 9.83 % | 0.00 % |
| **20/40** | **5.55 %** | 23.68 % | 3.05 % |
| 12/25 | 10.86 % | 28.11 % | 3.05 % |

**Rule of thumb:** the 50/100 defaults suit heavily downscaled images. For
native multi-megapixel photos start around **20/40**. Below ~60 on `T_HIGH`,
flat regions begin returning sensor noise, so there is a floor.

Dense texture wants high thresholds while soft edges want low ones — a single
global threshold cannot serve both. That is a limitation of Canny itself, not
of this implementation.

Thresholds ride in the UART header, so tuning costs **no re-synthesis** — and
`host/tile_canny.py` reproduces the FPGA bit-exactly, so tune in software (free,
instant) before spending a hardware run.

---

## 5. The five passes

```
 m_in   --[P1 Gaussian 3x3]--> m_blur
 m_blur --[P2 Sobel]--+------> m_mag   (8-bit magnitude)
                      +------> m_dir   (2-bit direction)
 m_mag + m_dir --[P3 NMS]----> m_nms
 m_nms  --[P4 Double Thresh]-> m_cls   (0 / 128 / 255)
 m_cls  --[P5 Hysteresis]----> m_a <-> m_b   loop until nothing changes
```

Direction quantisation (`ax=|Gx|`, `ay=|Gy|`, Q8: tan22.5°≈106/256, tan67.5°≈618/256):

```
ay*256 <= ax*106      -> dir 0 (0°)    compare W  / E
ay*256 >= ax*618      -> dir 2 (90°)   compare N  / S
sign(Gx)==sign(Gy)    -> dir 3 (135°)  compare NW / SE
otherwise             -> dir 1 (45°)   compare NE / SW
```

Hysteresis: a weak pixel (128) with any strong (255) 8-neighbour is promoted.
Ping-pong `m_a`↔`m_b`, stop when an iteration promotes nothing or at 256.
Surviving weak pixels are mapped to 0 **at read-out**, not in an extra pass, so
the pre-hysteresis classification stays inspectable over UART.

---

## 6. Pipeline timing (get this wrong and the image comes out shifted)

```
fetch presents raddr
 +1   BRAM dout valid        -> window3x3 in_vld
 +2   window3x3 tap stage
 +3   window3x3 window regs  -> win_vld / win_addr / win_border
 +4   alignment stage        -> m_dir dout lands here too
 +5   operator stage 1
 +6   operator stage 2       -> write at wa2
```

The window centre lags its input by `W+1` pixels, so each pass feeds
`N + W + 1` samples (N real, then W+1 zeros) to flush all N windows out.

All five operators have **identical 2-clock latency**, so the core tracks
validity once for all of them.

---

## 7. BRAM budget

| Buffer | Width | BRAM36 | Purpose |
|---|---|---|---|
| `m_in` | 8 | 4 | raw tile from UART |
| `m_blur` | 8 | 4 | Gaussian output |
| `m_mag` | 8 | 4 | gradient magnitude |
| `m_dir` | 2 | 1 | quantised direction |
| `m_nms` | 8 | 4 | after NMS |
| `m_cls` | 8 | 4 | after double threshold |
| `m_a` | 8 | 4 | hysteresis ping |
| `m_b` | 8 | 4 | hysteresis pong |
| **Total** | | **29 of 50** | **58 %** |

Line buffers in `window3x3` are forced to distributed RAM so they don't eat
into this.

---

## 8. UART protocol

**PC → FPGA:** `A5 5A` · `CMD` · `W_lo W_hi` · `H_lo H_hi` · `T_LOW T_HIGH` · `W*H bytes`
**FPGA → PC:** `5A A5` · `W*H bytes`

W/H are checked against the synthesis parameters — a mismatch lights `led[3]`
instead of silently producing garbage.

### CMD — which buffer comes back (the debugging backbone)

| CMD | Returns |
|---|---|
| `0x01` | final Canny result |
| `0x02` | `m_in` echo — proves the link with zero processing |
| `0x03` | after Gaussian |
| `0x04` | gradient magnitude |
| `0x05` | after NMS |
| `0x06` | after double threshold |

If the output is wrong, dump each stage; whichever first disagrees with the
software model is the broken pass. This turns "the design is broken" into "op_nms
is broken".

---

## 9. Known risks

| Risk | Mitigation |
|---|---|
| **C4/D4 tx/rx swapped** — vendor master XDCs are inconsistent about perspective | Most likely cause of a dead self-test. Swap the two `PACKAGE_PIN` values, rebuild. 2 minutes. |
| Sobel direction compare misses timing at 50 MHz | Already split across 2 pipeline stages. If WNS is still negative, split the constant multiplies into a third stage. |
| LEDs read inverted | Set `LED_ACTIVE_LOW = 1`; do not edit RTL |
| N11 needs a dedicated-route waiver | Commented line ready in `canny.xdc` |
| Hysteresis hits the 256 cap | `led[3]` lights; raise `MAX_HYST` in `canny_core.v` |
