#!/usr/bin/env python3
"""
canny_model.py -- BIT-EXACT golden reference for the FPGA Canny pipeline.

This is not "a Canny implementation". It is a deliberate, operation-for-operation
mirror of what fpga/rtl/canny_core.v does: same integer truncation, same shift amounts,
same integer tangent comparison for direction, same zeroed borders, same iterative
hysteresis with the same 256-iteration cap.

Any difference between this and the Verilog simulation output is a BUG in one of
them. That is the whole point -- it turns M1 verification from "looks like edges"
into a byte-comparison.

Usage:
    python canny_model.py ../sim/input.hex
    python canny_model.py ../sim/input.hex --tlow 25 --thigh 60
    python canny_model.py ../sim/input.hex --sweep
"""

import argparse
import os
import sys

import numpy as np
from PIL import Image

# Integer tangent constants, Q8 fixed point.
#   tan(22.5 deg) = 0.41421 -> 106/256
#   tan(67.5 deg) = 2.41421 -> 618/256
TAN22 = 106
TAN67 = 618

# Hysteresis propagates one pixel per iteration along a weak chain, so the count
# needed scales with the longest weak edge. Measured: 28 for the city photo, 89 for
# the synthetic pattern. 256 gives headroom; at 128x128 each iteration is 16384
# cycles = 0.33 ms @ 50 MHz, so the worst case is 84 ms -- negligible next to the
# 1.4 s UART transfer.
HYST_MAX_ITER = 256         # hard cap, must match the RTL parameter

STRONG = 255
WEAK = 128


def _win(img):
    """Return the 9 shifted planes of a 3x3 window, in RTL naming order.

    p00 p01 p02      NW N NE
    p10 p11 p12   =   W C  E
    p20 p21 p22      SW S SE

    Each plane is the same shape as `img`; index [y][x] of plane pNN holds the
    value that the RTL's window register pNN would hold when the window centre
    is at (x, y). Border pixels are garbage here and get masked to 0 by the
    caller, exactly as the RTL forces its 1-pixel border to 0.
    """
    p = np.pad(img.astype(np.int32), 1, mode="constant")
    h, w = img.shape
    return [p[dy:dy + h, dx:dx + w] for dy in (0, 1, 2) for dx in (0, 1, 2)]


def _zero_border(a):
    a[0, :] = 0
    a[-1, :] = 0
    a[:, 0] = 0
    a[:, -1] = 0
    return a


def gaussian3x3(img):
    """P1: [1 2 1; 2 4 2; 1 2 1] / 16, integer truncating shift."""
    p00, p01, p02, p10, p11, p12, p20, p21, p22 = _win(img)
    acc = (p00 + 2 * p01 + p02
           + 2 * p10 + 4 * p11 + 2 * p12
           + p20 + 2 * p21 + p22)
    return _zero_border((acc >> 4).astype(np.uint8))


def sobel(img):
    """P2: magnitude = min(255, (|Gx|+|Gy|) >> 2); direction quantised to 0/1/2/3.

    The centre pixel p11 is intentionally unused -- both Sobel kernels have 0 there.
    It is still unpacked so the naming stays identical to the RTL window registers.

    Shift is 2, not 3. |Gx|+|Gy| spans 0..2040, so >>3 would fit 8 bits with no
    clipping at all -- but measurement showed real images only reach ~118/255, i.e.
    over half the range wasted and edges crammed into coarse low codes. >>2 spans
    0..510 and saturates, which costs 0-3 pixels on real images (all of them far
    above T_HIGH anyway, so still strong) and halves NMS quantisation ties.
    """
    p00, p01, p02, p10, p11, p12, p20, p21, p22 = _win(img)

    gx = (-p00 + p02) + (-2 * p10 + 2 * p12) + (-p20 + p22)
    gy = (-p00 - 2 * p01 - p02) + (p20 + 2 * p21 + p22)

    ax, ay = np.abs(gx), np.abs(gy)

    mag = np.minimum((ax + ay) >> 2, 255).astype(np.uint8)
    _zero_border(mag)

    # dir 0 = 0 deg (compare W/E), 1 = 45 (NE/SW), 2 = 90 (N/S), 3 = 135 (NW/SE)
    same_sign = (gx >= 0) == (gy >= 0)
    dirn = np.where(ay * 256 <= ax * TAN22, 0,
           np.where(ay * 256 >= ax * TAN67, 2,
           np.where(same_sign, 3, 1))).astype(np.uint8)
    return mag, dirn


def nms(mag, dirn):
    """P3: keep the centre only if it is >= both neighbours along the gradient."""
    p00, p01, p02, p10, p11, p12, p20, p21, p22 = _win(mag)
    c = p11

    keep = np.select(
        [dirn == 0, dirn == 1, dirn == 2, dirn == 3],
        [(c >= p10) & (c >= p12),      # 0 deg   -> W  / E
         (c >= p02) & (c >= p20),      # 45 deg  -> NE / SW
         (c >= p01) & (c >= p21),      # 90 deg  -> N  / S
         (c >= p00) & (c >= p22)],     # 135 deg -> NW / SE
        default=False)

    return _zero_border(np.where(keep, c, 0).astype(np.uint8))


def double_threshold(img, t_low, t_high):
    """P4: 255 = strong, 128 = weak, 0 = suppressed."""
    out = np.where(img >= t_high, STRONG,
          np.where(img >= t_low, WEAK, 0)).astype(np.uint8)
    return _zero_border(out)


def hysteresis(cls):
    """P5: weak pixels touching a strong 8-neighbour are promoted; repeat until stable."""
    cur = cls.copy()
    iters = 0
    for iters in range(1, HYST_MAX_ITER + 1):
        planes = _win((cur == STRONG).astype(np.uint8))
        # any of the 8 neighbours strong (index 4 is the centre itself)
        near_strong = np.zeros(cur.shape, dtype=bool)
        for i, pl in enumerate(planes):
            if i != 4:
                near_strong |= pl.astype(bool)
        promote = (cur == WEAK) & near_strong
        if not promote.any():
            break
        cur = np.where(promote, STRONG, cur).astype(np.uint8)
        _zero_border(cur)

    hit_cap = iters == HYST_MAX_ITER and promote.any()
    final = np.where(cur == STRONG, STRONG, 0).astype(np.uint8)
    return _zero_border(final), iters, hit_cap


def run(img, t_low, t_high):
    """Run all five passes. Returns the dict of every intermediate buffer."""
    blur = gaussian3x3(img)
    mag, dirn = sobel(blur)
    sup = nms(mag, dirn)
    cls = double_threshold(sup, t_low, t_high)
    final, iters, hit_cap = hysteresis(cls)
    return {"in": img, "blur": blur, "mag": mag, "dir": dirn,
            "nms": sup, "cls": cls, "final": final,
            "hyst_iters": iters, "hyst_hit_cap": hit_cap}


def read_hex(path):
    vals = []
    with open(path) as f:
        for raw in f:
            line = raw.split("//")[0].strip()
            for tok in line.split():
                vals.append(0 if set(tok.lower()) <= set("xz") else int(tok, 16))
    return np.array(vals, dtype=np.uint8)


def write_hex(path, arr):
    with open(path, "w") as f:
        f.write("".join(f"{v:02x}\n" for v in arr.reshape(-1)))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile", help="input .hex or .bin")
    ap.add_argument("--size", type=int, default=128)
    ap.add_argument("--tlow", type=int, default=25)
    ap.add_argument("--thigh", type=int, default=60)
    ap.add_argument("--outdir", default=None, help="default: same dir as infile")
    ap.add_argument("--sweep", action="store_true",
                    help="try a grid of thresholds and report edge density for each")
    args = ap.parse_args()

    if not os.path.isfile(args.infile):
        sys.exit(f"error: no such file: {args.infile}")

    if args.infile.lower().endswith(".hex"):
        data = read_hex(args.infile)
    else:
        data = np.frombuffer(open(args.infile, "rb").read(), dtype=np.uint8)
    img = data.reshape(args.size, args.size)

    if args.sweep:
        print(f"{'T_LOW':>6} {'T_HIGH':>7} {'edge %':>8} {'hyst iters':>11}")
        for t_high in (40, 60, 80, 100, 120):
            for t_low in (t_high // 4, t_high // 3, t_high // 2):
                r = run(img, t_low, t_high)
                pct = 100.0 * (r["final"] > 0).mean()
                print(f"{t_low:6d} {t_high:7d} {pct:7.2f}% {r['hyst_iters']:11d}")
        return

    outdir = args.outdir or os.path.dirname(os.path.abspath(args.infile))
    os.makedirs(outdir, exist_ok=True)
    r = run(img, args.tlow, args.thigh)

    # dir is 2-bit; scale to 0/85/170/255 purely so the preview PNG is visible
    previews = {"blur": r["blur"], "mag": r["mag"], "dir": (r["dir"] * 85).astype(np.uint8),
                "nms": r["nms"], "cls": r["cls"], "final": r["final"]}
    for name, arr in previews.items():
        Image.fromarray(arr).save(os.path.join(outdir, f"model_{name}.png"))
    for name in ("blur", "mag", "nms", "cls", "final"):
        write_hex(os.path.join(outdir, f"gold_{name}.hex"), r[name])
    write_hex(os.path.join(outdir, "gold_dir.hex"), r["dir"])

    print(f"input        : {args.infile}  {args.size}x{args.size}")
    print(f"thresholds   : T_LOW={args.tlow}  T_HIGH={args.thigh}")
    print(f"mag          : max={r['mag'].max()} mean={r['mag'].mean():.1f}")
    print(f"strong px    : {int((r['cls'] == STRONG).sum())}")
    print(f"weak px      : {int((r['cls'] == WEAK).sum())}")
    print(f"hysteresis   : converged in {r['hyst_iters']} iterations"
          + ("  *** HIT THE ITERATION CAP ***" if r["hyst_hit_cap"] else ""))
    print(f"final edges  : {int((r['final'] > 0).sum())} px "
          f"({100.0 * (r['final'] > 0).mean():.2f}%)")
    print(f"wrote        : model_*.png and gold_*.hex in {outdir}")


if __name__ == "__main__":
    main()
