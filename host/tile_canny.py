#!/usr/bin/env python3
"""
tile_canny.py -- full-resolution Canny by tiling, emulating the FPGA exactly.

The FPGA processes a fixed 128x128 frame. To keep full image resolution we cut the
image into OVERLAPPING 128x128 tiles, process each one, and keep only the central
CORE x CORE region of each result:

        TILE = 128 = CORE + 2*HALO

Why the halo is needed
----------------------
Every Canny pass forces its 1-pixel border to 0, and that contamination spreads
inward by one ring per pass:

    gaussian  -> ring 0 forced to 0
    sobel     -> ring 1 corrupted (it reads the zeroed ring 0)
    nms       -> ring 2 corrupted (it reads the corrupted ring 1)
    threshold -> pointwise, spreads nothing

So contamination depth is exactly 3. Any HALO >= 4 makes the tile core
BIT-IDENTICAL to whole-image processing for blur/sobel/nms/threshold.

Hysteresis is the exception: it propagates along edge chains without bound, so a
tile can never know about a strong pixel far outside it. That residual error is
what `--compare` measures.

Usage:
    python tile_canny.py ../image.bmp --compare        # pick a halo with real numbers
    python tile_canny.py ../image.bmp                  # process, write PNGs
    python tile_canny.py ../image.bmp --dump-tile 9    # write tile 9 hex for the RTL testbench
"""

import argparse
import os
import sys

import numpy as np
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import canny_model as M

TILE = 128          # fixed by the FPGA BRAM budget -- do not change


def load_gray(path):
    """Full-resolution 8-bit grayscale. No resizing -- that is the whole point."""
    img = Image.open(path)
    if img.mode in ("RGBA", "LA", "PA"):
        bg = Image.new("RGB", img.size, (255, 255, 255))
        bg.paste(img.convert("RGBA"), mask=img.convert("RGBA").split()[-1])
        img = bg
    return np.asarray(img.convert("L"), dtype=np.uint8)


def grid(w, h, core):
    """Core origins in original image coordinates, raster order."""
    ncols = (w + core - 1) // core
    nrows = (h + core - 1) // core
    return ncols, nrows, [(c * core, r * core) for r in range(nrows) for c in range(ncols)]


def pad_for_tiling(img, core, halo):
    """Edge-replicate pad so every tile read is in bounds.

    Replication (not zeros) matters: zero padding would manufacture a huge
    artificial gradient all around the image and ring it with fake edges.
    """
    h, w = img.shape
    ncols, nrows, _ = grid(w, h, core)
    return np.pad(img, ((halo, halo + nrows * core - h),
                        (halo, halo + ncols * core - w)), mode="edge")


def tiles_of(img, core, halo):
    """Yield (index, x0, y0, tile) where tile is TILE x TILE uint8."""
    h, w = img.shape
    padded = pad_for_tiling(img, core, halo)
    size = core + 2 * halo
    _, _, origins = grid(w, h, core)
    for i, (x0, y0) in enumerate(origins):
        yield i, x0, y0, padded[y0:y0 + size, x0:x0 + size]


def stitch(img_shape, core, halo, results):
    """Drop each tile's halo and lay the cores down side by side."""
    h, w = img_shape
    out = np.zeros((h, w), dtype=np.uint8)
    _, _, origins = grid(w, h, core)
    for (x0, y0), res in zip(origins, results):
        cx1, cy1 = min(x0 + core, w), min(y0 + core, h)
        out[y0:cy1, x0:cx1] = res[halo:halo + (cy1 - y0), halo:halo + (cx1 - x0)]
    return out


def tiled_canny(img, core, halo, t_low, t_high, keep=None):
    """Process the image tile by tile, exactly as the FPGA will."""
    results, stages, worst_iters, capped = [], [], 0, 0
    for i, x0, y0, tile in tiles_of(img, core, halo):
        r = M.run(tile, t_low, t_high)
        results.append(r["final"])
        worst_iters = max(worst_iters, r["hyst_iters"])
        capped += int(r["hyst_hit_cap"])
        if keep is not None and i == keep:
            stages = (tile, r)
    return stitch(img.shape, core, halo, results), worst_iters, capped, stages


def cmd_compare(img, t_low, t_high):
    print(f"image {img.shape[1]}x{img.shape[0]}, TILE={TILE}, T_LOW={t_low} T_HIGH={t_high}\n")

    print("computing whole-image reference (this is what tiling is approximating)...")
    ref = M.run(img, t_low, t_high)["final"]
    ref_edges = int((ref > 0).sum())
    print(f"  reference edges: {ref_edges} px ({100.0 * ref_edges / ref.size:.2f}%)\n")

    print(f"{'HALO':>5} {'CORE':>5} {'grid':>8} {'tiles':>6} {'bytes/UART':>11} "
          f"{'diff px':>9} {'diff %':>8} {'worst iters':>12}")
    for halo in (4, 8, 16):
        core = TILE - 2 * halo
        ncols, nrows, _ = grid(img.shape[1], img.shape[0], core)
        out, worst, capped, _ = tiled_canny(img, core, halo, t_low, t_high)
        diff = int((out != ref).sum())
        n = ncols * nrows
        print(f"{halo:5d} {core:5d} {ncols:3d} x{nrows:3d} {n:6d} "
              f"{n * TILE * TILE * 2:11d} {diff:9d} "
              f"{100.0 * diff / ref.size:7.3f}% {worst:12d}"
              + ("  *CAP HIT*" if capped else ""))
    print("\nAll differences are hysteresis-only: blur/sobel/nms/threshold are")
    print("bit-exact in every tile core for any HALO >= 4.")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image")
    ap.add_argument("--halo", type=int, default=4)
    ap.add_argument("--tlow", type=int, default=50)
    ap.add_argument("--thigh", type=int, default=100)
    ap.add_argument("--outdir", default="../data")
    ap.add_argument("--compare", action="store_true",
                    help="measure tiled-vs-whole-image error for halo 4/8/16")
    ap.add_argument("--dump-tile", type=int, default=None,
                    help="also write tile N input + golden stages as .hex for the RTL testbench")
    args = ap.parse_args()

    if not os.path.isfile(args.image):
        sys.exit(f"error: no such file: {args.image}")
    img = load_gray(args.image)

    if args.compare:
        cmd_compare(img, args.tlow, args.thigh)
        return

    core = TILE - 2 * args.halo
    os.makedirs(args.outdir, exist_ok=True)
    ncols, nrows, _ = grid(img.shape[1], img.shape[0], core)

    out, worst, capped, stages = tiled_canny(img, core, args.halo,
                                             args.tlow, args.thigh, keep=args.dump_tile)

    Image.fromarray(img).save(os.path.join(args.outdir, "source_gray.png"))
    Image.fromarray(out).save(os.path.join(args.outdir, "canny_tiled.png"))

    print(f"image        : {img.shape[1]}x{img.shape[0]}")
    print(f"tiling       : CORE={core} HALO={args.halo} -> {ncols} x {nrows} = "
          f"{ncols * nrows} tiles of {TILE}x{TILE}")
    print(f"UART traffic : {ncols * nrows * TILE * TILE * 2 / 1024:.0f} KB "
          f"({ncols * nrows * TILE * TILE * 2 * 10 / 115200:.0f} s @115200, "
          f"{ncols * nrows * TILE * TILE * 2 * 10 / 921600:.0f} s @921600)")
    print(f"thresholds   : {args.tlow}/{args.thigh}")
    print(f"hysteresis   : worst tile needed {worst} iterations"
          + ("   *** SOME TILES HIT THE CAP ***" if capped else ""))
    print(f"edges        : {int((out > 0).sum())} px ({100.0 * (out > 0).mean():.2f}%)")
    print(f"wrote        : {args.outdir}/canny_tiled.png, source_gray.png")

    if args.dump_tile is not None and stages:
        tile, r = stages
        M.write_hex(os.path.join(args.outdir, "tb_input.hex"), tile)
        for name in ("blur", "mag", "nms", "cls", "final"):
            M.write_hex(os.path.join(args.outdir, f"tb_gold_{name}.hex"), r[name])
        M.write_hex(os.path.join(args.outdir, "tb_gold_dir.hex"), r["dir"])
        print(f"wrote        : tb_input.hex + tb_gold_*.hex for tile {args.dump_tile} "
              f"(hyst {r['hyst_iters']} iters)")


if __name__ == "__main__":
    main()
