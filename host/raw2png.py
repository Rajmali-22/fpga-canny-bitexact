#!/usr/bin/env python3
"""
raw2png.py -- turn an FPGA/simulation output buffer back into a viewable image.

Accepts either a .hex file (one hex byte per line, as written by Verilog's
$writememh) or a raw .bin file, and writes a PNG.

Usage:
    python raw2png.py out_final.hex
    python raw2png.py out_final.hex --size 128 -o final.png
    python raw2png.py output.bin --size 128
"""

import argparse
import os
import sys

import numpy as np
from PIL import Image


def read_hex(path):
    """Read $writememh output. Tolerates blank lines, // comments and 'xx' fills."""
    vals = []
    with open(path) as f:
        for lineno, raw in enumerate(f, 1):
            line = raw.split("//")[0].strip()
            if not line:
                continue
            for tok in line.split():
                if set(tok.lower()) <= set("xz"):
                    vals.append(0)          # uninitialised -> black
                    continue
                try:
                    vals.append(int(tok, 16))
                except ValueError:
                    sys.exit(f"error: {path}:{lineno}: not a hex byte: {tok!r}")
    return np.array(vals, dtype=np.uint8)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("infile", help="input .hex or .bin")
    ap.add_argument("--size", type=int, default=128, help="frame size, square (default 128)")
    ap.add_argument("-o", "--out", default=None, help="output PNG (default: <infile>.png)")
    ap.add_argument("--scale", type=int, default=1,
                    help="nearest-neighbour upscale for easier viewing (default 1)")
    args = ap.parse_args()

    if not os.path.isfile(args.infile):
        sys.exit(f"error: no such file: {args.infile}")

    if args.infile.lower().endswith(".hex"):
        data = read_hex(args.infile)
    else:
        data = np.frombuffer(open(args.infile, "rb").read(), dtype=np.uint8)

    want = args.size * args.size
    if data.size != want:
        print(f"warning: got {data.size} bytes, expected {want} "
              f"for {args.size}x{args.size} -- padding/truncating", file=sys.stderr)
        fixed = np.zeros(want, dtype=np.uint8)
        fixed[:min(want, data.size)] = data[:want]
        data = fixed

    img = Image.fromarray(data.reshape(args.size, args.size))
    if args.scale > 1:
        img = img.resize((args.size * args.scale,) * 2, Image.NEAREST)

    out = args.out or os.path.splitext(args.infile)[0] + ".png"
    img.save(out)

    nz = int((data > 0).sum())
    print(f"read  : {args.infile}  ({data.size} bytes)")
    print(f"stats : min={data.min()} max={data.max()} mean={data.mean():.1f} "
          f"nonzero={nz} ({100.0 * nz / data.size:.1f}%)")
    print(f"wrote : {out}")


if __name__ == "__main__":
    main()
