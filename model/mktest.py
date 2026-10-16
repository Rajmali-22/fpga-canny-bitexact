#!/usr/bin/env python3
"""
mktest.py -- synthetic test pattern for verifying the Canny RTL.

Generates geometry whose correct edge response is known by inspection:

  * a filled square      -> pure horizontal + vertical edges (dir 0 and dir 2)
  * a filled circle      -> every direction, continuously
  * two diagonal bars    -> dir 1 (45 deg) and dir 3 (135 deg) specifically
  * a soft gradient wedge-> should produce NO edges (tests threshold behaviour)

If the direction quantiser or NMS is wrong, a city photo still looks plausible.
On this pattern it is immediately obvious.

Usage:
    python mktest.py --outdir ../sim
"""

import argparse
import os

import numpy as np
from PIL import Image


def make(size):
    img = np.full((size, size), 32, dtype=np.uint8)
    yy, xx = np.mgrid[0:size, 0:size]
    s = size / 128.0          # scale factors tuned at 128x128

    # filled square -- axis-aligned edges
    a, b = int(10 * s), int(45 * s)
    img[a:b, a:b] = 220

    # filled circle -- all directions
    cy, cx, r = 30 * s, 92 * s, 18 * s
    img[((yy - cy) ** 2 + (xx - cx) ** 2) <= r * r] = 200

    # 45 deg bar (runs up-right): constant x+y band
    d = xx + yy
    img[(d >= int(96 * s)) & (d < int(104 * s)) & (yy > int(58 * s))] = 240

    # 135 deg bar (runs down-right): constant x-y band
    d2 = xx - yy
    img[(d2 >= int(-40 * s)) & (d2 < int(-32 * s)) & (yy > int(58 * s))] = 240

    # smooth horizontal gradient in the bottom strip -- should yield no edges
    top = int(112 * s)
    img[top:, :] = np.clip(40 + (xx[top:, :] * 100 // size), 0, 255).astype(np.uint8)

    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--size", type=int, default=128)
    ap.add_argument("--outdir", default=".")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    img = make(args.size)
    flat = img.reshape(-1)

    with open(os.path.join(args.outdir, "test.bin"), "wb") as f:
        f.write(flat.tobytes())
    with open(os.path.join(args.outdir, "test.hex"), "w") as f:
        f.write("".join(f"{v:02x}\n" for v in flat))
    Image.fromarray(img).save(os.path.join(args.outdir, "test.png"))

    print(f"wrote {args.size}x{args.size} test pattern ({flat.size} bytes) to {args.outdir}")
    print("  square @ top-left, circle @ top-right, 45/135 bars @ middle, gradient @ bottom")


if __name__ == "__main__":
    main()
