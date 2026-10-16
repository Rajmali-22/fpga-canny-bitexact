#!/usr/bin/env python3
"""
fpga_link.py -- send an image to the FPGA over UART, get the processed image back.

This is the whole PC side of the system. It cuts the full-resolution image into
overlapping 128x128 tiles, sends each one, reads the result back, drops the
halo, and stitches everything into a single PNG on your PC.

Protocol per tile (see rtl/top.v):
    PC   -> FPGA :  A5 5A  CMD  W_lo W_hi  H_lo H_hi  T_LOW T_HIGH  <16384 bytes>
    FPGA -> PC   :  5A A5  <16384 bytes>

Typical use
-----------
    # 1. link check -- send one tile, ask for it straight back, compare
    python fpga_link.py --port COM3 --selftest

    # 2. one tile through the full pipeline, fast to iterate on
    python fpga_link.py ../image.bmp --port COM3 --tile 9

    # 3. the whole image (24 tiles, ~70 s at 115200)
    python fpga_link.py ../image.bmp --port COM3

    # 4. if the result looks wrong, dump an intermediate stage instead
    python fpga_link.py ../image.bmp --port COM3 --tile 9 --cmd 4   # magnitude

CMD values: 1 final, 2 input echo, 3 blur, 4 magnitude, 5 nms, 6 threshold
"""

import argparse
import os
import sys
import time

import numpy as np
import serial
from PIL import Image

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import tile_canny as T

TILE = 128
NPIX = TILE * TILE
REQ_MAGIC = bytes([0xA5, 0x5A])
RSP_MAGIC = bytes([0x5A, 0xA5])

CMD_NAMES = {1: "final", 2: "echo", 3: "blur", 4: "mag", 5: "nms", 6: "thresh"}


def build_request(cmd, t_low, t_high, pixels):
    return (REQ_MAGIC
            + bytes([cmd,
                     TILE & 0xFF, (TILE >> 8) & 0xFF,
                     TILE & 0xFF, (TILE >> 8) & 0xFF,
                     t_low, t_high])
            + pixels)


def read_exact(ser, n, what):
    """Read exactly n bytes or raise with something actually diagnosable."""
    buf = bytearray()
    while len(buf) < n:
        chunk = ser.read(n - len(buf))
        if not chunk:
            raise TimeoutError(
                f"timed out waiting for {what}: got {len(buf)} of {n} bytes.\n"
                f"  - is the bitstream programmed?\n"
                f"  - are uart_rxd/uart_txd swapped in canny.xdc?\n"
                f"  - does the baud rate here match the BAUD parameter in top.v?")
        buf += chunk
    return bytes(buf)


def sync_response(ser):
    """Hunt for 5A A5. Lets us recover instead of silently returning shifted data."""
    window = bytearray()
    deadline = time.time() + 30.0
    while time.time() < deadline:
        b = ser.read(1)
        if not b:
            continue
        window += b
        if len(window) > 2:
            del window[0]
        if bytes(window) == RSP_MAGIC:
            return
    raise TimeoutError(
        "never saw the 5A A5 response magic.\n"
        "  - check led[3] on the board: if lit, the FPGA rejected the frame size\n"
        "  - check led[0..2] to see which phase it is stuck in")


def do_tile(ser, cmd, t_low, t_high, tile):
    ser.reset_input_buffer()
    ser.write(build_request(cmd, t_low, t_high, tile.reshape(-1).tobytes()))
    ser.flush()
    sync_response(ser)
    raw = read_exact(ser, NPIX, "tile payload")
    return np.frombuffer(raw, dtype=np.uint8).reshape(TILE, TILE)


def cmd_selftest(ser, t_low, t_high):
    """CMD=2 asks the FPGA to return mem_in untouched. Any difference is a link fault."""
    print("link self-test: sending a known pattern, asking for it back verbatim")
    rng = np.arange(NPIX, dtype=np.uint32)
    pattern = ((rng * 37 + (rng >> 7) * 11) & 0xFF).astype(np.uint8).reshape(TILE, TILE)

    t0 = time.time()
    got = do_tile(ser, 2, t_low, t_high, pattern)
    dt = time.time() - t0

    bad = int((got != pattern).sum())
    print(f"  round trip: {2 * NPIX} bytes in {dt:.2f} s "
          f"({2 * NPIX / dt / 1024:.1f} KB/s)")
    if bad == 0:
        print("  PASS -- every byte matched. The UART link is good.")
        return 0
    first = int(np.argmax((got != pattern).reshape(-1)))
    print(f"  FAIL -- {bad} of {NPIX} bytes differ, first at index {first} "
          f"(sent 0x{pattern.reshape(-1)[first]:02x}, got 0x{got.reshape(-1)[first]:02x})")
    print("  A small number of scattered errors means baud mismatch or noise.")
    print("  Everything wrong from one index onward means a dropped/extra byte.")
    return 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", nargs="?", help="source image (omit for --selftest)")
    ap.add_argument("--port", default="COM3")
    ap.add_argument("--baud", type=int, default=2000000,
                    help="must match the BAUD parameter in top.v (default 2000000). "
                         "Use --baud 115200 if the board still has the old bitstream.")
    ap.add_argument("--halo", type=int, default=4)
    ap.add_argument("--tlow", type=int, default=50)
    ap.add_argument("--thigh", type=int, default=100)
    ap.add_argument("--cmd", type=int, default=1, choices=sorted(CMD_NAMES))
    ap.add_argument("--tile", type=int, default=None,
                    help="process only this tile index (fast bring-up)")
    ap.add_argument("--outdir", default="../data")
    ap.add_argument("--selftest", action="store_true",
                    help="UART echo test only, no image needed")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=5.0, write_timeout=30.0)
    except serial.SerialException as e:
        sys.exit(f"error: cannot open {args.port}: {e}\n"
                 f"  Check Device Manager > Ports (COM & LPT). Close any other\n"
                 f"  terminal that has the port open.")

    with ser:
        time.sleep(0.2)
        ser.reset_input_buffer()

        if args.selftest:
            sys.exit(cmd_selftest(ser, args.tlow, args.thigh))

        if not args.image:
            sys.exit("error: an image is required unless --selftest is given")
        if not os.path.isfile(args.image):
            sys.exit(f"error: no such file: {args.image}")

        img = T.load_gray(args.image)
        core = TILE - 2 * args.halo
        ncols, nrows, _ = T.grid(img.shape[1], img.shape[0], core)
        all_tiles = list(T.tiles_of(img, core, args.halo))

        wanted = all_tiles if args.tile is None else [all_tiles[args.tile]]
        os.makedirs(args.outdir, exist_ok=True)

        print(f"image   : {args.image}  {img.shape[1]}x{img.shape[0]}")
        print(f"tiling  : CORE={core} HALO={args.halo} -> {ncols} x {nrows} = "
              f"{len(all_tiles)} tiles")
        print(f"link    : {args.port} @ {args.baud} baud")
        print(f"cmd     : {args.cmd} ({CMD_NAMES[args.cmd]})   "
              f"thresholds {args.tlow}/{args.thigh}")
        print()

        results, t0 = [], time.time()
        for n, (i, x0, y0, tile) in enumerate(wanted, 1):
            ts = time.time()
            res = do_tile(ser, args.cmd, args.tlow, args.thigh, tile)
            results.append(res)
            done = time.time() - t0
            eta = done / n * (len(wanted) - n)
            print(f"  tile {i:3d}  ({x0:4d},{y0:4d})  {time.time() - ts:5.2f} s"
                  f"   [{n}/{len(wanted)}]  eta {eta:5.1f} s")

        print()
        tag = CMD_NAMES[args.cmd]

        if args.tile is not None:
            out = os.path.join(args.outdir, f"fpga_tile{args.tile}_{tag}.png")
            Image.fromarray(results[0]).save(out)
            print(f"wrote   : {out}  (full 128x128 tile including halo)")
        else:
            stitched = T.stitch(img.shape, core, args.halo, results)
            out = os.path.join(args.outdir, f"fpga_{tag}.png")
            Image.fromarray(stitched).save(out)
            print(f"wrote   : {out}  {img.shape[1]}x{img.shape[0]}")
            if args.cmd == 1:
                print(f"edges   : {int((stitched > 0).sum())} px "
                      f"({100.0 * (stitched > 0).mean():.2f}%)")

        print(f"total   : {time.time() - t0:.1f} s")


if __name__ == "__main__":
    main()
