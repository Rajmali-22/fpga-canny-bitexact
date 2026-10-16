#!/usr/bin/env python3
"""
run_regression.py -- RTL simulation regression for canny_core.

Compiles fpga/rtl with Icarus Verilog, runs sim/tb_core.v on a set of test images,
and compares EVERY stage (blur, magnitude, NMS, threshold, final) against
model/canny_model.py. Exits non-zero on any mismatch.

Usage (from the repo root or anywhere):
    python sim/run_regression.py
Needs: iverilog + vvp on PATH (e.g. from OSS CAD Suite), numpy, pillow.
"""
import os, shutil, subprocess, sys, tempfile, time
import numpy as np
from PIL import Image

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for _d in ("model", "host"):          # works before and after the planned move to model/
    sys.path.insert(0, os.path.join(ROOT, _d))
import canny_model as cm   # noqa: E402
import mktest              # noqa: E402

N = 128
RTL = os.path.join(ROOT, "fpga", "rtl")
SOURCES = ["canny_core.v", "window3x3.v", "dpram.v", "op_gauss.v", "op_sobel.v",
           "op_nms.v", "op_thresh.v", "op_hyst.v"]
STAGES = ("blur", "mag", "nms", "cls", "final")


def write_hex(path, a):
    with open(path, "w") as f:
        f.write("".join(f"{v:02x}\n" for v in a.reshape(-1)))


def read_hex(path):
    toks = []
    for line in open(path):
        toks += line.split("//")[0].split()
    return np.array([int(t, 16) for t in toks], dtype=np.uint8).reshape(N, N)


def cases():
    pat = mktest.make(N)
    src = np.array(Image.open(os.path.join(ROOT, "image.bmp")).convert("L"))
    h, w = src.shape
    mid = src[h // 2 - 64:h // 2 + 64, w // 2 - 64:w // 2 + 64]
    rng = np.random.default_rng(1)
    return [("pattern_25_60", pat, 25, 60),
            ("pattern_50_100", pat, 50, 100),
            ("photo_topleft_20_40", src[0:N, 0:N], 20, 40),
            ("photo_middle_20_40", mid, 20, 40),
            ("photo_middle_50_100", mid, 50, 100),
            ("noise_20_40", rng.integers(0, 256, (N, N), dtype=np.uint8), 20, 40),
            ("flat_255_20_40", np.full((N, N), 255, np.uint8), 20, 40)]


def main():
    for tool in ("iverilog", "vvp"):
        if not shutil.which(tool):
            sys.exit(f"error: {tool} not found on PATH")
    work = tempfile.mkdtemp(prefix="canny_sim_")
    vvp_file = os.path.join(work, "tb_core.vvp")
    here = os.path.dirname(os.path.abspath(__file__))
    subprocess.run(["iverilog", "-g2005", "-o", vvp_file, os.path.join(here, "tb_core.v")]
                   + [os.path.join(RTL, s) for s in SOURCES], check=True)

    print(f"{'case':22s} " + " ".join(f"{s:>5s}" for s in STAGES)
          + f"  {'iters rtl/model':>15s}  {'compute ms @50MHz':>17s}")
    failed = False
    for name, img, tl, th in cases():
        img = np.ascontiguousarray(img, dtype=np.uint8)
        inp = os.path.join(work, f"in_{name}.hex")
        write_hex(inp, img)
        t0 = time.time()
        r = subprocess.run(["vvp", "-n", vvp_file, f"+IN={inp}", f"+OUT={os.path.join(work, name)}",
                            f"+TL={tl}", f"+TH={th}"], capture_output=True, text=True, cwd=work)
        res = [l for l in r.stdout.splitlines() if l.startswith("RESULT")]
        if not res:
            print(f"{name:22s} SIMULATION FAILED\n{r.stdout[-800:]}\n{r.stderr[-800:]}")
            failed = True
            continue
        kv = dict(p.split("=") for p in res[0].split()[1:])
        gold = cm.run(img, tl, th)
        diffs = [int((read_hex(os.path.join(work, f"{name}_{s}.hex")) != gold[s]).sum()) for s in STAGES]
        iters_ok = int(kv["hyst_iters"]) == gold["hyst_iters"]
        failed |= any(diffs) or not iters_ok
        print(f"{name:22s} " + " ".join(f"{d:5d}" for d in diffs)
              + f"  {kv['hyst_iters']:>7s}/{gold['hyst_iters']:<7d}  {int(kv['cycles']) / 50e3:17.2f}"
              + f"   ({time.time() - t0:.0f} s)")
    shutil.rmtree(work, ignore_errors=True)
    print("FAIL" if failed else "PASS -- RTL is bit-exact with canny_model.py on every stage")
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
