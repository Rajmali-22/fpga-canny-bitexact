# RTL simulation regression

Runs `canny_core` in Icarus Verilog on 7 test cases and compares every stage
(blur, magnitude, NMS, threshold, final) and the hysteresis iteration count
against `model/canny_model.py`. No board needed.

```bash
python sim/run_regression.py
```

Needs `iverilog` and `vvp` on PATH (OSS CAD Suite includes both), plus numpy and pillow.
Exits with status 1 on any mismatch, so it can run in CI.
