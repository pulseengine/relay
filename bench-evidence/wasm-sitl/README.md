# wasm-SITL evidence — v1.137

Traces behind `relvids/falcon-v1.137-wasm-tick-rate.mp4`. Every pixel in that
video is plotted from these CSVs; nothing in it is drawn by hand.

## How they were produced

```
cd tests/cascade-sitl-wasm

# after — the host declares its real period
TRACE_CSV=after-250hz.csv  GNSS_DIV=0 cargo run --release -- /tmp/composed-v137.wasm 2500 0.004

# before — the host declares 1 kHz while running at 250 Hz, which is exactly
# what v0.7 did unconditionally, so this reproduces the old defect through the
# new interface rather than requiring an old build
DECLARED_DT=0.001 TRACE_CSV=before-250hz.csv GNSS_DIV=0 \
  cargo run --release -- /tmp/composed-v137.wasm 2500 0.004
```

Columns: `t_s,altitude_m,commanded_m`. Commanded altitude is 2.0 m throughout.

## Final altitude, 10 s, commanded 2.0 m

| host rate | v0.7 (declared 1 kHz) | v0.8 (declares its own) |
|---|---|---|
| 250 Hz | 29.17 m | **2.00 m** |
| 400 Hz | 12.43 m | **2.00 m** |
| 1000 Hz | 2.00 m | 2.00 m |

The 1000 Hz row is the reason this survived to v1.136: at the one rate v0.7
assumed, the two builds are indistinguishable.

## Superseded as a FIX, kept as a RECORD

These traces measure the v0.7 five-stage cascade, whose per-stage hardcoded
clocks this evidence demonstrates. #393 replaced that component with one wrapping
falcon-core's FlightCore, which has a single clock and shows no rate dependence
at all (0.22 m at 100, 250 and 1000 Hz). So the defect these CSVs document is
gone by a different route than the one the video narrates.

They are kept because they are the measurement that found it, and because the
video was rendered from exactly these numbers.

## What this evidence does NOT cover

`GNSS_DIV=0` — no position fix is supplied in these runs, and IMU noise is 0.
That isolates the tick-rate defect. It says nothing about the separate
IMU-only limitation tracked in #380, which is NOT fixed in v1.137: with the
position fix supplied and fused, the composed cascade still destabilises above
roughly 0.07 m/s² accelerometer noise, where the native cascade holds 0.39 m
at 0.2. Those numbers are in the issue, not in this video.
