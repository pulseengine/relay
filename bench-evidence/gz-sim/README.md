# bench-evidence/gz-sim/

Bench-evidence captured against a real running `gz sim` Harmonic
instance, driving the falcon-cascade through `falcon-sitl-gz`'s
gz-transport bridge (v0.18.0+).

The companion to `bench-evidence/px4-sitl/` from v0.18.2 — same
shape of evidence, different question:

| Directory | What it proves |
|---|---|
| `px4-sitl/`  | Verified **safety override** flies in PX4-SITL — `relay-lc::Geofence` + `relay-sc::CommandStore` command RTL on a real flight controller. |
| `gz-sim/`    | The **complete cascade** (`relay-ekf` + `relay-pos` + `relay-att` + `relay-rate` + `relay-mix-quad`) flies as the FC under real Gazebo Harmonic physics. |

## Producing fresh evidence

From a clean checkout with `gz-harmonic` installed:

```bash
# Terminal 1 — start gz-sim with the falcon-quad world.
gz sim -r -v3 examples/falcon-sitl-gz/worlds/falcon-quad.sdf

# Terminal 2 — bridge the cascade in, write evidence under this dir.
cargo run -p falcon-sitl-gz --features gazebo --release -- \
  --backend=gazebo --world=falcon --model=quad \
  --home=47.3977,8.5456,488 \
  --scenario=hover --duration=30 \
  --evidence-dir=bench-evidence/gz-sim
```

The runner produces two files per run:

| File | Content |
|---|---|
| `<ts>-gazebo-<scenario>-harness.log` | Verdict + steps + climb metrics + diagnostic counters (`imu_recv`, `navsat_recv`, `motor_send`). |
| `<ts>-gazebo-<scenario>-ticks.csv`   | One row per 10 ms tick: position NED, IMU body-frame accel/gyro, motor PWM, running counters. |

`<ts>` is the Unix-epoch second at run start, matching the
`bench-evidence/px4-sitl/` naming convention.

## Reading a verdict

`imu_recv > 0` confirms gz is publishing the IMU topic and the bridge
is parsing it; `navsat_recv > 0` confirms NavSat → Home → NED is
working; `motor_send > 0` confirms the bridge is publishing rotor
commands. If `motor_send > 0` but the body doesn't climb, the SDF's
`<motorConstant>` needs tuning — the bridge wiring is fine.

The PASS criterion (`net_climb > 0.1 m`) is a *liveness check*, not
a flight-quality benchmark. The full falsifiable criteria from
`docs/SIMULATOR.md` (hover ±0.5 m for 30 s, step settling < 2 s,
mission completion, wind disturbance) become separate scenarios
(`--scenario=step|mission|disturbance`) in v0.19.x.

## What's currently here

This directory is **empty** at v0.19.0 — the software side ships
(SDF world + bridge wiring + bench-evidence sink), but a real
`gz sim` run is the bench step. v0.19.x lands the first actual
flight evidence once the gz install lands and a hover scenario
records a green verdict.
