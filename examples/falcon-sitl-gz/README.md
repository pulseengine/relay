# `falcon-sitl-gz` — Gazebo SITL backend scaffold

Per [`docs/SIMULATOR.md`](../../docs/SIMULATOR.md) **option C**:
same SITL contract `examples/falcon-sitl-hover` exercises, but with
a **pluggable `Physics` trait** so a real Gazebo Sim bridge can
drop in without touching the verified cascade or its assertions.

## What ships

| File             | What it is                                              |
|------------------|---------------------------------------------------------|
| `src/physics.rs` | `Physics` trait + `MockPhysics` reference impl + `GazeboPhysics` stub |
| `src/main.rs`    | CLI runner with one demonstration scenario (hover)      |
| `Cargo.toml`     | Reuses the verified cascade crates (relay-ekf, relay-pos, relay-att, relay-rate, relay-mix-quad, relay-lc, relay-sc) |

The `Physics` trait is **two methods**: `step(motor_pwm, dt)` and
`measure(noise_std) -> (ImuSample, position_ned)`. Verified path is
unchanged — same pattern as the HITL harness's `HitlBench` and
`FrameSource` traits.

## Running

```bash
# in-process MockPhysics (default; toy 6-DoF integrator)
cargo run -p falcon-sitl-gz

# Gazebo STUB — no bridge yet; verdict will be FAIL.
cargo run -p falcon-sitl-gz -- --backend=gazebo --world=falcon --model=quad

# Real Gazebo bridge (v0.18.0; needs `gz sim` running).
cargo run -p falcon-sitl-gz --features gazebo -- \
  --backend=gazebo --world=falcon --model=quad
```

### v0.18 — real `gz-transport-rs` bridge

Behind the `gazebo` feature, `GazeboPhysics` is a real
[gz-transport-rs](https://crates.io/crates/gz-transport-rs)-backed
implementation:

- Subscribes to `gz.msgs.IMU` on
  `/world/{world}/model/{model}/link/base_link/sensor/imu_sensor/imu`
- **v0.18.1** — subscribes to `gz.msgs.NavSat` on
  `/world/{world}/model/{model}/link/base_link/sensor/navsat_sensor/navsat`
  for GPS-derived position. Use `--home=lat,lon,alt_m` to set the
  launch-site anchor for the equirectangular lat/lon → NED
  projection (same shape as `MavlinkBench::Home` from v0.12).
  Without `--home`, defaults to `Home::ORIGIN` (lat=0, lon=0,
  alt=0) — almost certainly not what you want for a real bench run.
- Publishes `gz.msgs.Double` to each of four rotor joints'
  `/world/{world}/model/{model}/joint/rotor_{0..3}_joint/cmd_vel`
- Converts gz-sim ENU body frame ↔ falcon NED body frame at the
  measure boundary (`(x, y, z)_ned = (x, -y, -z)_enu`)
- PWM → motor RPM via `pwm * 1000 rad/s` (MulticopterMotorModel-
  compatible; the exact constant depends on your SDF model — adjust
  in `physics.rs::pwm_to_rad_per_s`)

Full bench command (v0.19.0 — with evidence capture):

```bash
# Terminal 1 — start gz-sim with the bundled world.
gz sim -r -v3 examples/falcon-sitl-gz/worlds/falcon-quad.sdf

# Terminal 2 — bridge the cascade in, write evidence under bench-evidence/.
cargo run -p falcon-sitl-gz --features gazebo --release -- \
  --backend=gazebo --world=falcon --model=quad \
  --home=47.3977,8.5456,488 --duration=30 \
  --scenario=hover \
  --evidence-dir=bench-evidence/gz-sim
```

Produces two files per run under `--evidence-dir`:

| File | Content |
|---|---|
| `<ts>-gazebo-<scenario>-harness.log` | Verdict + steps + climb metrics + diagnostic counters (`imu_recv`, `navsat_recv`, `motor_send`). |
| `<ts>-gazebo-<scenario>-ticks.csv`   | One row per 10 ms tick: position NED, IMU body accel/gyro, motor PWM, running counters. |

Same shape as `bench-evidence/px4-sitl/` from v0.18.2.

**Default cargo builds do NOT include this.** `gz-transport-rs` pulls
in tokio + libzmq (compiled from C source via `zeromq-src`),
~30-60 s extra build time. Opt in only when you have a bench.

## What this is NOT

* A working Gazebo loop. The `GazeboPhysics` impl is a stub —
  `step()` prints "stub" and `measure()` returns zeros. The verdict
  prints `FAIL`, which is the *correct* signal that the bench
  wire-up is required.
* A replacement for `examples/falcon-sitl-hover`. The existing 17
  scenarios stay there; this crate is the scaffold for the next
  layer of "would our stuff fly" evidence.

## Bench wire-up recipe (the user step)

To make this real on a bench:

### 1. Install Gazebo Sim

```bash
# macOS via Homebrew
brew tap osrf/simulation
brew install gz-harmonic    # Ignition / Gazebo Sim Harmonic LTS

# Ubuntu 24.04
sudo apt-get install gz-harmonic
```

### 2. Author an SDF world

**v0.19.0 ships a starting world: `worlds/falcon-quad.sdf`.**
Model = 700 g, ~250 mm wheelbase, 4 × `MulticopterMotorModel` with
~5× hover-thrust headroom, world anchored to PX4-SITL's stock home
(47.3977°N, 8.5456°E, 488 m AMSL). Bench-tune `<motorConstant>` /
`<maxRotVelocity>` if the quad is over- or under-powered.

The minimum the bundled world covers (and the bridge requires):

* a `model` of a quadrotor.
* an **IMU sensor plugin** on the body link, publishing to
  `/world/falcon/model/quad/link/base_link/sensor/imu_sensor/imu`.
* a **NavSat (GPS) sensor plugin**, publishing to
  `/world/falcon/model/quad/link/base_link/sensor/navsat_sensor/navsat`.
* four **MulticopterMotorModel plugins** on the rotor joints,
  subscribing to `/world/falcon/model/quad/joint/rotor_{0..3}_joint/cmd_vel`.

### 3. Replace the stub bodies in `src/physics.rs`

`GazeboPhysics::step()` and `GazeboPhysics::measure()` both have
`TODO(bench):` comments naming the topics + the conversion. The
bridge needs:

* **gz-transport Rust bindings**. Two options:
  - Use [`gz-transport-rs`](https://github.com/gazebosim/gz-transport-rs)
    (if available + maintained at bench-time).
  - Generate Protobuf bindings from `gz/msgs/*.proto` directly via
    `prost-build`, then wrap them in a thin wasm-transport client.
* **Frame conversion**. Gazebo's body frame is X-forward Y-left
  Z-up (ENU world); falcon uses X-forward Y-right Z-down (NED).
  Same conversion the MavlinkBench's `Home::project_ned_cm`
  equirectangular step does for lat/lon → NED.
* **Time alignment**. Use Gazebo's clock topic
  (`/world/<world>/clock`) so `Timestamp` matches the sim clock,
  not wall time.

### 4. Verify

The PASS criterion the scaffold uses (`net_climb > 0.1 m`) is
deliberately easy — the goal is to prove the loop closes. Real
acceptance criteria are in `docs/SIMULATOR.md` (hover within
±0.5 m for 30 s, step settling < 2 s + overshoot < 20 %, mission
completion under wind, geofence-trip latency).

## v0.16.1 stub → v0.18.0 real bridge

v0.16.1 shipped this as a stub (the `GazeboPhysics` `step()` and
`measure()` were TODOs that printed warnings and returned zeros).
v0.18.0 promotes it to a real bridge behind the `gazebo` feature
flag, using `gz-transport-rs` (pure-Rust Gazebo transport, no C++
gz-sim install required at build time).

The pattern matches v0.11's `HackRfBench` (RF stub + real-bench
docs) and v0.12's `MavlinkBench`'s `--peer=` (round-trip wired in
software; PX4 + gz-sim are user's bench):

- **Without `--features gazebo`** — stub stays for users who just
  want the scaffold contract; build is lean (no gz-transport deps).
- **With `--features gazebo`** — real bridge ships, builds against
  gz-transport-rs + tokio + zmq, ready to talk to a running
  `gz sim`. The bench step is still installing gz-sim + authoring
  the SDF world, but the Rust side is now real code.
