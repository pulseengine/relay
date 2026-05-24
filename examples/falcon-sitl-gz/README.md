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

# Gazebo stub — verdict will be FAIL until the bench wire-up is done
cargo run -p falcon-sitl-gz -- --backend=gazebo --world=falcon --model=quad
```

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

A minimal `falcon-quad.sdf` needs:

* a `model` of a quadrotor (use `gz-sim-models/x500` as a baseline).
* an **IMU sensor plugin** on the body link, publishing to
  `/world/falcon/model/quad/link/imu_link/sensor/imu_sensor/imu`.
* a **NavSat (GPS) sensor plugin**, publishing to
  `/.../sensor/gps_sensor/navsat`.
* four **MulticopterMotorModel plugins** on the rotor joints,
  subscribing to `/world/falcon/model/quad/joint/<rotor_n>/cmd_vel`.

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

## Why scaffold, not full implementation?

`gz-transport` is C++ + Protobuf; Rust bindings are immature. A
real bridge is ~one bench-day of work the user does. The scaffold
+ the verified cascade together are what's deliverable in software
(no hardware/sim install required). Pattern matches v0.11's
`HackRfBench` (RF stub + real-bench docs) and v0.12's
`MavlinkBench`'s `--peer=` (round-trip wired in software; PX4 +
gz-sim are user's bench).
