# falcon-sitl-hover

**falcon v0.3 example — pure-Rust closed-loop SITL: EKF + rate PID stabilize the body.**

Closes the inner control loop in-process: a rigid-body plant
integrates dynamics, the v0.2 [`relay-ekf`](../../crates/relay-ekf/)
estimates attitude, the v0.3 [`relay-rate`](../../crates/relay-rate/)
PID drives commanded body rates, motors close the loop. No Gazebo,
no MAVLink — runs anywhere `cargo` runs, deterministic given a seed.

> Why pure-Rust SITL? Gazebo Harmonic + MAVLink lockstep is the v0.4
> path (where full attitude/position control makes a real hover
> meaningful). For v0.3 a pure-Rust plant lets the rate loop be
> verified in CI without a 1 GB Gazebo install. Both stay in tree;
> the pure-Rust harness keeps running as the fast-iteration test
> rig even after Gazebo lands.

## Run

```sh
cargo run -p falcon-sitl-hover --release
```

Three scenarios run in series:

```
--- scenario: step ---
  samples              5000
  final ω (rad/s)      [+0.5007, -0.3003, +0.4006]
  peak ω above sp      0.0061 rad/s
  overshoot            1.2 %
  RMS error (steady)   0.0012 rad/s
  convergence time     0.139s
  loop wall time       393 µs
  outcome              PASS

--- scenario: disturbance ---
  recovery time        0.141s after impulse
  outcome              PASS

--- scenario: hover ---
  convergence time     0.175s
  outcome              PASS

falcon-sitl-hover: PASS
```

Run a single scenario:

```sh
cargo run -p falcon-sitl-hover --release -- --scenario step
cargo run -p falcon-sitl-hover --release -- --scenario disturbance
cargo run -p falcon-sitl-hover --release -- --scenario hover
```

With noise on the IMU:

```sh
cargo run -p falcon-sitl-hover --release -- --noise 0.05
```

`--noise σ` adds Gaussian-ish noise to both accel (scale σ m/s²) and
gyro (σ × 0.1 rad/s).

## Scenarios

### step

Commanded setpoint `[0.5, -0.3, 0.4] rad/s` applied at t=0 to a
vehicle at rest. Verifies the rate loop drives the body to track.

**Pass criterion:** convergence to `|error| ≤ 0.05 rad/s` within
1.0 s, overshoot ≤ 30 %, final error ≤ 0.02 rad/s, no NaN/∞.

### disturbance

Vehicle is held at zero rates for 1 s, then a `+1 rad/s` impulse is
slammed into the y-axis. Verifies the rate loop drives the
disturbance back to zero.

**Pass criterion:** rate magnitude back to ≤ 0.05 rad/s within
0.5 s of the impulse.

### hover

Vehicle starts with non-zero rates `[0.7, -0.5, 0.3]` rad/s and
setpoint zero. Tests the "panic stabilize" behaviour — operator
arms with the airframe already tumbling, controller settles it.

**Pass criterion:** rates settle to `|ω| ≤ 0.02 rad/s` within 1.0 s.

## Plant

Rigid-body rotational dynamics on a small quadcopter:

```text
ω_dot = (τ − μ * ω) / I
q_dot = ½ q ⊗ (0, ω)
```

with `I = 5 g·m²` (500 g, 10-inch quad), `μ = 0.001 N·m·s/rad`
friction. Quaternion is renormalised each tick to prevent drift.
The plant in [`src/main.rs`](src/main.rs:60) is ~50 lines and
deliberately simple — this is a controller test rig, not a
full-fidelity simulator.

## What's tested

```sh
cargo test -p relay-rate          # 13 PID unit + proptest cases
cargo test -p falcon-sitl-hover   # 5 closed-loop scenarios
```

The relay-rate tests cover (v0.3 surrogates for SWREQ-FALCON-RATE-P*):

- **RATE-P01**: integral state bounded ±i_max under arbitrary inputs
  (deterministic + proptest with 256+ default cases, 4096 in fuzz mode)
- **RATE-P02**: no NaN/∞ from NaN/∞/extreme inputs
- **RATE-P03 (surrogate)**: step response on pure-integrator plant
  converges within 2 s with <0.01 rad/s steady-state error
- **RATE-P04 (surrogate)**: tick wall-time empirically ≤ 1 µs

The closed-loop bench validates the **cascade**: EKF + rate PID
working against a physically plausible plant. Failures in either
component (an EKF NaN, a PID windup, a torque-saturation race) show
up here as missed convergence budgets.

## What this is NOT

- **Not Gazebo SITL** — v0.4 hookup.
- **Not flight-controlled** — no attitude or position controller
  yet; the rate loop alone won't keep a vehicle level under wind.
- **Not yet AOT-compiled** — runs as a host binary. The relay-rate
  crate IS no_std and ready for synth→gale, but this binary uses std
  for I/O.
- **Not signed/in a WASM bundle** — sigil signing of the bench
  happens with the v0.6 hardware bring-up release.

## Files

```
examples/falcon-sitl-hover/
├── Cargo.toml
├── README.md            (this file)
└── src/
    └── main.rs          (plant + scenarios + CLI + tests)
```

Depends on:

```
crates/relay-ekf/        (v0.2 Mahony complementary filter)
crates/relay-rate/       (v0.3 body-rate PID with anti-windup)
```

## Falcon release table

This example ships with **v0.3**. See [`falcon/README.md`](../../docs/falcon.md)
for the full v0.1 → v1.0 plan.

Tracked at [`artifacts/features/FEAT-FALCON-rollout.yaml`](../../artifacts/features/FEAT-FALCON-rollout.yaml).
