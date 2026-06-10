# Simulator landscape — "would our stuff actually fly, and how"

This document answers a recurring question: **what evidence do we
have, and what evidence could we get, that falcon's verified
control cascade actually flies?**

The honest short answer: falcon flies *today* in our own pure-Rust
SITL, and falcon's *safety chain* integrates with PX4 + Gazebo
today via the v0.14.0/v0.14.2 HITL harness. What's *missing* is
falcon-as-the-flight-controller inside a real-physics simulator
(Gazebo Sim or similar). The gap is bounded engineering, not a
new verification frontier.

## What we have today (v0.14.3)

### 1. Pure-Rust closed-loop SITL — `examples/falcon-sitl-hover`

**Falcon IS the flight controller.** The cascade runs against a
toy 6-DoF rigid-body integrator (`relay-ekf-stub` + the cascade
crates). 17 scenarios pass:

| Scenario     | What it proves                                            |
|--------------|-----------------------------------------------------------|
| `step`       | EKF tracks a position step                                |
| `disturbance`| Cascade rejects wind / push                               |
| `hover`      | Body holds attitude + position under noisy IMU            |
| `attitude`   | Rate loop tracks attitude setpoints                       |
| `mission`    | Waypoint sequence completes                               |
| `fault`      | EkfHealthMonitor trips, relay-sc RTL fires, body lands    |
| `untethered` | Network ID beacon emits while flying                      |
| `geofence`   | Position breach trips relay-lc, RTL recovers              |

This **IS** "would our stuff fly". Loose physics, but the
verified cascade closes the loop and stabilises a simulated body
under disturbance + fault injection.

Falsifiable claim: *if the cascade did not converge, every
scenario's pass assertion (`!nan_seen`, position-bound check,
RTL-dispatched flag) would trip*. They don't.

### 2. PX4-SITL × MavlinkBench — `examples/falcon-hitl-rfspoof` + `docs/px4-sitl-bench.md`

**PX4 IS the flight controller; falcon watches + commands.**
PX4-SITL (with jMAVSim or Gazebo Sim) emits `GLOBAL_POSITION_INT`
on UDP 14550 → falcon's `MavlinkBench` decodes → verified
`relay-lc::Geofence::check` runs on PX4's position → on breach,
relay-sc dispatches RTL → harness encodes `COMMAND_LONG` with
`MAV_CMD_NAV_RETURN_TO_LAUNCH` and writes it back to PX4 → PX4
flies home.

This proves the **safety chain** integrates with a real, mature
flight stack against real 3D physics. It does NOT prove the
cascade IS the flight stack.

One command from the recipe in `docs/px4-sitl-bench.md`:
```bash
cargo run -p falcon-hitl-rfspoof -- --preset=px4-sitl
```

## The gap — falcon AS the flight controller, inside real physics

Neither of the above answers the strict question "does falcon's
*entire* cascade make a real-physics quad fly?"

To close that gap we need: **Gazebo Sim ↔ falcon-cascade bridge**.

```
        ┌──────────────────────────────────┐
        │ Gazebo Sim (gz-sim)              │
        │  • 6-DoF rigid-body physics      │
        │  • IMU + GPS + barometer sensors │
        │  • Motor + propeller plugins     │
        └────────────┬─────────────────────┘
                     │  IMU/GPS frames @ 200 Hz / 10 Hz
                     ▼
        ┌──────────────────────────────────┐
        │ falcon-cascade (the brain)       │
        │   relay-ekf  → relay-pos →       │
        │   relay-att  → relay-rate →      │
        │   relay-mix-quad                 │
        └────────────┬─────────────────────┘
                     │  4× motor PWM @ 200 Hz
                     ▼
        ┌──────────────────────────────────┐
        │ Gazebo motor plugin → physics    │
        └──────────────────────────────────┘
```

Concrete shape:

* **New crate** `examples/falcon-sitl-gz` (or `falcon-gz-bridge`)
  — thin Rust binary that:
  - Connects to gz-sim's transport (Gazebo's Protobuf + ZeroMQ
    `gz-transport` library; Rust bindings exist via the
    `gz-msgs` crate family or via direct UDP/Protobuf).
  - Subscribes to `/world/<world>/model/<quad>/link/imu_link/
    sensor/imu_sensor/imu` and `/.../sensor/gps_sensor/navsat`.
  - Calls the verified cascade tick (one `relay-ekf` step +
    `relay-pos` + `relay-att` + `relay-rate` + `relay-mix-quad`).
  - Publishes motor commands to `/world/<world>/model/<quad>/
    joint/<rotor_n>_joint/cmd_vel`.

* **Verdict shape**: a small set of scenarios mirroring the
  SITL ones (hover stability, step response, waypoint mission)
  measured in Gazebo physics — RMS position drift, settling
  time, peak overshoot, mission completion.

* **What "flies" means in this context**:
  - **Hover** — body holds within ±0.5 m of setpoint for ≥ 30 s
    under Gazebo's default wind noise.
  - **Step response** — settling time < 2 s, overshoot < 20 % on
    a 5 m position step.
  - **Mission** — falcon-sitl-hover's mission waypoint sequence
    completes inside a 100 m fence with the same RTL behaviour
    if the geofence trips.
  - **Disturbance** — a Gazebo-scripted wind gust (10 m/s for
    2 s) doesn't tumble the body.

Honest scope: the bridge is **maybe one PR of engineering** (the
verified safety + control code is all in place); the visual
verdict ("here's a video of the quad hovering") is a *user-bench*
step because it needs a display + gz-sim install.

## Why not just put falcon inside PX4 as an "external EKF"?

PX4 supports an external state estimator via the `VISION_POSITION_
ESTIMATE` MAVLink message. We *could* feed `relay-ekf` output
into PX4 and keep PX4's nav stack downstream. That's a partial
falcon-as-FC story — proves the EKF flies, but leaves the position
/ attitude / rate loops as PX4's. Less complete than the gz-sim
bridge, but it's incremental (could be a v0.15.x stepping stone
to the full gz-sim bridge).

## The decision

Three concrete next-step options, roughly ordered by deliverable
size:

| Option | Scope | What it proves |
|---|---|---|
| **A. SITL Gazebo bridge** (`examples/falcon-sitl-gz`) | Medium PR | Full cascade flies under real physics |
| **B. PX4 external-EKF integration** | Small PR | Verified EKF flies in a real autopilot context |
| **C. Add Gazebo backend to `falcon-sitl-hover`** | Medium PR | Same scenarios, real physics — drop-in upgrade |

Option C is interesting because it preserves the existing 17
scenarios' *contract* (same assertions, same RTL behaviour) but
swaps the toy physics for Gazebo's — direct "does the same flight
behaviour hold under real physics?" answer.

## What this document does NOT claim

- That a real flight-controller dev sign-off needs less than this.
  v1.0 is a long way past "the simulator passes" — it's
  "FAA/EASA-credible test campaigns + bench + flight test".
- That gz-sim's IMU + GPS noise models match field hardware.
  They're close enough for control-loop validation but not for
  EKF noise-tuning.
- That the cascade's `relay-mix-quad` output (per-motor PWM in
  [0, 1]) maps directly onto Gazebo's motor command range — a
  small linear scaling step lives in the bridge.

## Where v0.15.x stands on this

v0.15.0 ships:
  - FV-FALCON-ARCH-002 — spar codegen recheck (`--format wit`
    now works; documents the abstraction divergence between
    spar-AADL-derived WIT and the hand-authored cascade WIT).
  - This document.

v0.15.1 candidate (per the v0.14 deferred list):
  - Witness `--harness` runner binary (wasmtime + WASI) so the
    cascade-target coverage closes.

v0.15.2 / v0.16 candidate (per the question this document
addresses):
  - Option A / B / C above — user picks the size.

The bridge is software the verified-stack pattern is *ready to
absorb*. None of the existing safety properties change; the bridge
just gives Gazebo physics the same input/output interface the
toy SITL has today.
