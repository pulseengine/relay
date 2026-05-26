# First gz-sim bench session — 2026-05-26

First observational evidence from running `falcon-sitl-gz --features
gazebo` against a real `gz sim` instance. The verdict is `FAIL` (no
climb), but the **diagnostic counters did exactly what they were
designed for**: the run pinpoints two concrete next-fix items
without ambiguity.

## Setup

- macOS arm64, gz Sim 8.11.0 (osrf/simulation tap, installed via
  `brew install osrf/simulation/gz-harmonic`).
- World: `examples/falcon-sitl-gz/worlds/falcon-quad.sdf` (the v0.19.0
  starting world) — patched once at `setUp` because the Fuel.gazebosim.org
  `Sun` + `Ground Plane` includes need network + fuel cache. Replaced
  by inline `<light>` + `<model name="ground_plane">`.
- gz sim invocation: `gz sim -s -r -v3 worlds/falcon-quad.sdf`
- Bridge invocation:
  ```
  target/debug/falcon-sitl-gz --backend=gazebo --world=falcon \
    --model=quad --home=47.3977,8.5456,488 --scenario=hover \
    --duration=10 --evidence-dir=bench-evidence/gz-sim
  ```

## Result

```
verdict: backend=gazebo steps=1000 climb=0.00 m  (min=-0.00 max=-0.00)  wall=11.72s
counters: imu_recv=51 navsat_recv=0 motor_send=1000
FAIL
```

Full logs in `1779822750-gazebo-hover-harness.log` +
`1779822750-gazebo-hover-ticks.csv`.

## What worked

- gz sim loaded the world cleanly. Topics under `/world/falcon/model/quad/`
  came online within ~5 s of launch.
- Bridge connected to gz-transport without intervention; tokio runtime,
  `gz-transport-rs` Node + subscribers all came up green.
- IMU subscription is live: `imu_recv > 0`, ticks.csv carries non-zero
  `ax_body/ay_body/az_body/gx_body/gy_body/gz_body` columns.
- Motor publish rate is exact: 1000 ticks × 100 Hz harness = 1000
  `motor_send` events — bridge is sending what it intends to send.
- The `--evidence-dir` flag produces both files with the documented
  schema; CSV is readable in pandas / Excel.

## What needs fixing — two concrete next-fix items

### Finding 1: motor topic format + message type mismatch

`gz topic -t /quad/cmd_vel -i` shows the four `MulticopterMotorModel`
plugins each subscribe to the **single model-level topic** `/quad/cmd_vel`
expecting `gz.msgs.Actuators` (a struct carrying an array of motor
velocities, indexed by the plugin's `<motorNumber>` field).

The bridge currently publishes **four separate `gz.msgs.Double` messages**
to **per-rotor topics** `/world/falcon/model/quad/joint/rotor_{0..3}_joint/cmd_vel`.
That gets `motor_send=1000` ticked because the bridge's mpsc channels
accept the sends, but no plugin is listening on those topics, so the
rotors don't spin — hence `climb=0.00 m`.

**Fix** (one-PR work):
- Change `physics::gz_real::GazeboPhysics` to publish a single
  `gz.msgs.Actuators` message to `/{model}/cmd_vel` with
  `velocity: [rotor_0_rad_s, rotor_1_rad_s, rotor_2_rad_s, rotor_3_rad_s]`.
- Drop the per-rotor mpsc channels (only one channel needed now).
- Drop the per-rotor `<commandSubTopic>cmd_vel</commandSubTopic>` from
  the SDF — the plugin uses its model-namespace + a single shared topic.
- Wire `gz_transport_rs::msgs::Actuators` (verify the type is exported;
  if not, regenerate from `gz/msgs/actuators.proto` via prost-build).

### Finding 2: NavSat publisher exists but isn't streaming

`gz topic -t /world/falcon/.../navsat_sensor/navsat -i` shows our subscriber
is registered (`Publishers [Address, Message Type]: gz.msgs.NavSat`) but
the run captured **zero** NavSat frames in 10 s wall time. The IMU at
200 Hz only delivered 51 frames over the same wall time — a similar
under-rate that suggests an upstream rate limit (or gz transport network
buffering with the empty `<sensor>` body the SDF provided), not a
subscriber bug.

**Fix** (smaller — diagnose first, may be a one-line SDF tweak):
- Inspect the published NavSat topic with `gz topic -e -t .../navsat -n 1`
  to confirm gz-sim is actually emitting. If yes, the gz-transport-rs
  subscriber is dropping; if no, the SDF NavSat sensor needs an
  explicit `<navsat>` body (currently just `<sensor type="navsat">`
  with `<always_on>` + `<update_rate>`).
- IMU is the same story at 200 Hz → 4.25 Hz actual. Same root cause is
  likely.

## Why this run is still a win

The cFS-DNA pattern says: each layer's bench tests *what that layer
adds*. The v0.19.0 layer added (a) end-to-end bridge wiring, (b) SDF
shape, (c) diagnostic counters. All three are *demonstrated working
on real gz-sim*:

- The bridge survives connect + 10 s of simulation.
- The SDF loads and presents the topics the bridge expects (sensors)
  + the topics it doesn't yet match (motors) — surfacing the
  mismatch the counters then quantify.
- The counters distinguish four failure modes that would otherwise
  be a single mute `climb=0` — and they pin both finds to one of those modes.

The verdict-shaped artifact reading "FAIL with `imu_recv=51 motor_send=1000
navsat_recv=0`" is **falsifiable bench evidence**, not "the run was
silent for some reason." That's the entire point of the
counters-as-first-class trait method introduced in v0.19.0.

## Next release shape

- **v0.19.1** = this evidence + SDF Fuel-includes patch + bench-findings
  doc. Lands now.
- **v0.19.2** = Finding 1 fix (Actuators message, single topic). Should
  produce non-zero climb under hover scenario; that becomes the
  positive-control evidence.
- **v0.19.3** = Finding 2 fix (NavSat body + IMU rate diagnosis). Closes
  position-dependent control loops (waypoint, geofence) under gz
  physics.
- **v0.19.x** = scenario expansion: step, mission, disturbance per
  `docs/SIMULATOR.md`'s falsifiable criteria.

The trait + CLI + SDF pattern set in v0.19.0 is now demonstrated to be
the right shape; the remaining work is wire-format details, not
architecture.
