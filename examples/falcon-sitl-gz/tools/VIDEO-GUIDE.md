# falcon — video shot guide

The story: **every layer of an autonomous drone, formally proven — flying a
real mission in physics.** The flight footage is genuine (real Gazebo
physics, real control, no scripting); the *proofs* are the star.

Honest framing — say it out loud, it's a strength: the mission tracking is
*recognizable but not razor-crisp* (the inner loop is marginally stable —
v0.32+ work). What IS crisp: the altitude hold, the estimator consistency,
the fault recovery, and the **machine-checked proofs**.

---

## Shots

### 1. See the drone fly the mission (the hero, screen-record the GUI)
```bash
./examples/falcon-sitl-gz/tools/watch_mission.sh mission 55
```
A Gazebo window opens with the quadrotor + world; it arms, climbs to 2 m,
flies a 2 m square (corner-by-corner, holds heading), returns home. Record
the window. (Re-run if a run looks off — gz has run-to-run variance.)

### 2. The data view (overlay / cut-away)
Already rendered in `/tmp/falcon-video/`:
- `01-mission-path.png` — top-down East–North path coloured by time +
  waypoints + **home error ~0.2 m**, and the **crisp altitude hold**.
Regenerate from any flight: `./tools/plot_flight.py <ticks.csv> out.png`.

### 3. Fault tolerance — rotor loss (data / terminal)
```bash
cargo test -p falcon-sitl-gz fault_tolerance -- --nocapture
```
"Kill a rotor mid-flight → the FDI isolates it → the Kani-verified allocator
reconfigures (relinquishes yaw) → the body settles upright." Pair with the
Kani receipt:
```bash
cargo kani -p relay-mix-quad --harness verify_mix_rotor_out_bound | tail -3
#  → VERIFICATION:- SUCCESSFUL
```

### 4. The simplex shield (the moat — data / terminal)
```bash
cargo test -p falcon-sitl-gz shield_keeps -- --nocapture
cargo kani -p relay-geo --harness verify_shield_contract | tail -3
#  → VERIFICATION:- SUCCESSFUL
```
"An unverified agile policy tries to tumble the drone; the certified shield
contains it inside the proven safe set."

### 5. The proof receipts (on-screen text — the payoff)
```bash
cargo test -p relay-iekf -p relay-geo -p relay-mix-quad -p relay-traj 2>&1 | grep 'test result'
rivet validate | tail -1                 #  Result: PASS
# Lean (kernel-checked Lyapunov): bazel build //proofs/lean:geometric_lyapunov
```

---

## Narration facts (all true, all checkable)

- A **formally-verified flight stack**, v0.21 → v0.31, each layer with a
  **mechanical gate**.
- **Kani (bounded model checking) proofs, re-run green:** the control
  allocator + its single-rotor-out reconfiguration (MIX-P08) + the rotor
  fault detector + the **simplex safety shield**.
- **Lean — a *kernel-checked* Lyapunov stability proof** for the geometric
  SE(3) controller (`V̇ = −k_Ω‖ω‖² ≤ 0`, plus the LaSalle precondition).
- **Invariant-EKF** estimator with an online **NEES consistency monitor**
  and **acceleration-compensated tilt** (keeps heading honest under motion).
- **Trajectory:** Mueller jerk-minimizing quintics + the **differential-
  flatness feedforward** into the controller.
- **Fault tolerance:** detect → isolate → reconfigure → reduced-attitude
  recovery, all gated.
- **The moat:** a learned/agile policy flies *inside a verified shield* that
  falls back to the proven-stable controller at the recoverable-set boundary.
- Flying it: a real **waypoint mission in Gazebo Harmonic**, returns home to
  ~0.2 m, rock-steady altitude — on the same `no_std`/`no_alloc` flight code.

## Honest caveats (own them on camera)
- Horizontal tracking is recognizable, not razor-crisp (inner-loop margin —
  v0.32+). Altitude/estimator/proofs are crisp.
- Hover has a ~1-in-5 yaw-bistability bad run (a known, documented gap) —
  prefer the mission for the live shot, re-run if needed.
- It is SITL (Gazebo), not yet hardware/HIL.
