# falcon — video shot guide

The story: **every layer of an autonomous drone, formally proven — flying a
real mission in physics.** The flight footage is genuine (real Gazebo
physics, real control, no scripting); the *proofs* are the star.

Honest framing — say it out loud, it's a strength: the mission is now **crisp**
(returns home <1 m) since the v0.35 adaptive process-noise fix and the v1.16
position-loop integral; the earlier "recognizable but not razor-crisp" caveat
is retired. What's also crisp: the altitude hold, the estimator consistency,
the fault recovery, the realism robustness (wind/drag/sensor-noise, v1.16+),
and the **machine-checked proofs**.

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
Generate from any flight's ticks CSV: `./tools/plot_flight.py <ticks.csv> out.png`
— a top-down East–North path coloured by time + waypoints + **home error
<1 m**, and the **crisp altitude hold**. (Headless 3D capture that works on
macOS — no GUI/screen-record needed: `./scripts/record_headless.sh markers
mission 55`.)

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

- A **formally-verified flight stack**, v0.21 → v1.21, each layer with a
  **mechanical gate** (estimator, control, allocation, FSM, failsafes,
  realism robustness).
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
- gz has run-to-run variance — a fraction of runs drift or diverge (the
  marginal-yaw/mag instability); prefer the best-of-N take (`record_headless.sh
  markers` does this automatically), re-run if a run looks off.
- It is **SITL (Gazebo) + proof + emulation**, not yet hardware/HIL — the
  honest gap matrix is in `docs/dossier/v1.0-practical-readiness.md`. The
  remaining gaps are on the hardware side of the abstraction seam.
