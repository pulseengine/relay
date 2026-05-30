# Roadmap — to perfect, formally-proven autonomous flight (v0.20 → v1.0)

Status: approved direction (2026-05-30). Supersedes the ad-hoc v0.19.x
stepping-stone plan. Authoritative source for the falcon cascade's
technical direction.

## The mission framing

PulseEngine's question is *"AI writes the code. Who proves it's safe?"*
This roadmap answers it for flight: reach **frontier flight capability**
(PX4-class autonomy, then Swift-class agility) while being the only stack
that **formally proves** the result — Kani + Verus + Lean + Rocq +
witness MC/DC + sigil attestation, end to end.

Two destinations, fused in the chosen order **certified floor → agile
ceiling**:
- **Certified autonomy** (v0.20–v0.27): provable PX4-class capability —
  full nav, position-hold, missions, fault tolerance, dossier.
- **Verified agility** (v0.28+): a learned/NMPC agile policy flying
  *inside a formally-verified runtime-assurance shield* that falls back
  to the Lyapunov-certified geometric controller. The literal answer to
  "who proves the AI is safe?"

## Honest starting point (v0.19.9)

The cascade is, architecturally, a **~2010-era stack missing its entire
navigation half**:

- **Estimation** — `relay-ekf` is a Mahony *complementary filter*,
  **attitude only**. `position_ned`/`velocity_ned` are hardcoded zero
  (`crates/relay-ekf/plain/src/lib.rs:95-98`). It trusts the
  accelerometer as a gravity reference — true only at rest, which IS
  root cause RC#3.
- **Control** — `relay-pos` (cascaded P-PI) and `relay-att`
  (quaternion-error P) both use **small-angle linearization** (<~30°).
- **Why the bench "hovers"** — it feeds the controllers Gazebo's
  ground-truth position. The aircraft cannot yet estimate where it is.
- **Verified** — proptest everywhere; Kani on `relay-mix-quad` (MIX-P05)
  and `relay-arm` (ARM-P01). Lean (RATE-P03 Lyapunov) and Rocq (ATT-P01)
  proofs are deferred — this roadmap cashes them.

So "make it fly perfectly" is **not** a tuning problem. It needs the
navigation half built, the controllers moved off small-angle onto
geometric SE(3), and the whole thing proven.

## State of the art we are targeting (2024–2026)

| Layer | Frontier reference | Why it's the target |
|---|---|---|
| Estimation | **Invariant EKF (IEKF)** on SE₂(3) — consistency-guaranteed Lie-group filter. PX4 **EKF2** = 24-state error-state EKF (att+pos+vel+biases+mag+wind, delayed-fusion horizon, multi-hypothesis yaw GSF). | IEKF leapfrogs EKF2 (2015-era): the invariant error is *group-affine* (state-independent error dynamics) → a clean geometric object Lean/Rocq prove well, sharing the SE(3) foundation with the controller. |
| Attitude/pos control | **Geometric SE(3) control** (Lee, Leok, McClamroch 2010) — almost-global, Lyapunov-provable. | Replaces small-angle linearization; the almost-global Lyapunov proof is the Lean track's crown jewel. |
| Inner loop / robustness | **INDI** (Incremental Nonlinear Dynamic Inversion) — sensor-based, model-light, robust; the basis of modern aggressive + fault-tolerant flight. | Rejects model uncertainty; enables rotor-loss survival. |
| Allocation | Pseudo-inverse + **sequential desaturation / active-set**, reconfigurable under rotor loss. | Replaces the static 4×4 matrix; handles saturation + faults + arbitrary airframes. |
| Agility | **Learned policy** (Swift, *Nature* 2023 — champion-level drone racing via deep RL, collective-thrust + body-rate interface) / NMPC. | The ceiling. Wrapped in a verified shield, not trusted bare. |
| Verification frontier | SMT-verified neural-Lyapunov certificates, reachability (ReachNN). | We go further: formal proofs (Lean/Kani/Verus), not SMT-on-NN alone. |

Sources: PX4 EKF2 architecture (DeepWiki / PX4 docs / PR #22262);
Invariant EKF (Barrau & Bonnabel; arXiv:2404.10665 IterIEKF; J. Field
Robotics 2025 Mars-quadrotor IEKF); Geometric SE(3) (arXiv:1003.2005,
Lee/Leok/McClamroch); INDI fault-tolerant (arXiv:2002.07837, Sun et al.);
Swift (Nature s41586-023-06419-4); neural-Lyapunov certification
(arXiv:2503.04129).

## Decisions locked (2026-05-30)

- **Estimator (v0.21): Invariant EKF (IEKF) — leapfrog.** Accept the
  research risk for the consistency guarantee + the shared SE(3)
  verification foundation. Not the pragmatic error-state EKF.
- **North star: both, sequenced** — certified floor (v0.20–27) then
  agile ceiling (v0.28+), culminating in a v1.0 that is both and fully
  attested.

## The 10-release climb

Each release pairs a capability jump with the SOTA tech AND its
verification gate (defense-in-depth) AND a falsification statement.

### v0.20 — Full closed-loop position-hold
- **Capability:** the full cascade holds a *position* (not just
  attitude+altitude) in gz. Closes the SIMULATOR "hover" scenario.
- **Tech:** fix `relay-pos` hover-thrust normalisation (default 0.5 ≠ the
  2 kg body's 0.72); differential-flatness feed-forward.
- **Gate:** gz hover ±0.5 m / 30 s, 4/4 reproducible; rivet POS artifact.
- **Falsify:** if the loop diverges, the position-bound assertion trips.
- **Risk:** low. *(Note: still uses gz-truth position — v0.21 removes
  that crutch.)*

### v0.21 — ★ Full-state Invariant EKF (the keystone)
- **Capability:** onboard estimation of position + velocity + attitude +
  IMU biases, fusing IMU + GPS + baro. Removes the gz-truth crutch.
  **Kills RC#3** (proper predict/correct gravity handling).
- **Tech:** Invariant EKF on SE₂(3) (group-affine error dynamics).
- **Gate:** proptest + a **consistency proof** (the invariant error is a
  Lie-group symmetry); NEES/consistency bench vs gz truth; defense-in-
  depth Kani on the bounded update.
- **Falsify:** if NEES leaves its χ² envelope, the filter is
  inconsistent — fail.
- **Risk:** HIGH (research-shaped). The crux release.

### v0.22 — Magnetometer + multi-hypothesis yaw
- **Capability:** drift-free heading; recover yaw without mag (GPS-only).
- **Tech:** mag fusion + Gaussian-Sum-Filter multi-hypothesis yaw.
- **Gate:** observability oracle; yaw-drift bound over a long bench.
- **Risk:** medium.

### v0.23 — Geometric SE(3) control + Lean Lyapunov proof
- **Capability:** almost-global attitude/position tracking; large-angle
  manoeuvres the small-angle stack can't do.
- **Tech:** geometric control on SE(3) (Lee 2010).
- **Gate:** **Lean almost-global Lyapunov stability theorem** via
  `rules_lean4` — finally discharges the deferred RATE-P03; Rocq ATT-P01.
- **Falsify:** publish the basin of attraction; if a trajectory inside
  the proven basin diverges in sim, the model/proof is wrong.
- **Risk:** medium-high.

### v0.24 — Control allocation
- **Capability:** correct saturation behaviour; any airframe geometry.
- **Tech:** pseudo-inverse + sequential desaturation / active-set.
- **Gate:** Kani bounds (MIX-P05-style) on the allocator output set.
- **Risk:** low-medium.

### v0.25 — INDI inner loop
- **Capability:** robustness to model error / mass change; aggressive
  flight.
- **Tech:** Incremental Nonlinear Dynamic Inversion (sensor-based).
- **Gate:** robustness-margin proptest; disturbance-rejection bench.
- **Risk:** medium.

### v0.26 — Fault-tolerant control (rotor loss)
- **Capability:** survive single / double-opposing rotor failure.
- **Tech:** INDI fault-tolerant + reconfigurable allocation.
- **Gate:** reachability/safety oracle; the relay-lc/sc fault chain.
  Large dossier value (DO-178C / ISO-26262 fault cases).
- **Risk:** high.

### v0.27 — Trajectory generation + all SIMULATOR scenarios
- **Capability:** missions, 5 m step (<2 s, <20% overshoot), 10 m/s gust
  rejection — **all four SIMULATOR.md scenarios green**, with RTL.
- **Tech:** differential-flatness / minimum-snap trajectories.
- **Gate:** the four scenario verdicts in the verification gate.
- **Risk:** medium. *This is the "certified autonomy floor" complete.*

### v0.28 — Learned/NMPC agility inside a verified shield
- **Capability:** Swift-class agile flight.
- **Tech:** RL policy (collective-thrust + body-rate) or NMPC, **wrapped
  in a verified simplex / runtime-assurance shield**: if the policy
  leaves the proven safe set, the v0.23 Lyapunov-certified geometric
  controller takes over.
- **Gate:** formal proof of the shield's safe-set invariance + the
  certified fallback. **The moat** — AI agility with a proven floor.
- **Risk:** high.

### v1.0 — Perfectly fly, formally proven
- **Capability:** the full picture — certified autonomy + verified
  agility, hex/coax airframe variants.
- **Tech:** all of the above, fused.
- **Gate:** the complete stack — Kani + Verus + Lean + Rocq + witness
  MC/DC + sigil attestation — and the six-domain credit dossier.

## Sequencing principles

1. **Estimation before control.** A geometric SE(3) controller on a
   filter that can't localise is pointless. v0.21 (IEKF) is the keystone;
   everything above is gated on it. v0.20 is a quick win on gz-truth to
   keep momentum while v0.21 is built.
2. **Provable spine, learned tip.** The geometric controller (v0.23) is
   the verified backbone with an almost-global Lyapunov proof. The
   learned/NMPC layer (v0.28) sits *inside a shield that falls back to
   that spine*. This inverts industry order (learn first, certify never)
   into PulseEngine order (prove the floor, then let AI push the ceiling).
3. **One geometry, two uses.** IEKF and SE(3) control share the same
   Lie-group foundation — the estimator's consistency proof and the
   controller's stability proof rest on the same verified math.
