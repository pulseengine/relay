# Where we are vs PX4 — parity assessment (2026-06-11, at falcon-v1.56.0)

A statement of the gap between the falcon/relay flight stack and PX4 (and the
commercial field), and the work still to cover. Supersedes the v1.44.0 edition,
which *planned* the v1.45→v1.54 capability round — that round, plus v1.55
(CI sovereignty) and v1.56 (the Wasm component family), has now shipped.

## The one-line statement

> falcon is **capability-complete in simulation with a formally-verified,
> security-hardened core**; PX4 is **production-ready with millions of real
> flight-hours**. The remaining gap is *not* features and *not* the control
> mathematics — it is **flight-proven maturity**: real drivers on silicon,
> per-airframe calibration and tuning, on-MCU WCET measurement, and first
> flight. That is the next round. We lead on verification and security; we trail,
> entirely, on flight hours.

## What shipped since v1.44 (the capability-parity round, now done)

Every PX4 capability gap the v1.44 edition listed as "buildable without hardware"
now has a verified crate behind it — each present in-tree, each gated by a
mechanical oracle (Kani / KAT / SITL test) before its release:

| PX4 capability | Our crate (shipped) | Release |
|---|---|---|
| Mag + baro driver bodies (mock-bus) | `falcon-baromag` | v1.45 |
| Manual / Stabilized / Acro RC modes | `relay-rc` | v1.46 |
| MAVLink parameter system + schema | `relay-param` | v1.47 |
| Onboard black-box log + replay | `relay-log` | v1.48 |
| Airframe variants (hex / coax) | `relay-mix-multi` | v1.49 |
| Sensor redundancy / voting / GPS-loss fallback | `relay-sensvote` | v1.50 |
| Rangefinder + optical-flow fusion | `relay-flowrange` | v1.51 |
| Mode completeness (follow-me, land-in-place, RTL override) | `relay-modextra` | v1.52 |
| Pre-flight built-in-test + arming-check gate | `relay-preflight` | v1.53 |
| Component-world fusion (meld) + trace reconciliation | (v1.54 capstone) | v1.54 |
| CI sovereignty: self-hosted runners + Kani roll-up gate | (infra) | v1.55 |
| Wasm component family as a versioned release bundle | `scripts/build-components.sh` | v1.56 |

The trace moved with the build: at v1.44 rivet read 113 `approved` / 57
`implemented`; it now carries ~180 sw-reqs with `implemented` roughly doubled,
backed by 128 sw-verification + 32 unit-verification artifacts and a 10-hazard
STPA-Sec security analysis.

## How much of PX4 do we cover?

**The wrong axis is "feature %"; the right axis is two separate scores.**

- **Capability surface (features that exist and pass their tests in SITL):**
  broadly at parity — estimator, cascade control, all flight modes, three
  airframe classes, params, logging, failsafes, pre-flight checks, sensor
  voting, flow/range fusion, authenticated C2. Call it ~95% of the *flyable
  feature set*.
- **Flight-proven maturity:** ~0%. PX4's real mass is ~15 years × thousands of
  airframes × millions of flight-hours of field-hardening, a driver ecosystem
  for hundreds of boards/sensors/ESCs, and calibration/tuning baked in from
  experience. We have **zero real flights**, ~4 real driver bodies (the rest are
  mock-bus seams), and no on-silicon calibration or WCET *measurement* (the WCET
  *proof* exists; cycle counts on a real MCU do not).

So: **most of the capability, none of the maturity.** The second axis is the
entire hardware round (below), and it is the honest gap — not a number to inflate.

## Where we genuinely lead PX4 — and the commercial PX4 derivatives

Not "more features" — a different correctness basis:

- **Machine-checked proofs.** Kernel-verified Lean Lyapunov proof for the
  geometric SE(3) controller (0 sorry / 0 axiom), Kani BMC on loop/safety
  invariants, a compositional WCET proof. PX4's EKF2 and controllers are neither
  verified nor proof-carrying — **and neither are the commercial derivatives**
  (Auterion/Skynode, ModalAI/VOXL inherit PX4's unverified core).
- **Built-in authenticated C2** (`relay-sec`): anti-replay + Ascon-AEAD + X25519
  session keys with forward secrecy + rekey-on-reboot. PX4 has **no native
  authenticated command link** (MAVLink signing is optional and weak). Most
  commercial stacks add security at the radio/link layer, not in the flight stack.
- **Traceable MBSE as infrastructure** (spar→rivet→witness→sigil): every
  requirement traced to architecture → code → MC/DC truth-table → signed
  attestation. This is the thing certified-avionics shops spend person-years
  building by hand.

The thesis: **"PX4's capability, made provable and secure."**

## The commercial comparison (honest)

| | falcon/relay | PX4 (+Auterion/ModalAI) | DJI | Certified avionics (DO-178C DAL-A) |
|---|---|---|---|---|
| Control sophistication | frontier (IEKF/SE(3)/ADRC) | solid, conventional | excellent (closed) | conservative, proven |
| Formal verification | **machine-checked proofs** | none | none | testing/process (DO-333 formal methods rarely applied) |
| Built-in C2 security | **yes, verified** | minimal | proprietary | mission-specific |
| Flight-proven maturity | **none** | enormous | enormous | enormous, certified |
| Hardware/driver ecosystem | minimal | huge | vertically integrated | per-platform |
| Actually certified | dossier *scaffold* only | no | no | **yes — the bar** |

The defensible claim: falcon is the only entry whose *correctness is
proof-carrying*. Even DO-178C DAL-A — the gold standard — certifies by exhaustive
testing and process rigor, not machine-checked proofs. So we hold a verification
property nobody in this table has — while holding **none** of PX4's flight hours.
Both halves are true; stating only the first is the over-claim to avoid.

## The hardware boundary — the next round (v1.57+)

Deliberately deferred. The 7-item register that stands between "provably correct
in simulation" and "flown":

1. Real sensor/actuator driver **bodies on silicon** (beyond the 4 we have).
2. Sensor **calibration** (accel/gyro/mag/baro) on a real board.
3. Per-airframe **flight tuning** (the math is proven; gains for a real frame are not).
4. WCET **leaf measurement** (proof done; cycle counts on the target MCU not).
5. Physical **HITL transport** (UART/USB/UDP) end-to-end.
6. **libm qualification** (transcendental math on the flight path; centralize behind a math seam first).
7. **First flight** — the irreducible field test no proof substitutes for.

Plus the integration project: meld→loom→synth→gale onto the board (the v1.56
component bundle is the input to this hand-off). Tracked as the hardware-round
epic; each register item is an assignable slice others can pick up.

## The line not to cross

A machine-checked Lyapunov proof guarantees the *math* is right. It says nothing
about whether a real ESC browns out at 6S or an IMU saturates under vibration.
We are **research-ready and provably-correct in simulation, not flight-proven.**
Every external statement of where we stand must carry both clauses.
