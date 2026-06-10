# Where we are vs PX4 — parity assessment (2026-06-10, at falcon-v1.44.0)

A statement of the gap between the falcon/relay flight stack and PX4, and the
work still to cover. Written to drive the v1.45→v1.54 release round — ten
releases of software/SITL/verification work that close PX4 *capability* gaps
**before** any physical-hardware work begins.

## The one-line statement

> falcon is **research-ready and SITL-complete with a formally-verified core**;
> PX4 is **production-ready with thousands of real flights**. The remaining gap
> is *not* the control mathematics (ours is frontier SOTA + machine-checked
> proofs PX4 has no equivalent of) — it is **software breadth** (manual modes,
> parameters, logging, airframe variants, sensor redundancy) and, separately,
> **the hardware boundary** (real drivers on silicon, tuning, calibration, first
> flight). This round closes the software breadth; the hardware boundary is the
> round after.

## Where we genuinely lead PX4

- **Formally-verified cascade**: IEKF on SE₂(3), geometric SE(3) control with a
  *kernel-checked Lean Lyapunov proof* (0 sorry/0 axiom), ADRC, a simplex-shield
  safety envelope, compositional WCET proof. PX4's EKF2/controllers are neither
  verified nor proof-carrying.
- **Verified authenticated C2 link** (relay-sec v1.35–v1.37): anti-replay +
  Ascon-AEAD + X25519 session keys with forward secrecy + rekey-on-reboot. PX4
  has **no built-in** authenticated command link.
- **Verification rigor**: Kani BMC on loop/safety invariants, Lean/Rocq proofs,
  proptest, independent clean-room audits, an STPA-Sec hazard analysis, a
  six-domain certification dossier scaffold.

This is the "parity, but provable" thesis: match PX4's capability, win on
verification + security.

## Where PX4 leads us — the capability gap (buildable WITHOUT hardware)

These are real PX4 features we lack, and **none require physical hardware** —
they are no_std/SITL/codec work, exactly this round's scope:

| Gap | PX4 has | We have | Closes in |
|---|---|---|---|
| Manual / Stabilized / Acro modes (RC stick → rate/attitude) | yes | offboard/MAVLink only | v1.46 |
| Parameter system over MAVLink (PARAM_*) + schema | yes | relay-tbl stores values, no MAVLink loop / no min-max-enum | v1.47 |
| Onboard flight log / black box + replay | ulog | relay-ds gates packets, no archival | v1.48 |
| Airframe variants (hex / coax) | yes | quad mixer only | v1.49 |
| Sensor redundancy / voting / GPS-loss fallback | yes | single IMU/GPS, no voting | v1.50 |
| Rangefinder + optical-flow fusion | yes | baro/GNSS only (precision-land controller exists, no flow) | v1.51 |
| Mode completeness (guided per-wp yaw, follow-me, land-in-place, RTL alt override) | yes | mission + fixed loiter/RTL | v1.52 |
| Pre-flight built-in-test + arming checks | yes | arm gate is level+airborne only | v1.53 |
| Magnetometer + barometer driver bodies | yes | GNSS done (v1.42); mag/baro are seams only | v1.45 |
| Component-world fusion (meld) + trace reconciliation | n/a (our architecture) | WORLD-P01 unbuilt; ~56 reqs built-but-unbumped | v1.54 |

## Where PX4 leads us — the hardware boundary (the NEXT round, not this one)

Deliberately deferred to after v1.54 (the readiness dossier's 7-item register):
real sensor/actuator driver bodies **on silicon**, sensor calibration, flight
tuning per airframe, WCET leaf *measurement* (proof is done, cycle counts
aren't), physical HITL transport (UART/USB/UDP), libm qualification, and **first
flight**. Plus the separate integration project (meld→loom→synth→gale onto the
board). These are v1.55+ (hardware bring-up) and the certification capstone.

## Honest note on the rivet trace

rivet currently shows **113 `sw-req` approved vs 57 implemented**, which *reads*
as "half-built." That is misleading: most of the 113 are **built-but-unbumped**
(code + tests + proofs exist; the status was never advanced from `approved` to
`implemented`, exactly as the relay-sec reqs were until bumped). A handful are
genuinely unbuilt (WORLD-P01/P02 fusion + variants, PAYLOAD-P01, MISSION-GDSS
geofence-upload). v1.54 reconciles the status so the trace reflects reality.

## The plan: v1.45 → v1.54 (software/SITL parity, no hardware)

1. **v1.45** mag + baro driver bodies (finish the F6 sensor trio; mock-bus)
2. **v1.46** RC input + manual flight modes (Stabilized / Acro)
3. **v1.47** MAVLink parameter system + schema validation
4. **v1.48** onboard flight logger (black box) + replay
5. **v1.49** airframe variants: hexacopter + coaxial mixers
6. **v1.50** sensor redundancy: dual-IMU voting + GPS-loss INS fallback
7. **v1.51** rangefinder + optical-flow fusion into the IEKF
8. **v1.52** flight-mode completeness (guided yaw, follow-me, land-in-place, RTL override)
9. **v1.53** pre-flight built-in-test + arming-check gate
10. **v1.54** consolidation capstone: meld world fusion + lock/FV/EKF hygiene + rivet status reconciliation = the hardware-ready gate

Then — and only then — **v1.55 hardware bring-up** and **v1.56 the beyond-PX4
certification capstone**.

Every release keeps the discipline: a mechanical oracle (Kani/KAT/SITL test),
honest limits published, no_std/no_alloc on the flight path, traceability
leading, released via `release-execution` (a tag, not just a merge).
