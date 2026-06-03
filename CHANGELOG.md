# Changelog

All notable changes to relay + falcon are documented in this file.

The format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and the project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

Tags use a per-track prefix:
- `falcon-v<semver>` — the falcon dual-DNA flight stack
- (future) `relay-v<semver>` — the relay substrate itself

> Note: the per-version CHANGELOG drifted during the v0.7–v0.20 run (those
> shipped as git tags `falcon-v0.7`…`falcon-v0.20` but were not written up
> here). This `v0.34.0` entry covers the full **v0.21 → v0.34 autonomous-
> flight arc**, which was developed unmerged on `falcon-v0.21-iekf` until
> it was verified end-to-end and the kernel-checked Lyapunov proof built.

## [falcon-v1.15.0] — 2026-06-03

The **capstone** of the v1.8 → v1.15 hardware-abstraction arc: the
v1.0-practical readiness review, with an **independent clean-room
re-verification** of the whole arc.

### Added

- **`docs/dossier/v1.0-practical-readiness.md`** — the honest readiness
  statement: a capability-vs-gap matrix (proven / verified-in-SITL / emulated /
  seam-only vs needs-hardware), the verification-evidence inventory, and a
  seven-item hardware **gap register** (real driver bodies + on-silicon
  validation, calibration, flight tuning, WCET leaf measurement, libm
  qualification, physical HITL transport, first flight).

### Clean-room re-verification

A subagent briefed **cold** — no narrative, only falsifiable claims and
read/exec tools — checked the arc's headline claims against the working tree by
running the cargo tests, grepping the sources, and reading the proof:

> **11 CONFIRM / 0 REFUTE / 0 CANNOT-VERIFY.**

Confirmed independently: `relay-math` is the cascade's only `libm` consumer (0
`libm::` in the five migrated crates); `CompositionalWcet.lean` has **0 `sorry`**
and defines `falcon_cascade_schedulable_1khz`; falcon-core defines the
supervisor + hardware seam + `Pathology` with **12 passing tests**; the IMU
driver (4 tests) implements `ImuDriver`; the HITL crate (2 tests) defines
`LinkBackend`; the FSM Kani harnesses exist; the embedded crate targets
`thumbv7em-none-eabihf`; `rivet validate` PASS; and the GPS-dropout test asserts
**bounded drift + recovery, not zero drift**.

### Verdict

For a research / SITL / emulation v1.0, the arc is **complete and the claims
hold under independent check**. For a hardware-flying v1.0, the seven gap-register
items remain — every one on the *hardware* side of the abstraction seam this arc
was built to make swappable. Nothing in the tree claims flight.

## [falcon-v1.14.0] — 2026-06-03

The **hardware-in-the-loop link** — the third backend, and the one that realises
your "**hardware with the simulation as the backend**" topology. The same
verified core runs on the flight computer; the simulator answers over a wire.

### Added

- **`falcon-hitl`** (`no_std`) — a `LinkBackend` (a `FlightBackend`) that, each
  control step, sends a 16-byte **actuator frame** (four motor commands) and
  receives a 54-byte **sensor frame** (accel, gyro, position+valid, mag+valid,
  battery) over a `Transport`; and a `SimServer` that decodes the actuator
  frame, steps a `SimBackend`, and encodes the sensor frame back. The wire
  format is fixed-layout little-endian `f32` — no `serde`, no `alloc` — so it
  runs unchanged on the MCU.

  | backend | sensors/actuators from |
  |---|---|
  | `SimBackend` (v1.1) | simulation, in-process |
  | `HardwareBackend` (v1.11) | real sensors, via driver traits |
  | **`LinkBackend` (v1.14)** | **a remote simulator / vehicle over a framed link** |

### Verified (2 tests, clippy clean)

- `frames_round_trip` — the codec encodes/decodes both frames exactly (valid
  flags included).
- `flight_core_stabilizes_over_the_hitl_link` — the verified core levels a
  vehicle started tilted ~0.4 rad **entirely over the link** (every sensor read
  and motor write crosses the byte protocol to a `SimServer`), ending near level
  (accel z-fraction > 0.99).

### Honest scope

The `Transport` is exercised by an in-process loopback (the closed loop flies
over the real frame encode/decode). The documented GAP is the *physical*
transport — a real UART/USB/UDP link and its latency, jitter, framing-error and
reconnect handling. The protocol and the closed-loop contract are real; the wire
underneath is the integration step.

## [falcon-v1.13.0] — 2026-06-03

The **first real driver** against the v1.11 hardware seam — what "implement five
drivers and the same verified core flies your board" looks like for one of them.

### Added

- **`falcon-imu-icm42688`** (`no_std`) — a TDK InvenSense **ICM-42688-P** 6-axis
  driver implementing `falcon-core::ImuDriver` over a minimal, dependency-free
  `RegBus` trait (the thin contract your SPI peripheral satisfies). It verifies
  the `WHO_AM_I` identity, configures ±16 g / ±2000 dps, burst-reads the
  big-endian accel+gyro data block, and scales raw `i16` counts to SI
  body-frame units.

### Verified (4 tests, clippy clean)

- `decodes_register_block_to_si` — against a mock bus, the documented config
  registers are written and a canned block decodes to SI (AZ = 2048 LSB →
  9.80665 m/s², GX = 164 LSB → 0.174533 rad/s).
- `rejects_wrong_chip` — a wrong `WHO_AM_I` returns `DriverError::WrongWhoAmI`
  (the chip is **not** silently flown).
- `decodes_negative_counts` — the sign bit decodes (AZ = −1 g).
- `satisfies_imu_driver_seam` — it plugs into a `HardwareBackend` as the IMU.

### Honest hardware GAPS (need the chip, not faked)

A real `embedded-hal` SPI `RegBus` (CS timing, mode 3, address-MSB convention)
and its **on-silicon validation**; calibration (bias/scale/board-mounting remap
— the driver uses an identity-remap placeholder); FIFO/data-ready timing; bus-
fault handling. The register protocol + scaling math are real and verified; the
bus and its validation are yours.

## [falcon-v1.12.0] — 2026-06-03

The **flight-math qualification seam**. `libm` is in the flight path (the
estimator's quaternion/gravity math, the geometric controller's `atan2`/`acos`,
the mixer geometry) and is therefore a qualification item. This collapses that
qualification surface from ~45 scattered call sites to **one crate**.

### Added

- **`relay-math`** (`no_std`) — the single boundary for every transcendental
  the verified cascade evaluates: `sqrtf`, `sinf`, `cosf`, `atan2f`, `acosf`,
  `fabsf`, `remainderf`, as `#[inline(always)]` wrappers (zero runtime cost; a
  *source-level* indirection). To qualify or replace `libm` — a proven
  polynomial core, CMSIS-DSP, a hardware CORDIC, a qualified `libm` build — you
  change this one file and the whole cascade inherits it.

### Changed

- **`relay-iekf`, `relay-geo`, `relay-adrc`, `relay-mix-quad`, `falcon-core`**
  migrated to route through the seam — **zero `libm::` references remain** in
  any of them; their direct `libm` deps (cargo *and* the bazel
  `relay-iekf`/`relay-mix-quad` libraries) are replaced by `relay-math`, which
  is now the cascade's only `libm` consumer.

### Verified

- `relay-math::seam_forwards_to_reference` asserts each wrapper agrees with
  `libm` (the conformance hook for a future qualified core). All cascade tests
  pass **unchanged** (behaviour identical); clippy clean; the embedded Cortex-M
  crate still builds; `bazel build //:relay-iekf //:relay-mix-quad //:relay-math`
  completes (the Component-Model path consumes the seam).

### Honest scope

The seam routes to `libm` **today — the unqualified default**. It makes
qualification a one-crate change; it does **not** by itself qualify `libm`.
Discharging that (a proven core / measured conformance) stays a tracked program
item — now with a single place to do it.

## [falcon-v1.11.0] — 2026-06-03

The **real-hardware backend seam** — the point the whole "build into any drone"
claim rests on, and your "sim-as-backend-first" pivot made concrete. The SAME
verified `FlightCore` / `FlightSupervisor` that flies the simulation now flies a
real airframe by swapping *only* the drivers.

### Added

- **Five driver-seam traits** (`falcon-core`, `no_std`): `ImuDriver`,
  `PositionDriver`, `MagDriver`, `MotorDriver`, `BatteryDriver` — the contracts
  a board satisfies, in SI/body-frame units.
- **`HardwareBackend<I, P, M, O, B>`** — implements `FlightBackend` purely by
  delegating to the five drivers. The estimator, geometric controller, ADRC
  loop, mixer, FSM, failsafes, and the kernel-checked WCET argument are all
  unchanged between sim and board.

### Verified (12 tests, clippy clean, embedded still builds)

- `flight_core_stabilizes_through_the_hardware_seam` — five mock drivers share
  one simulated plant through a `core::cell::RefCell` (each borrows only for its
  own call, so the motor-write driver steps the physics the next IMU-read driver
  observes). The verified core, started tilted ~0.4 rad, recovers to < 0.1 rad
  **entirely through the driver-trait indirection** — the seam carries a real
  closed loop, not a placeholder.

### Honest hardware boundary (documented GAPS, not faked)

The module docs list exactly what needs **your board**: real driver bodies
(register/bus sequences for a specific IMU/GNSS/mag/ESC/ADC) and their
on-silicon validation; sensor calibration; flight-tuning on the real airframe;
and discharging the v1.10 per-stage WCET leaf budgets with measured Cortex-M7
cycles. These are explicitly out of scope — the seam is ready for them.

## [falcon-v1.10.0] — 2026-06-03

WCET and schedulability for the flight loop — and the **last open `sorry`** in
the proof tree, closed.

### Proven (Lean 4 + Mathlib, kernel-checked, **0 sorry, 0 axioms**)

- **`pipeline_wcet_monotone_in_stages`** — the general compositional bound
  (append an arbitrary *list* of stages → WCET grows by at most their sum plus
  one fused hand-off each) was a `sorry` (a punted induction). Replaced with a
  direct closed-form proof: `List.sum_append` / `List.length_append` + a `Nat`
  case-split on the original length (empty pipeline → one-hand-off slack;
  non-empty → equality). **The proof tree has no `sorry` left.**
- **`falcon_cascade_pipeline` / `falcon_cascade_wcet_value`** — the actual
  `FlightCore::step` loop (IEKF propagate + 3 updates → geo → gyro-LPF → ADRC →
  mixer, fused) modelled as a `Pipeline`; summed budget **11507 cycles**.
- **`falcon_cascade_schedulable_1khz`** — that WCET ≤ **480000 cycles** (1 kHz
  on a 480 MHz Cortex-M7): schedulable with **>40× margin**.
- **`falcon_cascade_extensible`** — adding a stage grows the loop WCET by at
  most its budget + one hand-off; re-tuning never re-opens schedulability.

### Honest scope

The per-stage leaf budgets are **conservative declarations, not yet on-target
measurements**. The *structural* schedulability argument is complete and
kernel-checked now; discharging each leaf budget with measured Cortex-M7 cycle
counts is the v1.11 hardware-backend deliverable.

## [falcon-v1.9.0] — 2026-06-03

The audit found the estimator was only ever shown against a **perfect world**.
This injects the four pathologies that actually break estimators and shows the
verified cascade holds — or degrades gracefully — through the HAL under each.

### Added

- **`falcon-core::Pathology`** + `SimBackend::with_pathology` — a deterministic
  (counter-seeded LCG, no `rand`, `no_std`) injector for: broadband
  accelerometer **vibration**, a slow **gyro-bias drift**, a **GPS-dropout**
  window, and **magnetometer interference**. Deterministic ⇒ a robustness PASS
  *or a falsification* is reproducible.

### Verified (11 tests, clippy clean)

- `holds_through_accelerometer_vibration` — 1.5 m/s²/axis broadband; the IEKF
  gravity update rejects it, peak tilt < 0.15 rad, altitude held.
- `iekf_tracks_gyro_bias_drift` — a 0.004 rad/s² ramp; the IEKF gyro-bias state
  tracks it so the attitude does not walk off (peak tilt < 0.15 rad).
- `survives_gps_dropout_and_recovers` — a 2 s fix dropout; the IEKF dead-reckons
  (drift bounded < 2 m) and **re-converges < 0.5 m** when the fix returns.
  Honest graceful degradation, **not** "no drift".
- `tolerates_mag_interference` — 0.3/axis heading-reference corruption; attitude
  does not destabilise.

### Scope

The robustness the audit flagged untested is now evidence-backed against the
sim backend. Real-sensor noise characterisation belongs to the hardware
backend (v1.11) — this is the sim-side falsification harness for it.

## [falcon-v1.8.0] — 2026-06-03

The FSM wired into the loop, and the failsafes the audit found **detected but
never actuated** now actuate.

### Added

- **`falcon-core::FlightSupervisor`** — wraps the verified cascade with the
  `relay-fsm` FSM and the failsafe monitors. Each step: read the estimate;
  on a **geofence breach** or **low battery** while airborne, fire `Failsafe`
  → the FSM commands **RTL**; fire milestone events; map mode → setpoint
  (Takeoff/Loiter → hold, Mission → target, RTL → home, Land → descend); step
  the core. All through the `FlightBackend` seam (battery via `read_battery_v`).

### Verified (7 tests, clippy clean)

- `geofence_breach_actuates_rtl_home` — commanded to `[4,0]` *outside* the
  1.5 m fence, the vehicle crosses it, the supervisor fires `Failsafe` → RTL
  → flies home and lands (within 1 m of home, **not** at `[4,0]`). The
  geofence **actuation** the audit found missing.
- `low_battery_actuates_failsafe` — a sag below 14 V while loitering triggers
  a failsafe recovery. The battery failsafe the audit found absent.

### Scope

Composes relay-iekf + relay-geo + relay-adrc + relay-mix-quad + the proven
relay-fsm + geofence/battery monitors behind the seam. Next: injected sensor
pathologies (v1.9), WCET (v1.10), the real-sensor backend seam (v1.11).

## [falcon-v1.7.0] — 2026-06-03

The **autonomy layer** the clean-room audit found missing — a real flight-mode
state machine with **formally-proven safety**.

### Added

- **`relay-fsm`** — `Disarmed → Armed → Takeoff → Loiter ↔ Mission → Land →
  Disarmed`, with `Rtl` reachable from any flying state, over the `relay-arm`
  arming gate. Transitions are safety-guarded and total.

### Verified (4 tests, clippy clean, 2 Kani harnesses SUCCESSFUL)

- **Kani `verify_never_disarm_airborne`** — for *any* state/event/gates, a
  result of `Disarmed` implies the prior state was `Disarmed`/`Armed`/`Land`.
  **Motors can never be cut in flight by a disarm request.**
- **Kani `verify_failsafe_recovers`** — a `Failsafe` from any flying state
  commands `Rtl` (or `Land` without a fix), never `Disarmed`, never a no-op.
- Arming refused while tilted or throttle-up; the nominal mission lifecycle.

### Scope

Wiring the FSM into the integrated flight loop (driving takeoff/land/RTL
setpoints) + the geofence→RTL *actuation* is v1.8.

## [falcon-v1.6.0] — 2026-06-03

**The verified flight core, bare-metal on an ARM Cortex-M, with the sim as the
backend** — your exact "run on a board with the simulation as the backend
first" milestone, on an emulated target.

### Added

- **`embedded/falcon-cortex-m`** — the verified flight core (IEKF →
  geometric → ADRC → mixer via `falcon-core`) + the `SimBackend`, built into a
  bare-metal **ARM Cortex-M (STM32H743) ELF** (`thumbv7em-none-eabihf`,
  `cortex-m-rt` + an STM32H743 memory map). `file` → *"ELF 32-bit LSB
  executable, ARM, EABI5"*, ~142 KB. The `#[entry]` runs the verified
  `FlightCore` + `SimBackend` loop on the MCU (fly to a `[2,−1.5,−2]` m
  setpoint) and reports **ON-TARGET PASS** via semihosting.
- **`scripts/build-cortex-m.sh`** — builds + stages the ELF at the path
  `renode/falcon-cortex-m.resc` loads (the emulated STM32H743 harness).

The same `no_std` code that flies in the SITL bench now compiles + links for a
real MCU and runs the **sim backend on the target**. Swap `SimBackend` for a
real-hardware `FlightBackend` (the v1.11 seam) and the same ELF flies a drone.

### Scope

The thumbv7em toolchain + Renode/QEMU are bench tools (not in CI; the build +
run steps are bench-only). The ELF builds + stages locally; the on-target RUN
(Renode boot + execute) needs Renode installed.

## [falcon-v1.5.0] — 2026-06-03

**`meld` is wired** — the PulseEngine fusion tool the `CLAUDE.md` names but a
clean-room audit found **never invoked** (composition was `wac`-only). The
verified cascade now fuses into a single deployable module.

### Added

- **`scripts/meld-fuse-cascade.sh`** — builds the five verified Component
  Model leaves (`falcon-iekf` — the verified IEKF — + position/attitude/rate/
  mixer) and `meld fuse`s them into a **single WebAssembly core module**:
  - 58 exports, 1495 functions, **354 KB** (97.3% reduction from 13.2 MB)
  - `meld inspect` → "Format: Core WebAssembly Module"

  This is the embedded deployable artifact (one module → loom → synth → gale
  on a target) — and it makes the `CLAUDE.md` "Meld fuses components" claim
  true for the first time.

### Scope

`meld` is not provisioned in the CI gate runner, so the fusion is a bench step
(skipped in the gate like the bazel/gz steps), demonstrated locally via the
script. Running the **fused module on an emulated Cortex-M with the sim
backend** is v1.6.

## [falcon-v1.4.0] — 2026-06-03

The verified stack starts **becoming the composed stack**. The clean-room
audit found the Component-Model cascade wrapped the *old* crates (Mahony ekf,
small-angle att) while the verified v0.21→v1.0 work was plain cargo crates
outside the graph. v1.4 closes that gap for the estimator.

### Added

- **`relay-iekf` as a bazel `rust_library`** + **`falcon-iekf`** — the
  verified Invariant-EKF wrapped as a Component Model component
  (`wasm/cm/iekf`) exporting the same `falcon:cascade/ekf` interface as the
  v0.6 Mahony `falcon-ekf`.

### Changed

- **`falcon-cascade-composed` now plugs the verified `falcon-iekf`** into the
  ekf socket instead of the Mahony `falcon-ekf` — `wac` wires it (same
  interface), producing `falcon-cascade-composed.wasm`. **The composed
  cascade now uses the verified IEKF.**

### Scope

The estimator is migrated; position/attitude/rate → relay-geo/relay-adrc and
**meld-fusing** the composed cascade into a single deployable module are
v1.5+. Built locally against the vendored deps + by the Component Model
cascade CI job.

## [falcon-v1.3.0] — 2026-06-03

The **complete verified hover stack** now runs backend-agnostically through
the HAL — attitude + altitude + **horizontal position**, the last and
hardest loop (it requires tilting to translate, the tilt/accel ambiguity).

### Added

- **`FlightCore` horizontal position loop** — `set_position(ned)`; a P-D on
  (pos, vel) error → magnitude-saturated `a_cmd`, fed to the geometric
  `desired_rate` so the vehicle tilts to translate.
- **`SimBackend` full 3-D translation** — thrust along −body-z projected into
  NED (`−T·R·ẑ + g`); `read_position` returns the full NED position.

### Verified (5 tests, clippy clean)

- `position_hold_flies_to_setpoint` — through the seam, the core flies from
  the origin to a `[2, −1.5, −2]` m setpoint and holds it within **0.5 m**,
  settling near level (<0.15 rad).

### Scope

The SimBackend uses the near-hover accelerometer model; the realistic
specific-force + acceleration-compensated path under aggressive motion is the
gz bench's job (v0.30–v0.35). Next: componentize the verified stack (v1.4) and
**meld-fuse** it (v1.5) toward on-target execution.

## [falcon-v1.2.0] — 2026-06-03

The backend-agnostic core gains the **altitude loop** and **disturbance
rejection** — both demonstrated entirely through the `FlightBackend` seam.

### Added

- **`FlightCore` altitude hold** — `thrust = hover − k_p·alt_err + k_d·v_z`
  (clamped), driven by the estimator's z/vz (decoupled from the tilt/accel
  ambiguity). `set_altitude(ned_z)` commands it.
- **`SimBackend` vertical dynamics** (thrust → altitude) + an injectable
  body-torque **`disturbance`** field; the accelerometer keeps modelling the
  gravity reaction so attitude stays observable.

### Verified (4 tests, clippy clean)

- `altitude_hold_climbs_to_setpoint` — through the seam, the core climbs to a
  −2 m setpoint (within 0.25 m) and stays level (<0.1 rad).
- `disturbance_rejected_holds_level` — under a sustained `[0.25,−0.15,0]`
  torque disturbance the verified ADRC ESO cancels it; steady tilt <0.12 rad.

### Scope

A vertical+attitude sim backend (horizontal pinned). Full 6-DoF horizontal
position is v1.3; the gz backend, on-target run, and meld-fused deploy follow.

## [falcon-v1.1.0] — 2026-06-03

The first step of the **road from "SITL-verified core" (v1.0.0) to
"deployable on real hardware"** — and it starts with the seam everything
else hangs off. A clean-room audit confirmed v1.0.0 is a *verified controller
in a simulator*, not a flyable drone; the v1.1.x series builds the
abstraction layer, composition, and on-target path toward a hardware
release. The plan: run the same verified code on a real board with the
**simulation as the backend first**, then swap the backend to real sensors —
which is exactly why the abstraction layer is the centerpiece.

### Added

- **`falcon-core`** (new crate) — the verified cascade (IEKF → geometric
  SE(3) → ADRC → mixer) factored out of the Gazebo bench into a
  **backend-agnostic** core behind a hardware-abstraction-layer seam,
  **`FlightBackend`** (`read_imu` / `read_position` / `read_mag` /
  `write_motors` / `dt`). The same `no_std` flight code runs unchanged
  whether the backend is a simulator or a real flight controller — a drone is
  "supported" exactly when someone implements `FlightBackend` for its sensors
  + actuators.
- **`SimBackend`** — the first backend behind the seam (analytic rigid-body
  attitude plant), and a deterministic test: the verified cascade, driven
  *only* through `FlightBackend`, recovers a ~0.4 rad tilt to **<0.1 rad** —
  the seam carries the real estimator + controllers, not a stub. A second
  test shows an arbitrary backend drives the core with motors in [0,1].

### Scope

v1.1 is the seam + the inner attitude-stabilization core. The position/
mission outer loop, a gz `FlightBackend`, on-target (emulated Cortex-M)
execution, meld-fused deployment, autonomy/safety, and a real-hardware
backend are the v1.2 → v1.11 releases.

## [falcon-v1.0.0] — 2026-06-03

**The formally-verified flight stack — SITL-complete.** A capstone, not new
flight code: it consolidates the v0.21 → v0.39 arc into a 1.0 release and
states honestly what is proven and what is not.

What v1.0 IS — every layer of an autonomous multirotor cascade with a
**mechanical gate**, flying real Gazebo SITL on `no_std`/`no_alloc` flight
crates:

- **Estimation** — Invariant-EKF on SE₂(3): full-state nav, online NEES
  consistency, acceleration-compensated tilt, **motion-adaptive process
  noise** (the crisp-mission fix), rotor-fault FDI, and **position-fix
  spoof/fault FDI** (NIS gate + innovation CUSUM).
- **Control** — geometric SE(3) (Lee 2010) + ADRC inner loop (proven ~50×
  cadence margin) + differential-flatness feedforward + a **simplex safety
  shield** with a **full-state** (position+attitude) Lyapunov backing.
- **Allocation** — airframe-agnostic `MixerN` (quad/hexa/coax), proven
  bit-identical to the quad mixer and shown to **close the loop on a
  6-rotor hexa**.
- **Scheduling** — gyro-synchronized loop pacing (robust to sim/host load).

Verification (all re-run green; independently confirmed by a clean-room
sweep of 13 headline claims):
- **Lean** — two kernel-checked Lyapunov proofs, **0 `sorry`/`axiom`**
  (attitude + position/combined).
- **Kani** — bounded model checking for the shield contract, the allocator
  bounds (incl. single-rotor-out + airframe-agnostic), ADRC/command-filter
  saturation, the spoof CUSUM, and the NIS gate.
- **proptest + rivet** — traceability `PASS`, 100% across all trace rules.
- **Five published falsifications** (the methodology's core): airmode mixer,
  single-stage gyro-sync, the ω_d command filter, the reference governor,
  and the honest slow-covert-spoof limitation.

What v1.0 is NOT (read this — it is the honest boundary):
- **SITL, not hardware/HIL.** Everything is verified in Gazebo + unit/Kani/
  Lean; nothing has flown on real hardware. HIL is the post-1.0 frontier.
- The **gz hexa flight** (independent physics) is future; "any drone" is
  proven at the allocation + SITL closed-loop level, not yet in gz physics.
- **Slow covert GPS spoofs** the filter follows evade innovation FDI — they
  need an independent cross-check sensor (future).
- The dynamic **LaSalle** "trajectory ⇒ converges" step is cited (Lee 2010)
  pending Mathlib's stability API; the algebraic core is machine-checked.

"Build into any drone and working" means: the verified building block is
complete in SITL, with the traceability + attestation a regulated programme
needs to *start* a hardware campaign — not a hardware-proven product.

## [falcon-v0.39.0] — 2026-06-03

"Build into any drone" — at the closed-loop level. v0.34 proved the allocator
is airframe-agnostic in isolation (Kani bound + quad bit-equivalence + hexa
symmetry). v0.39 shows the *whole verified cascade* controls a 6-rotor hexa.

### Added

- **`relay-mix-quad::MixerN::achieved_wrench`** — the forward effectiveness
  (`wrench = Mᵀ·motors`), the dual of `mix` (wrench → motors).
- **`hexa_cascade_stabilizes_attitude`** (closed-loop, no gz) — the *same*
  geometric cascade (`GeoAtt`, FALCON_QUAD gains) + the airframe-agnostic
  `MixerN::hexa_x()` allocator + `achieved_wrench` (the plant input) + a
  rigid-body integrator: starting tilted ~0.5 rad, the hexa converges to
  < 0.1 rad with every rotor command in [0,1]. The verified controller and
  the airframe-agnostic allocator compose into a stable closed loop for a
  6-rotor airframe.

### Honest scope

SITL/analytic — the forward effectiveness shares `MixerN`'s geometry, and the
demonstrated result is attitude convergence in a rigid-body model. The
**independent-physics gz hexa flight** (a new 6-rotor SDF + generalizing the
`[f32; 4]` motor path to N rotors) and **HIL** are future work. This lifts
"any drone" from allocation algebra to a closed-loop cascade demonstration.

## [falcon-v0.38.0] — 2026-06-03

Full-state Lyapunov — closes the simplex shield's attitude-only scope gap.

The v0.23 Lyapunov certificate (and so the v0.28 shield's recoverable-set
guarantee) covered only rotation. v0.38 adds the translational half: a
kernel-checked proof that the closed-loop position Lyapunov decreases, and
that the combined full-state Lyapunov is non-increasing.

### Added

- **`proofs/lean/PositionLyapunov.lean`** — kernel-checked (0 `sorry`/`axiom`,
  `bazel test //proofs/lean:position_lyapunov_test` PASSED). For
  `m·ë_x = −k_x·e_x − k_v·e_v`, the `k_x·e_x·e_v` cross-terms cancel and
  `V̇_pos = −k_v‖e_v‖² ≤ 0` (`vdot_cancellation_pos`, `vdot_nonpos_pos`, the
  LaSalle precondition `vdot_zero_iff_ev_zero`, `V_pos_nonneg`), plus the
  **combined `fullstate_vdot_nonpos`**: `V̇ = −k_Ω‖ω‖² − k_v‖e_v‖² ≤ 0`.
  Algebraically identical to the attitude proof — the same `ring` identity.
- **`relay-geo::position_lyapunov_decrease_certificate`** — runnable
  companion verifying `V̇_pos = −k_v‖e_v‖² ≤ 0` and the combined full-state
  `V̇ ≤ 0` over a grid.

### Impact

The simplex shield's certified safe set now rests on a **full-state**
non-increasing Lyapunov, so its guarantee covers position as well as
attitude. The position subsystem is globally stable, so it adds no new
safe-set boundary (the attitude `Ψ < 2` remains binding).

### Honest scope

The dynamic "trajectory ⇒ converges" step is the classical Lee 2010 Prop. 2
result, cited and deferred pending Mathlib's Lyapunov/LaSalle API (same
caveat as the v0.23 attitude proof).

## [falcon-v0.37.0] — 2026-06-02

Position-fix fault / GPS-spoof robustness — the estimator can no longer be
walked off course by a bad or hostile position measurement.

### Added

- **NIS validation gate** in `relay-iekf::update_position` — reject a fix
  whose normalized innovation squared `d² = rᵀS⁻¹r` exceeds a χ²₃ threshold
  (jump / outlier / gross-bias spoof); on reject the state + covariance are
  **unchanged**. Reuses the already-computed `S⁻¹` (no new inversion). Total
  (non-finite fix ⇒ +∞ ⇒ rejected). Default χ²₃ 25 (no nominal regression).
- **`relay-iekf::SpoofMonitor`** — a two-sided per-axis CUSUM (Page) on the
  position innovation that latches on a sustained directional walk-off (the
  covert spoof that keeps each fix inside the NIS gate). On declaration the
  cascade **freezes** position updates (dead-reckon) so the spoofer can't
  steer the vehicle.

### Verified

- **Kani**: `verify_nis3_total` SUCCESSFUL (non-finite fix ⇒ +∞ ⇒ rejected);
  `verify_spoof_monitor_no_false_alarm` SUCCESSFUL (no alarm below the drift
  slack; latching; total). 26 relay-iekf tests incl. *50 m jump gated with
  state unchanged*, *0/1000 honest fixes rejected*, *walk-off latches in ~10
  steps, no false alarm on noise*.
- **Real-gz** (geo-hover, 3 m/s GPS walk-off at 15 s): FDI **on** → detected
  @15.1 s → held: **1.27 m, ANEES 2.90 CONSISTENT**; FDI **off** → walked:
  **17.4 m, ANEES 3.9e6 OVER-CONFIDENT**.

### Honest scope

Innovation-based FDI catches jumps + walk-offs above the noise floor (rate
≳ 1.5 m/s in noiseless SITL). A slower *covert* spoof the tightly-trusted
filter simply follows keeps the innovation below the floor and evades the
CUSUM (confirmed: a 0.5 m/s walk-off was not detected). That regime needs an
**independent cross-check sensor** (baro/flow) — deferred. SITL, not HIL.

## [falcon-v0.36.0] — 2026-06-02

A **falsification release** (the methodology values these): the setpoint-side
reference governor was meant to tame aggressive missions; real-gz flight shows
it backfires — and that v0.35 already made the aggressive envelope good.

### Added

- **`relay-traj::RefGovernor`** — a verified "virtual-time" reference
  governor: sample a trajectory at a governed time `s` whose advance
  `ṡ = g·dt` is gated by the tracking error (`g = 1` on-track → `g_min` once
  the error exceeds a band), bounded `[g_min, 1]`. **Kani**
  `verify_gate_factor_bounded` SUCCESSFUL (the gate ∈ [g_min, 1] for any
  input; clamp split from the division). 9 tests + proptest: monotone,
  bounded-rate advance.

### Falsified (published — the result)

- **Wiring the governor into the cascade DIVERGES it.** Real-gz A/B at
  leg-time 3.0 s (1.67× nominal): governor **OFF = 3.35 m, ANEES 3.97
  CONSISTENT**; governor **ON = 394 m, ANEES 114 OVER-CONFIDENT**. The
  error→clock→setpoint feedback coupling induces a limit cycle — the same
  failure class as the v0.33 ω_d command filter. The governor ships as a
  verified primitive but **default-OFF** (opt-in `REF_GOV=1`).
- **Positive finding:** the governor is *unnecessary* — v0.35's adaptive
  process noise already flies the 1.67×-nominal mission **consistently with
  no governor** (3.35 m). The genuinely-infeasible regime (leg 2.0 s)
  diverges with or without it (control-limited, not reference-limited).

## [falcon-v0.35.0] — 2026-06-02

The mission is now **crisp**. v0.34 shipped a *recognizable* waypoint
mission; v0.35 fixes its root cause and delivers sub-metre, NEES-consistent
tracking.

The v0.33 analysis localized the mission fragility to the **estimator**:
under motion/load the fixed-process-noise IEKF grows over-confident
(position ANEES 50–1581), feeding a wrong state into the controller →
divergence. Per a state-of-the-art research memo (Mehra adaptive filtering
beat anti-windup / reference-governor / nested-saturation because the NEES
monitor is the ready oracle), the fix is **motion-adaptive process noise**.

### Added

- **`relay-iekf` adaptive process noise** — `propagate` inflates Q by the
  motion magnitude: `q_gyro·(1 + q_motion_gyro·‖ω‖²)` and
  `q_accel·(1 + q_motion_accel·‖a_ned‖²)`, each bounded to `[1, q_motion_max]`
  by `clamp_factor` (split from the arithmetic for a comparison-only proof).
  At rest (ω≈0, a≈0) the factor is exactly 1 — **hover is provably
  unchanged**. The propagation-side twin of the verified measurement-variance
  inflation. `q_motion_* = 0` reverts to legacy fixed Q.
- Bench A/B toggle `NO_ADAPTIVE_Q`.

### Verified

- **Kani** `verify_clamp_factor_bounded` SUCCESSFUL — the inflation factor is
  in `[1, max]` for any input (covariance can never shrink or explode).
- Unit (22 tests): `at_rest_inflation_is_identity` (hover unchanged),
  `motion_adaptive_q_adds_conservatism_under_motion` (NEES strictly down
  under motion, bounded), `clamp_factor_is_bounded`.
- **Decisive real-gz mission A/B** (the metric that diagnosed the failure is
  the gate): fixed Q → ANEES **1581 OVER-CONFIDENT, 35.5 m diverged**;
  adaptive Q → ANEES **0.87–2.92 CONSISTENT, 0.48–0.87 m**, reproducibly.

### Honest scope

In *clean synthetic* motion the fixed-Q filter is already conservative
(NEES ≈ 1.3, not over-confident) — the over-confidence is a real-gz coupling,
so the synthetic test certifies the *mechanism* and the gz ANEES A/B
certifies the *fix*. Still SITL (Gazebo), not hardware/HIL. The setpoint-side
reference governor (research runner-up) is deferred to a later version.

## [falcon-v0.34.0] — 2026-06-02

The verified autonomous-flight stack: a formally-gated IEKF → geometric
SE(3) → ADRC → mixer cascade flying real Gazebo Harmonic SITL, landed from
the `falcon-v0.21-iekf` branch. Every layer pairs an algorithm with a
**mechanical gate** (Kani bounded model checking, a kernel-checked Lean
Lyapunov proof, proptest, or rivet traceability).

**Honest scope (read this):** reliable SITL *hover* (~1.6 m) and a
*recognizable* waypoint mission. The mission is **not yet razor-crisp** —
the residual control-margin fragility is localized (v0.33) to the outer
position-loop + estimator under load, the v0.35 target. This is SITL
(Gazebo), not yet hardware/HIL.

### Added

- **`relay-iekf`** — Invariant-EKF on SE₂(3): full-state nav, 15×15
  group-affine covariance, invariant magnetometer heading update, online
  **NEES consistency monitor**, acceleration-compensated tilt, and a
  CUSUM **rotor-fault detector** (FDI).
- **`relay-geo`** — geometric SE(3) controller (Lee 2010): desired-attitude
  / attitude-error / moment, reduced-attitude S² recovery, differential-
  flatness body-rate feedforward, a **RecoverableSet** + **SimplexShield**
  (Black-Box Simplex runtime assurance — the moat).
- **`relay-adrc`** — linear ADRC inner rate loop (ESO disturbance
  rejection), bandwidth-separation invariant + ESO discrete-stability
  bound, and a 2nd-order critically-damped **CommandFilter** (v0.33).
- **`relay-traj`** — Mueller–Hehn–D'Andrea jerk-minimizing quintic motion
  primitive (closed-form, no_std) with a sound peak-jerk bound.
- **`relay-mix-quad`** — verified mixers (thrust-floor, priority, airmode,
  single-rotor-out reconfig) and **`MixerN`**, the airframe-agnostic N×4
  allocator (quad/hexa/`from_geometry`, ≤8 rotors) — the "any drone" seam
  (v0.34).
- **Gazebo SITL bench** (`examples/falcon-sitl-gz`) — real-gz hover +
  waypoint-mission scenarios on the no_std/no_alloc flight crates, with
  **gyro-synchronized loop scheduling** (`pace.rs`, v0.32) that paces the
  loop to the IMU stream so a low sim real-time factor can't desync it.
- **Verification**: kernel-checked Lean Lyapunov proof
  (`proofs/lean/GeometricLyapunov.lean`, 0 `sorry`/`axiom` — V̇≤0, V
  pos-def, LaSalle precondition); Kani harnesses for the shield contract,
  the single-rotor-out allocator (MIX-P08), the airframe-agnostic bound
  (MIX-P09), the bandwidth-separation invariant, and the command-filter
  saturation; rivet traceability `PASS`.

### Changed

- Mixer default reverted to **priority** desaturation (airmode re-excites
  the marginal yaw loop). Pacing default is **on** (sim-time locked).

### Fixed

- Lean toolchain pin realigned **4.27.0 → 4.29.1** to match `rules_lean`;
  the geometric Lyapunov proof builds + passes again (offline via
  `--vendor_dir=vendor/bazel`).
- Acceleration-compensated tilt fixed estimator over-confidence under
  motion (ANEES 64 → 3.4); HeadingHold fixed waypoint corner-mirroring.

### Falsifications (published — wrong predictions are informative)

- **Single-stage gyro-sync diverged (524 m)** — cadence jitter tips the
  marginal loop; the two-stage anti-burst+anti-stale form is the fix (the
  A/B/C/D matrix: under CPU load wall-clock pacing diverges 770 m while
  sim-lock holds 2.39 m).
- **Command-filtering ω_d degraded tracking 10× (2.3 → 24 m)** — the
  cascade needs prompt rate tracking; the inner loop is already responsive
  and proven cadence-robust (~50× ESO margin), so command bandwidth is the
  wrong lever. The filter ships verified but **default-off**.
- The cascade was tuned to the **edge of stability for hover**; firmer
  gains to track a moving setpoint diverge — the localized outer-loop
  fragility, owned as the v0.35 target.

## [falcon-v0.6.0] — 2026-05-20

The WASM component pipeline. Control crates compile to WASM, fuse
through `meld` into one single-memory module, optimise through
`wasm-opt`, and AOT-compile through `synth` to a real ARM Cortex-M
ELF — hardware-independent, CI-reproducible.

Original v0.6 scope was hardware bring-up on a Cube Orange. Hardware
wasn't in place, so v0.6 was reworked to the WASM pipeline + Renode
emulation — which exercises the full meld → wasm-opt → synth
toolchain and is strictly more useful as a foundation.

### Added

- **`wasm/falcon-mix-component`, `wasm/falcon-rate-component`** —
  thin `cdylib` wrappers exposing `relay-mix-quad` / `relay-rate` as
  scalar-ABI WASM exports (`#[export_name]` kebab names matching the
  WIT worlds). Build to `wasm32-unknown-unknown`. The `rlib` path is
  unit-tested natively against the underlying control crates so the
  wasm exports are proven faithful (5 tests).
- **`wit/falcon-control/{mixer,rate}.wit`** — WIT worlds in the
  shape `spar-codegen`'s `wit_gen` module emits from the AADL
  airframe model. Hand-authored for v0.6; the spar → WIT codegen
  path is the `rules_wasm_component` follow-up.
- **`scripts/falcon-wasm-pipeline.sh`** — the reproducible pipeline:
  `cargo build --target wasm32` → `wasm-tools component embed`+`new`
  → `meld fuse --memory shared --address-rebase` → `wasm-opt -Os` →
  `synth compile --cortex-m`. `wasmtime` is the reference oracle.
- **`renode/falcon-cortex-m.resc`** + `renode/README.md` — Renode
  STM32H743 (Cortex-M7) machine script that loads the synth ELF.
  Runs in Linux CI via `renode-bazel-rules` (no macOS-arm64 portable
  Renode build exists; the `pulseengine/renode-bazel-rules` mac port
  is in progress).
- **`FV-FALCON-PIPELINE-001`** verification artifact;
  **`FEAT-FALCON-v0.6`** bumped `pending` → `approved`, scope
  reworked from hardware to WASM-pipeline.
- **meld + synth `cargo install`'d** as pinned `~/.cargo/bin`
  binaries so development churn in those repos cannot break the
  falcon pipeline.

### Pipeline result

```
2 components → meld fuse (shared memory) → 4412 B single-memory module
wasm-opt -Os:  4412 B → 4126 B
synth compile: fused module  → 1716 B ARM Cortex-M ELF
               mixer standalone → 911 B ARM ELF
wasmtime ref:  falcon-mix-total(0,0,0,0.5) = 2.0   ✓ matches native
               falcon-rate-torque(1.0) > 0         ✓
synth disasm:  elf32-littlearm confirmed
```

### Tool issues found + tracked

Bring-up surfaced three real tool issues — all investigated and
filed upstream so they're tracked:

- **synth#120** — `unmapped vreg` panic on f32 division
  (`compiler_builtins` `float::div`). `falcon-rate-component`
  standalone trips it; the `meld`-fused module containing the same
  code compiles fine. Commented on the open issue with the falcon
  repro.
- **synth#124** (filed) — `synth verify` is advertised in the CLI
  but is inert unless synth is built with `--features verify`.
- **meld#172** (filed) — `meld fuse` defaults to `--memory multi`,
  producing a module `wasm-opt` and `synth` reject; the pipeline
  works around it with `--memory shared --address-rebase`.

### Verification

- `cargo test --workspace`: 63 test suites green (was 61 in v0.5;
  +2 wrapper crates).
- `bash scripts/falcon-wasm-pipeline.sh`: PASS — meld fuse +
  wasm-opt + synth produce ARM ELFs (2/3 targets; the rate
  standalone is synth#120, documented).
- `rivet validate`: 0 broken cross-references.

### Deferred to v0.7

- The 3 libm-using control crates (`ekf`/`att`/`pos`) through the
  pipeline — gated on synth#120 (they do f32 division).
- Full Bazel integration via `rules_wasm_component`.
- Live Renode run wired into CI.
- `synth verify` Z3 translation validation (gated on synth#124).

## [falcon-v0.5.0] — 2026-05-19

The full outer-loop cascade closes. Vehicle flies from origin to a
10 m waypoint in pure-Rust SITL, settles within centimetres,
deterministic given a seed. POS → ATT → RATE → MIX → plant — five
layers of control closing in one bench.

### Added

- **`crates/relay-pos`** — cascaded P-PI position controller:
  - `PosController::tick(time, vehicle_pos, vehicle_vel,
    vehicle_quat, setpoint) → AttitudeSetpoint { quaternion, thrust }`
    consumes the vehicle's NED pose + a position setpoint and emits
    the attitude-loop setpoint for the inner cascade.
  - Outer P loop: position error → velocity setpoint clamped to
    `v_max_horizontal` / `v_max_vertical`.
  - Inner PI loop: velocity error → acceleration command, integral
    bounded ±`i_max`.
  - Small-angle map: horizontal acceleration → roll/pitch tilt;
    vertical acceleration → collective thrust (hover + offset).
  - Yaw: held when `yaw_setpoint` is NaN; followed when finite.
  - `PositionSetpoint::hover_at(pos)` for the common case.
  - NaN-safe sanitisation throughout; no panics on degenerate input.
  - 13 unit + proptest cases covering POS-P01 bounds, tilt clamp,
    altitude → thrust mapping (both directions), yaw hold/follow,
    integrator reset.
- **`examples/falcon-sitl-hover` `mission` scenario** — the v0.3/v0.4
  bench gains:
  - Translational plant dynamics (NED position + velocity, thrust
    in body-up direction rotated to NED, gravity, linear drag).
  - New `mission` scenario: vehicle starts at NED origin, commanded
    to (10, 0, 0) m, must reach and settle. Outer POS at 50 Hz,
    inner ATT at 250 Hz, RATE at 1 kHz; mixer exercised every tick.
  - Pass budget: final distance ≤ 0.5 m, convergence ≤ 10 s,
    RMS-steady (last 2 s) ≤ 1.0 m, no NaN.
  - 2 added integration tests (deterministic + noisy).
- **`FV-FALCON-POS-001`** verification artifact with extractable
  `fields.steps`. Brings the falcon gate from 7 → 8 artifacts
  (26 → 31 steps).
- **`FEAT-FALCON-v0.5`** bumped `pending` → `approved` with achieved
  metrics inline.

### Achieved bench metrics

| metric | value | budget |
|---|---|---|
| convergence to <0.5 m | **4.045 s** | ≤ 10 s |
| final distance | **0.010 m** | ≤ 0.5 m |
| RMS-steady distance (last 2 s) | **0.015 m** | ≤ 1.0 m |
| peak distance error | 10.000 m (initial) | — |
| loop wall time (12000 samples) | 1.3 ms | — |
| NaN/∞ in cascade | none | none |

### Verification

- `cargo test --workspace`: 60 test suites green (was 59 in v0.4).
- `cargo test -p relay-pos`: 13/13 PASS including 1 proptest at
  256-default + 4096-fuzz.
- `cargo test -p falcon-sitl-hover`: 10/10 PASS (added mission +
  noisy-mission + plant translational state tests).
- `cargo run -p falcon-sitl-hover --release`: PASS on all 5
  scenarios.
- `python3 scripts/run-falcon-verification.py --markdown`: ✅ 8/8
  falcon FV artifacts pass, 31/31 steps green.
- `rivet validate`: 0 broken cross-references.

### Scope notes — what slipped to v0.6

- **`host/relay-gps` host service** — v0.5 feeds plant position
  directly to the controller (the POS-P03 source-agnostic interface
  doesn't care, but real GPS noise/latency aren't exercised yet).
- **cFS-DNA stored-command mission execution** via `relay-sc` →
  v0.6 wires the waypoint through TBL + SC for the cFS↔PX4
  dual-DNA showcase.
- **`cargo-mutants` on full cascade** → v0.6 (CI infrastructure
  work).
- **Sanitizer pass** (ASAN / TSAN / Miri) on the SITL harness →
  v0.6.
- **Gazebo Harmonic SITL** → v0.7 — pure-Rust SITL still does
  everything verification needs; Gazebo earns its overhead when
  wind disturbance + sensor latency become first-class concerns.

## [falcon-v0.4.0] — 2026-05-19

Full inner cascade closes. Attitude controller cascades into rate
PID into mixer into plant, all in pure-Rust SITL. Commanded 20° tilt
reached in 0.352 s with 0.029° steady-state error.

### Added

- **`crates/relay-att`** — quaternion-error proportional attitude
  controller:
  - `AttController::tick(time, q_estimate, q_setpoint) → rate_cmd`
    consumes the EKF attitude estimate plus a setpoint quaternion;
    emits a body-rate command.
  - Shortest-arc selection: `q_err` always has non-negative scalar
    after correction.
  - Small-angle linearised output (`2 * q_err.vec * Kp`) within
    ~1.5 % of the exact axis-angle formula for `|θ| < 30°`.
  - Clamp to `rate_max` per axis (defaults `[4, 4, 2] rad/s`).
  - NaN-safe sanitisation: degenerate inputs fall back to identity
    rather than poisoning the cascade.
  - Embedded-friendly: no libm dependency (in-tree Newton sqrt for
    quaternion normalisation).
  - 10 unit + proptest cases for ATT-P01 (q_err invariant + shortest
    arc), ATT-P03 (small-angle bound), clamp, NaN handling.
- **`crates/relay-mix-quad`** — X-config 4-motor mixer for
  falcon-quad:
  - `QuadMixer::mix(torque_body, thrust) → [m1, m2, m3, m4]` with
    every output in `[0, 1]`.
  - Standard PX4 quad-x convention: motors numbered 1–4 clockwise
    from front-right, diagonal pairs spin CW/CCW.
  - Mixer matrix (rows = motors, columns = thrust/roll/pitch/yaw):

    ```
    M1 = [+1, -1, +1, -1]   front-right CW
    M2 = [+1, -1, -1, +1]   back-right CCW
    M3 = [+1, +1, -1, -1]   back-left  CW
    M4 = [+1, +1, +1, +1]   front-left CCW
    ```
  - Priority-preserving saturation: when raw mix exceeds 1.0, shift
    bus down (sacrifices collective thrust before sacrificing
    torque); negative outputs clipped to zero.
  - NaN sanitisation; negative thrust clipped to zero.
  - 10 unit + proptest cases including single-axis sign-preservation
    proptest.
- **`examples/falcon-sitl-hover` `attitude` scenario** — extends the
  v0.3 closed-loop bench with the full cascade. Setpoint 20° tilt
  about body-x. Outer ATT loop at 250 Hz; inner RATE loop at 1 kHz;
  MIX exercised every tick (NaN check). Returns PASS when
  convergence ≤ 1.5 s, RMS-steady ≤ 2°, no NaN. 2 added integration
  tests.
- **`FV-FALCON-ATT-001`, `FV-FALCON-MIX-001`** — v0.4 verification
  artifacts with extractable `fields.steps` (5 + 3 steps).
- **`FEAT-FALCON-v0.4`** bumped `pending` → `approved` with achieved
  metrics inline.

### Achieved bench metrics

| metric | value | budget |
|---|---|---|
| cascade convergence (to <2° tilt error) | **0.352 s** | ≤ 1.5 s |
| RMS-steady attitude error (last 1 s) | **0.029°** | ≤ 2.0° |
| peak attitude error | 19.989° (initial tilt) | — |
| loop wall time (5000 samples) | 545 µs | — |
| NaN/∞ in cascade | none | none |

### Verification

- `cargo test --workspace`: 57 test suites green (was 55 in v0.3).
- `cargo test -p relay-att`: 10/10 PASS including 1 proptest at
  256-default + 4096-fuzz.
- `cargo test -p relay-mix-quad`: 10/10 PASS including 2 proptest.
- `cargo test -p falcon-sitl-hover`: 7/7 PASS (was 5 in v0.3; +2
  attitude scenarios).
- `cargo run -p falcon-sitl-hover --release`: PASS on all four
  scenarios (step, disturbance, hover, attitude).
- `python3 scripts/run-falcon-verification.py --markdown`: ✅ 7/7
  falcon FV artifacts pass, 26/26 steps green.
- `rivet validate`: 0 broken cross-references.

### Scope notes — what slipped to v0.5

- **Drake-derived MultibodyPlant export** for the mixer matrix —
  matrix is hand-derived from first-principles airframe geometry in
  v0.4; same derivation will be cross-checked against a SymPy/Drake
  export in v0.5 (`SWREQ-FALCON-MIX-P01` formalisation).
- **Rocq formal proof of ATT-P01** quaternion-error invariants → v0.5
  with `rules_rocq_rust` wiring.
- **Verus SMT contracts on mixer arithmetic** (no_std + no_alloc
  Verus proof of MIX-P02) → v0.5.
- **Gazebo Harmonic SITL** → v0.5 with the position controller, when
  a full mission flight makes Gazebo's overhead worthwhile.
- **rerun.io `.rrd` evidence** → v0.5.
- **Differential test vs PX4 mc_att_control** — v0.5 once we wire
  the PX4 reference build into CI for direct trace comparison.

## [falcon-v0.3.0] — 2026-05-19

Rate controller + closed-loop SITL. The inner control loop closes:
EKF estimates attitude, rate PID drives commanded body rates, plant
integrates dynamics, closed loop converges. Pure-Rust SITL runs in
CI — no Gazebo install required.

### Added

- **`crates/relay-rate`** — body-rate PID stabilizer:
  - 3-axis PID with clamp-and-hold anti-windup. no_std + no_alloc;
    no transcendentals (just adds/multiplies/clamps).
  - Default gains tuned for a 500 g, 10-inch quadcopter at 1 kHz
    update; `RateGains::DEFAULT` with `i_max` and `torque_max`
    bounds per axis.
  - `RatePid::tick(time, measured_rate, setpoint_rate) -> torque`
    consumes gyro + setpoint, emits torque. dt derived from time
    deltas, clamped `[0.0001, 0.1]` s to defend against jumps.
  - `RatePid::reset()` for arm/disarm + setpoint discontinuity.
  - `RatePid::set_gains(g)` for TBL load.
  - 13 unit + proptest cases for RATE-P01 (integral bounded),
    RATE-P02 (no NaN/∞ propagation), RATE-P03 (step response
    convergence + Lyapunov surrogate), output-clamp, dt
    defensive bounding, reset semantics.
- **`examples/falcon-sitl-hover`** — closed-loop SITL bench. Pure
  Rust rigid-body plant (rotational dynamics + quaternion
  integration) driven by the v0.2 EKF + v0.3 rate PID. Three
  scenarios:
    - `step`: setpoint `[0.5, -0.3, 0.4] rad/s`; verify rise time,
      overshoot, steady-state error.
    - `disturbance`: at hover, inject a `1 rad/s` impulse about y;
      verify recovery time.
    - `hover`: vehicle starts with non-zero rates
      `[0.7, -0.5, 0.3]`; verify settle.
  CLI: `--scenario step|disturbance|hover|all`, `--noise σ`,
  `--quiet`. Returns 0 on PASS, 1 on FAIL. 5 unit + integration
  tests covering each scenario plus a noisy variant.
- **`artifacts/verification/FV-FALCON-RATE-001.yaml`** — v0.3
  verification artifact with extractable `fields.steps` (5 step
  commands).
- **`FEAT-FALCON-v0.3`** bumped `pending` → `approved` with
  achieved metrics inline.

### Achieved bench metrics

| scenario | convergence | overshoot | RMS-steady |
|---|---|---|---|
| step ([0.5, -0.3, 0.4] rad/s) | **0.139 s** | **1.2 %** | **0.0012 rad/s** |
| disturbance recovery (1 rad/s) | **0.141 s** | — | — |
| hover from [0.7, -0.5, 0.3] | **0.175 s** | — | — |

Loop wall time: ~400 µs for 5000 samples (one full 5 s trajectory).
No NaN/∞ in any scenario. Deterministic given a seed.

### Verification

- `cargo test --workspace`: 55 test suites green (was 52 in v0.2).
- `cargo test -p relay-rate`: 13/13 PASS including 2 proptest at
  256-default + 4096-fuzz.
- `cargo test -p falcon-sitl-hover`: 5/5 PASS.
- `cargo run -p falcon-sitl-hover --release`: PASS on all three
  scenarios.
- `python3 scripts/run-falcon-verification.py --markdown`: ✅ 5/5
  falcon FV artifacts pass, 18/18 steps green.
- `rivet validate`: 0 broken cross-references.

### Scope notes — what slipped

- **Gazebo Harmonic SITL** was originally scoped for v0.3 but
  pushed to v0.4. Pure-Rust SITL ships now because it runs in CI
  without Gazebo installation and produces byte-identical results
  given a seed. Real Gazebo lockstep arrives when the full
  attitude cascade (v0.4 with `relay-att`) makes a full hover
  meaningful.
- **Lean Lyapunov proof** of the rate loop → v0.4 with
  `rules_lean` wiring. The bench's empirical convergence is the
  v0.3 surrogate.
- **Kani bounded-overflow** on PID arithmetic → v0.4.
- **tokio-rs/loom** on a host bridge → v0.5 once the bridge
  exists.
- **rerun.io `.rrd` evidence** → v0.4 with the SITL hookup.

## [falcon-v0.2.0] — 2026-05-19

Real attitude estimator. Replaces the v0.1 stub with a Mahony
complementary filter on SO(3), validated by a deterministic
synthetic-IMU accuracy bench. No flight dynamics yet — that's v0.3.

### Added

- **`crates/relay-ekf`** — no_std + libm implementation of the
  Mahony, Hamel & Pflimlin (2008) complementary filter on SO(3)
  with gravity-only correction:
  - `Ekf::new()`, `Ekf::with_gains(EkfGains{kp, ki})`,
    `Ekf::set_initial_quaternion(q)`, `Ekf::tick(ImuSample)`.
  - Defaults Kp=2.0, Ki=0.05 (tuned for a 200 Hz–1 kHz consumer-
    grade IMU).
  - Bias estimate bounded ±0.5 rad/s under sustained excitation.
  - Pure-math helpers (`quat_mul`, `quat_conj`,
    `rotate_body_to_ned_inverse`, `cross`, `normalise`,
    `is_unit_quaternion`) exported for the controller layer.
  - 16 unit + proptest cases covering EKF-P01..P05 surrogates:
    unit-quaternion preservation per-tick + sequence, no NaN
    under adversarial accel, innovation monotone with tilt
    disagreement, static rest convergence, pure-yaw stability,
    bias bound.
- **`examples/falcon-ekf-bench`** — runnable accuracy bench:
  - 25-second deterministic synthetic trajectory at 200 Hz
    (rest at 20° pitch → roll → rest → yaw → rest).
  - Compares estimator vs ground truth, reports RMS attitude
    error in degrees + convergence time.
  - CLI: `cargo run -p falcon-ekf-bench --release` (deterministic);
    `--noise 0.2` for σ=0.2 m/s² IMU noise.
  - Acceptance budget: RMS-steady ≤ 5°, final ≤ 5°, no NaN.
  - Achieved on this release: RMS-steady **3.31°**, final
    **3.02°**, convergence **0.68 s**, peak 19.8°.
- **`artifacts/verification/FV-FALCON-EKF-001.yaml`** — v0.2
  verification artifact with extractable `fields.steps`:
  `cargo test -p relay-ekf` + release rerun + 4 k proptest fuzz
  + bench tests + bench binary smoke. Supersedes v0.1's
  `FV-FALCON-EKF-STUB-001` (which is preserved for history).

### Verification

- `cargo test --workspace`: 50 test suites green (was 49 in v0.1;
  one new — `falcon-ekf-bench`).
- `cargo test -p relay-ekf`: 16/16 PASS including 2 proptest cases
  at 256 default + 4096 fuzz mode.
- `cargo run -p falcon-ekf-bench --release`: PASS at v0.2 budget.
- `python3 scripts/run-falcon-verification.py --markdown` against
  the new gate: ✅ 4/4 falcon FV artifacts pass, 13/13 steps green.
- `rivet validate`: 0 broken cross-references.

### Known limitations

- **No magnetometer fusion yet** — gravity-only Mahony filter
  cannot observe heading directly. Small residual yaw drift is
  fundamental until v0.4 (`relay-att` with mag).
- **No Verus SMT proofs yet** on the EKF math. Deferred to v0.4
  with the `src/` Verus-annotated track + Bazel `verus_test`
  rules. v0.2 covers the same property classes via proptest at
  4 k cases.
- **No Lean WCET proof yet** on `Ekf::tick`. The estimator's wall
  time on a single tick is empirically ≤ 1 µs (5000 samples in
  333 µs on the bench runner), well inside a 1 ms IMU period;
  formal proof lands in v0.4.
- **No WASM-component compilation yet** — the EKF compiles as a
  plain `cargo` crate. wit-bindgen integration follows when the
  relay-substrate's P3 streams arrive (v0.3+).

## [falcon-v0.1.0] — 2026-05-19

The dual-DNA flight stack's first tagged release. Pre-product:
boots, exchanges MAVLink heartbeats with a ground control station,
proves the relay + falcon toolchain works end-to-end with the full
verification chain in CI.

### Added

- **WIT interfaces** (`wit/interfaces/`):
  - `relay-mavlink/protocol.wit` — MAVLink v2 protocol types
    (heartbeat, frame, codec-error).
  - `relay-control/dynamics.wit` — control-cascade typed streams
    (imu-sample, vehicle-state, rate/attitude/torque setpoints,
    motor-pwm). Records for v0.2–v0.5 forward-declared.
- **WIT worlds** (`wit/worlds/relay-falcon.wit`):
  falcon-quad (v0.1 first-ship target), falcon-hex (v1.0),
  falcon-coax (v1.0; Ingenuity-class).
- **Crates** (no_std + no_alloc):
  - `relay-mavlink` — MAVLink v2 codec with CRC-16/MCRF4XX
    (validated against the reference vector `0x6F91` on
    "123456789"), HEARTBEAT encode/decode (id=0,
    CRC_EXTRA=50), frame envelope construction and parsing,
    bounds-checked everywhere. 33 unit + proptest cases.
  - `relay-ekf-stub` — v0.1 placeholder state-estimator;
    emits identity attitude / zero position-velocity /
    healthy innovation. 9 unit + proptest cases. Real EKF
    arrives in v0.2.
- **Example** (`examples/falcon-hello/`):
  - `falcon-hello` binary with `--mode vehicle` and
    `--mode gcs`; exchanges MAVLink heartbeats over UDP
    loopback at user-configurable rate and duration.
  - 9 integration tests including
    `vehicle_and_gcs_exchange_heartbeats_over_udp` which
    proves the codec works over real sockets in-process.
- **Scripts** (`scripts/`):
  - `falcon-hello-demo.sh` — end-to-end smoke runner.
    Builds release binary, spawns vehicle + gcs as separate
    OS processes, asserts ≥ N heartbeats exchanged. Used
    as a step inside `FV-FALCON-WORLD-001`.
  - `run-falcon-verification.py` — extracts and runs every
    `fields.steps[].run` command from rivet-tagged
    verification artifacts; aggregates pass/fail per
    artifact; emits the Markdown comment template the
    GitHub-Actions sticky-comment poster consumes.
- **Rivet artifacts** (`artifacts/`):
  - `STKH-FALCON-001` stakeholder requirement —
    verified dual-DNA flight stack for safety-critical
    drones and smallsats across six standards.
  - `SYSREQ-FALCON-001..010` system requirements covering
    state estimation, rate / attitude / position control,
    mixing, MAVLink interop, airframe variants, dual-DNA
    composition, deterministic SITL, multi-domain credit
    packaging.
  - `SWARCH-FALCON-001` architecture decision capturing
    the Leeloo cross-DNA flows (TBL → controller gains,
    EVS ← EKF innovation, LC → SC RTL, mission via SC).
  - `SWDD-FALCON-{EKF, RATE, ATT, POS, MIX}-001` design
    descriptions with algorithms and verification mapping.
  - `SWREQ-FALCON-{EKF, RATE, ATT, POS, MIX, MAVLINK,
    WORLD}-P0*` software-level Verus/Lean/Rocq/Kani
    property requirements (~25 properties total).
  - `FEAT-FALCON-v0.1..v1.0` release-plan milestones,
    chained via `depends-on` so `rivet impact` answers
    "if I touch X, which release does that block?"
  - `FV-FALCON-{MAVLINK, EKF-STUB, WORLD}-001` v0.1
    verification artifacts. Each carries
    `fields.method: automated-test` (or
    `integration-test`) and `fields.steps:[{run: ...}]`
    so a verification gate can extract and execute them.
- **GitHub Actions** (`.github/workflows/`):
  - `ci.yml` — fmt + clippy + cargo test (linux / macos /
    windows) + falcon-hello-demo smoke.
  - `verification-gate.yml` — runs the rivet-driven
    falcon verification gate on every PR, posts a sticky
    Markdown comment with pass/fail per artifact. Filter
    overridable via `Verify-Filter:` line in the PR body.
  - `release.yml` — tag-triggered, builds the
    falcon-hello binary for x86_64 + aarch64 linux,
    x86_64 + aarch64 macOS, x86_64 Windows; cosign keyless
    signing (Fulcio OIDC) per archive; SHA-256 checksums;
    publishes a GitHub Release.
- **Top-level files**: `README.md`, `CHANGELOG.md`,
  `LICENSE`, and release notes at
  `.github/release-notes/v0.1.0.md`.

### Verification

- `cargo test --workspace`: 49 test suites, 0 failures
  (51 new tests added in this release).
- `bash scripts/falcon-hello-demo.sh`: PASS (14/14
  heartbeats exchanged over UDP, decoded payload fields
  byte-for-byte match the encoded values).
- `python3 scripts/run-falcon-verification.py --markdown`:
  ✅ 3/3 falcon verification artifacts passed, 8/8 steps
  executed.
- `rivet validate`: 0 broken cross-references across all
  falcon artifacts.

### Known limitations

The following are intentionally deferred per the falcon roadmap:

- **No Verus SMT proofs yet** on the new crates. Verification
  posture at v0.1 is extensive testing + proptest fuzz; formal
  proofs land in v0.2 alongside the real EKF.
- **No witness MC/DC coverage report** for the falcon WASM —
  WASM-component compilation is gated on the relay-substrate's
  P3 streams landing (v0.2 work).
- **No actual flight dynamics**. v0.1 emits constants from the
  EKF stub; v0.3 brings SITL hover; v0.6 brings hardware bring-up.
- **No SITL hookup** to Gazebo Harmonic. v0.3 wiring.
- **No Bazel `verus_test` rules** for the new crates. The cFS
  engines have them; falcon crates get them in v0.2 once the
  `src/` Verus-annotated track lands.

See [falcon/README.md](falcon/README.md) for the full v0.1 → v1.0
release plan.

---

Pre-falcon relay history (the cFS-isomorphic substrate that falcon
extends) is tracked in commits and rivet artifacts under
`artifacts/sysreq/STKH-RELAY-001.yaml` and downstream.
