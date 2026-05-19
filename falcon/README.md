# falcon — verified dual-dna flight stack

**a flight-software application built inside [relay](../).**
falcon is what relay looks like when the cFS-isomorphic mission layer
(SCH, LC, HK, SC, DS, FM, HS, CS, MD, MM, TBL, CI, TO) is fused with
the PX4-style real-time control cascade (EKF, rate, attitude, position,
mixer) into one verified organism. *leeloo, not frankenstein.*

> falcon is part of [relay](../) — a verified stream-based component
> framework. relay is the substrate; falcon is the application.
> [wohl](https://github.com/pulseengine/wohl) is a parallel
> application of relay for home supervision.

## tagline

*falcon flies. relay routes.*

## what falcon is

- **dual-dna**: cFS mission semantics + PX4 control law, sharing one
  bus, one time, one verification CI. PID gains arrive through table
  services. EKF innovation emits through event services. geofence
  trips through limit-checker. mission ops *is* the outer ring of the
  control loop, not a separate process talking to it over a pipe.
- **verified at every layer**: every component carries the
  [overdo chain](https://pulseengine.eu/blog/overdoing-the-verification-chain/)
  — Verus + Rocq + Lean + Kani + proptest + tokio-rs/loom + sanitizers
  + cargo-mutants + witness MC/DC + rivet traceability + sigil signed
  evidence + criterion regression budget.
- **six safety domains, one chain**: DO-178C DAL-A (EASA U-space,
  military UAS), ISO 26262 ASIL-D (automotive look-aside),
  IEC 61508 SIL-4 (general FS), IEC 62304 Class C (medical delivery),
  ECSS-Q-ST-80C Cat A (smallsat constellation), EN 50128 SIL-4
  (railway inspection).
- **airframe-agnostic substrate**: the world spec
  `pulseengine:relay-worlds/falcon-quad` is one of several intended
  variants (`falcon-hex`, `falcon-coax`, `falcon-vtol`, `falcon-fw`).
  the control-cascade components are shared; only the mixer and the
  exported motor-pwm shape differ.

## status

**pre-v0.1**, scope-defined, scaffolding in flight. follow the
[release plan](#release-plan) below; each numbered release ships a
specific verified-evidence increment.

this is work in progress and intentionally so. *don't wait for v1.0 to
get value* — v0.1 is real, runnable, and produces signed evidence
artifacts from day one.

## is this for you?

falcon is **for** you if any of these match:

- you ship (or want to ship) drones into regulated airspace —
  EASA U-space BVLOS, FAA part 137/91/135, military UAS programs of
  record, medical delivery, infrastructure inspection.
- you're a smallsat / CubeSat / lunar program that wants cFS-grade
  flight semantics *plus* drone-style control loops (entry-descent-
  landing, propulsive operations).
- you're a tier-1 automotive integrator who needs ASIL-D control
  components with a paper trail.
- you want a flight stack whose evidence chain you can hand to an
  assessor across six different safety standards without rebuilding
  the dossier from scratch each time.

falcon is **probably not** for you if you want:

- a 30-USD hobby drone autopilot — use
  [PX4](https://px4.io/) directly; its racing-grade tuning and
  community are unmatched.
- a quick research platform — use
  [ArduPilot](https://ardupilot.org/) or
  [Crazyflie](https://www.bitcraze.io/) for fast iteration without
  the verification overhead.
- proprietary closed-source flight software — use
  [AuterionOS](https://auterion.com/) for the most mature commercial
  stack.

## quickstart — three runnable examples

each example exercises the whole pulseengine stack: WIT-typed
components → meld static fusion → kiln runtime (sim) or synth AOT
(hardware) → witness coverage → sigil signed bundle → rivet
traceability evidence.

### example 1: `falcon-hello` (v0.1)

minimum viable flight stack. boots, answers QGroundControl's
heartbeat, blinks status LED. no flight dynamics, no control law —
just proves the pipeline.

```sh
cd examples/falcon-hello
bazel build //...                          # compile WASM components
meld fuse falcon-hello.world.wit \
  --output target/falcon-hello.bundle.wasm # static fusion
kiln run target/falcon-hello.bundle.wasm \
  --mavlink serial:///dev/ttyUSB0          # speak MAVLink to QGC

# in QGroundControl: vehicle appears, heartbeat lights up.
```

exercises: wit-bindgen, meld, kiln, relay-mavlink, relay-software-bus.

### example 2: `falcon-ekf-bench` (v0.2)

complementary-filter attitude estimator on a synthetic IMU log.
no hardware. produces signed MC/DC coverage + Verus proof certificate
+ rivet evidence link.

```sh
cd examples/falcon-ekf-bench
bazel test //:verus_falcon_ekf             # Verus contracts pass
bazel test //:kani_falcon_ekf              # bounded model checking
bazel test //:proptest_falcon_ekf          # randomized property tests
witness instrument target/falcon-ekf.wasm \
  -o target/falcon-ekf.cov.wasm
witness run --harness "cargo test" \
  --module target/falcon-ekf.cov.wasm      # MC/DC measurement
witness report --format json               # signed coverage envelope
sigil sign target/falcon-ekf.wasm          # in-toto attestation
rivet validate                              # link integrity
rivet coverage                              # evidence-to-req map
```

exercises: wit-bindgen, meld, kiln, witness, sigil, rivet, the full
verification chain (Verus + Kani + proptest + MC/DC).

### example 3: `falcon-sitl-hover` (v0.3)

headless lockstep SITL — **Gazebo Harmonic** (LTS to sep-2028) with
PX4 sitl conventions (`make px4_sitl gz_x500` is the 2025 reference)
driving simulated IMU / GPS / baro into the falcon component graph.
EKF + rate stabilizer + attitude controller + mixer run inside the
WASM components; motors converge to hover thrust. byte-identical
given seed. produces a rerun.io `.rrd` file for replay and audit.

```sh
cd examples/falcon-sitl-hover
bazel test //...                            # full overdo chain green
falcon-sim run --world falcon-quad \
  --backend gazebo-harmonic --headless \
  --seed 42 --duration 30s \
  --record hover.rrd                        # deterministic SITL run
sigil sign hover.rrd                        # signed evidence artifact
rerun hover.rrd                             # replay + inspect
```

exercises: the entire control cascade (EKF → rate → att → pos → mix),
deterministic Gazebo Harmonic SITL via MAVLink lockstep, rerun.io
evidence recording, full overdo chain on every component.

## release plan

honest incremental scope, witness-style. each version closes a
specific gap. signed binaries, CHANGELOG-disciplined, rivet-validated
release artifacts.

| version | what ships | verification delta | example |
|---|---|---|---|
| v0.1 | falcon-quad world spec wired; relay-mavlink answers QGC heartbeat; stub `relay-ekf` emits constants | Verus on stubs · rivet trace skeleton · sigil signature | `falcon-hello` |
| v0.2 | real `relay-ekf` (complementary filter, attitude only) | + Verus contracts on quaternion algebra · Lean WCET bound · witness MC/DC · proptest | `falcon-ekf-bench` |
| v0.3 | `relay-rate` controller (1 kHz PID + anti-windup); SITL stabilizes in attitude-hold mode | + Lean Lyapunov of rate loop · Kani overflow paths · tokio-rs/loom on host bridge | `falcon-sitl-hover` |
| v0.4 | `relay-att` controller (250 Hz quaternion error); `relay-mix` for falcon-quad geometry; SITL hovers | + Rocq for quaternion invariants · Verus matrix arithmetic · differential test vs PX4 reference | (extends v0.3) |
| v0.5 | `relay-pos` controller (50 Hz); `relay-gps` host service; SITL completes a waypoint mission | + cargo-mutants on full cascade · sanitizer pass · cFS-DNA inherits chain unchanged | (extends v0.3) |
| v0.6 | hardware bring-up on Cube Orange or pulseengine-board; tethered hover | + criterion budgets on silicon · synth+kiln WASM-on-MCU runtime · sigil firmware bundle | (HW) |
| v0.7 | untethered hover; fault injection on `relay-hs` watchdog | + fault injection suite · `relay-hs` triggers RTL on EKF divergence | (HW) |
| v0.8 | untethered waypoint mission; EASA U-space telemetry profile | + Z3 translation validation across loom optimizations | (HW) |
| v0.9 | geofence + return-to-launch under EW-simulated GPS loss | + abstract interpretation pass (MIRAI or Charon) | (HW) |
| v1.0 | six-domain credit dossier; `falcon-coax` + `falcon-hex` variant exports; Check-It checkers qualified for EKF + mixer | full overdo matrix green across all six domains in `rivet validate` output | (HW + dossier) |

## verification matrix (per new component)

each new control-cascade component carries the full overdo chain. the
existing cFS-DNA components in relay already do — falcon inherits.

| chain layer | relay-ekf | relay-rate | relay-att | relay-pos | relay-mix |
|---|---|---|---|---|---|
| Lean / Rocq (math) | EKF observability theorem | Lyapunov of rate loop | quaternion-error invariants | cascade stability | mixer geometry validity |
| Verus (SMT) | numerical bounds, state invariants | PID coefficient bounds, anti-windup | quaternion normalization | setpoint bounds | matrix arithmetic |
| Kani (bounded) | f32 → q15 quantization overflow | tick-rate edge cases | small-angle approx limits | mission edge cases | edge geometries |
| Z3 translation validation | loom passes on the EKF wasm | same | same | same | same |
| abstract interpretation | MIRAI prototype (v0.9) | TBD | TBD | TBD | TBD |
| proptest | random attitude perturbations | random commanded rates | random setpoints | random missions | random thrust vectors |
| tokio-rs/loom | host-side sensor reading | n/a | n/a | n/a | n/a |
| Miri / sanitizers | unsafe-region scan | n/a | n/a | n/a | n/a |
| cargo-mutants | test-adequacy on EKF | on PID | on quaternion | on mission | on mixer |
| witness MC/DC | post-codegen WASM | post-codegen WASM | post-codegen WASM | post-codegen WASM | post-codegen WASM |
| rivet | every req linked to evidence | same | same | same | same |

## six-domain credit alignment

the same chain earns credit in six standards. drone-specific reading:

| standard | falcon-specific use case | year credit becomes valuable |
|---|---|---|
| DO-178C DAL-A + DO-333 | EASA U-space BVLOS · military deep-strike | 2027 U-space enforcement |
| ISO 26262 ASIL-D | drone-as-coprocessor for robotaxi guidance · ADAS look-aside | now (any T1 platform) |
| IEC 61508 SIL-4 | inspection drones in nuclear / chemical / oil-gas | now |
| IEC 62304 Class C | medical delivery drones (defibrillator, blood, organs) | 2027 (FDA active) |
| EN 50128 SIL-4 | railway track + tunnel inspection drones | 2026–2028 (DB / SNCF) |
| ECSS-Q-ST-80C Cat A | CubeSat constellation flight software (cFS-DNA reuse) | already valuable |
| EU CRA | all connected drone products in EU | 2027 enforcement |

each row is a market where PX4's current posture cannot ship under
credit. falcon enters all of them on day-one of credit by inheriting
the chain.

## architecture — leeloo, not frankenstein

```
   ┌───────────────────────────────────────────────────────────┐
   │  cFS-DNA (already in relay)   — mission / ops layer       │
   │  ───────────────────────────────────────────────────────  │
   │  ground link (ci/to) ⇄ ccsds/cfdp protocols              │
   │  stored command (sc/sca) → scheduler (sch)                │
   │  limit checker (lc)  ← housekeeping (hk)                  │
   │  data storage (ds) + file manager (fm)                    │
   │  events (evs) + tables (tbl) + time + executive (es)      │
   │  health & safety (hs) + checksum (cs) + memory (md/mm)    │
   │                                                           │
   │  timescale: 1–10 hz command rate                          │
   └───────────────────────────────────────────────────────────┘
                  ▲ stream<vehicle-state>, stream<heartbeat>
                  │ stream<sensor-reading>
                  ▼ stream<command-message>, stream<setpoint>
   ┌───────────────────────────────────────────────────────────┐
   │  PX4-DNA (new in relay/falcon)   — control / dynamics     │
   │  ───────────────────────────────────────────────────────  │
   │  sensor drivers (imu, gps, mag, baro)     host services   │
   │  ↓ stream<imu-sample>                                     │
   │  ekf (state estimator)              ─ 1000 hz             │
   │  ↓ stream<vehicle-state>                                  │
   │  pos-control + navigator            ─   50 hz             │
   │  ↓ stream<attitude-setpoint>                              │
   │  att-control                        ─  250 hz             │
   │  ↓ stream<rate-setpoint>                                  │
   │  rate-control                       ─ 1000 hz             │
   │  ↓ stream<torque-setpoint>                                │
   │  control-allocator (mixer)          ─ 1000 hz             │
   │  ↓ stream<motor-pwm>                                      │
   │  pwm-out / dshot (host service)                           │
   └───────────────────────────────────────────────────────────┘
```

shared cross-DNA flows:

- PID gains via `relay-tbl` (TBL load → mixer/controller pickup).
- EKF innovation health via `relay-evs`.
- geofence violations via `relay-lc` → `relay-sc` (RTL trigger).
- stored mission via `relay-sc` → setpoint stream.
- watchdog via `relay-hs` → safe-state landing.

## testing & simulation strategy

four-tier testing pyramid. each tier closes a different blind spot
(per the [overdo principle](https://pulseengine.eu/blog/overdoing-the-verification-chain/)).
all tiers run in CI; only the bottom tier needs hardware.

| tier | what | tooling | byte-identical | when |
|---|---|---|---|---|
| 1: component proofs | Verus / Rocq / Lean / Kani / proptest / witness MC/DC | bazel + rules_verus / rules_lean / rules_rocq_rust | yes | every push |
| 2: lockstep SITL | full cascade in deterministic sim | **Gazebo Harmonic** (primary) · **MuJoCo 3** (secondary) | yes (seeded, headless) | every push |
| 3: differential SITL | second-opinion physics + ArduPilot guidance comparison | MuJoCo 3 + ArduPilot SITL | yes per backend | nightly |
| 4: HITL | falcon AOT binary on STM32H7 + Pixhawk 6X | custom HITL harness (we author it — no off-the-shelf WASM-on-MCU framework exists in 2025-2026) | reproducible | release gates v0.6+ |

**why Gazebo Harmonic as primary**: it's the PX4-default 2025 SITL
stack with built-in MAVLink lockstep; jMAVSim is deprecated. PX4 v1.16
added deterministic build hashes that pair with Gazebo's seeded fixed-
step lockstep for full pipeline reproducibility.

**why MuJoCo 3 as secondary**: CPU-deterministic, byte-identical given
seed, 50-200× real-time, Apache-2.0. perfect for fast CI runs and
differential testing against Gazebo.

**why not Isaac Sim / Genesis / Project AirSim**: GPU non-determinism
or closed-source. cannot produce certifiable byte-identical evidence.
(Genesis stays on the shelf for massive-batch proptest fuzzing where
throughput beats determinism — v0.5+ maybe.)

**Drake's special role**: not the runtime sim. instead, Drake derives
per-airframe `MultibodyPlant` models symbolically from URDF/SDF, and
those derivations feed the Verus mixer-geometry proofs *and* the
Gazebo SDF. one source of truth for the plant model, used by both the
simulator and the proof. **falcon would be the first published WASM
flight stack + SymPy/Drake-derived plant + Verus/Lean attestation
combination.** no public precedent.

**visualization**: [rerun.io](https://rerun.io) for live + signed
`.rrd` evidence (chunks content-addressed by sigil), Foxglove for
post-hoc mcap review by auditors, PlotJuggler for time-series diff in
CI. all three are MIT/Apache, headless-capable.

## airframe variants

| world | motors | mixer | first ship |
|---|---|---|---|
| `falcon-quad` | 4 (X-config) | 4×4 trivial | v0.1 |
| `falcon-hex` | 6 | 6×4, fault-tolerant | v1.0 |
| `falcon-coax` | 2 counter-rotating | 2×4 + servo flaps | v1.0 (Ingenuity-class) |
| `falcon-vtol` | 4 hover + 1 cruise | mode-switched | v2.x |
| `falcon-fw` | 1 thrust + 3 servo | aileron / elevator / rudder | v2.x |

motor count and mixer geometry are baked into each world's exported
`stream<motor-pwm>` type. the control-cascade components are shared
across variants; only the mixer is airframe-specific (and Verus-checked
against the airframe geometry).

## dna donors — what we steal from

the architectural decisions behind falcon are traceable to specific
prior art:

- **[PX4](https://docs.px4.io/main/en/flight_stack/controller_diagrams)**
  — cascade math (rate / attitude / position / allocator / commander
  state machine). BSD-licensed. extract module-by-module, re-express
  as WIT components with Verus contracts. MC/VTOL/FW share the same
  controllers, which matches falcon's variant strategy.
- **[Drake (TRI/MIT)](https://drake.mit.edu/)** — composition
  primitives + symbolic dynamics derivation. leaf-system + port +
  diagram pattern. SOS-based stability proofs, HJB reachability,
  MPC, QP-based control allocation. BSD-3, formal-methods-friendly.
  used in falcon to derive each airframe's verified mixer geometry
  from first principles, with the same `MultibodyPlant` feeding both
  the Verus proofs and the Gazebo SDF — single source of truth.
  external validation: [arxiv 2405.20502](https://arxiv.org/pdf/2405.20502)
  (TRI's SOS-based quadrotor safety certificates).
- **[F´ (F-Prime, NASA JPL)](https://github.com/nasa/fprime)** — port
  discipline + cmd/tlm/event/param quadrant. flew on Ingenuity (72
  successful sorties on Mars). Apache-2.0. mine FPP (F-prime prime,
  their port-type DSL) for WIT interface design lessons.
- **[NASA cFS](https://cfs.gsfc.nasa.gov/)** — the mission-layer
  isomorphism is already in relay (SCH, LC, HK, SC, DS, FM, HS, CS,
  TBL, CI, TO, MD, MM, CCSDS, CFDP).
- **[Martins et al., IJASS 2025](https://link.springer.com/article/10.1007/s42405-025-01044-z)**
  — peer-reviewed "F´ + ROS2 hybrid via protobuf bridge", motivated
  by "ROS2 lacks determinism, F´ lacks ecosystem." external validation
  of falcon's architectural bet — WIT+meld+P3-streams is the synthesis
  that doesn't need the protobuf bridge.

dishonorable mention:
- **SMACCMPilot / Ivory + Tower (Galois HACMS)** — spiritual ancestor;
  stream-typed concurrency DSL pattern; archived 2020. read their
  experience report and move on.
- **ArduPilot** — disqualified by GPL-v3 viral licensing.

## where to look in this tree

```
relay/
├── falcon/
│   └── README.md             (this file)
├── wit/
│   ├── worlds/
│   │   └── relay-falcon.wit  (the falcon-quad world spec)
│   ├── interfaces/
│   │   ├── relay-control/    (typed control-cascade streams)
│   │   └── relay-mavlink/    (MAVLink protocol bridge)
│   └── components/
│       └── {ekf,attitude-control,rate-control,pos-control,mixer}/
├── crates/
│   └── relay-{ekf,att,rate,pos,mix,mavlink}/
├── host/
│   └── relay-{imu,gps,mag,baro,pwm}/
├── examples/
│   ├── falcon-hello/         (v0.1: MAVLink heartbeat)
│   ├── falcon-ekf-bench/     (v0.2: verified EKF)
│   └── falcon-sitl-hover/    (v0.3: full cascade in SITL)
├── proofs/
│   ├── verus/control/        (numerical stability)
│   ├── lean/control/         (Lyapunov, cascade WCET)
│   └── rocq/control/         (matrix invariants)
└── artifacts/
    ├── sysreq/STKH-FALCON-001.yaml          (stakeholder req)
    ├── sysreq/SYSREQ-FALCON-*.yaml          (system reqs)
    ├── swreq/SWREQ-FALCON-*.yaml            (sw reqs / Verus properties)
    ├── swarch/SWARCH-FALCON-001.yaml        (dual-DNA architecture)
    ├── swdd/SWDD-FALCON-*.yaml              (design descriptions)
    ├── features/FEAT-FALCON-v*.yaml         (release milestones)
    └── verification/FV-FALCON-*.yaml        (formal verification)
```

## related projects

- **[relay](../)** — the substrate falcon is built on.
- **[wohl](https://github.com/pulseengine/wohl)** — home supervision
  application of relay; parallel to falcon at a different scale.
- **[meld](https://github.com/pulseengine/meld)** — static WASM
  component fusion (build-time wiring).
- **[kiln](https://github.com/pulseengine/kiln)** — WASM interpreter
  and runtime for safety-critical systems.
- **[synth](https://github.com/pulseengine/synth)** — WASM-to-ARM
  AOT transcoder with mechanized correctness proofs.
- **[gale](https://github.com/pulseengine/gale)** — verified Rust
  RTOS (Zephyr-replacement for ASIL-D); the embedded host for falcon
  on real silicon.
- **[witness](https://github.com/pulseengine/witness)** — MC/DC
  branch coverage for WASM; produces the structural-coverage evidence
  in falcon's overdo chain.
- **[sigil](https://github.com/pulseengine/sigil)** — supply-chain
  signing; produces the in-toto envelopes around falcon binaries
  and evidence artifacts.
- **[rivet](https://github.com/pulseengine/rivet)** — typed knowledge
  graph for ASPICE / V-model traceability; the evidence-to-requirement
  linker. all falcon rivet artifacts live under `artifacts/`.
- **[loom](https://github.com/pulseengine/loom)** — formally verified
  WASM optimizer; runs the Z3 translation validation between
  pre-optimization and post-optimization WASM.
- **[spar](https://github.com/pulseengine/spar)** — AADL v2.3
  toolchain + deployment solver; for v1.0+ when component-to-target
  assignment becomes formal.
- **[temper](https://github.com/pulseengine/temper)** — repository
  hardening; enforces falcon's branch protection + signed commits +
  required reviewers.

## why this exists

the safety-critical drone market is structurally vacant. PX4 ships
under experimental waivers; cFS doesn't fly drones; F´ flew one Mars
helicopter but isn't a general flight stack; ArduPilot is GPL; Auterion
is closed and commercial.

EASA U-space rules enter enforcement in 2027. EU Cyber Resilience Act
2027. FDA medical delivery drone framework in active proposal.
the certification-grade drone market doesn't yet have an open,
verified, multi-domain candidate.

falcon's bet is that the same verification chain, applied honestly
across every component, earns credit across all six safety standards
simultaneously — and that the leeloo dual-DNA framing produces a
better flight stack than either cFS-alone or PX4-alone could.

*do the work once. earn credit six times.*

---

## status of this README

this README is a *requirements artifact*, not a code artifact. it
describes what falcon will be. v0.1 is what makes the first row of
the release table green. follow `artifacts/features/FEAT-FALCON-v*`
in rivet for milestone tracking.

`rivet list --type feature --tag falcon --format json` returns the
machine-readable roadmap. `rivet coverage --tag falcon` returns the
evidence-to-requirement coverage. `rivet impact --since HEAD~1` shows
which falcon artifacts a given commit changes.

if you're reading this in 2027 and v1.0 has shipped, this README
should already have been replaced by a *facts* artifact rather than
an *intent* artifact. if not, send a PR.
