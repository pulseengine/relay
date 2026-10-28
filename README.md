# relay

**Stream-based component framework for safety-critical real-time systems, with formal proofs of specific safety properties (Verus, Kani, Rocq and Lean).**

Relay is a flight-software substrate: WIT-typed `stream<T>` components,
fused statically at build time by [meld](https://github.com/pulseengine/meld),
AOT-compiled to ARM/RISC-V by [synth](https://github.com/pulseengine/synth),
run on [Gale](https://github.com/pulseengine/gale) (a Rust port of Zephyr
kernel primitives with Verus, Rocq and Lean proofs on its source; scope in its
[claims ledger](https://github.com/pulseengine/gale/blob/main/docs/safety/verification-honesty.md))
via WASM Component Model Preview 3 streams.

What the proofs cover: **Verus** (SMT/Z3) contracts on the cFS-DNA
mission-layer crates (19 `verus_test` targets in `BUILD.bazel`),
**Kani** bounded model checking on their engines, **Rocq** models of
the sch, sc, lc, hs and cfdp engines plus the sin/cos approximations
(`proofs/rocq/`), and **Lean** for the Lyapunov algebra and WCET
composition (`proofs/lean/`). The PX4-DNA control cascade has no Verus
target, the IEKF estimator is property-tested, and composed-pipeline
MC/DC coverage is 0/2713 because the witness harness cannot yet drive
the stream exports ([#202](https://github.com/pulseengine/relay/issues/202)). Traceability via
[rivet](https://github.com/pulseengine/rivet).

Part of the [pulseengine](https://github.com/pulseengine) WebAssembly Component Model engine.

> *Relay routes. Falcon flies.*

## Two layers, one substrate

| layer | what it is | crates |
|---|---|---|
| **cFS-DNA** (mission/ops) | A port of NASA cFS apps and core services, with Kani (bounded model checking) proofs on the flight-critical crates. Drop-in replacements for the cFS framework that flew JWST, Artemis, OSIRIS-REx. | `relay-{sch, sc, sca, lc, hk, cs, ds, fm, hs, tbl, ci, to, md, mm, ccsds, cfdp}` |
| **PX4-DNA** (control/dynamics) | A formally verifiable multicopter control cascade as `no_std` crates + WIT components: an **Invariant-EKF** estimator (SE₂(3)), **geometric SE(3)** attitude control (Lean-proven Lyapunov), an **ADRC** inner loop, control allocation, trajectory generation, a flight-mode FSM, and a backend-agnostic flight core. | `relay-{iekf, geo, adrc, mix-quad, traj, fsm, math, mavlink}`, `falcon-core` |

Composition is through stream wiring via `meld` (build-time static
fusion), not runtime topic-name lookup. No router thread, no
shared mutable state.

## falcon — the dual-DNA flight stack

falcon fuses cFS-DNA mission semantics with the PX4-DNA control cascade.
The cascade (**IEKF → geometric SE(3) → ADRC → mixer**) flies
autonomous waypoint missions in **Gazebo Harmonic SITL**, runs bare-metal on
an emulated **Cortex-M (STM32H743)**, and sits behind a backend-agnostic
hardware-abstraction seam (`FlightBackend`) so the *same* `no_std` code runs
against a simulator, real sensors, or a HITL link.

Current focus (the `v1.16+` realism arc) hardens it against the physics a clean
sim hides — wind, aerodynamic drag, IMU bias-instability, noisy/intermittent
GNSS, barometer fusion, battery drain — each a CI-gated test plus the matching
gz Harmonic realism. See [`CHANGELOG.md`](CHANGELOG.md) for the release history
and [`docs/dossier/v1.0-practical-readiness.md`](docs/dossier/v1.0-practical-readiness.md)
for the honest capability-vs-gap matrix (SITL/proof/emulation-complete; the
remaining gaps are on the hardware side of the seam).

```sh
# Fly an autonomous waypoint mission in Gazebo Harmonic SITL
examples/falcon-sitl-gz/tools/watch_mission.sh mission 55

# Minimal MAVLink heartbeat demo (no sim required)
cargo run -p falcon-hello -- --mode vehicle    # + --mode gcs in another shell
```

## Verification

Every component carries the overdo chain — see
[Overdoing the verification chain](https://pulseengine.eu/blog/overdoing-the-verification-chain/).
The chain is reproducible per push via the rivet-driven gate:

```sh
python3 scripts/run-falcon-verification.py --markdown
```

```
✅ Rivet verification gate — falcon
3/3 passed
| count |   |
| Passed | 3 |
| Failed | 0 |
| Skipped (no steps) | 0 |
```

GitHub Actions posts the same Markdown as a sticky PR comment via
`.github/workflows/verification-gate.yml`.

## Build

```sh
cargo test --workspace        # all crate test suites
cargo run -p falcon-hello -- --help
bazel test //proofs/lean:...  # kernel-checked Lean proofs (Lyapunov, WCET)
bazel test //...              # verus + rocq + lean verification tracks
```

## Traceability

ASPICE V-model artifacts under `artifacts/` are validated by rivet:

```sh
rivet validate                                          # link integrity
rivet list --type feature                               # rollout milestones
rivet list --type unit-verification                     # evidence
rivet coverage                                          # requirement coverage
rivet impact --since HEAD~1                             # change impact
```

## Layout

```
relay/
├── falcon/                # falcon-specific docs (README + release plan)
├── crates/                # no_std + no_alloc guest components (cFS + falcon)
├── host/                  # host services (relay-sb, -es, -evs, -time)
├── wit/                   # WIT interfaces + worlds
│   ├── interfaces/        # typed stream interfaces
│   ├── components/        # per-component world specs
│   └── worlds/            # cross-component compositions (relay-falcon.wit, ...)
├── examples/              # runnable showcases
│   ├── falcon-hello/      # minimal MAVLink heartbeat demo
│   ├── falcon-sitl-gz/    # Gazebo Harmonic SITL bench (the flight stack)
│   └── falcon-hitl-rfspoof/ # HITL GPS-spoof safety harness
├── proofs/                # verus / rocq / lean source proofs
├── artifacts/             # rivet artifacts (STKH, SYSREQ, SWREQ, SWARCH, SWDD, FEAT, FV)
│   ├── sysreq/            # stakeholder + system requirements
│   ├── swreq/             # software requirements (Verus properties)
│   ├── swarch/            # software architecture decisions
│   ├── swdd/              # detailed designs
│   ├── features/          # release plan + capability tracking
│   └── verification/      # FV-* with extractable `fields.steps`
├── scripts/               # falcon-hello-demo, run-falcon-verification
├── tools/                 # post_verification_comment + helpers
└── .github/workflows/     # ci.yml, verification-gate.yml, release.yml
```

## License

Apache 2.0. See [LICENSE](LICENSE) for the full text.

## Related repos

[meld](https://github.com/pulseengine/meld) ·
[kiln](https://github.com/pulseengine/kiln) ·
[synth](https://github.com/pulseengine/synth) ·
[loom](https://github.com/pulseengine/loom) ·
[gale](https://github.com/pulseengine/gale) ·
[sigil](https://github.com/pulseengine/sigil) ·
[witness](https://github.com/pulseengine/witness) ·
[rivet](https://github.com/pulseengine/rivet) ·
[spar](https://github.com/pulseengine/spar) ·
[wohl](https://github.com/pulseengine/wohl) — home supervision application of relay.
