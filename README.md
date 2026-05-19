# relay

**Formally verified stream-based component framework for safety-critical real-time systems.**

Relay is a flight-software substrate: WIT-typed `stream<T>` components,
fused statically at build time by [meld](https://github.com/pulseengine/meld),
AOT-compiled to ARM/RISC-V by [synth](https://github.com/pulseengine/synth),
run on [Gale](https://github.com/pulseengine/gale) (verified Rust RTOS)
via WASM Component Model Preview 3 streams.

Triple-track formal verification: **Verus** (SMT/Z3) for component
logic, **Rocq** for stream-routing correctness, **Lean** for
scheduling theory and WCET. Traceability via
[rivet](https://github.com/pulseengine/rivet).

Part of the [pulseengine](https://github.com/pulseengine) toolchain.

> *Relay routes. Falcon flies.*

## Two layers, one substrate

| layer | what it is | crates |
|---|---|---|
| **cFS-DNA** (mission/ops) | A formally verified port of NASA cFS apps and core services. Drop-in replacements for the cFS framework that flew JWST, Artemis, OSIRIS-REx. | `relay-{sch, sc, sca, lc, hk, cs, ds, fm, hs, tbl, ci, to, md, mm, ccsds, cfdp}` |
| **PX4-DNA** (control/dynamics) | A formally verifiable port of the PX4 multicopter control cascade as WIT components. **New in v0.1.** | `relay-{ekf, ekf-stub, rate, att, pos, mix, mavlink}` |

Composition is through stream wiring via `meld` (build-time static
fusion), not runtime topic-name lookup. No router thread, no
shared mutable state.

## falcon — the dual-DNA showcase

falcon is the application world that fuses cFS-DNA mission semantics
with PX4-DNA control cascade for real airframes. **v0.1 ships
today** — see [`falcon/README.md`](falcon/README.md).

```sh
# Terminal A — ground control station
cargo run -p falcon-hello -- --mode gcs

# Terminal B — vehicle
cargo run -p falcon-hello -- --mode vehicle
```

Vehicle emits MAVLink HEARTBEAT messages over UDP at 1 Hz; GCS
decodes them. Output:

```
vehicle: tx seq=0 type=2 status=3 mavlink_v=2 (21 bytes)
gcs: rx heartbeat from 127.0.0.1:14551 type=2 autopilot=8 status=3 custom_mode=0
```

Connect QGroundControl with `--remote 127.0.0.1:14550` and the
vehicle appears in the QGC list.

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
cargo test --workspace        # 49 test suites, all green
cargo run -p falcon-hello -- --help
bazel test //...              # verus + rocq + lean tracks for the
                              # cFS engines (falcon Verus arrives in v0.2)
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
│   └── falcon-hello/      # v0.1 MAVLink heartbeat demo
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
