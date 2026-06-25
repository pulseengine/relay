# `wit/` — Component Model interfaces

Why this directory has the shape it does (it is not arbitrary, and nothing here
is dead — every file maps to a world, a crate, or a generator). Read this before
"cleaning up" a `.wit` file: several are **generated** and several are
**composition contracts** whose implementation lives elsewhere.

## The three kinds of file here

1. **Hand-written interfaces + worlds** (the majority). Authored against the
   dual-DNA architecture (cFS mission services + PX4 flight control). These are
   the source of truth for their contracts.

2. **spar-GENERATED, drift-gated** — do NOT hand-edit; CI regenerates and diffs
   them (`.github/workflows/spar.yml`). Each carries a `// Generated from AADL`
   header:
   - `dronecan/node.wit` — from `Falcon_DroneCAN::DroneCanNode.Fmu`
   - `relay-transport/secure-channel.wit` — from `Relay_Transport::SecureEndpoint.Channel`
   Regenerate: `spar codegen --root <Pkg::Sys.Impl> --format wit spar/*.aadl`.

3. **Composition contracts** under `components/` — these describe the *shape a
   world expects* from a component. The **implementation** of each lives in its
   crate's own `wit/` + `src/` (e.g. `components/stored-command/` ⇄ `crates/relay-sc`,
   `components/checksum/` ⇄ `crates/relay-ci`, `data-storage/`⇄`relay-ds`,
   `file-manager/`⇄`relay-fm`, `table-services/`⇄`relay-tbl`). They look
   duplicative but are not: contract vs implementation.

## Layout

| Path | What | Status |
|---|---|---|
| `worlds/` | composition roots (`relay-minimal`, `relay-falcon`) — what gets fused | hand-written |
| `interfaces/` | domain APIs (control, mavlink, time, events, common-types, …) | hand-written |
| `components/` | per-component composition contracts (⇄ the cFS-engine crates) | hand-written |
| `primitives/transformers.wit` | the verified stream-transformer library | hand-written |
| `falcon-cascade/` | the production control cascade (sync call-return shape) | hand-written |
| `falcon-cascade-step/` | a sync single-tick facade — the witness MC/DC harness target | tooling |
| `falcon-cascade-stream/` | the P3 async-stream variant of the cascade | experimental |
| `falcon-control/{mixer,rate}.wit` | scalar-shape reference for spar codegen validation | hand-written |
| `falcon-flight/flight.wit` | the verified flight core as a wasmtime-runnable component | hand-written |
| `dronecan/node.wit` | **generated** (spar) | drift-gated |
| `relay-transport/secure-channel.wit` | **generated** (spar) | drift-gated |

## Known rough edges (cosmetic, tracked — not cruft)

- **Package-naming drift**: `pulseengine:relay-*@0.1.0` (consistent) vs the
  `falcon:*` packages (scattered versions 0.6/0.7/1.26) vs the dash-namespace
  generated packages (`falcon-dronecan:node`). Worth standardizing; low priority.
- **Three `falcon-cascade*` variants** — sync (production), step (coverage
  harness), stream (async). The names are close; the table above is the key.
- **`interfaces/relay-{executive,software-bus,tables}`** — declared but not yet
  imported by any world (forward-compatible / roadmap). Keep or cut is a roadmap
  decision, not obvious cleanup.

The 25 GB on disk (`vendor/`, `target/`, `wasm/cm/*/target/`) is **gitignored
build cache**, not committed — see the repo `.gitignore`.
