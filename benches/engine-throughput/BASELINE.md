# Engine throughput — per-cycle baseline (#8)

Per-cycle wall-time for each flight engine's hot path at realistic table load.
These guard the Lean-proven WCET budgets (`proofs/lean/WcetAnalysis.lean`,
`CompositionalWcet.lean`) against silent regression. Traced by
**FV-FALCON-PERF-001**.

Run: `cargo bench -p engine-throughput-bench`

## Baseline (2026-06-13)

Captured on an Apple-silicon dev host (release profile). **Absolute numbers are
hardware-specific** — CI runs on smithy `rust-cpu` (Hetzner Ryzen) and will
differ. Use these for *relative* regression tracking on like-for-like hardware,
not as a cross-machine SLA.

| Engine | Hot path (per cycle) | Workload | Median | Catastrophic-regression ceiling |
|---|---|---|---|---|
| LC  | `evaluate` ×32 | 64 watchpoints / 32-reading sensor frame | 2.59 µs | 50 µs |
| SCH | `process_tick` ×10 | 128 slots / 10-tick major frame | 1.38 µs | 50 µs |
| SC  | `process_tick` | 256 ATS commands scanned | 405 ns | 20 µs |
| HS  | `check_health` | 32 app monitors | 385 ns | 20 µs |
| CFDP| `process_nak` | retransmit event | 4.07 ns | 5 µs |

The ceilings are **coarse catastrophic-regression guards** (~10–20× headroom over
the dev-host baseline), chosen so they survive hardware variance between the dev
host and CI runners while still catching an order-of-magnitude blow-up. They are
NOT tight SLAs — a tight per-runner SLA needs persisted baseline history (see
"Deferred" below).

## What this gates

A 10× regression in any engine's per-cycle cost is an early warning that a
deployed system is drifting toward the WCET ceiling the Lean proofs bound. The
benches reflect the *per-cycle budget* (a full scan at realistic load), not
isolated-function microbenchmarks.

## Deferred (follow-ups on #8)

- **Tight per-runner SLA gate**: criterion baseline-comparison needs persisted
  history per runner class. The coarse absolute ceilings here are the interim.
- **Nightly history persistence**: store criterion output across runs for trend
  visibility. Needs a nightly job + an artifact store.
