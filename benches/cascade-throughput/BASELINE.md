# Flight cascade throughput — per-cycle baseline (PERF-P01, #8)

Per-cycle wall-time for each stage of the falcon control cascade, and for the
whole cascade in sequence. `engine-throughput/BASELINE.md` covers the
cFS-lineage engines (LC/SCH/SC/HS/CFDP); **this file covers the control path
that actually flies**, which had no timing baseline of any kind before v1.135.

Run: `cargo bench -p cascade-throughput-bench`
Traced by **FV-FALCON-PERF-002**.

## Baseline (2026-08-25)

Apple-silicon dev host, release profile. **Absolute numbers are
hardware-specific** — treat them as a relative regression baseline on
like-for-like hardware, not as a cross-machine SLA and not as an on-silicon
figure.

| Stage | Hot path | Median | Share of cascade |
|---|---|---|---|
| `iekf_propagate` | one propagate at dt = 1 ms | **1.1256 µs** | **91 %** |
| `position_tick` | NED pos/vel → attitude setpoint | 73.40 ns | 5.9 % |
| `attitude_tick` | geometric SO(3), quat error → rate setpoint | 15.47 ns | 1.2 % |
| `rate_tick` | body-rate PID → torque | 10.25 ns | 0.8 % |
| `mixer_mix` | torque + thrust → 4 motor commands | 9.45 ns | 0.8 % |
| **`full_cascade_tick`** | **all five in sequence, one tick** | **1.2413 µs** | — |

## What the numbers say

**The estimator is the cascade.** IEKF is 91 % of per-tick cost; the other four
stages together are ~108 ns. Any work on cascade performance that is not work on
the estimator is rounding error. This matches the independent evidence from
jess's scry static bounds, which put iekf at 4 192 B of stack against 16–112 B
for the other four — two unrelated measurements agreeing on which stage carries
the mass.

**The 1 kHz budget is not close.** `full_cascade_tick` is 1.2413 µs against a
1 ms rate-loop period — **0.12 % of budget, ~806× margin** on this host. Even
allowing an order of magnitude for a Cortex-M7 at a fraction of the clock, the
control math is not what will bound the loop; the HAL, the scheduler and sensor
I/O will.

**`full_cascade_tick` is the honest per-tick figure** because the cascade is
single-rate: `cascade.step()` executes every stage once per tick, with no
divider (verified in `wasm/cm/cascade/src/lib.rs`). It is not a synthetic sum.

## What this does NOT establish

- **Not an on-silicon number.** This is an aarch64 dev host with a warm cache
  and a branch predictor a Cortex-M7 does not have. The compositional WCET proof
  in `proofs/lean/` bounds the cycle count; this bench does not measure it.
- **Not yet a comparison.** The PX4 head-to-head on the same board is the second
  half of PERF-P01 and is not done. Until it is, we can say what our cascade
  costs, not that it costs less than the alternative.
- **Not a tight SLA.** No persisted baseline history, so CI can only guard
  against catastrophic regression, not drift. Same limitation the
  engine-throughput bench records.

## Regression guard

A 10× regression in `full_cascade_tick` — or any stage crossing 10× its median
here — is an early warning that the control path is drifting toward the WCET
ceiling the Lean proofs bound. The ceilings are deliberately coarse so they
survive hardware variance between this host and CI runners.

| Bench | Baseline | Catastrophic-regression ceiling |
|---|---|---|
| `full_cascade_tick` | 1.2413 µs | 15 µs |
| `iekf_propagate` | 1.1256 µs | 15 µs |
| `position_tick` | 73.40 ns | 1 µs |
| `attitude_tick` | 15.47 ns | 500 ns |
| `rate_tick` | 10.25 ns | 500 ns |
| `mixer_mix` | 9.45 ns | 500 ns |

## Deferred

- **PX4 head-to-head on the same board** — the claim this bench exists to make
  possible.
- **On-silicon cycle counts** (Cortex-M7), which is what the WCET proof wants
  confronting with reality.
- **Persisted baseline history** for a tight per-runner SLA, rather than the
  coarse ceilings above.
