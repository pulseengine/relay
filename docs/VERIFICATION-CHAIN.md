# The four-track verification chain (#4)

Relay's distinguishing claim is not one proof technique but **four independent
ones confirming the same properties** of each engine. This is the crosswalk: for
every core engine, which track proves what, where the evidence lives, and how to
re-run it.

> **Status reconciliation (2026-06-13).** Issue #4's body is stale. It records
> "Rocq: not started", "Lean: not started", "Kani: not in CI". All three are
> wrong as of today: Rocq proofs exist per engine, Lean proofs exist, and the
> Kani gate runs in CI (`.github/workflows/kani.yml`, since v1.55). This document
> is the corrected state. What genuinely remains is narrower — see *Remaining*.

## The matrix

| Engine | Verus (Z3) | Kani (CBMC) | Rocq | Lean |
|---|---|---|---|---|
| relay-lc (Limit Checker) | ✅ `relay_lc_verus_test` | ✅ **9/9** harnesses | ✅ `lc_proofs.v` | ◐ via WCET |
| relay-sch (Scheduler) | ✅ `relay_sch_verus_test` | ✅ 6 harnesses | ✅ `sch_proofs.v` | ◐ via WCET |
| relay-sc (Stored Cmd) | ✅ `relay_sc_verus_test` | ✅ 3 harnesses | ✅ `sc_proofs.v` | ◐ via WCET |
| relay-hs (Health/Safety) | ✅ `relay_hs_verus_test` | ✅ 7 harnesses | ✅ `hs_proofs.v` | ◐ via WCET |
| relay-cfdp (CFDP core) | ✅ `relay_cfdp_verus_test` | ✅ 3 harnesses | ✅ `cfdp_proofs.v` | ◐ via WCET |

✅ present + runnable · ◐ covered indirectly (system-level, not per-engine).
28 Kani harnesses total. relay-lc's full suite was re-run live on 2026-06-13:
**`Complete - 9 successfully verified harnesses, 0 failures, 9 total.`**

## What each track proves (and does not)

### Verus (Z3 / SMT) — functional correctness, per function
Bounded output, comparison semantics, persistence logic, per-call invariant
preservation. Source: each engine's `src/engine.rs` (the Verus tree) with
contracts. Runs in Bazel CI (`//:relay_<e>_verus_test`). **Does not** prove
inter-function or system composition.

### Kani (CBMC / bounded model checking) — absence of UB, per bounded state space
Panic-freedom, no integer overflow, bounded output arrays, state-machine
transition validity — over the full bounded input space, catching edge cases a
Verus contract might under-specify. Source: each crate's plain-only
`plain/src/kani_proofs.rs` sibling (never the Verus-stripped `engine.rs`). Runs
locally (`cargo kani -p relay-<e>`) and in CI (the **Kani gate** rolls all
per-engine harnesses into one required branch-protection context).

### Rocq — properties of a faithful model
Hand-written Rocq model proofs of each engine's decision logic
(`proofs/rocq/<e>_proofs.v`, `rocq_proof_test` targets). **Honest scope:** these
prove properties of a Rocq *model* of the engine — they are **not yet a
`coq_of_rust` refinement** of the actual Rust (unlike Gale's translated
`sem_proofs.v`). The model↔impl refinement is the remaining Rocq gap.

### Lean — scheduling / timing / dynamics
`WcetAnalysis.lean` + `CompositionalWcet.lean` bound the per-cycle WCET budget
(connected to the throughput benches, FV-FALCON-PERF-001);
`BackpressureSafety.lean` proves stream-channel safety; the Lyapunov proofs cover
flight dynamics. **Honest scope:** Lean currently proves *system-level* timing
and dynamics, not *per-engine* Rate-Monotonic/priority-ceiling theorems — that
per-engine scheduling formalization is the remaining Lean gap.

## Running the chain

```
scripts/verify-chain.sh <engine>     # e.g. relay-lc — runs/enumerates all 4 tracks
scripts/verify-chain.sh --all        # every engine
```

Kani runs locally if `cargo-kani` is installed; Verus and Rocq are Bazel targets
the script names (they need the Bazel toolchain); Lean proofs are listed. The
script degrades gracefully when a tool is absent and reports each track as
RUN / PRESENT / ABSENT rather than failing.

## Remaining (the genuine #4 gaps, re-scoped)

1. **Rocq `coq_of_rust` refinement** — promote the hand-written model proofs to a
   refinement of the actual plain Rust, so the proof is about the shipped code.
2. **Per-engine Lean scheduling proofs** — Rate-Monotonic schedulability +
   priority-ceiling correctness from the Spar AADL model, per engine.
3. **One CI aggregate target** — `bazel test //:verus //:kani //:rocq //:lean`
   surfaced as a single "four-track agreement" gate.

Tracks 1–4 *exist* for all five engines today; the above is what turns "four
artifacts" into "four refinement-level proofs of the shipped code, gated as one."
