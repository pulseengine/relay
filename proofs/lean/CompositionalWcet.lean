import Mathlib.Tactic.Ring
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.NormNum
import Mathlib.Data.List.Basic

/-!
# Compositional WCET for Relay Primitives

This is the **novel result** that distinguishes Relay from cFS-in-Rust:
the WCET of a composed pipeline of stream transformers is bounded by the
sum of the WCETs of its constituent primitives plus a small composition
overhead.

The cFS-style approach (and the existing `WcetAnalysis.lean`) treats each
engine as a black box with a magic `per_iteration_cycles` constant. This
file derives those constants from primitive bounds, so:

  • Adding a new engine doesn't require a new WCET hand-estimate.
  • Refactoring the pipeline preserves the WCET argument.
  • Hot-updating a transformer at runtime can be checked against its
    declared WCET budget *before* swap-in.

Together with `WcetAnalysis.lean`, this closes the gap between the
verified primitive code (Verus) and the schedulability argument (Lean).

References:
  - `crates/relay-primitives/src/compare.rs` (CMP-P01..P03)
  - `crates/relay-primitives/src/persistence.rs` (PER-P01..P05)
  - `crates/relay-primitives/src/rate_divide.rs` (RD-P01..P04)
  - `crates/relay-primitives/src/time_gate.rs` (TG-P01..P03)
  - `crates/relay-primitives/src/merge.rs` (MRG-P01..P04)
  - `crates/relay-primitives/src/filter.rs` (FLT-P01..P02)
-/

/-! ## Primitive WCETs

Each constant is the worst-case cycle count for a single invocation of
the corresponding pure function on Cortex-M4F (no cache miss, code in
TCM, operands in registers). These are conservative upper bounds.
-/

def compare_cycles : Nat := 5
def persistence_decide_cycles : Nat := 8
def persistence_apply_cycles : Nat := 4
def rate_divide_cycles : Nat := 6
def time_gate_cycles : Nat := 3
def merge_cycles : Nat := 4
def filter_cycles : Nat := 3

/-! ## Pipeline composition -/

def fused_handoff_cycles : Nat := 1
def buffered_handoff_cycles : Nat := 10

structure Pipeline where
  stages : List Nat
  handoff_cost : Nat

/-- Compositional WCET: sum of stage costs plus handoff for each transition.
    Uses `List.sum` so mathlib lemmas about sums over `++` apply. -/
def Pipeline.wcet (p : Pipeline) : Nat :=
  p.stages.sum + p.handoff_cost * (p.stages.length - 1)

/-! ## The compositional bound

This is the headline theorem. Proves WCET is monotone under composition:
appending a stage adds at most its WCET plus one handoff to the total.

The proof splits on whether the original pipeline is empty. For the empty
case, the original had 0 handoffs and the new pipeline has 0 handoffs
(since 1 - 1 = 0 in Nat), so the difference is just the new stage. For
the non-empty case, appending adds exactly one handoff.
-/

theorem wcet_compose_monotone (p : Pipeline) (s : Nat) :
    ({ p with stages := p.stages ++ [s] } : Pipeline).wcet ≤
      p.wcet + s + p.handoff_cost := by
  simp only [Pipeline.wcet, List.sum_append, List.sum_singleton,
             List.length_append, List.length_singleton]
  -- After simp, goal involves:
  --   LHS: p.stages.sum + s + p.handoff_cost * (p.stages.length + 1 - 1)
  --   RHS: (p.stages.sum + p.handoff_cost * (p.stages.length - 1)) + s + p.handoff_cost
  -- Nat: length + 1 - 1 = length
  rw [Nat.add_sub_cancel]
  -- Now remaining: length * handoff ≤ (length - 1) * handoff + handoff
  -- Split on whether length is zero.
  by_cases hlen : p.stages.length = 0
  · rw [hlen]; simp
  · have hpos : p.stages.length ≥ 1 := Nat.one_le_iff_ne_zero.mpr hlen
    have heq : p.handoff_cost * p.stages.length =
               p.handoff_cost * (p.stages.length - 1) + p.handoff_cost := by
      conv_lhs => rw [show p.stages.length = (p.stages.length - 1) + 1 from by omega]
      ring
    omega

/-! ## The LC pipeline, derived from primitives -/

/-- LC's per-watchpoint inner loop, expressed as a composition of primitives:
    compare → persistence::decide → persistence::apply.
    Fused (lives in one Verus function), so handoff_cost = fused. -/
def lc_inner_pipeline : Pipeline := {
  stages := [compare_cycles, persistence_decide_cycles, persistence_apply_cycles]
  handoff_cost := fused_handoff_cycles
}

/-- The derived per-watchpoint WCET, computed from primitive bounds. -/
def lc_per_iteration_derived : Nat := lc_inner_pipeline.wcet

theorem lc_per_iteration_value : lc_per_iteration_derived = 19 := by
  unfold lc_per_iteration_derived lc_inner_pipeline Pipeline.wcet
  unfold compare_cycles persistence_decide_cycles persistence_apply_cycles
  unfold fused_handoff_cycles
  native_decide

/-- The derived bound is at most the hand-estimated bound from
    `WcetAnalysis.lean` (which used 20). -/
theorem lc_compositional_at_most_estimated :
    lc_per_iteration_derived ≤ 20 := by
  unfold lc_per_iteration_derived lc_inner_pipeline Pipeline.wcet
  unfold compare_cycles persistence_decide_cycles persistence_apply_cycles
  unfold fused_handoff_cycles
  native_decide

/-! ## A longer pipeline: filter + merge + compare

Demonstration that composition scales. A router that merges two sensor
streams, filters by a predicate, then compares against a threshold.
-/

def router_pipeline : Pipeline := {
  stages := [merge_cycles, filter_cycles, compare_cycles]
  handoff_cost := fused_handoff_cycles
}

def router_wcet : Nat := router_pipeline.wcet

theorem router_wcet_value : router_wcet = 14 := by
  unfold router_wcet router_pipeline Pipeline.wcet
  unfold merge_cycles filter_cycles compare_cycles fused_handoff_cycles
  native_decide

/-- Adding a stage to the router: WCET grows by at most the stage cost
    plus one handoff. A direct corollary of `wcet_compose_monotone`. -/
theorem router_plus_persistence_bounded :
    ({ router_pipeline with stages := router_pipeline.stages ++ [persistence_decide_cycles] } : Pipeline).wcet
      ≤ router_wcet + persistence_decide_cycles + fused_handoff_cycles := by
  have := wcet_compose_monotone router_pipeline persistence_decide_cycles
  unfold router_wcet
  unfold router_pipeline at this
  simpa using this

/-! ## Schedulability implication

Since `wcet_compose_monotone` is monotone, any pipeline built from
verified primitives has a computable, sound upper bound. Plugging that
bound into the RMA test in `WcetAnalysis.lean` gives schedulability for
any composition — without needing fresh WCET estimates per engine.
-/

theorem pipeline_wcet_monotone_in_stages (p : Pipeline) (extras : List Nat) :
    let p' : Pipeline := { p with stages := p.stages ++ extras }
    p'.wcet ≤ p.wcet + extras.sum + p.handoff_cost * extras.length := by
  -- Direct proof (no induction): unfold both WCETs, cancel the stage/extra
  -- sums, and the goal reduces to a single fact about the handoff term —
  --   h·(L + E − 1) ≤ h·(L − 1) + h·E
  -- which is *equality* when L ≥ 1 and a `≤` slack of exactly one handoff when
  -- L = 0 (the empty original pipeline declared 0 handoffs). Same split as
  -- `wcet_compose_monotone`, generalised from one stage to a list.
  simp only [Pipeline.wcet, List.sum_append, List.length_append]
  by_cases hL : p.stages.length = 0
  · -- empty original: h·(0 + E − 1) = h·(E − 1) ≤ h·E = h·0 + h·E
    rw [hL]
    have hmul : p.handoff_cost * (0 + extras.length - 1)
              ≤ p.handoff_cost * (0 - 1) + p.handoff_cost * extras.length := by
      simp only [Nat.zero_add, Nat.zero_sub, Nat.mul_zero]
      exact Nat.mul_le_mul_left _ (Nat.sub_le _ 1)
    omega
  · -- non-empty original: L + E − 1 = (L − 1) + E, so the handoff term splits
    -- exactly and the bound holds with equality.
    have hpos : p.stages.length ≥ 1 := Nat.one_le_iff_ne_zero.mpr hL
    have hsplit : p.stages.length + extras.length - 1
                = (p.stages.length - 1) + extras.length := by omega
    have hkey : p.handoff_cost * (p.stages.length + extras.length - 1)
              = p.handoff_cost * (p.stages.length - 1)
              + p.handoff_cost * extras.length := by
      rw [hsplit, Nat.mul_add]
    omega

/-! ## The Falcon flight cascade, as a WCET pipeline (v1.10)

Applies the (now sorry-free) compositional machinery above to the ACTUAL
flight loop — `FlightCore::step` in `crates/falcon-core`: the verified
cascade IEKF → geometric SE(3) → ADRC → mixer that runs every control tick.

The stages run in-line in one `step` (no buffered hand-off between them), so
the hand-off cost is `fused_handoff_cycles`. The leaf costs below are
**declared per-stage budgets**, conservative on Cortex-M7 (STM32H743, FPU,
code in TCM). They are NOT yet hardware-measured — discharging them with
on-target cycle counts is the v1.11 hardware-backend deliverable. The point
of this section is that the *schedulability argument is structurally complete
now*: given budgets, the compositional theorem yields a sound loop-WCET bound
and the period check passes. Measurement then only has to confirm each leaf
sits under its declared budget — it never has to re-derive the composition.
-/

/-- Declared per-stage budgets (cycles, Cortex-M7), worst case. The IEKF
    covariance propagate dominates; each measurement update is an EKF gain +
    state/cov correction. Conservative, pending on-target measurement (v1.11). -/
def iekf_propagate_budget : Nat := 4000
def iekf_update_budget : Nat := 2000
def geo_desired_rate_budget : Nat := 800
def gyro_lpf_budget : Nat := 100
def adrc_tick_budget : Nat := 400
def mixer_mix_budget : Nat := 200

/-- One control tick: propagate, the three measurement updates (gravity always;
    position + mag when a fix/reading is present — worst case both), the
    geometric desired-rate, the gyro low-pass, the ADRC inner loop, the mixer.
    Fused: the whole cascade lives in `FlightCore::step`. -/
def falcon_cascade_pipeline : Pipeline := {
  stages := [iekf_propagate_budget,
             iekf_update_budget, iekf_update_budget, iekf_update_budget,
             geo_desired_rate_budget, gyro_lpf_budget,
             adrc_tick_budget, mixer_mix_budget]
  handoff_cost := fused_handoff_cycles
}

def falcon_cascade_wcet : Nat := falcon_cascade_pipeline.wcet

/-- The summed budget: 4000 + 3·2000 + 800 + 100 + 400 + 200 + 7·1 = 11507. -/
theorem falcon_cascade_wcet_value : falcon_cascade_wcet = 11507 := by
  unfold falcon_cascade_wcet falcon_cascade_pipeline Pipeline.wcet
  unfold iekf_propagate_budget iekf_update_budget geo_desired_rate_budget
         gyro_lpf_budget adrc_tick_budget mixer_mix_budget fused_handoff_cycles
  native_decide

/-- The control-period budget: a 1 kHz outer loop on a 480 MHz Cortex-M7 gives
    480 000 cycles per tick. (The inner attitude loop runs faster, but the full
    cascade — including the IEKF updates — executes at the estimator rate.) -/
def loop_period_cycles_1khz : Nat := 480000

/-- **Schedulability**: the Falcon cascade's compositional WCET fits the 1 kHz
    control period on the M7 with >40× margin. The bound is sound by
    `pipeline_wcet_monotone_in_stages`; only the leaf budgets await on-target
    measurement (v1.11). -/
theorem falcon_cascade_schedulable_1khz :
    falcon_cascade_wcet ≤ loop_period_cycles_1khz := by
  unfold falcon_cascade_wcet falcon_cascade_pipeline Pipeline.wcet loop_period_cycles_1khz
  unfold iekf_propagate_budget iekf_update_budget geo_desired_rate_budget
         gyro_lpf_budget adrc_tick_budget mixer_mix_budget fused_handoff_cycles
  native_decide

/-- Adding a stage to the flight cascade (e.g. a future wind-estimator update)
    grows the loop WCET by at most its budget plus one fused hand-off — a direct
    instance of the compositional theorem, so re-tuning the pipeline never
    re-opens the schedulability argument. -/
theorem falcon_cascade_extensible (extra_budget : Nat) :
    ({ falcon_cascade_pipeline with
        stages := falcon_cascade_pipeline.stages ++ [extra_budget] } : Pipeline).wcet
      ≤ falcon_cascade_wcet + extra_budget + fused_handoff_cycles := by
  have := wcet_compose_monotone falcon_cascade_pipeline extra_budget
  unfold falcon_cascade_wcet
  simpa using this
