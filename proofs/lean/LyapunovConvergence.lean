import Mathlib

/-!
# Lyapunov exponential convergence — closing the deferred dynamic step

Every prior Lyapunov file in this directory proves a differential *inequality*
on the closed-loop Lyapunov value and then **defers** the final "trajectory ⇒
converges" step, citing LaSalle — which Mathlib does not have.

`StrictLyapunov.lean` upgraded that inequality to the negative-*definite*
exponential form `V̇ ≤ −γ·V` (γ>0), realised on the real relay-geo controller
by `strict_lyapunov_decrease_certificate`. This file discharges the remaining
step for that exponential form — **without LaSalle and without the ODE flow
API** — by the classical integrating-factor argument:

  with `w(t) = v(t)·exp(γt)`, `w'(t) = (v'(t) + γ v(t))·exp(γt) ≤ 0`, so `w` is
  antitone; hence `v(t)·exp(γt) ≤ v(0)`, i.e. `v(t) ≤ v(0)·exp(−γt) → 0`.

So the closed loop's Lyapunov value decays exponentially and the state
converges. The only remaining modelling assumption — that the scalar
`v(t) = V(state(t))` is differentiable along the flow with `deriv v = V̇` (the
quantity `StrictLyapunov`/relay-geo bound) — is an explicit hypothesis here,
the honest interface to the vector-field-level certificates. No sorry, no axiom.
-/

namespace RelayGeoConv

open Real Filter Topology

/-- **Integrating factor is antitone.** If `v` is differentiable and obeys the
    exponential differential inequality `v'(t) ≤ −γ·v(t)` everywhere, then
    `w(t) = v(t)·exp(γt)` has nonpositive derivative, hence is antitone. -/
theorem intfactor_antitone (v : ℝ → ℝ) (γ : ℝ)
    (hv : Differentiable ℝ v) (hineq : ∀ t, deriv v t ≤ -γ * v t) :
    Antitone (fun t => v t * Real.exp (γ * t)) := by
  have hw : Differentiable ℝ (fun t => v t * Real.exp (γ * t)) :=
    hv.mul ((differentiable_id.const_mul γ).exp)
  refine antitone_of_deriv_nonpos hw (fun x => ?_)
  have hd_lin : HasDerivAt (fun t : ℝ => γ * t) γ x := by
    simpa using (hasDerivAt_id x).const_mul γ
  have hd_exp : HasDerivAt (fun t => Real.exp (γ * t)) (Real.exp (γ * x) * γ) x :=
    hd_lin.exp
  have hd_v : HasDerivAt v (deriv v x) x := hv.differentiableAt.hasDerivAt
  have hd_w : HasDerivAt (fun t => v t * Real.exp (γ * t))
      (deriv v x * Real.exp (γ * x) + v x * (Real.exp (γ * x) * γ)) x :=
    hd_v.mul hd_exp
  rw [hd_w.deriv]
  have hexp : (0 : ℝ) < Real.exp (γ * x) := Real.exp_pos _
  have ha : deriv v x + γ * v x ≤ 0 := by nlinarith [hineq x]
  have heq : deriv v x * Real.exp (γ * x) + v x * (Real.exp (γ * x) * γ)
           = Real.exp (γ * x) * (deriv v x + γ * v x) := by ring
  rw [heq]
  exact mul_nonpos_of_nonneg_of_nonpos (le_of_lt hexp) ha

/-- **Exponential-decay bound.** The Lyapunov value is dominated by its
    initial value times `exp(−γt)`: `v(t) ≤ v(0)·exp(−γt)` for `t ≥ 0`. This is
    the machine-checked discharge of the previously-deferred integration step,
    for the strict (negative-definite) Lyapunov inequality. -/
theorem exp_decay_bound (v : ℝ → ℝ) (γ : ℝ)
    (hv : Differentiable ℝ v) (hineq : ∀ t, deriv v t ≤ -γ * v t)
    {t : ℝ} (ht : 0 ≤ t) :
    v t ≤ v 0 * Real.exp (-γ * t) := by
  have h := intfactor_antitone v γ hv hineq ht
  -- `Antitone`-application leaves the lambda unreduced; `simp only` beta-reduces
  -- and simplifies `exp (γ*0) = 1`.
  simp only [mul_zero, Real.exp_zero, mul_one] at h
  have hmul := mul_le_mul_of_nonneg_right h (le_of_lt (Real.exp_pos (-γ * t)))
  have heq : v t * Real.exp (γ * t) * Real.exp (-γ * t) = v t := by
    have h0 : γ * t + -γ * t = 0 := by ring
    rw [mul_assoc, ← Real.exp_add, h0, Real.exp_zero, mul_one]
  rwa [heq] at hmul

/-- **Asymptotic convergence — the deferred result, discharged.** For a
    nonnegative Lyapunov value obeying `v'(t) ≤ −γ·v(t)` with `γ > 0`, the state
    Lyapunov value tends to `0` as `t → ∞`. This is "trajectory ⇒ converges"
    for the exponential Lyapunov certificate — via integrating factor + squeeze,
    NOT LaSalle. -/
theorem tendsto_zero_of_diff_ineq (v : ℝ → ℝ) (γ : ℝ) (hγ : 0 < γ)
    (hv : Differentiable ℝ v) (hineq : ∀ t, deriv v t ≤ -γ * v t)
    (hnonneg : ∀ t, 0 ≤ v t) :
    Tendsto v atTop (nhds 0) := by
  have hg : Tendsto (fun t : ℝ => v 0 * Real.exp (-γ * t)) atTop (nhds 0) := by
    have hlin : Tendsto (fun t : ℝ => γ * t) atTop atTop :=
      Filter.Tendsto.const_mul_atTop hγ tendsto_id
    have hexp : Tendsto (fun t : ℝ => Real.exp (-(γ * t))) atTop (nhds 0) :=
      Real.tendsto_exp_neg_atTop_nhds_zero.comp hlin
    have hmul : Tendsto (fun t : ℝ => v 0 * Real.exp (-(γ * t))) atTop (nhds (v 0 * 0)) :=
      hexp.const_mul (v 0)
    simpa [neg_mul, mul_zero] using hmul
  refine squeeze_zero' ?_ ?_ hg
  · exact Filter.Eventually.of_forall (fun t => hnonneg t)
  · filter_upwards [eventually_ge_atTop (0 : ℝ)] with t ht
    exact exp_decay_bound v γ hv hineq ht

end RelayGeoConv
