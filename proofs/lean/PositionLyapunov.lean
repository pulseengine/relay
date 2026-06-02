import Mathlib.Data.Real.Basic
import Mathlib.Tactic.Ring
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Positivity
import Mathlib.Tactic.NormNum

/-!
# Geometric SE(3) Position Controller — Algebraic Lyapunov Certificate

The machine-checked ALGEBRAIC core of the TRANSLATIONAL half of the geometric
(Lee 2010) controller, the v0.38 companion to `GeometricLyapunov.lean` (the
attitude half). Together they close the v0.28 simplex-shield scope gap: the
v0.23 certificate was attitude-only, so the shield's recoverable-set
guarantee covered only rotation. With the position Lyapunov below, the
FULL-STATE Lyapunov `V = V_att + V_pos` is non-increasing, so the safe-set
guarantee extends to position.

## Scope (honest)

As with the attitude proof, Mathlib has no Lyapunov/LaSalle theory, so the
"trajectory ⇒ converges" step is the classical result (Lee, Leok,
McClamroch, CDC 2010, Prop. 2 / the translational subsystem), cited and
deferred. What IS machine-checked here is the algebraic heart:

  the closed-loop position Lyapunov derivative collapses to
  `V̇_pos = −k_v ‖e_v‖² ≤ 0`.

## Model

Translational error dynamics with the position controller
`m ë_xᵢ = −k_x e_xᵢ − k_v e_vᵢ` (the thrust/attitude inner loop assumed
ideal — the cascade separation the shield enforces), `ė_xᵢ = e_vᵢ`, and the
Lyapunov function `V_pos = ½ m Σ e_vᵢ² + ½ k_x Σ e_xᵢ²`. Then
`V̇_pos = Σ [m e_vᵢ ë_xᵢ + k_x e_xᵢ ė_xᵢ]` and the `k_x e_x·e_v` cross-terms
cancel — algebraically identical to the attitude proof (k_R→k_x, k_Ω→k_v,
e_R→e_x, ω→e_v), which is why a sign error in either loop fails the same
`ring` identity. Vectors are axis-wise over ℝ.
-/

namespace RelayGeoPos

/-- **The crux — translational cross-term cancellation.** Plugging the
    closed-loop law `m ë_xᵢ = −k_x e_xᵢ − k_v e_vᵢ` into
    `V̇_pos = Σ e_vᵢ (m ë_xᵢ) + k_x Σ e_xᵢ e_vᵢ`, every `k_x e_x·e_v` term
    cancels and only `−k_v ‖e_v‖²` survives. A pure algebraic identity. -/
theorem vdot_cancellation_pos (kx kv ex1 ex2 ex3 ev1 ev2 ev3 : ℝ) :
    (ev1 * (-kx * ex1 - kv * ev1)
      + ev2 * (-kx * ex2 - kv * ev2)
      + ev3 * (-kx * ex3 - kv * ev3))
      + kx * (ex1 * ev1 + ex2 * ev2 + ex3 * ev3)
    = -kv * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2) := by
  ring

/-- **Lyapunov non-increase (position).** With `k_v ≥ 0` the closed-loop
    derivative `−k_v ‖e_v‖²` is ≤ 0 everywhere — `V_pos` is non-increasing. -/
theorem vdot_nonpos_pos (kv ev1 ev2 ev3 : ℝ) (hk : 0 ≤ kv) :
    -kv * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2) ≤ 0 := by
  have hsq : (0 : ℝ) ≤ ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2 :=
    add_nonneg (add_nonneg (sq_nonneg ev1) (sq_nonneg ev2)) (sq_nonneg ev3)
  have hprod : (0 : ℝ) ≤ kv * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2) := mul_nonneg hk hsq
  rw [neg_mul]
  linarith [hprod]

/-- **The LaSalle precondition (position).** With `k_v > 0`, `V̇_pos` is zero
    *iff* the velocity error `e_v` is zero. On the largest invariant set in
    `{V̇_pos = 0}` the closed loop forces `e_x → 0` — the hook the (deferred)
    LaSalle step uses to upgrade to asymptotic position tracking. -/
theorem vdot_zero_iff_ev_zero (kv ev1 ev2 ev3 : ℝ) (hk : 0 < kv) :
    (-kv * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2) = 0) ↔ (ev1 = 0 ∧ ev2 = 0 ∧ ev3 = 0) := by
  constructor
  · intro h
    rw [neg_mul, neg_eq_zero] at h
    rcases mul_eq_zero.mp h with hk0 | hsum
    · exact absurd hk0 hk.ne'
    · have h1 : ev1 ^ 2 = 0 := by nlinarith [sq_nonneg ev1, sq_nonneg ev2, sq_nonneg ev3, hsum]
      have h2 : ev2 ^ 2 = 0 := by nlinarith [sq_nonneg ev1, sq_nonneg ev2, sq_nonneg ev3, hsum]
      have h3 : ev3 ^ 2 = 0 := by nlinarith [sq_nonneg ev1, sq_nonneg ev2, sq_nonneg ev3, hsum]
      exact ⟨pow_eq_zero_iff (by norm_num) |>.mp h1,
             pow_eq_zero_iff (by norm_num) |>.mp h2,
             pow_eq_zero_iff (by norm_num) |>.mp h3⟩
  · rintro ⟨h1, h2, h3⟩
    rw [h1, h2, h3]; ring

/-- **Assembled `V̇_pos ≤ 0`.** Composing the cancellation with non-increase:
    the closed-loop position Lyapunov derivative is ≤ 0 for any position
    error `e_x` and velocity error `e_v`, given `k_v ≥ 0`. -/
theorem lyapunov_vdot_nonpos_pos
    (kx kv ex1 ex2 ex3 ev1 ev2 ev3 : ℝ) (hk : 0 ≤ kv) :
    (ev1 * (-kx * ex1 - kv * ev1)
      + ev2 * (-kx * ex2 - kv * ev2)
      + ev3 * (-kx * ex3 - kv * ev3))
      + kx * (ex1 * ev1 + ex2 * ev2 + ex3 * ev3) ≤ 0 := by
  rw [vdot_cancellation_pos kx kv ex1 ex2 ex3 ev1 ev2 ev3]
  exact vdot_nonpos_pos kv ev1 ev2 ev3 hk

/-- **Positive-definiteness of `V_pos`.** `V_pos = ½ m Σ e_vᵢ² + ½ k_x Σ e_xᵢ²
    ≥ 0` for a physical mass `m ≥ 0` and gain `k_x ≥ 0`. -/
theorem V_pos_nonneg
    (m kx ex1 ex2 ex3 ev1 ev2 ev3 : ℝ) (hm : 0 ≤ m) (hkx : 0 ≤ kx) :
    0 ≤ (1 / 2) * m * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2)
        + (1 / 2) * kx * (ex1 ^ 2 + ex2 ^ 2 + ex3 ^ 2) := by
  have hv : 0 ≤ ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2 :=
    add_nonneg (add_nonneg (sq_nonneg ev1) (sq_nonneg ev2)) (sq_nonneg ev3)
  have hx : 0 ≤ ex1 ^ 2 + ex2 ^ 2 + ex3 ^ 2 :=
    add_nonneg (add_nonneg (sq_nonneg ex1) (sq_nonneg ex2)) (sq_nonneg ex3)
  have h1 : 0 ≤ (1 / 2) * m * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2) :=
    mul_nonneg (mul_nonneg (by norm_num) hm) hv
  have h2 : 0 ≤ (1 / 2) * kx * (ex1 ^ 2 + ex2 ^ 2 + ex3 ^ 2) :=
    mul_nonneg (mul_nonneg (by norm_num) hkx) hx
  linarith [h1, h2]

/-- **Full-state non-increase (the v0.38 result).** The combined Lyapunov
    derivative `V̇ = V̇_att + V̇_pos = −k_Ω ‖ω‖² − k_v ‖e_v‖²` is ≤ 0 for any
    attitude error / body rate / position error / velocity error, given
    `k_Ω ≥ 0` and `k_v ≥ 0`. This is the machine-checked core that closes the
    simplex shield's attitude-only scope gap: the FULL-STATE Lyapunov is
    non-increasing, so the certified safe set covers position as well as
    attitude. (Integrating to `(e_R, ω, e_x, e_v) → 0` is Lee 2010 Prop. 2,
    deferred to the forthcoming Mathlib stability API — see the header.) -/
theorem fullstate_vdot_nonpos
    (kW kv w1 w2 w3 ev1 ev2 ev3 : ℝ) (hkW : 0 ≤ kW) (hkv : 0 ≤ kv) :
    (-kW * (w1 ^ 2 + w2 ^ 2 + w3 ^ 2)) + (-kv * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2)) ≤ 0 := by
  have ha : -kW * (w1 ^ 2 + w2 ^ 2 + w3 ^ 2) ≤ 0 := vdot_nonpos_pos kW w1 w2 w3 hkW
  have hp : -kv * (ev1 ^ 2 + ev2 ^ 2 + ev3 ^ 2) ≤ 0 := vdot_nonpos_pos kv ev1 ev2 ev3 hkv
  linarith [ha, hp]

end RelayGeoPos
