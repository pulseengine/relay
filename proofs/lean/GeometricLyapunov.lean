import Mathlib.Data.Real.Basic
import Mathlib.Tactic.Ring
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Positivity
import Mathlib.Tactic.NormNum

/-!
# Geometric SE(3) Attitude Controller — Algebraic Lyapunov Certificate

The machine-checked ALGEBRAIC core of the v0.23 stability argument for the
geometric (Lee 2010) attitude controller built in `relay-geo`. Paired with
the runnable numerical certificate `lyapunov_decrease_certificate` in
`crates/relay-geo/plain/src/lib.rs` (which checks the same facts over a grid
of attitudes/rates against the real `moment()` and `psi()`).

## Scope (honest)

Mathlib (as of Feb 2026) has ODE flows but **no Lyapunov / LaSalle /
asymptotic-stability theory** (arXiv:2602.13247). A full "trajectory ⇒
converges" proof would require first formalizing LaSalle in Mathlib — out of
scope. What IS machine-checkable here, and what these theorems establish, is
the ALGEBRAIC heart on which the dynamic result rests:

  the closed-loop Lyapunov derivative collapses to `V̇ = −k_Ω‖ω‖² ≤ 0`.

The remaining steps — `V̇ = 0 ⇔ ω = 0` (the LaSalle precondition) and
"`V̇ ≤ 0` + LaSalle ⇒ `(e_R, ω) → 0` on `Ψ < 2`" — are the classical result
(Lee, Leok, McClamroch, CDC 2010, Prop. 1), cited and deferred to the
forthcoming Mathlib flow/stability API. See
`docs/research/v0.23-lean-lyapunov-sota.md`.

## Model

For the scalar-gain controller `J ω̇ᵢ = −k_R e_Rᵢ − k_Ω ωᵢ` (the gyroscopic
`ω×Jω` term cancels in `M − ω×Jω`), with Lyapunov function
`V = ½ Σ Jᵢ ωᵢ² + 2 k_R Ψ` and the configuration-error derivative identity
`Ψ̇ = ½ e_R·ω`, the cross-terms cancel. (The factor 2 / ½ is `relay-geo`'s
convention: its `attitude_error` returns the full `(R_dᵀR−RᵀR_d)^∨`, i.e.
2× Lee's `e_R`; see the crate docs.) Vectors are modelled axis-wise over ℝ
to keep the identities in the `ring` fragment.
-/

namespace RelayGeo

/-- **The crux — cross-term cancellation.** Plugging the closed-loop law
    `J ω̇ᵢ = −k_R eᵢ − k_Ω ωᵢ` into `V̇ = ω·(Jω̇) + 2k_R·Ψ̇`, with
    `2k_R·Ψ̇ = k_R (e·ω)`, every `e·ω` term cancels and only `−k_Ω‖ω‖²`
    survives. A pure algebraic identity — a sign error in the moment law
    (the v0.19–0.21 yaw saga) would make this fail to hold. -/
theorem vdot_cancellation (kR kW e1 e2 e3 w1 w2 w3 : ℝ) :
    (w1 * (-kR * e1 - kW * w1)
      + w2 * (-kR * e2 - kW * w2)
      + w3 * (-kR * e3 - kW * w3))
      + kR * (e1 * w1 + e2 * w2 + e3 * w3)
    = -kW * (w1 ^ 2 + w2 ^ 2 + w3 ^ 2) := by
  ring

/-- **Lyapunov non-increase.** With `k_Ω ≥ 0` the closed-loop derivative
    `−k_Ω‖ω‖²` is ≤ 0 everywhere — V is non-increasing along the flow. -/
theorem vdot_nonpos (kW w1 w2 w3 : ℝ) (hk : 0 ≤ kW) :
    -kW * (w1 ^ 2 + w2 ^ 2 + w3 ^ 2) ≤ 0 := by
  have hsq : (0 : ℝ) ≤ w1 ^ 2 + w2 ^ 2 + w3 ^ 2 :=
    add_nonneg (add_nonneg (sq_nonneg w1) (sq_nonneg w2)) (sq_nonneg w3)
  have hprod : (0 : ℝ) ≤ kW * (w1 ^ 2 + w2 ^ 2 + w3 ^ 2) := mul_nonneg hk hsq
  rw [neg_mul]
  linarith [hprod]

/-- **Assembled `V̇ ≤ 0`.** Composing the cancellation with non-increase:
    the closed-loop Lyapunov derivative (computed from the real moment law)
    is ≤ 0 for any attitude error `e` and body rate `ω`, given `k_Ω ≥ 0`.
    This is the machine-checked stability core; integrating it to
    `(e_R, ω) → 0` on `Ψ < 2` is Lee 2010 Prop. 1 (deferred — see header). -/
theorem lyapunov_vdot_nonpos
    (kR kW e1 e2 e3 w1 w2 w3 : ℝ) (hk : 0 ≤ kW) :
    (w1 * (-kR * e1 - kW * w1)
      + w2 * (-kR * e2 - kW * w2)
      + w3 * (-kR * e3 - kW * w3))
      + kR * (e1 * w1 + e2 * w2 + e3 * w3) ≤ 0 := by
  rw [vdot_cancellation kR kW e1 e2 e3 w1 w2 w3]
  exact vdot_nonpos kW w1 w2 w3 hk

/-- **Positive-definiteness of V.** The kinetic part `½ Σ Jᵢ ωᵢ²` is ≥ 0
    for a physical inertia (`Jᵢ ≥ 0`), and the full Lyapunov function
    `V = ½ Σ Jᵢ ωᵢ² + 2 k_R Ψ ≥ 0` given the configuration error `Ψ ≥ 0`
    (true on `Ψ < 2`) and `k_R ≥ 0`. -/
theorem V_nonneg
    (j1 j2 j3 kR psi w1 w2 w3 : ℝ)
    (hj1 : 0 ≤ j1) (hj2 : 0 ≤ j2) (hj3 : 0 ≤ j3)
    (hkR : 0 ≤ kR) (hpsi : 0 ≤ psi) :
    0 ≤ (1 / 2) * (j1 * w1 ^ 2 + j2 * w2 ^ 2 + j3 * w3 ^ 2) + 2 * kR * psi := by
  -- `positivity` can't use the sign HYPOTHESES on j1/j2/j3/kR/psi (their
  -- signs aren't syntactic); discharge each product with `mul_nonneg`.
  have h1 : 0 ≤ j1 * w1 ^ 2 := mul_nonneg hj1 (sq_nonneg w1)
  have h2 : 0 ≤ j2 * w2 ^ 2 := mul_nonneg hj2 (sq_nonneg w2)
  have h3 : 0 ≤ j3 * w3 ^ 2 := mul_nonneg hj3 (sq_nonneg w3)
  have h4 : 0 ≤ 2 * kR * psi := mul_nonneg (mul_nonneg (by norm_num : (0:ℝ) ≤ 2) hkR) hpsi
  linarith [h1, h2, h3, h4]

end RelayGeo
