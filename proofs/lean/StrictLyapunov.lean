import Mathlib.Data.Real.Basic
import Mathlib.Tactic.Ring
import Mathlib.Tactic.Linarith
import Mathlib.Tactic.Positivity
import Mathlib.Tactic.NormNum

/-!
# Strict (cross-term) Lyapunov certificate — the exponential-decay inequality

The v1.105 advance over `GeometricLyapunov.lean` / `PositionLyapunov.lean`.
Those files prove the closed-loop derivative is negative *semi*-definite
(`V̇ = −k_Ω‖ω‖² ≤ 0`) and cite the trajectory-integration step as **deferred**,
because integrating a *semi*-definite `V̇ ≤ 0` to convergence needs LaSalle's
invariance principle — which Mathlib (Feb 2026) does **not** have.

This file removes that dependence on LaSalle. Following Lee–Leok–McClamroch
2010 **Prop. 2**, a *strict* (cross-term–coupled) Lyapunov function
`V = ½ e_Ω·J e_Ω + k_R Ψ + c⟨e_R,e_Ω⟩` makes `V̇` negative-*definite*: in the
reduced coordinates `z = (‖e_R‖, ‖e_Ω‖) =: (r, s)`,

  * `V`  is sandwiched by quadratic forms `c_lo(r²+s²) ≤ V ≤ c_hi(r²+s²)`,
  * `−V̇` dominates a quadratic form `c_D(r²+s²) ≤ −V̇`,

both certified by **Sylvester's 2×2 criterion** (leading coeff > 0 and
determinant ≥ 0). Combining them gives the **exponential-decay differential
inequality**

  `c_hi · V̇  ≤  −c_D · V`        (equivalently `V̇ ≤ −(c_D/c_hi) V`).

That is exactly the hypothesis of **Grönwall's inequality**, which Mathlib
*does* now have (arXiv:2602.13247 flows/Grönwall). So the remaining deferred
step shrinks from "formalize LaSalle" (multi-year) to "apply Mathlib's
Grönwall to the closed-loop flow" — `V(t) ≤ V(0)·exp(−(c_D/c_hi)·t) → 0`.

## Scope (honest)

What is machine-checked here is the **algebraic certificate**: the Sylvester
positivity facts and the assembly into `c_hi·V̇ ≤ −c_D·V`, parameterised by
the bound constants. The bound constants themselves arise from the controller
gains + the coupling `c` + the configuration-error bound `‖e_R‖² ≤ Ψ(2−Ψ)` on
`Ψ < 2` (Lee 2010); supplying them for the real `relay-geo` gains, and the
Grönwall flow-integration, are the next slices. Everything below is in the
pure `ring` / `nlinarith` fragment — no `sorry`, no axiom.
-/

namespace RelayGeoStrict

/-- **Completing the square (pure ring identity).** The 2×2 quadratic form
    scaled by its leading coefficient is a sum of two squares plus the
    determinant times `y²`. This is the algebraic heart of Sylvester's
    criterion — a sign slip in any matrix entry would break it. -/
theorem quad_complete (a b d x y : ℝ) :
    a * (a * x ^ 2 + 2 * b * x * y + d * y ^ 2)
      = (a * x + b * y) ^ 2 + (a * d - b ^ 2) * y ^ 2 := by
  ring

/-- **Sylvester's 2×2 criterion (positive semidefinite).** A symmetric form
    `a x² + 2b xy + d y²` is ≥ 0 for all `(x,y)` when the leading coefficient
    is positive and the determinant is nonnegative (`b² ≤ a d`). -/
theorem quad_form_nonneg (a b d x y : ℝ) (ha : 0 < a) (hdet : b ^ 2 ≤ a * d) :
    0 ≤ a * x ^ 2 + 2 * b * x * y + d * y ^ 2 := by
  -- a·(form) = (a x + b y)² + (a d − b²) y² ≥ 0, and a > 0 ⇒ form ≥ 0.
  nlinarith [sq_nonneg (a * x + b * y),
             mul_nonneg (sub_nonneg.mpr hdet) (sq_nonneg y),
             ha, sq_nonneg x, sq_nonneg y]

/-- **Radial lower bound.** If the *shifted* matrix `(a−λ, b, d−λ)` still meets
    Sylvester, the form dominates `λ(x²+y²)`. Used to extract the dissipation
    floor `c_D` from `−V̇` and the positive-definite floor `c_lo` from `V`. -/
theorem quad_radial_lower (a b d lam x y : ℝ)
    (ha : 0 < a - lam) (hdet : b ^ 2 ≤ (a - lam) * (d - lam)) :
    lam * (x ^ 2 + y ^ 2) ≤ a * x ^ 2 + 2 * b * x * y + d * y ^ 2 := by
  have h := quad_form_nonneg (a - lam) b (d - lam) x y ha hdet
  nlinarith [h]

/-- **Radial upper bound.** Symmetric to `quad_radial_lower` with the matrix
    `(Λ−a, −b, Λ−d)`: the form is capped by `Λ(x²+y²)`. Gives the radial cap
    `c_hi` on `V`. -/
theorem quad_radial_upper (a b d Lam x y : ℝ)
    (ha : 0 < Lam - a) (hdet : b ^ 2 ≤ (Lam - a) * (Lam - d)) :
    a * x ^ 2 + 2 * b * x * y + d * y ^ 2 ≤ Lam * (x ^ 2 + y ^ 2) := by
  have h := quad_form_nonneg (Lam - a) (-b) (Lam - d) x y ha (by nlinarith [hdet])
  nlinarith [h]

/-- **Exponential-decay differential inequality (division-free).** From a
    radial cap `V ≤ c_hi(r²+s²)` and a dissipation floor `c_D(r²+s²) ≤ −V̇`,
    with `c_hi > 0` and `c_D ≥ 0`, the Lyapunov value satisfies
    `c_hi · V̇ ≤ −c_D · V`. Dividing by `c_hi` gives `V̇ ≤ −(c_D/c_hi) V`, the
    Grönwall hypothesis whose flow-integration yields exponential decay. -/
theorem exp_decay_inequality
    (V Vdot r2 s2 cHi cD : ℝ)
    (hchi : 0 < cHi) (hcd : 0 ≤ cD)
    (hVhi : V ≤ cHi * (r2 + s2))
    (hVd : Vdot ≤ -cD * (r2 + s2)) :
    cHi * Vdot ≤ -cD * V := by
  have h1 : cHi * Vdot ≤ cHi * (-cD * (r2 + s2)) :=
    mul_le_mul_of_nonneg_left hVd (le_of_lt hchi)
  have h2 : cD * V ≤ cD * (cHi * (r2 + s2)) :=
    mul_le_mul_of_nonneg_left hVhi hcd
  nlinarith [h1, h2]

/-- **The assembled strict-Lyapunov exponential certificate (Lee 2010 Prop. 2,
    algebraic core).** In reduced coordinates `(r,s) = (‖e_R‖, ‖e_Ω‖)`, given

      * `V = a_V r² + 2 b_V r s + d_V s²`, with `(c_hi − a_V, b_V, c_hi − d_V)`
        Sylvester-positive (a radial cap `c_hi`) and `a_V ≥ 0`, and
      * `−V̇ = a_D r² + 2 b_D r s + d_D s²`, with `(a_D − c_D, b_D, d_D − c_D)`
        Sylvester-positive (a dissipation floor `c_D ≥ 0`),

    the closed loop obeys `c_hi · V̇ ≤ −c_D · V`. This is the negative-definite
    (strict) upgrade of `GeometricLyapunov.lyapunov_vdot_nonpos`; with
    `c_D > 0` it is the exponential-stability inequality, no LaSalle required —
    only Grönwall on the flow (deferred to the Mathlib flow API). -/
theorem strict_lyapunov_exp_decay
    (r s V Vdot : ℝ)
    (aV bV dV cHi : ℝ) (haV : 0 ≤ aV)
    (hcap : 0 < cHi - aV) (hcapd : bV ^ 2 ≤ (cHi - aV) * (cHi - dV))
    (hVdef : V = aV * r ^ 2 + 2 * bV * r * s + dV * s ^ 2)
    (aD bD dD cD : ℝ) (hcd : 0 ≤ cD)
    (hflo : 0 < aD - cD) (hflod : bD ^ 2 ≤ (aD - cD) * (dD - cD))
    (hVddef : -Vdot = aD * r ^ 2 + 2 * bD * r * s + dD * s ^ 2) :
    cHi * Vdot ≤ -cD * V := by
  have hchi : 0 < cHi := by linarith [hcap, haV]
  have hVhi : V ≤ cHi * (r ^ 2 + s ^ 2) := by
    rw [hVdef]; exact quad_radial_upper aV bV dV cHi r s hcap hcapd
  have hdis : cD * (r ^ 2 + s ^ 2) ≤ -Vdot := by
    rw [hVddef]; exact quad_radial_lower aD bD dD cD r s hflo hflod
  have hVd : Vdot ≤ -cD * (r ^ 2 + s ^ 2) := by linarith [hdis]
  exact exp_decay_inequality V Vdot (r ^ 2) (s ^ 2) cHi cD hchi hcd hVhi hVd

/-- **relay-geo concrete rate (v1.106).** The runnable certificate
    `strict_lyapunov_decrease_certificate` (crates/relay-geo) measures, over a
    grid on Ψ<2 for the REAL controller (gains k_R=8, k_Ω=2,
    J=diag(0.0217,0.0217,0.04), cross-term coupling c=0.02), the radial bounds
    `V ≤ 32(r²+s²)` and a strictly positive dissipation floor `1(r²+s²) ≤ −V̇`
    (measured c_hi≈31.9 and c_D≈1.90; here we take the conservative INTEGER
    bounds c_hi=32, c_D=1 to keep the certificate literal-robust). Instantiating
    `exp_decay_inequality` at those constants yields `32·V̇ ≤ −1·V`: the deployed
    closed loop decays at rate γ ≥ 1/32 > 0 (the measured rate ≈0.056 is
    higher). This kernel-checked corollary ties the abstract strict certificate
    to the numerically-established bounds of the real geometric controller. -/
theorem relay_geo_exp_rate (V Vdot r2 s2 : ℝ)
    (hVhi : V ≤ 32 * (r2 + s2)) (hVd : Vdot ≤ -1 * (r2 + s2)) :
    32 * Vdot ≤ -1 * V :=
  exp_decay_inequality V Vdot r2 s2 32 1 (by norm_num) (by norm_num) hVhi hVd

/-- **Non-vacuity witness.** The certificate's hypotheses are satisfiable: with
    the unit-ish forms `V = r² + s²` (`a_V=d_V=1, b_V=0, c_hi=2`) and
    `−V̇ = 2r² + 2s²` (`a_D=d_D=2, b_D=0, c_D=1`), every Sylvester side
    condition holds, so the certificate yields `2·V̇ ≤ −V` — a concrete
    exponential rate `γ = 1/2`. Guards against a true-but-empty theorem. -/
example (r s : ℝ) :
    (2 : ℝ) * (-(2 * r ^ 2 + 2 * s ^ 2)) ≤ -1 * (r ^ 2 + s ^ 2) := by
  have h := strict_lyapunov_exp_decay
    r s (r ^ 2 + s ^ 2) (-(2 * r ^ 2 + 2 * s ^ 2))
    1 0 1 2 (by norm_num) (by norm_num) (by norm_num) (by ring)
    2 0 2 1 (by norm_num) (by norm_num) (by norm_num) (by ring)
  -- h : 2 * (-(2 r² + 2 s²)) ≤ -1 * (r² + s²)
  linarith [h]

end RelayGeoStrict
