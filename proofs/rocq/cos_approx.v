(* Machine-checked APPROXIMATION-error bound for relay-math's f32 cos kernel
   (MATHF32-P05, approximation half — the Coq-Interval layer).

   CLAIM (approximation LAYER only): over the reduced range r in [-0.8, 0.8]
   — the interval the Cody-Waite reduction clamps to, containing
   [-pi/4, pi/4] — the exact real value of the cos polynomial Q(r) differs
   from the TRUE cosine by at most 1e-9:
       |Q(r) - cos r| <= 1e-9   for all r in [-0.8, 0.8].
   This is the minimax remainder. The OTHER half — that the f32 Horner
   evaluation is faithful to this same polynomial (the rounding remainder
   |Mcos - Ecos| <= 2^-23) — is proven separately by Gappa in
   proofs/gappa/cos_poly_rounding.gappa. Together they are the cosine
   analogue of the sine pair (MATHF32-P03 rounding + MATHF32-P04
   approximation).

   COMPOSITION (not asserted here): because both proofs use the IDENTICAL f32
   coefficients (see below), Ecos = Q as real functions, so a triangle
   inequality gives |Mcos - cos r| <= 2^-23 + 1e-9 ~= 1.2e-7 over the reduced
   range. Assembling that total (and carrying the reduction cancellation,
   MATHF32-P06) is a further step; this file proves ONLY the approximation
   remainder.

   NON-VACUITY (teeth): the bound is tight and the tactic genuinely refutes a
   tighter one. `interval` proves 1e-9 at i_degree 18; it FAILS to prove 1e-10
   even at i_degree 20 (and 1e-11 fails at i_degree 22). A bound that passes at
   any value would prove nothing — this one stops passing just below 1e-9.

   Note the cosine core is ~an order of magnitude more accurate than the sine
   core (1e-9 vs the sine's 1e-8, MATHF32-P04): cos_poly carries the exact
   leading terms `1 - 0.5*z` and only fits the residual, so less of the value
   comes from the minimax fit.

   Coefficients are the EXACT f32 dyadic values, copied verbatim from
   proofs/gappa/cos_poly_rounding.gappa so the two layers compose:
       C1 =  0x1.55554a0p-5  =  357913760 / 2^33
       C2 = -0x1.6c0c340p-10 = -381731648 / 2^38
       C3 =  0x1.99eb9c0p-16 =  429832640 / 2^44
   Kernel source: crates/relay-math/src/lib.rs, cos_poly():
       z = r*r;  1.0 - 0.5*z + z*z*(C1 + z*(C2 + z*C3))

   `interval` emits a Flocq/Coq-Interval proof term; rocq_interval_proof
   (rules_rocq_rust) kernel-checks it with the withPackages coqc — the tactic's
   own success is never trusted (CC-002). *)

Require Import Reals.
Require Import Interval.Tactic.
Local Open Scope R_scope.

Definition C1 : R :=  357913760 / 8589934592.
Definition C2 : R := -381731648 / 274877906944.
Definition C3 : R :=  429832640 / 17592186044416.

Definition Q (r : R) : R :=
  let z := r * r in
  1 - (1/2) * z + z * z * (C1 + z * (C2 + z * C3)).

Theorem cos_approx_bound :
  forall r : R, -8/10 <= r <= 8/10 -> Rabs (Q r - cos r) <= 1/1000000000.
Proof.
  intros r Hr. unfold Q, C1, C2, C3.
  interval with (i_bisect r, i_taylor r, i_degree 18).
Qed.
