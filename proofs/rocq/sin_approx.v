(* Machine-checked APPROXIMATION-error bound for relay-math's f32 sin kernel
   (MATHF32-P04 — the Coq-Interval layer, rules_rocq_rust#41/#43).

   CLAIM (approximation LAYER only): over the reduced range r in [-0.8, 0.8]
   — the interval the Cody-Waite reduction clamps to; note sin_poly's doc
   comment says |r| <= pi/4 (~0.785) but reduce() actually clamps to 0.8, so
   we prove the wider CLAMP range, which contains [-pi/4, pi/4] — the exact
   real value of the sin polynomial P(r) differs from the TRUE sine by at
   most 1e-8:
       |P(r) - sin r| <= 1e-8   for all r in [-0.8, 0.8].
   This is the minimax remainder — the OTHER ingredient behind the exhaustive
   1.19e-7 absolute-error bound (MATHF32-P02). The FIRST ingredient — that the
   f32 Horner evaluation is faithful to this same polynomial (the rounding
   remainder |Msin - Esin| <= 2^-24) — is proven separately by Gappa in
   proofs/gappa/sin_poly_rounding.gappa (MATHF32-P03).

   COMPOSITION (not asserted here): because both proofs use the IDENTICAL f32
   coefficients (see below), Esin = P as real functions, so a triangle
   inequality gives |Msin - sin r| <= 2^-24 + 1e-8 ~= 6.9e-8 over the reduced
   range. Assembling that total (and carrying the reduction cancellation,
   MATHF32-P06) is a further step; this file proves ONLY the approximation
   remainder. Do not read it as closing the whole envelope error.

   NON-VACUITY (teeth): the bound is tight and the tactic genuinely refutes a
   tighter one. `interval` proves 1e-8 at i_degree 15; it FAILS to prove 8e-9
   even at i_degree 18 (and 5e-9 fails at i_degree 22). A bound that passes at
   any value would prove nothing — this one stops passing just below 1e-8.

   Coefficients are the EXACT f32 dyadic values, copied verbatim from
   proofs/gappa/sin_poly_rounding.gappa so the two layers compose:
       S1 = -0x1.5555460p-3  = -357913696 / 2^31
       S2 =  0x1.11073c0p-7  =  286290880 / 2^35
       S3 = -0x1.9943f20p-13 = -429145888 / 2^41
   Kernel source: crates/relay-math/src/lib.rs, sin_poly():
       z = r*r;  r + r*z*(S1 + z*(S2 + z*S3))

   `interval` emits a Flocq/Coq-Interval proof term; rocq_interval_proof
   (rules_rocq_rust) kernel-checks it with the withPackages coqc — the tactic's
   own success is never trusted (CC-002). *)

Require Import Reals.
Require Import Interval.Tactic.
Local Open Scope R_scope.

Definition S1 : R := -357913696 / 2147483648.
Definition S2 : R :=  286290880 / 34359738368.
Definition S3 : R := -429145888 / 2199023255552.

Definition P (r : R) : R :=
  let z := r * r in
  r + r * z * (S1 + z * (S2 + z * S3)).

Theorem sin_approx_bound :
  forall r : R, -8/10 <= r <= 8/10 -> Rabs (P r - sin r) <= 1/100000000.
Proof.
  intros r Hr. unfold P, S1, S2, S3.
  interval with (i_bisect r, i_taylor r, i_degree 15).
Qed.
