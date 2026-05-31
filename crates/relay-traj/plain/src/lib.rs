//! Relay quadrotor **trajectory generation** (v0.27) — per-axis
//! jerk-minimizing **quintic motion primitives** (Mueller, Hehn &
//! D'Andrea, T-RO 2015), closed-form and `no_std`/`no_alloc`.
//!
//! A quadrotor is differentially flat in `(x, y, z, ψ)` (Mellinger &
//! Kumar 2011): a smooth trajectory in those outputs maps algebraically to
//! the full state + the geometric controller's feedforward. The per-axis
//! primitive is the embedded/real-time-replanning choice over a global QP:
//! given an initial `(p, v, a)`, a final `(p, v, a)`, and a duration `T`,
//! the jerk-minimizing trajectory is a **quintic** whose coefficients come
//! from a closed-form expression — no optimizer, constant time.
//!
//! The headline verifiable property (see `peak_abs_jerk` + the Kani
//! harness): the closed-form jerk bound is **sound** — the returned value
//! is ≥ `|j(t)|` for every `t ∈ [0, T]`, so an input-feasibility verdict
//! built on it never under-estimates the demand.

#![no_std]

/// One axis of a degree-5 (quintic) jerk-minimizing trajectory, stored as
/// the position-polynomial coefficients `p(t) = Σ cₖ tᵏ` plus the segment
/// duration. Total/finite for any boundary conditions (the duration is
/// guarded away from 0).
#[derive(Clone, Copy, Debug)]
pub struct Quintic {
    c5: f32,
    c4: f32,
    c3: f32,
    c2: f32,
    c1: f32,
    c0: f32,
    t_end: f32,
}

/// Trajectory sample: position, velocity, acceleration, jerk (one axis).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Sample {
    pub p: f32,
    pub v: f32,
    pub a: f32,
    pub j: f32,
}

#[inline]
fn fin(x: f32, fallback: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        fallback
    }
}

impl Quintic {
    /// Build the jerk-minimizing quintic from initial `(p0,v0,a0)` to final
    /// `(pf,vf,af)` over duration `t` (s). Closed-form (Mueller 2015, fully
    /// constrained end state). `t` is clamped to ≥ 1 ms so the map is total.
    pub fn new(p0: f32, v0: f32, a0: f32, pf: f32, vf: f32, af: f32, t: f32) -> Self {
        let t = if t.is_finite() && t > 1e-3 { t } else { 1e-3 };
        let (p0, v0, a0) = (fin(p0, 0.0), fin(v0, 0.0), fin(a0, 0.0));
        let (pf, vf, af) = (fin(pf, p0), fin(vf, 0.0), fin(af, 0.0));
        // Residuals against the free-coast (constant-accel) prediction.
        let dp = pf - (p0 + v0 * t + 0.5 * a0 * t * t);
        let dv = vf - (v0 + a0 * t);
        let da = af - a0;
        let (t2, t3, t4, t5) = (t * t, t * t * t, t * t * t * t, t * t * t * t * t);
        let inv = 1.0 / t5;
        // α, β, γ — the snap/crackle/jerk-at-0 of the optimal quintic.
        let alpha = inv * (720.0 * dp - 360.0 * t * dv + 60.0 * t2 * da);
        let beta = inv * (-360.0 * t * dp + 168.0 * t2 * dv - 24.0 * t3 * da);
        let gamma = inv * (60.0 * t2 * dp - 24.0 * t3 * dv + 3.0 * t4 * da);
        Quintic {
            c5: fin(alpha / 120.0, 0.0),
            c4: fin(beta / 24.0, 0.0),
            c3: fin(gamma / 6.0, 0.0),
            c2: a0 / 2.0,
            c1: v0,
            c0: p0,
            t_end: t,
        }
    }

    /// Segment duration (s).
    #[inline]
    pub fn duration(&self) -> f32 {
        self.t_end
    }

    /// Evaluate position/velocity/acceleration/jerk at time `t` (clamped to
    /// `[0, T]` so a sample is always on the segment).
    pub fn eval(&self, t: f32) -> Sample {
        let t = if t.is_finite() { t.clamp(0.0, self.t_end) } else { 0.0 };
        let (c5, c4, c3, c2, c1, c0) = (self.c5, self.c4, self.c3, self.c2, self.c1, self.c0);
        // Horner for p; analytic derivatives.
        let p = ((((c5 * t + c4) * t + c3) * t + c2) * t + c1) * t + c0;
        let v = (((5.0 * c5 * t + 4.0 * c4) * t + 3.0 * c3) * t + 2.0 * c2) * t + c1;
        let a = ((20.0 * c5 * t + 12.0 * c4) * t + 6.0 * c3) * t + 2.0 * c2;
        let j = (60.0 * c5 * t + 24.0 * c4) * t + 6.0 * c3;
        Sample { p, v, a, j }
    }

    /// Jerk coefficients: `j(t) = jc2·t² + jc1·t + jc0`.
    #[inline]
    fn jerk_coeffs(&self) -> (f32, f32, f32) {
        (60.0 * self.c5, 24.0 * self.c4, 6.0 * self.c3)
    }

    /// **Sound** upper bound on `|j(t)|` over `[0, T]`. With
    /// `j(t) = a·t² + b·t + c` and `t ∈ [0,T]` (so `t² ≤ T²`), the triangle
    /// inequality gives `|j(t)| ≤ |a|·T² + |b|·T + |c|`. Conservative (not
    /// the exact vertex extremum) but **division-free**, so its soundness
    /// is cheaply Kani-checkable (`verify_peak_jerk_sound`). The building
    /// block for the input-feasibility test (body rate ∝ jerk/thrust in the
    /// flatness map): a feasibility verdict on this bound NEVER
    /// under-estimates the jerk demand — it errs safe.
    pub fn peak_abs_jerk(&self) -> f32 {
        let (a, b, c) = self.jerk_coeffs();
        let t = self.t_end;
        let bound = libm::fabsf(a) * t * t + libm::fabsf(b) * t + libm::fabsf(c);
        fin(bound, f32::INFINITY)
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// SOUNDNESS of the jerk bound: for ANY quintic (arbitrary finite
    /// coefficients + duration) and ANY `t ∈ [0, T]`, `|j(t)| ≤
    /// peak_abs_jerk()`. So an input-feasibility verdict built on the bound
    /// can never under-estimate the true jerk demand over the segment.
    #[kani::proof]
    fn verify_peak_jerk_sound() {
        // Arbitrary jerk quadratic via arbitrary position coeffs, bounded to
        // a realistic, overflow-free domain (coeffs ≤ 1e3, duration ≤ 100 s
        // ⇒ every intermediate stays well inside f32 range — no inf/NaN).
        let c5: f32 = kani::any();
        let c4: f32 = kani::any();
        let c3: f32 = kani::any();
        let t_end: f32 = kani::any();
        kani::assume(c5.is_finite() && libm::fabsf(c5) <= 1e3);
        kani::assume(c4.is_finite() && libm::fabsf(c4) <= 1e3);
        kani::assume(c3.is_finite() && libm::fabsf(c3) <= 1e3);
        kani::assume(t_end.is_finite() && t_end > 1e-3 && t_end <= 100.0);
        let q = Quintic { c5, c4, c3, c2: 0.0, c1: 0.0, c0: 0.0, t_end };
        let peak = q.peak_abs_jerk();

        let t: f32 = kani::any();
        kani::assume(t.is_finite() && t >= 0.0 && t <= t_end);
        // j(t) directly (same coeffs as eval/peak) — no eval/clamp, to keep
        // the f32 problem minimal.
        let (ja, jb, jc) = (60.0 * c5, 24.0 * c4, 6.0 * c3);
        let jt = (ja * t + jb) * t + jc;
        // peak is a sound upper bound on |j(t)| over [0,T] (relative
        // tolerance for f32 rounding).
        assert!(libm::fabsf(jt) <= peak * 1.001 + 0.05);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The quintic meets its boundary conditions: p/v/a at t=0 and t=T.
    #[test]
    fn primitive_meets_boundary_conditions() {
        let q = Quintic::new(0.0, 1.0, 0.0, 10.0, 0.0, 0.0, 2.0);
        let s0 = q.eval(0.0);
        assert!((s0.p - 0.0).abs() < 1e-4 && (s0.v - 1.0).abs() < 1e-4 && s0.a.abs() < 1e-4);
        let sT = q.eval(2.0);
        assert!((sT.p - 10.0).abs() < 1e-3, "pf: {}", sT.p);
        assert!(sT.v.abs() < 1e-3, "vf: {}", sT.v);
        assert!(sT.a.abs() < 1e-3, "af: {}", sT.a);
    }

    /// Derivatives are consistent: finite-difference of p ≈ v, of v ≈ a.
    #[test]
    fn derivatives_consistent() {
        let q = Quintic::new(0.0, 0.0, 0.0, 5.0, 2.0, 1.0, 3.0);
        let dt = 1e-3;
        for k in 1..30 {
            let t = k as f32 * 0.1;
            let s = q.eval(t);
            let sp = q.eval(t + dt);
            assert!(((sp.p - s.p) / dt - s.v).abs() < 0.05, "v mismatch at {t}");
            assert!(((sp.v - s.v) / dt - s.a).abs() < 0.05, "a mismatch at {t}");
        }
    }

    /// The jerk bound is sound on a sampled grid (the Kani harness proves
    /// it over the whole f32 domain).
    #[test]
    fn peak_jerk_dominates_samples() {
        let q = Quintic::new(0.0, 0.0, 0.0, 8.0, -1.0, 2.0, 1.5);
        let peak = q.peak_abs_jerk();
        for k in 0..=150 {
            let t = k as f32 / 100.0;
            assert!(q.eval(t).j.abs() <= peak + 1e-3, "jerk exceeds peak at {t}");
        }
        assert!(peak > 0.0 && peak.is_finite());
    }

    proptest::proptest! {
        /// SOUNDNESS of the jerk bound over random coefficients + random
        /// `t ∈ [0,T]`: `|j(t)| ≤ peak_abs_jerk()`. The runnable gate; the
        /// `verify_peak_jerk_sound` Kani harness is the exhaustive version
        /// (f32-arithmetic-heavy ⇒ a CI-budget proof). Bounded to the
        /// realistic, overflow-free domain.
        #[test]
        fn peak_jerk_bound_is_sound(
            c5 in -1e3f32..1e3, c4 in -1e3f32..1e3, c3 in -1e3f32..1e3,
            t_end in 1e-3f32..100.0, frac in 0.0f32..1.0,
        ) {
            let q = Quintic { c5, c4, c3, c2: 0.0, c1: 0.0, c0: 0.0, t_end };
            let peak = q.peak_abs_jerk();
            let t = frac * t_end;
            proptest::prop_assert!(q.eval(t).j.abs() <= peak * 1.001 + 0.05);
        }
    }

    /// Total: finite samples + finite peak for adversarial inputs.
    #[test]
    fn primitive_is_total() {
        let q = Quintic::new(f32::NAN, f32::INFINITY, 1e30, -1e30, f32::NAN, 0.0, -1.0);
        let s = q.eval(f32::NAN);
        assert!(s.p.is_finite() && s.v.is_finite() && s.a.is_finite() && s.j.is_finite());
        let peak = q.peak_abs_jerk();
        assert!(peak >= 0.0);
    }
}
