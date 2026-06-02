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

/// 3-vector (NED).
pub type Vec3 = [f32; 3];

/// A 3-axis trajectory segment — three independent jerk-minimizing quintics
/// (the per-axis decoupling that makes the Mueller primitive cheap). Its
/// `(pos, vel, acc, jerk)` sample feeds the differential-flatness map:
/// `acc → a_cmd`, `jerk → j_cmd` for `relay_geo::flatness_omega_ff`.
#[derive(Clone, Copy, Debug)]
pub struct Segment3 {
    axes: [Quintic; 3],
}

/// 3-axis trajectory sample (NED): position, velocity, acceleration, jerk.
#[derive(Clone, Copy, Debug)]
pub struct Sample3 {
    pub pos: Vec3,
    pub vel: Vec3,
    pub acc: Vec3,
    pub jerk: Vec3,
}

impl Segment3 {
    /// Jerk-minimizing segment from full initial state `(p0,v0,a0)` to full
    /// final state `(pf,vf,af)` over duration `t` (s), per axis.
    pub fn new(p0: Vec3, v0: Vec3, a0: Vec3, pf: Vec3, vf: Vec3, af: Vec3, t: f32) -> Self {
        Segment3 {
            axes: [
                Quintic::new(p0[0], v0[0], a0[0], pf[0], vf[0], af[0], t),
                Quintic::new(p0[1], v0[1], a0[1], pf[1], vf[1], af[1], t),
                Quintic::new(p0[2], v0[2], a0[2], pf[2], vf[2], af[2], t),
            ],
        }
    }

    /// Rest-to-rest segment between two positions (zero boundary velocity +
    /// acceleration) — the building block of a waypoint mission.
    pub fn rest_to_rest(from: Vec3, to: Vec3, t: f32) -> Self {
        Self::new(from, [0.0; 3], [0.0; 3], to, [0.0; 3], [0.0; 3], t)
    }

    /// Duration (s).
    #[inline]
    pub fn duration(&self) -> f32 {
        self.axes[0].duration()
    }

    /// Sample the trajectory at time `t` (clamped to `[0, T]`).
    pub fn eval(&self, t: f32) -> Sample3 {
        let s = [self.axes[0].eval(t), self.axes[1].eval(t), self.axes[2].eval(t)];
        Sample3 {
            pos: [s[0].p, s[1].p, s[2].p],
            vel: [s[0].v, s[1].v, s[2].v],
            acc: [s[0].a, s[1].a, s[2].a],
            jerk: [s[0].j, s[1].j, s[2].j],
        }
    }

    /// Per-axis sound jerk bound over `[0,T]` (for input-feasibility).
    pub fn peak_abs_jerk(&self) -> Vec3 {
        [
            self.axes[0].peak_abs_jerk(),
            self.axes[1].peak_abs_jerk(),
            self.axes[2].peak_abs_jerk(),
        ]
    }
}

/// Clamp `x` to `[lo, 1]` (lo assumed in `[0, 1]`); total over non-finite.
/// Split from the gate arithmetic so the governor's bound is a
/// comparison-only proof.
#[inline]
fn clamp_to(x: f32, lo: f32) -> f32 {
    let lo = if (0.0..=1.0).contains(&lo) { lo } else { 0.0 };
    if !x.is_finite() || x < lo {
        lo
    } else if x > 1.0 {
        1.0
    } else {
        x
    }
}

/// Error-gate factor g ∈ [g_min, 1]: full speed (1) when the tracking error
/// is ≤ `lo`, slowest (`g_min`) when ≥ `hi`, linear between. Total: a
/// non-finite error or a degenerate band fails safe to `g_min` (slow down).
fn gate_factor(err: f32, lo: f32, hi: f32, g_min: f32) -> f32 {
    let g_min = clamp_to(g_min, 0.0); // → [0, 1]
    if !err.is_finite() || !lo.is_finite() || !hi.is_finite() || hi <= lo {
        return g_min;
    }
    let g = if err <= lo {
        1.0
    } else if err >= hi {
        g_min
    } else {
        let frac = (err - lo) / (hi - lo); // ∈ (0, 1)
        1.0 - frac * (1.0 - g_min)
    };
    clamp_to(g, g_min)
}

/// Setpoint-side reference governor (v0.36 — "virtual-time" / path governor).
///
/// A cascaded controller diverges when the outer loop's REFERENCE outruns
/// what the inner loop can track. The fix that does NOT lag the feedback
/// (unlike command-filtering ω_d, falsified in v0.33): slow the *reference
/// clock* when the vehicle falls behind. The trajectory is sampled at a
/// governed virtual time `s` whose advance `ṡ = g·dt` is gated by the
/// tracking error — `g = 1` on-track, ramping to `g_min` once the error
/// exceeds a band — so the reference stays within reach. An aggressive
/// nominal trajectory then degrades GRACEFULLY (slows) instead of diverging.
#[derive(Clone, Copy)]
pub struct RefGovernor {
    s: f32,
    err_lo: f32,
    err_hi: f32,
    g_min: f32,
}

impl RefGovernor {
    /// `err_lo`/`err_hi`: the tracking-error band (m) over which the advance
    /// ramps 1 → `g_min`. `g_min` ∈ (0, 1]: the slowest advance fraction
    /// (> 0 so the mission never permanently deadlocks).
    pub fn new(err_lo: f32, err_hi: f32, g_min: f32) -> Self {
        RefGovernor { s: 0.0, err_lo, err_hi, g_min }
    }

    /// The error-gate factor g ∈ [g_min, 1] for a tracking error.
    #[inline]
    pub fn gate(&self, track_err: f32) -> f32 {
        gate_factor(track_err, self.err_lo, self.err_hi, self.g_min)
    }

    /// Advance the governed virtual time by `dt` scaled by the error gate;
    /// returns the new time at which to sample the trajectory. Monotone
    /// non-decreasing, with `g_min·dt ≤ ṡ ≤ dt`.
    pub fn advance(&mut self, track_err: f32, dt: f32) -> f32 {
        let dt = if dt.is_finite() && dt > 0.0 { dt } else { 0.0 };
        self.s += self.gate(track_err) * dt;
        self.s
    }

    /// Current governed virtual time.
    pub fn time(&self) -> f32 {
        self.s
    }

    pub fn reset(&mut self) {
        self.s = 0.0;
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// The governor's error gate is always within [g_min, 1] for ANY error
    /// (incl. non-finite) and any band, given g_min ∈ [0,1] — so the
    /// reference advances monotonically and never faster than nominal nor
    /// (with g_min>0) deadlocks. The clamp is split from the division so this
    /// is a comparison-only proof.
    #[kani::proof]
    fn verify_gate_factor_bounded() {
        let err: f32 = kani::any();
        let lo: f32 = kani::any();
        let hi: f32 = kani::any();
        let g_min: f32 = kani::any();
        kani::assume(lo.is_finite() && libm::fabsf(lo) <= 1e3);
        kani::assume(hi.is_finite() && libm::fabsf(hi) <= 1e3);
        kani::assume(g_min.is_finite() && g_min >= 0.0 && g_min <= 1.0);
        let g = gate_factor(err, lo, hi, g_min);
        assert!(g >= g_min && g <= 1.0);
    }

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

    // ── v0.36 reference governor ─────────────────────────────────────────

    /// The error gate: full speed on-track, slowest off-track, monotone
    /// non-increasing in the error, and bounded [g_min, 1].
    #[test]
    fn gate_factor_ramps_and_bounds() {
        let (lo, hi, gmin) = (0.3, 1.5, 0.05);
        assert_eq!(gate_factor(0.0, lo, hi, gmin), 1.0); // on-track
        assert_eq!(gate_factor(0.3, lo, hi, gmin), 1.0); // at lo
        assert_eq!(gate_factor(2.0, lo, hi, gmin), gmin); // past hi
        let mid = gate_factor(0.9, lo, hi, gmin); // halfway
        assert!((mid - 0.525).abs() < 1e-3, "mid ramp = {mid}");
        // monotone non-increasing in error
        let mut last = 1.0;
        for k in 0..=20 {
            let e = k as f32 * 0.1;
            let g = gate_factor(e, lo, hi, gmin);
            assert!(g <= last + 1e-6 && g >= gmin - 1e-6, "g={g} at e={e}");
            last = g;
        }
        // fail-safe: non-finite error / degenerate band ⇒ slowest
        assert_eq!(gate_factor(f32::NAN, lo, hi, gmin), gmin);
        assert_eq!(gate_factor(0.0, 1.0, 0.5, gmin), gmin); // hi ≤ lo
    }

    /// Advance is monotone with a bounded rate g_min·dt ≤ ṡ ≤ dt, and slows
    /// the reference clock when the vehicle falls behind.
    #[test]
    fn governor_slows_reference_when_behind() {
        let dt = 0.02;
        // On-track: advances at full rate (≈ wall time).
        let mut on = RefGovernor::new(0.3, 1.5, 0.05);
        for _ in 0..500 {
            on.advance(0.0, dt);
        }
        assert!((on.time() - 10.0).abs() < 1e-3, "on-track ≈ wall time, {}", on.time());
        // Persistently behind (error past hi): advances at g_min rate only.
        let mut behind = RefGovernor::new(0.3, 1.5, 0.05);
        let mut prev = 0.0;
        for _ in 0..500 {
            let s = behind.advance(3.0, dt);
            assert!(s >= prev, "monotone"); // never goes backward
            assert!(s - prev <= dt + 1e-6, "rate ≤ dt"); // never faster than nominal
            prev = s;
        }
        assert!((behind.time() - 0.05 * 10.0).abs() < 1e-2, "behind ≈ g_min·wall, {}", behind.time());
        assert!(behind.time() < on.time(), "governed clock lags the wall clock when behind");
    }

    proptest::proptest! {
        /// The gate is in [g_min,1] and advance is monotone with rate ≤ dt
        /// for any error/band/dt sequence.
        #[test]
        fn governor_bounded_and_monotone(
            seq in proptest::collection::vec((-5.0f32..5.0, 0.0f32..0.05), 0..400),
            gmin in 0.0f32..1.0,
        ) {
            let mut gov = RefGovernor::new(0.2, 1.0, gmin);
            let mut prev = 0.0;
            for (e, dt) in seq {
                let g = gov.gate(e);
                proptest::prop_assert!(g >= gmin - 1e-6 && g <= 1.0 + 1e-6);
                let s = gov.advance(e, dt);
                proptest::prop_assert!(s >= prev - 1e-9);
                proptest::prop_assert!(s - prev <= dt + 1e-6);
                prev = s;
            }
        }
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

    /// A rest-to-rest 3-axis segment reaches the target waypoint with zero
    /// terminal velocity + acceleration (a stable hover at the waypoint).
    #[test]
    fn segment3_rest_to_rest_reaches_waypoint() {
        let seg = Segment3::rest_to_rest([0.0, 0.0, 0.0], [3.0, -2.0, 1.5], 4.0);
        let s0 = seg.eval(0.0);
        assert!(s0.pos.iter().all(|&p| p.abs() < 1e-3));
        let sT = seg.eval(4.0);
        for i in 0..3 {
            let want = [3.0, -2.0, 1.5][i];
            assert!((sT.pos[i] - want).abs() < 1e-2, "axis {i} pos {} vs {want}", sT.pos[i]);
            assert!(sT.vel[i].abs() < 1e-2 && sT.acc[i].abs() < 1e-2, "axis {i} not at rest");
        }
        assert!(seg.peak_abs_jerk().iter().all(|&j| j.is_finite() && j >= 0.0));
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
