//! # relay-math — the flight-math qualification seam (v1.12.0)
//!
//! **The single boundary between the verified flight path and the
//! transcendental-math implementation.** Every `sqrt`, `sin`, `cos`, `atan2`,
//! `acos`, `fabs`, and `remainder` that the verified cascade evaluates —
//! across `relay-iekf`, `relay-geo`, `relay-adrc`, `relay-mix-quad`, and
//! `falcon-core` — routes through one of the thin wrappers below.
//!
//! ## Why this exists
//!
//! `libm` is in the flight path (the estimator's quaternion/gravity math, the
//! geometric controller's `atan2`/`acos`, the mixer's geometry). It is a
//! **qualification item**: a DO-178C / ISO 26262 program must show every
//! library function in the flight path is correct to the required level. With
//! the calls scattered across ~45 sites in five crates, that argument has 45
//! entry points. With this seam, it has **one**: qualify (or replace) the
//! bodies here — a proven polynomial core, CMSIS-DSP, a hardware CORDIC, or a
//! qualified `libm` build — and the whole cascade inherits it **without a
//! single flight-code edit**.
//!
//! The wrappers are `#[inline(always)]`, so routing through the seam costs
//! nothing at runtime — it is a *source-level* indirection that exists purely
//! to give qualification a single place to stand.
//!
//! ## The qualification surface (what must be qualified here)
//!
//! | function   | flight-path use                                              |
//! |------------|--------------------------------------------------------------|
//! | [`sqrtf`]  | vector norms (IEKF innovations, position/accel saturation)   |
//! | [`sinf`]   | quaternion/rotation kinematics, mixer geometry               |
//! | [`cosf`]   | quaternion/rotation kinematics, mixer geometry               |
//! | [`atan2f`] | geometric desired-rate (tilt direction), heading             |
//! | [`acosf`]  | tilt angle from the body-z projection                        |
//! | [`fabsf`]  | error magnitudes, clamps, consistency gates                  |
//! | [`remainderf`] | heading wrap to (−π, π]                                   |
//!
//! Today each wrapper forwards to `libm`. That is the *unqualified* default;
//! the qualification step changes only this file.

#![no_std]

/// Square root. Qualification boundary — see crate docs.
#[inline(always)]
pub fn sqrtf(x: f32) -> f32 {
    libm::sqrtf(x)
}

// ── f32-only sin/cos kernels (v1.125, MATHF32-P01) ─────────────────────
//
// libm's `sinf`/`cosf` are f32 APIs with **f64 internals** (f64 minimax
// polynomials + `rem_pio2f`'s f64 argument reduction). Those were the ONLY
// f64 operations left in the shipped flight component (480 ops in 4 libm
// functions, measured on falcon-flight-v1.123 — jess#144), and on the M4
// estimator partition (FPv4-SP: no f64 hardware) every one of them
// soft-floats inside `so3_exp` on the hot path. These kernels are f32
// end-to-end:
//
// * Reduction: Cody–Waite, π/2 split into three f32 parts — exact
//   products for quadrant counts |n| ≤ 256, which covers the QUALIFIED
//   ENVELOPE |x| ≤ 128 (≈ 40π). Every flight-path caller is far inside
//   (attitude/heading angles are wrapped; the notch's ω₀ < π).
// * Kernels: Cephes/MUSL-lineage minimax polynomials on |r| ≤ π/4
//   (degree-7 sin, degree-8 cos) — the classic single-precision cores.
// * Totality: non-finite input → 0.0 (the codebase's sanitize
//   convention); outside the envelope the reduction degrades gracefully
//   (bounded, finite — never NaN) but accuracy is only ASSERTED inside.
//   Conformance vs libm is MEASURED in ulp across the envelope (tests).

/// Cody–Waite π/2 split (f32 parts; hi has 12 trailing zero bits so
/// `n·PIO2_HI` is exact for |n| ≤ 4096).
const PIO2_HI: f32 = 1.570_312_5; // 0x3FC90000
const PIO2_MID: f32 = 4.837_513e-4; // 0x39FDAA22
const PIO2_LO: f32 = 7.549_79e-8; // 0x33A22168
#[allow(clippy::approx_constant)] // deliberate: paired with the Cody–Waite PIO2 split below
const FRAC_2_PI: f32 = 0.636_619_77;

/// Reduce x to (quadrant n mod 4, remainder r with |r| ≲ π/4), f32-only.
#[inline(always)]
fn reduce(x: f32) -> (i32, f32) {
    // round-half-away in pure core (no_std has no f32::round): exact for
    // the envelope's quadrant counts (|n| ≤ 82 at |x| ≤ 128).
    let t = x * FRAC_2_PI;
    let n = if t >= 0.0 { (t + 0.5) as i64 } else { (t - 0.5) as i64 } as f32;
    let r = ((x - n * PIO2_HI) - n * PIO2_MID) - n * PIO2_LO;
    // Belt for far-out-of-envelope inputs where the reduction has
    // degraded: the polynomials are only evaluated on a bounded r, so the
    // output stays finite and in [-1, 1] regardless.
    ((n as i32) & 3, r.clamp(-0.8, 0.8))
}

/// sin on the reduced range |r| ≤ π/4 (Cephes single-precision core).
#[inline(always)]
fn sin_poly(r: f32) -> f32 {
    const S1: f32 = -1.666_665_5e-1;
    const S2: f32 = 8.332_161e-3;
    const S3: f32 = -1.951_529_6e-4;
    let z = r * r;
    r + r * z * (S1 + z * (S2 + z * S3))
}

/// cos on the reduced range |r| ≤ π/4 (Cephes single-precision core).
#[inline(always)]
fn cos_poly(r: f32) -> f32 {
    const C1: f32 = 4.166_664_6e-2;
    const C2: f32 = -1.388_731_6e-3;
    const C3: f32 = 2.443_315_7e-5;
    let z = r * r;
    1.0 - 0.5 * z + z * z * (C1 + z * (C2 + z * C3))
}

/// Sine. Qualification boundary — f32-only kernel (no f64 anywhere in the
/// call graph); envelope |x| ≤ 128, non-finite → 0.0. See module notes.
#[inline(always)]
pub fn sinf(x: f32) -> f32 {
    if !x.is_finite() {
        return 0.0;
    }
    let (q, r) = reduce(x);
    match q {
        0 => sin_poly(r),
        1 => cos_poly(r),
        2 => -sin_poly(r),
        _ => -cos_poly(r),
    }
}

/// Cosine. Qualification boundary — f32-only kernel (no f64 anywhere in
/// the call graph); envelope |x| ≤ 128, non-finite → 0.0. See module notes.
#[inline(always)]
pub fn cosf(x: f32) -> f32 {
    if !x.is_finite() {
        return 0.0;
    }
    let (q, r) = reduce(x);
    match q {
        0 => cos_poly(r),
        1 => -sin_poly(r),
        2 => -cos_poly(r),
        _ => sin_poly(r),
    }
}

/// Two-argument arctangent. Qualification boundary — see crate docs.
#[inline(always)]
pub fn atan2f(y: f32, x: f32) -> f32 {
    libm::atan2f(y, x)
}

/// Arccosine. Qualification boundary — see crate docs.
#[inline(always)]
pub fn acosf(x: f32) -> f32 {
    libm::acosf(x)
}

/// Absolute value. Qualification boundary — see crate docs.
#[inline(always)]
pub fn fabsf(x: f32) -> f32 {
    libm::fabsf(x)
}

/// IEEE remainder (used for heading wrap). Qualification boundary — see crate docs.
#[inline(always)]
pub fn remainderf(x: f32, y: f32) -> f32 {
    libm::remainderf(x, y)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The seam forwards faithfully — each wrapper agrees with libm on
    /// representative arguments. (When the bodies are later replaced by a
    /// qualified core, this test becomes the conformance check against the
    /// reference, to the qualified tolerance.)
    #[test]
    fn seam_forwards_to_reference() {
        assert_eq!(sqrtf(2.0), libm::sqrtf(2.0));
        assert_eq!(atan2f(1.0, 2.0), libm::atan2f(1.0, 2.0));
        assert_eq!(acosf(0.3), libm::acosf(0.3));
        assert_eq!(fabsf(-1.5), libm::fabsf(-1.5));
        assert_eq!(remainderf(7.0, 2.0), libm::remainderf(7.0, 2.0));
    }

    fn ulp_diff(a: f32, b: f32) -> u32 {
        if a == b {
            return 0;
        }
        let (ia, ib) = (a.to_bits() as i64, b.to_bits() as i64);
        // map to a monotonic integer line (sign-magnitude → offset)
        let ma = if ia < 0 { i64::MIN / 2 - ia } else { ia };
        let mb = if ib < 0 { i64::MIN / 2 - ib } else { ib };
        (ma - mb).unsigned_abs().min(u32::MAX as u64) as u32
    }

    /// MATHF32-P01 conformance, MEASURED: the f32-only kernels agree with
    /// libm (the previous qualified-default reference) across the whole
    /// qualified envelope |x| ≤ 128 — 4M-point dense sweep + the wrap
    /// boundaries. Budget: ≤ 2 ulp inside |x| ≤ 4π (every flight caller),
    /// ≤ 8 ulp across the full envelope (reduction error grows with the
    /// quadrant count; still far below any control-loop significance).
    #[test]
    fn f32_kernels_conform_to_reference_across_envelope() {
        let mut worst_inner = 0u32;
        let mut worst_outer = 0u32;
        let n = 4_000_000usize;
        for k in 0..n {
            let x = -128.0 + 256.0 * (k as f32 + 0.5) / n as f32;
            let ds = ulp_diff(sinf(x), libm::sinf(x));
            let dc = ulp_diff(cosf(x), libm::cosf(x));
            let d = ds.max(dc);
            if x.abs() <= 4.0 * core::f32::consts::PI {
                worst_inner = worst_inner.max(d);
            } else {
                worst_outer = worst_outer.max(d);
            }
        }
        assert!(worst_inner <= 2, "inner-envelope worst {worst_inner} ulp");
        assert!(worst_outer <= 8, "outer-envelope worst {worst_outer} ulp");
    }

    /// Totality + range: for ANY f32 bit pattern the kernels return a
    /// finite value in [-1, 1] (non-finite input → 0.0 by spec).
    #[test]
    fn f32_kernels_total_and_bounded() {
        for bits in [0u32, 0x7F80_0000, 0xFF80_0000, 0x7FC0_0000, 0x0000_0001, 0x7F7F_FFFF] {
            let x = f32::from_bits(bits);
            for v in [sinf(x), cosf(x)] {
                assert!(v.is_finite() && (-1.0001..=1.0001).contains(&v), "x={x} -> {v}");
            }
        }
        let mut lcg = 0x1357_9BDFu32;
        for _ in 0..2_000_000 {
            lcg = lcg.wrapping_mul(1664525).wrapping_add(1013904223);
            let x = f32::from_bits(lcg);
            for v in [sinf(x), cosf(x)] {
                assert!(v.is_finite() && (-1.0001..=1.0001).contains(&v), "x={x} -> {v}");
            }
        }
    }

    /// The flight-path identity the estimator leans on: sin² + cos² ≈ 1
    /// across the inner envelope (so3_exp's rotation stays unit-norm).
    #[test]
    fn pythagorean_identity_inner_envelope() {
        for k in 0..400_000 {
            let x = -12.6 + 25.2 * (k as f32) / 400_000.0;
            let (s, c) = (sinf(x), cosf(x));
            let err = (s * s + c * c - 1.0).abs();
            assert!(err < 3.0e-7, "x={x}: s²+c²-1 = {err}");
        }
    }
}
