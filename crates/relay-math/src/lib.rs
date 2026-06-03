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

/// Sine. Qualification boundary — see crate docs.
#[inline(always)]
pub fn sinf(x: f32) -> f32 {
    libm::sinf(x)
}

/// Cosine. Qualification boundary — see crate docs.
#[inline(always)]
pub fn cosf(x: f32) -> f32 {
    libm::cosf(x)
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
        assert_eq!(sinf(0.7), libm::sinf(0.7));
        assert_eq!(cosf(0.7), libm::cosf(0.7));
        assert_eq!(atan2f(1.0, 2.0), libm::atan2f(1.0, 2.0));
        assert_eq!(acosf(0.3), libm::acosf(0.3));
        assert_eq!(fabsf(-1.5), libm::fabsf(-1.5));
        assert_eq!(remainderf(7.0, 2.0), libm::remainderf(7.0, 2.0));
    }
}
