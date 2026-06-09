//! Relay Avoid — collision prevention + precision landing.
//!
//! Two safety-envelope controllers that sit on top of the position loop:
//!
//!  * **Collision prevention** — a velocity LIMITER. Given the rangefinder
//!    distance to an obstacle ahead, it caps the approach speed to the
//!    braking-limited speed `v ≤ √(2·a_max·(dist − margin))`, so the vehicle can
//!    always decelerate to a stop before the safety margin. Inside the margin
//!    the cap is zero (stop). This is PX4's Collision Prevention, here with the
//!    capping logic Kani-proven.
//!  * **Precision landing** — a vision-marker centering controller. Given the
//!    horizontal offset of a landing marker (from a downward camera), it
//!    produces a horizontal velocity correction toward the marker, clamped to a
//!    max speed, plus a within-radius convergence check.
//!
//! Verification split (the codebase pattern): the COMPARISON / clamp logic is
//! Kani-proven (cap never exceeds the command or the allowed speed; the
//! correction magnitude never exceeds the max and points toward the marker);
//! the f32 transcendental `√` in the braking speed is proptest-gated (Kani on
//! libm sqrt is intractable), and its output is guarded non-negative/finite by
//! construction so the Kani bounds hold regardless of its exact value.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

#[inline]
fn finite_or(x: f32, d: f32) -> f32 {
    if x.is_finite() {
        x
    } else {
        d
    }
}

/// Cap an approach speed `v_cmd` (≥ 0, toward the obstacle) at `v_allowed`. Pure
/// comparison: the result never exceeds EITHER input and is never negative.
/// Non-finite inputs sanitise to a safe stop (0).
pub fn cap_approach(v_cmd: f32, v_allowed: f32) -> f32 {
    let v = finite_or(v_cmd, 0.0).max(0.0);
    let a = finite_or(v_allowed, 0.0).max(0.0);
    if a < v {
        a
    } else {
        v
    }
}

/// Braking-limited speed for a usable clear distance `usable` (m) and max
/// deceleration `a_max` (m/s²): `√(2·a_max·usable)`. Guarded so the result is
/// always finite and ≥ 0 (a negative/NaN input ⇒ 0 ⇒ stop), WITHOUT relying on
/// the exact value of the square root.
pub fn braking_speed(usable: f32, a_max: f32) -> f32 {
    let u = finite_or(usable, 0.0).max(0.0);
    let a = finite_or(a_max, 0.0).max(0.0);
    let s = relay_math::sqrtf(2.0 * a * u);
    if s.is_finite() && s >= 0.0 {
        s
    } else {
        0.0
    }
}

/// The collision-prevention velocity cap: limit the commanded approach speed so
/// the vehicle can brake to a stop before `margin` of an obstacle `dist` metres
/// ahead. Inside the margin the result is 0 (stop). `a_max` is the available
/// deceleration. The result is always in `[0, v_cmd]`.
pub fn limit_approach_speed(v_cmd: f32, dist: f32, margin: f32, a_max: f32) -> f32 {
    let usable = finite_or(dist, 0.0) - finite_or(margin, 0.0);
    cap_approach(v_cmd, braking_speed(usable, a_max))
}

/// 2D horizontal offset (north, east) in metres from the camera centre to the
/// landing marker.
pub type Offset = [f32; 2];

/// Precision-landing horizontal velocity correction: drive toward the marker at
/// `-kp · offset`, clamped to `v_max` magnitude. Points OPPOSITE the offset
/// (toward the marker, reducing it). Total: NaN/∞ ⇒ no correction.
pub fn precision_velocity(offset: Offset, kp: f32, v_max: f32) -> Offset {
    let kp = finite_or(kp, 0.0).max(0.0);
    let vmax = finite_or(v_max, 0.0).max(0.0);
    let ox = finite_or(offset[0], 0.0);
    let oy = finite_or(offset[1], 0.0);
    let mut vx = -kp * ox;
    let mut vy = -kp * oy;
    let mag2 = vx * vx + vy * vy;
    if mag2 > vmax * vmax && mag2 > 0.0 {
        let scale = vmax / relay_math::sqrtf(mag2);
        if scale.is_finite() {
            vx *= scale;
            vy *= scale;
        } else {
            vx = 0.0;
            vy = 0.0;
        }
    }
    [vx, vy]
}

/// Has the vehicle converged over the marker — horizontal offset within
/// `radius` (the precision-landing success criterion)? Total; NaN ⇒ false.
pub fn within_radius(offset: Offset, radius: f32) -> bool {
    let ox = finite_or(offset[0], f32::INFINITY);
    let oy = finite_or(offset[1], f32::INFINITY);
    let r = finite_or(radius, 0.0).max(0.0);
    ox * ox + oy * oy <= r * r
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn stops_inside_the_margin() {
        // obstacle 1.0 m ahead, margin 1.5 m ⇒ inside margin ⇒ stop.
        assert_eq!(limit_approach_speed(5.0, 1.0, 1.5, 3.0), 0.0);
    }

    #[test]
    fn caps_approach_far_away_but_allows_full_when_clear() {
        // 20 m clear, a_max 3 ⇒ v_allowed = sqrt(2*3*18.5)=~10.5 > 4 ⇒ full 4.
        assert_eq!(limit_approach_speed(4.0, 20.0, 1.5, 3.0), 4.0);
        // close in: 3 m, margin 1.5 ⇒ usable 1.5 ⇒ v_allowed=sqrt(9)=3 ⇒ cap to 3.
        let capped = limit_approach_speed(5.0, 3.0, 1.5, 3.0);
        assert!((capped - 3.0).abs() < 1e-4, "got {capped}");
    }

    #[test]
    fn never_exceeds_command_or_goes_negative() {
        for &(v, d, m, a) in &[(2.0, 5.0, 1.0, 3.0), (10.0, 0.5, 1.0, 4.0), (-1.0, 5.0, 1.0, 3.0)] {
            let r = limit_approach_speed(v, d, m, a);
            assert!(r >= 0.0 && r <= v.max(0.0));
        }
    }

    #[test]
    fn precision_correction_points_at_marker() {
        // marker 2 m north, 0 east ⇒ correction is southward (negative north).
        let v = precision_velocity([2.0, 0.0], 0.5, 1.0);
        assert!(v[0] < 0.0 && v[1].abs() < 1e-6);
        // clamped to v_max magnitude.
        let big = precision_velocity([100.0, 100.0], 5.0, 1.0);
        let mag = (big[0] * big[0] + big[1] * big[1]).sqrt();
        assert!(mag <= 1.0 + 1e-4, "magnitude {mag} exceeds v_max");
    }

    #[test]
    fn within_radius_check() {
        assert!(within_radius([0.1, 0.1], 0.3));
        assert!(!within_radius([0.5, 0.0], 0.3));
    }

    #[test]
    fn nan_inputs_fail_safe() {
        assert_eq!(limit_approach_speed(f32::NAN, 5.0, 1.0, 3.0), 0.0);
        assert_eq!(precision_velocity([f32::NAN, 1.0], 0.5, 1.0)[0], 0.0);
        assert!(!within_radius([f32::NAN, 0.0], 1.0));
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// The f32 safety invariants the Kani harnesses defer (they involve √):
        /// the limited approach speed is always in [0, v_cmd], and the precision
        /// correction magnitude never exceeds v_max.
        #[test]
        fn limited_speed_in_range(
            v in 0.0f32..30.0, d in 0.0f32..200.0, m in 0.0f32..5.0, a in 0.1f32..10.0,
        ) {
            let r = limit_approach_speed(v, d, m, a);
            prop_assert!(r >= 0.0 && r <= v + 1e-3);
        }

        #[test]
        fn correction_within_vmax(
            ox in -50.0f32..50.0, oy in -50.0f32..50.0, kp in 0.0f32..5.0, vmax in 0.0f32..5.0,
        ) {
            let v = precision_velocity([ox, oy], kp, vmax);
            let mag = relay_math::sqrtf(v[0] * v[0] + v[1] * v[1]);
            prop_assert!(mag <= vmax + 1e-3, "mag {} > vmax {}", mag, vmax);
        }
    }
}
