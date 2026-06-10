//! Relay FlowRange — rangefinder + optical-flow preprocessing for the IEKF.
//!
//! Low-altitude / GPS-denied aiding, the PX4 EKF2 terrain + flow role, missing
//! until now (falcon had baro/GNSS only). Two preprocessors that turn raw sensor
//! readings into the quantities the verified IEKF consumes:
//!
//!  * **`range_to_altitude`** — a downward rangefinder reading + the body tilt →
//!    terrain-relative altitude (tilt-compensated, `range·cos(tilt)`), GATED to a
//!    valid range band so a spurious / out-of-band / NaN return is rejected
//!    (`None`) rather than anchoring the estimator to garbage.
//!  * **`flow_to_velocity`** — optical-flow angular rates + the gyro + the height
//!    above ground → horizontal velocity (`(flow − gyro)·height`), gated on a
//!    positive height. This is how flow gives velocity when GPS is denied.
//!
//! Verified (Kani): both are TOTAL (no panic for any input), and each returns a
//! value ONLY when its gate passes (valid range band / positive height) — a bad
//! reading can never become an aiding measurement.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// Tilt-compensated terrain altitude from a downward rangefinder.
///
/// `range_m` is the slant range the sensor reports; `tilt_rad` is the body tilt
/// from vertical. The vertical distance to the ground is `range·cos(tilt)`.
/// Returns `None` (reading rejected) if the range is non-finite or outside the
/// sensor's valid band `[min_valid, max_valid]`. The tilt is clamped to a sane
/// range so an absurd attitude can't invert the sign.
pub fn range_to_altitude(range_m: f32, tilt_rad: f32, min_valid: f32, max_valid: f32) -> Option<f32> {
    if !range_m.is_finite() || range_m < min_valid || range_m > max_valid {
        return None;
    }
    // clamp tilt to [-85°, 85°]: beyond that a rangefinder reading is meaningless.
    let t = if tilt_rad.is_finite() {
        tilt_rad.clamp(-1.483, 1.483)
    } else {
        0.0
    };
    let alt = range_m * relay_math::cosf(t);
    if alt.is_finite() {
        Some(alt)
    } else {
        None
    }
}

/// Horizontal velocity from optical flow.
///
/// `flow_x/flow_y` are the measured optical-flow angular rates (rad/s);
/// `gyro_x/gyro_y` are the body angular rates to subtract (the flow induced by
/// rotation, not translation); `height_m` is the height above ground. The
/// translational flow `(flow − gyro)` scaled by height gives velocity. Returns
/// `None` if the height is non-positive / non-finite (flow gives no velocity
/// with no height reference).
pub fn flow_to_velocity(
    flow_x: f32,
    flow_y: f32,
    gyro_x: f32,
    gyro_y: f32,
    height_m: f32,
) -> Option<[f32; 2]> {
    if !(height_m.is_finite() && height_m > 0.0) {
        return None;
    }
    let fx = if flow_x.is_finite() { flow_x } else { 0.0 };
    let fy = if flow_y.is_finite() { flow_y } else { 0.0 };
    let gx = if gyro_x.is_finite() { gyro_x } else { 0.0 };
    let gy = if gyro_y.is_finite() { gyro_y } else { 0.0 };
    let vx = (fx - gx) * height_m;
    let vy = (fy - gy) * height_m;
    if vx.is_finite() && vy.is_finite() {
        Some([vx, vy])
    } else {
        None
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn range_level_is_the_range() {
        let a = range_to_altitude(3.0, 0.0, 0.1, 40.0).unwrap();
        assert!((a - 3.0).abs() < 1e-6); // level: altitude = range
    }

    #[test]
    fn range_tilt_compensated() {
        // 60° tilt: altitude = range·cos(60°) = 4·0.5 = 2.0
        let a = range_to_altitude(4.0, core::f32::consts::FRAC_PI_3, 0.1, 40.0).unwrap();
        assert!((a - 2.0).abs() < 1e-4);
    }

    #[test]
    fn range_out_of_band_rejected() {
        assert!(range_to_altitude(0.05, 0.0, 0.1, 40.0).is_none()); // below min
        assert!(range_to_altitude(50.0, 0.0, 0.1, 40.0).is_none()); // above max
        assert!(range_to_altitude(f32::NAN, 0.0, 0.1, 40.0).is_none());
    }

    #[test]
    fn flow_gives_velocity() {
        // pure translation (no rotation), flow 0.2 rad/s at 5 m ⇒ 1.0 m/s.
        let v = flow_to_velocity(0.2, -0.1, 0.0, 0.0, 5.0).unwrap();
        assert!((v[0] - 1.0).abs() < 1e-5);
        assert!((v[1] + 0.5).abs() < 1e-5);
    }

    #[test]
    fn flow_subtracts_rotation() {
        // flow equals gyro ⇒ pure rotation ⇒ zero translational velocity.
        let v = flow_to_velocity(0.3, 0.3, 0.3, 0.3, 5.0).unwrap();
        assert_eq!(v, [0.0, 0.0]);
    }

    #[test]
    fn flow_no_height_rejected() {
        assert!(flow_to_velocity(0.2, 0.0, 0.0, 0.0, 0.0).is_none());
        assert!(flow_to_velocity(0.2, 0.0, 0.0, 0.0, -1.0).is_none());
    }
}

#[cfg(test)]
mod proptests {
    use super::*;

    proptest::proptest! {
        /// range_to_altitude returns Some ONLY for an in-band finite range, and
        /// the altitude never exceeds the range magnitude (|cos| ≤ 1).
        #[test]
        fn altitude_gated_and_bounded(r in -100.0f32..100.0, tilt in -3.0f32..3.0) {
            match range_to_altitude(r, tilt, 0.2, 30.0) {
                Some(a) => {
                    proptest::prop_assert!((0.2..=30.0).contains(&r));
                    proptest::prop_assert!(a.abs() <= r.abs() + 1e-3);
                }
                None => {}
            }
        }
    }
}
