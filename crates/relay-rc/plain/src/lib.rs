//! Relay RC — pilot stick input → manual flight-mode setpoints.
//!
//! The everyday PX4 manual modes, missing until now (falcon was offboard/MAVLink
//! only). Maps a normalized RC stick input to either:
//!
//!  * **Stabilized** — sticks command a bounded ATTITUDE setpoint (roll/pitch
//!    tilt, yaw rate); centre stick = level, so releasing the sticks auto-levels.
//!    Feeds the verified attitude controller (relay-geo/relay-att).
//!  * **Acro** — sticks command a bounded BODY-RATE setpoint directly, feeding
//!    the verified rate controller (relay-adrc/relay-rate). No auto-level.
//!
//! The safety-relevant property — proven by Kani — is that the OUTPUT IS ALWAYS
//! BOUNDED: for ANY stick input (including NaN/∞ from a glitching receiver) the
//! commanded tilt never exceeds `max_tilt`, the rates never exceed their max, and
//! thrust stays in [0, 1]. A bad RC frame can never command an unbounded angle or
//! rate.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// Normalized pilot stick input. Each axis in [-1, 1]; throttle in [0, 1].
/// Out-of-range / NaN values are sanitised by the mapping functions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcInput {
    /// Roll stick (right positive).
    pub roll: f32,
    /// Pitch stick (forward/nose-down positive).
    pub pitch: f32,
    /// Yaw stick (clockwise positive).
    pub yaw: f32,
    /// Throttle (0 = idle, 1 = full).
    pub throttle: f32,
}

/// Stabilized-mode output: a bounded attitude setpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeCmd {
    /// Roll angle setpoint (rad).
    pub roll_rad: f32,
    /// Pitch angle setpoint (rad).
    pub pitch_rad: f32,
    /// Yaw-rate setpoint (rad/s).
    pub yaw_rate: f32,
    /// Collective thrust [0, 1].
    pub thrust: f32,
}

/// Acro-mode output: a bounded body-rate setpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateCmd {
    /// Roll-rate setpoint (rad/s).
    pub roll_rate: f32,
    /// Pitch-rate setpoint (rad/s).
    pub pitch_rate: f32,
    /// Yaw-rate setpoint (rad/s).
    pub yaw_rate: f32,
    /// Collective thrust [0, 1].
    pub thrust: f32,
}

/// Sanitise a stick axis to [-1, 1]; NaN/∞ ⇒ 0 (centre = safe).
#[inline]
pub(crate) fn unit(x: f32) -> f32 {
    if x.is_nan() {
        0.0
    } else if x > 1.0 {
        1.0
    } else if x < -1.0 {
        -1.0
    } else {
        x
    }
}

/// Sanitise throttle to [0, 1]; NaN ⇒ 0 (idle = safe).
#[inline]
pub(crate) fn throttle01(x: f32) -> f32 {
    if x.is_nan() {
        0.0
    } else if x > 1.0 {
        1.0
    } else if x < 0.0 {
        0.0
    } else {
        x
    }
}

/// Clamp a positive limit to a finite non-negative value (NaN/neg ⇒ 0).
#[inline]
fn limit(x: f32) -> f32 {
    if x.is_finite() && x > 0.0 {
        x
    } else {
        0.0
    }
}

/// Stabilized mode: sticks → a bounded attitude setpoint. Centre roll/pitch sticks
/// command level (0 rad), so releasing auto-levels. `max_tilt_rad` bounds roll and
/// pitch; `max_yaw_rate` bounds the yaw rate.
pub fn stabilized(rc: RcInput, max_tilt_rad: f32, max_yaw_rate: f32) -> AttitudeCmd {
    let tilt = limit(max_tilt_rad);
    let yawr = limit(max_yaw_rate);
    AttitudeCmd {
        roll_rad: unit(rc.roll) * tilt,
        pitch_rad: unit(rc.pitch) * tilt,
        yaw_rate: unit(rc.yaw) * yawr,
        thrust: throttle01(rc.throttle),
    }
}

/// Acro mode: sticks → a bounded body-rate setpoint. `max_rate` bounds every axis.
pub fn acro(rc: RcInput, max_rate: f32) -> RateCmd {
    let r = limit(max_rate);
    RateCmd {
        roll_rate: unit(rc.roll) * r,
        pitch_rate: unit(rc.pitch) * r,
        yaw_rate: unit(rc.yaw) * r,
        thrust: throttle01(rc.throttle),
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn rc(roll: f32, pitch: f32, yaw: f32, throttle: f32) -> RcInput {
        RcInput { roll, pitch, yaw, throttle }
    }

    #[test]
    fn centre_stick_auto_levels() {
        let c = stabilized(rc(0.0, 0.0, 0.0, 0.5), 0.5, 1.0);
        assert_eq!(c.roll_rad, 0.0);
        assert_eq!(c.pitch_rad, 0.0);
        assert_eq!(c.yaw_rate, 0.0);
        assert_eq!(c.thrust, 0.5);
    }

    #[test]
    fn full_stick_hits_the_limit() {
        let c = stabilized(rc(1.0, -1.0, 1.0, 1.0), 0.5, 2.0);
        assert!((c.roll_rad - 0.5).abs() < 1e-6);
        assert!((c.pitch_rad + 0.5).abs() < 1e-6);
        assert!((c.yaw_rate - 2.0).abs() < 1e-6);
    }

    #[test]
    fn over_range_and_nan_are_bounded() {
        // glitching receiver: out-of-range + NaN must clamp, not blow up.
        let c = stabilized(rc(5.0, f32::NAN, -9.0, 2.0), 0.4, 1.5);
        assert!((c.roll_rad - 0.4).abs() < 1e-6);
        assert_eq!(c.pitch_rad, 0.0); // NaN → centre
        assert!((c.yaw_rate + 1.5).abs() < 1e-6);
        assert_eq!(c.thrust, 1.0);
    }

    #[test]
    fn acro_rates_bounded() {
        let c = acro(rc(2.0, -2.0, 0.5, 0.7), 3.0);
        assert!((c.roll_rate - 3.0).abs() < 1e-6);
        assert!((c.pitch_rate + 3.0).abs() < 1e-6);
        assert!((c.yaw_rate - 1.5).abs() < 1e-6);
        assert_eq!(c.thrust, 0.7);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// For ANY stick input + limits, the stabilized attitude command is
        /// bounded by the tilt/yaw limits and thrust in [0,1].
        #[test]
        fn stabilized_output_bounded(
            r in -10.0f32..10.0, p in -10.0f32..10.0, y in -10.0f32..10.0,
            t in -1.0f32..3.0, tilt in 0.0f32..1.5, yawr in 0.0f32..5.0,
        ) {
            let c = stabilized(RcInput { roll: r, pitch: p, yaw: y, throttle: t }, tilt, yawr);
            prop_assert!(c.roll_rad.abs() <= tilt + 1e-4);
            prop_assert!(c.pitch_rad.abs() <= tilt + 1e-4);
            prop_assert!(c.yaw_rate.abs() <= yawr + 1e-4);
            prop_assert!((0.0..=1.0).contains(&c.thrust));
        }

        /// Acro rates are bounded by max_rate for any stick input.
        #[test]
        fn acro_output_bounded(
            r in -10.0f32..10.0, p in -10.0f32..10.0, y in -10.0f32..10.0,
            t in -1.0f32..3.0, mr in 0.0f32..8.0,
        ) {
            let c = acro(RcInput { roll: r, pitch: p, yaw: y, throttle: t }, mr);
            prop_assert!(c.roll_rate.abs() <= mr + 1e-4);
            prop_assert!(c.pitch_rate.abs() <= mr + 1e-4);
            prop_assert!(c.yaw_rate.abs() <= mr + 1e-4);
            prop_assert!((0.0..=1.0).contains(&c.thrust));
        }
    }
}
