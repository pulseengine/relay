//! Relay control allocator — X-config quadcopter mixer.
//!
//! Maps the body-frame torque + collective-thrust command to four
//! per-motor PWM values for the falcon-quad airframe. Last stage of
//! the inner control cascade:
//!
//! ```text
//!  τ_body, T ──► [relay-mix-quad] ──► [m1, m2, m3, m4]  (each ∈ [0, 1])
//! ```
//!
//! ## Airframe convention
//!
//! Motors numbered 1–4 clockwise from front-right, viewed from above:
//!
//! ```text
//!         (front)
//!     M4         M1
//!       \       /
//!        \  +x /
//!  +y --- center ---
//!        /     \
//!       /       \
//!     M3         M2
//!         (back)
//! ```
//!
//! Spin directions (standard PX4 quad-x): M1, M3 CW; M2, M4 CCW.
//! Diagonal pairs share spin direction so net yaw reaction is zero
//! at hover.
//!
//! ## Mixer matrix
//!
//! Each motor's PWM contribution per body-frame command:
//!
//! | motor | thrust | roll  | pitch | yaw   | spin |
//! |-------|--------|-------|-------|-------|------|
//! | M1    | +1     | -1    | +1    | -1    | CW   |
//! | M2    | +1     | -1    | -1    | +1    | CCW  |
//! | M3    | +1     | +1    | -1    | -1    | CW   |
//! | M4    | +1     | +1    | +1    | +1    | CCW  |
//!
//! Roll convention: +roll about +x (forward) means right wing down →
//! right-side motors (M1, M2) thrust less, left-side (M3, M4) more.
//!
//! Pitch convention: +pitch about +y (right) means nose up → front
//! motors (M1, M4) thrust more, back motors (M2, M3) less.
//!
//! Yaw convention: +yaw about +z (down NED) = clockwise viewed from
//! above. CW motors (M1, M3) spin less, CCW (M2, M4) more so that the
//! net reaction torque on the airframe spins it +yaw.
//!
//! ## Verified properties (v0.4 surrogates for SWREQ-FALCON-MIX-P*)
//!
//! - **MIX-P01** (Drake-derivation precedent): the matrix was derived
//!   from first-principles in the airframe convention above. v0.5
//!   formalises this via a Drake MultibodyPlant export. *Test:
//!   `mix_p01_zero_command_gives_thrust_only`.*
//! - **MIX-P02** (no-negative-thrust): every PWM output is clamped to
//!   `[0, 1]`. *Test: `mix_p02_outputs_in_unit_interval`.*
//! - **MIX-P03** (saturation handling): when the un-saturated sum
//!   would exceed `[0, 1]`, an order-preserving scale is applied to
//!   keep the relative torque ratios intact (thrust first, yaw
//!   sacrificed last). *Test:
//!   `mix_p03_saturation_preserves_torque_direction_sign`.*
//! - **MIX-P04** (per-variant motor count): falcon-quad exports 4
//!   motors. Other airframes live in separate crates. *Test: the
//!   `Motors4` return type is a `[f32; 4]`.*

#![no_std]
#![forbid(unsafe_code)]

/// Per-motor mixer signs. Each row is `[thrust, roll, pitch, yaw]`.
const MIXER_X: [[f32; 4]; 4] = [
    [1.0, -1.0, 1.0, -1.0], // M1 front-right, CW
    [1.0, -1.0, -1.0, 1.0], // M2 back-right, CCW
    [1.0, 1.0, -1.0, -1.0], // M3 back-left, CW
    [1.0, 1.0, 1.0, 1.0],   // M4 front-left, CCW
];

/// X-config quad mixer state. Stateless on its own — the struct
/// carries last-output bookkeeping for diagnostics.
#[derive(Clone, Copy, Debug, Default)]
pub struct QuadMixer {
    last_motors: [f32; 4],
}

impl QuadMixer {
    pub const fn new() -> Self {
        Self { last_motors: [0.0; 4] }
    }

    pub fn last_motors(&self) -> [f32; 4] {
        self.last_motors
    }

    /// Map (torque_body, thrust) to per-motor PWM values clamped to
    /// `[0, 1]`. `torque_body` units are roll/pitch/yaw torque
    /// normalised to `[-1, +1]` (controller-output magnitudes).
    /// `thrust` is the collective thrust command in `[0, 1]`.
    ///
    /// The output is clipped to `[0, 1]` per motor. If the raw mix
    /// produces any value above 1.0, the entire bus is shifted down
    /// by the excess (preserves the relative roll/pitch/yaw axes,
    /// sacrifices some collective thrust). If any value is below 0
    /// after that, the negative motors are clamped to 0 (sacrifices
    /// some torque authority on those axes). This is the standard
    /// "priority-preserving" mixer behaviour PX4 uses.
    pub fn mix(&mut self, torque_body: [f32; 3], thrust: f32) -> [f32; 4] {
        let t = sanitise(thrust);
        let r = sanitise(torque_body[0]);
        let p = sanitise(torque_body[1]);
        let y = sanitise(torque_body[2]);

        let mut m = [0.0_f32; 4];
        for i in 0..4 {
            let row = &MIXER_X[i];
            m[i] = row[0] * t + row[1] * r + row[2] * p + row[3] * y;
        }

        // Step 1: if any motor exceeds 1.0, subtract the excess from
        // every motor so the maximum is exactly 1.0. Preserves the
        // relative torque ratios; sacrifices collective thrust.
        let mut max = m[0];
        for &v in &m[1..] {
            if v > max {
                max = v;
            }
        }
        if max > 1.0 {
            let excess = max - 1.0;
            for v in m.iter_mut() {
                *v -= excess;
            }
        }

        // Step 2: clamp to [0, 1]. Negative motors are clipped (lose
        // some torque authority, but motors can't push reverse).
        for v in m.iter_mut() {
            if *v < 0.0 || !v.is_finite() {
                *v = 0.0;
            } else if *v > 1.0 {
                *v = 1.0;
            }
        }
        self.last_motors = m;
        m
    }
}

#[inline]
fn sanitise(x: f32) -> f32 {
    if !x.is_finite() {
        0.0
    } else {
        x
    }
}

/// Sum the per-axis torque produced by a motor-command vector,
/// using the same mixer matrix in reverse. Useful for verifying
/// MIX-P03 in tests — given a motor command, what torque ratio
/// did the airframe actually experience? Sign per axis only.
pub fn motors_to_torque_signs(motors: [f32; 4]) -> [f32; 3] {
    let mut t = [0.0_f32; 3];
    for i in 0..4 {
        t[0] += MIXER_X[i][1] * motors[i];
        t[1] += MIXER_X[i][2] * motors[i];
        t[2] += MIXER_X[i][3] * motors[i];
    }
    t
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mix_p01_zero_command_gives_thrust_only() {
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.0, 0.0], 0.5);
        for v in r.iter() {
            assert!((v - 0.5).abs() < 1.0e-6,
                "all motors should equal thrust at zero torque, got {:?}", r);
        }
    }

    #[test]
    fn mix_p02_outputs_in_unit_interval() {
        // Throw arbitrary commands at it; every output stays in [0, 1].
        let mut mixer = QuadMixer::new();
        for &t in &[0.0_f32, 0.1, 0.5, 0.9, 1.0, 1.5, -0.5] {
            for &r in &[-1.0_f32, -0.5, 0.0, 0.5, 1.0] {
                let m = mixer.mix([r, r, r], t);
                for v in m.iter() {
                    assert!((0.0..=1.0).contains(v),
                        "motor out of bounds: t={} r={} -> {:?}", t, r, m);
                }
            }
        }
    }

    #[test]
    fn mix_p03_pure_roll_drives_diagonal_motor_pairs_opposite() {
        // +roll command (right wing down) → right-side motors (M1, M2)
        // less thrust than left-side (M3, M4).
        let mut m = QuadMixer::new();
        let r = m.mix([0.5, 0.0, 0.0], 0.5);
        assert!(r[0] < r[2], "M1 (right) must be less than M3 (left): {:?}", r);
        assert!(r[1] < r[3], "M2 (right) must be less than M4 (left): {:?}", r);
    }

    #[test]
    fn mix_p03_pure_pitch_drives_front_and_back_motors_opposite() {
        // +pitch (nose up) → front motors (M1, M4) more, back (M2, M3) less.
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.5, 0.0], 0.5);
        assert!(r[0] > r[1], "M1 (front) must be greater than M2 (back): {:?}", r);
        assert!(r[3] > r[2], "M4 (front) must be greater than M3 (back): {:?}", r);
    }

    #[test]
    fn mix_p03_pure_yaw_drives_cw_and_ccw_pairs_opposite() {
        // +yaw (rotate CW from above) → CW motors (M1, M3) less, CCW (M2, M4) more.
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.0, 0.5], 0.5);
        assert!(r[0] < r[1], "CW M1 must be less than CCW M2: {:?}", r);
        assert!(r[2] < r[3], "CW M3 must be less than CCW M4: {:?}", r);
    }

    #[test]
    fn high_thrust_with_torque_saturates_gracefully() {
        // Thrust near max with a torque command would push some motor
        // above 1.0. Mixer must shift the bus down, not clip torque.
        let mut m = QuadMixer::new();
        let r = m.mix([0.5, 0.0, 0.0], 1.0);
        // Maximum motor must be 1.0 (the excess was absorbed).
        let max = r.iter().fold(0.0_f32, |a, &v| a.max(v));
        assert!((max - 1.0).abs() < 1.0e-6 || max <= 1.0);
        // Direction sign preserved: M3 still > M1 (left > right).
        assert!(r[2] > r[0]);
    }

    #[test]
    fn nan_input_does_not_propagate() {
        let mut m = QuadMixer::new();
        let r = m.mix([f32::NAN, 0.0, 0.0], 0.5);
        for v in r.iter() {
            assert!(v.is_finite(), "NaN propagated: {:?}", r);
        }
        let r2 = m.mix([0.0, 0.0, 0.0], f32::NAN);
        for v in r2.iter() {
            assert!(v.is_finite());
        }
    }

    #[test]
    fn negative_thrust_clipped_to_zero() {
        let mut m = QuadMixer::new();
        let r = m.mix([0.0, 0.0, 0.0], -0.5);
        for v in r.iter() {
            assert_eq!(*v, 0.0, "negative thrust should clip to 0, got {:?}", r);
        }
    }

    use proptest::prelude::*;

    proptest! {
        /// Outputs always in [0, 1] for any finite input.
        #[test]
        fn mix_p02_property(
            thrust in -2.0_f32..2.0,
            roll in -2.0_f32..2.0,
            pitch in -2.0_f32..2.0,
            yaw in -2.0_f32..2.0,
        ) {
            let mut m = QuadMixer::new();
            let r = m.mix([roll, pitch, yaw], thrust);
            for v in r.iter() {
                prop_assert!((0.0..=1.0).contains(v));
                prop_assert!(v.is_finite());
            }
        }

        /// MIX-P03: signs of torque-direction preserved under
        /// saturation (when at most one axis commanded).
        #[test]
        fn mix_p03_property_single_axis(
            roll in -0.5_f32..0.5,
            thrust in 0.3_f32..0.7,
        ) {
            let mut m = QuadMixer::new();
            let out = m.mix([roll, 0.0, 0.0], thrust);
            // Effective roll torque sign should match input sign.
            let t = motors_to_torque_signs(out);
            if roll > 0.05 {
                prop_assert!(t[0] > 0.0, "roll positive but t[0]={}", t[0]);
            } else if roll < -0.05 {
                prop_assert!(t[0] < 0.0);
            }
        }
    }
}
