//! Relay Mix Multi — hexacopter + coaxial-octo motor mixers.
//!
//! Extends the airframe coverage beyond the quad (relay-mix-quad): an N-rotor
//! control allocator that maps a desired wrench (collective thrust + roll/pitch/
//! yaw torque) to per-rotor commands for hexacopter (6) and coaxial-octo / X8
//! (8) geometries — closing the WORLD-P02 variant gap so the SAME verified
//! cascade flies multiple airframes.
//!
//! The safety invariant — the one that matters for any mixer — holds for ANY
//! geometry and ANY wrench: **every motor command is in [0, 1]** (no negative
//! thrust, never over-unity), because each output is `clamp01` of the allocated
//! value. Kani proves the clamp; a proptest proves the full allocation stays in
//! range across the geometries (the f32 matrix multiply is proptest territory).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// A desired control wrench. `thrust` is the collective [0,1]-ish demand; the
/// torques are normalized roll/pitch/yaw demands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Wrench {
    /// Collective thrust demand.
    pub thrust: f32,
    /// Roll torque demand.
    pub roll: f32,
    /// Pitch torque demand.
    pub pitch: f32,
    /// Yaw torque demand.
    pub yaw: f32,
}

/// An N-rotor mixer: each row is the rotor's `[thrust, roll, pitch, yaw]`
/// effectiveness.
#[derive(Clone, Copy, Debug)]
pub struct Mixer<const M: usize> {
    rows: [[f32; 4]; M],
}

/// Clamp to [0, 1]; NaN ⇒ 0 (a rotor never gets a NaN/negative/over-unity
/// command). The safety floor every mixer output passes through.
#[inline]
pub fn clamp01(x: f32) -> f32 {
    if x.is_nan() {
        0.0
    } else if x < 0.0 {
        0.0
    } else if x > 1.0 {
        1.0
    } else {
        x
    }
}

impl<const M: usize> Mixer<M> {
    /// A mixer from an explicit effectiveness matrix.
    pub fn new(rows: [[f32; 4]; M]) -> Self {
        Self { rows }
    }

    /// Allocate a wrench to per-rotor commands, each clamped to [0, 1].
    pub fn mix(&self, w: Wrench) -> [f32; M] {
        let mut out = [0.0f32; M];
        for i in 0..M {
            let r = &self.rows[i];
            let v = r[0] * w.thrust + r[1] * w.roll + r[2] * w.pitch + r[3] * w.yaw;
            out[i] = clamp01(v);
        }
        out
    }
}

/// Hexacopter (flat-top "X") mixer: 6 rotors at 60° spacing, alternating spin.
/// Standard PX4-style geometry (thrust 1.0 each; roll/pitch by arm angle; yaw by
/// spin direction).
pub fn hex_x() -> Mixer<6> {
    // angles 90,-90,-30,150,30,-150 deg → (sin = roll, cos = pitch); yaw ±.
    Mixer::new([
        [1.0, -1.000, 0.000, 1.0],
        [1.0, 1.000, 0.000, -1.0],
        [1.0, 0.500, -0.866, 1.0],
        [1.0, -0.500, 0.866, -1.0],
        [1.0, -0.500, -0.866, -1.0],
        [1.0, 0.500, 0.866, 1.0],
    ])
}

/// Coaxial-octo (X8): 4 arms, each with a top + bottom rotor (opposite spin).
/// 8 rotors; the top/bottom of an arm share roll/pitch, oppose in yaw.
pub fn octo_coax() -> Mixer<8> {
    Mixer::new([
        [1.0, -0.707, 0.707, 1.0], // arm 1 top
        [1.0, -0.707, 0.707, -1.0], // arm 1 bottom
        [1.0, 0.707, -0.707, 1.0], // arm 2 top
        [1.0, 0.707, -0.707, -1.0], // arm 2 bottom
        [1.0, 0.707, 0.707, -1.0], // arm 3 top
        [1.0, 0.707, 0.707, 1.0], // arm 3 bottom
        [1.0, -0.707, -0.707, -1.0], // arm 4 top
        [1.0, -0.707, -0.707, 1.0], // arm 4 bottom
    ])
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn all_in_range<const M: usize>(out: &[f32; M]) -> bool {
        out.iter().all(|&x| (0.0..=1.0).contains(&x))
    }

    #[test]
    fn hover_wrench_spins_all_rotors() {
        let w = Wrench { thrust: 0.5, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let out = hex_x().mix(w);
        assert!(all_in_range(&out));
        assert!(out.iter().all(|&x| (x - 0.5).abs() < 1e-6)); // even hover
    }

    #[test]
    fn hex_outputs_bounded_under_saturating_command() {
        // a huge mixed demand must clamp, never exceed 1 or go negative.
        let w = Wrench { thrust: 1.0, roll: 5.0, pitch: -5.0, yaw: 3.0 };
        let out = hex_x().mix(w);
        assert!(all_in_range(&out));
    }

    #[test]
    fn coax_outputs_bounded() {
        let w = Wrench { thrust: 0.6, roll: -2.0, pitch: 1.0, yaw: -4.0 };
        let out = octo_coax().mix(w);
        assert!(all_in_range(&out));
    }

    #[test]
    fn nan_command_is_zeroed_not_propagated() {
        let w = Wrench { thrust: f32::NAN, roll: 0.0, pitch: 0.0, yaw: 0.0 };
        let out = hex_x().mix(w);
        assert!(all_in_range(&out));
        assert!(out.iter().all(|&x| x == 0.0));
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// For ANY wrench, every hex + coax rotor command is in [0,1] (no
        /// negative thrust, never over-unity) — the airframe-agnostic invariant.
        #[test]
        fn outputs_in_unit_interval(
            t in -10.0f32..10.0, r in -10.0f32..10.0, p in -10.0f32..10.0, y in -10.0f32..10.0,
        ) {
            let w = Wrench { thrust: t, roll: r, pitch: p, yaw: y };
            for &x in hex_x().mix(w).iter() {
                prop_assert!((0.0..=1.0).contains(&x));
            }
            for &x in octo_coax().mix(w).iter() {
                prop_assert!((0.0..=1.0).contains(&x));
            }
        }
    }
}
