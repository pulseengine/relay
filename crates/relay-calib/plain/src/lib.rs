//! Relay Calib — verified sensor-calibration math.
//!
//! An uncalibrated mag/accel diverges even a proven estimator: a fixed gyro bias
//! integrates into attitude drift, an accel bias/scale error tilts the gravity
//! reference, and magnetometer hard/soft-iron warps heading. This crate is the
//! calibration leaf the falcon estimator consumes — pure, `no_std`/no_alloc/
//! `forbid(unsafe)` math:
//!
//!   * [`gyro_null`] — gyro bias = the mean of at-rest samples (null-rate).
//!   * [`accel_6point`] — per-axis bias + scale from the ±g face readings
//!     (the 6-orientation tumble distilled to a positive-g and negative-g
//!     reading per axis).
//!   * [`mag_hardiron`] / [`mag_softiron_diag`] — magnetometer offset (hard iron)
//!     and a diagonal soft-iron scale, from the min/max sweep over a rotation.
//!     (The full off-diagonal ellipsoid fit is a documented follow-up; the
//!     hard-iron offset is the dominant correction and is Kani-tractable.)
//!
//! The solved [`CalParams`] feed [`CalParams::apply_gyro`]/`apply_accel`/
//! `apply_mag`, which the estimator runs on each raw sample before the IEKF
//! propagate/update. [`CalParams::identity`] (zero bias, unit scale) is a no-op —
//! the explicit replacement for the estimator's prior identity-remap placeholder.
//! The offsets serialise to the relay-param store via [`CalParams::to_named`].
//!
//! ## Verification split (the codebase's f32 discipline)
//!   * **Kani** proves TOTALITY + bounds: the solvers never panic / index out of
//!     bounds for any sample buffer; `apply` is total; identity is a no-op.
//!   * **proptest** proves the NUMERICS: synthetic data with a KNOWN injected
//!     bias/scale — the solver recovers it and `apply` corrects it back (synthetic
//!     ground truth is the legitimate oracle for calibration math).

#![no_std]
#![forbid(unsafe_code)]

/// A 3-vector (body-frame axes), matching `relay_iekf::Vec3`.
pub type Vec3 = [f32; 3];

/// Standard gravity (m/s²) — the accel 6-orientation reference magnitude.
pub const G: f32 = 9.806_65;

/// Per-sensor calibration: gyro bias, accel bias+scale, mag offset+scale.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CalParams {
    /// Gyro zero-rate bias (rad/s), subtracted from raw.
    pub gyro_bias: Vec3,
    /// Accel zero-g bias (m/s²), subtracted from raw.
    pub accel_bias: Vec3,
    /// Accel per-axis scale factor (unitless), multiplied after de-biasing.
    pub accel_scale: Vec3,
    /// Mag hard-iron offset (gauss), subtracted from raw.
    pub mag_offset: Vec3,
    /// Mag soft-iron diagonal scale (unitless), multiplied after de-offsetting.
    pub mag_scale: Vec3,
}

impl Default for CalParams {
    fn default() -> Self {
        Self::identity()
    }
}

impl CalParams {
    /// The identity calibration: zero bias, unit scale — `apply_*` is the
    /// identity map. The explicit replacement for the estimator's prior
    /// identity-remap placeholder (raw samples flowed in uncorrected).
    pub const fn identity() -> Self {
        CalParams {
            gyro_bias: [0.0; 3],
            accel_bias: [0.0; 3],
            accel_scale: [1.0; 3],
            mag_offset: [0.0; 3],
            mag_scale: [1.0; 3],
        }
    }

    /// Calibrated gyro = raw − bias.
    pub fn apply_gyro(&self, raw: Vec3) -> Vec3 {
        sub(raw, self.gyro_bias)
    }

    /// Calibrated accel = scale · (raw − bias).
    pub fn apply_accel(&self, raw: Vec3) -> Vec3 {
        hadamard(self.accel_scale, sub(raw, self.accel_bias))
    }

    /// Calibrated mag = scale · (raw − offset).
    pub fn apply_mag(&self, raw: Vec3) -> Vec3 {
        hadamard(self.mag_scale, sub(raw, self.mag_offset))
    }

    /// The 15 (param-id, value) pairs for the relay-param store. PX4-style
    /// CAL_* naming; the caller registers/loads them against a `ParamStore`.
    pub fn to_named(&self) -> [(&'static str, f32); 15] {
        [
            ("CAL_GYRO0_XOFF", self.gyro_bias[0]),
            ("CAL_GYRO0_YOFF", self.gyro_bias[1]),
            ("CAL_GYRO0_ZOFF", self.gyro_bias[2]),
            ("CAL_ACC0_XOFF", self.accel_bias[0]),
            ("CAL_ACC0_YOFF", self.accel_bias[1]),
            ("CAL_ACC0_ZOFF", self.accel_bias[2]),
            ("CAL_ACC0_XSCALE", self.accel_scale[0]),
            ("CAL_ACC0_YSCALE", self.accel_scale[1]),
            ("CAL_ACC0_ZSCALE", self.accel_scale[2]),
            ("CAL_MAG0_XOFF", self.mag_offset[0]),
            ("CAL_MAG0_YOFF", self.mag_offset[1]),
            ("CAL_MAG0_ZOFF", self.mag_offset[2]),
            ("CAL_MAG0_XSCALE", self.mag_scale[0]),
            ("CAL_MAG0_YSCALE", self.mag_scale[1]),
            ("CAL_MAG0_ZSCALE", self.mag_scale[2]),
        ]
    }
}

#[inline]
fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

#[inline]
fn hadamard(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] * b[0], a[1] * b[1], a[2] * b[2]]
}

/// Gyro null-rate calibration: the bias is the mean of the at-rest samples.
/// Total: an empty slice yields zero bias (no calibration). Never panics.
pub fn gyro_null(samples: &[Vec3]) -> Vec3 {
    let n = samples.len();
    if n == 0 {
        return [0.0; 3];
    }
    let mut s = [0.0f32; 3];
    for v in samples {
        s[0] += v[0];
        s[1] += v[1];
        s[2] += v[2];
    }
    let inv = 1.0 / n as f32;
    [s[0] * inv, s[1] * inv, s[2] * inv]
}

/// Accel 6-orientation bias/scale solve. `pos[a]` is the axis-`a` reading when
/// axis `a` is oriented to read +g; `neg[a]` is the reading at −g. From the two
/// per-axis endpoints: `bias = (pos + neg) / 2`, `scale = g / ((pos − neg) / 2)`.
/// Total: a degenerate (pos == neg) axis keeps unit scale (no division by zero).
pub fn accel_6point(pos: Vec3, neg: Vec3, g: f32) -> (Vec3, Vec3) {
    let mut bias = [0.0f32; 3];
    let mut scale = [1.0f32; 3];
    let mut a = 0;
    while a < 3 {
        bias[a] = (pos[a] + neg[a]) * 0.5;
        let half = (pos[a] - neg[a]) * 0.5;
        scale[a] = if half != 0.0 { g / half } else { 1.0 };
        a += 1;
    }
    (bias, scale)
}

/// Per-axis (min, max) over a sample set. Total: empty → ([0;3], [0;3]).
fn min_max(samples: &[Vec3]) -> (Vec3, Vec3) {
    if samples.is_empty() {
        return ([0.0; 3], [0.0; 3]);
    }
    let mut lo = samples[0];
    let mut hi = samples[0];
    for v in samples {
        let mut a = 0;
        while a < 3 {
            if v[a] < lo[a] {
                lo[a] = v[a];
            }
            if v[a] > hi[a] {
                hi[a] = v[a];
            }
            a += 1;
        }
    }
    (lo, hi)
}

/// Magnetometer hard-iron offset: the per-axis midpoint of the min/max sweep over
/// a full rotation. Total: empty → zero offset. Never panics.
pub fn mag_hardiron(samples: &[Vec3]) -> Vec3 {
    let (lo, hi) = min_max(samples);
    [(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5]
}

/// Magnetometer soft-iron DIAGONAL scale: normalise each axis's half-range to the
/// mean half-range (a diagonal approximation of the soft-iron ellipsoid). Total:
/// a degenerate (zero-range) axis keeps unit scale. The off-diagonal ellipsoid
/// fit is a documented follow-up.
pub fn mag_softiron_diag(samples: &[Vec3]) -> Vec3 {
    let (lo, hi) = min_max(samples);
    let half = [(hi[0] - lo[0]) * 0.5, (hi[1] - lo[1]) * 0.5, (hi[2] - lo[2]) * 0.5];
    let avg = (half[0] + half[1] + half[2]) / 3.0;
    let mut scale = [1.0f32; 3];
    let mut a = 0;
    while a < 3 {
        scale[a] = if half[a] != 0.0 { avg / half[a] } else { 1.0 };
        a += 1;
    }
    scale
}

/// Solve a full [`CalParams`] from the raw calibration data: at-rest gyro
/// samples, the accel ±g face endpoints, and the mag rotation sweep.
pub fn solve(gyro_rest: &[Vec3], accel_pos: Vec3, accel_neg: Vec3, g: f32, mag_sweep: &[Vec3]) -> CalParams {
    let (accel_bias, accel_scale) = accel_6point(accel_pos, accel_neg, g);
    CalParams {
        gyro_bias: gyro_null(gyro_rest),
        accel_bias,
        accel_scale,
        mag_offset: mag_hardiron(mag_sweep),
        mag_scale: mag_softiron_diag(mag_sweep),
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn close(a: Vec3, b: Vec3, tol: f32) -> bool {
        (a[0] - b[0]).abs() < tol && (a[1] - b[1]).abs() < tol && (a[2] - b[2]).abs() < tol
    }

    #[test]
    fn identity_is_a_no_op() {
        let c = CalParams::identity();
        let raw = [1.5, -2.0, 3.25];
        assert_eq!(c.apply_gyro(raw), raw);
        assert_eq!(c.apply_accel(raw), raw);
        assert_eq!(c.apply_mag(raw), raw);
    }

    #[test]
    fn gyro_null_recovers_known_bias() {
        // samples = bias + a symmetric dither that averages out.
        let bias = [0.01, -0.02, 0.005];
        let samples = [
            [bias[0] + 0.001, bias[1] - 0.001, bias[2] + 0.002],
            [bias[0] - 0.001, bias[1] + 0.001, bias[2] - 0.002],
            [bias[0], bias[1], bias[2]],
        ];
        assert!(close(gyro_null(&samples), bias, 1e-6));
    }

    #[test]
    fn accel_6point_recovers_bias_and_scale() {
        // ground truth: bias b, scale s. The +g face reads s·g + b, the −g face
        // reads −s·g + b (raw = truth/scale + bias inverted: reading = truth*... ).
        let b = [0.3, -0.2, 0.1];
        let s = [1.02, 0.98, 1.05];
        // reading at +g on axis a = (G / s_inv)?  Construct: raw_pos = b + G/s? No:
        // apply: corrected = scale·(raw − bias) must equal ±G. So raw = ±G/scale + bias.
        let pos = [G / s[0] + b[0], G / s[1] + b[1], G / s[2] + b[2]];
        let neg = [-G / s[0] + b[0], -G / s[1] + b[1], -G / s[2] + b[2]];
        let (bias, scale) = accel_6point(pos, neg, G);
        assert!(close(bias, b, 1e-4), "bias {bias:?} vs {b:?}");
        assert!(close(scale, s, 1e-4), "scale {scale:?} vs {s:?}");
        // and applying the solved cal to the +g reading yields +G per axis.
        let cal = CalParams { accel_bias: bias, accel_scale: scale, ..CalParams::identity() };
        let corrected = cal.apply_accel(pos);
        assert!(close(corrected, [G, G, G], 1e-3));
    }

    #[test]
    fn mag_hardiron_recovers_offset() {
        // samples sweeping a sphere of radius r centred at offset → midpoint = offset.
        let off = [0.2, -0.1, 0.05];
        let r = 0.5;
        let sweep = [
            [off[0] + r, off[1], off[2]],
            [off[0] - r, off[1], off[2]],
            [off[0], off[1] + r, off[2]],
            [off[0], off[1] - r, off[2]],
            [off[0], off[1], off[2] + r],
            [off[0], off[1], off[2] - r],
        ];
        assert!(close(mag_hardiron(&sweep), off, 1e-6));
    }

    #[test]
    fn mag_softiron_equalises_axis_ranges() {
        // an axis with a squashed range gets scaled up toward the mean range.
        let sweep = [
            [2.0, 0.0, 0.0],
            [-2.0, 0.0, 0.0], // x half-range 2
            [0.0, 1.0, 0.0],
            [0.0, -1.0, 0.0], // y half-range 1
            [0.0, 0.0, 3.0],
            [0.0, 0.0, -3.0], // z half-range 3
        ];
        let s = mag_softiron_diag(&sweep);
        // mean half-range = 2; so scale = [2/2, 2/1, 2/3] = [1, 2, 0.667].
        assert!(close(s, [1.0, 2.0, 2.0 / 3.0], 1e-4));
    }

    #[test]
    fn empty_inputs_are_total() {
        assert_eq!(gyro_null(&[]), [0.0; 3]);
        assert_eq!(mag_hardiron(&[]), [0.0; 3]);
        assert_eq!(mag_softiron_diag(&[]), [1.0; 3]);
    }

    #[test]
    fn degenerate_axis_keeps_unit_scale() {
        // pos == neg on an axis → no division by zero, unit scale.
        let (_, scale) = accel_6point([1.0, 1.0, 1.0], [1.0, 1.0, 1.0], G);
        assert_eq!(scale, [1.0; 3]);
    }

    #[test]
    fn to_named_has_15_cal_params() {
        let named = CalParams::identity().to_named();
        assert_eq!(named.len(), 15);
        assert_eq!(named[0].0, "CAL_GYRO0_XOFF");
        assert_eq!(named[6].0, "CAL_ACC0_XSCALE");
        // identity scales are 1.0, offsets 0.0.
        assert_eq!(named[6].1, 1.0);
        assert_eq!(named[0].1, 0.0);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// For ANY in-range bias/scale, the accel 6-point solve recovers them and
        /// applying the result corrects a +g face reading back to +G.
        #[test]
        fn accel_solve_recovers_synthetic(
            bx in -1.0f32..1.0, by in -1.0f32..1.0, bz in -1.0f32..1.0,
            sx in 0.8f32..1.2, sy in 0.8f32..1.2, sz in 0.8f32..1.2,
        ) {
            let b = [bx, by, bz];
            let s = [sx, sy, sz];
            let pos = [G / s[0] + b[0], G / s[1] + b[1], G / s[2] + b[2]];
            let neg = [-G / s[0] + b[0], -G / s[1] + b[1], -G / s[2] + b[2]];
            let (bias, scale) = accel_6point(pos, neg, G);
            prop_assert!((bias[0] - b[0]).abs() < 1e-3);
            prop_assert!((bias[1] - b[1]).abs() < 1e-3);
            prop_assert!((bias[2] - b[2]).abs() < 1e-3);
            prop_assert!((scale[0] - s[0]).abs() < 1e-3);
            prop_assert!((scale[1] - s[1]).abs() < 1e-3);
            prop_assert!((scale[2] - s[2]).abs() < 1e-3);
            let cal = CalParams { accel_bias: bias, accel_scale: scale, ..CalParams::identity() };
            let c = cal.apply_accel(pos);
            prop_assert!((c[0] - G).abs() < 1e-2);
            prop_assert!((c[1] - G).abs() < 1e-2);
            prop_assert!((c[2] - G).abs() < 1e-2);
        }

        /// gyro_null on a constant-bias buffer (any length ≥ 1) returns that bias.
        #[test]
        fn gyro_null_constant(bx in -1.0f32..1.0, n in 1usize..32) {
            let buf = [[bx, bx, bx]; 32];
            let got = gyro_null(&buf[..n]);
            prop_assert!((got[0] - bx).abs() < 1e-4);
        }
    }
}
