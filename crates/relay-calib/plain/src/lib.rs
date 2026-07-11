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

pub mod flow;

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod flow_tests {
    use super::flow::*;
    use super::*;

    // ── Gyro null flow ──────────────────────────────────────────────────

    #[test]
    fn gyro_flow_captures_bias_and_matches_solver() {
        let mut f = GyroNullFlow::new(4, 0.1);
        let samples = [
            [0.010, -0.020, 0.005],
            [0.012, -0.018, 0.004],
            [0.008, -0.022, 0.006],
            [0.011, -0.019, 0.005],
        ];
        for s in &samples[..3] {
            assert!(matches!(f.step(*s), WindowStatus::Collecting { .. }));
        }
        assert_eq!(f.step(samples[3]), WindowStatus::Done);
        // Equivalence with the verified solver over the same samples.
        let expect = gyro_null(&samples);
        let got = f.bias().unwrap();
        for a in 0..3 {
            assert!((got[a] - expect[a]).abs() < 1e-7);
        }
    }

    #[test]
    fn gyro_flow_motion_restarts_window() {
        let mut f = GyroNullFlow::new(3, 0.1);
        f.step([0.01, 0.0, 0.0]);
        f.step([0.01, 0.0, 0.0]);
        // Bump: over threshold ⇒ restart, nothing from before survives.
        assert!(matches!(f.step([0.5, 0.0, 0.0]), WindowStatus::Collecting { remaining: 3 }));
        assert_eq!(f.restarts(), 1);
        assert_eq!(f.bias(), None);
        // NaN is also disqualifying.
        f.step([0.01, 0.0, 0.0]);
        assert!(matches!(f.step([f32::NAN, 0.0, 0.0]), WindowStatus::Collecting { remaining: 3 }));
        assert_eq!(f.restarts(), 2);
    }

    // ── Accel 6-point flow ──────────────────────────────────────────────

    const G: f32 = 9.81;

    fn feed_face(f: &mut Accel6PointFlow, sample: Vec3, n: u32) -> SixPointStatus {
        let mut last = SixPointStatus::WaitingForFace;
        for _ in 0..n {
            last = f.step(sample);
        }
        last
    }

    #[test]
    fn accel_flow_captures_all_faces_and_matches_solver() {
        let mut f = Accel6PointFlow::new(3, G);
        // Slight bias on x (+0.1) so the solve is non-trivial.
        assert!(matches!(feed_face(&mut f, [G + 0.1, 0.0, 0.0], 3), SixPointStatus::FaceCaptured { face: Face::XPos }));
        assert!(matches!(feed_face(&mut f, [-G + 0.1, 0.0, 0.0], 3), SixPointStatus::FaceCaptured { face: Face::XNeg }));
        assert!(matches!(feed_face(&mut f, [0.0, G, 0.0], 3), SixPointStatus::FaceCaptured { face: Face::YPos }));
        assert!(matches!(feed_face(&mut f, [0.0, -G, 0.0], 3), SixPointStatus::FaceCaptured { face: Face::YNeg }));
        assert!(matches!(feed_face(&mut f, [0.0, 0.0, G], 3), SixPointStatus::FaceCaptured { face: Face::ZPos }));
        assert_eq!(feed_face(&mut f, [0.0, 0.0, -G], 3), SixPointStatus::Done);
        assert_eq!(f.captured_mask(), 0b11_1111);
        let (bias, scale) = f.solve().unwrap();
        let (eb, es) = accel_6point([G + 0.1, G, G], [-G + 0.1, -G, -G], G);
        for a in 0..3 {
            assert!((bias[a] - eb[a]).abs() < 1e-5);
            assert!((scale[a] - es[a]).abs() < 1e-5);
        }
        // x bias ≈ +0.1, x scale ≈ 1.0.
        assert!((bias[0] - 0.1).abs() < 1e-4);
        assert!((scale[0] - 1.0).abs() < 1e-4);
    }

    #[test]
    fn accel_flow_rejects_tilted_and_shaky_faces() {
        let mut f = Accel6PointFlow::new(3, G);
        // Tilted 45° — no dominant axis within gates ⇒ never recognised.
        assert_eq!(f.step([G * 0.7, G * 0.7, 0.0]), SixPointStatus::WaitingForFace);
        // Start a valid face then shake out of it ⇒ that window restarts.
        assert!(matches!(f.step([G, 0.0, 0.0]), SixPointStatus::Sampling { face: Face::XPos, remaining: 2 }));
        assert_eq!(f.step([G * 0.5, G * 0.5, 0.0]), SixPointStatus::WaitingForFace);
        // Window restarted: needs the full count again.
        assert!(matches!(f.step([G, 0.0, 0.0]), SixPointStatus::Sampling { face: Face::XPos, remaining: 2 }));
    }

    #[test]
    fn accel_flow_never_rerecords_a_face() {
        let mut f = Accel6PointFlow::new(2, G);
        assert!(matches!(feed_face(&mut f, [G, 0.0, 0.0], 2), SixPointStatus::FaceCaptured { face: Face::XPos }));
        // Presenting the same face again is ignored (WaitingForFace).
        assert_eq!(f.step([G, 0.0, 0.0]), SixPointStatus::WaitingForFace);
        assert_eq!(f.captured_mask(), 0b00_0001);
    }

    // ── Mag sweep flow ──────────────────────────────────────────────────

    #[test]
    fn mag_flow_good_sweep_matches_solvers() {
        // A ring in x/y with offset (0.1, -0.2, 0.05) and z wobble wide
        // enough to pass the anisotropy gate (a 3-D tumble, as instructed).
        let mut f = MagSweepFlow::new(8);
        let mut samples = [[0.0f32; 3]; 8];
        let ring = [
            (0.4f32, 0.0f32),
            (0.283, 0.283),
            (0.0, 0.4),
            (-0.283, 0.283),
            (-0.4, 0.0),
            (-0.283, -0.283),
            (0.0, -0.4),
            (0.283, -0.283),
        ];
        for (i, (x, y)) in ring.iter().enumerate() {
            let z = if i % 2 == 0 { 0.35 } else { -0.25 };
            samples[i] = [0.1 + x, -0.2 + y, 0.05 + z];
            f.step(samples[i]);
        }
        let (verdict, fit) = f.finish(0.05, 3.0);
        assert_eq!(verdict, MagSweepVerdict::Accepted);
        let (offset, scale) = fit.unwrap();
        let eo = mag_hardiron(&samples);
        let es = mag_softiron_diag(&samples);
        for a in 0..3 {
            assert!((offset[a] - eo[a]).abs() < 1e-6, "offset axis {a}");
            assert!((scale[a] - es[a]).abs() < 1e-6, "scale axis {a}");
        }
    }

    #[test]
    fn mag_flow_rejects_thin_sweep() {
        // A lazy "sweep": barely any rotation — z span is tiny.
        let mut f = MagSweepFlow::new(4);
        for i in 0..4 {
            let t = i as f32 * 0.05;
            f.step([0.4 + t * 0.01, 0.02 * t, 0.001 * t]);
        }
        let (verdict, fit) = f.finish(0.05, 3.0);
        assert!(matches!(verdict, MagSweepVerdict::Rejected { .. }));
        assert!(fit.is_none());
    }

    // ── Persistence round-trip ──────────────────────────────────────────

    /// CALIB-P02 end-to-end: run all three flows, solve, PERSIST via the
    /// PARAM-P03 store (CAL_* params + a CAL_VALID flag), "reboot" (fresh
    /// store, load from the same NVM), and get bit-identical CalParams back
    /// — while a fresh/never-calibrated device loads CAL_VALID=0 (the value
    /// the pre-arm `calibration_present` gate consumes).
    #[test]
    fn calibrate_persist_reboot_roundtrip() {
        use relay_param::persist::{load, save, ArrayNvm, Layout, LoadOutcome};
        use relay_param::{param_id, ParamDef, ParamStore};

        // 1. Flows produce a calibration.
        let mut gy = GyroNullFlow::new(4, 0.1);
        for _ in 0..4 {
            gy.step([0.01, -0.02, 0.005]);
        }
        let gyro_bias = gy.bias().unwrap();
        let mut acc = Accel6PointFlow::new(2, 9.81);
        for s in [
            [9.91f32, 0.0, 0.0],
            [-9.71, 0.0, 0.0],
            [0.0, 9.81, 0.0],
            [0.0, -9.81, 0.0],
            [0.0, 0.0, 9.81],
            [0.0, 0.0, -9.81],
        ] {
            acc.step(s);
            acc.step(s);
        }
        let (accel_bias, accel_scale) = acc.solve().unwrap();
        let cal = CalParams {
            gyro_bias,
            accel_bias,
            accel_scale,
            mag_offset: [0.1, -0.2, 0.05],
            mag_scale: [1.1, 0.9, 1.0],
        };

        // 2. Persist: 15 CAL_* params + CAL_VALID, via the PARAM-P03 codec.
        const LAYOUT: Layout = Layout::new(16);
        const CAP: usize = LAYOUT.required_capacity();
        fn schema() -> ParamStore<16> {
            let mut s = ParamStore::new();
            for (name, _) in CalParams::identity().to_named() {
                // Generous physical bounds; defaults = identity calibration.
                let d = CalParams::identity().to_named();
                let default = d.iter().find(|(n, _)| *n == name).unwrap().1;
                s.register(ParamDef { id: param_id(name), min: -50.0, max: 50.0, default });
            }
            s.register(ParamDef { id: param_id("CAL_VALID"), min: 0.0, max: 1.0, default: 0.0 });
            s
        }
        let mut store = schema();
        for (name, v) in cal.to_named() {
            assert_eq!(store.set(&param_id(name), v), relay_param::SetResult::Applied);
        }
        store.set(&param_id("CAL_VALID"), 1.0);
        let mut nvm: ArrayNvm<CAP> = ArrayNvm::new();
        save(&store, &mut nvm, LAYOUT, 1).unwrap();

        // 3. "Reboot": fresh store, load, rebuild CalParams.
        let mut store2 = schema();
        let r = load(&mut store2, &nvm, LAYOUT, 1);
        assert_eq!(r.outcome, LoadOutcome::Loaded);
        assert_eq!(store2.get(&param_id("CAL_VALID")), Some(1.0));
        let mut vals = [0.0f32; 15];
        for (i, (name, _)) in CalParams::identity().to_named().iter().enumerate() {
            vals[i] = store2.get(&param_id(name)).unwrap();
        }
        let back = CalParams::from_values(vals);
        assert_eq!(back, cal, "reboot must reproduce the calibration bit-exactly");

        // 4. A NEVER-CALIBRATED device: fresh NVM ⇒ defaults ⇒ CAL_VALID=0 —
        // the value that keeps pre-arm `calibration_present` false.
        let blank: ArrayNvm<CAP> = ArrayNvm::new();
        let mut store3 = schema();
        assert_eq!(load(&mut store3, &blank, LAYOUT, 1).outcome, LoadOutcome::FreshDefaults);
        assert_eq!(store3.get(&param_id("CAL_VALID")), Some(0.0));
    }

    #[test]
    fn calparams_named_roundtrip_bit_exact() {
        let cal = CalParams {
            gyro_bias: [0.01, -0.02, 0.003],
            accel_bias: [0.1, -0.05, 0.2],
            accel_scale: [1.01, 0.99, 1.002],
            mag_offset: [0.12, -0.08, 0.03],
            mag_scale: [1.05, 0.95, 1.0],
        };
        let named = cal.to_named();
        let mut vals = [0.0f32; 15];
        for (i, (_, v)) in named.iter().enumerate() {
            vals[i] = *v;
        }
        let back = CalParams::from_values(vals);
        assert_eq!(back, cal);
    }
}

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
