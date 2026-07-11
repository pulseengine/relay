//! ON-VEHICLE calibration FLOWS (CALIB-P02, v1.117) — the state machines
//! around the verified solvers.
//!
//! The solver math (gyro_null / accel_6point / mag_hardiron+softiron) has been
//! Kani-verified since CALIB-P01; what was missing is the FLOW that runs on
//! the vehicle: collect the right samples, reject the wrong ones (motion
//! during a gyro null, a tilted or shaky face during the 6-point, a lazy mag
//! sweep), and refuse to produce a calibration from bad data. A quality gate
//! that stores a bad fit is worse than no calibration — every flow here either
//! completes with solver-grade inputs or reports WHY it hasn't.
//!
//! Streaming + fixed-size: no sample buffers (the gyro/mag flows keep running
//! aggregates that are algebraically identical to the solvers over the same
//! samples — pinned by equivalence tests), `no_std`, no panics.

use crate::{accel_6point, CalParams, Vec3};

// ── Gyro null flow ───────────────────────────────────────────────────────────

/// Streaming at-rest gyro-bias capture with MOTION REJECTION: any sample whose
/// magnitude exceeds the motion threshold restarts the window (a bumped
/// vehicle never averages motion into its bias).
pub struct GyroNullFlow {
    sum: Vec3,
    n: u32,
    needed: u32,
    motion_thresh: f32,
    restarts: u32,
}

/// Progress of a flow that collects a fixed window.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindowStatus {
    /// Still collecting; `remaining` samples to go.
    Collecting { remaining: u32 },
    /// Window complete — the result is available.
    Done,
}

impl GyroNullFlow {
    /// `needed` samples of stillness; any sample with |ω| ∞-norm above
    /// `motion_thresh` (rad/s) restarts the window.
    pub fn new(needed: u32, motion_thresh: f32) -> Self {
        GyroNullFlow {
            sum: [0.0; 3],
            n: 0,
            needed: needed.max(1),
            motion_thresh: if motion_thresh.is_finite() && motion_thresh > 0.0 {
                motion_thresh
            } else {
                0.1
            },
            restarts: 0,
        }
    }

    /// Feed one gyro sample (rad/s). Non-finite samples restart (sensor glitch
    /// during a calibration window is disqualifying, not ignorable).
    pub fn step(&mut self, gyro: Vec3) -> WindowStatus {
        if self.n >= self.needed {
            return WindowStatus::Done;
        }
        let still = gyro.iter().all(|a| a.is_finite() && a.abs() <= self.motion_thresh);
        if !still {
            self.sum = [0.0; 3];
            self.n = 0;
            self.restarts = self.restarts.saturating_add(1);
            return WindowStatus::Collecting { remaining: self.needed };
        }
        self.sum[0] += gyro[0];
        self.sum[1] += gyro[1];
        self.sum[2] += gyro[2];
        self.n += 1;
        if self.n >= self.needed {
            WindowStatus::Done
        } else {
            WindowStatus::Collecting { remaining: self.needed - self.n }
        }
    }

    /// The captured bias — `Some` only once the window completed. Identical to
    /// [`crate::gyro_null`] over the accepted samples (equivalence-tested).
    pub fn bias(&self) -> Option<Vec3> {
        if self.n >= self.needed {
            let inv = 1.0 / self.n as f32;
            Some([self.sum[0] * inv, self.sum[1] * inv, self.sum[2] * inv])
        } else {
            None
        }
    }

    /// How many times motion restarted the window (operator feedback).
    pub fn restarts(&self) -> u32 {
        self.restarts
    }
}

// ── Accel 6-point flow ───────────────────────────────────────────────────────

/// The six calibration orientations (which body axis points UP, i.e. reads +g
/// on that axis... the axis aligned AGAINST gravity reads +g for an accel
/// measuring specific force at rest).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Face {
    XPos = 0,
    XNeg = 1,
    YPos = 2,
    YNeg = 3,
    ZPos = 4,
    ZNeg = 5,
}

/// Operator-guided 6-orientation accel capture with per-face QUALITY GATES:
/// a face is recognised only when one axis dominates near ±g and the others
/// are near zero; a face window restarts on shake; each face records once.
pub struct Accel6PointFlow {
    /// Per-face mean accumulator + count; index = `Face as usize`.
    sum: [Vec3; 6],
    n: [u32; 6],
    captured: [bool; 6],
    /// Face endpoint means (dominant-axis reading), filled as faces complete.
    face_mean: [Vec3; 6],
    needed: u32,
    g: f32,
    /// Dominant axis must be at least this fraction of g.
    dom_frac: f32,
    /// Off axes must be below this fraction of g.
    off_frac: f32,
    current: Option<Face>,
}

/// Progress of the 6-point flow.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SixPointStatus {
    /// No recognisable stable face is presented (tilted/shaking/already done).
    WaitingForFace,
    /// Sampling the given face; `remaining` samples to go.
    Sampling { face: Face, remaining: u32 },
    /// The given face just completed.
    FaceCaptured { face: Face },
    /// All six faces captured — the solve is available.
    Done,
}

impl Accel6PointFlow {
    /// `needed` still samples per face; `g` the local gravity magnitude.
    pub fn new(needed: u32, g: f32) -> Self {
        Accel6PointFlow {
            sum: [[0.0; 3]; 6],
            n: [0; 6],
            captured: [false; 6],
            face_mean: [[0.0; 3]; 6],
            needed: needed.max(1),
            g: if g.is_finite() && g > 0.0 { g } else { 9.81 },
            dom_frac: 0.8,
            off_frac: 0.3,
            current: None,
        }
    }

    /// Which face (if any) a sample presents, per the quality gate.
    fn classify(&self, a: Vec3) -> Option<Face> {
        if !a.iter().all(|v| v.is_finite()) {
            return None;
        }
        let dom = self.dom_frac * self.g;
        let off = self.off_frac * self.g;
        let faces = [
            (Face::XPos, a[0] >= dom, a[1].abs() <= off && a[2].abs() <= off),
            (Face::XNeg, a[0] <= -dom, a[1].abs() <= off && a[2].abs() <= off),
            (Face::YPos, a[1] >= dom, a[0].abs() <= off && a[2].abs() <= off),
            (Face::YNeg, a[1] <= -dom, a[0].abs() <= off && a[2].abs() <= off),
            (Face::ZPos, a[2] >= dom, a[0].abs() <= off && a[1].abs() <= off),
            (Face::ZNeg, a[2] <= -dom, a[0].abs() <= off && a[1].abs() <= off),
        ];
        for (f, dom_ok, off_ok) in faces {
            if dom_ok && off_ok {
                return Some(f);
            }
        }
        None
    }

    /// Feed one accel sample (m/s², specific force at rest). A sample that
    /// stops matching the in-progress face (shake/tilt) restarts that face's
    /// window; an already-captured face is ignored (never re-recorded).
    pub fn step(&mut self, accel: Vec3) -> SixPointStatus {
        if self.captured.iter().all(|&c| c) {
            return SixPointStatus::Done;
        }
        let face = match self.classify(accel) {
            Some(f) if !self.captured[f as usize] => f,
            _ => {
                // Lost the face mid-window ⇒ that window restarts.
                if let Some(f) = self.current.take() {
                    self.sum[f as usize] = [0.0; 3];
                    self.n[f as usize] = 0;
                }
                return SixPointStatus::WaitingForFace;
            }
        };
        if self.current != Some(face) {
            // Switched faces: restart the new face's window.
            self.sum[face as usize] = [0.0; 3];
            self.n[face as usize] = 0;
            self.current = Some(face);
        }
        let i = face as usize;
        self.sum[i][0] += accel[0];
        self.sum[i][1] += accel[1];
        self.sum[i][2] += accel[2];
        self.n[i] += 1;
        if self.n[i] >= self.needed {
            let inv = 1.0 / self.n[i] as f32;
            self.face_mean[i] = [self.sum[i][0] * inv, self.sum[i][1] * inv, self.sum[i][2] * inv];
            self.captured[i] = true;
            self.current = None;
            if self.captured.iter().all(|&c| c) {
                SixPointStatus::Done
            } else {
                SixPointStatus::FaceCaptured { face }
            }
        } else {
            SixPointStatus::Sampling { face, remaining: self.needed - self.n[i] }
        }
    }

    /// Bitmask of captured faces (operator progress display).
    pub fn captured_mask(&self) -> u8 {
        let mut m = 0u8;
        for (i, &c) in self.captured.iter().enumerate() {
            if c {
                m |= 1 << i;
            }
        }
        m
    }

    /// The solved (bias, scale) — `Some` only with all six faces captured.
    /// Endpoints feed the Kani-verified [`accel_6point`] unchanged.
    pub fn solve(&self) -> Option<(Vec3, Vec3)> {
        if !self.captured.iter().all(|&c| c) {
            return None;
        }
        let pos = [
            self.face_mean[Face::XPos as usize][0],
            self.face_mean[Face::YPos as usize][1],
            self.face_mean[Face::ZPos as usize][2],
        ];
        let neg = [
            self.face_mean[Face::XNeg as usize][0],
            self.face_mean[Face::YNeg as usize][1],
            self.face_mean[Face::ZNeg as usize][2],
        ];
        Some(accel_6point(pos, neg, self.g))
    }
}

// ── Mag sweep flow ───────────────────────────────────────────────────────────

/// Streaming magnetometer sweep with a FIT-QUALITY gate: a sweep that did not
/// actually cover the rotation (thin per-axis span, or wildly anisotropic
/// coverage) is REJECTED rather than fitted — storing a bad hard/soft-iron fit
/// corrupts every heading after it.
pub struct MagSweepFlow {
    lo: Vec3,
    hi: Vec3,
    n: u32,
    needed: u32,
}

/// The mag sweep verdict.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MagSweepVerdict {
    /// Coverage was sufficient; offset/scale are available.
    Accepted,
    /// Sweep rejected: the smallest per-axis half-range (gauss) and the
    /// min/max half-range ratio that failed the gate — operator retries.
    Rejected { min_half_range: f32, anisotropy: f32 },
}

impl MagSweepFlow {
    pub fn new(needed: u32) -> Self {
        MagSweepFlow {
            lo: [f32::INFINITY; 3],
            hi: [f32::NEG_INFINITY; 3],
            n: 0,
            needed: needed.max(1),
        }
    }

    /// Feed one mag sample (gauss). Non-finite samples are ignored (they
    /// cannot widen the envelope).
    pub fn step(&mut self, mag: Vec3) -> WindowStatus {
        if mag.iter().all(|v| v.is_finite()) {
            for (a, &m) in mag.iter().enumerate() {
                if m < self.lo[a] {
                    self.lo[a] = m;
                }
                if m > self.hi[a] {
                    self.hi[a] = m;
                }
            }
            self.n += 1;
        }
        if self.n >= self.needed {
            WindowStatus::Done
        } else {
            WindowStatus::Collecting { remaining: self.needed - self.n }
        }
    }

    /// Judge the sweep and, if coverage passes, return (hard-iron offset,
    /// soft-iron diagonal scale) — identical to [`crate::mag_hardiron`] /
    /// [`crate::mag_softiron_diag`] over the same samples (equivalence-tested).
    /// `min_span` is the minimum acceptable per-axis half-range (gauss);
    /// `max_anisotropy` the maximum allowed max/min half-range ratio.
    pub fn finish(
        &self,
        min_span: f32,
        max_anisotropy: f32,
    ) -> (MagSweepVerdict, Option<(Vec3, Vec3)>) {
        if self.n < self.needed {
            return (MagSweepVerdict::Rejected { min_half_range: 0.0, anisotropy: f32::INFINITY }, None);
        }
        let half = [
            (self.hi[0] - self.lo[0]) * 0.5,
            (self.hi[1] - self.lo[1]) * 0.5,
            (self.hi[2] - self.lo[2]) * 0.5,
        ];
        let mut min_h = half[0];
        let mut max_h = half[0];
        for &h in &half[1..] {
            if h < min_h {
                min_h = h;
            }
            if h > max_h {
                max_h = h;
            }
        }
        let anisotropy = if min_h > 0.0 { max_h / min_h } else { f32::INFINITY };
        if !(min_h.is_finite() && min_h >= min_span && anisotropy <= max_anisotropy) {
            return (MagSweepVerdict::Rejected { min_half_range: min_h, anisotropy }, None);
        }
        let offset = [
            (self.lo[0] + self.hi[0]) * 0.5,
            (self.lo[1] + self.hi[1]) * 0.5,
            (self.lo[2] + self.hi[2]) * 0.5,
        ];
        let avg = (half[0] + half[1] + half[2]) / 3.0;
        let mut scale = [1.0f32; 3];
        for a in 0..3 {
            scale[a] = if half[a] != 0.0 { avg / half[a] } else { 1.0 };
        }
        (MagSweepVerdict::Accepted, Some((offset, scale)))
    }
}

// ── Persistence round-trip ───────────────────────────────────────────────────

impl CalParams {
    /// Rebuild from the 15 values in [`CalParams::to_named`] ORDER — the
    /// param-store load path (PARAM-P03). `to_named` → `from_values` is a
    /// bit-exact round trip (tested).
    pub fn from_values(v: [f32; 15]) -> Self {
        CalParams {
            gyro_bias: [v[0], v[1], v[2]],
            accel_bias: [v[3], v[4], v[5]],
            accel_scale: [v[6], v[7], v[8]],
            mag_offset: [v[9], v[10], v[11]],
            mag_scale: [v[12], v[13], v[14]],
        }
    }
}
