//! Relay navigation filter — full-state **Invariant EKF** on SE₂(3).
//!
//! Replaces the attitude-only Mahony complementary filter (`relay-ekf`).
//! Estimates the extended pose (rotation R, NED velocity v, NED position
//! p) plus IMU biases (gyro, accel). See `docs/IEKF-DESIGN.md` for the
//! full math.
//!
//! ## Why this fixes RC#3
//!
//! The Mahony filter trusts the accelerometer as an instantaneous gravity
//! reference — wrong whenever the body accelerates (v0.20 measured the
//! estimate diverging to 51° while the body was level). The IEKF never
//! does that: it **predicts** attitude from the gyro through the
//! rigid-body dynamics `v̇ = R·a + g`, with the accelerometer as a
//! dynamics *input*, and corrects with GPS/baro position. On SE₂(3) the
//! estimation error is *group-affine* (state-independent propagation),
//! which is what gives provable consistency.
//!
//! ## This file = build-order step 1 (foundation)
//!
//! Nominal state + SO(3) helpers + IMU propagation, with the geometric
//! invariants proptest-gated. The 15×15 covariance, the group-affine Φ
//! propagation, and the right-invariant position/baro updates land next
//! (steps 2–5 in the design doc). The nominal propagation here is shared
//! verbatim by the full filter — it is the same for the IEKF and any
//! error-state EKF; only the error/covariance treatment differs.

#![no_std]

/// Standard gravity in NED (down is +z), m/s².
pub const GRAVITY_NED: [f32; 3] = [0.0, 0.0, 9.81];

// ───────────────────────── SO(3) / quaternion helpers ─────────────────
//
// Hamilton convention, scalar-first `[w, x, y, z]`, unit quaternion
// representing the body→NED rotation. All total (NaN-safe) so the
// proptest/Kani totality gates hold.

/// Quaternion (Hamilton, scalar-first).
pub type Quat = [f32; 4];
/// 3-vector.
pub type Vec3 = [f32; 3];

#[inline]
fn q_mul(a: Quat, b: Quat) -> Quat {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Renormalise to unit length; revert to identity if degenerate
/// (NaN/∞/zero) — keeps `‖q‖ = 1` total.
#[inline]
fn q_normalize(q: Quat) -> Quat {
    let n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if n2.is_finite() && n2 > 1e-12 {
        let inv = 1.0 / libm::sqrtf(n2);
        let r = [q[0] * inv, q[1] * inv, q[2] * inv, q[3] * inv];
        if r.iter().all(|c| c.is_finite()) {
            return r;
        }
    }
    [1.0, 0.0, 0.0, 0.0]
}

/// SO(3) exponential of a rotation vector `φ` (axis·angle, rad) → unit
/// quaternion. Small-angle-safe via the sinc series near `‖φ‖ → 0`.
#[inline]
fn so3_exp(phi: Vec3) -> Quat {
    let t2 = phi[0] * phi[0] + phi[1] * phi[1] + phi[2] * phi[2];
    if !t2.is_finite() {
        return [1.0, 0.0, 0.0, 0.0];
    }
    let theta = libm::sqrtf(t2);
    // half-angle; sinc(θ/2)/2 with a Taylor fallback for small θ.
    let (w, k) = if theta < 1e-6 {
        // cos(θ/2) ≈ 1 − θ²/8 ; (sin(θ/2)/θ) ≈ ½ − θ²/48
        (1.0 - t2 / 8.0, 0.5 - t2 / 48.0)
    } else {
        let h = 0.5 * theta;
        (libm::cosf(h), libm::sinf(h) / theta)
    };
    q_normalize([w, k * phi[0], k * phi[1], k * phi[2]])
}

/// Rotate a vector from body into NED by the quaternion (R·v).
#[inline]
fn q_rotate(q: Quat, v: Vec3) -> Vec3 {
    // v + 2 w (u×v) + 2 u×(u×v),  u = q.xyz
    let u = [q[1], q[2], q[3]];
    let uv = cross(u, v);
    let uuv = cross(u, uv);
    [
        v[0] + 2.0 * (q[0] * uv[0] + uuv[0]),
        v[1] + 2.0 * (q[0] * uv[1] + uuv[1]),
        v[2] + 2.0 * (q[0] * uv[2] + uuv[2]),
    ]
}

#[inline]
fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

#[inline]
fn sanitise3(v: Vec3) -> Vec3 {
    [
        if v[0].is_finite() { v[0] } else { 0.0 },
        if v[1].is_finite() { v[1] } else { 0.0 },
        if v[2].is_finite() { v[2] } else { 0.0 },
    ]
}

// ───────────────────────────── Nominal state ──────────────────────────

/// The IEKF nominal state: extended pose (R, v, p) + IMU biases.
#[derive(Clone, Copy, Debug)]
pub struct NavState {
    /// Body→NED rotation (unit quaternion, Hamilton scalar-first).
    pub q: Quat,
    /// NED velocity (m/s).
    pub v: Vec3,
    /// NED position (m).
    pub p: Vec3,
    /// Gyro bias (rad/s).
    pub b_g: Vec3,
    /// Accelerometer bias (m/s²).
    pub b_a: Vec3,
}

impl NavState {
    /// Level, at rest, at the origin, zero biases.
    pub const fn identity() -> Self {
        NavState {
            q: [1.0, 0.0, 0.0, 0.0],
            v: [0.0; 3],
            p: [0.0; 3],
            b_g: [0.0; 3],
            b_a: [0.0; 3],
        }
    }

    /// Body tilt from vertical (rad) implied by the estimate — the
    /// quantity v0.20 showed the Mahony filter getting wrong by 50°.
    /// `R[2][2]` is the NED-down component of the body-down axis.
    pub fn tilt_rad(&self) -> f32 {
        let q = self.q;
        let r22 = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]);
        libm::acosf(r22.clamp(-1.0, 1.0))
    }
}

/// One IMU sample (body frame).
#[derive(Clone, Copy, Debug)]
pub struct Imu {
    /// Body angular rate (rad/s).
    pub gyro: Vec3,
    /// Body specific force / accelerometer (m/s²).
    pub accel: Vec3,
}

/// The filter. (Step 1: nominal state only; covariance lands in step 2.)
pub struct Iekf {
    state: NavState,
}

impl Iekf {
    pub fn new(state: NavState) -> Self {
        Iekf { state }
    }

    pub fn level() -> Self {
        Iekf { state: NavState::identity() }
    }

    pub fn state(&self) -> NavState {
        self.state
    }

    /// IMU propagation over `dt` (clamped to a sane range). Implements
    /// the nominal dynamics from the design doc:
    /// ```text
    ///   ω = ω_m − b_g ;  a = a_m − b_a
    ///   R⁺ = R · Exp(ω dt)
    ///   v⁺ = v + (R·a + g) dt
    ///   p⁺ = p + v dt + ½ (R·a + g) dt²
    /// ```
    pub fn propagate(&mut self, imu: Imu, dt: f32) {
        let dt = if dt.is_finite() { dt.clamp(1e-4, 0.1) } else { 1e-3 };
        let s = &mut self.state;

        let gyro = sanitise3(imu.gyro);
        let accel = sanitise3(imu.accel);
        let omega = [gyro[0] - s.b_g[0], gyro[1] - s.b_g[1], gyro[2] - s.b_g[2]];
        let acc_b = [accel[0] - s.b_a[0], accel[1] - s.b_a[1], accel[2] - s.b_a[2]];

        // Specific force rotated into NED, plus gravity → inertial accel.
        let acc_n_body = q_rotate(s.q, acc_b);
        let a_ned = [
            acc_n_body[0] + GRAVITY_NED[0],
            acc_n_body[1] + GRAVITY_NED[1],
            acc_n_body[2] + GRAVITY_NED[2],
        ];

        // Attitude: right-multiply by the body-frame incremental rotation.
        s.q = q_normalize(q_mul(s.q, so3_exp([omega[0] * dt, omega[1] * dt, omega[2] * dt])));

        // Position uses the pre-update velocity (semi-implicit is a step-2
        // refinement); velocity then integrates the inertial accel.
        for i in 0..3 {
            s.p[i] += s.v[i] * dt + 0.5 * a_ned[i] * dt * dt;
            s.v[i] += a_ned[i] * dt;
        }
        s.q = sanitise_quat(s.q);
    }
}

#[inline]
fn sanitise_quat(q: Quat) -> Quat {
    if q.iter().all(|c| c.is_finite()) {
        q_normalize(q)
    } else {
        [1.0, 0.0, 0.0, 0.0]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn qnorm(q: Quat) -> f32 {
        libm::sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3])
    }

    /// At rest and level, the specific force a quad's accelerometer reads
    /// is the +z (up) reaction to gravity: in NED body-down terms the
    /// measured accel is `[0,0,−g]`, so `R·a + g = 0` and the state holds
    /// still. (The estimate must NOT drift — the v0.20 RC#3 failure.)
    #[test]
    fn level_at_rest_holds_still() {
        let mut f = Iekf::level();
        let imu = Imu { gyro: [0.0; 3], accel: [0.0, 0.0, -9.81] };
        for _ in 0..1000 {
            f.propagate(imu, 0.01);
        }
        let s = f.state();
        assert!(s.tilt_rad().to_degrees() < 0.01, "tilt {}", s.tilt_rad());
        assert!(s.v.iter().all(|c| c.abs() < 1e-3), "v drifted {:?}", s.v);
        assert!((qnorm(s.q) - 1.0).abs() < 1e-5);
    }

    /// A pure yaw rate integrates to a yaw rotation with no tilt — the
    /// attitude is *predicted*, not inferred from gravity.
    #[test]
    fn pure_yaw_rate_integrates_to_yaw_no_tilt() {
        let mut f = Iekf::level();
        // 1 rad/s yaw for 1 s → ~57.3° yaw, zero tilt.
        let imu = Imu { gyro: [0.0, 0.0, 1.0], accel: [0.0, 0.0, -9.81] };
        for _ in 0..100 {
            f.propagate(imu, 0.01);
        }
        let s = f.state();
        assert!(s.tilt_rad().to_degrees() < 0.5, "yaw should not tilt: {}", s.tilt_rad());
        // yaw ≈ atan2(2(wz+xy), 1−2(y²+z²)) ≈ 1 rad
        let q = s.q;
        let yaw = libm::atan2f(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        assert!((yaw - 1.0).abs() < 0.05, "yaw {yaw}");
    }

    /// Constant forward specific force tilts the body and accelerates it
    /// north — the accelerometer is a dynamics INPUT, the hallmark of the
    /// IEKF (vs Mahony treating it as a gravity reference).
    #[test]
    fn forward_accel_moves_north() {
        let mut f = Iekf::level();
        // Body level; accel reads gravity reaction (−g down) + 1 m/s² north.
        let imu = Imu { gyro: [0.0; 3], accel: [1.0, 0.0, -9.81] };
        for _ in 0..100 {
            f.propagate(imu, 0.01);
        }
        let s = f.state();
        assert!(s.v[0] > 0.5, "should gain north velocity, got {:?}", s.v);
        assert!(s.p[0] > 0.0, "should move north, got {:?}", s.p);
    }

    proptest::proptest! {
        /// Invariant: ‖q‖ = 1 and the whole state stays finite under any
        /// finite IMU stream and any (even degenerate) dt.
        #[test]
        fn iekf_invariants_hold(
            samples in proptest::collection::vec(
                (-20.0_f32..20.0, -20.0_f32..20.0, -20.0_f32..20.0,
                 -50.0_f32..50.0, -50.0_f32..50.0, -50.0_f32..50.0,
                 0.0_f32..0.2),
                0..300),
        ) {
            let mut f = Iekf::level();
            for (gx, gy, gz, ax, ay, az, dt) in samples {
                f.propagate(Imu { gyro: [gx, gy, gz], accel: [ax, ay, az] }, dt);
                let s = f.state();
                proptest::prop_assert!((qnorm(s.q) - 1.0).abs() < 1e-3);
                proptest::prop_assert!(s.q.iter().all(|c| c.is_finite()));
                proptest::prop_assert!(s.v.iter().all(|c| c.is_finite()));
                proptest::prop_assert!(s.p.iter().all(|c| c.is_finite()));
            }
        }
    }
}
