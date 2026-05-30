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
// Index loops are clearer than iterator adapters for fixed-size matrix
// algebra; the bounds are all compile-time constants.
#![allow(clippy::needless_range_loop)]

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

/// Body→NED rotation matrix from the unit quaternion (`R·v_body = v_ned`).
/// Used to build the IEKF error-propagation Jacobian.
#[inline]
fn quat_to_rotmat(q: Quat) -> [[f32; 3]; 3] {
    let (w, x, y, z) = (q[0], q[1], q[2], q[3]);
    [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - w * z), 2.0 * (x * z + w * y)],
        [2.0 * (x * y + w * z), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - w * x)],
        [2.0 * (x * z - w * y), 2.0 * (y * z + w * x), 1.0 - 2.0 * (x * x + y * y)],
    ]
}

/// Skew-symmetric matrix `[v]×` (so `[v]× a = v × a`).
#[inline]
fn skew(v: Vec3) -> [[f32; 3]; 3] {
    [[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]]
}

// ─────────────────────────── 15×15 linear algebra ─────────────────────
//
// Error-state ordering: [δθ(0..3), δv(3..6), δp(6..9), δb_g(9..12),
// δb_a(12..15)]. Dense f32, no alloc — small enough that O(N³) matmul is
// cheap at IMU rate.

/// Error-state dimension.
pub const N: usize = 15;
/// 15×15 dense matrix.
pub type Mat = [[f32; N]; N];

fn mat_zero() -> Mat {
    [[0.0; N]; N]
}

fn mat_identity() -> Mat {
    let mut m = mat_zero();
    for i in 0..N {
        m[i][i] = 1.0;
    }
    m
}

/// `a · b`.
fn mat_mul(a: &Mat, b: &Mat) -> Mat {
    let mut out = mat_zero();
    for i in 0..N {
        for k in 0..N {
            let aik = a[i][k];
            if aik == 0.0 {
                continue;
            }
            for j in 0..N {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    out
}

/// `aᵀ`.
fn mat_transpose(a: &Mat) -> Mat {
    let mut out = mat_zero();
    for i in 0..N {
        for j in 0..N {
            out[i][j] = a[j][i];
        }
    }
    out
}

/// Force exact symmetry `P ← ½(P + Pᵀ)` — cheap guard against f32 drift
/// accumulating asymmetry over many propagations.
fn symmetrise(p: &mut Mat) {
    for i in 0..N {
        for j in (i + 1)..N {
            let avg = 0.5 * (p[i][j] + p[j][i]);
            p[i][j] = avg;
            p[j][i] = avg;
        }
    }
}

/// Write a (scaled) 3×3 block into `m` at top-left `(r0, c0)`.
fn set_block3(m: &mut Mat, r0: usize, c0: usize, b: &[[f32; 3]; 3], scale: f32) {
    for i in 0..3 {
        for j in 0..3 {
            m[r0 + i][c0 + j] = scale * b[i][j];
        }
    }
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

/// Noise tuning (continuous-time spectral densities; added to P as
/// `Q·dt`). Defaults are order-of-magnitude reasonable for a consumer
/// MEMS IMU; the gz NEES gate (step 5) tunes them.
#[derive(Clone, Copy)]
pub struct IekfConfig {
    /// Gyro white-noise variance (rad²/s²) → attitude process noise.
    pub q_gyro: f32,
    /// Accel white-noise variance (m²/s⁴) → velocity process noise.
    pub q_accel: f32,
    /// Gyro-bias random-walk variance (rad²/s⁴).
    pub q_bias_gyro: f32,
    /// Accel-bias random-walk variance (m²/s⁶).
    pub q_bias_accel: f32,
    /// Initial 1σ for attitude (rad), velocity (m/s), position (m),
    /// gyro bias (rad/s), accel bias (m/s²).
    pub p0: [f32; 5],
}

impl IekfConfig {
    pub const DEFAULT: Self = Self {
        q_gyro: 1e-4,
        q_accel: 1e-2,
        q_bias_gyro: 1e-7,
        q_bias_accel: 1e-5,
        p0: [0.1, 0.5, 1.0, 0.01, 0.1],
    };
}

impl Default for IekfConfig {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// The Invariant EKF: nominal state + 15×15 error covariance.
pub struct Iekf {
    state: NavState,
    /// Error-state covariance (ordering [δθ, δv, δp, δb_g, δb_a]).
    p: Mat,
    cfg: IekfConfig,
}

impl Iekf {
    pub fn new(state: NavState) -> Self {
        Self::with_config(state, IekfConfig::DEFAULT)
    }

    pub fn with_config(state: NavState, cfg: IekfConfig) -> Self {
        let mut p = mat_zero();
        let v = [
            cfg.p0[0] * cfg.p0[0], cfg.p0[1] * cfg.p0[1], cfg.p0[2] * cfg.p0[2],
            cfg.p0[3] * cfg.p0[3], cfg.p0[4] * cfg.p0[4],
        ];
        for blk in 0..5 {
            for i in 0..3 {
                p[blk * 3 + i][blk * 3 + i] = v[blk];
            }
        }
        Iekf { state, p, cfg }
    }

    pub fn level() -> Self {
        Self::new(NavState::identity())
    }

    pub fn state(&self) -> NavState {
        self.state
    }

    /// The current 15×15 error covariance.
    pub fn covariance(&self) -> &Mat {
        &self.p
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

        // Rotation matrix BEFORE the attitude update — the Jacobian
        // linearises about the current estimate.
        let r_hat = quat_to_rotmat(s.q);

        // Attitude: right-multiply by the body-frame incremental rotation.
        s.q = q_normalize(q_mul(s.q, so3_exp([omega[0] * dt, omega[1] * dt, omega[2] * dt])));

        // Position uses the pre-update velocity (semi-implicit is a later
        // refinement); velocity then integrates the inertial accel.
        for i in 0..3 {
            s.p[i] += s.v[i] * dt + 0.5 * a_ned[i] * dt * dt;
            s.v[i] += a_ned[i] * dt;
        }
        s.q = sanitise_quat(s.q);

        // ── Covariance propagation: P⁺ = Φ P Φᵀ + Q·dt, Φ = I + A·dt ──
        //
        // Right-invariant (world-frame) error dynamics ξ̇ = A ξ (design
        // doc). The pose block is group-affine (state-independent): only
        // [g]× and I; the bias columns carry −R̂ (this is the standard
        // bias coupling; the −v̂×R̂, −p̂×R̂ "imperfect-IEKF" refinements
        // are deferred — they are higher-order and the NEES gate will say
        // if they are needed).
        //
        //        δθ    δv    δp    δb_g   δb_a
        //  δθ̇  [ 0     0     0    −R̂      0   ]
        //  δv̇  [ [g]×  0     0     0      −R̂  ]
        //  δṗ  [ 0     I     0     0       0   ]
        let mut phi = mat_identity();
        set_block3(&mut phi, 3, 0, &skew(GRAVITY_NED), dt); // [g]× · dt
        {
            let mut i3 = [[0.0; 3]; 3];
            for i in 0..3 {
                i3[i][i] = 1.0;
            }
            set_block3(&mut phi, 6, 3, &i3, dt); // I · dt  (ṗ = v)
        }
        // bias columns: −R̂ · dt
        let neg_r = {
            let mut m = [[0.0; 3]; 3];
            for i in 0..3 {
                for j in 0..3 {
                    m[i][j] = -r_hat[i][j];
                }
            }
            m
        };
        set_block3(&mut phi, 0, 9, &neg_r, dt); // δθ ← −R̂ δb_g
        set_block3(&mut phi, 3, 12, &neg_r, dt); // δv ← −R̂ δb_a

        let phi_t = mat_transpose(&phi);
        let pp = mat_mul(&phi, &self.p);
        self.p = mat_mul(&pp, &phi_t);

        // Process noise Q·dt on the diagonal blocks.
        let qd = [
            self.cfg.q_gyro * dt, self.cfg.q_accel * dt, 0.0,
            self.cfg.q_bias_gyro * dt, self.cfg.q_bias_accel * dt,
        ];
        for blk in 0..5 {
            for i in 0..3 {
                self.p[blk * 3 + i][blk * 3 + i] += qd[blk];
            }
        }
        symmetrise(&mut self.p);
    }

    /// Right-invariant NED **position** measurement update (e.g. GPS, or
    /// the gz NavSat→NED position). `meas_var` is the per-axis variance.
    ///
    /// **Direct** NED position Jacobian `H = [0, 0, I, 0, 0]`, innovation
    /// `r = z − p̂`. Attitude is corrected through the covariance
    /// cross-terms the (right-invariant, group-affine) propagation builds
    /// — NOT through a direct `−[p̂]×` term, which is the pure-invariant
    /// form but blows up for absolute position at altitude. This is the
    /// standard, robust choice for IMU/GPS invariant filters; the
    /// invariance lives in the propagation, and the NEES gate validates
    /// the resulting consistency. Standard Kalman update with the SE₂(3)
    /// left-multiply attitude injection. False (no-op) if S is singular.
    pub fn update_position(&mut self, z_ned: Vec3, meas_var: f32) -> bool {
        let p_hat = self.state.p;
        let z = sanitise3(z_ned);
        let r = [z[0] - p_hat[0], z[1] - p_hat[1], z[2] - p_hat[2]];

        // H (3×15): cols 6..9 = I (direct position observation).
        let mut h = [[0.0f32; N]; 3];
        for i in 0..3 {
            h[i][6 + i] = 1.0;
        }

        // HP = H·P (3×15).
        let mut hp = [[0.0f32; N]; 3];
        for i in 0..3 {
            for k in 0..N {
                let hik = h[i][k];
                if hik == 0.0 {
                    continue;
                }
                for j in 0..N {
                    hp[i][j] += hik * self.p[k][j];
                }
            }
        }

        // S = HP·Hᵀ + R (3×3).
        let mut s = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                let mut acc = 0.0;
                for k in 0..N {
                    acc += hp[i][k] * h[j][k];
                }
                s[i][j] = acc;
            }
            s[i][i] += meas_var;
        }
        let s_inv = match inv3(&s) {
            Some(v) => v,
            None => return false,
        };

        // PHt = P·Hᵀ (15×3); since P is symmetric, PHt[k][i] = HP[i][k].
        // K = PHt·S⁻¹ (15×3).
        let mut k_gain = [[0.0f32; 3]; N];
        for row in 0..N {
            for col in 0..3 {
                let mut acc = 0.0;
                for m in 0..3 {
                    acc += hp[m][row] * s_inv[m][col];
                }
                k_gain[row][col] = acc;
            }
        }

        // ξ = K·r (15).
        let mut xi = [0.0f32; N];
        for row in 0..N {
            let mut acc = 0.0;
            for c in 0..3 {
                acc += k_gain[row][c] * r[c];
            }
            xi[row] = acc;
        }

        // Inject (right-invariant): R̂ ← Exp(ξθ)·R̂ (LEFT multiply, world
        // frame); v, p, biases first-order additive.
        let dtheta = [xi[0], xi[1], xi[2]];
        self.state.q = sanitise_quat(q_mul(so3_exp(dtheta), self.state.q));
        for i in 0..3 {
            self.state.v[i] += xi[3 + i];
            self.state.p[i] += xi[6 + i];
            self.state.b_g[i] += xi[9 + i];
            self.state.b_a[i] += xi[12 + i];
        }

        // P ← (I − K H) P.
        let mut kh = mat_zero();
        for i in 0..N {
            for c in 0..3 {
                let kic = k_gain[i][c];
                if kic == 0.0 {
                    continue;
                }
                for j in 0..N {
                    kh[i][j] += kic * h[c][j];
                }
            }
        }
        let mut imkh = mat_identity();
        for i in 0..N {
            for j in 0..N {
                imkh[i][j] -= kh[i][j];
            }
        }
        self.p = mat_mul(&imkh, &self.p);
        symmetrise(&mut self.p);
        true
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

/// Closed-form inverse of a 3×3 matrix; `None` if near-singular.
fn inv3(m: &[[f32; 3]; 3]) -> Option<[[f32; 3]; 3]> {
    let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
    if !det.is_finite() || det.abs() < 1e-12 {
        return None;
    }
    let inv_det = 1.0 / det;
    let mut out = [[0.0f32; 3]; 3];
    out[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
    out[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
    out[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
    out[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
    out[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
    out[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
    out[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
    out[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
    out[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;
    if out.iter().flatten().all(|c| c.is_finite()) {
        Some(out)
    } else {
        None
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

    /// A position measurement pulls the estimate toward it and SHRINKS the
    /// corresponding covariance — the basic Kalman-update sanity.
    #[test]
    fn position_update_pulls_toward_measurement() {
        let mut f = Iekf::level();
        let imu = Imu { gyro: [0.0; 3], accel: [0.0, 0.0, -9.81] };
        for _ in 0..50 {
            f.propagate(imu, 0.01);
        }
        let var_before = f.covariance()[6][6]; // north-position variance
        assert!(f.update_position([5.0, 0.0, 0.0], 0.01));
        let s = f.state();
        assert!(s.p[0] > 0.5, "estimate should move north: {:?}", s.p);
        assert!(f.covariance()[6][6] < var_before, "north-pos variance should shrink");
    }

    /// Fed the gravity-reaction accel + repeated (noiseless) position
    /// measurements at a fixed true point, the estimate converges to it —
    /// IMU prediction + position correction together localise the body.
    #[test]
    fn converges_to_true_static_position() {
        let mut f = Iekf::level();
        let imu = Imu { gyro: [0.0; 3], accel: [0.0, 0.0, -9.81] };
        let truth = [3.0, -2.0, -10.0];
        for _ in 0..800 {
            f.propagate(imu, 0.01);
            f.update_position(truth, 0.05);
        }
        let s = f.state();
        for i in 0..3 {
            assert!((s.p[i] - truth[i]).abs() < 0.3, "p[{i}] = {} vs {}", s.p[i], truth[i]);
        }
        assert!(s.tilt_rad().to_degrees() < 5.0, "stays roughly level: {}", s.tilt_rad().to_degrees());
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
                // Covariance stays a valid covariance: finite, exactly
                // symmetric, non-negative diagonal (necessary for PSD).
                let p = f.covariance();
                for i in 0..N {
                    proptest::prop_assert!(p[i][i] >= 0.0, "neg diag {} = {}", i, p[i][i]);
                    for j in 0..N {
                        proptest::prop_assert!(p[i][j].is_finite());
                        proptest::prop_assert!((p[i][j] - p[j][i]).abs() <= 1e-3 * (1.0 + p[i][j].abs()));
                    }
                }
            }
        }
    }

    /// Without measurements, uncertainty must GROW: the covariance trace
    /// after propagation exceeds the initial trace (process noise + the
    /// gravity/velocity coupling injecting attitude uncertainty into
    /// position). A filter whose covariance shrank with no data would be
    /// overconfident — the classic inconsistency the IEKF avoids.
    #[test]
    fn covariance_grows_without_measurements() {
        let mut f = Iekf::level();
        let tr0: f32 = (0..N).map(|i| f.covariance()[i][i]).sum();
        let imu = Imu { gyro: [0.0; 3], accel: [0.0, 0.0, -9.81] };
        for _ in 0..200 {
            f.propagate(imu, 0.01);
        }
        let tr1: f32 = (0..N).map(|i| f.covariance()[i][i]).sum();
        assert!(tr1 > tr0, "covariance trace must grow: {tr0} -> {tr1}");
        assert!(tr1.is_finite());
    }
}
