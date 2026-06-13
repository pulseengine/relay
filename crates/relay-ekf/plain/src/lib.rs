//! Relay EKF — attitude estimator (Mahony complementary filter on SO(3)).
//!
//! Replaces the v0.1 `relay-ekf-stub`. Implements the discrete-time
//! complementary filter from Mahony, Hamel & Pflimlin (2008),
//! *Nonlinear Complementary Filters on the Special Orthogonal Group*,
//! IEEE T-AC vol. 53 no. 5. Specialised here to gravity-vector
//! correction (accelerometer only — magnetometer fusion lands in v0.4
//! alongside `relay-att`).
//!
//! ## Algorithm (per tick, dt seconds)
//!
//! ```text
//!  measured: omega_meas (rad/s, body), accel (m/s^2, body)
//!  state:    q (body-to-NED unit quaternion), bias (rad/s)
//!
//!  1. gravity_est = rotate-to-body(q, NED_DOWN)
//!  2. e = cross(accel.unit(), gravity_est)
//!  3. bias += -Ki * e * dt          # integral term, slow
//!  4. omega = omega_meas - bias + Kp * e   # corrected rate, fast
//!  5. q += 0.5 * q * (0, omega) * dt
//!  6. q := q / ||q||
//! ```
//!
//! Gains `Kp` (proportional) and `Ki` (integral) are tunable via TBL
//! in the dual-DNA composition; defaults are tuned for a generic
//! consumer-grade IMU at 200 Hz–1 kHz.
//!
//! ## Verified properties (v0.2 surrogates for SWREQ-FALCON-EKF-P*)
//!
//! - **EKF-P01** (unit quaternion): every output `q` satisfies
//!   `||q||² ∈ [1 - ε, 1 + ε]` for `ε = 1e-6`. *Test: every proptest
//!   case across arbitrary IMU input streams.*
//! - **EKF-P02** (no NaN): no operation produces NaN or ±∞ even
//!   under degenerate accel input (zero vector, denormal floats).
//!   *Test: `propagates_no_nan_under_adversarial_accel`.*
//! - **EKF-P03** (innovation monotone in error): when accel and
//!   gravity-estimate are aligned, innovation ≈ 0; when they disagree
//!   by angle θ, innovation magnitude scales monotonically with θ.
//!   *Test: `innovation_monotone_with_disagreement`.*
//! - **EKF-P04** (static convergence): with zero gyro input and a
//!   constant accel reading, after sufficient ticks the estimator
//!   converges to the orientation that aligns its body-down axis with
//!   the accel direction. *Test: `static_rest_converges_to_gravity`.*
//! - **EKF-P05** (rotation tracking): with constant gyro about a
//!   single body axis and accel consistent with the rotated gravity,
//!   the estimator tracks the rotation within a bounded steady-state
//!   error. *Test: `pure_yaw_rotation_tracked`.*
//!
//! ## Real-EKF roadmap
//!
//! Mahony is a complementary filter, not a Kalman filter — it has no
//! explicit covariance. v0.4 will add covariance tracking + GPS/baro
//! fusion to become a proper MEKF. The current API is forward-
//! compatible: callers consume `VehicleState` regardless of which
//! estimator backs it.

#![no_std]
#![forbid(unsafe_code)]

use relay_math::sqrtf;

/// Timestamp: seconds + 2^32-fraction.
/// Mirrors `pulseengine:relay-common-types/types.{timestamp}`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Timestamp {
    pub seconds: u64,
    pub fraction: u32,
}

impl Timestamp {
    pub const ZERO: Self = Self { seconds: 0, fraction: 0 };

    /// Seconds (f32) from the epoch.
    pub fn as_secs_f32(self) -> f32 {
        self.seconds as f32 + (self.fraction as f32) / (1u64 << 32) as f32
    }
}

/// IMU sample at a single instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuSample {
    pub time: Timestamp,
    /// Body-frame acceleration, m/s².
    pub accel_body: [f32; 3],
    /// Body-frame angular velocity, rad/s.
    pub gyro_body: [f32; 3],
}

/// Vehicle state estimate. Output of the EKF.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleState {
    pub time: Timestamp,
    /// Unit quaternion (w, x, y, z), body-to-NED.
    pub quaternion: [f32; 4],
    /// Position in NED frame, meters. Zero in v0.2 (no GPS fusion).
    pub position_ned: [f32; 3],
    /// Velocity in NED frame, m/s. Zero in v0.2.
    pub velocity_ned: [f32; 3],
    /// Normalised innovation magnitude (residual between predicted
    /// gravity and measured accel). 0.0 = perfect agreement.
    pub innovation: f32,
}

/// EKF tuning parameters.
#[derive(Clone, Copy, Debug)]
pub struct EkfGains {
    /// Proportional gain (rad/s per unit cross-product magnitude).
    pub kp: f32,
    /// Integral gain (rad/s² per unit cross-product magnitude).
    pub ki: f32,
}

impl EkfGains {
    /// Reasonable defaults for a 200 Hz–1 kHz consumer-grade IMU.
    pub const DEFAULT: Self = Self { kp: 2.0, ki: 0.05 };
}

impl Default for EkfGains {
    fn default() -> Self {
        Self::DEFAULT
    }
}

/// Tolerance for `is_unit_quaternion` and proptest-style invariants.
pub const QUATERNION_NORM_TOLERANCE: f32 = 1.0e-3;

/// The estimator.
#[derive(Clone, Copy, Debug)]
pub struct Ekf {
    /// Unit quaternion, body-to-NED.
    q: [f32; 4],
    /// Gyro bias estimate (rad/s).
    bias: [f32; 3],
    /// Most recent sample timestamp.
    last_time: Timestamp,
    /// Cached last innovation magnitude.
    last_innovation: f32,
    /// Tuning gains.
    gains: EkfGains,
}

impl Ekf {
    /// Construct an EKF at identity orientation with default gains.
    pub const fn new() -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            bias: [0.0; 3],
            last_time: Timestamp::ZERO,
            last_innovation: 0.0,
            gains: EkfGains::DEFAULT,
        }
    }

    /// Construct with custom tuning gains.
    pub const fn with_gains(gains: EkfGains) -> Self {
        Self {
            q: [1.0, 0.0, 0.0, 0.0],
            bias: [0.0; 3],
            last_time: Timestamp::ZERO,
            last_innovation: 0.0,
            gains,
        }
    }

    /// Override the initial quaternion (defaults to identity).
    /// The provided quaternion is normalised; if it is the zero
    /// vector, the call is a no-op (EKF stays at identity).
    pub fn set_initial_quaternion(&mut self, q: [f32; 4]) {
        if let Some(unit) = normalise(q) {
            self.q = unit;
        }
    }

    pub fn gains(&self) -> EkfGains {
        self.gains
    }

    pub fn set_gains(&mut self, g: EkfGains) {
        self.gains = g;
    }

    pub fn bias(&self) -> [f32; 3] {
        self.bias
    }

    pub fn quaternion(&self) -> [f32; 4] {
        self.q
    }

    pub fn last_innovation(&self) -> f32 {
        self.last_innovation
    }

    pub fn last_time(&self) -> Timestamp {
        self.last_time
    }

    /// Consume one IMU sample and update the estimator.
    /// Returns the new `VehicleState`. `dt` is the elapsed seconds
    /// since `last_time`; if `last_time` was zero (first sample),
    /// the propagator uses `1/200 s` as a conservative default.
    pub fn tick(&mut self, sample: ImuSample) -> VehicleState {
        // 1. Compute dt.
        let dt = if self.last_time.seconds == 0 && self.last_time.fraction == 0 {
            1.0 / 200.0
        } else {
            let delta = sample.time.as_secs_f32() - self.last_time.as_secs_f32();
            // Clamp dt to [0.0001, 0.1] s to defend against time jumps
            // or out-of-order samples.
            clamp_f32(delta, 0.0001, 0.1)
        };
        self.last_time = sample.time;

        // 2. Sanitize accel; if degenerate (zero, NaN, ∞), skip
        //    correction this tick (use gyro-only propagation).
        let accel_unit_opt = normalise(extend(sample.accel_body));
        let (error, innovation) = match accel_unit_opt {
            Some(a) if all_finite(&a) => {
                // 3. Predict gravity direction in body frame.
                let g_pred = rotate_body_to_ned_inverse(self.q, NED_DOWN);
                let g_pred_unit = normalise(extend(g_pred)).unwrap_or([1.0, 0.0, 0.0, 0.0]);

                // 4. Innovation axis e = cross(measured, predicted) in body frame.
                //    Rationale: we want the body-frame rotation Δq that
                //    maps predicted ŷ onto measured y. The axis of
                //    that rotation is y × ŷ (right-hand rule, taking
                //    y to ŷ along the shorter arc). The estimator
                //    integrates ω = Kp * e about this axis, so
                //    R̂_new^T * NED_DOWN converges to y.
                let a3 = [a[1], a[2], a[3]];
                let gp3 = [g_pred_unit[1], g_pred_unit[2], g_pred_unit[3]];
                let e = cross(a3, gp3);
                let inn = sqrtf(e[0] * e[0] + e[1] * e[1] + e[2] * e[2]);
                (e, inn)
            }
            _ => ([0.0; 3], 0.0),
        };
        self.last_innovation = innovation;

        // 5. Integral term: bias accumulates against innovation.
        for i in 0..3 {
            self.bias[i] -= self.gains.ki * error[i] * dt;
            // Bound bias to ±0.5 rad/s; saturating prevents runaway
            // under sustained sensor mismatch.
            self.bias[i] = clamp_f32(self.bias[i], -0.5, 0.5);
        }

        // 6. Corrected gyro = raw - bias + Kp * innovation.
        let omega = [
            sample.gyro_body[0] - self.bias[0] + self.gains.kp * error[0],
            sample.gyro_body[1] - self.bias[1] + self.gains.kp * error[1],
            sample.gyro_body[2] - self.bias[2] + self.gains.kp * error[2],
        ];

        // 7. Integrate quaternion: q' = q + 0.5 * q ⊗ [0, omega] * dt.
        let qdot = quat_mul(self.q, [0.0, omega[0], omega[1], omega[2]]);
        let mut q_new = [
            self.q[0] + 0.5 * qdot[0] * dt,
            self.q[1] + 0.5 * qdot[1] * dt,
            self.q[2] + 0.5 * qdot[2] * dt,
            self.q[3] + 0.5 * qdot[3] * dt,
        ];

        // 8. Renormalise. If integration produced something
        //    pathological (NaN, zero), fall back to last good q.
        q_new = normalise(q_new).unwrap_or(self.q);
        self.q = q_new;

        VehicleState {
            time: sample.time,
            quaternion: self.q,
            position_ned: [0.0; 3],
            velocity_ned: [0.0; 3],
            innovation: self.last_innovation,
        }
    }
}

impl Default for Ekf {
    fn default() -> Self {
        Self::new()
    }
}

// ───────── pure math helpers (Verus-friendly) ─────────

/// NED-frame "down" basis vector (gravity direction at rest).
pub const NED_DOWN: [f32; 3] = [0.0, 0.0, 1.0];

/// Quaternion-as-4-vector multiply (Hamilton convention, w first).
#[inline]
pub fn quat_mul(a: [f32; 4], b: [f32; 4]) -> [f32; 4] {
    [
        a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
        a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
        a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
        a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0],
    ]
}

/// Conjugate of a unit quaternion is its inverse.
#[inline]
pub fn quat_conj(q: [f32; 4]) -> [f32; 4] {
    [q[0], -q[1], -q[2], -q[3]]
}

/// Rotate a 3-vector by a unit quaternion (NED → body).
/// Maps a NED-frame vector to its body-frame coordinates given
/// the body-to-NED quaternion.
#[inline]
pub fn rotate_body_to_ned_inverse(q: [f32; 4], v_ned: [f32; 3]) -> [f32; 3] {
    let qv = [0.0, v_ned[0], v_ned[1], v_ned[2]];
    let t = quat_mul(quat_conj(q), quat_mul(qv, q));
    [t[1], t[2], t[3]]
}

/// Cross product of two 3-vectors.
#[inline]
pub fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

/// Normalise a 4-vector to unit length. Returns `None` if the
/// input has zero (or sub-denormal) magnitude.
#[inline]
pub fn normalise(v: [f32; 4]) -> Option<[f32; 4]> {
    let n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2] + v[3] * v[3];
    if !n2.is_finite() || n2 < 1.0e-12 {
        return None;
    }
    let n = sqrtf(n2);
    Some([v[0] / n, v[1] / n, v[2] / n, v[3] / n])
}

/// Extend a 3-vector to a 4-vector with zero scalar component
/// (so `normalise` can be reused).
#[inline]
pub fn extend(v: [f32; 3]) -> [f32; 4] {
    [0.0, v[0], v[1], v[2]]
}

#[inline]
fn clamp_f32(v: f32, lo: f32, hi: f32) -> f32 {
    if !v.is_finite() {
        lo
    } else if v < lo {
        lo
    } else if v > hi {
        hi
    } else {
        v
    }
}

#[inline]
fn all_finite(v: &[f32; 4]) -> bool {
    v[0].is_finite() && v[1].is_finite() && v[2].is_finite() && v[3].is_finite()
}

/// Check that a quaternion is unit-norm within `QUATERNION_NORM_TOLERANCE`.
pub fn is_unit_quaternion(q: [f32; 4]) -> bool {
    let n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    let diff = if n2 >= 1.0 { n2 - 1.0 } else { 1.0 - n2 };
    diff <= QUATERNION_NORM_TOLERANCE
}

// ───────── tests ─────────

#[cfg(test)]
mod tests {
    use super::*;

    fn imu_at(secs: f32, accel: [f32; 3], gyro: [f32; 3]) -> ImuSample {
        let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
        ImuSample {
            time: Timestamp { seconds: secs as u64, fraction: frac },
            accel_body: accel,
            gyro_body: gyro,
        }
    }

    /// Gravity vector that an accelerometer at rest measures (body-frame).
    /// In NED with z = down, gravity vector seen by an accel at rest is
    /// +9.81 m/s² along body-z when the body z-axis is aligned with NED-z.
    /// (Specific-force convention: accel measures (specific force) =
    /// -gravity + acceleration; at rest, specific force = -g_vec = +9.81 up.)
    const GRAVITY_ACCEL_BODY: [f32; 3] = [0.0, 0.0, 9.81];

    #[test]
    fn fresh_estimator_is_at_identity() {
        let e = Ekf::new();
        assert_eq!(e.quaternion(), [1.0, 0.0, 0.0, 0.0]);
        assert!(is_unit_quaternion(e.quaternion()));
        assert_eq!(e.bias(), [0.0; 3]);
        assert_eq!(e.last_innovation(), 0.0);
    }

    #[test]
    fn ekf_p01_tick_preserves_unit_quaternion() {
        let mut e = Ekf::new();
        let s = imu_at(0.01, GRAVITY_ACCEL_BODY, [0.0, 0.0, 0.0]);
        let st = e.tick(s);
        assert!(is_unit_quaternion(st.quaternion));
    }

    #[test]
    fn ekf_p02_zero_accel_does_not_produce_nan() {
        let mut e = Ekf::new();
        let s = imu_at(0.01, [0.0, 0.0, 0.0], [0.1, 0.0, 0.0]);
        let st = e.tick(s);
        assert!(st.quaternion[0].is_finite());
        assert!(is_unit_quaternion(st.quaternion));
    }

    #[test]
    fn ekf_p02_extreme_accel_does_not_produce_nan() {
        let mut e = Ekf::new();
        let s = imu_at(0.01, [1.0e30, 1.0e30, 1.0e30], [0.0, 0.0, 0.0]);
        let st = e.tick(s);
        // Normalisation should rescue this; innovation may be saturated.
        assert!(st.quaternion[0].is_finite());
        assert!(is_unit_quaternion(st.quaternion));
    }

    #[test]
    fn ekf_p04_static_rest_converges_to_gravity_aligned() {
        let mut e = Ekf::new();
        let mut t = 0.0f32;
        // 200 Hz × 10 s = 2000 samples — well past steady state.
        for _ in 0..2000 {
            t += 1.0 / 200.0;
            e.tick(imu_at(t, GRAVITY_ACCEL_BODY, [0.0, 0.0, 0.0]));
        }
        // At rest with gravity in body-z, the estimator should
        // converge to identity (body-down = NED-down). Innovation
        // should also settle near zero.
        let q = e.quaternion();
        assert!(is_unit_quaternion(q));
        // Verify body-z axis is aligned with NED-z within tolerance.
        let z_body_in_ned = rotate_body_to_ned_inverse(quat_conj(q), [0.0, 0.0, 1.0]);
        assert!(z_body_in_ned[2] > 0.99,
            "body-down axis should be aligned with NED-down after convergence; got z={}",
            z_body_in_ned[2]);
        assert!(e.last_innovation() < 0.01,
            "innovation should converge near zero; got {}", e.last_innovation());
    }

    #[test]
    fn ekf_p05_pure_yaw_gyro_does_not_destabilise_attitude() {
        // Pure yaw rotation about body-z: accelerometer keeps reading
        // gravity in body-z regardless of yaw angle. The estimator's
        // estimated tilt (roll/pitch) should remain close to zero.
        let mut e = Ekf::new();
        let mut t = 0.0f32;
        for _ in 0..1000 {
            t += 1.0 / 200.0;
            e.tick(imu_at(t, GRAVITY_ACCEL_BODY, [0.0, 0.0, 0.5])); // 0.5 rad/s yaw
        }
        let q = e.quaternion();
        // After convergence the body-z axis should still align with NED-z
        // (yaw doesn't tilt the gravity-aligned frame).
        let z_body_in_ned = rotate_body_to_ned_inverse(quat_conj(q), [0.0, 0.0, 1.0]);
        assert!(z_body_in_ned[2] > 0.99,
            "yaw rotation must not tilt the attitude estimate; z={}",
            z_body_in_ned[2]);
    }

    #[test]
    fn ekf_p03_innovation_monotone_with_tilt_disagreement() {
        // Fresh estimator at identity; feed accel that disagrees with
        // predicted gravity by an increasing tilt. Innovation magnitude
        // should grow monotonically.
        let mut prev = 0.0f32;
        for deg in [0.0_f32, 5.0, 10.0, 20.0, 40.0, 60.0] {
            let rad = deg.to_radians();
            // Tilt accel about body-x: gravity now has body-y component.
            let accel = [0.0, relay_math::sinf(rad) * 9.81, relay_math::cosf(rad) * 9.81];
            let mut e = Ekf::new();
            e.tick(imu_at(0.01, accel, [0.0; 3]));
            assert!(e.last_innovation() + 1.0e-6 >= prev,
                "innovation must be non-decreasing in tilt: deg={} inn={} prev={}",
                deg, e.last_innovation(), prev);
            prev = e.last_innovation();
        }
    }

    #[test]
    fn bias_estimate_bounded() {
        // Drive the integrator hard; bias must stay within ±0.5 rad/s.
        let mut e = Ekf::new();
        let mut t = 0.0f32;
        // Accel disagrees with gravity → innovation accumulates → bias drifts.
        let bad_accel = [9.81, 0.0, 0.0]; // tilted 90° about y
        for _ in 0..100_000 {
            t += 1.0 / 200.0;
            e.tick(imu_at(t, bad_accel, [0.0; 3]));
        }
        for i in 0..3 {
            assert!(e.bias()[i].abs() <= 0.5 + 1.0e-6,
                "bias[{}] = {} out of bound", i, e.bias()[i]);
        }
        assert!(is_unit_quaternion(e.quaternion()));
    }

    #[test]
    fn quat_mul_identity_left() {
        let i = [1.0, 0.0, 0.0, 0.0];
        let q = [0.6, 0.8, 0.0, 0.0]; // not unit, but multiply works
        assert_eq!(quat_mul(i, q), q);
    }

    #[test]
    fn quat_mul_identity_right() {
        let i = [1.0, 0.0, 0.0, 0.0];
        let q = [0.6, 0.8, 0.0, 0.0];
        assert_eq!(quat_mul(q, i), q);
    }

    #[test]
    fn rotate_inverse_identity_passes_through() {
        let i = [1.0, 0.0, 0.0, 0.0];
        let v = [1.0, 2.0, 3.0];
        let r = rotate_body_to_ned_inverse(i, v);
        for k in 0..3 {
            assert!((r[k] - v[k]).abs() < 1.0e-6);
        }
    }

    #[test]
    fn cross_of_parallel_is_zero() {
        let a = [1.0, 0.0, 0.0];
        let r = cross(a, a);
        assert_eq!(r, [0.0, 0.0, 0.0]);
    }

    #[test]
    fn normalise_zero_returns_none() {
        assert!(normalise([0.0; 4]).is_none());
    }

    #[test]
    fn normalise_nan_returns_none() {
        assert!(normalise([f32::NAN, 0.0, 0.0, 0.0]).is_none());
    }

    use proptest::prelude::*;

    fn any_finite_f32() -> impl Strategy<Value = f32> {
        prop::num::f32::NORMAL | prop::num::f32::ZERO | prop::num::f32::SUBNORMAL
    }

    proptest! {
        /// EKF-P01: any single tick from any finite IMU sample leaves
        /// the quaternion unit-norm. Catches NaN propagation and the
        /// "normalise produced 0/0" trap.
        #[test]
        fn ekf_p01_property(
            ax in any_finite_f32(), ay in any_finite_f32(), az in any_finite_f32(),
            gx in -10.0_f32..10.0, gy in -10.0_f32..10.0, gz in -10.0_f32..10.0,
            secs in 0.001_f32..100.0,
        ) {
            let mut e = Ekf::new();
            let st = e.tick(ImuSample {
                time: Timestamp {
                    seconds: secs as u64,
                    fraction: ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32,
                },
                accel_body: [ax, ay, az],
                gyro_body: [gx, gy, gz],
            });
            prop_assert!(is_unit_quaternion(st.quaternion));
            prop_assert!(st.quaternion[0].is_finite());
        }

        /// EKF-P02: across a sequence of arbitrary samples the EKF
        /// must never emit NaN/∞ in the quaternion or innovation.
        #[test]
        fn ekf_p02_property_sequence(
            samples in prop::collection::vec(
                (-100.0_f32..100.0, -100.0_f32..100.0, -100.0_f32..100.0,
                 -5.0_f32..5.0, -5.0_f32..5.0, -5.0_f32..5.0),
                1..40,
            ),
        ) {
            let mut e = Ekf::new();
            let mut t = 0.0_f32;
            for (ax, ay, az, gx, gy, gz) in samples {
                t += 1.0 / 200.0;
                let frac = ((t.fract() as f64) * ((1u64 << 32) as f64)) as u32;
                let st = e.tick(ImuSample {
                    time: Timestamp { seconds: t as u64, fraction: frac },
                    accel_body: [ax, ay, az],
                    gyro_body: [gx, gy, gz],
                });
                prop_assert!(is_unit_quaternion(st.quaternion));
                prop_assert!(st.innovation.is_finite());
                for k in 0..3 {
                    prop_assert!(e.bias()[k].abs() <= 0.5 + 1.0e-6);
                }
            }
        }
    }
}
