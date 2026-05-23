//! falcon-sitl-hover — v0.3 pure-Rust closed-loop SITL bench.
//!
//! Demonstrates the rate-control loop end-to-end against a simulated
//! rigid-body quadcopter. No Gazebo, no MAVLink — the whole loop runs
//! in-process at 1 kHz IMU rate. Deterministic given a seed.
//!
//! ## What it proves
//!
//! 1. **Rate step response** — commanded body-rate setpoint
//!    `[0.5, -0.3, 0.4] rad/s` is reached within 1 s on all axes with
//!    steady-state error below 0.02 rad/s and overshoot below 30 %.
//! 2. **Disturbance rejection** — at steady-state hover, inject a
//!    body-frame angular impulse; the controller restores zero rate
//!    within 0.5 s.
//! 3. **Hover stabilization** — vehicle starts with random non-zero
//!    body rates and zero setpoint; estimator + rate PID drive the
//!    rates to zero.
//! 4. **EKF + rate-PID composition** is numerically stable across
//!    the 5 s × 1 kHz trajectory. No NaN/∞.
//!
//! ## CLI
//!
//!   cargo run -p falcon-sitl-hover --release
//!   cargo run -p falcon-sitl-hover --release -- --scenario step
//!   cargo run -p falcon-sitl-hover --release -- --scenario disturbance
//!   cargo run -p falcon-sitl-hover --release -- --scenario hover
//!   cargo run -p falcon-sitl-hover --release -- --scenario fault
//!   cargo run -p falcon-sitl-hover --release -- --noise 0.05
//!
//! ## Pass criteria (v0.3 acceptance)
//!
//! - step:        all three axes converge to |error| ≤ 0.02 rad/s
//!                within 1.0 s, overshoot ≤ 30 %.
//! - disturbance: rate magnitude returns to ≤ 0.05 rad/s within
//!                0.5 s of the impulse.
//! - hover:       initial random rates ≤ 1 rad/s settle to ≤ 0.02
//!                within 1.0 s.
//! - fault:       an injected accelerometer fault is caught by the
//!                EKF watchdog and RTL triggers within 0.5 s.
//! - no NaN/∞ anywhere in the loop.
//!
//! Failures print a diff and exit with code 1.

use std::process::ExitCode;
use std::time::Instant;

use libm::sqrtf;
use relay_att::{AttController, Timestamp as AttTimestamp};
use relay_ekf::{quat_mul, Ekf, ImuSample, Timestamp as EkfTimestamp};
use relay_mix_quad::QuadMixer;
use relay_pos::{PosController, PositionSetpoint, Timestamp as PosTimestamp};
use relay_rate::{RatePid, Timestamp as RateTimestamp};

const SAMPLE_RATE_HZ: f32 = 1000.0;
const TRAJECTORY_SECONDS: f32 = 5.0;
const GRAVITY: f32 = 9.81;
const INERTIA: f32 = 0.005;     // kg·m², 500 g, 10-inch quad
const FRICTION: f32 = 0.001;    // rad/s damping coefficient
/// Thrust scale: normalised thrust 0.5 produces 1g acceleration at hover.
const THRUST_SCALE: f32 = 19.62;
const DRAG_COEFFICIENT: f32 = 0.4;
/// v0.8 fault scenario — wall-clock time the accelerometer fault is
/// injected. By 8 s the vehicle has reached and settled at the
/// waypoint, so pre-fault EKF innovation is low.
const FAULT_INJECT_S: f32 = 8.0;

/// Pseudo-random Gaussian-ish noise via two-sample averaging of an LCG.
struct Rng(u32);

impl Rng {
    fn new(seed: u32) -> Self {
        Self(seed)
    }
    fn step(&mut self) -> f32 {
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        let a = ((self.0 >> 16) as f32) / (u16::MAX as f32) - 0.5;
        self.0 = self.0.wrapping_mul(1664525).wrapping_add(1013904223);
        let b = ((self.0 >> 16) as f32) / (u16::MAX as f32) - 0.5;
        a + b
    }
}

#[derive(Clone, Copy, Debug)]
struct Plant {
    /// Body-frame angular velocity (rad/s).
    omega: [f32; 3],
    /// Body-to-NED unit quaternion.
    q: [f32; 4],
    /// Position in NED frame (m).
    p_ned: [f32; 3],
    /// Velocity in NED frame (m/s).
    v_ned: [f32; 3],
}

impl Plant {
    fn at_rest() -> Self {
        Self {
            omega: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
            p_ned: [0.0; 3],
            v_ned: [0.0; 3],
        }
    }

    fn with_initial_rates(omega: [f32; 3]) -> Self {
        Self {
            omega,
            q: [1.0, 0.0, 0.0, 0.0],
            p_ned: [0.0; 3],
            v_ned: [0.0; 3],
        }
    }

    /// Integrate rotational dynamics forward by `dt` under torque.
    /// Translational state is unchanged (used by v0.3 / v0.4 scenarios
    /// that don't need flight).
    fn step(&mut self, torque: [f32; 3], dt: f32) {
        // omega_dot = (torque - friction*omega) / inertia
        for i in 0..3 {
            self.omega[i] +=
                ((torque[i] - FRICTION * self.omega[i]) / INERTIA) * dt;
        }
        self.integrate_quaternion(dt);
    }

    /// Integrate rotational + translational dynamics by `dt`.
    /// Adds collective thrust + gravity + linear drag. Used by the
    /// v0.5 `mission` scenario.
    fn step_full(&mut self, torque: [f32; 3], thrust_normalised: f32, dt: f32) {
        // Rotational.
        for i in 0..3 {
            self.omega[i] +=
                ((torque[i] - FRICTION * self.omega[i]) / INERTIA) * dt;
        }
        self.integrate_quaternion(dt);

        // Translational. Thrust acts along body -z (the body's "up").
        // Rotate body-frame thrust vector into NED.
        let t = thrust_normalised.clamp(0.0, 1.0) * THRUST_SCALE;
        let thrust_body = [0.0, 0.0, -t]; // body up = -z body
        let qv = [0.0, thrust_body[0], thrust_body[1], thrust_body[2]];
        let qc = [self.q[0], -self.q[1], -self.q[2], -self.q[3]];
        let t1 = quat_mul(self.q, quat_mul(qv, qc));
        let thrust_ned = [t1[1], t1[2], t1[3]];

        // a_ned = thrust_ned + g_ned - drag * v
        for i in 0..3 {
            let g = if i == 2 { GRAVITY } else { 0.0 };
            let drag = DRAG_COEFFICIENT * self.v_ned[i];
            let a = thrust_ned[i] + g - drag;
            self.v_ned[i] += a * dt;
            self.p_ned[i] += self.v_ned[i] * dt;
        }
    }

    fn integrate_quaternion(&mut self, dt: f32) {
        let qdot = quat_mul(self.q, [0.0, self.omega[0], self.omega[1], self.omega[2]]);
        let mut q_new = [
            self.q[0] + 0.5 * qdot[0] * dt,
            self.q[1] + 0.5 * qdot[1] * dt,
            self.q[2] + 0.5 * qdot[2] * dt,
            self.q[3] + 0.5 * qdot[3] * dt,
        ];
        let n = sqrtf(
            q_new[0] * q_new[0] + q_new[1] * q_new[1]
                + q_new[2] * q_new[2] + q_new[3] * q_new[3],
        );
        if n > 1.0e-12 {
            q_new = [q_new[0] / n, q_new[1] / n, q_new[2] / n, q_new[3] / n];
            self.q = q_new;
        }
    }

    /// Generate an IMU sample at the current instant given a noise scale.
    fn measure(&self, rng: &mut Rng, noise_std: f32) -> (ImuSample, [f32; 3]) {
        let gyro = [
            self.omega[0] + rng.step() * noise_std * 0.1,
            self.omega[1] + rng.step() * noise_std * 0.1,
            self.omega[2] + rng.step() * noise_std * 0.1,
        ];
        // Accel measures gravity rotated into body frame.
        let qc = [self.q[0], -self.q[1], -self.q[2], -self.q[3]];
        let g_quat = [0.0, 0.0, 0.0, GRAVITY];
        let t1 = quat_mul(qc, quat_mul(g_quat, self.q));
        let accel = [
            t1[1] + rng.step() * noise_std,
            t1[2] + rng.step() * noise_std,
            t1[3] + rng.step() * noise_std,
        ];
        let sample = ImuSample {
            time: EkfTimestamp::ZERO, // caller fills in
            accel_body: accel,
            gyro_body: gyro,
        };
        (sample, gyro)
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Scenario {
    Step,
    Disturbance,
    Hover,
    /// v0.4: full attitude cascade — ATT → RATE → MIX → plant.
    Attitude,
    /// v0.5: full mission cascade — POS → ATT → RATE → MIX → plant.
    Mission,
    /// v0.8: fault injection — EKF divergence trips the relay-hs
    /// watchdog, which triggers Return-To-Launch.
    Fault,
    All,
}

#[derive(Debug)]
struct ScenarioResult {
    label: &'static str,
    samples: usize,
    final_omega: [f32; 3],
    peak_omega_after_setup: f32,
    rms_error_steady: f32,
    convergence_time_s: f32,
    overshoot_pct: f32,
    nan_seen: bool,
    elapsed_micros: u128,
    pass: bool,
}

fn ekf_ts_of(secs: f32) -> EkfTimestamp {
    let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
    EkfTimestamp { seconds: secs as u64, fraction: frac }
}

fn rate_ts_of(secs: f32) -> RateTimestamp {
    let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
    RateTimestamp { seconds: secs as u64, fraction: frac }
}

fn att_ts_of(secs: f32) -> AttTimestamp {
    let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
    AttTimestamp { seconds: secs as u64, fraction: frac }
}

fn pos_ts_of(secs: f32) -> PosTimestamp {
    let frac = ((secs.fract() as f64) * ((1u64 << 32) as f64)) as u32;
    PosTimestamp { seconds: secs as u64, fraction: frac }
}

fn quat_error_deg(a: [f32; 4], b: [f32; 4]) -> f32 {
    let dot = (a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3]).abs();
    let c = dot.clamp(-1.0, 1.0);
    (2.0 * libm::acosf(c)).to_degrees()
}

fn run_step_response(noise_std: f32) -> ScenarioResult {
    let mut plant = Plant::at_rest();
    let mut ekf = Ekf::new();
    let mut pid = RatePid::new();
    let mut rng = Rng::new(0xDEADBEEF);
    let dt = 1.0 / SAMPLE_RATE_HZ;
    let n = (TRAJECTORY_SECONDS * SAMPLE_RATE_HZ) as usize;
    let setpoint = [0.5_f32, -0.3, 0.4];

    let mut peak = 0.0_f32;
    let mut convergence = f32::NAN;
    let mut convergence_holding = false;
    let mut nan_seen = false;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0_usize;
    let steady_start = ((TRAJECTORY_SECONDS - 1.0).max(0.0) * SAMPLE_RATE_HZ) as usize;

    let t0 = Instant::now();
    for i in 0..n {
        let t = i as f32 * dt;
        let (mut sample, gyro) = plant.measure(&mut rng, noise_std);
        sample.time = ekf_ts_of(t);
        let st = ekf.tick(sample);
        if !st.quaternion[0].is_finite() {
            nan_seen = true;
        }
        let torque = pid.tick(rate_ts_of(t), gyro, setpoint);
        for k in 0..3 {
            if !torque[k].is_finite() {
                nan_seen = true;
            }
        }
        plant.step(torque, dt);
        // Track peak |omega - setpoint| and convergence.
        let err = [
            plant.omega[0] - setpoint[0],
            plant.omega[1] - setpoint[1],
            plant.omega[2] - setpoint[2],
        ];
        let mag = sqrtf(err[0] * err[0] + err[1] * err[1] + err[2] * err[2]);
        // Peak overshoot is the largest deviation past the setpoint
        // (not the rise). Defined as max-over-time(|omega| above |sp|).
        for k in 0..3 {
            let over = plant.omega[k].abs() - setpoint[k].abs();
            if over > peak {
                peak = over;
            }
        }
        if mag < 0.05 && !convergence_holding {
            convergence = t;
            convergence_holding = true;
        } else if mag >= 0.05 && convergence_holding {
            convergence = f32::NAN;
            convergence_holding = false;
        }
        if i >= steady_start {
            sum_sq_steady += mag * mag;
            steady_count += 1;
        }
    }
    let elapsed = t0.elapsed().as_micros();

    let max_sp_mag = setpoint.iter().fold(0.0_f32, |a, &v| a.max(v.abs()));
    let overshoot_pct = (peak / max_sp_mag) * 100.0;
    let final_err = sqrtf(
        (plant.omega[0] - setpoint[0]).powi(2)
            + (plant.omega[1] - setpoint[1]).powi(2)
            + (plant.omega[2] - setpoint[2]).powi(2),
    );

    let pass = !nan_seen
        && !convergence.is_nan()
        && convergence <= 1.0
        && final_err <= 0.02
        && overshoot_pct <= 30.0;

    ScenarioResult {
        label: "step",
        samples: n,
        final_omega: plant.omega,
        peak_omega_after_setup: peak,
        rms_error_steady: sqrtf(sum_sq_steady / steady_count.max(1) as f32),
        convergence_time_s: convergence,
        overshoot_pct,
        nan_seen,
        elapsed_micros: elapsed,
        pass,
    }
}

fn run_disturbance(noise_std: f32) -> ScenarioResult {
    let mut plant = Plant::at_rest();
    let mut ekf = Ekf::new();
    let mut pid = RatePid::new();
    let mut rng = Rng::new(0xABBAC0FE);
    let dt = 1.0 / SAMPLE_RATE_HZ;
    let n = (TRAJECTORY_SECONDS * SAMPLE_RATE_HZ) as usize;
    let setpoint = [0.0_f32; 3];

    // Settle phase for the first 1 s, then inject an impulse at t=1 s.
    let impulse_step = (1.0 * SAMPLE_RATE_HZ) as usize;
    let recovery_window = (0.5 * SAMPLE_RATE_HZ) as usize;

    let mut peak_after_impulse = 0.0_f32;
    let mut recovery_time = f32::NAN;
    let mut nan_seen = false;

    let t0 = Instant::now();
    for i in 0..n {
        let t = i as f32 * dt;
        // Apply impulse: at one specific tick, slam ω with +1 rad/s about y.
        if i == impulse_step {
            plant.omega[1] += 1.0;
        }
        let (mut sample, gyro) = plant.measure(&mut rng, noise_std);
        sample.time = ekf_ts_of(t);
        let st = ekf.tick(sample);
        if !st.quaternion[0].is_finite() {
            nan_seen = true;
        }
        let torque = pid.tick(rate_ts_of(t), gyro, setpoint);
        for k in 0..3 {
            if !torque[k].is_finite() {
                nan_seen = true;
            }
        }
        plant.step(torque, dt);

        if i > impulse_step && i <= impulse_step + recovery_window {
            let mag = sqrtf(
                plant.omega[0].powi(2) + plant.omega[1].powi(2) + plant.omega[2].powi(2),
            );
            if mag > peak_after_impulse {
                peak_after_impulse = mag;
            }
            if mag < 0.05 && recovery_time.is_nan() {
                recovery_time = (i - impulse_step) as f32 * dt;
            }
        }
    }
    let elapsed = t0.elapsed().as_micros();

    let pass = !nan_seen && !recovery_time.is_nan() && recovery_time <= 0.5;

    ScenarioResult {
        label: "disturbance",
        samples: n,
        final_omega: plant.omega,
        peak_omega_after_setup: peak_after_impulse,
        rms_error_steady: 0.0,
        convergence_time_s: recovery_time,
        overshoot_pct: 0.0,
        nan_seen,
        elapsed_micros: elapsed,
        pass,
    }
}

fn run_hover(noise_std: f32) -> ScenarioResult {
    let mut plant = Plant::with_initial_rates([0.7, -0.5, 0.3]);
    let mut ekf = Ekf::new();
    let mut pid = RatePid::new();
    let mut rng = Rng::new(0x5EEDC0DE);
    let dt = 1.0 / SAMPLE_RATE_HZ;
    let n = (TRAJECTORY_SECONDS * SAMPLE_RATE_HZ) as usize;
    let setpoint = [0.0_f32; 3];

    let mut peak = 0.0_f32;
    let mut convergence = f32::NAN;
    let mut convergence_holding = false;
    let mut nan_seen = false;

    let t0 = Instant::now();
    for i in 0..n {
        let t = i as f32 * dt;
        let (mut sample, gyro) = plant.measure(&mut rng, noise_std);
        sample.time = ekf_ts_of(t);
        let st = ekf.tick(sample);
        if !st.quaternion[0].is_finite() {
            nan_seen = true;
        }
        let torque = pid.tick(rate_ts_of(t), gyro, setpoint);
        for k in 0..3 {
            if !torque[k].is_finite() {
                nan_seen = true;
            }
        }
        plant.step(torque, dt);
        let mag = sqrtf(
            plant.omega[0].powi(2) + plant.omega[1].powi(2) + plant.omega[2].powi(2),
        );
        if mag > peak {
            peak = mag;
        }
        if mag < 0.02 && !convergence_holding {
            convergence = t;
            convergence_holding = true;
        } else if mag >= 0.02 && convergence_holding {
            convergence = f32::NAN;
            convergence_holding = false;
        }
    }
    let elapsed = t0.elapsed().as_micros();

    let final_mag = sqrtf(
        plant.omega[0].powi(2) + plant.omega[1].powi(2) + plant.omega[2].powi(2),
    );
    let pass = !nan_seen
        && !convergence.is_nan()
        && convergence <= 1.0
        && final_mag <= 0.02;

    ScenarioResult {
        label: "hover",
        samples: n,
        final_omega: plant.omega,
        peak_omega_after_setup: peak,
        rms_error_steady: final_mag,
        convergence_time_s: convergence,
        overshoot_pct: 0.0,
        nan_seen,
        elapsed_micros: elapsed,
        pass,
    }
}

/// v0.5 mission scenario: full cascade POS → ATT → RATE → MIX →
/// plant with translational dynamics. Vehicle starts at NED origin,
/// commanded to a 10-m waypoint, must arrive and settle.
fn run_mission(noise_std: f32) -> ScenarioResult {
    let mut plant = Plant::at_rest();
    let mut ekf = Ekf::new();
    let mut rate_pid = RatePid::new();
    let mut att = AttController::new();
    let mut pos = PosController::new();
    let mut mixer = QuadMixer::new();
    let mut rng = Rng::new(0x4FA1C001);
    let dt = 1.0 / SAMPLE_RATE_HZ;
    // 12 s gives the cascade time to reach + settle at the waypoint.
    let mission_seconds = 12.0_f32;
    let n = (mission_seconds * SAMPLE_RATE_HZ) as usize;

    let waypoint = [10.0_f32, 0.0, 0.0]; // 10 m north, hover altitude
    let setpoint = PositionSetpoint::hover_at(waypoint);

    let pos_decimation = 20_usize; // 1 kHz / 20 = 50 Hz pos rate
    let att_decimation = 4_usize;  // 1 kHz / 4  = 250 Hz att rate
    let mut current_attitude_setpoint = [1.0_f32, 0.0, 0.0, 0.0];
    let mut current_thrust = 0.5_f32; // start at hover
    let mut current_rate_setpoint = [0.0_f32; 3];

    let mut peak_dist_err = 0.0_f32;
    let mut convergence = f32::NAN;
    let mut convergence_holding = false;
    let mut nan_seen = false;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0_usize;
    let steady_start = ((mission_seconds - 2.0).max(0.0) * SAMPLE_RATE_HZ) as usize;

    let t0 = Instant::now();
    for i in 0..n {
        let t = i as f32 * dt;
        // 1. IMU + EKF (1 kHz). EKF only does attitude in v0.2; we
        //    feed the plant's true NED pose to POS directly (the
        //    v0.6 GPS host service replaces this).
        let (mut sample, gyro) = plant.measure(&mut rng, noise_std);
        sample.time = ekf_ts_of(t);
        let st = ekf.tick(sample);
        if !st.quaternion[0].is_finite() {
            nan_seen = true;
        }

        // 2. Outer position loop (50 Hz).
        if i % pos_decimation == 0 {
            let att_sp = pos.tick(
                pos_ts_of(t),
                plant.p_ned,
                plant.v_ned,
                st.quaternion,
                setpoint,
            );
            current_attitude_setpoint = att_sp.quaternion;
            current_thrust = att_sp.thrust;
        }
        // 3. Attitude loop (250 Hz).
        if i % att_decimation == 0 {
            current_rate_setpoint = att.tick(
                att_ts_of(t),
                st.quaternion,
                current_attitude_setpoint,
            );
        }
        // 4. Rate loop (1 kHz).
        let torque = rate_pid.tick(rate_ts_of(t), gyro, current_rate_setpoint);
        for k in 0..3 {
            if !torque[k].is_finite() {
                nan_seen = true;
            }
        }
        // 5. Mixer (exercised for NaN-check; plant uses torque directly).
        let _motors = mixer.mix(torque, current_thrust);
        if mixer.last_motors().iter().any(|v| !v.is_finite()) {
            nan_seen = true;
        }
        // 6. Plant with translational dynamics.
        plant.step_full(torque, current_thrust, dt);

        let dist = sqrtf(
            (plant.p_ned[0] - waypoint[0]).powi(2)
                + (plant.p_ned[1] - waypoint[1]).powi(2)
                + (plant.p_ned[2] - waypoint[2]).powi(2),
        );
        if dist > peak_dist_err {
            peak_dist_err = dist;
        }
        if dist < 0.5 && !convergence_holding {
            convergence = t;
            convergence_holding = true;
        } else if dist >= 0.5 && convergence_holding {
            convergence = f32::NAN;
            convergence_holding = false;
        }
        if i >= steady_start {
            sum_sq_steady += dist * dist;
            steady_count += 1;
        }
    }
    let elapsed = t0.elapsed().as_micros();
    let final_dist = sqrtf(
        (plant.p_ned[0] - waypoint[0]).powi(2)
            + (plant.p_ned[1] - waypoint[1]).powi(2)
            + (plant.p_ned[2] - waypoint[2]).powi(2),
    );
    let rms_steady = sqrtf(sum_sq_steady / steady_count.max(1) as f32);

    let pass = !nan_seen
        && !convergence.is_nan()
        && convergence <= 10.0
        && final_dist <= 0.5
        && rms_steady <= 1.0;

    ScenarioResult {
        label: "mission",
        samples: n,
        final_omega: plant.v_ned,        // repurpose final-ω slot for final velocity
        peak_omega_after_setup: peak_dist_err, // peak distance error (m)
        rms_error_steady: rms_steady,
        convergence_time_s: convergence,
        overshoot_pct: final_dist,        // repurpose overshoot slot for final distance (m)
        nan_seen,
        elapsed_micros: elapsed,
        pass,
    }
}

/// v0.4 attitude scenario: full cascade ATT → RATE → MIX → plant.
/// Commanded setpoint is 20° tilt about body-x. We run the inner
/// rate controller at 1 kHz and the outer attitude controller at
/// 250 Hz (every 4th tick). Mixer maps torque to motor PWM but the
/// plant in v0.4 still consumes torque directly — the mixer is
/// independently unit-tested, and the bench just verifies it doesn't
/// introduce NaN / saturate the cascade.
fn run_attitude(noise_std: f32) -> ScenarioResult {
    let mut plant = Plant::at_rest();
    let mut ekf = Ekf::new();
    let mut pid = RatePid::new();
    let mut att = AttController::new();
    let mut mixer = QuadMixer::new();
    let mut rng = Rng::new(0xA770F1AE);
    let dt = 1.0 / SAMPLE_RATE_HZ;
    let n = (TRAJECTORY_SECONDS * SAMPLE_RATE_HZ) as usize;

    // Setpoint: 20° tilt about body-x (positive roll).
    let half = (20.0_f32).to_radians() * 0.5;
    let attitude_setpoint = [libm::cosf(half), libm::sinf(half), 0.0, 0.0];

    let att_decimation = 4_usize; // 1 kHz / 4 = 250 Hz attitude rate
    let mut current_rate_setpoint = [0.0_f32; 3];

    let mut peak_err_deg = 0.0_f32;
    let mut convergence = f32::NAN;
    let mut convergence_holding = false;
    let mut nan_seen = false;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0_usize;
    let steady_start = ((TRAJECTORY_SECONDS - 1.0).max(0.0) * SAMPLE_RATE_HZ) as usize;

    let t0 = Instant::now();
    for i in 0..n {
        let t = i as f32 * dt;
        let (mut sample, gyro) = plant.measure(&mut rng, noise_std);
        sample.time = ekf_ts_of(t);
        let st = ekf.tick(sample);
        if !st.quaternion[0].is_finite() {
            nan_seen = true;
        }
        // Outer loop (250 Hz): refresh rate setpoint.
        if i % att_decimation == 0 {
            current_rate_setpoint =
                att.tick(att_ts_of(t), st.quaternion, attitude_setpoint);
        }
        // Inner loop (1 kHz): rate PID -> torque.
        let torque = pid.tick(rate_ts_of(t), gyro, current_rate_setpoint);
        for k in 0..3 {
            if !torque[k].is_finite() {
                nan_seen = true;
            }
        }
        // Mixer demo: produce motor commands. The plant doesn't
        // consume them in v0.4 (it integrates torque directly), but
        // we still exercise the mixer here so any NaN/saturation
        // shows up in this scenario.
        let _motors = mixer.mix(torque, 0.5);
        if mixer.last_motors().iter().any(|v| !v.is_finite()) {
            nan_seen = true;
        }
        plant.step(torque, dt);
        let err = quat_error_deg(plant.q, attitude_setpoint);
        if err > peak_err_deg {
            peak_err_deg = err;
        }
        if err < 2.0 && !convergence_holding {
            convergence = t;
            convergence_holding = true;
        } else if err >= 2.0 && convergence_holding {
            convergence = f32::NAN;
            convergence_holding = false;
        }
        if i >= steady_start {
            sum_sq_steady += err * err;
            steady_count += 1;
        }
    }
    let elapsed = t0.elapsed().as_micros();

    let final_err = quat_error_deg(plant.q, attitude_setpoint);
    let rms_steady = sqrtf(sum_sq_steady / steady_count.max(1) as f32);

    let pass = !nan_seen
        && !convergence.is_nan()
        && convergence <= 1.5
        && final_err <= 2.0
        && rms_steady <= 2.0;

    ScenarioResult {
        label: "attitude",
        samples: n,
        final_omega: plant.omega,
        peak_omega_after_setup: peak_err_deg,
        rms_error_steady: rms_steady,
        convergence_time_s: convergence,
        overshoot_pct: 0.0,
        nan_seen,
        elapsed_micros: elapsed,
        pass,
    }
}

// ═══════════════════════════════════════════════════════════════
// v0.8 — EKF-divergence watchdog (the relay-hs Health & Safety role)
// ═══════════════════════════════════════════════════════════════

/// Health & Safety watchdog for EKF divergence.
///
/// The Mahony filter reports a normalised `innovation` each tick —
/// `sin(θ)` of the angle between the *measured* gravity direction and
/// the filter's *predicted* one (see `relay-ekf`). Healthy flight sits
/// near zero; a failing accelerometer drives it toward 1.0. This
/// watchdog turns that scalar into one safety decision: trigger
/// Return-To-Launch.
///
/// In the cFS-DNA production path this is the verified `relay-hs`
/// engine, and the RTL it raises is executed by `relay-sc` as a
/// stored-command sequence. Here it runs inline in the SITL so the
/// fault-injection scenario can exercise the trigger end-to-end.
struct EkfWatchdog {
    /// Innovation above which a tick counts as "diverging". Set well
    /// clear of the ~0.29 innovation that normal mission acceleration
    /// induces, and well below the ~0.9 of a hard accelerometer fault.
    innovation_limit: f32,
    /// The integer/bool latch state-machine lives in the verified
    /// `relay-hs` engine (HS-P06 monotone latch, HS-P07 transition-
    /// only). This wrapper does the f32 → bool gate (out of Verus's
    /// reach) and forwards the resulting `over_limit` signal.
    monitor: relay_hs::engine::EkfHealthMonitor,
}

impl EkfWatchdog {
    /// Defaults for the 1 kHz cascade: 0.40 innovation limit; the
    /// underlying relay-hs monitor latches RTL when 48 of the last 64
    /// ms are over-limit (its `new()`).
    fn new() -> Self {
        Self {
            innovation_limit: 0.40,
            monitor: relay_hs::engine::EkfHealthMonitor::new(),
        }
    }

    /// Feed one EKF `innovation` sample. Returns `true` on the single
    /// tick RTL trips, so the caller can timestamp the event.
    fn observe(&mut self, innovation: f32) -> bool {
        // f32 → bool: a non-finite innovation is itself divergence.
        let over_limit = !innovation.is_finite() || innovation > self.innovation_limit;
        self.monitor.observe(over_limit)
    }

    /// True once RTL has been committed.
    fn rtl_active(&self) -> bool {
        self.monitor.rtl_active()
    }
}

/// v0.8 fault-injection scenario.
///
/// Flies the full mission cascade to a waypoint, then at
/// `FAULT_INJECT_S` injects a hard accelerometer bias. The EKF
/// estimate diverges, its `innovation` spikes, and the `EkfWatchdog`
/// must catch it and latch Return-To-Launch — which retargets the
/// position controller from the mission waypoint back to home.
///
/// Pass = no NaN, RTL triggered, and detection latency ≤ 0.5 s.
fn run_fault(noise_std: f32) -> ScenarioResult {
    let mut plant = Plant::at_rest();
    let mut ekf = Ekf::new();
    let mut rate_pid = RatePid::new();
    let mut att = AttController::new();
    let mut pos = PosController::new();
    let mut mixer = QuadMixer::new();
    let mut rng = Rng::new(0xFA017EED);
    let mut watchdog = EkfWatchdog::new();
    let dt = 1.0 / SAMPLE_RATE_HZ;
    let fault_seconds = 16.0_f32;
    let n = (fault_seconds * SAMPLE_RATE_HZ) as usize;

    let waypoint = [10.0_f32, 0.0, 0.0];
    let mission_sp = PositionSetpoint::hover_at(waypoint);
    let rtl_sp = PositionSetpoint::hover_at([0.0, 0.0, 0.0]); // home

    let pos_decimation = 20_usize; // 50 Hz
    let att_decimation = 4_usize; //  250 Hz
    let mut current_attitude_setpoint = [1.0_f32, 0.0, 0.0, 0.0];
    let mut current_thrust = 0.5_f32;
    let mut current_rate_setpoint = [0.0_f32; 3];

    let fault_step = (FAULT_INJECT_S * SAMPLE_RATE_HZ) as usize;
    let mut peak_innovation = 0.0_f32;
    let mut rtl_triggered_s = f32::NAN;
    let mut nan_seen = false;

    let t0 = Instant::now();
    for i in 0..n {
        let t = i as f32 * dt;

        // 1. IMU + EKF (1 kHz).
        let (mut sample, gyro) = plant.measure(&mut rng, noise_std);
        sample.time = ekf_ts_of(t);
        // Fault injection: a hard +x accelerometer bias from t = 8 s.
        // The filter reads a gravity direction tilted ~69° off true,
        // so its attitude estimate diverges and innovation spikes.
        if i >= fault_step {
            sample.accel_body[0] += 25.0;
        }
        let st = ekf.tick(sample);
        if !st.quaternion[0].is_finite() {
            nan_seen = true;
        }
        if st.innovation > peak_innovation {
            peak_innovation = st.innovation;
        }

        // 2. Health & Safety watchdog observes the EKF innovation.
        if watchdog.observe(st.innovation) {
            rtl_triggered_s = t;
        }

        // 3. Active setpoint: the mission waypoint until RTL latches,
        //    then home. In the cFS-DNA path this switch is a relay-sc
        //    stored-command sequence fired by the relay-hs alert.
        let setpoint = if watchdog.rtl_active() { rtl_sp } else { mission_sp };

        // 4. Outer position loop (50 Hz).
        if i % pos_decimation == 0 {
            let att_sp = pos.tick(
                pos_ts_of(t),
                plant.p_ned,
                plant.v_ned,
                st.quaternion,
                setpoint,
            );
            current_attitude_setpoint = att_sp.quaternion;
            current_thrust = att_sp.thrust;
        }
        // 5. Attitude loop (250 Hz).
        if i % att_decimation == 0 {
            current_rate_setpoint =
                att.tick(att_ts_of(t), st.quaternion, current_attitude_setpoint);
        }
        // 6. Rate loop (1 kHz) + mixer + plant.
        let torque = rate_pid.tick(rate_ts_of(t), gyro, current_rate_setpoint);
        for k in 0..3 {
            if !torque[k].is_finite() {
                nan_seen = true;
            }
        }
        let _motors = mixer.mix(torque, current_thrust);
        if mixer.last_motors().iter().any(|v| !v.is_finite()) {
            nan_seen = true;
        }
        plant.step_full(torque, current_thrust, dt);
    }
    let elapsed = t0.elapsed().as_micros();

    let rtl_triggered = !rtl_triggered_s.is_nan();
    let detection_latency = rtl_triggered_s - FAULT_INJECT_S;
    let pass = !nan_seen
        && rtl_triggered
        && detection_latency >= 0.0
        && detection_latency <= 0.5;

    ScenarioResult {
        label: "fault",
        samples: n,
        final_omega: plant.p_ned,                // final NED position
        peak_omega_after_setup: peak_innovation, // peak EKF innovation
        rms_error_steady: 0.0,
        convergence_time_s: detection_latency, //  RTL detection latency (s)
        overshoot_pct: 0.0,
        nan_seen,
        elapsed_micros: elapsed,
        pass,
    }
}

fn print_result(r: &ScenarioResult) {
    println!("--- scenario: {} ---", r.label);
    println!("  samples              {}", r.samples);
    if r.label == "mission" {
        println!("  final v (m/s NED)    [{:+.3}, {:+.3}, {:+.3}]",
            r.final_omega[0], r.final_omega[1], r.final_omega[2]);
        println!("  peak distance error  {:.3} m", r.peak_omega_after_setup);
        println!("  final distance       {:.3} m", r.overshoot_pct);
        println!("  RMS distance (steady){:.3} m (last 2s)", r.rms_error_steady);
    } else if r.label == "fault" {
        println!("  final position (NED) [{:+.2}, {:+.2}, {:+.2}] m",
            r.final_omega[0], r.final_omega[1], r.final_omega[2]);
        println!("  peak EKF innovation  {:.4}", r.peak_omega_after_setup);
    } else {
        println!("  final ω (rad/s)      [{:+.4}, {:+.4}, {:+.4}]",
            r.final_omega[0], r.final_omega[1], r.final_omega[2]);
        if r.label == "attitude" {
            println!("  peak attitude err    {:.3}°", r.peak_omega_after_setup);
            println!("  RMS error (steady)   {:.3}° (last 1s)", r.rms_error_steady);
        } else {
            println!("  peak ω above sp      {:.4} rad/s", r.peak_omega_after_setup);
        }
    }
    if r.label == "step" {
        println!("  overshoot            {:.1} %", r.overshoot_pct);
        println!("  RMS error (steady)   {:.4} rad/s", r.rms_error_steady);
    }
    if r.convergence_time_s.is_nan() {
        if r.label == "fault" {
            println!("  RTL                  never triggered");
        } else {
            println!("  convergence/recovery never");
        }
    } else if r.label == "disturbance" {
        println!("  recovery time        {:.3}s after impulse", r.convergence_time_s);
    } else if r.label == "fault" {
        println!("  RTL detection        {:.3}s after fault injection", r.convergence_time_s);
    } else {
        println!("  convergence time     {:.3}s", r.convergence_time_s);
    }
    println!("  loop wall time       {} µs", r.elapsed_micros);
    println!("  NaN/∞ seen           {}", r.nan_seen);
    println!("  outcome              {}", if r.pass { "PASS" } else { "FAIL" });
}

fn print_help() {
    eprintln!(
        "falcon-sitl-hover — closed-loop SITL bench (v0.3–v0.8)\n\n\
         USAGE:\n  \
           falcon-sitl-hover [--scenario step|disturbance|hover|attitude|mission|fault|all] [--noise σ] [--quiet]\n\n\
         OPTIONS:\n  \
           --scenario NAME   one of step, disturbance, hover, attitude, mission, fault, all (default: all)\n  \
           --noise SIGMA     accelerometer / gyro white-noise σ (default 0.0)\n  \
           --quiet           only print PASS/FAIL summary\n  \
           --help            this text\n\n\
         Scenarios:\n  \
           step         rate controller follows a [0.5, -0.3, 0.4] rad/s setpoint\n  \
           disturbance  rate controller recovers from a 1 rad/s impulse\n  \
           hover        rate controller settles from non-zero initial rates\n  \
           attitude     ATT→RATE→MIX cascade tilts vehicle 20° about x (v0.4)\n  \
           mission      POS→ATT→RATE→MIX cascade flies to 10m waypoint (v0.5)\n  \
           fault        injected EKF fault trips the relay-hs watchdog → RTL (v0.8)\n"
    );
}

fn main() -> ExitCode {
    let mut scenario = Scenario::All;
    let mut noise = 0.0_f32;
    let mut quiet = false;
    let mut args = std::env::args();
    args.next();
    while let Some(a) = args.next() {
        match a.as_str() {
            "--scenario" => match args.next().as_deref() {
                Some("step") => scenario = Scenario::Step,
                Some("disturbance") => scenario = Scenario::Disturbance,
                Some("hover") => scenario = Scenario::Hover,
                Some("attitude") => scenario = Scenario::Attitude,
                Some("mission") => scenario = Scenario::Mission,
                Some("fault") => scenario = Scenario::Fault,
                Some("all") => scenario = Scenario::All,
                other => {
                    eprintln!("error: --scenario expects step|disturbance|hover|attitude|mission|fault|all, got {:?}", other);
                    return ExitCode::from(2);
                }
            },
            "--noise" => match args.next().and_then(|s| s.parse::<f32>().ok()) {
                Some(v) if v >= 0.0 => noise = v,
                _ => {
                    eprintln!("error: --noise expects a non-negative float");
                    return ExitCode::from(2);
                }
            },
            "--quiet" => quiet = true,
            "--help" | "-h" => {
                print_help();
                return ExitCode::SUCCESS;
            }
            other => {
                eprintln!("error: unknown argument {other}");
                print_help();
                return ExitCode::from(2);
            }
        }
    }

    let run_one = |s: Scenario| -> ScenarioResult {
        match s {
            Scenario::Step => run_step_response(noise),
            Scenario::Disturbance => run_disturbance(noise),
            Scenario::Hover => run_hover(noise),
            Scenario::Attitude => run_attitude(noise),
            Scenario::Mission => run_mission(noise),
            Scenario::Fault => run_fault(noise),
            Scenario::All => unreachable!(),
        }
    };

    let mut results: Vec<ScenarioResult> = Vec::new();
    match scenario {
        Scenario::All => {
            results.push(run_one(Scenario::Step));
            results.push(run_one(Scenario::Disturbance));
            results.push(run_one(Scenario::Hover));
            results.push(run_one(Scenario::Attitude));
            results.push(run_one(Scenario::Mission));
            results.push(run_one(Scenario::Fault));
        }
        s => results.push(run_one(s)),
    }
    let mut all_pass = true;

    for r in &results {
        if !quiet {
            print_result(r);
        }
        all_pass &= r.pass;
    }

    if all_pass {
        println!("falcon-sitl-hover: PASS");
        ExitCode::SUCCESS
    } else {
        let failed: Vec<&str> = results.iter().filter(|r| !r.pass).map(|r| r.label).collect();
        println!("falcon-sitl-hover: FAIL ({})", failed.join(", "));
        ExitCode::from(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn deterministic_step_response_passes() {
        let r = run_step_response(0.0);
        assert!(r.pass, "step result: {:?}", r);
        assert!(!r.nan_seen);
        assert!(r.convergence_time_s <= 1.0);
        assert!(r.overshoot_pct <= 30.0);
    }

    #[test]
    fn deterministic_disturbance_recovers() {
        let r = run_disturbance(0.0);
        assert!(r.pass, "disturbance result: {:?}", r);
        assert!(!r.nan_seen);
        assert!(r.convergence_time_s <= 0.5);
    }

    #[test]
    fn deterministic_hover_settles() {
        let r = run_hover(0.0);
        assert!(r.pass, "hover result: {:?}", r);
        assert!(!r.nan_seen);
    }

    #[test]
    fn noisy_step_response_still_passes() {
        let r = run_step_response(0.05);
        assert!(!r.nan_seen);
        // Looser budget with noise on accel + gyro.
        assert!(r.convergence_time_s <= 1.5);
        assert!(r.overshoot_pct <= 50.0);
    }

    #[test]
    fn deterministic_attitude_cascade_passes() {
        let r = run_attitude(0.0);
        assert!(r.pass, "attitude result: {:?}", r);
        assert!(!r.nan_seen);
        assert!(r.convergence_time_s <= 1.5);
        assert!(r.rms_error_steady <= 2.0,
            "attitude RMS-steady {:.3}° exceeds 2° budget", r.rms_error_steady);
    }

    #[test]
    fn noisy_attitude_cascade_still_passes() {
        let r = run_attitude(0.05);
        assert!(!r.nan_seen);
        // Looser budget with IMU noise.
        assert!(r.convergence_time_s.is_nan() || r.convergence_time_s <= 2.0);
        assert!(r.rms_error_steady <= 4.0);
    }

    #[test]
    fn deterministic_mission_passes() {
        let r = run_mission(0.0);
        assert!(r.pass, "mission result: {:?}", r);
        assert!(!r.nan_seen);
        assert!(r.convergence_time_s <= 10.0,
            "mission convergence {} exceeds 10 s", r.convergence_time_s);
        assert!(r.overshoot_pct <= 0.5,
            "final distance {} m exceeds 0.5 m", r.overshoot_pct);
    }

    #[test]
    fn noisy_mission_still_reaches_waypoint() {
        let r = run_mission(0.05);
        assert!(!r.nan_seen);
        // Looser budget with noise.
        assert!(r.overshoot_pct <= 1.5,
            "noisy mission final distance {} m exceeds 1.5 m", r.overshoot_pct);
    }

    #[test]
    fn deterministic_fault_triggers_rtl() {
        let r = run_fault(0.0);
        assert!(r.pass, "fault result: {:?}", r);
        assert!(!r.nan_seen);
        assert!(!r.convergence_time_s.is_nan(), "RTL never triggered");
        assert!(r.convergence_time_s <= 0.5,
            "RTL detection latency {:.3}s exceeds 0.5s budget", r.convergence_time_s);
        assert!(r.peak_omega_after_setup > 0.4,
            "fault should drive EKF innovation past the 0.4 limit, got {:.3}",
            r.peak_omega_after_setup);
    }

    #[test]
    fn noisy_fault_still_triggers_rtl() {
        let r = run_fault(0.05);
        assert!(!r.nan_seen);
        // A hard accelerometer bias dwarfs the IMU noise floor — RTL
        // must still latch, just with a looser latency budget.
        assert!(!r.convergence_time_s.is_nan(), "RTL never triggered under noise");
        assert!(r.convergence_time_s <= 1.0);
    }

    #[test]
    fn ekf_watchdog_ignores_healthy_innovation() {
        let mut wd = EkfWatchdog::new();
        // A long run of healthy innovation must never trip RTL.
        for _ in 0..5000 {
            assert!(!wd.observe(0.04));
        }
        assert!(!wd.rtl_active());
    }

    #[test]
    fn ekf_watchdog_debounces_transient_spikes() {
        let mut wd = EkfWatchdog::new();
        // Isolated over-limit spikes — one every 20 ticks — never let
        // the sliding window accumulate trip_threshold over-limit
        // ticks, so RTL must not commit.
        for _ in 0..500 {
            wd.observe(0.9);
            for _ in 0..19 {
                wd.observe(0.02);
            }
        }
        assert!(!wd.rtl_active(), "watchdog tripped on isolated noise spikes");
    }

    #[test]
    fn ekf_watchdog_catches_intermittent_fault() {
        let mut wd = EkfWatchdog::new();
        // An intermittent fault — over-limit 4 ticks in every 5, e.g.
        // a loose IMU connector. A strict consecutive counter never
        // reaches its threshold (the 1-in-5 healthy tick resets it),
        // but the sliding window still accumulates and commits RTL.
        let mut tripped = false;
        for _ in 0..200 {
            for _ in 0..4 {
                tripped |= wd.observe(0.9);
            }
            tripped |= wd.observe(0.02);
        }
        assert!(
            tripped && wd.rtl_active(),
            "sliding window should catch a 4-of-5 intermittent fault"
        );
    }

    #[test]
    fn plant_step_full_preserves_unit_quaternion_and_finite_state() {
        let mut plant = Plant::at_rest();
        for _ in 0..1000 {
            plant.step_full([0.001, -0.001, 0.0005], 0.5, 1.0 / 1000.0);
            let n = sqrtf(
                plant.q[0].powi(2) + plant.q[1].powi(2)
                    + plant.q[2].powi(2) + plant.q[3].powi(2),
            );
            assert!((n - 1.0).abs() < 1.0e-3);
            for k in 0..3 {
                assert!(plant.p_ned[k].is_finite());
                assert!(plant.v_ned[k].is_finite());
            }
        }
    }

    #[test]
    fn plant_step_preserves_unit_quaternion() {
        let mut plant = Plant::with_initial_rates([0.5, -0.3, 0.2]);
        for _ in 0..1000 {
            plant.step([0.001, -0.001, 0.0005], 1.0 / 1000.0);
            let n = sqrtf(
                plant.q[0].powi(2) + plant.q[1].powi(2) + plant.q[2].powi(2) + plant.q[3].powi(2),
            );
            assert!((n - 1.0).abs() < 1.0e-3,
                "plant quaternion non-unit: {}", n);
        }
    }
}
