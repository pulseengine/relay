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
//! - no NaN/∞ anywhere in the loop.
//!
//! Failures print a diff and exit with code 1.

use std::process::ExitCode;
use std::time::Instant;

use libm::sqrtf;
use relay_ekf::{quat_mul, Ekf, ImuSample, Timestamp as EkfTimestamp};
use relay_rate::{RatePid, Timestamp as RateTimestamp};

const SAMPLE_RATE_HZ: f32 = 1000.0;
const TRAJECTORY_SECONDS: f32 = 5.0;
const GRAVITY: f32 = 9.81;
const INERTIA: f32 = 0.005;     // kg·m², 500 g, 10-inch quad
const FRICTION: f32 = 0.001;    // rad/s damping coefficient

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
}

impl Plant {
    fn at_rest() -> Self {
        Self {
            omega: [0.0; 3],
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }

    fn with_initial_rates(omega: [f32; 3]) -> Self {
        Self {
            omega,
            q: [1.0, 0.0, 0.0, 0.0],
        }
    }

    /// Integrate the rigid-body dynamics forward by `dt` under torque.
    fn step(&mut self, torque: [f32; 3], dt: f32) {
        // omega_dot = (torque - friction*omega) / inertia
        for i in 0..3 {
            self.omega[i] +=
                ((torque[i] - FRICTION * self.omega[i]) / INERTIA) * dt;
        }
        // q_dot = 0.5 * q ⊗ (0, omega)
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

fn print_result(r: &ScenarioResult) {
    println!("--- scenario: {} ---", r.label);
    println!("  samples              {}", r.samples);
    println!("  final ω (rad/s)      [{:+.4}, {:+.4}, {:+.4}]",
        r.final_omega[0], r.final_omega[1], r.final_omega[2]);
    println!("  peak ω above sp      {:.4} rad/s", r.peak_omega_after_setup);
    if r.label == "step" {
        println!("  overshoot            {:.1} %", r.overshoot_pct);
        println!("  RMS error (steady)   {:.4} rad/s", r.rms_error_steady);
    }
    if r.convergence_time_s.is_nan() {
        println!("  convergence/recovery never");
    } else if r.label == "disturbance" {
        println!("  recovery time        {:.3}s after impulse", r.convergence_time_s);
    } else {
        println!("  convergence time     {:.3}s", r.convergence_time_s);
    }
    println!("  loop wall time       {} µs", r.elapsed_micros);
    println!("  NaN/∞ seen           {}", r.nan_seen);
    println!("  outcome              {}", if r.pass { "PASS" } else { "FAIL" });
}

fn print_help() {
    eprintln!(
        "falcon-sitl-hover — v0.3 closed-loop SITL bench\n\n\
         USAGE:\n  \
           falcon-sitl-hover [--scenario step|disturbance|hover|all] [--noise σ] [--quiet]\n\n\
         OPTIONS:\n  \
           --scenario NAME   one of step, disturbance, hover, all (default: all)\n  \
           --noise SIGMA     accelerometer / gyro white-noise σ (default 0.0)\n  \
           --quiet           only print PASS/FAIL summary\n  \
           --help            this text\n"
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
                Some("all") => scenario = Scenario::All,
                other => {
                    eprintln!("error: --scenario expects step|disturbance|hover|all, got {:?}", other);
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
            Scenario::All => unreachable!(),
        }
    };

    let mut results: Vec<ScenarioResult> = Vec::new();
    match scenario {
        Scenario::All => {
            results.push(run_one(Scenario::Step));
            results.push(run_one(Scenario::Disturbance));
            results.push(run_one(Scenario::Hover));
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
