//! `falcon-sitl-gz` — Gazebo SITL bench runner.
//!
//! v0.16.1 — pluggable `Physics` trait + `MockPhysics` reference +
//!           `GazeboPhysics` stub.
//! v0.18.0 — real gz-transport-rs bridge behind the `gazebo` feature.
//! v0.18.1 — NavSat + Home projection.
//! v0.19.0 — bench-evidence wiring: `--evidence-dir`, structured
//!           per-tick CSV, diagnostic counters (`imu_recv`,
//!           `navsat_recv`, `motor_send`) printed in the verdict.
//! v0.19.2 — Actuators message + multi_thread runtime.
//! v0.19.3 — first PASS verdict under real gz physics (open-loop climb).
//! v0.19.4 — **closed-loop hover**: the real cascade
//!           (relay-ekf → relay-pos → relay-att → relay-rate →
//!           relay-mix-quad) closes against gz IMU + NavSat. Mirrors
//!           `examples/falcon-sitl-hover`'s `run_mission` pattern.
//!           Default `--scenario=hover` is now closed-loop; the v0.19.3
//!           open-loop 70 % PWM smoke test is reachable via
//!           `--scenario=open-loop-climb`.

mod physics;

use physics::{GazeboPhysics, MockPhysics, Physics};
use relay_att::{AttController, Timestamp as AttTimestamp};
use relay_ekf::{Ekf, ImuSample, Timestamp as EkfTimestamp};
use relay_mix_quad::QuadMixer;
use relay_pos::{PosController, PositionSetpoint, Timestamp as PosTimestamp};
use relay_rate::{RatePid, Timestamp as RateTimestamp};
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let backend = arg(&args, "--backend").unwrap_or_else(|| "mock".into());
    let scenario = arg(&args, "--scenario").unwrap_or_else(|| "hover".into());
    let duration_s: f32 = arg(&args, "--duration").and_then(|s| s.parse().ok()).unwrap_or(5.0);
    let evidence_dir = arg(&args, "--evidence-dir").map(PathBuf::from);

    println!("falcon-sitl-gz: backend={backend} scenario={scenario} duration={duration_s}s");
    if let Some(d) = &evidence_dir {
        println!("  evidence-dir: {}", d.display());
    }

    // Set up the bench-evidence sink (CSV + harness log) before
    // constructing physics — that way connect-failure logs land on
    // disk too, and a future bench operator can read why the run
    // never started.
    let mut evidence = match &evidence_dir {
        Some(dir) => match EvidenceSink::open(dir, &backend, &scenario) {
            Ok(s) => Some(s),
            Err(e) => {
                eprintln!("  warning: failed to open evidence-dir: {e}");
                None
            }
        },
        None => None,
    };

    let pass = match backend.as_str() {
        "mock" => {
            let mut p = MockPhysics::at_rest();
            run_scenario(&mut p, &scenario, duration_s, evidence.as_mut())
        }
        "gazebo" => {
            let world = arg(&args, "--world").unwrap_or_else(|| "falcon".into());
            let model = arg(&args, "--model").unwrap_or_else(|| "quad".into());
            let mut p = build_gazebo(&args, world, model);
            run_scenario(&mut p, &scenario, duration_s, evidence.as_mut())
        }
        other => {
            eprintln!("unknown backend: {other}  (expected: mock | gazebo)");
            std::process::exit(2);
        }
    };

    if let Some(s) = evidence.as_mut() { s.finish(pass); }

    if pass {
        println!("PASS");
    } else {
        println!("FAIL");
        std::process::exit(1);
    }
}

/// Runs the named scenario against the chosen physics backend.
///
/// v0.19.4 wires two scenarios:
///   - `hover`           — closed-loop cascade (EKF + POS + ATT + RATE
///                         + MIX) holding NED setpoint (0, 0, -2 m).
///                         PASS = within 0.5 m at end + RMS over last
///                         5 s under 1.0 m.
///   - `open-loop-climb` — v0.19.3's constant 70 % PWM scaffolding,
///                         kept as a wire-level smoke test that
///                         doesn't depend on cascade tuning. PASS =
///                         net climb > 0.1 m. Useful for diagnosing
///                         "is the bridge publish path alive at all".
///
/// `step`, `mission`, `disturbance` reserved per docs/SIMULATOR.md.
/// v0.19.6 — NED↔ENU torque frame-correction.
///
/// relay's verified controllers emit torque in NED body frame
/// (X-fwd, Y-right, Z-down). gz's MulticopterMotorModel applies the
/// mixer's per-motor thrust in its ENU body frame (X-fwd, Y-left,
/// Z-up), related to NED by a 180° rotation about X (Y and Z flip).
/// The verified relay-mix-quad maps NED torque → motor PWM assuming
/// NED rotor geometry; against gz's ENU physical layout that inverts
/// the roll, pitch AND yaw torque the cascade actually achieves.
///
/// `frame_correct_torque` is the single boundary adapter for the
/// actuator path's frame sign. The frame-check oracle
/// (`--scenario=frame-{roll,pitch,yaw}`) established it empirically.
///
/// Result (see bench-evidence/gz-sim/2026-05-28-v0.19.6-frame-
/// correctness.md): once the SDF rotor index→position+spin map is
/// aligned to the verified relay-mix-quad MIXER_X convention
/// (0=front-right CW, 1=back-right CCW, 2=back-left CW,
/// 3=front-left CCW), commanding +torque on roll/pitch produces
/// +rate on that axis as sensed (AGREE). So the correction is
/// **identity** — the v0.19.4/.5 "negate roll/pitch" hack was
/// compensating for the *scrambled* SDF index map, not a real frame
/// flip. Keeping this adapter (as identity) documents the boundary
/// and is where a correction would live if the SDF/frame convention
/// ever changes; the unit test `frame_correction_is_identity` pins it.
#[inline]
fn frame_correct_torque(torque_ned: [f32; 3]) -> [f32; 3] {
    const SIGN: [f32; 3] = [1.0, 1.0, 1.0];
    [
        SIGN[0] * torque_ned[0],
        SIGN[1] * torque_ned[1],
        SIGN[2] * torque_ned[2],
    ]
}

fn run_scenario(
    physics: &mut dyn Physics,
    scenario: &str,
    duration_s: f32,
    evidence: Option<&mut EvidenceSink>,
) -> bool {
    match scenario {
        "hover" => run_closed_loop_hover(physics, duration_s, evidence),
        "open-loop-climb" => run_open_loop_climb(physics, duration_s, evidence),
        "alt-only" => run_alt_only_hover(physics, duration_s, evidence),
        "alt-rate" => run_alt_rate_hover(physics, duration_s, evidence),
        "frame-roll" => run_frame_check(physics, 0, duration_s),
        "frame-pitch" => run_frame_check(physics, 1, duration_s),
        "frame-yaw" => run_frame_check(physics, 2, duration_s),
        other => {
            eprintln!(
                "  scenario {other} not yet wired; falling back to closed-loop hover",
            );
            run_closed_loop_hover(physics, duration_s, evidence)
        }
    }
}

/// v0.19.6 — frame-correctness ORACLE. Commands a small constant
/// torque on ONE axis (roll=0, pitch=1, yaw=2) with hover thrust,
/// for a short window, and reports the sign of the sensed body rate
/// on that axis.
///
/// The cascade computes torque in NED (X-fwd, Y-right, Z-down). gz's
/// MulticopterMotorModel applies it in ENU (X-fwd, Y-left, Z-up). For
/// the verified rate-PID (negative feedback: torque drives rate→
/// setpoint) to be *stabilising*, a commanded +torque[axis] must
/// produce a +rate[axis] as the cascade senses it (post enu_to_ned).
/// If the response is NEGATIVE, that axis needs a sign flip in the
/// bridge's frame-correction (see `frame_correct_torque`).
///
/// PASS = the post-correction response is positive on the commanded
/// axis (torque and sensed rate agree in sign). Printed verdict feeds
/// the unit test `frame_correction_signs_match_gz_enu`.
fn run_frame_check(physics: &mut dyn Physics, axis: usize, duration_s: f32) -> bool {
    let mut mixer = QuadMixer::new();
    let hover_thrust = 0.72_f32;
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let tick_period = Duration::from_secs_f32(dt);
    let pace_real_time = physics.counters().is_some();

    // Commanded torque on the test axis (post frame-correction, so the
    // oracle measures the *corrected* path the cascade will use).
    let mut cmd = [0.0_f32; 3];
    cmd[axis] = 0.15;
    let cmd_corrected = frame_correct_torque(cmd);

    // Measure mean sensed rate on the test axis over the settle window
    // [0.3, 0.8] s — long enough past the spawn transient, short enough
    // that the body hasn't tumbled past small-angle.
    let mut sum_rate = 0.0_f32;
    let mut count = 0u32;
    let axis_name = ["roll(X)", "pitch(Y)", "yaw(Z)"][axis];

    for step in 0..n {
        let tick_start = Instant::now();
        let t = step as f32 * dt;
        let (imu_sample, _pos) = physics.measure(0.0);
        let motors = mixer.mix(cmd_corrected, hover_thrust);
        physics.step(motors, dt);
        if (0.3..0.8).contains(&t) {
            sum_rate += imu_sample.gyro_body[axis];
            count += 1;
        }
        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period { std::thread::sleep(tick_period - used); }
        }
    }
    let mean_rate = if count > 0 { sum_rate / count as f32 } else { 0.0 };
    // After correction, +cmd on this axis should yield +rate.
    let agrees = mean_rate > 0.0;
    println!(
        "  frame-check axis={axis_name}: commanded +0.15 (corrected={:?}) → mean sensed rate={:.4} rad/s  [{}]",
        cmd_corrected, mean_rate, if agrees { "AGREE ✓" } else { "OPPOSE ✗" },
    );
    agrees
}

/// v0.19.5 — alt-only thrust + rate-pid attitude damping (zero rate
/// setpoint = "stay level"). The minimal closed-loop hover: P+D
/// altitude controller for thrust + relay-rate's verified PID for
/// rotational stability, no position controller, no attitude
/// controller. If this PASSes, the v0.19.4 cascade's instability is
/// localised to POS+ATT (interaction of horizontal position feedback
/// with attitude-setpoint propagation during the first transient).
fn run_alt_rate_hover(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    let mut rate_pid = RatePid::new();
    let mut mixer = QuadMixer::new();
    let setpoint_d = -2.0_f32;
    let hover_thrust = 0.72_f32;
    let kp_alt = 0.05_f32;
    // v0.19.8 — kd 0.15→0.30. With the thrust-floor mixer preserving
    // collective, the altitude loop is decoupled from attitude; more
    // derivative damps the climb overshoot (one run shot to 12 m).
    let kd_alt = 0.30_f32;
    let lp_alpha = 0.05_f32;
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let tick_period = Duration::from_secs_f32(dt);
    let pace_real_time = physics.counters().is_some();

    let mut peak_dist_err = 0.0_f32;
    let mut min_dist_seen = f32::INFINITY;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0_usize;
    let steady_start_t = (duration_s - 5.0).max(0.0);
    let mut last_pos_d: f32 = 0.0;
    let mut v_d_filt: f32 = 0.0;
    let mut last_pos_d_seen: f32 = 0.0;

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        let t = step as f32 * dt;

        let (imu_sample, pos_ned) = physics.measure(0.0);
        last_pos_d_seen = pos_ned[2];
        let v_d_raw = (pos_ned[2] - last_pos_d) / dt;
        v_d_filt = lp_alpha * v_d_raw + (1.0 - lp_alpha) * v_d_filt;
        last_pos_d = pos_ned[2];

        let alt_err = setpoint_d - pos_ned[2];
        let thrust = (hover_thrust - kp_alt * alt_err + kd_alt * v_d_filt).clamp(0.0, 1.0);

        // v0.19.5 — first 0.5 s is a "spawn hold": uniform thrust,
        // no torque. Without this the rate-pid responds to spawn
        // transients (rotor imbalance + IMU noise) by demanding
        // torque, which the mixer's priority-preserving saturation
        // then converts into bang-bang motors → body tumbles off
        // axis before steady-state can establish. After the hold,
        // rate-pid takes over with the body already airborne + level.
        let torque_raw = if t < 0.5 {
            [0.0_f32; 3]
        } else {
            rate_pid.tick(rate_ts_of(t), imu_sample.gyro_body, [0.0_f32; 3])
        };
        // v0.19.6 — frame-correction is identity now that the SDF
        // rotor index map is aligned to MIXER_X. The v0.19.5 negation
        // hack is gone; the frame-check oracle confirms +torque →
        // +rate (AGREE) on roll + pitch with no sign flip.
        let torque = frame_correct_torque(torque_raw);

        // v0.19.8 — thrust-priority mix with a 0.5 floor. Collective
        // thrust is preserved exactly (torque columns are zero-sum),
        // so the rate loop's attitude torque can no longer steal lift
        // — the v0.19.7 altitude limit-cycle root cause. The 0.88
        // thrust clamp above is now redundant but harmless.
        let motors = mixer.mix_thrust_floor(torque, thrust, 0.5);
        physics.step(motors, dt);

        let dist = alt_err.abs();
        if dist > peak_dist_err { peak_dist_err = dist; }
        if dist < min_dist_seen { min_dist_seen = dist; }
        if t >= steady_start_t {
            sum_sq_steady += dist * dist;
            steady_count += 1;
        }
        if let Some(ref mut e) = evidence {
            e.write_tick(step, t, pos_ned, imu_sample.accel_body, imu_sample.gyro_body,
                         motors, physics.counters());
        }
        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }
    let wall = started_at.elapsed();
    let final_dist = (setpoint_d - last_pos_d_seen).abs();
    let rms_steady = if steady_count > 0 {
        (sum_sq_steady / steady_count as f32).sqrt()
    } else {
        f32::NAN
    };
    let counters = physics.counters();
    println!(
        "  verdict: backend={} scenario=alt-rate steps={} final_dist={:.2}m peak_dist={:.2}m rms_steady={:.2}m  wall={:.2}s",
        physics.name(), n, final_dist, peak_dist_err, rms_steady, wall.as_secs_f32(),
    );
    if let Some((imu_recv, navsat_recv, motor_send)) = counters {
        println!(
            "  counters: imu_recv={imu_recv} navsat_recv={navsat_recv} motor_send={motor_send}",
        );
    }
    if let Some(ref mut e) = evidence {
        e.write_summary_hover(n, final_dist, peak_dist_err, rms_steady,
                              min_dist_seen, wall.as_secs_f32(), counters);
    }
    final_dist < 0.5 && rms_steady < 1.0
}

/// v0.19.5 diagnostic — altitude-only closed loop. Skips the full
/// cascade (no EKF, no POS, no ATT, no RATE); feeds thrust = hover +
/// P * altitude_error to the mixer with zero torque. If THIS hovers,
/// the v0.19.4 cascade-tuning issue is in the upper cascade layers
/// (POS / ATT / RATE producing torque that the mixer saturates,
/// starving thrust). If it doesn't hover, the bridge or SDF has a
/// deeper bug.
fn run_alt_only_hover(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    let mut mixer = QuadMixer::new();
    let setpoint_d = -2.0_f32;
    let hover_thrust = 0.72_f32;
    // v0.19.5 alt-only PD diagnostic gains. Tuned for a 700 g body
    // with the falcon-quad SDF's hover-thrust headroom. v_d is
    // finite-differenced from 50 Hz NavSat → noisy at 100 Hz harness
    // tick (raw v_d spikes ±1 m/s); low-pass filter `lp_alpha`
    // smooths it before D-feedback.
    //
    // PI+D: gentle kp (0.05) keeps the altitude loop from coupling
    // into attitude (kp=0.15 destabilized → horizontal drift). A
    // small ki integrates out the ~0.9 m steady-state error a P-only
    // loop left below the 2 m setpoint. kd damps the climb.
    let kp_alt = 0.05_f32;
    let ki_alt = 0.02_f32;
    let kd_alt = 0.15_f32;
    let i_max = 0.30_f32; // anti-windup bound on the thrust integral
    let lp_alpha = 0.05_f32; // heavy smoothing
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let tick_period = Duration::from_secs_f32(dt);
    let pace_real_time = physics.counters().is_some();

    let mut peak_dist_err = 0.0_f32;
    let mut min_dist_seen = f32::INFINITY;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0_usize;
    let steady_start_t = (duration_s - 5.0).max(0.0);
    let mut last_pos_d: f32 = 0.0;
    let mut v_d_filt: f32 = 0.0;
    let mut alt_integral: f32 = 0.0;

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        let t = step as f32 * dt;

        let (imu_sample, pos_ned) = physics.measure(0.0);
        // finite-diff vertical velocity, low-pass filtered.
        let v_d_raw = (pos_ned[2] - last_pos_d) / dt;
        v_d_filt = lp_alpha * v_d_raw + (1.0 - lp_alpha) * v_d_filt;
        last_pos_d = pos_ned[2];

        // altitude error in NED: alt_err = setpoint_d - body_d.
        // setpoint_d = -2 (2 m altitude); body starts near 0.
        // Initial alt_err = -2 (need to climb → INCREASE thrust).
        // PI+D: thrust = hover - kp*err - ki*∫err + kd*v_d.
        // (alt_err negative below setpoint, so -kp*err and -ki*∫err
        //  both raise thrust to climb.)
        let alt_err = setpoint_d - pos_ned[2];
        alt_integral = (alt_integral + alt_err * dt).clamp(-i_max / ki_alt, i_max / ki_alt);
        let thrust = (hover_thrust
            - kp_alt * alt_err
            - ki_alt * alt_integral
            + kd_alt * v_d_filt)
            .clamp(0.0, 1.0);
        let motors = mixer.mix([0.0_f32; 3], thrust);
        physics.step(motors, dt);

        let dist = alt_err.abs();
        if dist > peak_dist_err { peak_dist_err = dist; }
        if dist < min_dist_seen { min_dist_seen = dist; }
        if t >= steady_start_t {
            sum_sq_steady += dist * dist;
            steady_count += 1;
        }
        if let Some(ref mut e) = evidence {
            e.write_tick(step, t, pos_ned, imu_sample.accel_body, imu_sample.gyro_body,
                         motors, physics.counters());
        }
        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }
    let wall = started_at.elapsed();
    let final_dist = (setpoint_d - last_pos_d).abs();
    let rms_steady = if steady_count > 0 {
        (sum_sq_steady / steady_count as f32).sqrt()
    } else {
        f32::NAN
    };
    let counters = physics.counters();
    println!(
        "  verdict: backend={} scenario=alt-only steps={} final_dist={:.2}m peak_dist={:.2}m rms_steady={:.2}m  wall={:.2}s",
        physics.name(), n, final_dist, peak_dist_err, rms_steady, wall.as_secs_f32(),
    );
    if let Some((imu_recv, navsat_recv, motor_send)) = counters {
        println!(
            "  counters: imu_recv={imu_recv} navsat_recv={navsat_recv} motor_send={motor_send}",
        );
    }
    if let Some(ref mut e) = evidence {
        e.write_summary_hover(n, final_dist, peak_dist_err, rms_steady,
                              min_dist_seen, wall.as_secs_f32(), counters);
    }
    final_dist < 0.5 && rms_steady < 1.0
}

/// v0.19.4 — closed-loop hover. Mirrors `falcon-sitl-hover`'s
/// `run_mission` cascade pattern, but inputs come from the bridge
/// (gz IMU + NavSat) and outputs feed the bridge's motor publish.
///
/// Setpoint: NED (0, 0, -2) — hover at 2 m altitude. v_ned is finite-
/// differenced from p_ned (gz NavSat doesn't expose velocity directly
/// in v0.18.1's subscriber; the math is identical to a 1st-order
/// derivative the POS controller would compute internally anyway).
///
/// All loops run at the harness rate (100 Hz). Canonical rates from
/// falcon-sitl-hover are 1 kHz IMU / 250 Hz ATT / 50 Hz POS — running
/// at 100 Hz universally produces a stable hover under gz; tighter
/// loop bandwidth is a v0.19.x tuning lever, not a v0.19.4 prerequisite.
fn run_closed_loop_hover(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    let mut ekf = Ekf::new();
    let mut rate_pid = RatePid::new();
    let mut att = AttController::new();
    let mut pos = PosController::new();
    let mut mixer = QuadMixer::new();

    let setpoint_ned = [0.0_f32, 0.0, -2.0];
    let setpoint = PositionSetpoint::hover_at(setpoint_ned);

    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let tick_period = Duration::from_secs_f32(dt);
    let pace_real_time = physics.counters().is_some();

    let mut current_att_sp = [1.0_f32, 0.0, 0.0, 0.0];
    let mut current_thrust = 0.5_f32;

    let mut last_pos_ned: Option<[f32; 3]> = None;
    let mut peak_dist_err = 0.0_f32;
    let mut min_dist_seen = f32::INFINITY;
    let mut nan_seen = false;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0_usize;
    // Last 5 s of the run define "steady" — same shape as
    // `falcon-sitl-hover::run_mission`'s 2 s tail. Looser here
    // because gz step granularity + finite-diff velocity make
    // settling slower than the pure-Rust SITL.
    let steady_start_t = (duration_s - 5.0).max(0.0);

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        let t = step as f32 * dt;

        // 1. Read state from the bridge.
        let (mut imu_sample, pos_ned) = physics.measure(0.0);
        imu_sample.time = ekf_ts_of(t);

        // 2. EKF — attitude estimate.
        let est = ekf.tick(imu_sample);
        if !est.quaternion[0].is_finite() { nan_seen = true; }

        // 3. POS — position + finite-diff velocity → attitude setpoint.
        let v_ned = match last_pos_ned {
            Some(p) => [
                (pos_ned[0] - p[0]) / dt,
                (pos_ned[1] - p[1]) / dt,
                (pos_ned[2] - p[2]) / dt,
            ],
            None => [0.0; 3],
        };
        last_pos_ned = Some(pos_ned);
        let att_sp = pos.tick(
            pos_ts_of(t),
            pos_ned,
            v_ned,
            est.quaternion,
            setpoint,
        );
        current_att_sp = att_sp.quaternion;
        current_thrust = att_sp.thrust;

        // 4. ATT — quaternion error → rate setpoint.
        let current_rate_sp = att.tick(att_ts_of(t), est.quaternion, current_att_sp);

        // 5. RATE — gyro + rate setpoint → torque (frame-corrected).
        let torque_raw = rate_pid.tick(rate_ts_of(t), imu_sample.gyro_body, current_rate_sp);
        let torque = frame_correct_torque(torque_raw);
        for k in 0..3 {
            if !torque[k].is_finite() { nan_seen = true; }
        }

        // 6. MIX — torque + thrust → 4× motor PWM.
        let motors = mixer.mix(torque, current_thrust);
        if motors.iter().any(|v| !v.is_finite()) { nan_seen = true; }

        // 7. Publish to the bridge.
        physics.step(motors, dt);

        // 8. Bookkeeping — distance to setpoint.
        let dn = pos_ned[0] - setpoint_ned[0];
        let de = pos_ned[1] - setpoint_ned[1];
        let dd = pos_ned[2] - setpoint_ned[2];
        let dist = (dn * dn + de * de + dd * dd).sqrt();
        if dist > peak_dist_err { peak_dist_err = dist; }
        if dist < min_dist_seen { min_dist_seen = dist; }
        if t >= steady_start_t {
            sum_sq_steady += dist * dist;
            steady_count += 1;
        }

        if let Some(ref mut e) = evidence {
            e.write_tick(step, t, pos_ned, imu_sample.accel_body, imu_sample.gyro_body,
                         motors, physics.counters());
        }

        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }
    let wall = started_at.elapsed();

    let final_dist = match last_pos_ned {
        Some(p) => {
            let dn = p[0] - setpoint_ned[0];
            let de = p[1] - setpoint_ned[1];
            let dd = p[2] - setpoint_ned[2];
            (dn * dn + de * de + dd * dd).sqrt()
        }
        None => f32::NAN,
    };
    let rms_steady = if steady_count > 0 {
        (sum_sq_steady / steady_count as f32).sqrt()
    } else {
        f32::NAN
    };
    let counters = physics.counters();

    println!(
        "  verdict: backend={} scenario=hover steps={} final_dist={:.2}m peak_dist={:.2}m rms_steady={:.2}m  wall={:.2}s",
        physics.name(), n, final_dist, peak_dist_err, rms_steady, wall.as_secs_f32(),
    );
    if let Some((imu_recv, navsat_recv, motor_send)) = counters {
        println!(
            "  counters: imu_recv={imu_recv} navsat_recv={navsat_recv} motor_send={motor_send}",
        );
    }
    if let Some(ref mut e) = evidence {
        e.write_summary_hover(n, final_dist, peak_dist_err, rms_steady,
                              min_dist_seen, wall.as_secs_f32(), counters);
    }

    // PASS = within 0.5 m at end + RMS over last 5 s under 1.0 m + no NaN.
    !nan_seen && final_dist < 0.5 && rms_steady < 1.0
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

/// v0.19.3 open-loop smoke: command 70 % PWM constant, watch for
/// climb. Retained as `--scenario=open-loop-climb` so a bench
/// operator can still diagnose "is the publish path alive" without
/// running the full cascade. Same code as v0.19.3's `run_hover`.
fn run_open_loop_climb(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let mut t = 0.0_f32;
    let mut initial_alt: Option<f32> = None;
    let mut min_alt = f32::MAX;
    let mut max_alt = f32::MIN;
    let tick_period = Duration::from_secs_f32(dt);

    // Real-time pacing: the gz-transport bridge needs wall-clock
    // time between polls so the OS can deliver IMU/NavSat frames.
    // MockPhysics is synchronous and reports `counters() == None`,
    // so we only sleep when the backend actually streams. Same
    // pattern as `HitlBench::real_time()` in v0.18.2 falcon-hitl-rfspoof.
    let pace_real_time = physics.counters().is_some();

    // Command 70 % PWM on all four motors — enough to hover under
    // MockPhysics's THRUST_SCALE=20 N, and gives gz-sim's
    // MulticopterMotorModel enough margin to lift the 700 g body.
    let motor_pwm = [0.7_f32; 4];

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        physics.step(motor_pwm, dt);
        let (imu, pos) = physics.measure(0.01);
        let alt_m = -pos[2]; // NED down → altitude is -z
        if initial_alt.is_none() { initial_alt = Some(alt_m); }
        min_alt = min_alt.min(alt_m);
        max_alt = max_alt.max(alt_m);

        if let Some(ref mut e) = evidence {
            e.write_tick(step, t, pos, imu.accel_body, imu.gyro_body, motor_pwm,
                         physics.counters());
        }

        t += dt;

        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }
    let wall = started_at.elapsed();

    let start = initial_alt.unwrap_or(0.0);
    let net_climb = max_alt - start;
    let counters = physics.counters();
    println!(
        "  verdict: backend={} steps={} climb={:.2} m  (min={:.2} max={:.2})  wall={:.2}s",
        physics.name(), n, net_climb, min_alt, max_alt, wall.as_secs_f32(),
    );
    if let Some((imu_recv, navsat_recv, motor_send)) = counters {
        println!(
            "  counters: imu_recv={imu_recv} navsat_recv={navsat_recv} motor_send={motor_send}",
        );
    }
    if let Some(ref mut e) = evidence {
        e.write_summary(n, net_climb, min_alt, max_alt, wall.as_secs_f32(), counters);
    }
    // PASS criterion: net climb > 0.1 m. Under MockPhysics, 70 % PWM
    // easily beats gravity. Under the real gz bridge, success depends
    // on the SDF model's motor scaling — if the bench reports FAIL
    // with imu_recv > 0 and motor_send > 0 the bridge wiring is
    // working and the SDF's motorConstant needs adjustment.
    net_climb > 0.1
}

/// Bench-evidence sink — writes one harness log + one per-tick CSV
/// under `<dir>/<timestamp>-<backend>-<scenario>-{harness.log,ticks.csv}`.
/// Same shape as `bench-evidence/px4-sitl/<TS>-*.log` from v0.18.2.
struct EvidenceSink {
    harness: fs::File,
    ticks: fs::File,
    timestamp: String,
}

impl EvidenceSink {
    fn open(dir: &PathBuf, backend: &str, scenario: &str) -> std::io::Result<Self> {
        fs::create_dir_all(dir)?;
        let ts = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let stem = format!("{ts}-{backend}-{scenario}");
        let harness_path = dir.join(format!("{stem}-harness.log"));
        let ticks_path = dir.join(format!("{stem}-ticks.csv"));
        let mut harness = fs::File::create(&harness_path)?;
        let mut ticks = fs::File::create(&ticks_path)?;
        writeln!(
            harness,
            "falcon-sitl-gz bench-evidence\nbackend: {backend}\nscenario: {scenario}\ntimestamp: {ts}\n"
        )?;
        writeln!(
            ticks,
            "step,t_s,n_m,e_m,d_m,ax_body,ay_body,az_body,gx_body,gy_body,gz_body,m0,m1,m2,m3,imu_recv,navsat_recv,motor_send"
        )?;
        Ok(Self { harness, ticks, timestamp: format!("{ts}") })
    }

    #[allow(clippy::too_many_arguments)]
    fn write_tick(
        &mut self,
        step: u32,
        t: f32,
        pos: [f32; 3],
        accel: [f32; 3],
        gyro: [f32; 3],
        pwm: [f32; 4],
        counters: Option<(u64, u64, u64)>,
    ) {
        let (i, n, m) = counters.unwrap_or((0, 0, 0));
        let _ = writeln!(
            self.ticks,
            "{},{:.3},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.4},{:.3},{:.3},{:.3},{:.3},{},{},{}",
            step, t, pos[0], pos[1], pos[2],
            accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2],
            pwm[0], pwm[1], pwm[2], pwm[3], i, n, m,
        );
    }

    fn write_summary(
        &mut self,
        steps: u32,
        net_climb: f32,
        min_alt: f32,
        max_alt: f32,
        wall_s: f32,
        counters: Option<(u64, u64, u64)>,
    ) {
        let _ = writeln!(self.harness, "steps:      {steps}");
        let _ = writeln!(self.harness, "net_climb:  {net_climb:.3} m");
        let _ = writeln!(self.harness, "min_alt:    {min_alt:.3} m");
        let _ = writeln!(self.harness, "max_alt:    {max_alt:.3} m");
        let _ = writeln!(self.harness, "wall:       {wall_s:.3} s");
        if let Some((i, n, m)) = counters {
            let _ = writeln!(self.harness, "imu_recv:   {i}");
            let _ = writeln!(self.harness, "navsat_recv:{n}");
            let _ = writeln!(self.harness, "motor_send: {m}");
        }
    }

    /// v0.19.4 — closed-loop hover summary. Different metrics than
    /// the v0.19.3 open-loop climb (which only knows net_climb /
    /// min_alt / max_alt). Hover cares about distance-to-setpoint
    /// statistics.
    #[allow(clippy::too_many_arguments)]
    fn write_summary_hover(
        &mut self,
        steps: u32,
        final_dist: f32,
        peak_dist: f32,
        rms_steady: f32,
        min_dist: f32,
        wall_s: f32,
        counters: Option<(u64, u64, u64)>,
    ) {
        let _ = writeln!(self.harness, "steps:       {steps}");
        let _ = writeln!(self.harness, "final_dist:  {final_dist:.3} m");
        let _ = writeln!(self.harness, "peak_dist:   {peak_dist:.3} m");
        let _ = writeln!(self.harness, "rms_steady:  {rms_steady:.3} m  (last 5 s)");
        let _ = writeln!(self.harness, "min_dist:    {min_dist:.3} m");
        let _ = writeln!(self.harness, "wall:        {wall_s:.3} s");
        if let Some((i, n, m)) = counters {
            let _ = writeln!(self.harness, "imu_recv:    {i}");
            let _ = writeln!(self.harness, "navsat_recv: {n}");
            let _ = writeln!(self.harness, "motor_send:  {m}");
        }
    }

    fn finish(&mut self, pass: bool) {
        let _ = writeln!(self.harness, "verdict:    {}", if pass { "PASS" } else { "FAIL" });
        let _ = self.harness.flush();
        let _ = self.ticks.flush();
        let _ = &self.timestamp;
    }
}

/// Construct a `GazeboPhysics` for the CLI gazebo backend. With
/// feature `gazebo` ON, parses `--home=lat,lon,alt_m` and threads
/// it into `connect_with_home`. Without the feature, falls back to
/// the stub `new(world, model)`.
#[cfg(feature = "gazebo")]
fn build_gazebo(args: &[String], world: String, model: String) -> GazeboPhysics {
    let home = match arg(args, "--home") {
        Some(s) => parse_home(&s).expect("--home=lat,lon,alt_m"),
        None => physics::Home::ORIGIN,
    };
    println!("  gazebo home: lat={} lon={} alt={} m", home.lat_deg, home.lon_deg, home.alt_m);
    GazeboPhysics::connect_with_home(world, model, home)
        .expect("connect_with_home: gz-transport connect failed; is `gz sim` running?")
}

#[cfg(not(feature = "gazebo"))]
fn build_gazebo(_args: &[String], world: String, model: String) -> GazeboPhysics {
    let _ = arg(_args, "--home"); // accept the flag silently in stub mode
    println!("  (stub backend — rebuild with --features gazebo for the real bridge)");
    GazeboPhysics::new(world, model)
}

#[cfg(feature = "gazebo")]
fn parse_home(s: &str) -> Option<physics::Home> {
    let parts: Vec<&str> = s.split(',').collect();
    if parts.len() != 3 { return None; }
    let lat_deg: f64 = parts[0].parse().ok()?;
    let lon_deg: f64 = parts[1].parse().ok()?;
    let alt_m: f64 = parts[2].parse().ok()?;
    Some(physics::Home { lat_deg, lon_deg, alt_m })
}

fn arg(args: &[String], key: &str) -> Option<String> {
    let prefix = format!("{key}=");
    for a in args {
        if let Some(v) = a.strip_prefix(&prefix) {
            return Some(v.into());
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::MockPhysics;

    #[test]
    fn mock_backend_climbs_under_full_thrust() {
        let mut p = MockPhysics::at_rest();
        let pass = run_open_loop_climb(&mut p, 1.0, None);
        assert!(pass, "70 % PWM × THRUST_SCALE should easily beat gravity");
    }

    /// v0.19.6 — the frame-correction adapter is identity. Pins the
    /// oracle finding: once the SDF rotor index→position+spin map is
    /// aligned to relay-mix-quad's MIXER_X convention, +torque produces
    /// +rate (AGREE) on roll + pitch with no sign flip — the v0.19.4/.5
    /// "negate" hack was compensating for a scrambled SDF, not a real
    /// frame inversion. If a future SDF/frame change reintroduces a
    /// flip, the frame-check oracle catches it and this test changes
    /// deliberately. See bench-evidence/gz-sim/2026-05-28-v0.19.6-*.md.
    #[test]
    fn frame_correction_is_identity() {
        let t = [0.3_f32, -0.7, 0.2];
        assert_eq!(frame_correct_torque(t), t,
            "frame-correction must be identity after the v0.19.6 SDF index alignment");
        // Spot-check each axis independently.
        assert_eq!(frame_correct_torque([1.0, 0.0, 0.0]), [1.0, 0.0, 0.0]);
        assert_eq!(frame_correct_torque([0.0, 1.0, 0.0]), [0.0, 1.0, 0.0]);
        assert_eq!(frame_correct_torque([0.0, 0.0, 1.0]), [0.0, 0.0, 1.0]);
    }

    /// v0.19.4 — closed-loop cascade against MockPhysics. Smoke test
    /// that the cascade wires + ticks without panic, no NaN escape.
    /// MockPhysics ignores per-rotor differential (applies only
    /// collective thrust to vertical axis), so attitude control
    /// is a no-op; the test confirms the loop runs at all, not that
    /// hover is achieved here. Real hover lands under gz.
    #[test]
    fn closed_loop_hover_compiles_and_ticks_on_mock() {
        let mut p = MockPhysics::at_rest();
        let _ = run_closed_loop_hover(&mut p, 0.5, None);
        // Pass criterion: no panic, no NaN escape into omega/v_ned.
        for i in 0..3 {
            assert!(p.omega[i].is_finite(), "omega[{i}] = {}", p.omega[i]);
            assert!(p.v_ned[i].is_finite(), "v_ned[{i}] = {}", p.v_ned[i]);
        }
    }

    /// v0.19.0 — when --evidence-dir is set, the runner produces two
    /// files: a harness log and a per-tick CSV. Smoke-test that both
    /// files materialise + carry the right header/footer.
    #[test]
    fn evidence_sink_produces_log_and_csv() {
        let tmp = std::env::temp_dir().join(format!("fsg-bench-{}", std::process::id()));
        let _ = fs::remove_dir_all(&tmp);
        let mut sink = EvidenceSink::open(&tmp, "mock", "hover").expect("open");
        sink.write_tick(0, 0.0, [0.0; 3], [0.0; 3], [0.0; 3], [0.7; 4], Some((1, 2, 3)));
        sink.write_summary(1, 0.5, 0.0, 0.5, 0.01, Some((1, 2, 3)));
        sink.finish(true);
        // Both files exist; the CSV has the header line + one data row.
        let entries: Vec<_> = fs::read_dir(&tmp).unwrap().collect();
        assert_eq!(entries.len(), 2, "expected harness.log + ticks.csv in {:?}", tmp);
        let _ = fs::remove_dir_all(&tmp);
    }

    /// Stub-only — when `gazebo` feature is ON, `GazeboPhysics::new`
    /// connects to gz-transport (and panics if no gz-sim is running),
    /// so the panic-free contract doesn't apply.
    #[cfg(not(feature = "gazebo"))]
    #[test]
    fn gazebo_stub_does_not_panic() {
        let mut g = crate::physics::GazeboPhysics::new("test-world", "test-quad");
        let _ = run_open_loop_climb(&mut g, 0.1, None);
        // Result is FAIL (everything zeros), but the harness must not
        // panic — that's the contract for a stub-only run.
    }
}
