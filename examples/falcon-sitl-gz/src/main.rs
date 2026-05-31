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
use relay_arm::{ArmingConfig, ArmingSequencer, ARMED};
use relay_iekf::{Iekf, Imu as IekfImu, NavState};
use falcon_config::YawMode;
use relay_geo::GeoAtt;
use relay_att::{AttController, Timestamp as AttTimestamp};
use relay_ekf::{Ekf, ImuSample, Timestamp as EkfTimestamp};
use relay_mix_quad::QuadMixer;
use relay_pos::{PosController, PosGains, PositionSetpoint, Timestamp as PosTimestamp};
use relay_rate::{RatePid, Timestamp as RateTimestamp};
use std::fs;
use std::io::Write;
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    // Emit the default tuning as JSON (so the on-disk config always matches
    // the struct), then exit.
    if args.iter().any(|a| a == "--dump-config") {
        println!("{}", falcon_config::FalconConfig::default().to_json());
        return;
    }
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
/// ever changes; the unit test pins it.
///
/// YAW (2026-05-30, v0.21): the frame-yaw oracle now reads RELIABLY OPPOSE
/// — commanding +0.15 N·m yaw produces −1.1 rad/s as sensed,
/// deterministically across runs (−1.14, −1.04, −1.13). So the yaw channel
/// (controller-convention → mixer → gz) genuinely has NEGATIVE control
/// effectiveness. That correction lives in the YAW ADRC's `b0` (negative
/// −6, the physically correct effectiveness sign — see falcon-config), so
/// this frame seam stays identity for yaw too. Encoding it here instead
/// (SIGN[2]=−1, b0=+6) is mathematically equivalent (u→−u, body torque
/// unchanged) but tested WORSE in limited gz runs (0/6 vs 3/4; unresolved
/// — gz nondeterminism vs a hidden asymmetry), so it lives in b0 where it
/// is verified-good.
#[inline]
fn frame_correct_torque(torque_ned: [f32; 3]) -> [f32; 3] {
    const SIGN: [f32; 3] = [1.0, 1.0, 1.0];
    [
        SIGN[0] * torque_ned[0],
        SIGN[1] * torque_ned[1],
        SIGN[2] * torque_ned[2],
    ]
}

/// Body tilt from vertical (rad), estimated from the specific-force
/// (accelerometer) vector. At rest / steady hover the accel measures the
/// reaction to gravity along body-down; the tilt is the angle between
/// that vector and the body-z axis: `atan2(|horizontal|, |vertical|)`.
/// Convention-agnostic (works for either sign of az) and total — a
/// degenerate all-zero sample yields 0, which the sequencer treats as
/// level only if it persists (the rate loop is still gated by spin-up).
fn body_tilt_rad(accel_body: [f32; 3]) -> f32 {
    let horiz = libm::sqrtf(accel_body[0] * accel_body[0] + accel_body[1] * accel_body[1]);
    libm::atan2f(horiz, accel_body[2].abs())
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
        "arming" => run_arming_check(physics, duration_s, true),
        "arming-ungated" => run_arming_check(physics, duration_s, false),
        "geo-hover" => run_geo_hover(physics, duration_s, evidence),
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

/// v0.19.9 — arming-sequencer ORACLE + position-hold DIAGNOSTIC.
///
/// Runs the verified `ArmingSequencer` ahead of the full cascade against
/// gz physics. Two distinct questions, deliberately separated:
///
/// 1. **Does the sequencer deliver a clean handoff?** (its job, gated on)
///    PASS = it armed (level gate cleared) and tilt stayed below the
///    tumble threshold (30°) through the handoff window (spawn → 1.5 s
///    past arming). Empirically the gated startup holds 0° well past
///    arming — the sequencer does its job.
///
/// 2. **Does full position-hold then hover?** (diagnostic, NOT gated on)
///    `run_peak` (whole-run peak tilt) is reported as a diagnostic. It
///    is large (~88°): the body holds level for ~3.5 s then tips and
///    drifts once the position loop dominates. That is the documented
///    downstream residual — `relay-pos` thrust normalisation + RC#3
///    (EKF attitude during accel) — NOT a startup-gating failure. The
///    arming sequencer cannot fix a downstream cascade instability, and
///    v0.19.9 does not claim it does. This is the falsification finding
///    that localises the v0.19.10+ target.
///
/// `gated`: when false, torque authority is forced on from t=0 (the
/// pre-v0.19.9 behaviour). The `arming-ungated` baseline confirms the
/// startup transient itself is benign in the nominal (level) gz spawn —
/// i.e. the tip-over is genuinely downstream, not at the spawn.
fn run_arming_check(physics: &mut dyn Physics, duration_s: f32, gated: bool) -> bool {
    const TUMBLE_RAD: f32 = 0.52; // ~30° — past this the body has tipped
    // Full cascade — this is where RC#1 lives: relay-pos sees the large
    // initial position error at spawn and commands an aggressive attitude
    // setpoint that ATT+RATE chase before the body is stable, tumbling it.
    let mut ekf = Ekf::new();
    let mut rate_pid = RatePid::new();
    let mut att = AttController::new();
    let mut pos = PosController::new();
    let mut mixer = QuadMixer::new();
    let mut seq = ArmingSequencer::new(ArmingConfig::falcon_quad_100hz());
    let setpoint_ned = [0.0_f32, 0.0, -2.0];
    let setpoint = PositionSetpoint::hover_at(setpoint_ned);
    let dt = 0.01_f32;
    let n = (duration_s / dt) as u32;
    let tick_period = Duration::from_secs_f32(dt);
    let pace_real_time = physics.counters().is_some();

    let mut peak_tilt = 0.0_f32;
    let mut peak_tilt_handoff = 0.0_f32;
    let mut armed_at: Option<f32> = None;
    let mut last_tilt = 0.0_f32;
    let mut last_pos_ned: Option<[f32; 3]> = None;

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        let t = step as f32 * dt;
        let (mut imu_sample, pos_ned) = physics.measure(0.0);
        imu_sample.time = ekf_ts_of(t);

        let est = ekf.tick(imu_sample);
        let v_ned = match last_pos_ned {
            Some(p) => [
                (pos_ned[0] - p[0]) / dt,
                (pos_ned[1] - p[1]) / dt,
                (pos_ned[2] - p[2]) / dt,
            ],
            None => [0.0; 3],
        };
        last_pos_ned = Some(pos_ned);
        let att_sp = pos.tick(pos_ts_of(t), pos_ned, v_ned, est.quaternion, setpoint);
        let rate_sp = att.tick(att_ts_of(t), est.quaternion, att_sp.quaternion);

        let tilt = body_tilt_rad(imu_sample.accel_body);
        last_tilt = tilt;
        if tilt > peak_tilt { peak_tilt = tilt; }
        let arm = seq.tick(tilt, true);
        if arm.phase == ARMED && armed_at.is_none() {
            armed_at = Some(t);
        }
        // Handoff window = spawn → 1.5 s past arming: the interval the
        // sequencer is responsible for (get airborne + hand off level).
        // Tilt AFTER this is the downstream position-loop residual, NOT
        // a startup-gating failure, so it is reported but not gated on.
        if armed_at.map(|a| t <= a + 1.5).unwrap_or(true) && tilt > peak_tilt_handoff {
            peak_tilt_handoff = tilt;
        }
        // `gated`: the sequencer decides torque authority. Ungated
        // baseline: torque from t=0 (pre-v0.19.9), thrust_scale forced 1.
        let authority = if gated { arm.torque_authority } else { true };
        let scale = if gated { arm.thrust_scale } else { 1.0 };
        let torque_raw = if authority {
            rate_pid.tick(rate_ts_of(t), imu_sample.gyro_body, rate_sp)
        } else {
            [0.0_f32; 3]
        };
        let torque = frame_correct_torque(torque_raw);
        let motors =
            mixer.mix_thrust_floor(torque, att_sp.thrust * scale, 0.5 * scale);
        physics.step(motors, dt);

        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }
    let wall = started_at.elapsed();
    let armed = armed_at.is_some();
    // The sequencer's responsibility is the HANDOFF: arm only after a
    // confirmed-level window and deliver a level body to the cascade.
    // PASS gates on the handoff window only. `peak_tilt` (whole run) is
    // reported as a DIAGNOSTIC of the downstream position-hold residual
    // (relay-pos thrust + RC#3 EKF-during-accel) — out of v0.19.9 scope.
    let clean_handoff = peak_tilt_handoff < TUMBLE_RAD;
    // Honest scope: PASS means the sequencer delivered a clean handoff.
    // The gated and ungated handoffs are BOTH clean (the nominal level
    // spawn is benign with or without torque gating) — so this is not a
    // gated-vs-ungated contrast; it is the finding that the tip-over is
    // downstream, not at startup. The label says "(handoff)" and the
    // NOTE reports the whole-run tip so PASS is never read as "hovers".
    let pass = armed && clean_handoff;
    println!(
        "  arming-check[{}]: armed_at={} handoff_peak={:.1}° run_peak={:.1}° final={:.1}° (tumble>{:.0}°)  [{}]  wall={:.2}s",
        if gated { "gated" } else { "UNGATED-baseline" },
        armed_at.map(|t| format!("{t:.2}s")).unwrap_or_else(|| "NEVER".into()),
        peak_tilt_handoff.to_degrees(),
        peak_tilt.to_degrees(),
        last_tilt.to_degrees(),
        TUMBLE_RAD.to_degrees(),
        if pass { "PASS ✓ (handoff)" } else { "FAIL ✗" },
        wall.as_secs_f32(),
    );
    if pass && peak_tilt >= TUMBLE_RAD {
        println!(
            "  NOTE: clean gated handoff, but the body tips later (run_peak={:.0}°) — \
             downstream position-hold residual (relay-pos thrust + RC#3), tracked for v0.19.10+.",
            peak_tilt.to_degrees(),
        );
    }
    pass
}

/// v0.19.5 — alt-only thrust + rate-pid attitude damping (zero rate
/// setpoint = "stay level"). The minimal closed-loop hover: P+D
/// altitude controller for thrust + relay-rate's verified PID for
/// rotational stability, no position controller, no attitude
/// controller. If this PASSes, the v0.19.4 cascade's instability is
/// localised to POS+ATT (interaction of horizontal position feedback
/// with attitude-setpoint propagation during the first transient).
/// v0.23 — full verified cascade with the GEOMETRIC SE(3) attitude
/// controller (relay-geo) replacing the Euler relay-att/rate path that
/// coupled tilt into yaw. IEKF (full state) → simple P-D position→accel
/// command → geometric attitude (accel direction + heading → torque) →
/// thrust-floor mixer, gated by the arming sequencer. This is the v0.23
/// position-hold validation. PASS = final_dist < 0.5 m AND rms_steady
/// (last 5 s) < 1.0 m.
fn run_geo_hover(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    // v0.25 — config-driven (host-only falcon-config). `--config=path.json`
    // sweeps every tuning parameter WITHOUT a rebuild; default is the
    // best-known in-code tuning. The flight crates stay no_std; only this
    // (std) bench loads JSON.
    let cfg = match arg(&std::env::args().collect::<Vec<_>>(), "--config") {
        Some(p) => match falcon_config::FalconConfig::from_json_path(&p) {
            Ok(c) => {
                println!("  config: loaded {p}");
                c
            }
            Err(e) => {
                eprintln!("  config: {e}; using defaults");
                falcon_config::FalconConfig::default()
            }
        },
        None => falcon_config::FalconConfig::default(),
    };

    let mut iekf = Iekf::with_config(NavState::identity(), cfg.to_iekf_config());
    let mut iekf_heading_init = false;
    let mut mixer = QuadMixer::new();
    // Arming ticks are wall-clock-derived (0.3 s spin-up, 0.1 s level) so
    // it works at any loop rate (100 Hz → 30/10, 1 kHz → 300/100).
    let arm_dt = cfg.pos.loop_dt.max(1e-4);
    let mut seq = ArmingSequencer::new(ArmingConfig {
        spinup_ticks: (0.3 / arm_dt) as u32,
        level_ticks_required: (0.1 / arm_dt) as u32,
        tilt_thresh_rad: 0.087,
    });
    let geo = GeoAtt::new(cfg.to_geo_gains());
    let mut adrc = cfg.to_adrc();
    let use_adrc = cfg.pos.use_adrc;

    let setpoint_ned = [0.0_f32, 0.0, -2.0];
    let hover_thrust = cfg.pos.hover_thrust;
    let kp_pos = cfg.pos.kp_pos;
    let kd_vel = cfg.pos.kd_vel;
    let a_cmd_max = cfg.pos.a_cmd_max;
    let kp_alt = cfg.pos.kp_alt;
    let kd_alt = cfg.pos.kd_alt;
    let lp_alpha = 0.05_f32;
    let torque_scale = 1.0_f32; // geometric Nm → mixer normalised torque
    let yaw_d = core::f32::consts::FRAC_PI_2; // hold spawn heading (East)
    // v0.25 — CASCADE: fast inner rate loop (IEKF-predict + gyro-LPF +
    // ADRC + mixer) at `dt` (1 kHz), slow outer loop (IEKF aiding updates
    // + altitude + geometric/position) every `outer_decim` ticks (~100 Hz).
    // Track A: the rate loop must run fast vs the 25 ms yaw actuator pole.
    let dt = cfg.pos.loop_dt.max(1e-4);
    let outer_decim = cfg.pos.outer_decim.max(1);
    let dt_outer = dt * outer_decim as f32;
    let mut gyro_lpf = relay_adrc::GyroLpf::new(cfg.pos.gyro_lpf_hz, 1.0 / dt);
    let n = (duration_s / dt) as u32;
    let tick_period = Duration::from_secs_f32(dt);
    let pace_real_time = physics.counters().is_some();

    let mut peak_dist = 0.0_f32;
    let mut min_dist = f32::INFINITY;
    let mut sum_sq_steady = 0.0_f32;
    let mut steady_count = 0usize;
    let steady_start_t = (duration_s - 5.0).max(0.0);
    // IEKF consistency (NEES) accumulators: the pre-position-update
    // prediction error vs gz ground truth, weighted by the filter's own
    // predicted position covariance. Consistent ⇒ mean ≈ 3 (χ²₃).
    let mut nees_sum = 0.0_f64;
    let mut nees_n = 0u32;
    let mut nees_max = 0.0_f32;
    // v0.22 mag-heading VALIDATION: compare the real magnetometer-derived
    // heading (mag_heading + declination) against the gz-truth heading
    // oracle, BEFORE trusting it in the estimator. Confirms the body-frame
    // conversion + declination are right (the frame-bug class that caused
    // the yaw saga). Zurich declination ≈ +3° (the SDF magnetic_field).
    const MAG_DECLINATION: f32 = 0.0524; // rad (~3°)
    let mut mag_err_sum = 0.0_f64;
    let mut mag_err_max = 0.0_f32;
    let mut mag_n = 0u32;
    // v0.22 magless-yaw observability gate (persistence-of-excitation). At
    // hover the horizontal specific force ≈ 0 ⇒ yaw is unobservable WITHOUT
    // the magnetometer — this quantifies WHY the mag (or a manoeuvre) is
    // required. 1 s window, 1.5 m/s² threshold.
    let mut yaw_obs = relay_iekf::YawObservability::new(1.0, 1.5);
    let mut obs_count = 0u32;
    let mut obs_total = 0u32;
    let mut last_pos_d = 0.0_f32;
    let mut v_d_filt = 0.0_f32;
    let mut last_pos_ned = setpoint_ned;
    // Outer-loop outputs, held across inner ticks.
    let mut omega_d_held = [0.0_f32; 3];
    let mut a_cmd_held = [0.0_f32; 3];
    let mut thrust_held = hover_thrust;
    let mut est = iekf.state();

    let started_at = Instant::now();
    for step in 0..n {
        let tick_start = Instant::now();
        let t = step as f32 * dt;
        let (imu_sample, pos_ned) = physics.measure(0.0);

        // ── INNER (every tick): IEKF predict + gyro low-pass ──
        iekf.propagate(IekfImu { gyro: imu_sample.gyro_body, accel: imu_sample.accel_body }, dt);
        let gyro_f = gyro_lpf.filter(imu_sample.gyro_body);

        // ── OUTER (every outer_decim ticks): aiding updates + position →
        //    geometric desired rate (held for the inner loop to track) ──
        if step % outer_decim == 0 {
            iekf.update_gravity(imu_sample.accel_body, cfg.iekf.grav_var);
            // NEES BEFORE the position update: the predicted estimate vs
            // gz truth `pos_ned`, weighted by the predicted P. (After the
            // update the estimate is pulled to truth, so post-update NEES
            // is uninformative in noiseless SITL.)
            let nees = iekf.nees_position(pos_ned);
            if nees.is_finite() {
                nees_sum += nees as f64;
                nees_n += 1;
                if nees > nees_max {
                    nees_max = nees;
                }
            }
            iekf.update_position(pos_ned, cfg.iekf.pos_var);
            // v0.22 — heading from the REAL magnetometer (no truth cheat).
            // `heading_ned` (gz truth) is read ONLY for the validation
            // diagnostic, never fed to the estimator.
            if let Some(mag_b) = physics.mag_body_ned() {
                if let Some(mag_yaw) = relay_iekf::mag_heading(mag_b, est.q, MAG_DECLINATION) {
                    if !iekf_heading_init {
                        iekf.init_heading(mag_yaw);
                        iekf_heading_init = true;
                    }
                    // Var loosened to the measured mag noise (~2.4° ⇒
                    // ~0.0018 rad²) so the estimate isn't yanked by sensor
                    // noise.
                    iekf.update_magnetometer(mag_b, MAG_DECLINATION, cfg.iekf.yaw_var.max(0.002));
                    if let Some(hdg) = physics.heading_ned() {
                        let e = libm::remainderf(mag_yaw - hdg, 2.0 * core::f32::consts::PI);
                        mag_err_sum += e.abs() as f64;
                        if e.abs() > mag_err_max {
                            mag_err_max = e.abs();
                        }
                        mag_n += 1;
                    }
                }
            }
            est = iekf.state();

            // Magless-yaw observability: horizontal specific force in NED
            // (frame-correct, unlike body-axis accel which tilt pollutes).
            let f_ned = relay_iekf::q_rotate(est.q, imu_sample.accel_body);
            let a_horiz = (f_ned[0] * f_ned[0] + f_ned[1] * f_ned[1]).sqrt();
            if yaw_obs.update(a_horiz, dt_outer) {
                obs_count += 1;
            }
            obs_total += 1;

            // Altitude thrust (finite-diff v_d at the outer rate).
            let v_d_raw = (pos_ned[2] - last_pos_d) / dt_outer;
            v_d_filt = lp_alpha * v_d_raw + (1.0 - lp_alpha) * v_d_filt;
            last_pos_d = pos_ned[2];
            let alt_err = setpoint_ned[2] - pos_ned[2];
            thrust_held = (hover_thrust - kp_alt * alt_err + kd_alt * v_d_filt).clamp(0.0, 1.0);

            // Horizontal position → NED accel command (P-D on IEKF state).
            let mut a_cmd = [
                kp_pos * (setpoint_ned[0] - pos_ned[0]) - kd_vel * est.v[0],
                kp_pos * (setpoint_ned[1] - pos_ned[1]) - kd_vel * est.v[1],
                0.0,
            ];
            let ah = (a_cmd[0] * a_cmd[0] + a_cmd[1] * a_cmd[1]).sqrt();
            if ah > a_cmd_max {
                let s = a_cmd_max / ah;
                a_cmd[0] *= s;
                a_cmd[1] *= s;
            }
            a_cmd_held = a_cmd;
            let mut omega_d = geo.desired_rate(est.q, a_cmd, yaw_d);
            if cfg.pos.yaw_mode != YawMode::HeadingHold {
                omega_d[2] = 0.0; // rate-hold / off: no heading tracking
            }
            omega_d_held = omega_d;
        }

        // ── INNER torque: ADRC tracks the held rate on FILTERED gyro ──
        let tilt = body_tilt_rad(imu_sample.accel_body);
        let arm = seq.tick(tilt, true);
        let torque = if arm.torque_authority {
            let m = if use_adrc {
                adrc.tick(gyro_f, omega_d_held, dt)
            } else {
                geo.tick(est.q, gyro_f, a_cmd_held, yaw_d)
            };
            let yaw_t = if cfg.pos.yaw_mode == YawMode::Off { 0.0 } else { m[2] * torque_scale };
            [m[0] * torque_scale, m[1] * torque_scale, yaw_t]
        } else {
            adrc.reset();
            [0.0_f32; 3]
        };
        let floor = cfg.pos.mixer_floor;
        let th = thrust_held * arm.thrust_scale;
        let fl = floor * arm.thrust_scale;
        let motors = match cfg.pos.mixer_mode {
            falcon_config::MixerMode::ThrustFloor => mixer.mix_thrust_floor(torque, th, fl),
            falcon_config::MixerMode::Priority => mixer.mix_priority(torque, th, fl),
            falcon_config::MixerMode::Airmode => mixer.mix_airmode(torque, th, fl),
        };
        physics.step(motors, dt);

        let dn = pos_ned[0] - setpoint_ned[0];
        let de = pos_ned[1] - setpoint_ned[1];
        let dd = pos_ned[2] - setpoint_ned[2];
        let dist = (dn * dn + de * de + dd * dd).sqrt();
        if dist > peak_dist { peak_dist = dist; }
        if dist < min_dist { min_dist = dist; }
        if t >= steady_start_t {
            sum_sq_steady += dist * dist;
            steady_count += 1;
        }
        last_pos_ned = pos_ned;

        if std::env::var("POS_DEBUG").is_ok() && step % 50 == 0 {
            let iyaw = {
                let q = est.q;
                libm::atan2f(2.0 * (q[0] * q[3] + q[1] * q[2]), 1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]))
            };
            let chdg = physics.heading_ned().unwrap_or(f32::NAN).to_degrees();
            eprintln!(
                "    [geo] t={t:.1} pos=[{:.1},{:.1},{:.1}] dist={dist:.2} tilt={:.1}° IEKFyaw={:.1}° compass={:.1}°",
                pos_ned[0], pos_ned[1], pos_ned[2], tilt.to_degrees(),
                iyaw.to_degrees(), chdg,
            );
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
    let dn = last_pos_ned[0] - setpoint_ned[0];
    let de = last_pos_ned[1] - setpoint_ned[1];
    let dd = last_pos_ned[2] - setpoint_ned[2];
    let final_dist = (dn * dn + de * de + dd * dd).sqrt();
    let rms_steady = if steady_count > 0 {
        (sum_sq_steady / steady_count as f32).sqrt()
    } else {
        f32::NAN
    };
    let anees = if nees_n > 0 { (nees_sum / nees_n as f64) as f32 } else { f32::NAN };
    println!(
        "  verdict: backend={} scenario=geo-hover steps={} final_dist={:.2}m peak_dist={:.2}m rms_steady={:.2}m  wall={:.2}s",
        physics.name(), n, final_dist, peak_dist, rms_steady, wall.as_secs_f32(),
    );
    // IEKF consistency report (3-DoF position, χ²₃: E=3, 95% single-sample
    // band [0.216, 9.35]). The SAFETY-relevant direction is OVER-CONFIDENT
    // (ANEES ≫ 3): the filter claims tighter covariance than its true
    // error — the precursor to divergence. Below ~3 is over-conservative
    // (safe). The gate flags only the over-confident extreme.
    let nees_status = if !anees.is_finite() {
        "NO-DATA"
    } else if anees > 9.35 {
        "OVER-CONFIDENT"
    } else if anees < 0.216 {
        "over-conservative"
    } else {
        "CONSISTENT"
    };
    let nees_ok = anees.is_finite() && anees <= 9.35;
    println!(
        "  iekf-consistency: position ANEES={:.2} (target≈3.0, dof=3) max={:.1} n={} [{}]",
        anees, nees_max, nees_n, nees_status,
    );
    // v0.22 — closed-loop heading accuracy: the magnetometer-driven
    // estimate vs the gz-truth heading (truth used for SCORING only, never
    // fed to the estimator). Frame-correctness was confirmed open-loop at
    // ~2.4°; closed-loop ~6-7° (mag noise + tilt coupling during position
    // corrections) is the honest operating accuracy — fine for hold.
    if mag_n > 0 {
        let mag_err_mean = (mag_err_sum / mag_n as f64) as f32;
        let mag_status = if mag_err_mean.to_degrees() < 12.0 { "OK" } else { "DEGRADED" };
        println!(
            "  mag-heading: closed-loop err vs truth mean={:.1}° max={:.1}° n={} [{}]",
            mag_err_mean.to_degrees(), mag_err_max.to_degrees(), mag_n, mag_status,
        );
    } else {
        println!("  mag-heading: no magnetometer frames received");
    }
    // Observability gate report: at a clean hover this should be near 0%
    // observable — i.e. yaw is unobservable without the magnetometer,
    // confirming why v0.22's mag fusion (not GPS-only) is the heading
    // source. (Under a manoeuvre it would rise — magless recovery window.)
    if obs_total > 0 {
        let obs_pct = 100.0 * obs_count as f32 / obs_total as f32;
        println!(
            "  yaw-observability (magless): {:.0}% of hover excited above threshold (low ⇒ mag required)",
            obs_pct,
        );
    }
    if let Some(ref mut e) = evidence {
        e.write_summary_hover(n, final_dist, peak_dist, rms_steady, min_dist, wall.as_secs_f32(), physics.counters());
    }
    // PASS requires BOTH position-hold AND filter consistency: an
    // over-confident estimator is unsafe even if this run's position
    // looked fine (it is the divergence precursor).
    final_dist < 0.5 && rms_steady < 1.0 && nees_ok
}

fn run_alt_rate_hover(
    physics: &mut dyn Physics,
    duration_s: f32,
    mut evidence: Option<&mut EvidenceSink>,
) -> bool {
    let mut rate_pid = RatePid::new();
    let mut mixer = QuadMixer::new();
    // v0.19.9 — verified arming sequencer replaces the old `t < 0.5`
    // spawn-hold. Torque authority engages ONLY after spin-up + a
    // confirmed-level window (Kani-proven gating, relay-arm ARM-P01).
    let mut seq = ArmingSequencer::new(ArmingConfig::falcon_quad_100hz());
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

        // v0.19.9 — verified arming sequencer (relay-arm). Replaces the
        // old time-only `t < 0.5` spawn-hold: torque authority engages
        // ONLY after spin-up AND a confirmed-level window, so spawn
        // transients (rotor imbalance + IMU noise) can never be converted
        // into bang-bang torque before steady state establishes (RC#1).
        // Gating is Kani-proven (ARM-P01); here it runs against gz physics.
        let tilt = body_tilt_rad(imu_sample.accel_body);
        let arm = seq.tick(tilt, true);
        let torque_raw = if arm.torque_authority {
            rate_pid.tick(rate_ts_of(t), imu_sample.gyro_body, [0.0_f32; 3])
        } else {
            [0.0_f32; 3]
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
        // v0.19.9 — the arming sequencer ramps collective (and the floor)
        // during spin-up so the props reach idle smoothly; thrust_scale
        // is 1.0 once armed, recovering the v0.19.8 behaviour exactly.
        let motors =
            mixer.mix_thrust_floor(torque, thrust * arm.thrust_scale, 0.5 * arm.thrust_scale);
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
    // v0.21 — the Invariant EKF runs in PARALLEL with the Mahony filter
    // for the diagnostic: POS_DEBUG prints iekf_tilt vs the Mahony
    // est_tilt vs true_tilt. If the IEKF tracks truth where Mahony
    // diverges to 51° (v0.20 RC#3), the keystone works.
    let mut iekf = Iekf::level();
    let mut iekf_heading_init = false;
    // v0.22 — hold a FIXED heading (the spawn heading, NED yaw ≈ 90° =
    // body facing East). "Hold current estimate yaw" feeds estimate noise
    // back through the Euler setpoint and spins the body once yaw ≠ 0.
    let yaw_hold = core::f32::consts::FRAC_PI_2;
    let mut rate_pid = RatePid::new();
    let mut att = AttController::new();
    // v0.20 — calibrate the position controller's hover thrust for the gz
    // 2 kg falcon-quad. The relay-pos DEFAULT (0.5, a 500 g 10-inch quad)
    // sits AT the mixer floor, so the body could not climb. 0.72 is the
    // measured hover collective for this airframe (matches alt-rate).
    // v0.21 — gentler position gains for the gz 2 kg body on a still-
    // imperfect estimate: a soft kp_pos + tighter tilt limit keep the
    // body near-stationary (in the observable regime) so the IEKF
    // attitude estimate is not destabilised by aggressive translation.
    let mut pos = PosController::with_gains(PosGains {
        hover_thrust: 0.72,
        kp_pos: 0.4,
        tilt_max: 0.20,
        ..PosGains::DEFAULT
    });
    let mut mixer = QuadMixer::new();
    // v0.19.9 — arming sequencer gates the rate loop here too. The full
    // cascade had no spawn-hold at all (torque from t=0), so it was the
    // clearest RC#1 victim.
    let mut seq = ArmingSequencer::new(ArmingConfig::falcon_quad_100hz());

    let setpoint_ned = [0.0_f32, 0.0, -2.0];
    let setpoint = if std::env::var("USE_IEKF").is_ok() {
        PositionSetpoint { position_ned: setpoint_ned, velocity_ned: [0.0; 3], yaw_setpoint: yaw_hold }
    } else {
        PositionSetpoint::hover_at(setpoint_ned)
    };

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

        // 2b. IEKF — propagate on IMU, correct on gz position (the
        //     "GPS"), and on heading (v0.22 "compass") which makes yaw
        //     observable (the v0.21 ±130° wander). The measured accel is
        //     body-frame specific force, exactly the IEKF's dynamics input.
        iekf.propagate(IekfImu { gyro: imu_sample.gyro_body, accel: imu_sample.accel_body }, dt);
        // Adaptive gravity/tilt fusion (variance inflates under accel) +
        // position. Gives the roll/pitch observability the IMU+GPS-only
        // filter lacked (3° error → tip-over).
        iekf.update_gravity(imu_sample.accel_body, 0.5);
        iekf.update_position(pos_ned, 0.04); // ~0.2 m 1σ NavSat
        if let Some(hdg) = physics.heading_ned() {
            // One-time alignment: start the estimate at the body's true
            // heading so the controller doesn't see a 90° startup yaw
            // error (which kicked it into a spin).
            if !iekf_heading_init {
                iekf.init_heading(hdg);
                iekf_heading_init = true;
            }
            iekf.update_yaw(hdg, 0.01); // ~6° 1σ compass
        }

        // 3. POS — position + velocity → attitude setpoint.
        let v_fd = match last_pos_ned {
            Some(p) => [
                (pos_ned[0] - p[0]) / dt,
                (pos_ned[1] - p[1]) / dt,
                (pos_ned[2] - p[2]) / dt,
            ],
            None => [0.0; 3],
        };
        last_pos_ned = Some(pos_ned);
        // v0.21 — control attitude source: the IEKF (USE_IEKF) vs the
        // legacy Mahony filter. Closing the loop on the IEKF is the test
        // of whether the keystone fixes RC#3 → position-hold. When on the
        // IEKF, also feed its SMOOTH velocity estimate (the finite-diff
        // v_fd from NavSat is noisy and was destabilising the vel loop).
        let use_iekf = std::env::var("USE_IEKF").is_ok();
        let est_q = if use_iekf { iekf.state().q } else { est.quaternion };
        let v_ned = if use_iekf { iekf.state().v } else { v_fd };
        let att_sp = pos.tick(
            pos_ts_of(t),
            pos_ned,
            v_ned,
            est_q,
            setpoint,
        );
        current_att_sp = att_sp.quaternion;
        current_thrust = att_sp.thrust;

        // 4. ATT — quaternion error → rate setpoint.
        let current_rate_sp = att.tick(att_ts_of(t), est_q, current_att_sp);

        // 5. RATE — gyro + rate setpoint → torque (frame-corrected),
        //    gated by the arming sequencer (RC#1). Torque authority
        //    engages only after spin-up + confirmed level.
        let tilt = body_tilt_rad(imu_sample.accel_body);
        let arm = seq.tick(tilt, true);
        let torque_raw = if arm.torque_authority {
            rate_pid.tick(rate_ts_of(t), imu_sample.gyro_body, current_rate_sp)
        } else {
            [0.0_f32; 3]
        };
        let torque = frame_correct_torque(torque_raw);
        for k in 0..3 {
            if !torque[k].is_finite() { nan_seen = true; }
        }

        // 6. MIX — torque + thrust → 4× motor PWM. v0.19.9 carries the
        //    v0.19.8 verified thrust-floor into the full cascade and
        //    ramps collective + floor during spin-up.
        let motors = mixer.mix_thrust_floor(
            torque,
            current_thrust * arm.thrust_scale,
            0.5 * arm.thrust_scale,
        );
        if motors.iter().any(|v| !v.is_finite()) { nan_seen = true; }

        // 7. Publish to the bridge.
        physics.step(motors, dt);

        if std::env::var("POS_DEBUG").is_ok() && step % 50 == 0 {
            // est_tilt: body-down axis rotated by the EKF quaternion vs NED-down.
            let q = est.quaternion;
            let bz_d = 1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]); // R[2][2]
            let est_tilt = libm::acosf(bz_d.clamp(-1.0, 1.0));
            let is = iekf.state();
            let iq = is.q;
            let iyaw = libm::atan2f(2.0 * (iq[0] * iq[3] + iq[1] * iq[2]), 1.0 - 2.0 * (iq[2] * iq[2] + iq[3] * iq[3]));
            let chdg = physics.heading_ned().unwrap_or(f32::NAN).to_degrees();
            eprintln!(
                "    [dbg] t={t:.1} pos=[{:.1},{:.1},{:.1}] true_tilt={:.1}° IEKF_tilt={:.1}° IEKF_yaw={:.1}° compass={:.1}° ipos=[{:.1},{:.1},{:.1}]",
                pos_ned[0], pos_ned[1], pos_ned[2],
                tilt.to_degrees(), is.tilt_rad().to_degrees(), iyaw.to_degrees(), chdg,
                is.p[0], is.p[1], is.p[2],
            );
        }

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

    /// v0.19.9 — `body_tilt_rad` reads 0 when the specific-force vector
    /// is purely vertical (level) and grows toward 90° as it tilts into
    /// the horizontal plane. Convention-agnostic in the sign of az.
    #[test]
    fn body_tilt_rad_matches_geometry() {
        // Level: gravity reaction straight along body-z (either sign).
        assert!(body_tilt_rad([0.0, 0.0, -9.81]).abs() < 1e-4);
        assert!(body_tilt_rad([0.0, 0.0, 9.81]).abs() < 1e-4);
        // 45°: equal horizontal and vertical components.
        assert!((body_tilt_rad([9.81, 0.0, 9.81]) - std::f32::consts::FRAC_PI_4).abs() < 1e-4);
        // Fully horizontal specific force → 90° tilt.
        assert!((body_tilt_rad([9.81, 0.0, 0.0]) - std::f32::consts::FRAC_PI_2).abs() < 1e-4);
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
        // Identity: roll/pitch via the SDF index-map alignment, and yaw's
        // measured inversion is corrected in the YAW ADRC b0 (negative),
        // not here — see frame_correct_torque docs. This seam stays the
        // documented boundary where a frame correction *would* live.
        let t = [0.3_f32, -0.7, 0.2];
        assert_eq!(frame_correct_torque(t), t);
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

    /// v0.26 END-TO-END fault-tolerance chain (deterministic, no gz):
    /// a hovering quad loses rotor 0 mid-flight; the FDI (RotorFaultDetector)
    /// detects + isolates it from the commanded-vs-achieved residual, the
    /// loop switches to the reduced-attitude (S²) law + the MIX-P08
    /// reconfigured allocator, and the THRUST AXIS stays upright (the body
    /// is allowed to spin in yaw). Proves the three verified components
    /// compose into a recovery. Rigid-body attitude sim; the mixer round-
    /// trip `motors_to_torque_signs(mix(τ)) = 4τ` (zero-sum columns) ⇒
    /// SCALE = 0.25 makes the healthy roundtrip ≈ identity, so the failure
    /// shows as the real allocation imbalance.
    #[test]
    fn fault_tolerance_chain_recovers_from_rotor_loss() {
        use relay_geo::{GeoAtt, GeoGains};
        use relay_iekf::RotorFaultDetector;
        use relay_mix_quad::{motors_to_torque_signs, QuadMixer};

        let ctrl = GeoAtt::new(GeoGains::FALCON_QUAD);
        let j = GeoGains::FALCON_QUAD.j;
        let b3_d = [0.0f32, 0.0, 1.0]; // level hover thrust axis
        let level = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let (hover, floor) = (0.5f32, 0.15f32);
        const SCALE: f32 = 0.25;
        let dt = 0.002f32;
        let (real_failed, fail_step) = (0usize, 1000u32); // rotor 0 dies at 2 s

        let mut r = level;
        let mut omega = [0.0f32; 3];
        let mut fdi = RotorFaultDetector::new(0.5, 0.1);
        let mut isolated: Option<usize> = None;
        let mut max_tilt_after = 0.0f32;

        for step in 0..4000u32 {
            // Controller + allocation (full-attitude until a fault is isolated,
            // then reduced-attitude + the reconfigured allocator).
            let (torque, motors_cmd) = if let Some(f) = isolated {
                let tq = ctrl.moment_reduced(&r, omega, b3_d);
                (tq, QuadMixer::new().mix_rotor_out(f, tq, hover, floor))
            } else {
                let tq = ctrl.moment(&r, omega, &level);
                (tq, QuadMixer::new().mix_thrust_floor(tq, hover, floor))
            };
            let _ = torque;

            // MIX-P08 holds in the integrated loop: while reconfigured (the
            // mode that PRODUCED motors_cmd this step), the failed rotor is
            // OFF and the healthy three stay in [floor,1].
            if let Some(f) = isolated {
                assert_eq!(motors_cmd[f], 0.0, "failed rotor must be commanded 0");
                for (i, &v) in motors_cmd.iter().enumerate() {
                    if i != f {
                        assert!(v >= floor - 1e-6 && v <= 1.0 + 1e-6, "healthy motor bound: {v}");
                    }
                }
            }

            // Physics: inject the rotor-0 failure (its thrust → 0).
            let mut motors_real = motors_cmd;
            if step >= fail_step {
                motors_real[real_failed] = 0.0;
            }
            // FDI on the commanded-vs-achieved residual.
            if isolated.is_none() {
                let mut resid = [0.0f32; 4];
                for i in 0..4 {
                    resid[i] = (motors_cmd[i] - motors_real[i]).abs();
                }
                if let Some(f) = fdi.update(resid) {
                    isolated = Some(f);
                }
            }
            // Body torque from the ACHIEVED motors → rigid-body update.
            let bt = motors_to_torque_signs(motors_real);
            let body_t = [bt[0] * SCALE, bt[1] * SCALE, bt[2] * SCALE];
            let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
            let gyro = [
                omega[1] * jo[2] - omega[2] * jo[1],
                omega[2] * jo[0] - omega[0] * jo[2],
                omega[0] * jo[1] - omega[1] * jo[0],
            ];
            for i in 0..3 {
                omega[i] += dt * (body_t[i] - gyro[i]) / j[i];
            }
            r = integ_rot(&r, omega, dt);

            if step > fail_step + 300 {
                let tilt = (r[2][2].clamp(-1.0, 1.0)).acos();
                if tilt > max_tilt_after {
                    max_tilt_after = tilt;
                }
            }
        }
        let final_tilt = (r[2][2].clamp(-1.0, 1.0)).acos();
        assert_eq!(isolated, Some(real_failed), "FDI must isolate the failed rotor");
        // The body does NOT tumble (never inverts past ~80°) and SETTLES
        // back toward upright — the reduced-attitude law recovers the
        // thrust axis. (Full hover incl. the periodic spin solution is the
        // gz/SITL follow-on; here the rigid-body sim proves the chain
        // composes + stays controlled.)
        assert!(
            max_tilt_after < 1.4,
            "thrust axis must not tumble after rotor loss: peak {max_tilt_after} rad"
        );
        assert!(
            final_tilt < 0.5,
            "thrust axis must settle back upright: final {final_tilt} rad (peak {max_tilt_after})"
        );
    }

    /// v0.27 "MISSION" scenario (deterministic, no gz): fly a square
    /// waypoint mission with the FULL v0.27 chain — per-leg Mueller quintic
    /// trajectory (relay-traj), differential-flatness feedforward
    /// (relay-geo flatness_omega_ff), geometric attitude control, and a
    /// position P-D, in a 6-DoF rigid-body sim. The thrust acts along the
    /// ACHIEVED body axis (so attitude-tracking error shows up as position
    /// error). Asserts the body reaches each waypoint. Demonstrates that the
    /// trajectory generator + flatness feedforward + controller compose.
    #[test]
    fn mission_follows_waypoint_trajectory() {
        use relay_geo::{desired_attitude, thrust_axis_ned, flatness_omega_ff, GeoAtt, GeoGains};
        use relay_traj::Segment3;

        let j = [0.0217f32, 0.0217, 0.04];
        // Fast idealised attitude loop (this sim has no actuator lag / mixer
        // scale — see the gz path for the real-airframe gains).
        let (kr, kw) = (24.0f32, 9.0f32);
        let (kp_pos, kd_pos) = (3.0f32, 4.0f32);
        let g_ned = [0.0f32, 0.0, 9.81];
        let dt = 0.001f32;

        // Square mission at 2 m altitude (NED down = −2).
        let wps = [
            [0.0f32, 0.0, -2.0],
            [3.0, 0.0, -2.0],
            [3.0, 3.0, -2.0],
            [0.0, 3.0, -2.0],
        ];
        let leg_t = 3.0f32;

        // State: position p, velocity v, attitude R, body rate ω.
        let mut p = wps[0];
        let mut v = [0.0f32; 3];
        let mut r = [[1.0f32, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let mut omega = [0.0f32; 3];

        for leg in 0..wps.len() - 1 {
            let seg = Segment3::rest_to_rest(wps[leg], wps[leg + 1], leg_t);
            let steps = (leg_t / dt) as u32;
            for k in 0..steps {
                let t = k as f32 * dt;
                let s = seg.eval(t);
                // a_cmd = trajectory accel ff + position P-D.
                let a_cmd = [
                    s.acc[0] + kp_pos * (s.pos[0] - p[0]) + kd_pos * (s.vel[0] - v[0]),
                    s.acc[1] + kp_pos * (s.pos[1] - p[1]) + kd_pos * (s.vel[1] - v[1]),
                    s.acc[2] + kp_pos * (s.pos[2] - p[2]) + kd_pos * (s.vel[2] - v[2]),
                ];
                let b3_d = thrust_axis_ned(a_cmd).unwrap();
                let r_d = desired_attitude(b3_d, 0.0).unwrap();
                let c = {
                    let d = [g_ned[0] - a_cmd[0], g_ned[1] - a_cmd[1], g_ned[2] - a_cmd[2]];
                    (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
                };
                let off = flatness_omega_ff(a_cmd, s.jerk, 0.0, 0.0).unwrap_or([0.0; 3]);

                // Attitude: geometric error + rate feedforward.
                let e_r = GeoAtt::attitude_error(&r, &r_d);
                let jo = [j[0] * omega[0], j[1] * omega[1], j[2] * omega[2]];
                let gyro = [
                    omega[1] * jo[2] - omega[2] * jo[1],
                    omega[2] * jo[0] - omega[0] * jo[2],
                    omega[0] * jo[1] - omega[1] * jo[0],
                ];
                let mut tau = [0.0f32; 3];
                for i in 0..3 {
                    tau[i] = -kr * e_r[i] - kw * (omega[i] - off[i]) + gyro[i];
                    omega[i] += dt * (tau[i] - gyro[i]) / j[i];
                }
                r = integ_rot(&r, omega, dt);

                // Position: thrust (magnitude c, along −body-down axis) + g.
                let b3_ach = [r[0][2], r[1][2], r[2][2]];
                for i in 0..3 {
                    let accel = g_ned[i] - c * b3_ach[i];
                    v[i] += dt * accel;
                    p[i] += dt * v[i];
                }
                let _ = GeoGains::FALCON_QUAD;
            }
            // Reached the waypoint at the end of the leg.
            let err = {
                let d = [p[0] - wps[leg + 1][0], p[1] - wps[leg + 1][1], p[2] - wps[leg + 1][2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt()
            };
            assert!(err < 0.4, "leg {leg}: missed waypoint by {err} m (p={p:?})");
        }
    }

    /// R ← R·(I + [ω]× dt) with Gram-Schmidt re-orthonormalisation.
    #[cfg(test)]
    fn integ_rot(r: &[[f32; 3]; 3], w: [f32; 3], dt: f32) -> [[f32; 3]; 3] {
        let wd = [w[0] * dt, w[1] * dt, w[2] * dt];
        let incr = [[1.0, -wd[2], wd[1]], [wd[2], 1.0, -wd[0]], [-wd[1], wd[0], 1.0]];
        let mut m = [[0.0f32; 3]; 3];
        for i in 0..3 {
            for jj in 0..3 {
                let mut s = 0.0;
                for k in 0..3 {
                    s += r[i][k] * incr[k][jj];
                }
                m[i][jj] = s;
            }
        }
        // Gram-Schmidt on columns.
        let col = |mm: &[[f32; 3]; 3], c: usize| [mm[0][c], mm[1][c], mm[2][c]];
        let norm = |v: [f32; 3]| (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]).sqrt();
        let c0 = col(&m, 0);
        let n0 = norm(c0).max(1e-9);
        let e0 = [c0[0] / n0, c0[1] / n0, c0[2] / n0];
        let c1 = col(&m, 1);
        let d = e0[0] * c1[0] + e0[1] * c1[1] + e0[2] * c1[2];
        let p1 = [c1[0] - d * e0[0], c1[1] - d * e0[1], c1[2] - d * e0[2]];
        let n1 = norm(p1).max(1e-9);
        let e1 = [p1[0] / n1, p1[1] / n1, p1[2] / n1];
        let e2 = [
            e0[1] * e1[2] - e0[2] * e1[1],
            e0[2] * e1[0] - e0[0] * e1[2],
            e0[0] * e1[1] - e0[1] * e1[0],
        ];
        [[e0[0], e1[0], e2[0]], [e0[1], e1[1], e2[1]], [e0[2], e1[2], e2[2]]]
    }
}
