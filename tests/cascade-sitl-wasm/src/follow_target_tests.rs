//! #609 finding 4, Decision A: in Mission the frame `target` is a LIVE follow
//! setpoint (maintainer decision 2026-10-28).
//!
//! These tests drive [`NativeCascade`] — the call-for-call mirror of the
//! published component's `step` — so they exercise what a HOST sees: one
//! `target` per frame, no other channel. Each judges PLANT TRUTH
//! (`SimBackend::pos`), takes the PEAK over its window, and checks the mode on
//! every tick of it.
//!
//! The mirror's `step` signature is unchanged by the fix, so this file compiles
//! against the pre-fix mirror too; every test here fails there by BEHAVIOUR:
//! the frame target used to reset a one-leg mission every tick, so arrival
//! fired the mission-complete RTL and landed the vehicle while the host was
//! still streaming, and a non-finite target was flown into the motors.

use super::{Calib, NativeCascade, SensorCal, SupCalib};
use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const CRUISE: f32 = 2.0;
const PACK_V: f32 = 16.8;

/// SWREQ-FALCON-HOLD-P01's bars, read from the requirement: "within 0.5 m
/// vertically and 1.0 m horizontally of the setpoint for at least 60 s".
const HOLD_HORIZ_M: f32 = 1.0;
const HOLD_VERT_M: f32 = 0.5;
const HOLD_S: f32 = 60.0;

/// The follow-stream timeout the supervisor enforces (falcon-core's
/// `FOLLOW_TARGET_TIMEOUT_US`, 1.0 s), restated here so a change to it has to
/// change this test too.
const TIMEOUT_S: f32 = 1.0;

/// Time after ARRIVAL before the HOLD-P01 window is judged. A streamed target
/// is a step in the setpoint, and the transit charges the position integral
/// toward it (clamped), so the vehicle overshoots and the integral takes a long
/// time to unwind: the same core property `crates/falcon-core/tests/
/// loiter_hold_latch.rs` (3c) measured for a Mission leg and allows 90 s for.
/// Not changed by Decision A, reported by A1 rather than hidden.
const BRAKE_S: f32 = 90.0;

fn horiz(b: &SimBackend, at: [f32; 2]) -> f32 {
    ((b.pos[0] - at[0]).powi(2) + (b.pos[1] - at[1]).powi(2)).sqrt()
}

fn mode(n: &NativeCascade) -> Mode {
    n.host.supervisor().expect("supervisor built").mode()
}

fn flags(n: &NativeCascade) -> u32 {
    n.host
        .supervisor()
        .expect("supervisor built")
        .failsafe_flags()
}

/// One component frame: read the plant, hand the frame to the mirror exactly as
/// a host hands it to `step`, write the motors back. Returns the motors.
fn frame(n: &mut NativeCascade, b: &mut SimBackend, target: [f32; 3]) -> [f32; 4] {
    let imu = b.read_imu();
    let pos = b.read_position();
    let mag = b.read_mag();
    let heading = b.read_heading();
    let rpm = b.read_motor_rpm();
    let m = n.step(
        imu.accel,
        imu.gyro,
        target,
        pos,
        mag,
        heading,
        rpm,
        Some(PACK_V),
        b.dt(),
    );
    b.write_motors(&m);
    m
}

/// A configured mirror on a plant at rest at `launch`, the estimator converged
/// on the ground with the wind off, the wind then switched on.
fn rig(launch: [f32; 2], wind: [f32; 3], stream: [f32; 3]) -> (SimBackend, NativeCascade) {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.pos = [launch[0], launch[1], 0.0];
    let mut n = NativeCascade::new();
    n.configure(Calib {
        hover_thrust: 0.5,
        loop_rate_hz: 1.0 / DT,
        pos_var: 0.01,
        process_floor_vel: 0.0,
        process_floor_pos: 0.0,
        altitude_kp: 0.05,
        altitude_kd: 0.30,
        altitude_ki: 0.0,
        position_ki: 0.02,
    });
    n.configure_supervisor(SupCalib {
        home: [0.0, 0.0, 0.0],
        fence_radius: 80.0,
        cruise_alt: CRUISE,
        low_batt_v: 14.0,
    });
    // The first frame builds the supervisor.
    frame(&mut n, &mut b, stream);
    // A configured calibration (CalibrationPresent), through the seam's own
    // `configure-calibration` export; the same gyro-bias fixture
    // falcon-core's supervised tests use.
    n.configure_calibration(SensorCal {
        gyro_bias: [0.001, 0.0, 0.0],
        accel_bias: [0.0; 3],
        accel_scale: [1.0; 3],
        mag_offset: [0.0; 3],
        mag_scale: [1.0; 3],
        mag_offdiag: [0.0; 3],
    });
    for _ in 0..(20.0 / DT) as u32 {
        frame(&mut n, &mut b, stream);
    }
    b.wind = wind;
    (b, n)
}

/// Arm, take off, settle into Loiter. Panics if the vehicle never gets there.
fn takeoff(n: &mut NativeCascade, b: &mut SimBackend, stream: [f32; 3]) {
    n.command(Event::Arm, true, true);
    n.command(Event::RequestTakeoff, true, true);
    assert_eq!(
        mode(n),
        Mode::Takeoff,
        "arm + takeoff refused: {:?}",
        n.host.supervisor().and_then(|s| s.arm_blocked_check())
    );
    for i in 0..15_000 {
        if mode(n) == Mode::Loiter {
            return;
        }
        assert_eq!(mode(n), Mode::Takeoff, "left Takeoff at tick {i}");
        frame(n, b, stream);
    }
    panic!("never reached Loiter (stuck in {:?})", mode(n));
}

/// Stream `target` for `secs`, asserting `want` on EVERY tick and finite
/// motors; return the peak horizontal / vertical error from the target.
fn follow_window(
    n: &mut NativeCascade,
    b: &mut SimBackend,
    target: [f32; 3],
    want: Mode,
    secs: f32,
) -> (f32, f32) {
    let (mut ph, mut pv) = (0.0f32, 0.0f32);
    for i in 0..(secs / DT) as u32 {
        let m = frame(n, b, target);
        assert_eq!(
            mode(n),
            want,
            "mode changed at tick {i} ({:.2} s) of the window while the target {target:?} \
             was still being streamed",
            i as f32 * DT
        );
        assert!(m.iter().all(|v| v.is_finite()), "non-finite motors {m:?}");
        ph = ph.max(horiz(b, [target[0], target[1]]));
        pv = pv.max((b.pos[2] - target[2]).abs());
    }
    (ph, pv)
}

/// Stream `target` until the vehicle is within `radius` of it horizontally,
/// asserting Mission on every tick. Returns the time taken.
fn fly_to(
    n: &mut NativeCascade,
    b: &mut SimBackend,
    target: [f32; 3],
    radius: f32,
    max_s: f32,
) -> f32 {
    for i in 0..(max_s / DT) as u32 {
        if horiz(b, [target[0], target[1]]) < radius && (b.pos[2] - target[2]).abs() < 0.5 {
            return i as f32 * DT;
        }
        frame(n, b, target);
        assert_eq!(
            mode(n),
            Mode::Mission,
            "left Mission at tick {i} while flying to {target:?}"
        );
    }
    panic!(
        "never reached {target:?} in {max_s} s: {:.2} m off",
        horiz(b, [target[0], target[1]])
    );
}

/// Enter Mission while streaming `target`.
fn start_mission(n: &mut NativeCascade, b: &mut SimBackend, target: [f32; 3]) {
    frame(n, b, target);
    n.command(Event::RequestMission, true, true);
    assert_eq!(mode(n), Mode::Mission, "RequestMission refused");
}

/// (A1) A streamed target is reached and HELD for at least 60 s, in Mission,
/// with no RTL or Land. Under 2 m/s steady wind so the hold is a real one.
///
/// Pre-fix: arrival within WAYPOINT_RADIUS fired the mission-complete RTL, so
/// the per-tick mode check fails as soon as the vehicle gets there.
#[test]
fn a1_streamed_target_is_reached_and_held_in_mission() {
    let launch = [4.0, -3.0];
    let hover = [launch[0], launch[1], -CRUISE];
    let (mut b, mut n) = rig(launch, [2.0, 0.0, 0.0], hover);
    takeoff(&mut n, &mut b, hover);
    let target = [launch[0] + 12.0, launch[1] + 6.0, -3.0];
    start_mission(&mut n, &mut b, target);
    let t_reach = fly_to(&mut n, &mut b, target, 1.0, 60.0);
    // Brake + settle in Mission (mode checked every tick), then judge HOLD-P01.
    let (settle_h, _) = follow_window(&mut n, &mut b, target, Mode::Mission, BRAKE_S);
    let (ph, pv) = follow_window(&mut n, &mut b, target, Mode::Mission, HOLD_S);
    eprintln!(
        "A1: reached {target:?} in {t_reach:.1} s; overshoot peak {settle_h:.3} m in the \
         {BRAKE_S} s after arrival; then 60 s window peak horiz {ph:.3} m, vert {pv:.3} m; \
         mode Mission for all {:.0} s",
        BRAKE_S + HOLD_S
    );
    assert!(ph < HOLD_HORIZ_M, "did not hold the target: {ph:.3} m");
    assert!(
        pv < HOLD_VERT_M,
        "did not hold the target altitude: {pv:.3} m"
    );
}

/// (A2) A target moved mid-flight is followed. The move happens AFTER the first
/// target is reached, so the pre-fix one-leg mission has already RTL'd.
#[test]
fn a2_target_moved_mid_flight_is_followed() {
    let launch = [0.0, 5.0];
    let hover = [launch[0], launch[1], -CRUISE];
    let (mut b, mut n) = rig(launch, [0.0; 3], hover);
    takeoff(&mut n, &mut b, hover);
    let t1 = [10.0, 5.0, -CRUISE];
    start_mission(&mut n, &mut b, t1);
    let s1 = fly_to(&mut n, &mut b, t1, 1.0, 60.0);
    let (h1, _) = follow_window(&mut n, &mut b, t1, Mode::Mission, 10.0);
    // Move it 15 m away, at a different altitude.
    let t2 = [10.0, -10.0, -4.0];
    let s2 = fly_to(&mut n, &mut b, t2, 1.0, 60.0);
    let _ = follow_window(&mut n, &mut b, t2, Mode::Mission, BRAKE_S);
    let (ph, pv) = follow_window(&mut n, &mut b, t2, Mode::Mission, 20.0);
    eprintln!(
        "A2: t1 reached in {s1:.1} s, then 10 s in Mission (peak {h1:.3} m); moved to {t2:?}, \
         reached in {s2:.1} s; peak over {BRAKE_S}-{} s after: horiz {ph:.3} m, vert {pv:.3} m",
        BRAKE_S + 20.0
    );
    assert!(
        ph < HOLD_HORIZ_M && pv < HOLD_VERT_M,
        "did not settle on the moved target"
    );
    assert!(horiz(&b, [t1[0], t1[1]]) > 10.0, "still at the old target");
}

/// (A3) The stream STOPS — every frame from now on carries no usable target
/// (the frame has no absent flag, so a non-finite target is how it stops) —
/// and the offboard-timeout failsafe fires: relay-fsafe's policy for a stale
/// offboard stream, HOLD, flown as Loiter where the vehicle is, after 1.0 s.
///
/// Pre-fix: the NaN target became the mission leg and the vehicle never left
/// Mission (and its motors went non-finite).
#[test]
fn a3_target_stream_stopping_fires_the_offboard_timeout_hold() {
    let launch = [-6.0, 2.0];
    let hover = [launch[0], launch[1], -CRUISE];
    let (mut b, mut n) = rig(launch, [0.0; 3], hover);
    takeoff(&mut n, &mut b, hover);
    let target = [4.0, 2.0, -CRUISE];
    start_mission(&mut n, &mut b, target);
    fly_to(&mut n, &mut b, target, 1.0, 60.0);
    // Settled on the target before the stream stops, so the HOLD is judged on
    // where it stopped rather than on the follow's own transient.
    let _ = follow_window(&mut n, &mut b, target, Mode::Mission, BRAKE_S);
    assert_eq!(
        flags(&n) & (1 << 8),
        0,
        "target-stale raised while streaming"
    );

    let gone = [f32::NAN; 3];
    let stop_at = [b.pos[0], b.pos[1]];
    let mut fired = None;
    for i in 0..(5.0 / DT) as u32 {
        let m = frame(&mut n, &mut b, gone);
        assert!(
            m.iter().all(|v| v.is_finite()),
            "non-finite motors {m:?} at tick {i}"
        );
        match mode(&n) {
            Mode::Mission => {}
            Mode::Loiter => {
                fired = Some((i + 1) as f32 * DT);
                break;
            }
            other => panic!("stream stop must HOLD, not {other:?} (tick {i})"),
        }
    }
    let t = fired.expect("the offboard-timeout failsafe never fired within 5 s");
    assert!(
        t > TIMEOUT_S - 2.0 * DT && t <= TIMEOUT_S + 2.0 * DT,
        "fired after {t:.3} s, not the {TIMEOUT_S} s timeout"
    );
    assert_ne!(
        flags(&n) & (1 << 8),
        0,
        "the cause (bit 8 target-stale) must be published"
    );
    // HOLD means hold: stays in Loiter, inside HOLD-P01's 1.0 m of where the
    // stream stopped, for 60 s.
    let mut ph = 0.0f32;
    for i in 0..(HOLD_S / DT) as u32 {
        frame(&mut n, &mut b, gone);
        assert_eq!(mode(&n), Mode::Loiter, "left the HOLD at tick {i}");
        ph = ph.max(horiz(&b, stop_at));
    }
    eprintln!(
        "A3: stream stopped -> Loiter (HOLD) after {t:.3} s; peak {ph:.3} m from where it \
         stopped over the next {HOLD_S} s; bit 8 set"
    );
    assert!(ph < HOLD_HORIZ_M, "the HOLD wandered {ph:.3} m");
}

/// (A4) A non-finite target is REJECTED, not followed. Two cases:
///   * a Mission entered while the host has only ever sent non-finite targets
///     HOLDS (Loiter) on its first tick: there is nothing fresh to follow;
///   * a 0.5 s burst of NaN / inf frames mid-transit (under the 1.0 s timeout)
///     leaves the last accepted target in force: Mission throughout, motors
///     finite, still closing on it, and once the stream resumes the vehicle
///     reaches and holds it (so nothing non-finite reached the loop's state).
#[test]
fn a4_non_finite_target_is_not_followed() {
    let launch = [3.0, 3.0];
    let hover = [launch[0], launch[1], -CRUISE];

    // Never-finite: a fresh vehicle, the host streaming only NaN.
    let gone = [f32::NAN; 3];
    let (mut b, mut n) = rig(launch, [0.0; 3], gone);
    takeoff(&mut n, &mut b, gone);
    let at = [b.pos[0], b.pos[1]];
    frame(&mut n, &mut b, gone);
    n.command(Event::RequestMission, true, true);
    let m = frame(&mut n, &mut b, gone);
    assert!(m.iter().all(|v| v.is_finite()), "non-finite motors {m:?}");
    assert_eq!(
        mode(&n),
        Mode::Loiter,
        "Mission with no target ever must HOLD at once"
    );
    let mut ph = 0.0f32;
    for _ in 0..(10.0 / DT) as u32 {
        let m = frame(&mut n, &mut b, gone);
        assert!(m.iter().all(|v| v.is_finite()), "non-finite motors {m:?}");
        assert_eq!(mode(&n), Mode::Loiter);
        ph = ph.max(horiz(&b, at));
    }
    eprintln!("A4: Mission with no finite target ever -> Loiter on the first tick; peak {ph:.3} m");
    assert!(ph < HOLD_HORIZ_M);

    // A burst mid-transit.
    let (mut b, mut n) = rig(launch, [0.0; 3], hover);
    takeoff(&mut n, &mut b, hover);
    let target = [3.0, 18.0, -CRUISE];
    start_mission(&mut n, &mut b, target);
    for _ in 0..(2.0 / DT) as u32 {
        frame(&mut n, &mut b, target);
    }
    let before = horiz(&b, [target[0], target[1]]);
    // 0.5 s of garbage — under the 1.0 s timeout.
    let garbage = [
        [f32::NAN, 18.0, -CRUISE],
        [3.0, f32::INFINITY, -CRUISE],
        [3.0, 18.0, f32::NEG_INFINITY],
    ];
    for i in 0..(0.5 / DT) as u32 {
        let m = frame(&mut n, &mut b, garbage[i as usize % 3]);
        assert!(
            m.iter().all(|v| v.is_finite()),
            "non-finite motors {m:?} at tick {i}"
        );
        assert_eq!(
            mode(&n),
            Mode::Mission,
            "a sub-timeout gap must not end Mission"
        );
    }
    let after = horiz(&b, [target[0], target[1]]);
    assert!(
        after < before - 0.5,
        "stopped closing on the last good target"
    );
    // The stream resumes: the vehicle still gets there and holds it.
    let t_reach = fly_to(&mut n, &mut b, target, 1.0, 60.0);
    let _ = follow_window(&mut n, &mut b, target, Mode::Mission, BRAKE_S);
    let (ph, pv) = follow_window(&mut n, &mut b, target, Mode::Mission, 20.0);
    eprintln!(
        "A4: 0.5 s non-finite burst mid-transit: {before:.2} -> {after:.2} m from the target \
         (Mission, finite motors); reached {t_reach:.1} s after; peak over {BRAKE_S}-{} s \
         after: horiz {ph:.3} m, vert {pv:.3} m",
        BRAKE_S + 20.0
    );
    assert!(
        ph < HOLD_HORIZ_M && pv < HOLD_VERT_M,
        "did not hold after the burst"
    );
}

/// (A5) The host ends Mission explicitly: RequestLoiter, RequestRtl and
/// RequestLand each still work FROM A HELD FOLLOW. Pre-fix the vehicle was no
/// longer in Mission to be commanded (it had RTL'd on arrival), which is the
/// only reason this is red there: the transitions themselves are unchanged.
#[test]
fn a5_explicit_commands_end_mission() {
    let launch = [0.0, 0.0];
    let hover = [launch[0], launch[1], -CRUISE];
    let target = [8.0, 4.0, -CRUISE];

    // Loiter: holds where it paused, then back into Mission, then Land.
    let (mut b, mut n) = rig(launch, [0.0; 3], hover);
    takeoff(&mut n, &mut b, hover);
    start_mission(&mut n, &mut b, target);
    fly_to(&mut n, &mut b, target, 1.0, 60.0);
    let _ = follow_window(&mut n, &mut b, target, Mode::Mission, 10.0);
    n.command(Event::RequestLoiter, true, true);
    assert_eq!(mode(&n), Mode::Loiter);
    let (lh, _) = follow_window(&mut n, &mut b, target, Mode::Loiter, 10.0);
    start_mission(&mut n, &mut b, [0.0, 8.0, -CRUISE]);
    let _ = follow_window(&mut n, &mut b, [0.0, 8.0, -CRUISE], Mode::Mission, 5.0);
    n.command(Event::RequestLand, true, true);
    assert_eq!(mode(&n), Mode::Land);
    let mut landed = None;
    for i in 0..(180.0 / DT) as u32 {
        frame(&mut n, &mut b, [0.0, 8.0, -CRUISE]);
        if mode(&n) == Mode::Disarmed {
            landed = Some(i as f32 * DT);
            break;
        }
        assert_eq!(mode(&n), Mode::Land, "left Land at tick {i}");
    }
    let tl = landed.expect("RequestLand never touched down");

    // Rtl: flies home and lands, while the target is still being streamed.
    let (mut b, mut n) = rig([5.0, 5.0], [0.0; 3], [5.0, 5.0, -CRUISE]);
    takeoff(&mut n, &mut b, [5.0, 5.0, -CRUISE]);
    start_mission(&mut n, &mut b, target);
    fly_to(&mut n, &mut b, target, 1.0, 60.0);
    let _ = follow_window(&mut n, &mut b, target, Mode::Mission, 10.0);
    n.command(Event::RequestRtl, true, true);
    assert_eq!(mode(&n), Mode::Rtl);
    let mut done = None;
    for i in 0..(180.0 / DT) as u32 {
        frame(&mut n, &mut b, target);
        let m = mode(&n);
        if m == Mode::Disarmed {
            done = Some(i as f32 * DT);
            break;
        }
        assert!(
            matches!(m, Mode::Rtl | Mode::Land),
            "RTL went to {m:?} at tick {i}"
        );
    }
    let tr = done.expect("RequestRtl never came home and landed");
    let home_err = horiz(&b, [0.0, 0.0]);
    eprintln!(
        "A5: Loiter from a held follow (peak {lh:.3} m from the target in 10 s); Land \
         touched down in {tl:.1} s; RTL home + land in {tr:.1} s, {home_err:.2} m from home"
    );
    assert!(home_err < 1.0, "RTL landed {home_err:.2} m from home");
}
