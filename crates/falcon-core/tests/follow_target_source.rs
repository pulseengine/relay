//! #609 finding 4, Decision A, at the native API: the FOLLOW mission source and
//! its separation from the stored-waypoint mission (MISSION-P01).
//!
//! The behavioural tests a host sees (reach-and-hold, moved target, stream
//! stop, non-finite target, explicit commands) drive the component mirror in
//! `tests/cascade-sitl-wasm/src/follow_target_tests.rs`. These pin the native
//! contract underneath: the freshness rule, the timeout measured on the
//! supervisor clock when the caller simply stops calling, and that the two
//! sources do not leak into each other.

use falcon_core::failsafe_flags::TARGET_STALE;
use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FOLLOW_TARGET_TIMEOUT_US, FlightSupervisor, MissionSource, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn cal() -> falcon_core::relay_calib::CalParams {
    falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    }
}

/// Airborne in Loiter at 2 m over the origin.
fn airborne(src: MissionSource) -> (SimBackend, FlightSupervisor) {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(cal());
    sup.set_mission_source(src);
    for _ in 0..(5.0 / DT) as u32 {
        sup.step(&mut b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..15_000 {
        if sup.mode() == Mode::Loiter {
            break;
        }
        sup.step(&mut b);
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    (b, sup)
}

#[test]
fn the_default_source_is_the_stored_mission() {
    let sup = FlightSupervisor::new([0.0; 3], 50.0, 2.0, 14.0);
    assert_eq!(sup.mission_source(), MissionSource::Waypoints);
}

/// Freshness: a target with any non-finite component is rejected and does not
/// replace the one in force; before any target the stream is stale.
#[test]
fn non_finite_targets_are_rejected() {
    let mut sup = FlightSupervisor::new([0.0; 3], 50.0, 2.0, 14.0);
    sup.set_mission_source(MissionSource::Follow);
    assert_eq!(sup.follow_target(), None, "stale before the first target");
    assert_ne!(sup.failsafe_flags() & TARGET_STALE, 0);
    assert!(sup.set_follow_target([1.0, 2.0, -3.0]));
    assert_eq!(sup.follow_target(), Some([1.0, 2.0, -3.0]));
    assert_eq!(sup.failsafe_flags() & TARGET_STALE, 0);
    for bad in [
        [f32::NAN, 0.0, -2.0],
        [0.0, f32::INFINITY, -2.0],
        [0.0, 0.0, f32::NEG_INFINITY],
    ] {
        assert!(!sup.set_follow_target(bad), "{bad:?} must be rejected");
        assert_eq!(sup.follow_target(), Some([1.0, 2.0, -3.0]));
    }
}

/// The timeout on the supervisor clock, when the caller simply STOPS calling
/// `set_follow_target`: Mission holds (Loiter) on the first tick strictly past
/// FOLLOW_TARGET_TIMEOUT_US, and not one tick sooner.
#[test]
fn a_stopped_follow_stream_holds_after_the_timeout() {
    let (mut b, mut sup) = airborne(MissionSource::Follow);
    let t = [3.0, 0.0, -2.0];
    sup.set_follow_target(t);
    sup.command(Event::RequestMission, true, true);
    assert_eq!(sup.mode(), Mode::Mission);
    for _ in 0..(3.0 / DT) as u32 {
        sup.set_follow_target(t);
        sup.step(&mut b);
        assert_eq!(sup.mode(), Mode::Mission);
    }
    // Stop calling. The last accept was stamped at the clock BEFORE the last
    // step, so the stream is stale once more than the timeout has elapsed
    // since then.
    let limit = (FOLLOW_TARGET_TIMEOUT_US as f32 / 1.0e6 / DT).round() as u32;
    let mut ticks = 0u32;
    while sup.mode() == Mode::Mission {
        sup.step(&mut b);
        ticks += 1;
        assert!(ticks <= limit + 2, "never held");
    }
    assert_eq!(
        sup.mode(),
        Mode::Loiter,
        "the stale-offboard action is HOLD"
    );
    assert_ne!(sup.failsafe_flags() & TARGET_STALE, 0);
    // Silent tick j has the clock (1 + j) ticks past the last accept, stale
    // once that is STRICTLY over the timeout: j = limit exactly.
    assert_eq!(
        ticks, limit,
        "held after {ticks} silent ticks; the timeout is {limit} ticks"
    );
}

/// The two sources do not leak: under the stored mission a follow target is
/// ignored and arrival still completes the sortie with RTL (MISSION-P01).
#[test]
fn the_stored_mission_ignores_follow_targets_and_still_completes() {
    let (mut b, mut sup) = airborne(MissionSource::Waypoints);
    sup.set_mission([4.0, 0.0, -2.0]);
    sup.command(Event::RequestMission, true, true);
    let mut rtl = false;
    for _ in 0..(40.0 / DT) as u32 {
        sup.set_follow_target([-20.0, 0.0, -2.0]); // must change nothing
        sup.step(&mut b);
        if sup.mode() == Mode::Rtl {
            rtl = true;
            break;
        }
        assert_eq!(sup.mode(), Mode::Mission);
        assert!(b.pos[0] > -1.0, "flew toward the follow target");
    }
    assert!(rtl, "the one-leg mission must still end in RTL on arrival");
    assert_eq!(
        sup.failsafe_flags() & TARGET_STALE,
        0,
        "bit 8 is follow-only"
    );
}

/// ... and under Follow a stored mission is ignored: `set_mission` does not
/// switch the source back (the selector is explicit).
#[test]
fn follow_ignores_the_stored_mission() {
    let (mut b, mut sup) = airborne(MissionSource::Follow);
    sup.set_mission([4.0, 0.0, -2.0]);
    let t = [0.0, 4.0, -2.0];
    sup.set_follow_target(t);
    sup.command(Event::RequestMission, true, true);
    let (mut closest_t, mut closest_wp) = (f32::MAX, f32::MAX);
    for _ in 0..(30.0 / DT) as u32 {
        sup.set_follow_target(t);
        sup.step(&mut b);
        assert_eq!(sup.mode(), Mode::Mission, "arrival must end nothing");
        closest_t = closest_t.min(((b.pos[0] - t[0]).powi(2) + (b.pos[1] - t[1]).powi(2)).sqrt());
        closest_wp = closest_wp.min(((b.pos[0] - 4.0).powi(2) + b.pos[1].powi(2)).sqrt());
    }
    assert_eq!(sup.mission_source(), MissionSource::Follow);
    // Reached the follow target (within MISSION's 1.2 m acceptance radius, which
    // would have ended a stored mission) and never went near the stored leg.
    assert!(
        closest_t < 0.5,
        "did not follow the target: closest {closest_t:.2} m"
    );
    assert!(
        closest_wp > 3.0,
        "flew to the stored leg: closest {closest_wp:.2} m"
    );
}
