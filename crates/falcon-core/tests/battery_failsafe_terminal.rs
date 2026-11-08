//! #609 finding 5 (battery half), DOCUMENTATION PIN: a battery failsafe is
//! TERMINAL until a process restart (SWREQ-FALCON-SUPV-P02, "BATTERY IS
//! TERMINAL"; maintainer decision 2026-11-04, falcon-v1.141.0).
//!
//! relay-batt's low/critical flags latch until `BatteryEstimator::reset()`,
//! which nothing calls, and the seam refuses a rebuild over a latched pack
//! (`SupervisorHost::rebuild_permitted`). So after a battery failsafe a FRESH
//! pack does not make the vehicle armable again in the same process. This is
//! the behaviour the requirement now states; the test pins it so the
//! statement cannot drift. When the "pack replaced" reset (#650) lands, this
//! test is where its contract changes.

use falcon_core::host::{Configured, SupervisorConfig, SupervisorHost};
use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightSupervisor, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

#[test]
fn a_battery_failsafe_is_not_cleared_by_a_fresh_pack() {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.rot_drag = 0.02;
    b.battery_v = 16.8;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    for _ in 0..2500 {
        sup.step(&mut b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..15_000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    // The pack sags below LOW in flight: failsafe, land, disarm.
    b.battery_v = 13.2;
    let mut failsafed = false;
    for _ in 0..60_000 {
        sup.step(&mut b);
        failsafed |= sup.battery().low || sup.battery().critical;
        if sup.mode() == Mode::Disarmed {
            break;
        }
    }
    assert!(failsafed, "the sag did not trip the battery failsafe");
    assert_eq!(sup.mode(), Mode::Disarmed);
    // A FRESH pack, for 30 s on the ground.
    b.battery_v = 16.8;
    for _ in 0..(30.0 / DT) as u32 {
        sup.step(&mut b);
    }
    let bs = sup.battery();
    assert!(bs.soc > 0.9, "the fresh pack reads full (soc {})", bs.soc);
    assert!(
        bs.low || bs.critical,
        "the latch cleared by itself: the requirement's TERMINAL statement is now false"
    );
    sup.command(Event::Arm, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Disarmed,
        "re-armed after a battery failsafe"
    );
    assert_eq!(
        format!("{:?}", sup.arm_blocked_check()),
        "Some(BatteryOk)",
        "the refusal must name the battery"
    );
}

/// The seam cannot launder it either: a reconfigure over a latched pack is
/// REJECTED (it would rebuild the estimator with clear flags).
#[test]
fn the_seam_refuses_a_rebuild_over_a_latched_pack() {
    // `rebuild_guard_rejects_a_latched_battery` in host.rs pins the guard;
    // this pins the end-to-end consequence through `SupervisorHost`.
    let mut h: SupervisorHost = SupervisorHost::new();
    let cfg = SupervisorConfig {
        home: [0.0, 0.0, 0.0],
        fence_radius: 50.0,
        cruise_alt: 2.0,
        low_batt_v: 14.0,
    };
    assert_eq!(h.configure_supervisor(cfg), Configured::Accepted);
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.battery_v = 13.0; // below LOW from the start: latches on the ground
    for _ in 0..(10.0 / DT) as u32 {
        h.step([f32::NAN; 3], &mut b);
    }
    let latched = h.supervisor().map(|s| s.battery()).unwrap();
    assert!(latched.low || latched.critical, "the pack did not latch");
    b.battery_v = 16.8;
    for _ in 0..2500 {
        h.step([f32::NAN; 3], &mut b);
    }
    assert_eq!(
        h.configure_supervisor(cfg),
        Configured::Rejected,
        "a reconfigure rebuilt the estimator over a latched pack"
    );
}
