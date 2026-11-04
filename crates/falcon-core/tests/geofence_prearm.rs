//! #609 finding 5 (geofence half): being OUTSIDE the geofence is a LIVE
//! hazard, judged every cycle, and it refuses arming through its own row.
//!
//! THE DEFECT. SUPV-P02 clears the RTL latch on disarm and says the LIVE
//! hazard then blocks re-arming, but no pre-arm row checked the fence. With
//! home 20 m away and a 5 m fence, a vehicle that breached, was landed by
//! the pilot still outside the fence and disarmed, re-armed with no blocking
//! row while `failsafe_flags()` read 0x10 (GEOFENCE_BREACH). A first arm
//! outside the fence was permitted too.
//!
//! THE CONTRACT (maintainer decision 2026-11-04): a dedicated row,
//! `CheckId::InsideGeofence`, passes only while the estimated horizontal
//! distance from home is within the fence radius: the complement of the
//! in-flight breach term, the same quantity. A first arm outside the fence is
//! refused as well.
//!
//! The row is asserted by its Debug name so this file also compiles against
//! the pre-fix tree, where it fails.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightSupervisor, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn rig(home: [f32; 3], fence: f32) -> (SimBackend, FlightSupervisor) {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.rot_drag = 0.02;
    let mut sup = FlightSupervisor::new(home, fence, 2.0, 14.0);
    sup.set_calibration(falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    });
    for _ in 0..2500 {
        sup.step(&mut b);
    }
    (b, sup)
}

/// Carry the disarmed vehicle to north = `x` at 1 m/s, then let it sit.
fn carry_to(sup: &mut FlightSupervisor, b: &mut SimBackend, x: f32) {
    for _ in 0..60_000 {
        let d = x - b.pos[0];
        if d.abs() <= DT {
            break;
        }
        b.pos[0] += d.signum() * DT;
        sup.step(b);
    }
    for _ in 0..2500 {
        sup.step(b);
    }
}

fn blocked(sup: &FlightSupervisor) -> String {
    format!("{:?}", sup.arm_blocked_check())
}

/// Inside the fence: arms (the control).
#[test]
fn inside_the_fence_arms() {
    let (_b, mut sup) = rig([0.0, 0.0, 0.0], 5.0);
    sup.command(Event::Arm, true, true);
    assert_eq!(sup.mode(), Mode::Armed, "{}", blocked(&sup));
}

/// A FIRST arm outside the fence is refused, naming the fence row.
#[test]
fn a_first_arm_outside_the_fence_is_refused_by_the_fence_row() {
    let (_b, mut sup) = rig([20.0, 0.0, 0.0], 5.0);
    assert_eq!(blocked(&sup), "Some(InsideGeofence)");
    sup.command(Event::Arm, true, true);
    assert_eq!(sup.mode(), Mode::Disarmed, "armed outside the fence");
}

/// THE SCENARIO OF THE FINDING, as it can still arise now that a first arm
/// outside the fence is refused. Armed inside the fence, the vehicle is
/// carried out of it in flight (the breach latches an RTL), flies home, lands
/// and disarms; the in-flight latch clears (SUPV-P02). On the ground it is
/// then OUTSIDE the fence again (carried there, or its fix has walked):
/// re-arming is refused, and the fence row is the DECIDING row, the only
/// failing one. Brought back inside, it arms: the row is live, not latched.
#[test]
fn re_arming_outside_the_fence_after_a_breach_is_refused_by_the_fence_row() {
    let (mut b, mut sup) = rig([0.0, 0.0, 0.0], 1.5);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "{}", blocked(&sup));
    for _ in 0..15_000 {
        sup.step(&mut b);
        if sup.mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    // Out of the 1.5 m fence in flight: pushed north at 1 m/s for 8 s, as a
    // gust would (a jump would be rejected by the innovation gate).
    let mut rtl = false;
    for _ in 0..(8.0 / DT) as u32 {
        b.pos[0] += DT;
        sup.step(&mut b);
        rtl |= sup.mode() == Mode::Rtl;
    }
    for _ in 0..60_000 {
        sup.step(&mut b);
        rtl |= sup.mode() == Mode::Rtl;
        if sup.mode() == Mode::Disarmed {
            break;
        }
    }
    assert!(rtl, "the breach did not latch an RTL");
    assert_eq!(sup.mode(), Mode::Disarmed, "RTL never landed");
    for _ in 0..2500 {
        sup.step(&mut b);
    }
    assert_eq!(
        sup.failsafe_flags() & falcon_core::failsafe_flags::RTL_LATCHED,
        0,
        "the in-flight latch must have cleared on disarm (SUPV-P02)"
    );
    assert_eq!(blocked(&sup), "None", "home, inside the fence: must arm");

    // On the ground, carried 12 m north at walking pace (a jump
    // would be rejected by the estimator's innovation gate, as it should).
    carry_to(&mut sup, &mut b, 12.0);
    assert_ne!(
        sup.failsafe_flags() & falcon_core::failsafe_flags::GEOFENCE_BREACH,
        0
    );
    assert_eq!(blocked(&sup), "Some(InsideGeofence)");
    sup.command(Event::Arm, true, true);
    assert_eq!(sup.mode(), Mode::Disarmed, "re-armed outside the fence");

    // Back inside: the row clears by itself.
    carry_to(&mut sup, &mut b, 0.0);
    assert_eq!(blocked(&sup), "None");
    sup.command(Event::Arm, true, true);
    assert_eq!(sup.mode(), Mode::Armed);
}
