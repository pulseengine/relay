//! #609 finding 8: a motors-off tick must tell the estimator the vehicle is
//! NOT flying.
//!
//! THE DEFECT. `FlightSupervisor::step` runs `FlightCore::step_estimate_only`
//! for Disarmed / Armed / Terminated, and only `FlightCore::step` ever wrote
//! the estimator's `in_flight` flag (from the cascade's previous collective).
//! A supervised touchdown fires at AGL < 0.15 m while the landing law is still
//! commanding near-hover collective, so the flag was left TRUE on the ground,
//! and the accelerometer gravity update (#452/#483: admitted only while not
//! flying) stayed suppressed for as long as the vehicle sat there. With no
//! GNSS and a small gyro-bias drift the published attitude then walked off a
//! level, stationary airframe.
//!
//! THE CONTRACT. `step_estimate_only` commands zero motors, so on every such
//! tick the vehicle is, by construction, not under thrust: the estimator is
//! told so, exactly as `FlightCore::step` tells it from the previous command.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightBackend, FlightSupervisor, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn cal() -> falcon_core::relay_calib::CalParams {
    falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    }
}

/// The true tilt (rad) of the plant's body z axis from vertical.
fn true_tilt(b: &SimBackend) -> f32 {
    b.r[2][2].clamp(-1.0, 1.0).acos()
}

#[test]
fn a_grounded_vehicle_after_a_supervised_touchdown_keeps_a_level_estimate() {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.rot_drag = 0.02;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(cal());
    let mut n: u32 = 0;
    let mut tick = |sup: &mut FlightSupervisor, b: &mut SimBackend| {
        sup.step(b);
        n += 1;
        n
    };
    for _ in 0..(5.0 / DT) as u32 {
        tick(&mut sup, &mut b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "{:?}", sup.arm_blocked_check());
    for _ in 0..15_000 {
        tick(&mut sup, &mut b);
        if sup.mode() == Mode::Loiter {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Loiter, "never reached Loiter");
    for _ in 0..(5.0 / DT) as u32 {
        tick(&mut sup, &mut b);
    }
    sup.command(Event::RequestLand, true, false);
    let mut now = 0;
    for _ in 0..30_000 {
        now = tick(&mut sup, &mut b);
        if sup.mode() == Mode::Disarmed {
            break;
        }
    }
    assert_eq!(sup.mode(), Mode::Disarmed, "never touched down");

    // On the ground, motors off: GNSS lost, and a small gyro-bias ramp.
    b.path.gps_dropout_start = now;
    b.path.gps_dropout_len = u32::MAX / 2;
    b.path.gyro_bias_drift = 0.0004;
    assert!(b.read_position().is_none(), "GNSS must be off from here");

    let mut worst = 0.0f32;
    for _ in 0..(60.0 / DT) as u32 {
        tick(&mut sup, &mut b);
        let err = (sup.core().state().tilt_rad() - true_tilt(&b)).abs();
        worst = worst.max(err);
    }
    eprintln!(
        "60 s grounded after touchdown: worst |tilt_est - tilt_true| = {worst:.4} rad, \
         true tilt {:.4} rad",
        true_tilt(&b)
    );
    assert!(
        worst < 0.05,
        "the estimate walked off a level, grounded vehicle: {worst:.4} rad"
    );
}
