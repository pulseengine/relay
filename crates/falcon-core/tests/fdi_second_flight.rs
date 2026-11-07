//! #609 finding 16: the rotor-out FDI must be re-armed for every flight.
//!
//! THE DEFECT. The FDI's spin-up guard counted `CascadePartition::step_count`,
//! which is set only at construction, and its residual compares the achieved
//! RPM against `last_motor_cmd`, the previous control tick's command, which
//! nothing resets on the ground. A second flight in the same process therefore
//! spun up with no guard and against the LAST command of the previous flight:
//! a healthy rotor could be isolated on tick 0 of the second takeoff, which
//! drives the rotor-out Land.
//!
//! What must still hold: a GENUINELY isolated rotor stays isolated and blocks
//! re-arming (#565, `MotorsHealthy`; pinned by
//! `a_rotor_fault_isolated_during_landing_blocks_re_arming` and
//! `a_rotor_that_failed_in_flight_still_blocks_re_arming_after_the_latch_clears`
//! in `plain/src/tests.rs`), and a real failure in the second flight is still
//! detected as fast as in the first.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightSupervisor, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

fn cal() -> falcon_core::relay_calib::CalParams {
    falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    }
}

/// A calibrated supervisor on a plant with ESC RPM telemetry (the FDI's
/// source), estimator warmed on the ground.
fn rig() -> (SimBackend, FlightSupervisor) {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.rot_drag = 0.02;
    b.rpm_telemetry = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 14.0);
    sup.set_calibration(cal());
    for _ in 0..(5.0 / DT) as u32 {
        sup.step(&mut b);
    }
    (b, sup)
}

/// Arm, take off and reach Loiter, asserting NO rotor is isolated on the way.
///
/// The whole flight is flown with a first-order ESC/rotor lag (50 ms): the
/// achieved RPM trails the command at spin-up, the transient the spin-up guard
/// exists for. The lag stays on through Loiter and the landing. Until #609
/// finding 17 it had to be switched off in Loiter: the collective step when
/// the landing law engages (a few ticks into Land, ~2 m up, not at touchdown)
/// left all four rotors equally behind and the FDI isolated rotor 0. The
/// residual now rejects that common mode (tests/fdi_common_mode.rs).
fn takeoff_healthy(sup: &mut FlightSupervisor, b: &mut SimBackend, flight: u32) {
    b.motor_tau = 0.05;
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(
        sup.mode(),
        Mode::Takeoff,
        "flight {flight}: arm/takeoff refused ({:?})",
        sup.arm_blocked_check()
    );
    for i in 0..15_000u32 {
        sup.step(b);
        assert_eq!(
            sup.core().failed_motor(),
            None,
            "flight {flight}: a healthy rotor was isolated at takeoff tick {i} (mode {:?})",
            sup.mode()
        );
        if sup.mode() == Mode::Loiter {
            return;
        }
        assert_eq!(
            sup.mode(),
            Mode::Takeoff,
            "flight {flight}: left Takeoff at tick {i}"
        );
    }
    panic!("flight {flight}: never reached Loiter");
}

/// Hold `secs` in Loiter, healthy.
fn loiter(sup: &mut FlightSupervisor, b: &mut SimBackend, secs: f32, flight: u32) {
    for i in 0..(secs / DT) as u32 {
        sup.step(b);
        assert_eq!(
            sup.mode(),
            Mode::Loiter,
            "flight {flight}: left Loiter at tick {i}"
        );
        assert_eq!(
            sup.core().failed_motor(),
            None,
            "flight {flight}: phantom isolation"
        );
    }
}

/// Land and disarm, healthy, then sit on the ground.
fn land(sup: &mut FlightSupervisor, b: &mut SimBackend, flight: u32) {
    sup.command(Event::RequestLand, true, false);
    for _ in 0..30_000 {
        sup.step(b);
        assert_eq!(
            sup.core().failed_motor(),
            None,
            "flight {flight}: phantom isolation landing"
        );
        if sup.mode() == Mode::Disarmed {
            for _ in 0..(5.0 / DT) as u32 {
                sup.step(b);
            }
            return;
        }
    }
    panic!("flight {flight}: never disarmed");
}

/// Kill `rotor` in Loiter and return the control ticks to isolation.
fn ticks_to_isolate(sup: &mut FlightSupervisor, b: &mut SimBackend, rotor: usize) -> u32 {
    b.fail_rotor(rotor);
    for i in 1..=5_000u32 {
        sup.step(b);
        if let Some(r) = sup.core().failed_motor() {
            assert_eq!(r, rotor, "isolated the wrong rotor");
            return i;
        }
    }
    panic!("rotor {rotor} never isolated");
}

/// Fly, land, disarm, re-arm, take off: no phantom isolation, three flights.
#[test]
fn a_second_takeoff_does_not_isolate_a_healthy_rotor() {
    let (mut b, mut sup) = rig();
    for flight in 1..=3 {
        takeoff_healthy(&mut sup, &mut b, flight);
        loiter(&mut sup, &mut b, 10.0, flight);
        land(&mut sup, &mut b, flight);
    }
}

/// A real rotor failure in the SECOND flight is detected as fast as the same
/// failure in a first flight (the FDI is re-armed, not disabled).
#[test]
fn a_real_failure_in_the_second_flight_is_still_detected() {
    let (mut b1, mut s1) = rig();
    takeoff_healthy(&mut s1, &mut b1, 1);
    loiter(&mut s1, &mut b1, 10.0, 1);
    let first = ticks_to_isolate(&mut s1, &mut b1, 2);

    let (mut b2, mut s2) = rig();
    takeoff_healthy(&mut s2, &mut b2, 1);
    loiter(&mut s2, &mut b2, 10.0, 1);
    land(&mut s2, &mut b2, 1);
    takeoff_healthy(&mut s2, &mut b2, 2);
    loiter(&mut s2, &mut b2, 10.0, 2);
    let second = ticks_to_isolate(&mut s2, &mut b2, 2);
    eprintln!(
        "rotor 2 killed in Loiter: isolated after {first} ticks ({:.3} s) in a first flight, \
         {second} ticks ({:.3} s) in a second",
        first as f32 * DT,
        second as f32 * DT
    );
    assert!(
        second <= first + first / 10 + 2,
        "second-flight detection {second} ticks vs first-flight {first}"
    );
    // ... and the isolation still drives the rotor-out Land.
    s2.step(&mut b2);
    assert_eq!(s2.mode(), Mode::Land);
}

/// Re-arming the FDI must NOT forget a rotor it has isolated (#565).
///
/// Through the supervisor this cannot be reached today: `MotorsHealthy`
/// refuses the arm, so a vehicle with a latched dead motor never gets the
/// motors-off -> flying edge that calls `rearm_fdi`. The lib tests
/// `a_rotor_fault_isolated_during_landing_blocks_re_arming` and
/// `a_rotor_that_failed_in_flight_still_blocks_re_arming_after_the_latch_clears`
/// therefore pass even with a `rearm_fdi` that clears the fault (mutation-
/// tested). This pins the contract on `FlightCore` directly, so the arm gate
/// is not the only thing standing between a dead motor and a healthy-looking
/// core.
#[test]
fn rearming_the_fdi_keeps_an_isolated_rotor_isolated() {
    let mut b = SimBackend::new(LEVEL, DT);
    b.rpm_telemetry = true;
    b.pos = [0.0, 0.0, -2.0];
    let mut core = falcon_core::FlightCore::new(0.5, 1000.0);
    core.set_calibration(cal());
    core.set_position([0.0, 0.0, -2.0]);
    for _ in 0..(3.0 / DT) as u32 {
        core.step(&mut b);
    }
    assert_eq!(core.failed_motor(), None, "healthy hover first");
    b.fail_rotor(3);
    for _ in 0..2_000 {
        core.step(&mut b);
        if core.failed_motor().is_some() {
            break;
        }
    }
    assert_eq!(
        core.failed_motor(),
        Some(3),
        "the dead rotor must be isolated"
    );
    core.rearm_fdi();
    assert_eq!(
        core.failed_motor(),
        Some(3),
        "rearm_fdi forgot an isolated rotor"
    );
    for _ in 0..500 {
        core.step(&mut b);
    }
    assert_eq!(
        core.failed_motor(),
        Some(3),
        "isolation must stay latched after a re-arm"
    );
}
