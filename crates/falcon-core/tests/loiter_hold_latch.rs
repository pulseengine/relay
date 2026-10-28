//! #609 finding 4: Takeoff and Loiter must HOLD a horizontal point, not chase
//! the estimate.
//!
//! THE DEFECT. `FlightSupervisor::step` derived the Takeoff / Loiter setpoint
//! as `[est.p[0], est.p[1], -cruise_alt]`, recomputed from the CURRENT estimate
//! every tick. The horizontal error the position loop sees was therefore zero
//! by construction, its integral never charged, and nothing resisted wind: the
//! v1.141 review measured 11.7 m of drift in 30 s at 2 m/s and 59 m in 60 s at
//! 5 m/s on `SimBackend`, against 0.5-0.9 m on the @0.11.0 `set_position` path.
//!
//! THE RULE (maintainer decision 2026-10-28, "latch on entry"): the horizontal
//! hold point is captured from the estimate on the tick the supervisor first
//! derives a Takeoff / Loiter setpoint, and held until the vehicle leaves the
//! Takeoff/Loiter pair. Takeoff -> Loiter keeps the takeoff point. Every other
//! way into Loiter (Mission -> Loiter) re-latches where the vehicle is.
//!
//! Every test here judges PLANT TRUTH (`SimBackend::pos`), takes the PEAK over
//! its window rather than the final value, and asserts the mode on every tick
//! of the window, so a failsafe RTL to a home that happens to coincide with the
//! hold point cannot pass for the wrong reason. Home is never the launch point.
//!
//! WHAT THESE TESTS DO NOT CLAIM. Wind switched on at takeoff is a STEP
//! disturbance, and the core's position loop answers it slowly (its integral
//! takes tens of seconds to charge; see the `falcon-position-loop-bandwidth`
//! note in the HOLD-P01 history). Measured on this plant, the bare `FlightCore`
//! commanded by `set_position` -- the @0.11.0 path the review compared against
//! -- peaks 2.0 m at 2 m/s and 5.0 m at 5 m/s in the first 10 s and is inside
//! HOLD-P01's 1.0 m from about 50 s on. The latched supervisor reproduces that
//! response to within a few centimetres. The transient is a cascade-tuning
//! property of the core and is NOT changed here; these tests pin that the
//! supervisor now holds a point at all, and judge HOLD-P01's 60 s window once
//! the integral has charged ([`SETTLE_S`]).

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightCore, FlightSupervisor, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const CRUISE: f32 = 2.0;

/// SWREQ-FALCON-HOLD-P01's bars, read from the requirement, not chosen here:
/// "within 0.5 m vertically and 1.0 m horizontally of the setpoint for at least
/// 60 s". The requirement states them on the reference Gazebo plant; these
/// tests apply the same numbers to the analytic plant.
const HOLD_HORIZ_M: f32 = 1.0;
const HOLD_VERT_M: f32 = 0.5;
const HOLD_S: f32 = 60.0;

/// Time in Loiter before the HOLD-P01 window opens: the integral's charge time
/// after a wind step, measured above on the bare core (inside 1.0 m from ~50 s
/// at 5 m/s), not tuned to make these tests pass.
const SETTLE_S: f32 = 50.0;

/// Time in Loiter after a Mission pause before (c) judges it: the transit
/// leaves the position integral clamped toward the waypoint, and unwinding that
/// is slower than charging it from rest (measured: 8.2 m overshoot, still
/// 1.1 m out at 50-70 s after a 3 s transit).
const BRAKE_S: f32 = 90.0;

fn horiz(b: &SimBackend, at: [f32; 2]) -> f32 {
    let dx = b.pos[0] - at[0];
    let dy = b.pos[1] - at[1];
    (dx * dx + dy * dy).sqrt()
}

/// Calibrated, so a refusal to arm cannot be CalibrationPresent (the same
/// fixture the supervised rotor-out tests in `plain/src/lib.rs` use).
fn cal() -> falcon_core::relay_calib::CalParams {
    falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    }
}

/// Ground time with the wind OFF until the estimator has converged on the
/// launch fix, position AND velocity. The estimator boots at the NED origin, so
/// a launch point away from it is first seen as a jump the filter absorbs
/// partly as velocity: two seconds left it reading 1.6 m/s on a vehicle at
/// rest, which flew the takeoff away from the launch point on its own and would
/// have been charged to the latch. `SimBackend`'s ground contact clamps only
/// the vertical, so with wind on a vehicle with its motors off slides across a
/// frictionless floor and the "launch point" would be wherever it had slid to;
/// the wind is switched back on when this returns, just before takeoff.
fn ground_warm(sup: &mut FlightSupervisor, b: &mut SimBackend) {
    let wind = b.wind;
    b.wind = [0.0; 3];
    b.vel = [0.0; 3];
    for _ in 0..(20.0 / DT) as u32 {
        sup.step(b); // Disarmed / Armed: estimator warm, motors off
    }
    let e = sup.state();
    let perr = ((e.p[0] - b.pos[0]).powi(2) + (e.p[1] - b.pos[1]).powi(2)).sqrt();
    let vmag = (e.v[0] * e.v[0] + e.v[1] * e.v[1]).sqrt();
    assert!(
        perr < 0.05 && vmag < 0.05,
        "estimator not converged on the ground: {perr:.3} m, {vmag:.3} m/s"
    );
    b.wind = wind;
}

/// A plant at rest on the ground at `launch` (NED, z = 0), a supervisor whose
/// home is `home`, the estimator converged, the wind on.
fn rig(
    home: [f32; 3],
    launch: [f32; 2],
    wind: [f32; 3],
    cruise: f32,
) -> (SimBackend, FlightSupervisor) {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.pos = [launch[0], launch[1], 0.0];
    b.wind = wind;
    let mut sup = FlightSupervisor::new(home, 50.0, cruise, 14.0);
    sup.set_calibration(cal());
    ground_warm(&mut sup, &mut b);
    (b, sup)
}

/// Step until `sup` reaches `mode`, asserting it never leaves `allowed` on the
/// way. Returns the ticks taken.
fn step_until(
    sup: &mut FlightSupervisor,
    b: &mut SimBackend,
    mode: Mode,
    allowed: &[Mode],
    max: u32,
) -> u32 {
    for i in 0..max {
        if sup.mode() == mode {
            return i;
        }
        assert!(
            allowed.contains(&sup.mode()),
            "left the expected modes: {:?} at tick {i}",
            sup.mode()
        );
        sup.step(b);
    }
    panic!(
        "never reached {mode:?} in {max} ticks (stuck in {:?})",
        sup.mode()
    );
}

/// Fly `secs` in `mode`, asserting the mode on EVERY tick, and return the peak
/// horizontal distance from `at` and the peak vertical error from `cruise`.
fn hold_window(
    sup: &mut FlightSupervisor,
    b: &mut SimBackend,
    mode: Mode,
    at: [f32; 2],
    cruise: f32,
    secs: f32,
) -> (f32, f32) {
    let (mut ph, mut pv) = (0.0f32, 0.0f32);
    for i in 0..(secs / DT) as u32 {
        sup.step(b);
        assert_eq!(sup.mode(), mode, "mode changed at tick {i} of the window");
        ph = ph.max(horiz(b, at));
        pv = pv.max((-b.pos[2] - cruise).abs());
    }
    (ph, pv)
}

/// (1) Loiter under steady wind holds HOLD-P01's figures for 60 s, at the two
/// wind speeds the review probed (2 m/s, and 5 m/s from the north-east).
///
/// On the pre-fix supervisor this is the review's finding: the vehicle drifts
/// downwind until it breaches the 50 m geofence, and the per-tick mode check
/// fails on the failsafe RTL during the settle window.
#[test]
fn loiter_holds_the_takeoff_point_under_steady_wind() {
    for wind in [[2.0, 0.0, 0.0], [3.0, 4.0, 0.0]] {
        let launch = [6.0, -4.0];
        let (mut b, mut sup) = rig([0.0, 0.0, 0.0], launch, wind, CRUISE);
        sup.command(Event::Arm, true, true);
        sup.command(Event::RequestTakeoff, true, true);
        assert_eq!(sup.mode(), Mode::Takeoff);
        step_until(&mut sup, &mut b, Mode::Loiter, &[Mode::Takeoff], 15_000);
        let (settle_h, _) = hold_window(&mut sup, &mut b, Mode::Loiter, launch, CRUISE, SETTLE_S);
        let (ph, pv) = hold_window(&mut sup, &mut b, Mode::Loiter, launch, CRUISE, HOLD_S);
        eprintln!(
            "wind {wind:?}: settle-window peak {settle_h:.3} m; HOLD-P01 60 s window \
             peak horiz {ph:.3} m, peak vert {pv:.3} m"
        );
        assert!(
            ph < HOLD_HORIZ_M,
            "Loiter under {wind:?} m/s wind left the takeoff point by {ph:.3} m \
             (HOLD-P01: {HOLD_HORIZ_M} m) -- the hold point is not being held"
        );
        assert!(
            pv < HOLD_VERT_M,
            "Loiter under {wind:?} lost altitude by {pv:.3} m (HOLD-P01: {HOLD_VERT_M} m)"
        );
    }
}

/// (2) Takeoff under wind does not walk away.
///
/// The reference is MEASURED, not chosen: the bare `FlightCore` commanded by
/// `set_position` to the same point on the same plant -- the @0.11.0 path, and
/// the one the review called correct. The supervised climb must stay within
/// 10% (+5 cm) of that excursion throughout, and both must end the climb over
/// the launch point the same way. A tall climb (10 m) so the wind has time to
/// act. Pre-fix, the supervised climb drifts 23 m against the reference's 5 m.
#[test]
fn takeoff_under_wind_does_not_walk_away() {
    let launch = [6.0, -4.0];
    let wind = [3.0, 4.0, 0.0];
    let cruise = 10.0;
    let (mut b, mut sup) = rig([0.0, 0.0, 0.0], launch, wind, cruise);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff);
    let mut peak = 0.0f32;
    let mut ticks = 0u32;
    while sup.mode() == Mode::Takeoff {
        sup.step(&mut b);
        peak = peak.max(horiz(&b, launch));
        ticks += 1;
        assert!(ticks < 50_000, "takeoff never completed");
    }
    assert_eq!(sup.mode(), Mode::Loiter, "takeoff must end in Loiter");
    let at_loiter = horiz(&b, launch);

    // The reference: same plant, same core construction and calibration as
    // the supervisor's, same ground time, same tick count.
    let mut rb = SimBackend::new(LEVEL, DT);
    rb.ground_contact = true;
    rb.pos = [launch[0], launch[1], 0.0];
    let mut core = FlightCore::new(0.5, 1000.0);
    core.set_calibration(cal());
    for _ in 0..(20.0 / DT) as u32 {
        core.step_estimate_only(&mut rb);
    }
    rb.wind = wind;
    core.set_position([launch[0], launch[1], -cruise]);
    let mut ref_peak = 0.0f32;
    for _ in 0..ticks {
        core.step(&mut rb);
        ref_peak = ref_peak.max(horiz(&rb, launch));
    }
    let ref_end = horiz(&rb, launch);
    eprintln!(
        "takeoff to {cruise} m under 5 m/s wind ({:.1} s): supervised peak {peak:.3} m, \
         {at_loiter:.3} m off at Loiter entry; bare-core reference peak {ref_peak:.3} m, \
         {ref_end:.3} m off",
        ticks as f32 * DT
    );
    assert!(
        peak <= ref_peak * 1.10 + 0.05,
        "the supervised climb walked {peak:.3} m from the launch point; the core \
         holding that point walks {ref_peak:.3} m"
    );
    assert!(
        at_loiter <= ref_end * 1.10 + 0.05,
        "the climb ended {at_loiter:.3} m off the launch point; the reference ends \
         {ref_end:.3} m off"
    );
}

/// (3) The latch rules, pinned with home != launch so stale-latch, fresh-latch
/// and hold-home are distinguishable:
///
///   a. Armed -> Takeoff latches the launch point (not home);
///   b. Takeoff -> Loiter KEEPS it (holds the launch point, not where the
///      wind-blown climb ended);
///   c. Mission -> Loiter re-latches where the vehicle is when Loiter is
///      entered (not the waypoint, not the takeoff point);
///   d. after the flight ends (RTL -> Land -> Disarmed), a new Takeoff latches
///      afresh at its own launch point (not the stale Mission-pause point).
///
/// Rtl -> Loiter and Land -> Loiter are not transitions `relay_fsm` has, and
/// every failsafe lands in Rtl / Land / Terminated, so (d) is also the only way
/// back into Loiter after a failsafe.
#[test]
fn hold_point_latch_rules_across_mode_transitions() {
    let home = [0.0, 0.0, 0.0];
    let launch = [8.0, 0.0];
    let wind = [0.0, 2.0, 0.0];
    let (mut b, mut sup) = rig(home, launch, wind, CRUISE);

    // a + b: takeoff, loiter, and the vehicle holds the LAUNCH point.
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    let climb_ticks = step_until(&mut sup, &mut b, Mode::Loiter, &[Mode::Takeoff], 15_000);
    let top_of_climb = [b.pos[0], b.pos[1]];
    let _ = hold_window(&mut sup, &mut b, Mode::Loiter, launch, CRUISE, SETTLE_S);
    let (ph, _) = hold_window(&mut sup, &mut b, Mode::Loiter, launch, CRUISE, 20.0);
    eprintln!(
        "a/b: climb {:.1} s ended {:.2} m off launch; Loiter peak {ph:.3} m from the \
         launch point",
        climb_ticks as f32 * DT,
        ((top_of_climb[0] - launch[0]).powi(2) + (top_of_climb[1] - launch[1]).powi(2)).sqrt()
    );
    assert!(
        ph < HOLD_HORIZ_M,
        "Loiter must hold the takeoff point: {ph:.3} m"
    );

    // c: start a mission leg 20 m east, pause it 5 s in with RequestLoiter.
    let wp = [launch[0], launch[1] + 20.0, -CRUISE];
    sup.set_mission(wp);
    sup.command(Event::RequestMission, true, true);
    assert_eq!(sup.mode(), Mode::Mission);
    for _ in 0..(5.0 / DT) as u32 {
        sup.set_mission(wp); // the component re-sends the frame target every tick
        sup.step(&mut b);
        assert_eq!(sup.mode(), Mode::Mission);
    }
    sup.command(Event::RequestLoiter, true, true);
    assert_eq!(sup.mode(), Mode::Loiter);
    let e = sup.state().p; // the estimate the next step latches
    let pause = [e[0], e[1]];
    let off_launch = horiz(&b, launch);
    assert!(
        off_launch > 3.0,
        "the mission leg must have carried the vehicle clear of the launch point \
         for (c) to discriminate: only {off_launch:.2} m"
    );
    // Brake from transit speed, then judge. The transit leaves the position
    // integral charged TOWARD the waypoint (17 m of error against a clamped
    // integral), so the pause overshoots by several metres before the integral
    // unwinds: a core property, like the wind-step transient, and not what (c)
    // pins. (c) pins WHERE it settles.
    let (brake, _) = hold_window(&mut sup, &mut b, Mode::Loiter, pause, CRUISE, BRAKE_S);
    eprintln!("c: braking-window peak {brake:.3} m from the pause point");
    let (ph, _) = hold_window(&mut sup, &mut b, Mode::Loiter, pause, CRUISE, 20.0);
    let to_launch = horiz(&b, launch);
    let to_wp = horiz(&b, [wp[0], wp[1]]);
    eprintln!(
        "c: Mission->Loiter at {:.2},{:.2} ({off_launch:.2} m from launch): peak \
         {ph:.3} m from the pause point; ends {to_launch:.2} m from launch, \
         {to_wp:.2} m from the waypoint",
        pause[0], pause[1]
    );
    assert!(
        ph < HOLD_HORIZ_M,
        "Mission->Loiter must hold where it paused: {ph:.3} m"
    );
    assert!(
        to_launch > 2.0,
        "(c) must not fall back to the takeoff point"
    );
    assert!(to_wp > 2.0, "(c) must not keep flying to the waypoint");

    // d: return, land, disarm; then take off again from where it landed.
    sup.command(Event::RequestRtl, true, true);
    assert_eq!(sup.mode(), Mode::Rtl);
    step_until(
        &mut sup,
        &mut b,
        Mode::Disarmed,
        &[Mode::Rtl, Mode::Land],
        60_000,
    );
    ground_warm(&mut sup, &mut b); // wind off on the ground, as at first launch
    let relaunch = [b.pos[0], b.pos[1]];
    assert!(
        horiz(&b, pause) > 3.0,
        "(d) needs the relaunch point clear of the stale pause point"
    );
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    assert_eq!(sup.mode(), Mode::Takeoff, "re-arm after a landed flight");
    step_until(&mut sup, &mut b, Mode::Loiter, &[Mode::Takeoff], 15_000);
    let _ = hold_window(&mut sup, &mut b, Mode::Loiter, relaunch, CRUISE, SETTLE_S);
    let (ph, _) = hold_window(&mut sup, &mut b, Mode::Loiter, relaunch, CRUISE, 20.0);
    let to_pause = horiz(&b, pause);
    eprintln!(
        "d: re-takeoff at {:.2},{:.2}: peak {ph:.3} m from the relaunch point, \
         {to_pause:.2} m from the stale pause point",
        relaunch[0], relaunch[1]
    );
    assert!(
        ph < HOLD_HORIZ_M,
        "a new Takeoff must latch its own launch point, not a stale one: {ph:.3} m"
    );
}
