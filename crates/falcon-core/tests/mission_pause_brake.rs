//! #609 finding 4, Decision B as re-decided (maintainer, 2026-10-28): BRAKE,
//! THEN LATCH. A Loiter entered while the vehicle is moving (Mission -> Loiter)
//! must hold where the vehicle actually STOPS, not the point it was passing
//! when the command arrived; the position integral is left untouched.
//!
//! The scenario is #616's test (3c) (`loiter_hold_latch.rs`): launch 8 m north
//! of home, crosswind, a settled Loiter, a 5 s Mission leg 20 m east,
//! RequestLoiter. Run at 2 m/s (3c's wind) and 5 m/s (test (1)'s).
//!
//! WHAT IS MEASURED, without any supervisor accessor, so the same file judges
//! the pre-change supervisor too. The vehicle's HOLD POINT is taken from where
//! it ends up (the mean true position over the last 10 s of a 170 s Loiter),
//! and everything is judged against it:
//!   * peak overshoot: the largest distance from the hold point after the
//!     vehicle FIRST came to rest (true speed < 0.3 m/s). A point latched while
//!     moving is overshot from there on; a point latched where the vehicle
//!     stopped is not. (Before first rest the vehicle is still braking.)
//!   * inside-for-good: the last instant the vehicle was more than HOLD-P01's
//!     1.0 m from it;
//!   * the HOLD-P01 window: peak horizontal / vertical error over 60 s,
//!     opening [`SETTLE_S`] after the command (the settle allowance #616's
//!     test (1) gives a Loiter after takeoff).
//!
//! Plus where it stopped relative to the command point (how far it carried on).
//!
//! The hold point is checked to be a real hold: the last 10 s stay within
//! 0.1 m of their own mean, or the "hold point" would be a moving average.

use falcon_core::relay_fsm::{Event, Mode};
use falcon_core::{FlightSupervisor, SimBackend};

const DT: f32 = 0.002;
const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
const CRUISE: f32 = 2.0;
/// SWREQ-FALCON-HOLD-P01: "within 0.5 m vertically and 1.0 m horizontally of
/// the setpoint for at least 60 s".
const HOLD_HORIZ_M: f32 = 1.0;
const HOLD_VERT_M: f32 = 0.5;
const HOLD_S: f32 = 60.0;
/// #616's settle allowance before the HOLD-P01 window (loiter_hold_latch.rs).
const SETTLE_S: f32 = 50.0;
const RUN_S: f32 = 170.0;
/// "At rest" for the overshoot measurement: true horizontal speed below this.
const REST_MS: f32 = 0.3;

fn cal() -> falcon_core::relay_calib::CalParams {
    falcon_core::relay_calib::CalParams {
        gyro_bias: [0.001, 0.0, 0.0],
        ..falcon_core::relay_calib::CalParams::identity()
    }
}

fn d2(a: [f32; 2], b: [f32; 2]) -> f32 {
    ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt()
}

/// #616's rig: estimator converged on the ground with the wind off, then on.
fn rig(launch: [f32; 2], wind: [f32; 3]) -> (SimBackend, FlightSupervisor) {
    let mut b = SimBackend::new(LEVEL, DT);
    b.ground_contact = true;
    b.pos = [launch[0], launch[1], 0.0];
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, CRUISE, 14.0);
    sup.set_calibration(cal());
    for _ in 0..(20.0 / DT) as u32 {
        sup.step(&mut b);
    }
    b.wind = wind;
    (b, sup)
}

pub struct Pause {
    pub speed: f32,
    pub carried_on: f32,
    pub rest_s: f32,
    pub peak: f32,
    pub settle: f32,
    pub win_h: f32,
    pub win_v: f32,
}

/// Take off, settle 70 s in Loiter, fly the (3c) leg, pause it, and measure.
fn pause_after_leg(launch: [f32; 2], wind: [f32; 3]) -> Pause {
    let (mut b, mut sup) = rig(launch, wind);
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..15_000 {
        if sup.mode() == Mode::Loiter {
            break;
        }
        sup.step(&mut b);
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    for _ in 0..(70.0 / DT) as u32 {
        sup.step(&mut b);
        assert_eq!(sup.mode(), Mode::Loiter);
    }
    let wp = [launch[0], launch[1] + 20.0, -CRUISE];
    sup.set_mission(wp);
    sup.command(Event::RequestMission, true, true);
    for _ in 0..(5.0 / DT) as u32 {
        sup.step(&mut b);
        assert_eq!(sup.mode(), Mode::Mission);
    }
    let speed = (b.vel[0] * b.vel[0] + b.vel[1] * b.vel[1]).sqrt();
    let cmd_at = [b.pos[0], b.pos[1]];
    sup.command(Event::RequestLoiter, true, true);
    assert_eq!(sup.mode(), Mode::Loiter);
    let n = (RUN_S / DT) as usize;
    let mut xy = Vec::with_capacity(n);
    let mut z = Vec::with_capacity(n);
    let mut spd = Vec::with_capacity(n);
    for i in 0..n {
        sup.step(&mut b);
        assert_eq!(sup.mode(), Mode::Loiter, "left Loiter at tick {i}");
        xy.push([b.pos[0], b.pos[1]]);
        z.push(-b.pos[2]);
        spd.push((b.vel[0] * b.vel[0] + b.vel[1] * b.vel[1]).sqrt());
    }
    let tail = (10.0 / DT) as usize;
    let last = &xy[n - tail..];
    let hold = [
        last.iter().map(|p| p[0]).sum::<f32>() / tail as f32,
        last.iter().map(|p| p[1]).sum::<f32>() / tail as f32,
    ];
    let wander = last.iter().map(|p| d2(*p, hold)).fold(0.0f32, f32::max);
    assert!(
        wander < 0.1,
        "not a hold: the last 10 s wander {wander:.3} m"
    );
    let (w0, w1) = (
        (SETTLE_S / DT) as usize,
        ((SETTLE_S + HOLD_S) / DT) as usize,
    );
    let mut r = Pause {
        speed,
        carried_on: d2(cmd_at, hold),
        rest_s: 0.0,
        peak: 0.0,
        settle: 0.0,
        win_h: 0.0,
        win_v: 0.0,
    };
    let rest = spd.iter().position(|v| *v < REST_MS).unwrap_or(n - 1);
    r.rest_s = rest as f32 * DT;
    for (i, p) in xy.iter().enumerate() {
        let h = d2(*p, hold);
        if i >= rest {
            r.peak = r.peak.max(h);
        }
        if h >= HOLD_HORIZ_M {
            r.settle = (i + 1) as f32 * DT;
        }
        if (w0..w1).contains(&i) {
            r.win_h = r.win_h.max(h);
            r.win_v = r.win_v.max((z[i] - CRUISE).abs());
        }
    }
    r
}

fn report(tag: &str, r: &Pause) {
    eprintln!(
        "{tag}: paused at {:.2} m/s; first at rest after {:.1} s; held {:.2} m on from \
         the command point; peak overshoot {:.3} m from the hold point after first rest; inside {HOLD_HORIZ_M} m for good from \
         {:.1} s; HOLD-P01 window ({SETTLE_S}-{} s) peak horiz {:.3} m, vert {:.3} m",
        r.speed,
        r.rest_s,
        r.carried_on,
        r.peak,
        r.settle,
        SETTLE_S + HOLD_S,
        r.win_h,
        r.win_v
    );
}

/// (B1) Mission -> Loiter under the (3c) 2 m/s crosswind meets HOLD-P01 in the
/// window #616 allows a post-takeoff Loiter. Pre-change: latched at the
/// command point while doing ~3.4 m/s, 11.6 m overshoot, 1.59 m in the window.
#[test]
fn b1_mission_pause_brakes_then_holds_2ms() {
    let r = pause_after_leg([8.0, 0.0], [0.0, 2.0, 0.0]);
    report("B1 2 m/s", &r);
    assert!(
        r.win_h < HOLD_HORIZ_M,
        "HOLD-P01 window horiz {:.3} m",
        r.win_h
    );
    assert!(
        r.win_v < HOLD_VERT_M,
        "HOLD-P01 window vert {:.3} m",
        r.win_v
    );
}

/// (B2) The same under 5 m/s steady wind, with WIND-P01's integral left as it
/// was (Decision B as re-decided: nothing is reset).
#[test]
fn b2_mission_pause_brakes_then_holds_5ms() {
    let r = pause_after_leg([6.0, -4.0], [3.0, 4.0, 0.0]);
    report("B2 5 m/s", &r);
    assert!(
        r.win_h < HOLD_HORIZ_M,
        "HOLD-P01 window horiz {:.3} m",
        r.win_h
    );
    assert!(
        r.win_v < HOLD_VERT_M,
        "HOLD-P01 window vert {:.3} m",
        r.win_v
    );
}
