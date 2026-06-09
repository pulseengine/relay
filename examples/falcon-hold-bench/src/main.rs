//! falcon-hold-bench — flight-mode sequence + position/altitude-hold bench.
//!
//! Demonstrates the everyday PX4 GPS modes (Position / Altitude hold) on the
//! EXISTING verified crates — `relay_fsm` (the flight-mode state machine, Kani-
//! proved never-disarm-airborne) and `relay_pos` (the position loop) — by
//! closing a simple point-mass loop with an IDEAL inner attitude loop (the
//! commanded attitude is achieved instantly; the inner cascade is verified
//! separately by relay-geo/relay-adrc). This is the v1.39 oracle: the modes work
//! end-to-end, holding station against a steady wind.
//!
//! Two parts:
//!  1. FSM sequence — Disarmed → Armed → Takeoff → Loiter → Land → Disarmed,
//!     asserting the verified safety invariant holds (no disarm while airborne).
//!  2. Hold — in Loiter, `PosController` holds a 3D point under a constant wind
//!     acceleration; assert horizontal drift and altitude error settle inside a
//!     budget.

use std::process::ExitCode;

use relay_fsm::{Event, FlightFsm, Gates, Mode};
use relay_pos::{PosController, PositionSetpoint, Timestamp};

const G: f32 = 9.81;
/// Thrust→accel scale: collective 0.5 ⇒ hover (cancels gravity).
const MAX_THRUST_ACC: f32 = 2.0 * G;
const DT: f32 = 1.0 / 50.0; // 50 Hz position loop

/// Rotate a body vector into NED by a unit quaternion (w,x,y,z).
fn q_rotate(q: [f32; 4], v: [f32; 3]) -> [f32; 3] {
    let u = [q[1], q[2], q[3]];
    let cross = |a: [f32; 3], b: [f32; 3]| {
        [
            a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0],
        ]
    };
    let uv = cross(u, v);
    let uuv = cross(u, uv);
    [
        v[0] + 2.0 * (q[0] * uv[0] + uuv[0]),
        v[1] + 2.0 * (q[0] * uv[1] + uuv[1]),
        v[2] + 2.0 * (q[0] * uv[2] + uuv[2]),
    ]
}

/// Run the FSM through a normal flight and confirm the safety invariant:
/// a RequestDisarm COMMAND must be refused while airborne (the Kani-proved
/// never-disarm-airborne property), while a legitimate land (RequestLand →
/// Touchdown) reaches Disarmed.
fn fsm_sequence() -> (Mode, bool) {
    let mut fsm = FlightFsm::new();
    let ground = Gates { level: true, throttle_low: true, have_position: true };
    let flying = Gates { level: true, throttle_low: false, have_position: true };

    fsm.on(Event::Arm, ground);
    fsm.on(Event::RequestTakeoff, flying);
    fsm.on(Event::ReachedAltitude, flying);
    let in_loiter = fsm.mode();

    // A disarm COMMAND while airborne must be refused — vehicle stays airborne.
    fsm.on(Event::RequestDisarm, flying);
    let disarm_refused_airborne = fsm.is_airborne();

    // A legitimate landing does reach Disarmed.
    fsm.on(Event::RequestLand, flying);
    fsm.on(Event::Touchdown, ground);
    let landed_disarmed = !fsm.is_airborne();

    (in_loiter, disarm_refused_airborne && landed_disarmed)
}

/// Close the position-hold loop against a steady wind; return (horiz drift,
/// altitude error) at the end of the run.
fn hold_under_wind() -> (f32, f32) {
    let target = [0.0f32, 0.0, -10.0]; // hold 10 m up (down is +z)
    let wind = [0.6f32, -0.4, 0.0]; // steady horizontal wind accel (m/s²)

    let mut pos = [0.0f32, 0.0, -10.0];
    let mut vel = [0.0f32; 3];
    let mut q = [1.0f32, 0.0, 0.0, 0.0]; // achieved attitude (ideal inner loop)
    let mut ctrl = PosController::new();
    let sp = PositionSetpoint::hover_at(target);

    let steps = 50 * 30; // 30 s
    for k in 0..steps {
        let t = Timestamp { seconds: 0, fraction: ((k as f32 * DT) * 1e9) as u32 };
        let att = ctrl.tick(t, pos, vel, q, sp);
        q = att.quaternion; // ideal inner loop: achieved = commanded

        // Plant: rotate body thrust into NED, add gravity + wind.
        let thr = att.thrust * MAX_THRUST_ACC;
        let a_thrust = q_rotate(q, [0.0, 0.0, -thr]);
        let a_ned = [
            a_thrust[0] + wind[0],
            a_thrust[1] + wind[1],
            a_thrust[2] + G + wind[2],
        ];
        for i in 0..3 {
            pos[i] += vel[i] * DT + 0.5 * a_ned[i] * DT * DT;
            vel[i] += a_ned[i] * DT;
        }
    }

    let horiz = (pos[0] * pos[0] + pos[1] * pos[1]).sqrt();
    let alt_err = (pos[2] - target[2]).abs();
    (horiz, alt_err)
}

fn main() -> ExitCode {
    let (loiter_mode, fsm_ok) = fsm_sequence();
    let (horiz, alt_err) = hold_under_wind();

    println!("falcon-hold-bench — flight modes + position/altitude hold");
    println!("  FSM sequence reaches:    {loiter_mode:?} (expect Loiter)");
    println!("  FSM safety invariant:    {} (no disarm airborne)", fsm_ok);
    println!("  position-hold drift:     {horiz:.3} m   (budget ≤ 1.0)");
    println!("  altitude-hold error:     {alt_err:.3} m   (budget ≤ 0.5)");

    let pass = loiter_mode == Mode::Loiter && fsm_ok && horiz <= 1.0 && alt_err <= 0.5;
    if pass {
        println!("RESULT: PASS — modes sequence correctly and hold station against wind.");
        ExitCode::SUCCESS
    } else {
        println!("RESULT: FAIL");
        ExitCode::FAILURE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fsm_reaches_loiter_and_keeps_the_airborne_invariant() {
        let (mode, ok) = fsm_sequence();
        assert_eq!(mode, Mode::Loiter);
        assert!(ok, "FSM violated the never-disarm-airborne invariant");
    }

    #[test]
    fn position_and_altitude_hold_station_against_wind() {
        let (horiz, alt_err) = hold_under_wind();
        assert!(horiz <= 1.0, "horizontal drift {horiz:.3} m exceeds 1.0 m budget");
        assert!(alt_err <= 0.5, "altitude error {alt_err:.3} m exceeds 0.5 m budget");
    }
}
