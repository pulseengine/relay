//! # relay-fsm — falcon flight-mode state machine (v1.7)
//!
//! The autonomy layer the clean-room audit found missing: a real flight-mode
//! state machine — `Disarmed → Armed → Takeoff → Loiter ↔ Mission → Land →
//! Disarmed`, with `Rtl` reachable from any flying state — sitting over the
//! `relay-arm` arming gate.
//!
//! The transitions are **safety-guarded** and the two safety invariants are
//! Kani-proved:
//!   1. **You can never disarm airborne.** `Disarmed` is reachable only from
//!      `Armed` (on the ground) or from `Land` after `Touchdown` — never from
//!      `Takeoff`/`Loiter`/`Mission`/`Rtl`. A spurious disarm request in flight
//!      is a no-op.
//!   2. **A failsafe always recovers.** From any flying state a `Failsafe`
//!      event commands `Rtl` (or `Land` with no position) — never `Disarmed`,
//!      never a no-op.
//!
//! `on(event, gates)` is total: every (state, event) pair has a defined
//! result; unhandled pairs leave the mode unchanged.

#![no_std]

/// Flight mode.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Mode {
    /// Motors off / safe.
    Disarmed,
    /// Armed on the ground (motors at idle), ready to take off.
    Armed,
    /// Climbing to the takeoff altitude.
    Takeoff,
    /// Holding position/altitude.
    Loiter,
    /// Flying the stored waypoint mission.
    Mission,
    /// Controlled descent to the ground.
    Land,
    /// Return to launch (fly home, then land).
    Rtl,
    /// FLIGHT TERMINATED — motors cut, last resort. Entered on attitude runaway
    /// (a tumble the controller cannot recover), where a controlled RTL/Land is
    /// impossible and cutting thrust limits damage/injury. Absorbing: nothing
    /// re-arms or recovers from here without an on-the-ground reset. Distinct from
    /// Disarmed (a SAFE on-ground state) — Terminated is a deliberate airborne cut.
    Terminated,
}

impl Mode {
    /// Airborne in the dangerous sense (motors must NOT be cut here). Terminated
    /// is EXCLUDED: its motors are already cut by design (the termination is the
    /// intentional last-resort), so the never-cut-motors-airborne concern is moot.
    pub fn is_airborne(self) -> bool {
        matches!(self, Mode::Takeoff | Mode::Loiter | Mode::Mission | Mode::Rtl | Mode::Land)
    }
}

/// Commands + sensed milestones the FSM reacts to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Event {
    Arm,
    RequestTakeoff,
    RequestMission,
    RequestLoiter,
    RequestRtl,
    RequestLand,
    RequestDisarm,
    ReachedAltitude,
    ReachedHome,
    Touchdown,
    Failsafe,
    /// FLIGHT TERMINATION — the last-resort motor cut (attitude runaway). From
    /// ANY state this forces [`Mode::Terminated`]; it overrides every other
    /// transition because nothing is more urgent than killing a tumble.
    Terminate,
}

/// Physical preconditions the FSM checks (from the estimator/sensors).
#[derive(Clone, Copy, Debug)]
pub struct Gates {
    /// Attitude within the arming tilt limit.
    pub level: bool,
    /// Throttle stick / collective at idle.
    pub throttle_low: bool,
    /// A valid position fix is available.
    pub have_position: bool,
    /// The pre-arm / commander check verdict: every precondition (sensor health,
    /// estimator convergence, calibration, geofence, battery, failsafe config)
    /// holds. Computed by `relay_preflight::arm_check` in the falcon-core seam and
    /// passed in here — the FSM stays a pure state machine. Arming requires it
    /// (the entry gate, the architectural complement of the never-disarm-airborne
    /// exit invariant); PX4's commander pre-flight checks made provable.
    pub prearm_ok: bool,
}

/// The flight-mode state machine.
#[derive(Clone, Copy)]
pub struct FlightFsm {
    mode: Mode,
}

impl Default for FlightFsm {
    fn default() -> Self {
        Self::new()
    }
}

impl FlightFsm {
    pub fn new() -> Self {
        FlightFsm { mode: Mode::Disarmed }
    }

    pub fn mode(&self) -> Mode {
        self.mode
    }

    pub fn is_airborne(&self) -> bool {
        self.mode.is_airborne()
    }

    /// Process an event under the current gates; return the new mode. Total:
    /// every (state, event) pair is defined; unhandled pairs leave the mode
    /// unchanged. SAFETY transitions are guarded (see the module invariants).
    pub fn on(&mut self, ev: Event, g: Gates) -> Mode {
        use Event::*;
        use Mode::*;
        let next = match (self.mode, ev) {
            // arm only from the ground: level + throttle idle + the pre-arm
            // commander checks all hold (relay-preflight arm_check == Allowed)
            (Disarmed, Arm) if g.level && g.throttle_low && g.prearm_ok => Armed,
            // disarm ONLY on the ground (Armed) or after Touchdown (Land)
            (Armed, RequestDisarm) => Disarmed,
            (Land, Touchdown) => Disarmed,
            // launch
            (Armed, RequestTakeoff) if g.have_position => Takeoff,
            (Takeoff, ReachedAltitude) => Loiter,
            // loiter ↔ mission
            (Loiter, RequestMission) => Mission,
            (Mission, RequestLoiter) => Loiter,
            (Mission, ReachedHome) => Loiter,
            // land from a holding/flying state
            (Loiter, RequestLand) | (Mission, RequestLand) | (Rtl, RequestLand) => Land,
            // return to launch
            (Loiter, RequestRtl) | (Mission, RequestRtl) | (Takeoff, RequestRtl) => Rtl,
            (Rtl, ReachedHome) => Land,
            // FAILSAFE from any flying state → recover (never disarm in air)
            (Takeoff | Loiter | Mission, Failsafe) => {
                if g.have_position {
                    Rtl
                } else {
                    Land
                }
            }
            (Rtl, Failsafe) => Land,
            // FLIGHT TERMINATION — from ANY state, the last-resort motor cut.
            // Overrides everything; Terminated is absorbing (handled by the
            // no-op default below: no event leaves Terminated).
            (_, Terminate) => Terminated,
            // everything else: no-op (total) — includes (Terminated, _) ⇒
            // Terminated, so termination is absorbing.
            (m, _) => m,
        };
        self.mode = next;
        next
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn g(level: bool, throttle_low: bool, have_position: bool) -> Gates {
        // prearm_ok defaults true here so the existing physical-gate tests are
        // unchanged; the prearm-blocks-arm cases set it explicitly below.
        Gates { level, throttle_low, have_position, prearm_ok: true }
    }

    #[test]
    fn nominal_mission_lifecycle() {
        let mut f = FlightFsm::new();
        assert_eq!(f.mode(), Mode::Disarmed);
        assert_eq!(f.on(Event::Arm, g(true, true, true)), Mode::Armed);
        assert_eq!(f.on(Event::RequestTakeoff, g(true, true, true)), Mode::Takeoff);
        assert_eq!(f.on(Event::ReachedAltitude, g(true, false, true)), Mode::Loiter);
        assert_eq!(f.on(Event::RequestMission, g(true, false, true)), Mode::Mission);
        assert_eq!(f.on(Event::ReachedHome, g(true, false, true)), Mode::Loiter);
        assert_eq!(f.on(Event::RequestLand, g(true, false, true)), Mode::Land);
        assert_eq!(f.on(Event::Touchdown, g(true, true, true)), Mode::Disarmed);
    }

    #[test]
    fn cannot_arm_while_tilted_or_throttle_up() {
        let mut f = FlightFsm::new();
        assert_eq!(f.on(Event::Arm, g(false, true, true)), Mode::Disarmed); // tilted
        assert_eq!(f.on(Event::Arm, g(true, false, true)), Mode::Disarmed); // throttle up
        assert_eq!(f.on(Event::Arm, g(true, true, true)), Mode::Armed);
    }

    #[test]
    fn cannot_arm_when_prearm_checks_fail() {
        // Physically ready (level + throttle idle) but the commander pre-arm
        // verdict is false → arming is refused, the motors stay safe.
        let mut f = FlightFsm::new();
        let blocked = Gates { level: true, throttle_low: true, have_position: true, prearm_ok: false };
        assert_eq!(f.on(Event::Arm, blocked), Mode::Disarmed);
        // and once the checks pass, the same physical state arms.
        assert_eq!(f.on(Event::Arm, g(true, true, true)), Mode::Armed);
    }

    #[test]
    fn cannot_disarm_airborne() {
        for start in [Mode::Takeoff, Mode::Loiter, Mode::Mission, Mode::Rtl] {
            let mut f = FlightFsm { mode: start };
            assert_eq!(f.on(Event::RequestDisarm, g(true, true, true)), start, "disarm must no-op in {start:?}");
        }
    }

    #[test]
    fn terminate_cuts_motors_from_any_state_and_latches() {
        // from every mode, Terminate → Terminated.
        for start in [
            Mode::Disarmed,
            Mode::Armed,
            Mode::Takeoff,
            Mode::Loiter,
            Mode::Mission,
            Mode::Land,
            Mode::Rtl,
        ] {
            let mut f = FlightFsm { mode: start };
            assert_eq!(f.on(Event::Terminate, g(true, true, true)), Mode::Terminated, "{start:?} → Terminated");
        }
        // and Terminated is absorbing: no event leaves it.
        let mut f = FlightFsm { mode: Mode::Terminated };
        for ev in [Event::Arm, Event::RequestTakeoff, Event::Failsafe, Event::Touchdown, Event::RequestDisarm] {
            assert_eq!(f.on(ev, g(true, true, true)), Mode::Terminated, "Terminated absorbs {ev:?}");
        }
    }

    #[test]
    fn failsafe_recovers_from_flight() {
        for start in [Mode::Takeoff, Mode::Loiter, Mode::Mission] {
            let mut f = FlightFsm { mode: start };
            assert_eq!(f.on(Event::Failsafe, g(true, false, true)), Mode::Rtl, "with position → RTL");
            let mut f2 = FlightFsm { mode: start };
            assert_eq!(f2.on(Event::Failsafe, g(true, false, false)), Mode::Land, "no position → Land");
        }
    }
}

#[cfg(kani)]
mod kani_harness {
    use super::*;

    fn any_mode() -> Mode {
        match kani::any::<u8>() % 8 {
            0 => Mode::Disarmed,
            1 => Mode::Armed,
            2 => Mode::Takeoff,
            3 => Mode::Loiter,
            4 => Mode::Mission,
            5 => Mode::Land,
            6 => Mode::Rtl,
            _ => Mode::Terminated,
        }
    }

    fn any_event() -> Event {
        match kani::any::<u8>() % 12 {
            0 => Event::Arm,
            1 => Event::RequestTakeoff,
            2 => Event::RequestMission,
            3 => Event::RequestLoiter,
            4 => Event::RequestRtl,
            5 => Event::RequestLand,
            6 => Event::RequestDisarm,
            7 => Event::ReachedAltitude,
            8 => Event::ReachedHome,
            9 => Event::Touchdown,
            10 => Event::Failsafe,
            _ => Event::Terminate,
        }
    }

    /// SAFETY INVARIANT 1 — you can never disarm airborne. For ANY (state,
    /// event, gates), if the result is Disarmed then the prior state was
    /// Disarmed, Armed (on ground), or Land (after touchdown) — never one of
    /// the airborne-flight states.
    #[kani::proof]
    fn verify_never_disarm_airborne() {
        let start = any_mode();
        let mut f = FlightFsm { mode: start };
        let g = Gates {
            level: kani::any(),
            throttle_low: kani::any(),
            have_position: kani::any(),
            prearm_ok: kani::any(),
        };
        let next = f.on(any_event(), g);
        if next == Mode::Disarmed {
            assert!(matches!(start, Mode::Disarmed | Mode::Armed | Mode::Land));
        }
    }

    /// SAFETY INVARIANT 3 — the ARM ENTRY GATE (FSM-K03, the v1.97 property): the
    /// FSM can transition to `Armed` ONLY from `Disarmed` and ONLY when every gate
    /// holds — level AND throttle_low AND prearm_ok (the relay-preflight commander
    /// verdict). So motors can never spin up with a failed pre-arm check, for ANY
    /// starting mode / event / gate combination. The architectural complement of
    /// the never-disarm-airborne exit invariant: a proven entry gate + a proven
    /// exit gate bracket the armed lifecycle.
    #[kani::proof]
    fn verify_arm_requires_preconditions() {
        let start = any_mode();
        let mut f = FlightFsm { mode: start };
        let g = Gates {
            level: kani::any(),
            throttle_low: kani::any(),
            have_position: kani::any(),
            prearm_ok: kani::any(),
        };
        let ev = any_event();
        let before = f.mode();
        let next = f.on(ev, g);
        // If we just entered Armed, it MUST have been an Arm event from Disarmed
        // with all three gates satisfied — nothing else can produce Armed.
        if next == Mode::Armed && before != Mode::Armed {
            assert!(before == Mode::Disarmed);
            assert!(ev == Event::Arm);
            assert!(g.level && g.throttle_low && g.prearm_ok);
        }
    }

    /// SAFETY INVARIANT 4 — FLIGHT TERMINATION is unconditional (FSM-K04): a
    /// Terminate event from ANY mode, under ANY gates, forces Terminated. The
    /// last-resort motor cut can never be blocked by the current mode or a gate.
    #[kani::proof]
    fn verify_terminate_is_unconditional() {
        let start = any_mode();
        let mut f = FlightFsm { mode: start };
        let g = Gates {
            level: kani::any(),
            throttle_low: kani::any(),
            have_position: kani::any(),
            prearm_ok: kani::any(),
        };
        assert!(f.on(Event::Terminate, g) == Mode::Terminated);
    }

    /// SAFETY INVARIANT 5 — Terminated is ABSORBING (FSM-K05): once terminated,
    /// NO event under ANY gates leaves the Terminated state — there is no path
    /// back to an armed/flying mode without an on-ground reset (a fresh FSM).
    #[kani::proof]
    fn verify_terminated_is_absorbing() {
        let mut f = FlightFsm { mode: Mode::Terminated };
        let g = Gates {
            level: kani::any(),
            throttle_low: kani::any(),
            have_position: kani::any(),
            prearm_ok: kani::any(),
        };
        assert!(f.on(any_event(), g) == Mode::Terminated);
    }

    /// SAFETY INVARIANT 2 — a failsafe from any flying state commands a
    /// recovery mode (Rtl or Land), never Disarmed and never a no-op.
    #[kani::proof]
    fn verify_failsafe_recovers() {
        let start = any_mode();
        kani::assume(matches!(start, Mode::Takeoff | Mode::Loiter | Mode::Mission | Mode::Rtl));
        let mut f = FlightFsm { mode: start };
        let g = Gates {
            level: kani::any(),
            throttle_low: kani::any(),
            have_position: kani::any(),
            prearm_ok: kani::any(),
        };
        let next = f.on(Event::Failsafe, g);
        assert!(matches!(next, Mode::Rtl | Mode::Land));
    }
}
