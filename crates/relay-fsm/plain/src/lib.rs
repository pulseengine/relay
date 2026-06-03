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
}

impl Mode {
    /// Airborne in the dangerous sense (motors must NOT be cut here).
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
            // arm only from the ground, level + throttle idle
            (Disarmed, Arm) if g.level && g.throttle_low => Armed,
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
            // everything else: no-op (total)
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
        Gates { level, throttle_low, have_position }
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
    fn cannot_disarm_airborne() {
        for start in [Mode::Takeoff, Mode::Loiter, Mode::Mission, Mode::Rtl] {
            let mut f = FlightFsm { mode: start };
            assert_eq!(f.on(Event::RequestDisarm, g(true, true, true)), start, "disarm must no-op in {start:?}");
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
        match kani::any::<u8>() % 7 {
            0 => Mode::Disarmed,
            1 => Mode::Armed,
            2 => Mode::Takeoff,
            3 => Mode::Loiter,
            4 => Mode::Mission,
            5 => Mode::Land,
            _ => Mode::Rtl,
        }
    }

    fn any_event() -> Event {
        match kani::any::<u8>() % 11 {
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
            _ => Event::Failsafe,
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
        };
        let next = f.on(any_event(), g);
        if next == Mode::Disarmed {
            assert!(matches!(start, Mode::Disarmed | Mode::Armed | Mode::Land));
        }
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
        };
        let next = f.on(Event::Failsafe, g);
        assert!(matches!(next, Mode::Rtl | Mode::Land));
    }
}
