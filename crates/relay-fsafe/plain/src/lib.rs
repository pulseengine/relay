//! Relay Fsafe — the verified failsafe arbiter.
//!
//! The single state machine that takes every failsafe TRIGGER the vehicle can
//! see (RC loss, datalink loss, geofence breach, position loss, low / critical
//! battery, a stale offboard stream — relay-offboard, v1.40) and resolves them
//! to ONE prioritized [`FailsafeAction`]. This is the role PX4 fills in its
//! Commander state machine; here it is small enough to prove exhaustively.
//!
//! Verified safety properties (Kani, in `kani_proofs.rs`):
//!  * **total** — every trigger combination yields a defined action, no panic;
//!  * **most-severe-wins** — the action is at least as severe as the action of
//!    every active trigger (a quiet trigger can never mask a loud one);
//!  * **latching** — the latched action is monotone non-decreasing across
//!    updates: a momentary trigger is never silently downgraded (failsafes do
//!    not un-happen until an explicit reset on disarm);
//!  * **no blind RTL** — whenever position is lost the action is at least
//!    `Land`, NEVER `Rtl` — you cannot "return to launch" without a position
//!    fix to navigate home.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// The action the flight stack takes, ordered by SEVERITY (low → high). The
/// arbiter always selects the most-severe active action, so the numeric order
/// here IS the safety priority.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Default)]
pub enum FailsafeAction {
    /// No failsafe — fly the commanded mode.
    #[default]
    None,
    /// Hold position/altitude and wait (recoverable, e.g. a stale offboard
    /// stream that may resume).
    Hold,
    /// Return to launch and land (needs a position fix to navigate).
    Rtl,
    /// Land immediately where we are (used when we cannot navigate home).
    Land,
    /// Terminate (cut to a parachute / kill) — the last resort.
    Terminate,
}

impl FailsafeAction {
    /// Severity rank (higher = more severe). Derived from the enum order.
    pub fn severity(self) -> u8 {
        self as u8
    }
}

/// The failsafe triggers the arbiter sees this cycle (each from a monitor).
#[derive(Clone, Copy, Debug, Default)]
pub struct Triggers {
    /// RC / pilot link lost.
    pub rc_loss: bool,
    /// Ground-control datalink lost.
    pub datalink_loss: bool,
    /// Outside the geofence boundary.
    pub geofence_breach: bool,
    /// No valid position estimate (GNSS denied / estimator divergence).
    pub position_loss: bool,
    /// Battery below the low threshold.
    pub low_battery: bool,
    /// Battery below the critical threshold.
    pub critical_battery: bool,
    /// The offboard setpoint stream has gone stale (relay-offboard).
    pub offboard_stale: bool,
}

impl Triggers {
    /// The action MAPPED to each individual active trigger (the safety policy),
    /// reduced to the most-severe. With nothing active this is `None`.
    ///
    /// Policy rationale: link/geofence/low-battery → RTL (navigate home);
    /// position loss and critical battery → LAND (can't safely navigate / no
    /// time); a stale offboard stream → HOLD (recoverable). Position loss
    /// dominating RTL is what makes "no blind RTL" hold — Land ≻ Rtl.
    pub fn policy_action(self) -> FailsafeAction {
        let mut a = FailsafeAction::None;
        let mut bump = |cond: bool, act: FailsafeAction| {
            if cond && act > a {
                a = act;
            }
        };
        bump(self.offboard_stale, FailsafeAction::Hold);
        bump(self.rc_loss, FailsafeAction::Rtl);
        bump(self.datalink_loss, FailsafeAction::Rtl);
        bump(self.geofence_breach, FailsafeAction::Rtl);
        bump(self.low_battery, FailsafeAction::Rtl);
        bump(self.position_loss, FailsafeAction::Land);
        bump(self.critical_battery, FailsafeAction::Land);
        a
    }
}

/// The arbiter: latches the most-severe action seen since the last reset.
#[derive(Clone, Copy, Debug, Default)]
pub struct FailsafeArbiter {
    latched: FailsafeAction,
}

impl FailsafeArbiter {
    /// A fresh arbiter with no failsafe latched.
    pub fn new() -> Self {
        Self { latched: FailsafeAction::None }
    }

    /// Evaluate this cycle's triggers and return the action to take. The result
    /// is the most-severe of (the latched action, this cycle's policy action),
    /// so the action is monotone non-decreasing until [`reset`](Self::reset).
    pub fn evaluate(&mut self, t: Triggers) -> FailsafeAction {
        let immediate = t.policy_action();
        if immediate > self.latched {
            self.latched = immediate;
        }
        self.latched
    }

    /// The currently latched action.
    pub fn action(&self) -> FailsafeAction {
        self.latched
    }

    /// Clear the latch (call only on disarm / on the ground).
    pub fn reset(&mut self) {
        self.latched = FailsafeAction::None;
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn t() -> Triggers {
        Triggers::default()
    }

    #[test]
    fn nothing_active_is_none() {
        let mut a = FailsafeArbiter::new();
        assert_eq!(a.evaluate(t()), FailsafeAction::None);
    }

    #[test]
    fn rc_loss_returns_to_launch() {
        let mut a = FailsafeArbiter::new();
        assert_eq!(a.evaluate(Triggers { rc_loss: true, ..t() }), FailsafeAction::Rtl);
    }

    #[test]
    fn offboard_stale_holds() {
        let mut a = FailsafeArbiter::new();
        assert_eq!(a.evaluate(Triggers { offboard_stale: true, ..t() }), FailsafeAction::Hold);
    }

    #[test]
    fn most_severe_wins() {
        let mut a = FailsafeArbiter::new();
        // RC loss (Rtl) + critical battery (Land) ⇒ Land.
        let act = a.evaluate(Triggers { rc_loss: true, critical_battery: true, ..t() });
        assert_eq!(act, FailsafeAction::Land);
    }

    #[test]
    fn no_blind_rtl_when_position_lost() {
        let mut a = FailsafeArbiter::new();
        // RC loss would be Rtl, but position is lost ⇒ must be Land, never Rtl.
        let act = a.evaluate(Triggers { rc_loss: true, position_loss: true, ..t() });
        assert_eq!(act, FailsafeAction::Land);
        assert_ne!(act, FailsafeAction::Rtl);
    }

    #[test]
    fn latch_does_not_downgrade() {
        let mut a = FailsafeArbiter::new();
        a.evaluate(Triggers { critical_battery: true, ..t() }); // Land
        // triggers clear, but the latch holds the severe action.
        assert_eq!(a.evaluate(t()), FailsafeAction::Land);
        // a less-severe trigger cannot lower it.
        assert_eq!(a.evaluate(Triggers { offboard_stale: true, ..t() }), FailsafeAction::Land);
    }

    #[test]
    fn reset_clears_on_disarm() {
        let mut a = FailsafeArbiter::new();
        a.evaluate(Triggers { rc_loss: true, ..t() });
        a.reset();
        assert_eq!(a.action(), FailsafeAction::None);
    }
}
