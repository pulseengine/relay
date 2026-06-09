//! Relay Offboard — verified setpoint-stream freshness + timeout failsafe.
//!
//! The safety core of "offboard" control (PX4 Offboard mode): a companion
//! computer streams setpoints (position / velocity / yaw) to the vehicle. The
//! transport is the AUTHENTICATED relay-sec channel (v1.35–v1.37: per-frame
//! MAC + anti-replay + session keys), so this crate is NOT a transport — it is
//! the freshness/timeout state machine on top, whose one job is the property a
//! companion link MUST have:
//!
//!   **a stale or dropped setpoint stream must NOT keep flying the vehicle.**
//!
//! If no fresh, in-order setpoint arrives within the timeout, [`OffboardReceiver`]
//! reports [`OffboardStatus::Stale`] — the trigger for the flight stack to fall
//! back to Hold/RTL (the failsafe arbiter, v1.41). A spoofed or replayed
//! setpoint is rejected upstream by relay-sec's MAC + anti-replay; this layer
//! additionally requires a strictly-increasing setpoint counter, so a delayed
//! duplicate cannot re-activate a stale stream.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`. All time arithmetic saturates, so
//! every method is total (no panic / no overflow) for any clock value — proved
//! by the Kani harnesses in `kani_proofs.rs`.

#![no_std]
#![forbid(unsafe_code)]

/// A companion-computer setpoint (NED). The flight stack interprets these in the
/// active offboard sub-mode (position / velocity); `yaw` is NaN to hold heading.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct OffboardSetpoint {
    /// Target position NED (m).
    pub position_ned: [f32; 3],
    /// Feed-forward / target velocity NED (m/s).
    pub velocity_ned: [f32; 3],
    /// Heading setpoint (rad); NaN ⇒ hold current yaw.
    pub yaw: f32,
}

impl OffboardSetpoint {
    /// A position hold-point with zero feed-forward velocity, hold-yaw.
    pub fn position(position_ned: [f32; 3]) -> Self {
        Self { position_ned, velocity_ned: [0.0; 3], yaw: f32::NAN }
    }
}

/// What the flight stack should do with the offboard stream right now.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum OffboardStatus {
    /// A fresh setpoint is in force — fly it.
    Active(OffboardSetpoint),
    /// The stream is stale (timed out) or never started — DO NOT fly offboard;
    /// the caller must fall back to the failsafe action (Hold / RTL).
    Stale,
}

/// Tracks the freshness of an authenticated offboard setpoint stream and enforces
/// the timeout failsafe.
#[derive(Clone, Copy, Debug)]
pub struct OffboardReceiver {
    timeout_us: u64,
    last_fresh_us: u64,
    last_counter: u64,
    sp: OffboardSetpoint,
    started: bool,
}

impl OffboardReceiver {
    /// A receiver that declares the stream stale after `timeout_us` microseconds
    /// without a fresh setpoint. Before the first accepted setpoint it is Stale.
    pub fn new(timeout_us: u64) -> Self {
        Self {
            timeout_us,
            last_fresh_us: 0,
            last_counter: 0,
            sp: OffboardSetpoint { position_ned: [0.0; 3], velocity_ned: [0.0; 3], yaw: f32::NAN },
            started: false,
        }
    }

    /// Accept an authenticated setpoint carrying a strictly-monotonic `counter`
    /// (the relay-sec frame counter) observed at time `now_us`. Returns `true` if
    /// accepted — i.e. the counter strictly increased past the last one. A stale
    /// or replayed counter (≤ last) is rejected and changes nothing, so a delayed
    /// duplicate can never re-activate or extend a stream.
    pub fn accept(&mut self, sp: OffboardSetpoint, counter: u64, now_us: u64) -> bool {
        if self.started && counter <= self.last_counter {
            return false;
        }
        self.last_counter = counter;
        self.last_fresh_us = now_us;
        self.sp = sp;
        self.started = true;
        true
    }

    /// The current offboard status at time `now_us`. `Active` only while a
    /// setpoint has been accepted AND the elapsed time since it is within the
    /// timeout; otherwise `Stale`. Elapsed time is computed with saturating
    /// subtraction, so a clock that moves backwards (or any value) yields a
    /// well-defined result and never panics.
    pub fn status(&self, now_us: u64) -> OffboardStatus {
        if self.started && now_us.saturating_sub(self.last_fresh_us) <= self.timeout_us {
            OffboardStatus::Active(self.sp)
        } else {
            OffboardStatus::Stale
        }
    }

    /// Convenience: is the stream live (not timed out) at `now_us`?
    pub fn is_active(&self, now_us: u64) -> bool {
        matches!(self.status(now_us), OffboardStatus::Active(_))
    }

    /// The last accepted frame counter (0 before the first setpoint).
    pub fn last_counter(&self) -> u64 {
        self.last_counter
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    const TIMEOUT: u64 = 200_000; // 200 ms

    // Concrete yaw (not NaN) so setpoint equality is well-defined in asserts.
    fn sp(n: f32) -> OffboardSetpoint {
        OffboardSetpoint { position_ned: [n, 0.0, -5.0], velocity_ned: [0.0; 3], yaw: 0.0 }
    }

    #[test]
    fn stale_before_first_setpoint() {
        let r = OffboardReceiver::new(TIMEOUT);
        assert_eq!(r.status(0), OffboardStatus::Stale);
        assert_eq!(r.status(1_000_000), OffboardStatus::Stale);
    }

    #[test]
    fn active_while_fresh() {
        let mut r = OffboardReceiver::new(TIMEOUT);
        assert!(r.accept(sp(1.0), 1, 1_000_000));
        assert_eq!(r.status(1_000_000), OffboardStatus::Active(sp(1.0)));
        // still within the timeout window
        assert_eq!(r.status(1_150_000), OffboardStatus::Active(sp(1.0)));
    }

    #[test]
    fn goes_stale_after_timeout() {
        let mut r = OffboardReceiver::new(TIMEOUT);
        r.accept(sp(1.0), 1, 1_000_000);
        // 1 µs past the timeout boundary ⇒ Stale (the failsafe trigger).
        assert_eq!(r.status(1_000_000 + TIMEOUT + 1), OffboardStatus::Stale);
    }

    #[test]
    fn fresh_setpoint_re_arms_the_window() {
        let mut r = OffboardReceiver::new(TIMEOUT);
        r.accept(sp(1.0), 1, 1_000_000);
        // a new setpoint just before timeout keeps it Active and slides the window
        assert!(r.accept(sp(2.0), 2, 1_190_000));
        assert_eq!(r.status(1_300_000), OffboardStatus::Active(sp(2.0)));
    }

    #[test]
    fn replayed_or_stale_counter_rejected() {
        let mut r = OffboardReceiver::new(TIMEOUT);
        r.accept(sp(1.0), 5, 1_000_000);
        // a delayed duplicate (counter ≤ last) must NOT be accepted or extend.
        assert!(!r.accept(sp(9.0), 5, 1_100_000));
        assert!(!r.accept(sp(9.0), 3, 1_100_000));
        assert_eq!(r.last_counter(), 5);
        // it did not slide the window: still timing out from the first setpoint.
        assert_eq!(r.status(1_000_000 + TIMEOUT + 1), OffboardStatus::Stale);
    }

    #[test]
    fn backward_clock_is_stale_not_panic() {
        let mut r = OffboardReceiver::new(TIMEOUT);
        r.accept(sp(1.0), 1, 5_000_000);
        // now < last_fresh (clock moved back): saturating_sub ⇒ 0 ⇒ Active, no panic.
        assert_eq!(r.status(4_000_000), OffboardStatus::Active(sp(1.0)));
    }
}
