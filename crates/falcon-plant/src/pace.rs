//! Gyro-synchronized loop pacing (v0.32).
//!
//! Real flight controllers (Betaflight, PX4) run the control loop off the
//! IMU **data-ready (DRDY) interrupt**, not a free-running wall-clock timer.
//! We mirror that here: the bench's control loop advances when the gz IMU
//! sample stream delivers a *new* sample, instead of sleeping a fixed
//! wall-clock period.
//!
//! Why it matters: when the simulator's real-time factor drops (GUI + video
//! encoder competing for CPU), a wall-clock-paced loop keeps firing at 1 kHz
//! while the physics falls behind — it then over-drives *stale* state and
//! desyncs the marginally-stable inner loop (the v0.31 recording-divergence
//! root cause). Pacing to the IMU stream means the loop slows *with* the sim:
//! no over-driving, no desync. At RTF ≈ 1 (one IMU sample per ms) it is
//! identical to wall-clock pacing.
//!
//! This module is the pure *decision* — division-free, `Duration`-free
//! (microsecond integers) so it is unit- and Kani-testable in isolation from
//! the gz I/O it drives.

/// Outcome of one gyro-sync pacing decision.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pace {
    /// A new IMU sample arrived — proceed now; the carried value is the new
    /// watermark (the IMU receive-count to remember for next tick).
    Fresh(u64),
    /// No new sample yet and the deadline has not passed — keep waiting.
    Wait,
    /// Deadline exceeded without a fresh sample — proceed anyway (bounded
    /// fallback so a sensor stall can never hang the loop). The carried value
    /// is the (unchanged) watermark.
    Deadline(u64),
}

/// Decide whether to proceed with this control tick.
///
/// * `last_imu`   — IMU receive-count remembered at the previous tick.
/// * `now_imu`    — IMU receive-count right now.
/// * `waited_us`  — microseconds spent in this tick so far (since tick start).
/// * `deadline_us`— max microseconds to wait for a fresh sample before the
///   bounded fallback fires.
///
/// `Fresh` is preferred whenever a new sample exists, *even if* the deadline
/// has also passed — a real sample always beats the fallback.
#[inline]
pub fn pace_decision(last_imu: u64, now_imu: u64, waited_us: u64, deadline_us: u64) -> Pace {
    if now_imu > last_imu {
        Pace::Fresh(now_imu)
    } else if waited_us >= deadline_us {
        Pace::Deadline(now_imu)
    } else {
        Pace::Wait
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rtf_one_always_fresh_and_advances() {
        // At RTF≈1 the IMU advances by one each tick: always Fresh, watermark
        // tracks the stream — i.e. identical cadence to wall-clock pacing.
        let mut last = 0u64;
        for k in 1..=1000u64 {
            match pace_decision(last, k, 0, 1500) {
                Pace::Fresh(w) => {
                    assert_eq!(w, k);
                    last = w;
                }
                other => panic!("expected Fresh at tick {k}, got {other:?}"),
            }
        }
        assert_eq!(last, 1000);
    }

    #[test]
    fn stale_before_deadline_waits() {
        // No new sample yet, still inside the deadline window → Wait.
        assert_eq!(pace_decision(42, 42, 0, 1500), Pace::Wait);
        assert_eq!(pace_decision(42, 42, 1499, 1500), Pace::Wait);
    }

    #[test]
    fn stale_past_deadline_proceeds_without_hang() {
        // Deadline reached with no fresh sample → bounded fallback, watermark
        // unchanged (we did not consume a real sample).
        assert_eq!(pace_decision(42, 42, 1500, 1500), Pace::Deadline(42));
        assert_eq!(pace_decision(42, 42, 9000, 1500), Pace::Deadline(42));
    }

    #[test]
    fn fresh_beats_deadline_when_both() {
        // A real sample present AND the deadline passed → Fresh wins.
        assert_eq!(pace_decision(42, 43, 9000, 1500), Pace::Fresh(43));
    }

    #[test]
    fn low_rtf_skips_accumulated_samples_to_latest() {
        // If several IMU samples arrived since last tick (loop slower than the
        // stream), we jump the watermark to the latest — one tick consumes the
        // freshest state, never replays stale ones.
        assert_eq!(pace_decision(100, 105, 0, 1500), Pace::Fresh(105));
    }
}
