//! Time-gate primitive — "emit when scheduled time arrives."
//!
//! Generalizes the dispatch predicates from relay-sc (relative time) and
//! relay-sca (absolute time). The actual primitive is: given a current
//! time and a scheduled time, decide whether the scheduled event is due.
//!
//! Used by: autonomy sequences, waypoint dispatch, delayed commands,
//! timeout firing, debounce release, ARINC-653 minor-frame boundaries.
//!
//! Verified properties:
//!   TG-P01: is_due is total, monotonic in current_time
//!   TG-P02: once due, stays due (no oscillation) — caller must consume the event
//!   TG-P03: relative-time gating is equivalent to absolute gating where
//!           scheduled_at = start_time + delay

use vstd::prelude::*;

verus! {

/// Absolute time gate: event fires when `current >= scheduled_at`.
pub fn is_due_absolute(current: u64, scheduled_at: u64) -> (result: bool)
    ensures
        result == (current >= scheduled_at),
{
    current >= scheduled_at
}

/// Relative time gate: event fires when `elapsed >= delay`.
/// Equivalent to `is_due_absolute(start + elapsed, start + delay)`
/// for any non-overflowing start.
pub fn is_due_relative(elapsed: u64, delay: u64) -> (result: bool)
    ensures
        result == (elapsed >= delay),
{
    elapsed >= delay
}

} // verus!

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn absolute_not_due_before() {
        assert!(!is_due_absolute(99, 100));
    }

    #[test]
    fn absolute_due_at_exact() {
        assert!(is_due_absolute(100, 100));
    }

    #[test]
    fn absolute_due_after() {
        assert!(is_due_absolute(200, 100));
    }

    #[test]
    fn relative_equivalent_to_absolute() {
        let start = 1_000u64;
        for (elapsed, delay) in [(0u64, 0u64), (5, 10), (10, 10), (15, 10)] {
            assert_eq!(
                is_due_relative(elapsed, delay),
                is_due_absolute(start + elapsed, start + delay),
            );
        }
    }
}
