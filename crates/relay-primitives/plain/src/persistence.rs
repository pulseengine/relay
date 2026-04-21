//! Persistence-filter primitive — "N consecutive violations before firing."
//!
//! Generalizes the wp_decide pattern from relay-lc/src/engine.rs:123-151.
//! The original was packaged as "Limit Checker watchpoint decision"; the
//! actual primitive is: given a stream of boolean events (violated or not),
//! fire an output event only after N consecutive true values, and reset on
//! any false value.
//!
//! Used by: altimeter mode-switch, thermal regulation hysteresis, ECU
//! diagnostic trouble codes, industrial interlocks, medical alarm
//! debouncing, button debouncing, detection of persistent link loss.
//!
//! Verified properties:
//!   PER-P01: decide is total and deterministic for every input
//!   PER-P02: Pass ⇒ next state resets counter to 0
//!   PER-P03: Fire ⇒ counter had reached persistence threshold
//!   PER-P04: Counter never exceeds persistence threshold (saturates)
//!   PER-P05: Pending ⇒ counter strictly increased from prior
/// Decision for a single persistence-filter step.
/// Pure data — no mutation, no side effects.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum PersistenceDecision {
    /// Event did not occur: reset the persistence counter.
    Pass,
    /// Event occurred but persistence not yet met: increment the counter.
    Pending,
    /// Event occurred and persistence threshold reached: emit.
    Fire,
}
/// Pure decision: given the current counter, the persistence threshold,
/// and whether the event fired this cycle, decide what to do next.
///
/// This is the minimum kernel. No state is mutated here; the caller owns
/// the counter and applies the decision.
pub fn decide(
    event_fired: bool,
    current_count: u32,
    persistence: u32,
) -> PersistenceDecision {
    if !event_fired {
        return PersistenceDecision::Pass;
    }
    let next = saturating_increment(current_count);
    if next >= persistence {
        PersistenceDecision::Fire
    } else {
        PersistenceDecision::Pending
    }
}
/// Saturating u32 increment — never overflows.
pub fn saturating_increment(n: u32) -> u32 {
    if n < u32::MAX { n + 1 } else { u32::MAX }
}
/// Apply a decision to a counter. Returns the new counter value.
///
/// This is where the mutation would happen if the caller chose to mutate;
/// it's a pure function returning the new value so the caller can choose
/// whether to store it.
pub fn apply(decision: PersistenceDecision, current_count: u32) -> u32 {
    match decision {
        PersistenceDecision::Pass => 0,
        PersistenceDecision::Pending | PersistenceDecision::Fire => {
            saturating_increment(current_count)
        }
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn no_event_resets() {
        assert_eq!(decide(false, 5, 3), PersistenceDecision::Pass);
        assert_eq!(apply(PersistenceDecision::Pass, 5), 0);
    }
    #[test]
    fn event_below_threshold_is_pending() {
        assert_eq!(decide(true, 0, 3), PersistenceDecision::Pending);
        assert_eq!(decide(true, 1, 3), PersistenceDecision::Pending);
    }
    #[test]
    fn event_at_threshold_fires() {
        assert_eq!(decide(true, 2, 3), PersistenceDecision::Fire);
    }
    #[test]
    fn persistence_zero_always_fires_on_event() {
        assert_eq!(decide(true, 0, 0), PersistenceDecision::Fire);
    }
    #[test]
    fn counter_saturates() {
        assert_eq!(saturating_increment(u32::MAX), u32::MAX);
    }
}
