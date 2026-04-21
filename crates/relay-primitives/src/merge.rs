//! Merge primitive — round-robin arbitration between two input queues.
//!
//! The pure decision kernel: given availability of two queues and the
//! identity of whoever produced last, pick which queue to draw from now.
//! No mutation; the caller owns both queues and the last-was-left bit.
//!
//! Used by: sensor fusion (IMU + GPS), command-source multiplexing
//! (ground uplink + stored sequence), redundant telemetry tap merging,
//! priority-queue tie-breaking, dual-bus arbitration.
//!
//! Verified properties:
//!   MRG-P01: None iff both queues empty (no spurious emits)
//!   MRG-P02: Left when only left has data (no starvation of sole producer)
//!   MRG-P03: Right when only right has data (symmetric)
//!   MRG-P04: Under contention, alternates based on last_was_left (fairness)

use vstd::prelude::*;

verus! {

/// Merge arbitration outcome.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum MergeChoice {
    /// Neither queue has data — caller should wait.
    None,
    /// Draw from the left queue.
    Left,
    /// Draw from the right queue.
    Right,
}

/// Pure decision: round-robin merge arbitration.
/// `last_was_left` should be initialized to `false` so the first tie
/// favors Left (matches natural left-to-right reading order).
pub fn merge_choose(
    left_has: bool,
    right_has: bool,
    last_was_left: bool,
) -> (result: MergeChoice)
    ensures
        !left_has && !right_has ==> result === MergeChoice::None,
        left_has && !right_has ==> result === MergeChoice::Left,
        !left_has && right_has ==> result === MergeChoice::Right,
        left_has && right_has && !last_was_left ==> result === MergeChoice::Left,
        left_has && right_has && last_was_left ==> result === MergeChoice::Right,
{
    match (left_has, right_has) {
        (false, false) => MergeChoice::None,
        (true, false) => MergeChoice::Left,
        (false, true) => MergeChoice::Right,
        (true, true) => if last_was_left { MergeChoice::Right } else { MergeChoice::Left },
    }
}

/// Compute the next value of `last_was_left` after a choice is applied.
/// Pure: caller decides whether to store.
pub fn next_last_was_left(choice: MergeChoice, prior: bool) -> (result: bool)
    ensures
        choice === MergeChoice::Left ==> result == true,
        choice === MergeChoice::Right ==> result == false,
        choice === MergeChoice::None ==> result == prior,
{
    match choice {
        MergeChoice::Left => true,
        MergeChoice::Right => false,
        MergeChoice::None => prior,
    }
}

} // verus!

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn both_empty_is_none() {
        assert_eq!(merge_choose(false, false, false), MergeChoice::None);
        assert_eq!(merge_choose(false, false, true), MergeChoice::None);
    }

    #[test]
    fn only_left_picks_left() {
        assert_eq!(merge_choose(true, false, false), MergeChoice::Left);
        assert_eq!(merge_choose(true, false, true), MergeChoice::Left);
    }

    #[test]
    fn only_right_picks_right() {
        assert_eq!(merge_choose(false, true, false), MergeChoice::Right);
        assert_eq!(merge_choose(false, true, true), MergeChoice::Right);
    }

    #[test]
    fn alternates_under_contention() {
        // last was right (false) → pick Left next
        assert_eq!(merge_choose(true, true, false), MergeChoice::Left);
        // last was left (true) → pick Right next
        assert_eq!(merge_choose(true, true, true), MergeChoice::Right);
    }

    #[test]
    fn last_flag_updates_correctly() {
        assert_eq!(next_last_was_left(MergeChoice::Left, false), true);
        assert_eq!(next_last_was_left(MergeChoice::Right, true), false);
        assert_eq!(next_last_was_left(MergeChoice::None, true), true);
        assert_eq!(next_last_was_left(MergeChoice::None, false), false);
    }

    #[test]
    fn fairness_over_four_rounds() {
        // Both queues have data throughout; track who wins each round.
        let mut last = false;
        let mut left_wins = 0u32;
        let mut right_wins = 0u32;
        for _ in 0..4 {
            let c = merge_choose(true, true, last);
            match c {
                MergeChoice::Left => left_wins += 1,
                MergeChoice::Right => right_wins += 1,
                MergeChoice::None => panic!("should not be None"),
            }
            last = next_last_was_left(c, last);
        }
        assert_eq!(left_wins, 2);
        assert_eq!(right_wins, 2);
    }
}
