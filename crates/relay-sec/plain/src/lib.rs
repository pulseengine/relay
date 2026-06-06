//! Relay Sec — no_std, no_alloc safety-protected communication primitives.
//!
//! Shared by falcon (drone C2 link) and wohl (home-automation command path):
//! one verified authentication layer under the relay-ccsds wire format, so a
//! spoofed/replayed "disarm" or "unlock" command is rejected on either product.
//!
//! ## First slice: the anti-replay sliding window (the freshness oracle)
//!
//! Built BEFORE the MAC primitive on purpose. Replay defeat is a *state-machine*
//! property — counter monotonicity — provable with zero cryptography. Writing
//! the MAC first would tempt a "done" on a green crypto test while freshness
//! went unproven. The per-frame authentication (Ascon-AEAD128, NIST SP 800-232 —
//! ONE primitive providing both the MAC-only floor and the AEAD upgrade, in
//! `ascon`) and the frame wrap/verify layer land ON TOP of this window, whose
//! counter feeds the authenticated nonce.
//!
//! Model: RFC 6479 / IPsec-ESP sliding window. Counters are 1-based; 0 is the
//! reserved "never seen" sentinel. `bitmap` bit `k` records that counter
//! `highest - k` was accepted; bit 0 is `highest` itself.
//!
//! ## Verified properties (Kani, SEC-K0*, in `kani_proofs.rs`)
//!
//!   SEC-K01  a counter accepted once is Replay on re-submit (no double-accept)
//!   SEC-K02  accept never lowers the frontier (monotone `highest`)
//!   SEC-K03  accept is total — no panic / overflow / shift-UB for any input
//!   SEC-K04  anything older than the window is TooOld, and leaves state unchanged

#![no_std]
#![forbid(unsafe_code)]

pub mod ascon;

/// Width of the replay window: how far out-of-order a frame may arrive and
/// still be distinguished from a replay. Frames older than this are rejected.
pub const WINDOW_BITS: u64 = 64;

/// The verdict for a candidate frame counter.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum ReplayVerdict {
    /// Strictly newer than anything seen — advances the frontier. Accept.
    Fresh,
    /// Older than the frontier but within the window and not yet seen. Accept.
    Reordered,
    /// Already seen (or the reserved 0 counter). Reject.
    Replay,
    /// Older than the window can remember. Reject conservatively.
    TooOld,
}

impl ReplayVerdict {
    /// Whether this verdict admits the frame.
    pub fn is_accepted(self) -> bool {
        matches!(self, ReplayVerdict::Fresh | ReplayVerdict::Reordered)
    }
}

/// Anti-replay sliding window for one authenticated channel.
///
/// One per `(security-association, channel-id)`: a drone with a radio and a
/// cellular link keeps a window per link so a command recorded on one cannot be
/// replayed on the other.
#[derive(Clone, Copy)]
pub struct ReplayWindow {
    highest: u64,
    bitmap: u64,
}

impl Default for ReplayWindow {
    fn default() -> Self {
        Self::new()
    }
}

impl ReplayWindow {
    /// A fresh window that has accepted nothing.
    pub const fn new() -> Self {
        ReplayWindow {
            highest: 0,
            bitmap: 0,
        }
    }

    /// The highest counter accepted so far (0 if none).
    pub fn highest(&self) -> u64 {
        self.highest
    }

    /// Classify `counter` against the current window WITHOUT mutating state.
    pub fn check(&self, counter: u64) -> ReplayVerdict {
        if counter == 0 {
            return ReplayVerdict::Replay; // 0 is reserved as "never"
        }
        if self.highest == 0 {
            return ReplayVerdict::Fresh; // nothing accepted yet
        }
        if counter > self.highest {
            return ReplayVerdict::Fresh;
        }
        let diff = self.highest - counter;
        if diff >= WINDOW_BITS {
            return ReplayVerdict::TooOld;
        }
        if (self.bitmap >> diff) & 1 == 1 {
            ReplayVerdict::Replay
        } else {
            ReplayVerdict::Reordered
        }
    }

    /// Classify and, if acceptable, record `counter`. Returns the verdict.
    ///
    /// On `Fresh`/`Reordered` the window is updated so the same counter can
    /// never be accepted again; on `Replay`/`TooOld` the window is unchanged.
    pub fn accept(&mut self, counter: u64) -> ReplayVerdict {
        let verdict = self.check(counter);
        match verdict {
            ReplayVerdict::Fresh => {
                if self.highest == 0 {
                    self.bitmap = 1;
                } else {
                    let shift = counter - self.highest;
                    // shift >= 1 here (counter > highest). A jump past the
                    // window forgets all prior bits; otherwise slide and set 0.
                    self.bitmap = if shift >= WINDOW_BITS {
                        1
                    } else {
                        (self.bitmap << shift) | 1
                    };
                }
                self.highest = counter;
            }
            ReplayVerdict::Reordered => {
                let diff = self.highest - counter; // in 1..WINDOW_BITS
                self.bitmap |= 1u64 << diff;
            }
            ReplayVerdict::Replay | ReplayVerdict::TooOld => {}
        }
        verdict
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn first_counter_is_fresh() {
        let mut w = ReplayWindow::new();
        assert_eq!(w.accept(1), ReplayVerdict::Fresh);
        assert_eq!(w.highest(), 1);
    }

    #[test]
    fn zero_is_always_rejected() {
        let mut w = ReplayWindow::new();
        assert_eq!(w.accept(0), ReplayVerdict::Replay);
        assert_eq!(w.highest(), 0);
    }

    #[test]
    fn monotone_sequence_all_fresh() {
        let mut w = ReplayWindow::new();
        for c in 1..=100 {
            assert_eq!(w.accept(c), ReplayVerdict::Fresh);
        }
        assert_eq!(w.highest(), 100);
    }

    #[test]
    fn immediate_replay_rejected() {
        let mut w = ReplayWindow::new();
        w.accept(10);
        assert_eq!(w.accept(10), ReplayVerdict::Replay);
    }

    #[test]
    fn in_window_reorder_accepted_once() {
        let mut w = ReplayWindow::new();
        w.accept(10);
        // 7 is older than the frontier but within the window and unseen.
        assert_eq!(w.accept(7), ReplayVerdict::Reordered);
        // ...and now a replay.
        assert_eq!(w.accept(7), ReplayVerdict::Replay);
        // frontier unmoved by the reorder.
        assert_eq!(w.highest(), 10);
    }

    #[test]
    fn older_than_window_is_too_old() {
        let mut w = ReplayWindow::new();
        w.accept(200);
        // 200 - 64 = 136; anything <= 136 is outside the window.
        assert_eq!(w.accept(136), ReplayVerdict::TooOld);
        assert_eq!(w.accept(100), ReplayVerdict::TooOld);
        assert_eq!(w.highest(), 200);
    }

    #[test]
    fn large_jump_forgets_old_window() {
        let mut w = ReplayWindow::new();
        w.accept(5);
        w.accept(1000); // jump well past the window
        assert_eq!(w.highest(), 1000);
        // 5 is now ancient.
        assert_eq!(w.accept(5), ReplayVerdict::TooOld);
        // and a value just below the new frontier is fresh-reorderable.
        assert_eq!(w.accept(999), ReplayVerdict::Reordered);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// The core safety property over random interleavings: no counter is
        /// ever ACCEPTED (Fresh or Reordered) twice. Values that fall out of the
        /// window are conservatively rejected (TooOld), so they too are never
        /// double-accepted. A 1000-wide `seen` array keeps this no_std/no_alloc.
        #[test]
        fn no_counter_accepted_twice(seq in prop::collection::vec(1u64..1000, 0..400)) {
            let mut w = ReplayWindow::new();
            let mut seen = [false; 1000];
            for c in seq {
                if w.accept(c).is_accepted() {
                    prop_assert!(!seen[c as usize], "counter {} accepted twice", c);
                    seen[c as usize] = true;
                }
            }
        }

        /// The frontier never moves backwards.
        #[test]
        fn frontier_is_monotone(seq in prop::collection::vec(0u64..5000, 0..400)) {
            let mut w = ReplayWindow::new();
            let mut prev = 0u64;
            for c in seq {
                w.accept(c);
                prop_assert!(w.highest() >= prev);
                prev = w.highest();
            }
        }

        /// `check` and `accept` agree on the verdict (accept = check + record).
        #[test]
        fn check_matches_accept(first in 1u64..2000, second in 0u64..2000) {
            let mut w = ReplayWindow::new();
            w.accept(first);
            let predicted = w.check(second);
            prop_assert_eq!(predicted, w.accept(second));
        }
    }
}
