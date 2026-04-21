//! Rate-divider primitive — "emit every N ticks."
//!
//! Extracted from relay-md/src/engine.rs:140-184 where it was "Memory Dwell
//! scheduling". The actual primitive is: given a monotonically increasing
//! counter and a divisor, decide whether this cycle should emit.
//!
//! Used by: periodic telemetry, downsampling, control-loop rate scheduling,
//! watchdog strobes, log rotation cadence.
//!
//! Verified properties:
//!   RD-P01: should_emit is total and deterministic
//!   RD-P02: divisor == 0 treated as "never emit" (guards caller error)
//!   RD-P03: divisor == 1 emits every cycle
//!   RD-P04: emit pattern repeats with period = divisor

use vstd::prelude::*;

verus! {

/// Decide whether this cycle should emit, given the monotonic cycle counter.
///
/// A divisor of 0 is defined as "never emit" (safe default for uninitialized
/// config). A divisor of 1 emits every cycle. A divisor of N emits when
/// cycle % N == 0.
pub fn should_emit(cycle: u64, divisor: u32) -> (result: bool)
    ensures
        divisor == 0 ==> result == false,
        divisor == 1 ==> result == true,
        divisor >  1 ==> result == ((cycle % (divisor as u64)) == 0),
{
    if divisor == 0 {
        false
    } else if divisor == 1 {
        true
    } else {
        (cycle % (divisor as u64)) == 0
    }
}

} // verus!

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn divisor_zero_never_emits() {
        for c in 0..10 {
            assert!(!should_emit(c, 0));
        }
    }

    #[test]
    fn divisor_one_always_emits() {
        for c in 0..10 {
            assert!(should_emit(c, 1));
        }
    }

    #[test]
    fn divisor_five_emits_every_fifth() {
        assert!(should_emit(0, 5));
        assert!(!should_emit(1, 5));
        assert!(!should_emit(4, 5));
        assert!(should_emit(5, 5));
        assert!(should_emit(10, 5));
    }
}
