//! Kani bounded-model-checking harnesses for relay-bus (the transport seam).
//!
//! Kept in this sibling module (declared from lib.rs) following the relay
//! convention. These prove the `no_alloc` reference ring is a sound carrier:
//! total (no panic) under arbitrary operation sequences, and bounded.
#![cfg(kani)]

use crate::{MessageTransport, SpscRing};

/// BUS-K01 — a bounded sequence of ARBITRARY push/pop operations on the ring
/// is total (no panic, no overflow, no shift/modulo UB) and the length
/// invariant `0 <= len <= N` holds after every step. The operation choice and
/// the pushed values are symbolic.
#[kani::proof]
#[kani::unwind(11)]
fn ring_ops_total_and_bounded() {
    const N: usize = 4;
    let mut ring: SpscRing<u8, N> = SpscRing::new();
    let mut i = 0;
    while i < 10 {
        if kani::any() {
            let _ = ring.push(kani::any());
        } else {
            let _ = ring.pop();
        }
        assert!(ring.len() <= N);
        i += 1;
    }
}

/// BUS-K02 — a full ring refuses the next push (backpressure) and never
/// overwrites: after filling N slots, push returns false, len stays N, and the
/// oldest item is still recoverable in FIFO order.
#[kani::proof]
fn ring_full_refuses_no_overwrite() {
    const N: usize = 3;
    let mut ring: SpscRing<u8, N> = SpscRing::new();
    let a: u8 = kani::any();
    assert!(ring.push(a));
    assert!(ring.push(kani::any()));
    assert!(ring.push(kani::any()));
    assert!(!ring.push(kani::any())); // full -> refused
    assert!(ring.len() == N);
    assert!(ring.pop() == Some(a)); // FIFO preserved, no overwrite of the head
}
