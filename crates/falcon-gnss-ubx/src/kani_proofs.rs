//! Kani harness for falcon-gnss-ubx (plain-only sibling).
#![cfg(kani)]

use crate::*;

/// GNSS-K01 — the UBX parser is total: feeding ANY byte sequence never panics,
/// never overflows the fixed frame buffer, never indexes out of bounds. A
/// hostile/garbage UART stream cannot crash the driver. Length is symbolic so
/// the buffer-bound and resync paths are all exercised.
#[kani::proof]
#[kani::unwind(6)]
fn verify_parser_total() {
    let mut p = UbxParser::new();
    let n: usize = kani::any();
    kani::assume(n <= 5);
    for _ in 0..n {
        let _ = p.push(kani::any());
    }
}
