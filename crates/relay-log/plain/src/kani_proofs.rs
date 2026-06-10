//! Kani harnesses for relay-log (plain-only sibling).
#![cfg(kani)]

use crate::*;

/// LOG-K01 — the ring is total: recording an arbitrary number of entries against
/// a fresh log never panics / never indexes out of bounds, and the held count
/// never exceeds the capacity. (Concrete capacity 4; symbolic record count.)
#[kani::proof]
#[kani::unwind(11)]
fn verify_ring_total_and_bounded() {
    let mut log: FlightLog<4> = FlightLog::new();
    let n: u32 = kani::any();
    kani::assume(n <= 10);
    let mut i = 0;
    while i < n {
        log.record(LogEntry { t_ms: i, kind: 0, data: [0.0; 2] });
        assert!(log.len() <= 4);
        i += 1;
    }
    assert!(log.len() <= 4);
}

/// LOG-K02 — encode∘decode is the identity for any record (the black box never
/// corrupts a field): for symbolic timestamp/kind/payload bits, decoding the
/// encoded record reproduces it bit-for-bit.
#[kani::proof]
fn verify_encode_decode_identity() {
    let e = LogEntry {
        t_ms: kani::any(),
        kind: kani::any(),
        data: [
            f32::from_bits(kani::any()),
            f32::from_bits(kani::any()),
        ],
    };
    let back = LogEntry::decode(&e.encode());
    assert!(back.t_ms == e.t_ms);
    assert!(back.kind == e.kind);
    assert!(back.data[0].to_bits() == e.data[0].to_bits());
    assert!(back.data[1].to_bits() == e.data[1].to_bits());
}
