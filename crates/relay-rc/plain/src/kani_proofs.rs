//! Kani harnesses for relay-rc (plain-only sibling).
//!
//! The verification split (the codebase pattern, as in relay-avoid): the
//! COMPARISON/clamp logic is proven here over ALL inputs — a glitching receiver's
//! stick is clamped into [-1,1] (and throttle into [0,1]) BEFORE it is scaled, so
//! it can never command an unbounded angle/rate. The scaled-output bound (the f32
//! multiply by the limit) is proptest-gated — Kani on f32 multiplication is
//! intractable — and follows from the clamp + a non-negative limit.
#![cfg(kani)]

use crate::*;

/// RC-K01 — the stick clamp is the safety floor: `unit(x)` is in [-1, 1] for ANY
/// f32 input (incl. NaN/∞). Centre (0) is the safe default for NaN.
#[kani::proof]
fn verify_unit_clamped() {
    let x: f32 = kani::any();
    let u = unit(x);
    assert!(u >= -1.0 && u <= 1.0);
    if x.is_nan() {
        assert!(u == 0.0);
    }
}

/// RC-K02 — the throttle clamp is in [0, 1] for ANY f32 input; NaN ⇒ 0 (idle).
#[kani::proof]
fn verify_throttle_clamped() {
    let x: f32 = kani::any();
    let t = throttle01(x);
    assert!(t >= 0.0 && t <= 1.0);
    if x.is_nan() {
        assert!(t == 0.0);
    }
}

/// RC-K03 — the SBUS unpack is total and range-bounded: for ANY 25-byte frame,
/// `decode_sbus` never panics, and when it returns channels EVERY channel is an
/// 11-bit count (`<= 0x7FF`). This is the integer floor under the f32 stick
/// mapping — a glitching/garbage receiver frame can never produce an
/// out-of-range channel count or index out of bounds. (The scaled f32 stick
/// bound is proptest-gated; Kani on f32 multiply/divide is intractable.)
#[kani::proof]
fn verify_sbus_unpack_bounded() {
    let frame: [u8; SBUS_FRAME_LEN] = kani::any();
    if let Some(s) = decode_sbus(&frame) {
        let mut i = 0;
        while i < 16 {
            assert!(s.ch[i] <= 0x07FF);
            i += 1;
        }
    }
}

/// RC-K04 — the CRSF decode is total and range-bounded: for ANY 26-byte frame,
/// `decode_crsf_rc` never panics (incl. the CRC-8 loop and the shared 11-bit
/// unpack), and when it returns channels EVERY channel is `<= 0x7FF`. A garbage
/// CRSF frame can never produce an out-of-range channel or index out of bounds;
/// a bad CRC yields None (the integrity check SBUS lacks). The scaled f32 stick
/// bound is proptest-gated.
#[kani::proof]
fn verify_crsf_unpack_bounded() {
    let frame: [u8; CRSF_RC_FRAME_LEN] = kani::any();
    if let Some(c) = decode_crsf_rc(&frame) {
        let mut i = 0;
        while i < 16 {
            assert!(c.ch[i] <= 0x07FF);
            i += 1;
        }
    }
}
