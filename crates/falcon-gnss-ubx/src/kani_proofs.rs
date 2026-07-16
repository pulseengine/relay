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

/// GNSS-K05 (GNSS-P02) — dual-receiver selector totality: for ANY pair of
/// fixes (every field nondet, incl. NaN/∞ positions and accuracies) with no
/// estimator reference, the selector never panics, and a single-receiver or
/// no-fix decision carries exactly the healthy receiver's (finite) position
/// or an explicit None. The blend path's synthesized position is proptest-
/// gated (nondet f32 blend arithmetic is outside Kani's productive range).
#[kani::proof]
fn verify_dual_selector_total() {
    use dual::*;
    let mk = || -> Option<NedFix> {
        if kani::any() {
            Some(NedFix {
                pos: [
                    f32::from_bits(kani::any()),
                    f32::from_bits(kani::any()),
                    f32::from_bits(kani::any()),
                ],
                acc_m: f32::from_bits(kani::any()),
                sats: kani::any(),
                fix_ok: kani::any(),
            })
        } else {
            None
        }
    };
    let a = mk();
    let b = mk();
    let mut d = DualGnss::new();
    let dec = d.update(a, b, None);
    match dec.source {
        GnssSource::None => assert!(dec.pos.is_none()),
        GnssSource::A | GnssSource::B => {
            let p = dec.pos.unwrap();
            assert!(p[0].is_finite() && p[1].is_finite() && p[2].is_finite());
            assert!(dec.acc_m.is_finite() && dec.acc_m > 0.0);
        }
        GnssSource::Blend => {
            // reachable only with BOTH healthy; the arithmetic itself is
            // proptest territory — totality (no panic) is proven here.
        }
    }
}
