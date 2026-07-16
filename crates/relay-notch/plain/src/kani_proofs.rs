//! Kani harnesses for relay-notch (plain-only sibling).
//!
//! The verification split (the relay-rc pattern, plus a libm lesson): the
//! trig in `set()` is CBMC-intractable on symbolic arguments (libm's
//! rem_pio2_large unwinds >1500 iterations), so the harnesses keep trig
//! CONCRETE and prove the trig-free properties over all inputs:
//! totality of `apply` for any input AND any (poisoned) state, the
//! never-engage-outside-band safety direction of `set`, and the bank's
//! bit-exact bypass. The engages-in-band liveness direction and the
//! frequency-domain depth/phase are test-measured.
#![cfg(kani)]

use crate::*;

/// NOTCH-K01 — `apply` is TOTAL for ANY input bit pattern and ANY internal
/// state (all four state words nondet, coefficients from a concrete
/// engaged tuning): the output is always finite.
#[kani::proof]
fn verify_apply_total_any_state() {
    let mut n = Notch::default();
    n.set(70.0, 1000.0); // concrete trig — engaged hover-band notch
    assert!(n.is_engaged());
    n.poison_state([
        f32::from_bits(kani::any()),
        f32::from_bits(kani::any()),
        f32::from_bits(kani::any()),
        f32::from_bits(kani::any()),
    ]);
    let x = f32::from_bits(kani::any());
    let y = n.apply(x);
    assert!(y.is_finite());
}

/// NOTCH-K02 — the band gate over ALL inputs, trig-free by construction
/// (`band_ok` is comparisons only; `set` engages exactly when it holds —
/// the one-line linkage is unit-pinned in set_engagement_matches_band_ok):
/// any non-finite / below-floor / above-Nyquist / fs ≤ 0 pair is rejected,
/// and a DISENGAGED notch's `apply` is bit-exact unity for any input.
#[kani::proof]
fn verify_band_gate_and_disengaged_unity() {
    let f0: f32 = kani::any();
    let fs: f32 = kani::any();
    if band_ok(f0, fs) {
        assert!(f0.is_finite() && fs.is_finite());
        assert!(fs > 0.0);
        assert!(f0 >= MIN_F0_HZ);
        assert!(f0 <= MAX_F0_FRAC * fs);
    }
    let mut n = Notch::default();
    let x = f32::from_bits(kani::any());
    let y = n.apply(x);
    assert!(y.to_bits() == x.to_bits(), "disengaged is bit-exact unity");
}

/// NOTCH-K03 — the bank's RPM-absent bypass is bit-exact for ANY input
/// (concrete prior tuning; the None transition disengages everything).
#[kani::proof]
fn verify_bank_bypass_bit_exact() {
    let mut bank: HarmonicNotchBank<4> = HarmonicNotchBank::new(250.0);
    bank.update_rpm(Some([4000, 4100, 3950, 4050])); // concrete trig
    bank.update_rpm(None);
    assert!(bank.is_bypassed());
    let x = [
        f32::from_bits(kani::any()),
        f32::from_bits(kani::any()),
        f32::from_bits(kani::any()),
    ];
    let y = bank.apply(x);
    assert!(y[0].to_bits() == x[0].to_bits());
    assert!(y[1].to_bits() == x[1].to_bits());
    assert!(y[2].to_bits() == x[2].to_bits());
}
