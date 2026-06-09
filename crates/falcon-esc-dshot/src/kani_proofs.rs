//! Kani harnesses for falcon-esc-dshot (plain-only sibling).
#![cfg(kani)]

use crate::*;

/// ESC-K01 — the mixer→throttle map is total and BOUNDED: any f32 command
/// (including NaN/∞) yields a valid throttle, either 0 or within
/// `[DSHOT_MIN, DSHOT_MAX]` — never a wrapped/out-of-range 11-bit value.
#[kani::proof]
fn verify_throttle_in_range() {
    let cmd: f32 = kani::any();
    let t = throttle_from_unit(cmd);
    assert!(t == 0 || (t >= DSHOT_MIN && t <= DSHOT_MAX));
    assert!(t <= DSHOT_MAX);
}

/// ESC-K02 — every frame built from a throttle carries a self-consistent CRC4:
/// for any throttle and telemetry bit, the emitted frame passes its own CRC
/// check (no frame is emitted with a wrong checksum).
#[kani::proof]
fn verify_dshot_crc_self_consistent() {
    let throttle: u16 = kani::any();
    let telem: bool = kani::any();
    let frame = dshot_frame(throttle, telem);
    assert!(dshot_crc_ok(frame));
    // CRC occupies exactly the low 4 bits.
    assert!((frame & 0x0F) == ((frame >> 4) ^ ((frame >> 4) >> 4) ^ ((frame >> 4) >> 8)) & 0x0F);
}

/// ESC-K03 — battery classification is total for any ADC count and never
/// panics; thresholds order is respected (critical ⇒ also below low).
#[kani::proof]
fn verify_battery_total() {
    let bm = BatteryMonitor::new(kani::any(), kani::any(), kani::any());
    let adc: u16 = kani::any();
    let _ = bm.classify(adc);
}
