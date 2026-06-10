//! Kani harnesses for falcon-baromag (plain-only sibling).
#![cfg(kani)]

use crate::baro::*;
use crate::mag::*;
use crate::*;

/// A symbolic register bus: every read returns a fresh nondeterministic byte, so
/// the decode is exercised over ALL possible register contents.
struct AnyBus;
impl RegBus for AnyBus {
    fn read_reg(&mut self, _reg: u8) -> u8 {
        kani::any()
    }
    fn write_reg(&mut self, _reg: u8, _val: u8) {}
    fn read_burst(&mut self, _reg: u8, buf: &mut [u8]) {
        for b in buf.iter_mut() {
            *b = kani::any();
        }
    }
}

/// BAROMAG-K01 — the magnetometer decode is total: any register/data contents
/// yield a finite field vector, no panic.
#[kani::proof]
fn verify_mag_decode_total() {
    let mut m = Ist8310::new(AnyBus);
    let _ = m.init();
    let f = m.read_field();
    assert!(f.x_ut.is_finite() && f.y_ut.is_finite() && f.z_ut.is_finite());
    let _ = flat_heading(f);
}

/// BAROMAG-K02 — the barometer decode is total: any register/data contents yield
/// finite pressure/temperature, no panic, and the raw words are 24-bit.
#[kani::proof]
fn verify_baro_decode_total() {
    let mut b = Bmp388::new(AnyBus);
    let _ = b.init();
    let s = b.read();
    assert!(s.pressure_pa.is_finite() && s.temp_c.is_finite());
    assert!(s.raw_pressure <= 0x00FF_FFFF);
    assert!(s.raw_temp <= 0x00FF_FFFF);
}
