//! CRC-16/X.25 — verified mirror of `../plain/src/crc.rs`.
//!
//! Body is `#[verifier::external_body]` (bit-shift / xor loop —
//! Verus's productive range stops at clean integer arithmetic).
//! The CRC algorithm correctness (MAVLINK-P03) is pinned by the
//! proptest fuzz in the plain tests against known-good MAVLink
//! reference vectors.

use vstd::prelude::*;

verus! {

pub struct Crc16X25 {
    pub value: u16,
}

impl Crc16X25 {
    #[verifier::external_body]
    pub fn new() -> Crc16X25 {
        Crc16X25 { value: 0xFFFF }
    }

    #[verifier::external_body]
    pub fn accumulate(&mut self, byte: u8) {
        let mut tmp: u8 = byte ^ ((self.value & 0xFF) as u8);
        tmp = tmp ^ (tmp << 4);
        self.value = (self.value >> 8)
            ^ ((tmp as u16) << 8)
            ^ ((tmp as u16) << 3)
            ^ ((tmp as u16) >> 4);
    }

    #[verifier::external_body]
    pub fn accumulate_slice(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.accumulate(b);
        }
    }

    #[verifier::external_body]
    pub fn value(&self) -> u16 {
        self.value
    }
}

} // verus!
