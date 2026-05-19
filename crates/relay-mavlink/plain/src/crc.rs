//! CRC-16/X.25 — the MAVLink CRC variant.
//!
//! Polynomial: 0x1021 (CRC-CCITT).
//! Initial value: 0xFFFF.
//! Reflected: yes (input bytes processed with reflected polynomial).
//! Final XOR: 0x0000.
//!
//! Algorithm transcribed from the MAVLink reference (mavgen-c).
//! Includes the per-message `CRC_EXTRA` byte: after computing CRC
//! over the frame, accumulate one more byte (the message-specific
//! magic) and that's the final CRC.
//!
//! Per SWREQ-FALCON-MAVLINK-P03 this must match the reference impl
//! byte-for-byte; tests use known-good vectors.

/// CRC-16/X.25 accumulator. Initialize with `Crc16X25::new()`,
/// feed bytes one at a time via `accumulate()`, finalize with
/// `accumulate(crc_extra)` once you have the message-specific
/// CRC_EXTRA byte from the MAVLink XML.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Crc16X25 {
    value: u16,
}

impl Crc16X25 {
    /// Fresh accumulator initialised to the MAVLink seed (0xFFFF).
    pub const fn new() -> Self {
        Self { value: 0xFFFF }
    }

    /// Accumulate one byte into the CRC.
    pub fn accumulate(&mut self, b: u8) {
        // Reference: crc_accumulate() in mavgen-c.
        let mut tmp: u8 = b ^ (self.value as u8);
        tmp ^= tmp << 4;
        self.value = (self.value >> 8)
            ^ ((tmp as u16) << 8)
            ^ ((tmp as u16) << 3)
            ^ ((tmp as u16) >> 4);
    }

    /// Accumulate a slice of bytes in order.
    pub fn accumulate_slice(&mut self, bytes: &[u8]) {
        for &b in bytes {
            self.accumulate(b);
        }
    }

    /// Current CRC value. Low byte goes on the wire first
    /// (MAVLink is little-endian).
    pub const fn value(self) -> u16 {
        self.value
    }
}

impl Default for Crc16X25 {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Reference vector for CRC-16/MCRF4XX (the MAVLink variant):
    /// CRC of "123456789" with init=0xFFFF, no final xor-out, refin/refout
    /// implicit in the bit ordering of the mavgen-c algorithm, yields
    /// 0x6F91. This is the standard MCRF4XX check value (Catalogue of
    /// parametrised CRC algorithms, Cook 2018). MAVLink-correct.
    ///
    /// Note: the strict X.25 CRC (with xorout=0xFFFF) on the same input
    /// would give 0x906E — that's not what MAVLink uses, despite the
    /// historical naming of this struct as `Crc16X25`. The bit-ordering
    /// here matches mavgen-c's `crc_accumulate` byte-for-byte; only the
    /// xor-out differs.
    #[test]
    fn mavlink_reference_vector_123456789() {
        let mut crc = Crc16X25::new();
        crc.accumulate_slice(b"123456789");
        assert_eq!(crc.value(), 0x6F91);
    }

    #[test]
    fn empty_input_keeps_seed() {
        let crc = Crc16X25::new();
        assert_eq!(crc.value(), 0xFFFF);
    }

    #[test]
    fn single_byte_zero() {
        let mut crc = Crc16X25::new();
        crc.accumulate(0x00);
        // Trace from 0xFFFF on input 0x00 (verified by hand against mavgen-c):
        //   tmp = 0x00 ^ 0xFF = 0xFF
        //   tmp ^= tmp << 4 (u8): 0xFF ^ 0xF0 = 0x0F
        //   value = (0xFFFF >> 8) ^ (0x0F << 8) ^ (0x0F << 3) ^ (0x0F >> 4)
        //         = 0x00FF ^ 0x0F00 ^ 0x0078 ^ 0x0000
        //         = 0x0F87
        assert_eq!(crc.value(), 0x0F87);
    }

    #[test]
    fn accumulate_slice_equals_individual() {
        let bytes = [0xAB, 0xCD, 0xEF, 0x12, 0x34];
        let mut a = Crc16X25::new();
        for &b in &bytes {
            a.accumulate(b);
        }
        let mut b = Crc16X25::new();
        b.accumulate_slice(&bytes);
        assert_eq!(a.value(), b.value());
    }

    #[test]
    fn order_matters() {
        let mut a = Crc16X25::new();
        a.accumulate_slice(&[0x01, 0x02, 0x03]);
        let mut b = Crc16X25::new();
        b.accumulate_slice(&[0x03, 0x02, 0x01]);
        assert_ne!(a.value(), b.value());
    }
}
