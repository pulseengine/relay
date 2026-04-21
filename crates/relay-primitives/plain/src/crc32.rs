//! CRC32 integrity primitive.
//!
//! Standard IEEE 802.3 CRC32 with reflected polynomial 0xEDB88320.
//! Deterministic, pure, no allocation, no state.
//!
//! Extracted from relay-cs/src/engine.rs:27-69 where it was packaged as
//! "cFS Checksum Services". The actual primitive is universal: any
//! integrity check, any wire-format trailer, any storage validator uses it.
//!
//! Verified properties:
//!   CRC-P01: crc32_compute is deterministic (same input ⇒ same output)
//!   CRC-P02: empty input returns 0x00000000
//!   CRC-P03: the loop terminates for any finite input
/// CRC32 lookup table for polynomial 0xEDB88320 (standard reflected).
pub const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i: usize = 0;
    while i < 256 {
        let mut crc: u32 = i as u32;
        let mut j: usize = 0;
        while j < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320u32;
            } else {
                crc = crc >> 1;
            }
            j = j + 1;
        }
        table[i] = crc;
        i = i + 1;
    }
    table
};
/// Compute CRC32 over a byte slice. Pure, deterministic, total.
///
/// CRC-P01: same input always produces same output.
pub fn crc32_compute(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFFu32;
    let mut i: usize = 0;
    while i < data.len() {
        let byte = data[i];
        let raw_index = ((crc ^ (byte as u32)) % 256u32) as usize;
        crc = (crc >> 8) ^ CRC32_TABLE[raw_index];
        i = i + 1;
    }
    crc ^ 0xFFFF_FFFFu32
}
#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn empty_is_zero() {
        assert_eq!(crc32_compute(b""), 0x0000_0000);
    }
    #[test]
    fn known_value_123456789() {
        assert_eq!(crc32_compute(b"123456789"), 0xCBF4_3926);
    }
    #[test]
    fn deterministic() {
        assert_eq!(crc32_compute(b"hello"), crc32_compute(b"hello"));
    }
}
