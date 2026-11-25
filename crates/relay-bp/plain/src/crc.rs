//! CRC-16/X-25 (CRC type 1) and CRC-32C (CRC type 2) per RFC 9171 4.2.1.
//!
//! Both are reflected CRCs with all-ones init and all-ones final XOR:
//!
//! | name         | poly (reflected) | check("123456789") |
//! |--------------|------------------|--------------------|
//! | CRC-16/X-25  | 0x8408           | 0x906E             |
//! | CRC-32C      | 0x82F63B78       | 0xE3069283         |
//!
//! [`block_crc`] computes a block CRC the way RFC 9171 4.3.1/4.3.2 and erratum
//! 8043 (Verified) define it: over every byte of the block, with ONLY the CRC
//! value bytes taken as zero. The CBOR byte-string header in front of them
//! (0x42 / 0x44) is kept. The value is big-endian on the wire (4.2.2).
//!
//! The tables are built at compile time by `const fn`; there is no runtime
//! initialisation and no allocation.

use crate::CrcType;

/// Published check value of CRC-16/X-25 over the ASCII bytes "123456789".
pub const CRC16_X25_CHECK: u16 = 0x906e;
/// Published check value of CRC-32C over the ASCII bytes "123456789".
pub const CRC32C_CHECK: u32 = 0xe306_9283;

const POLY16: u16 = 0x8408;
const POLY32: u32 = 0x82f6_3b78;

const fn make_table16() -> [u16; 256] {
    let mut t = [0u16; 256];
    let mut i = 0;
    while i < 256 {
        t[i] = bitwise16(0, i as u8);
        i += 1;
    }
    t
}

const fn make_table32() -> [u32; 256] {
    let mut t = [0u32; 256];
    let mut i = 0;
    while i < 256 {
        t[i] = bitwise32(0, i as u8);
        i += 1;
    }
    t
}

/// The table-free reference step: shift one byte through the register.
/// `bitwise16(0, b)` is table entry `b`; the table step equals this step for
/// every state and byte (Kani `verify_crc16_table_matches_bitwise`).
pub const fn bitwise16(state: u16, byte: u8) -> u16 {
    let mut c = state ^ byte as u16;
    let mut k = 0;
    while k < 8 {
        c = if c & 1 != 0 {
            (c >> 1) ^ POLY16
        } else {
            c >> 1
        };
        k += 1;
    }
    c
}

/// The table-free reference step for CRC-32C.
pub const fn bitwise32(state: u32, byte: u8) -> u32 {
    let mut c = state ^ byte as u32;
    let mut k = 0;
    while k < 8 {
        c = if c & 1 != 0 {
            (c >> 1) ^ POLY32
        } else {
            c >> 1
        };
        k += 1;
    }
    c
}

static TABLE16: [u16; 256] = make_table16();
static TABLE32: [u32; 256] = make_table32();

/// One table-driven CRC-16/X-25 register step.
#[inline]
pub fn step16(state: u16, byte: u8) -> u16 {
    (state >> 8) ^ TABLE16[usize::from((state as u8) ^ byte)]
}

/// One table-driven CRC-32C register step.
#[inline]
pub fn step32(state: u32, byte: u8) -> u32 {
    (state >> 8) ^ TABLE32[usize::from((state as u8) ^ byte)]
}

/// CRC-16/X-25 over `data`.
pub fn crc16_x25(data: &[u8]) -> u16 {
    let mut s: u16 = 0xffff;
    for &b in data {
        s = step16(s, b);
    }
    !s
}

/// CRC-32C over `data`.
pub fn crc32c(data: &[u8]) -> u32 {
    let mut s: u32 = 0xffff_ffff;
    for &b in data {
        s = step32(s, b);
    }
    !s
}

/// The CRC of `block` (the complete CBOR encoding of one block, including any
/// break code) with the `crc_type.value_len()` bytes starting at `value_at`
/// taken as zero and every other byte, including the byte-string header,
/// taken as is (RFC 9171 erratum 8043). Returns `None` for `CrcType::None` or
/// when the value field does not lie inside `block`. Widened to `u32`.
pub fn block_crc(crc_type: CrcType, block: &[u8], value_at: usize) -> Option<u32> {
    let len = crc_type.value_len();
    if len == 0 {
        return None;
    }
    let value_end = value_at.checked_add(len)?;
    if value_end > block.len() {
        return None;
    }
    match crc_type {
        CrcType::Crc16 => {
            let mut s: u16 = 0xffff;
            for (i, &b) in block.iter().enumerate() {
                let b = if i >= value_at && i < value_end { 0 } else { b };
                s = step16(s, b);
            }
            Some(u32::from(!s))
        }
        CrcType::Crc32c => {
            let mut s: u32 = 0xffff_ffff;
            for (i, &b) in block.iter().enumerate() {
                let b = if i >= value_at && i < value_end { 0 } else { b };
                s = step32(s, b);
            }
            Some(!s)
        }
        CrcType::None => None,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::vec::Vec;

    #[test]
    fn crc16_x25_check_value() {
        // reveng catalogue, CRC-16/IBM-SDLC (alias X-25): check=0x906e.
        assert_eq!(crc16_x25(b"123456789"), CRC16_X25_CHECK);
    }

    #[test]
    fn crc32c_check_value() {
        // reveng catalogue, CRC-32/ISCSI (alias CRC-32C): check=0xe3069283.
        assert_eq!(crc32c(b"123456789"), CRC32C_CHECK);
    }

    #[test]
    fn crc32c_rfc3720_b4_vectors() {
        // RFC 3720 Appendix B.4 "CRC Examples". The RFC lists the CRC bytes in
        // iSCSI transmission order (least significant byte first), e.g.
        // "aa 36 91 8a" for 32 bytes of zeroes, i.e. the value 0x8a9136aa.
        let zeros = [0u8; 32];
        let ones = [0xffu8; 32];
        let inc: Vec<u8> = (0u8..32).collect();
        let dec: Vec<u8> = (0u8..32).rev().collect();
        assert_eq!(crc32c(&zeros).to_le_bytes(), [0xaa, 0x36, 0x91, 0x8a]);
        assert_eq!(crc32c(&ones).to_le_bytes(), [0x43, 0xab, 0xa8, 0x62]);
        assert_eq!(crc32c(&inc).to_le_bytes(), [0x4e, 0x79, 0xdd, 0x46]);
        assert_eq!(crc32c(&dec).to_le_bytes(), [0x5c, 0xdb, 0x3f, 0x11]);
        // RFC 3720 B.4 "An iSCSI - SCSI Read (10) Command PDU", CRC 56 3a 96 d9.
        let pdu: [u8; 48] = [
            0x01, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
            0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x14,
            0x00, 0x00, 0x00, 0x18, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ];
        assert_eq!(crc32c(&pdu).to_le_bytes(), [0x56, 0x3a, 0x96, 0xd9]);
    }

    #[test]
    fn crc_zeroes_only_value_bytes_erratum_8043() {
        // A canonical block with a CRC-16 field: ... 0x42 <v0> <v1>.
        // The value bytes hold garbage; the header byte 0x42 stays in.
        let block = [
            0x86, 0x07, 0x02, 0x00, 0x01, 0x43, 0x19, 0x01, 0x2c, 0x42, 0xde, 0xad,
        ];
        let value_at = 10;
        let mut zeroed_value = block;
        zeroed_value[10] = 0;
        zeroed_value[11] = 0;
        let expect = u32::from(crc16_x25(&zeroed_value));
        assert_eq!(block_crc(CrcType::Crc16, &block, value_at), Some(expect));

        // The pre-erratum misreading (zeroing the header too) gives a
        // different value, so the test would catch that implementation.
        let mut zeroed_header_too = zeroed_value;
        zeroed_header_too[9] = 0;
        assert_ne!(Some(u32::from(crc16_x25(&zeroed_header_too))), Some(expect));

        // Same for CRC-32C.
        let block32 = [0x86, 0x07, 0x02, 0x00, 0x02, 0x41, 0x00, 0x44, 1, 2, 3, 4];
        let mut z = block32;
        z[8..12].fill(0);
        assert_eq!(block_crc(CrcType::Crc32c, &block32, 8), Some(crc32c(&z)));
        let mut zh = z;
        zh[7] = 0;
        assert_ne!(crc32c(&zh), crc32c(&z));

        // A value field outside the block is not computed.
        assert_eq!(block_crc(CrcType::Crc16, &block, 11), None);
        assert_eq!(block_crc(CrcType::None, &block, 0), None);
    }
}
