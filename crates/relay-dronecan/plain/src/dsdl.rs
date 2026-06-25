//! Faithful port of the DroneCAN v0 (UAVCAN v0) DSDL bit serialization.
//!
//! THE conformance core. The wire convention (pydronecan's `be_from_le_bits` /
//! `le_from_be_bits`, the same the PX4/ArduPilot libcanard implementations use):
//! the bit stream is filled MSB-first within each byte, and a primitive of width
//! `w` is stored byte-**little-endian** with each byte emitted MSB-first.
//!
//! Consequence — and the bug this module fixes: a BYTE-ALIGNED field reduces to
//! `from_le_bytes` (which is why the float16/u32 decodes were already correct),
//! but a SUB-BYTE or BIT-MISALIGNED field (NodeStatus health/mode, esc.RawCommand
//! int14, esc.Status rpm) is NOT simple LSB-first packing — the earlier
//! hand-rolled `pos/8, 1<<(pos%8)` readers were wrong against the real DSDL, and
//! every self-constructed round-trip test missed it because encode and decode
//! shared the same wrong convention. Validated here against pydronecan reference
//! vectors (`tests::pydronecan_reference_vectors` in the message modules).
//!
//! All reads/writes are bounds-guarded — total, never panic.

/// One stream bit at global position `p` (MSB-first within each byte).
/// Out-of-range -> 0.
#[inline]
fn stream_bit(payload: &[u8], p: usize) -> u64 {
    let byte = p / 8;
    if byte >= payload.len() {
        return 0;
    }
    ((payload[byte] >> (7 - (p % 8))) & 1) as u64
}

/// Read a `w`-bit unsigned DSDL primitive at bit offset `o` (w <= 64).
/// `be_from_le_bits`: the value's bytes are little-endian, each byte MSB-first
/// in the stream — i.e. 8-bit chunks of the field are reversed. Total.
pub fn read_uint(payload: &[u8], o: usize, w: usize) -> u64 {
    let mut val: u64 = 0;
    let nchunks = w.div_ceil(8);
    let mut chunk = nchunks;
    // most-significant chunk (highest stream offset) first
    while chunk > 0 {
        chunk -= 1;
        let start = chunk * 8;
        let end = if start + 8 < w { start + 8 } else { w };
        let mut c: u64 = 0;
        let mut i = start;
        while i < end {
            c = (c << 1) | stream_bit(payload, o + i);
            i += 1;
        }
        val = (val << (end - start)) | c;
    }
    val
}

/// Read a `w`-bit signed DSDL primitive (two's complement), sign-extended to i64.
pub fn read_int(payload: &[u8], o: usize, w: usize) -> i64 {
    let v = read_uint(payload, o, w);
    if w == 0 || w >= 64 {
        return v as i64;
    }
    if v & (1 << (w - 1)) != 0 {
        (v as i64) - (1i64 << w)
    } else {
        v as i64
    }
}

/// Write a `w`-bit unsigned value at bit offset `o` (`le_from_be_bits`, the
/// inverse of [`read_uint`]). The value's little-endian bytes are emitted
/// MSB-first per stream byte. Out-of-range writes are dropped (total). Only the
/// low `w` bits of `value` are used.
pub fn write_uint(payload: &mut [u8], o: usize, w: usize, value: u64) {
    let nchunks = w.div_ceil(8);
    let mut chunk = 0;
    while chunk < nchunks {
        let start = chunk * 8;
        let end = if start + 8 < w { start + 8 } else { w };
        let cbits = end - start;
        let byte = (value >> (chunk * 8)) & 0xFF;
        let mut k = 0;
        while k < cbits {
            let bit = (byte >> (cbits - 1 - k)) & 1; // MSB-first within the chunk
            let p = o + start + k;
            let pb = p / 8;
            if pb < payload.len() {
                let mask = 1u8 << (7 - (p % 8));
                if bit != 0 {
                    payload[pb] |= mask;
                } else {
                    payload[pb] &= !mask;
                }
            }
            k += 1;
        }
        chunk += 1;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn byte_aligned_matches_le_bytes() {
        // a byte-aligned u32 read == from_le_bytes (proves the float16/u32 path
        // was always correct).
        let p = [0x04, 0x03, 0x02, 0x01];
        assert_eq!(read_uint(&p, 0, 32), 0x0102_0304);
    }

    #[test]
    fn nodestatus_subbyte_fields() {
        // byte 4 = 0x99 packs health=2 / mode=3 / sub_mode=1 (MSB-first), per the
        // pydronecan canonical NodeStatus 0403020199efbe.
        let p = [0x04, 0x03, 0x02, 0x01, 0x99, 0xef, 0xbe];
        assert_eq!(read_uint(&p, 32, 2), 2); // health
        assert_eq!(read_uint(&p, 34, 3), 3); // mode
        assert_eq!(read_uint(&p, 37, 3), 1); // sub_mode
    }

    #[test]
    fn raw_command_int14_against_reference() {
        // pydronecan canonical RawCommand [8191,0,-8192,4096] = ff7c0000080010
        let p = [0xff, 0x7c, 0x00, 0x00, 0x08, 0x00, 0x10];
        assert_eq!(read_int(&p, 0, 14), 8191);
        assert_eq!(read_int(&p, 14, 14), 0);
        assert_eq!(read_int(&p, 28, 14), -8192);
        assert_eq!(read_int(&p, 42, 14), 4096);
    }

    #[test]
    fn write_then_read_round_trips() {
        // the codec is self-inverse for any offset/width/value in range
        for &(o, w, v) in &[(0usize, 14usize, 8191u64), (14, 14, 0x2000), (32, 2, 2), (34, 3, 5), (5, 18, 0x3ABCD)] {
            let mut buf = [0u8; 16];
            write_uint(&mut buf, o, w, v);
            assert_eq!(read_uint(&buf, o, w), v & ((1u64 << w) - 1), "o={o} w={w}");
        }
    }

    #[test]
    fn write_reproduces_reference_bytes() {
        // encoding [8191,0,-8192,4096] must reproduce the pydronecan bytes
        let mut buf = [0u8; 7];
        write_uint(&mut buf, 0, 14, 8191);
        write_uint(&mut buf, 14, 14, 0);
        write_uint(&mut buf, 28, 14, (-8192i64 as u64) & 0x3FFF);
        write_uint(&mut buf, 42, 14, 4096);
        assert_eq!(buf, [0xff, 0x7c, 0x00, 0x00, 0x08, 0x00, 0x10]);
    }
}
