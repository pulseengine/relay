//! DroneCAN v0 transfer CRC — CRC-16-CCITT-FALSE (poly 0x1021, init 0xFFFF, no
//! reflection, no final XOR).
//!
//! For a multi-frame transfer the CRC is computed over the message's 64-bit
//! data-type SIGNATURE (little-endian) followed by the full reassembled
//! payload, and transmitted as the first two bytes of the first frame's data
//! field. The reassembler ([`crate::transfer`]) recomputes it and drops the
//! transfer on a mismatch (the `spar/dronecan.aadl` CRC sink).

/// CRC-16-CCITT-FALSE initial value.
pub const CRC_INIT: u16 = 0xFFFF;

/// Accumulate `data` into a running CRC-16-CCITT-FALSE. Bit-serial (no table —
/// table-free keeps it no_std-clean and the bounded loop Kani-tractable).
/// Total: never panics.
pub fn crc16_add(mut crc: u16, data: &[u8]) -> u16 {
    for &b in data {
        crc ^= (b as u16) << 8;
        let mut i = 0;
        while i < 8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
            i += 1;
        }
    }
    crc
}

/// Seed a transfer CRC with the message's 64-bit data-type signature (the
/// DroneCAN-specified prefix), little-endian, ready for the payload to follow.
pub fn crc16_signature(signature: u64) -> u16 {
    crc16_add(CRC_INIT, &signature.to_le_bytes())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ccitt_false_reference_vector() {
        // CRC-16/CCITT-FALSE("123456789") = 0x29B1 (the canonical check value).
        assert_eq!(crc16_add(CRC_INIT, b"123456789"), 0x29B1);
    }

    #[test]
    fn empty_input_keeps_seed() {
        assert_eq!(crc16_add(CRC_INIT, &[]), CRC_INIT);
    }

    #[test]
    fn accumulation_equals_one_shot() {
        let oneshot = crc16_add(CRC_INIT, b"hello world");
        let split = crc16_add(crc16_add(CRC_INIT, b"hello "), b"world");
        assert_eq!(oneshot, split);
    }

    #[test]
    fn signature_seed_then_payload() {
        // seeding with a signature then adding payload == adding sig-bytes||payload
        let sig: u64 = 0x0102_0304_0506_0708;
        let payload = [0xAA, 0xBB, 0xCC];
        let seeded = crc16_add(crc16_signature(sig), &payload);
        let mut combined = [0u8; 11];
        combined[..8].copy_from_slice(&sig.to_le_bytes());
        combined[8..].copy_from_slice(&payload);
        assert_eq!(seeded, crc16_add(CRC_INIT, &combined));
    }
}
