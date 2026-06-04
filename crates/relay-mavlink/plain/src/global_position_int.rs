//! GLOBAL_POSITION_INT message — MAVLink id 33.
//!
//! Filtered fused-position estimate from the FC. Most ground stations
//! (QGroundControl, MAVSDK, Auterion) subscribe to this; PX4 and
//! ArduPilot both emit it at 10 Hz or faster by default.
//!
//! This is the message the v0.12 HITL harness consumes to drive the
//! verified `relay-lc` Geofence from a real FC's view of the world.
//!
//! Payload (28 bytes, little-endian, MAVLink-canonical field order
//! — sorted by size descending):
//!
//!   offset  0..4    time_boot_ms     u32   ms since boot
//!   offset  4..8    lat              i32   degrees * 1e7
//!   offset  8..12   lon              i32   degrees * 1e7
//!   offset 12..16   alt              i32   mm AMSL (WGS-84 geoid)
//!   offset 16..20   relative_alt     i32   mm above home
//!   offset 20..22   vx               i16   cm/s, NED north
//!   offset 22..24   vy               i16   cm/s, NED east
//!   offset 24..26   vz               i16   cm/s, NED down
//!   offset 26..28   hdg              u16   cdeg, 0..35999, or 65535 if unknown
//!
//! CRC_EXTRA for GLOBAL_POSITION_INT: **104** (from MAVLink XML).
//! Verify against `mavgen-c` output if this ever needs to change.

/// MAVLink message ID for GLOBAL_POSITION_INT.
pub const GLOBAL_POSITION_INT_MSG_ID: u32 = 33;

/// GLOBAL_POSITION_INT payload length in bytes.
pub const GLOBAL_POSITION_INT_PAYLOAD_LEN: usize = 28;

/// GLOBAL_POSITION_INT CRC_EXTRA byte (from MAVLink XML).
pub const GLOBAL_POSITION_INT_CRC_EXTRA: u8 = 104;

/// Sentinel for unknown heading per MAVLink.
pub const HEADING_UNKNOWN: u16 = u16::MAX;

/// GLOBAL_POSITION_INT decoded form.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GlobalPositionInt {
    pub time_boot_ms: u32,
    /// Degrees * 1e7.
    pub lat_e7: i32,
    /// Degrees * 1e7.
    pub lon_e7: i32,
    /// mm AMSL (WGS-84 geoid).
    pub alt_mm: i32,
    /// mm above home.
    pub relative_alt_mm: i32,
    /// NED north velocity, cm/s.
    pub vx_cms: i16,
    /// NED east velocity, cm/s.
    pub vy_cms: i16,
    /// NED down velocity, cm/s.
    pub vz_cms: i16,
    /// Heading in cdeg, 0..35999, or HEADING_UNKNOWN (=65535).
    pub hdg_cdeg: u16,
}

impl GlobalPositionInt {
    /// Encode into the 28-byte payload form.
    pub fn encode_payload(&self) -> [u8; GLOBAL_POSITION_INT_PAYLOAD_LEN] {
        let mut out = [0u8; GLOBAL_POSITION_INT_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.time_boot_ms.to_le_bytes());
        out[4..8].copy_from_slice(&self.lat_e7.to_le_bytes());
        out[8..12].copy_from_slice(&self.lon_e7.to_le_bytes());
        out[12..16].copy_from_slice(&self.alt_mm.to_le_bytes());
        out[16..20].copy_from_slice(&self.relative_alt_mm.to_le_bytes());
        out[20..22].copy_from_slice(&self.vx_cms.to_le_bytes());
        out[22..24].copy_from_slice(&self.vy_cms.to_le_bytes());
        out[24..26].copy_from_slice(&self.vz_cms.to_le_bytes());
        out[26..28].copy_from_slice(&self.hdg_cdeg.to_le_bytes());
        out
    }

    /// Decode from a 28-byte payload slice. Returns `None` if the
    /// length is wrong (the caller validates the message-id at the
    /// frame layer).
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != GLOBAL_POSITION_INT_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            time_boot_ms: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            lat_e7: i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            lon_e7: i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            alt_mm: i32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
            relative_alt_mm: i32::from_le_bytes([
                payload[16],
                payload[17],
                payload[18],
                payload[19],
            ]),
            vx_cms: i16::from_le_bytes([payload[20], payload[21]]),
            vy_cms: i16::from_le_bytes([payload[22], payload[23]]),
            vz_cms: i16::from_le_bytes([payload[24], payload[25]]),
            hdg_cdeg: u16::from_le_bytes([payload[26], payload[27]]),
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample() -> GlobalPositionInt {
        GlobalPositionInt {
            time_boot_ms: 1_234_567,
            lat_e7: 475_023_456,    // 47.5023456°
            lon_e7: 190_401_234,    // 19.0401234°
            alt_mm: 120_000,        // 120.000 m AMSL
            relative_alt_mm: 5_000, // 5 m above home
            vx_cms: 850,            // 8.5 m/s north
            vy_cms: -25,            // -0.25 m/s east (slight west drift)
            vz_cms: -10,            // -0.1 m/s down (climbing)
            hdg_cdeg: 18_000,       // 180.00°
        }
    }

    #[test]
    fn msg_id_is_33() {
        assert_eq!(GLOBAL_POSITION_INT_MSG_ID, 33);
    }

    #[test]
    fn payload_length_is_28() {
        assert_eq!(GLOBAL_POSITION_INT_PAYLOAD_LEN, 28);
    }

    #[test]
    fn crc_extra_is_104() {
        // Fixed by spec; if this fails, somebody edited the MAVLink XML constant.
        assert_eq!(GLOBAL_POSITION_INT_CRC_EXTRA, 104);
    }

    #[test]
    fn round_trip_sample() {
        let original = sample();
        let bytes = original.encode_payload();
        let decoded = GlobalPositionInt::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn round_trip_zero() {
        let original = GlobalPositionInt {
            time_boot_ms: 0,
            lat_e7: 0,
            lon_e7: 0,
            alt_mm: 0,
            relative_alt_mm: 0,
            vx_cms: 0,
            vy_cms: 0,
            vz_cms: 0,
            hdg_cdeg: 0,
        };
        let bytes = original.encode_payload();
        let decoded = GlobalPositionInt::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn round_trip_extremes() {
        let original = GlobalPositionInt {
            time_boot_ms: u32::MAX,
            lat_e7: i32::MIN,
            lon_e7: i32::MAX,
            alt_mm: i32::MIN,
            relative_alt_mm: i32::MAX,
            vx_cms: i16::MIN,
            vy_cms: i16::MAX,
            vz_cms: i16::MIN,
            hdg_cdeg: HEADING_UNKNOWN,
        };
        let bytes = original.encode_payload();
        let decoded = GlobalPositionInt::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(GlobalPositionInt::decode_payload(&[0u8; 27]).is_none());
    }
    #[test]
    fn decode_rejects_long_payload() {
        assert!(GlobalPositionInt::decode_payload(&[0u8; 29]).is_none());
    }
    #[test]
    fn decode_rejects_empty() {
        assert!(GlobalPositionInt::decode_payload(&[]).is_none());
    }

    #[test]
    fn field_offsets_match_spec() {
        // MAVLink-canonical sorted-by-size descending order: u32×5, i16×3, u16×1.
        // Spot-check the four 4-byte fields' first byte position.
        let m = GlobalPositionInt {
            time_boot_ms: 0xAAAA_AAAA,
            lat_e7: 0x11_22_33_44u32 as i32,
            lon_e7: 0x55_66_77_78u32 as i32, // top nibble < 0x80 → positive i32
            alt_mm: 0x12345678,
            relative_alt_mm: 0x21436587u32 as i32,
            vx_cms: 0,
            vy_cms: 0,
            vz_cms: 0,
            hdg_cdeg: 0,
        };
        let b = m.encode_payload();
        assert_eq!(&b[0..4], &[0xAA, 0xAA, 0xAA, 0xAA]);
        assert_eq!(&b[4..8], &0x11_22_33_44u32.to_le_bytes());
        assert_eq!(&b[8..12], &0x55_66_77_78u32.to_le_bytes());
        assert_eq!(&b[12..16], &0x12345678u32.to_le_bytes());
        assert_eq!(&b[16..20], &0x21436587u32.to_le_bytes());
    }

    use proptest::prelude::*;
    proptest! {
        #[test]
        fn round_trip_arbitrary(
            time_boot_ms in any::<u32>(),
            lat_e7 in any::<i32>(),
            lon_e7 in any::<i32>(),
            alt_mm in any::<i32>(),
            relative_alt_mm in any::<i32>(),
            vx_cms in any::<i16>(),
            vy_cms in any::<i16>(),
            vz_cms in any::<i16>(),
            hdg_cdeg in any::<u16>(),
        ) {
            let original = GlobalPositionInt {
                time_boot_ms, lat_e7, lon_e7, alt_mm, relative_alt_mm,
                vx_cms, vy_cms, vz_cms, hdg_cdeg,
            };
            let bytes = original.encode_payload();
            let decoded = GlobalPositionInt::decode_payload(&bytes)
                .expect("decode arbitrary");
            prop_assert_eq!(original, decoded);
        }
    }
}
