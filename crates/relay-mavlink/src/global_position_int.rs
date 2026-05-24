//! GLOBAL_POSITION_INT (MAVLink id 33) — verified mirror of
//! `../plain/src/global_position_int.rs`.

use vstd::prelude::*;

verus! {

pub const GLOBAL_POSITION_INT_MSG_ID: u32 = 33;
pub const GLOBAL_POSITION_INT_PAYLOAD_LEN: usize = 28;
pub const GLOBAL_POSITION_INT_CRC_EXTRA: u8 = 104;
pub const HEADING_UNKNOWN: u16 = u16::MAX;

pub struct GlobalPositionInt {
    pub time_boot_ms: u32,
    pub lat_e7: i32,
    pub lon_e7: i32,
    pub alt_mm: i32,
    pub relative_alt_mm: i32,
    pub vx_cms: i16,
    pub vy_cms: i16,
    pub vz_cms: i16,
    pub hdg_cdeg: u16,
}

impl GlobalPositionInt {
    /// **MAVLINK-V02**: encoder writes exactly
    /// `GLOBAL_POSITION_INT_PAYLOAD_LEN` bytes.
    #[verifier::external_body]
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

    /// **MAVLINK-V03**: decoder returns `None` on length mismatch.
    #[verifier::external_body]
    pub fn decode_payload(payload: &[u8]) -> Option<GlobalPositionInt> {
        if payload.len() != GLOBAL_POSITION_INT_PAYLOAD_LEN {
            return None;
        }
        Some(GlobalPositionInt {
            time_boot_ms: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            lat_e7: i32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            lon_e7: i32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            alt_mm: i32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
            relative_alt_mm: i32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]),
            vx_cms: i16::from_le_bytes([payload[20], payload[21]]),
            vy_cms: i16::from_le_bytes([payload[22], payload[23]]),
            vz_cms: i16::from_le_bytes([payload[24], payload[25]]),
            hdg_cdeg: u16::from_le_bytes([payload[26], payload[27]]),
        })
    }
}

} // verus!
