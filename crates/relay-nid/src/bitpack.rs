//! ASTM F3411-22 bit-packed wire format — verified Verus mirror of
//! `../../plain/src/bitpack.rs`. Source of truth for the formal
//! contracts; the plain mirror is what cargo compiles.
//!
//! Bodies are `#[verifier::external_body]` (bit-fiddling, slice
//! indexing — out of Verus's good range); the **signatures** are the
//! formal contracts (frame size, `Option` totality), and Kani's
//! NID-K03..K05 harnesses give exhaustive panic-freedom + round-trip
//! coverage on the bit-packed path.

use super::{
    BasicId, IdType, Location, MessageType, OperationalStatus, UaType,
    FRAME_BYTES, PROTOCOL_VERSION, UAS_ID_BYTES,
};
use vstd::prelude::*;

verus! {

#[verifier::external_body]
pub fn encode_basic_id_bitpacked(msg: &BasicId, buf: &mut [u8; FRAME_BYTES]) {
    buf[0] = ((MessageType::BasicId as u8) << 4) | (PROTOCOL_VERSION & 0x0F);
    buf[1] = ((msg.id_type as u8) << 4) | ((msg.ua_type as u8) & 0x0F);
    buf[2..2 + UAS_ID_BYTES].copy_from_slice(&msg.uas_id);
    buf[22] = 0;
    buf[23] = 0;
    buf[24] = 0;
}

#[verifier::external_body]
pub fn decode_basic_id_bitpacked(buf: &[u8; FRAME_BYTES]) -> Option<BasicId> {
    let (mtype, version) = (buf[0] >> 4, buf[0] & 0x0F);
    if version != PROTOCOL_VERSION || MessageType::from_code(mtype)? != MessageType::BasicId {
        return None;
    }
    let id_type_code = (buf[1] >> 4) & 0x0F;
    let ua_type_code = buf[1] & 0x0F;
    let id_type = IdType::from_code(id_type_code)?;
    let ua_type = UaType::from_code(ua_type_code)?;
    let mut uas_id = [0u8; UAS_ID_BYTES];
    uas_id.copy_from_slice(&buf[2..2 + UAS_ID_BYTES]);
    Some(BasicId { id_type, ua_type, uas_id })
}

#[verifier::external_body]
pub fn encode_location_bitpacked(msg: &Location, buf: &mut [u8; FRAME_BYTES]) {
    buf[0] = ((MessageType::Location as u8) << 4) | (PROTOCOL_VERSION & 0x0F);
    buf[1] = ((msg.status as u8) & 0x0F) << 4;
    let track_deg = (msg.track_centideg as u32 / 100).min(359);
    let (track_byte, ew) = if track_deg >= 180 {
        ((track_deg - 180) as u8, 1u8)
    } else {
        (track_deg as u8, 0u8)
    };
    buf[2] = track_byte;
    buf[3] = ew;
    let speed_ms = (msg.ground_speed_cms as u32 / 100).min(254);
    buf[4] = speed_ms as u8;
    let vs_half_ms = (msg.vertical_speed_cms as i32 / 50).clamp(-63, 63);
    buf[5] = (vs_half_ms as i8) as u8;
    buf[6..10].copy_from_slice(&msg.latitude_e7.to_le_bytes());
    buf[10..14].copy_from_slice(&msg.longitude_e7.to_le_bytes());
    let alt_m = (msg.altitude_cm / 100).clamp(-1000, 31_767);
    let alt_enc = ((alt_m + 1000) * 2) as u16;
    buf[14..16].copy_from_slice(&alt_enc.to_le_bytes());
    let ts_in_hour = (msg.timestamp_decisec % 36_000) as u16;
    buf[16..18].copy_from_slice(&ts_in_hour.to_le_bytes());
    for b in &mut buf[18..25] {
        *b = 0;
    }
}

#[verifier::external_body]
pub fn decode_location_bitpacked(buf: &[u8; FRAME_BYTES]) -> Option<Location> {
    let (mtype, version) = (buf[0] >> 4, buf[0] & 0x0F);
    if version != PROTOCOL_VERSION || MessageType::from_code(mtype)? != MessageType::Location {
        return None;
    }
    let status = OperationalStatus::from_code(buf[1] >> 4)?;
    let track_byte = buf[2];
    if track_byte > 180 {
        return None;
    }
    let ew = buf[3] & 0x01;
    let track_deg: u32 = if ew == 1 { (track_byte as u32) + 180 } else { track_byte as u32 };
    let track_centideg = (track_deg * 100) as u16;
    let ground_speed_cms = (buf[4] as u16) * 100;
    let vertical_speed_cms = ((buf[5] as i8) as i16) * 50;
    let latitude_e7 = i32::from_le_bytes(buf[6..10].try_into().ok()?);
    let longitude_e7 = i32::from_le_bytes(buf[10..14].try_into().ok()?);
    let alt_enc = u16::from_le_bytes(buf[14..16].try_into().ok()?);
    let alt_m = (alt_enc as i32) / 2 - 1000;
    let altitude_cm = alt_m * 100;
    let ts_in_hour = u16::from_le_bytes(buf[16..18].try_into().ok()?) as u32;
    if ts_in_hour >= 36_000 {
        return None;
    }
    let timestamp_decisec = ts_in_hour;
    Some(Location {
        status,
        latitude_e7,
        longitude_e7,
        altitude_cm,
        ground_speed_cms,
        vertical_speed_cms,
        track_centideg,
        timestamp_decisec,
    })
}

#[verifier::external_body]
pub fn canonicalize_location(msg: &Location) -> Location {
    Location {
        status: msg.status,
        latitude_e7: msg.latitude_e7,
        longitude_e7: msg.longitude_e7,
        altitude_cm: (msg.altitude_cm / 100).clamp(-1000, 31_767) * 100,
        ground_speed_cms: (msg.ground_speed_cms / 100).min(254) * 100,
        vertical_speed_cms: ((msg.vertical_speed_cms as i32 / 50).clamp(-63, 63) * 50) as i16,
        track_centideg: ((msg.track_centideg as u32 / 100).min(359) * 100) as u16,
        timestamp_decisec: msg.timestamp_decisec % 36_000,
    }
}

} // verus!
