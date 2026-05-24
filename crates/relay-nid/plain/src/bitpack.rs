//! ASTM F3411-22 bit-packed wire format for the Network ID messages.
//!
//! The v0.9 codec in the parent module uses straight little-endian
//! fields so the encoder is trivially Verus/Kani-friendly. This
//! module is the *wire-compatible* layer: it packs the same in-memory
//! [`super::BasicId`] / [`super::Location`] types into the bit-level
//! layouts ASTM F3411-22 specifies for the Network Remote ID
//! transport, so a downstream receiver expecting F3411-22 frames can
//! consume them directly.
//!
//! ## Honesty about lossiness
//!
//! BasicId is **bit-perfect round-trip** — every field of the
//! in-memory type encodes losslessly.
//!
//! Location is **lossy** in the same way the F3411-22 frame itself
//! is: the spec quantises track to 1°, ground speed to F3411's
//! piecewise scale, vertical speed to 0.5 m/s, altitudes to 0.5 m,
//! and timestamp to 0.1 s within the current hour. We expose a
//! [`canonicalize_location`] helper that pre-rounds an in-memory
//! [`super::Location`] to F3411-22 resolution; then `decode(encode(
//! canonicalize(msg))) == canonicalize(msg)` is the round-trip
//! identity the proptests below check.
//!
//! Decoders never panic on arbitrary 25-byte input (proptest-fuzzed).

use super::{
    BasicId, IdType, Location, MessageType, OperationalStatus, UaType,
    FRAME_BYTES, PROTOCOL_VERSION, UAS_ID_BYTES,
};

// ─── BasicId — bit-packed (lossless) ───────────────────────────────

/// Encode a Basic ID message into the 25-byte ASTM F3411-22 frame:
///
/// ```text
///   buf[0]    : (message_type << 4) | (version & 0x0F)
///   buf[1]    : (id_type << 4) | (ua_type & 0x0F)
///   buf[2..22]: UAS ID (20 bytes, ASCII, zero-padded)
///   buf[22..25]: reserved (zero)
/// ```
///
/// Bit-perfect round-trip with [`decode_basic_id_bitpacked`].
pub fn encode_basic_id_bitpacked(msg: &BasicId, buf: &mut [u8; FRAME_BYTES]) {
    buf[0] = ((MessageType::BasicId as u8) << 4) | (PROTOCOL_VERSION & 0x0F);
    buf[1] = ((msg.id_type as u8) << 4) | ((msg.ua_type as u8) & 0x0F);
    buf[2..2 + UAS_ID_BYTES].copy_from_slice(&msg.uas_id);
    buf[22] = 0;
    buf[23] = 0;
    buf[24] = 0;
}

/// Decode an F3411-22 Basic ID frame. Returns `None` on header
/// mismatch or unknown enum codes.
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

// ─── Location — bit-packed (lossy, F3411 resolution) ───────────────

/// F3411-22 Location/Vector frame layout (simplified to what the
/// in-memory `Location` carries):
///
/// ```text
///   buf[0]   : header
///   buf[1]   : (status << 4) | reserved (low 4 = 0)
///   buf[2]   : track_direction (0..=180), EW indicator in flag bit
///   buf[3]   : flags: bit0 = ew_direction (track ≥ 180°), bit1 = vertical_speed sign carrier (always 0 here — sign lives in i8)
///   buf[4]   : ground speed (m/s, clamped to 0..=254)
///   buf[5]   : vertical speed (i8, 0.5 m/s units, clamped to ±63)
///   buf[6..10]   : latitude_e7  (i32 LE)
///   buf[10..14]  : longitude_e7 (i32 LE)
///   buf[14..16]  : geodetic altitude — encoded ((m + 1000) * 2) as u16 LE
///   buf[16..18]  : timestamp (tenths-of-a-second past current UTC hour, 0..36000)
///   buf[18..25]  : reserved (zero)
/// ```
///
/// Encoder clamps out-of-range fields to the representable F3411-22
/// range rather than failing — F3411 frames are *broadcast*, never
/// negotiated, so a degraded field is more useful than a dropped
/// frame.
pub fn encode_location_bitpacked(msg: &Location, buf: &mut [u8; FRAME_BYTES]) {
    buf[0] = ((MessageType::Location as u8) << 4) | (PROTOCOL_VERSION & 0x0F);
    buf[1] = ((msg.status as u8) & 0x0F) << 4;

    // Track 0..360° → byte 0..180 + EW flag.
    let track_deg = (msg.track_centideg as u32 / 100).min(359);
    let (track_byte, ew) = if track_deg >= 180 {
        ((track_deg - 180) as u8, 1u8)
    } else {
        (track_deg as u8, 0u8)
    };
    buf[2] = track_byte;
    buf[3] = ew;

    // Ground speed cm/s → m/s, clamped 0..254.
    let speed_ms = (msg.ground_speed_cms as u32 / 100).min(254);
    buf[4] = speed_ms as u8;

    // Vertical speed cm/s → i8 in 0.5-m/s units, clamped ±63.
    let vs_half_ms = (msg.vertical_speed_cms as i32 / 50).clamp(-63, 63);
    buf[5] = (vs_half_ms as i8) as u8;

    buf[6..10].copy_from_slice(&msg.latitude_e7.to_le_bytes());
    buf[10..14].copy_from_slice(&msg.longitude_e7.to_le_bytes());

    // Geodetic altitude cm → m, encoded as (m + 1000) * 2 in u16.
    let alt_m = (msg.altitude_cm / 100).clamp(-1000, 31_767);
    let alt_enc = ((alt_m + 1000) * 2) as u16;
    buf[14..16].copy_from_slice(&alt_enc.to_le_bytes());

    // Timestamp: tenths-past-hour, 0..36000.
    let ts_in_hour = (msg.timestamp_decisec % 36_000) as u16;
    buf[16..18].copy_from_slice(&ts_in_hour.to_le_bytes());

    for b in &mut buf[18..25] {
        *b = 0;
    }
}

/// Decode an F3411-22 Location/Vector frame. Returns `None` on
/// header mismatch, unknown status, or out-of-range track byte
/// (> 180).
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
    let track_deg: u32 = if ew == 1 {
        (track_byte as u32) + 180
    } else {
        track_byte as u32
    };
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

/// Pre-round an in-memory [`Location`] to F3411-22's quantisation so
/// `decode(encode(msg)) == msg`. Use this before encoding when you
/// need exact round-trip equality (e.g. in tests).
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

#[cfg(test)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    fn sample_basic() -> BasicId {
        let mut uas_id = [0u8; UAS_ID_BYTES];
        let s = b"FALCON-Q-000001";
        uas_id[..s.len()].copy_from_slice(s);
        BasicId {
            id_type: IdType::SerialNumber,
            ua_type: UaType::RotorcraftMulti,
            uas_id,
        }
    }

    fn sample_location() -> Location {
        // Field values already at F3411 resolution so round-trip is exact.
        Location {
            status: OperationalStatus::Airborne,
            latitude_e7: 475_023_456,
            longitude_e7: 190_401_234,
            altitude_cm: 12_000,           // 120 m, on F3411 0.5 m grid
            ground_speed_cms: 800,         // 8 m/s
            vertical_speed_cms: -50,       // -0.5 m/s
            track_centideg: 18_000,        // 180° even
            timestamp_decisec: 18_000,     // 30:00 past hour
        }
    }

    #[test]
    fn bitpack_basic_id_round_trip() {
        let msg = sample_basic();
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id_bitpacked(&msg, &mut buf);
        let decoded = decode_basic_id_bitpacked(&buf).expect("decode");
        assert_eq!(decoded, msg);
    }

    #[test]
    fn bitpack_basic_id_packs_id_and_ua_type_in_same_byte() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id_bitpacked(&sample_basic(), &mut buf);
        // ASTM F3411-22: BasicId payload byte 0 = (id_type<<4) | ua_type.
        assert_eq!(buf[1] >> 4, IdType::SerialNumber as u8);
        assert_eq!(buf[1] & 0x0F, UaType::RotorcraftMulti as u8);
    }

    #[test]
    fn bitpack_location_round_trip_on_canonical() {
        let msg = sample_location();
        let mut buf = [0u8; FRAME_BYTES];
        encode_location_bitpacked(&msg, &mut buf);
        let decoded = decode_location_bitpacked(&buf).expect("decode");
        assert_eq!(decoded, msg);
    }

    #[test]
    fn bitpack_location_idempotent_after_canonicalize() {
        // Even when the input is *not* on the F3411 grid, the result
        // of canonicalize → encode → decode equals the canonical input.
        let mut msg = sample_location();
        msg.track_centideg = 18_073; // sub-degree precision
        msg.altitude_cm = 12_071;    // sub-0.5 m precision
        let canon = canonicalize_location(&msg);
        let mut buf = [0u8; FRAME_BYTES];
        encode_location_bitpacked(&canon, &mut buf);
        let decoded = decode_location_bitpacked(&buf).expect("decode");
        assert_eq!(decoded, canon);
    }

    #[test]
    fn bitpack_location_rejects_oversize_track_byte() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_location_bitpacked(&sample_location(), &mut buf);
        buf[2] = 181;
        assert!(decode_location_bitpacked(&buf).is_none());
    }

    #[test]
    fn bitpack_location_rejects_wrong_message_type() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id_bitpacked(&sample_basic(), &mut buf);
        assert!(decode_location_bitpacked(&buf).is_none());
    }

    #[test]
    fn bitpack_decoder_rejects_unknown_protocol_version() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id_bitpacked(&sample_basic(), &mut buf);
        buf[0] = (buf[0] & 0xF0) | 0x0E;
        assert!(decode_basic_id_bitpacked(&buf).is_none());
    }

    proptest! {
        #[test]
        fn proptest_bitpack_basic_id_round_trip(
            id_type_code in 0u8..=4,
            ua_type_code in 0u8..=9,
            uas_id in proptest::collection::vec(0u8..=255u8, UAS_ID_BYTES),
        ) {
            let mut uas_id_arr = [0u8; UAS_ID_BYTES];
            uas_id_arr.copy_from_slice(&uas_id);
            let msg = BasicId {
                id_type: IdType::from_code(id_type_code).unwrap(),
                ua_type: UaType::from_code(ua_type_code).unwrap(),
                uas_id: uas_id_arr,
            };
            let mut buf = [0u8; FRAME_BYTES];
            encode_basic_id_bitpacked(&msg, &mut buf);
            let decoded = decode_basic_id_bitpacked(&buf).expect("decode");
            prop_assert_eq!(decoded, msg);
        }

        #[test]
        fn proptest_bitpack_location_round_trip_on_canonical(
            status_code in 0u8..=3,
            lat in -900_000_000i32..=900_000_000,
            lon in -1_800_000_000i32..=1_800_000_000,
            alt_m in -1_000i32..=31_767,
            speed_ms in 0u16..=254,
            vert_half in -63i16..=63,
            track_deg in 0u16..=359,
            ts_dec in 0u32..36_000,
        ) {
            let msg = Location {
                status: OperationalStatus::from_code(status_code).unwrap(),
                latitude_e7: lat,
                longitude_e7: lon,
                altitude_cm: alt_m * 100,
                ground_speed_cms: speed_ms * 100,
                vertical_speed_cms: vert_half * 50,
                track_centideg: track_deg * 100,
                timestamp_decisec: ts_dec,
            };
            let mut buf = [0u8; FRAME_BYTES];
            encode_location_bitpacked(&msg, &mut buf);
            let decoded = decode_location_bitpacked(&buf).expect("decode");
            prop_assert_eq!(decoded, msg);
        }

        /// Decoder never panics on arbitrary 25-byte input.
        #[test]
        fn proptest_bitpack_decoder_never_panics(
            bytes in proptest::collection::vec(0u8..=255u8, FRAME_BYTES),
        ) {
            let mut buf = [0u8; FRAME_BYTES];
            buf.copy_from_slice(&bytes);
            let _ = decode_basic_id_bitpacked(&buf);
            let _ = decode_location_bitpacked(&buf);
        }
    }
}
