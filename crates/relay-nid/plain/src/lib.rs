//! Relay Network Identification — EASA U-space "Network Remote ID"
//! codec for continuously broadcasting a UAS's identification +
//! position/velocity beacons over a network channel.
//!
//! v0.9 ships a focused, byte-stable codec that carries every field
//! a downstream EASA U-space service consumes (UAS ID, ID type, UA
//! type, position, velocity, timestamp, status) in a fixed 25-byte
//! frame. The frame layout is ASTM F3411-inspired but spec-simplified
//! — straight little-endian fields, no bit-packing — to keep the
//! Verus / Kani contracts that land in v0.9.x clean. Full
//! ASTM F3411-22 bit-packed compatibility is a follow-up; this layer
//! gives the SITL + downstream relay-to consumers a stable shape now.
//!
//! ## Message types
//!   - **Basic ID** (`MessageType::BasicId`, code `0`): UAS ID +
//!     ID-type + UA-type.
//!   - **Location / Vector** (`MessageType::Location`, code `1`):
//!     position (lat/lon/alt), velocity (track + ground speed +
//!     vertical speed), status, timestamp.
//!
//! ## Frame layout (25 bytes, little-endian)
//!
//! ```text
//!   byte 0    : header   = (message_type << 4) | (PROTOCOL_VERSION & 0x0F)
//!   bytes 1..24: payload (24 bytes, type-specific)
//! ```
//!
//! Both encoders write exactly 25 bytes. Decoders return `None` on
//! header or field validation failure — they never panic on
//! malformed input.

#![no_std]

/// Frame size in bytes — every Network ID message is exactly this long.
pub const FRAME_BYTES: usize = 25;

/// Bytes of the UAS ID field (per ASTM F3411 — 20 ASCII bytes,
/// zero-padded).
pub const UAS_ID_BYTES: usize = 20;

/// Current protocol version this crate emits.
pub const PROTOCOL_VERSION: u8 = 2;

/// Network ID message family.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MessageType {
    BasicId = 0,
    Location = 1,
}

impl MessageType {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(MessageType::BasicId),
            1 => Some(MessageType::Location),
            _ => None,
        }
    }
}

/// What kind of identifier the UAS ID field carries.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum IdType {
    None = 0,
    SerialNumber = 1,
    CaaRegistration = 2,
    Utm = 3,
    SessionId = 4,
}

impl IdType {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(IdType::None),
            1 => Some(IdType::SerialNumber),
            2 => Some(IdType::CaaRegistration),
            3 => Some(IdType::Utm),
            4 => Some(IdType::SessionId),
            _ => None,
        }
    }
}

/// UA category from the EASA / FAA enumerations (simplified).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum UaType {
    None = 0,
    Aeroplane = 1,
    RotorcraftMulti = 2,
    RotorcraftHelicopter = 3,
    Gyroplane = 4,
    HybridLift = 5,
    Ornithopter = 6,
    Glider = 7,
    Kite = 8,
    FreeBalloon = 9,
}

impl UaType {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(UaType::None),
            1 => Some(UaType::Aeroplane),
            2 => Some(UaType::RotorcraftMulti),
            3 => Some(UaType::RotorcraftHelicopter),
            4 => Some(UaType::Gyroplane),
            5 => Some(UaType::HybridLift),
            6 => Some(UaType::Ornithopter),
            7 => Some(UaType::Glider),
            8 => Some(UaType::Kite),
            9 => Some(UaType::FreeBalloon),
            _ => None,
        }
    }
}

/// Operational status of the vehicle.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum OperationalStatus {
    Undeclared = 0,
    Ground = 1,
    Airborne = 2,
    Emergency = 3,
}

impl OperationalStatus {
    fn from_code(code: u8) -> Option<Self> {
        match code {
            0 => Some(OperationalStatus::Undeclared),
            1 => Some(OperationalStatus::Ground),
            2 => Some(OperationalStatus::Airborne),
            3 => Some(OperationalStatus::Emergency),
            _ => None,
        }
    }
}

/// Basic ID message — UAS ID + ID type + UA type.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct BasicId {
    pub id_type: IdType,
    pub ua_type: UaType,
    /// 20-byte UAS identifier, ASCII, zero-padded.
    pub uas_id: [u8; UAS_ID_BYTES],
}

/// Location / Vector message — where the vehicle is and where it's going.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Location {
    pub status: OperationalStatus,
    /// Latitude in units of 1e-7 degrees.
    pub latitude_e7: i32,
    /// Longitude in units of 1e-7 degrees.
    pub longitude_e7: i32,
    /// Geodetic altitude in centimetres above WGS-84.
    pub altitude_cm: i32,
    /// Ground speed in cm/s.
    pub ground_speed_cms: u16,
    /// Vertical speed in cm/s (signed; up positive).
    pub vertical_speed_cms: i16,
    /// Track (course over ground) in centidegrees (0..36000).
    pub track_centideg: u16,
    /// Timestamp in tenths of seconds since UTC midnight (0..863999).
    pub timestamp_decisec: u32,
}

/// Encode a Basic ID message into the 25-byte frame.
pub fn encode_basic_id(msg: &BasicId, buf: &mut [u8; FRAME_BYTES]) {
    buf[0] = ((MessageType::BasicId as u8) << 4) | (PROTOCOL_VERSION & 0x0F);
    buf[1] = msg.id_type as u8;
    buf[2] = msg.ua_type as u8;
    buf[3..3 + UAS_ID_BYTES].copy_from_slice(&msg.uas_id);
    buf[23] = 0;
    buf[24] = 0;
}

/// Decode a Basic ID frame. Returns `None` if header or enum codes
/// are invalid.
pub fn decode_basic_id(buf: &[u8; FRAME_BYTES]) -> Option<BasicId> {
    let (msg_type_code, version) = (buf[0] >> 4, buf[0] & 0x0F);
    if version != PROTOCOL_VERSION || MessageType::from_code(msg_type_code)? != MessageType::BasicId
    {
        return None;
    }
    let id_type = IdType::from_code(buf[1])?;
    let ua_type = UaType::from_code(buf[2])?;
    let mut uas_id = [0u8; UAS_ID_BYTES];
    uas_id.copy_from_slice(&buf[3..3 + UAS_ID_BYTES]);
    Some(BasicId { id_type, ua_type, uas_id })
}

/// Encode a Location / Vector message into the 25-byte frame.
pub fn encode_location(msg: &Location, buf: &mut [u8; FRAME_BYTES]) {
    buf[0] = ((MessageType::Location as u8) << 4) | (PROTOCOL_VERSION & 0x0F);
    buf[1] = msg.status as u8;
    buf[2..6].copy_from_slice(&msg.latitude_e7.to_le_bytes());
    buf[6..10].copy_from_slice(&msg.longitude_e7.to_le_bytes());
    buf[10..14].copy_from_slice(&msg.altitude_cm.to_le_bytes());
    buf[14..16].copy_from_slice(&msg.ground_speed_cms.to_le_bytes());
    buf[16..18].copy_from_slice(&msg.vertical_speed_cms.to_le_bytes());
    buf[18..20].copy_from_slice(&msg.track_centideg.to_le_bytes());
    buf[20..24].copy_from_slice(&msg.timestamp_decisec.to_le_bytes());
    buf[24] = 0;
}

/// Decode a Location / Vector frame. Returns `None` on header
/// mismatch, an invalid status code, an out-of-range track
/// (≥ 360°), or an out-of-range timestamp (≥ 86 400 s).
pub fn decode_location(buf: &[u8; FRAME_BYTES]) -> Option<Location> {
    let (msg_type_code, version) = (buf[0] >> 4, buf[0] & 0x0F);
    if version != PROTOCOL_VERSION || MessageType::from_code(msg_type_code)? != MessageType::Location
    {
        return None;
    }
    let status = OperationalStatus::from_code(buf[1])?;
    let latitude_e7 = i32::from_le_bytes(buf[2..6].try_into().ok()?);
    let longitude_e7 = i32::from_le_bytes(buf[6..10].try_into().ok()?);
    let altitude_cm = i32::from_le_bytes(buf[10..14].try_into().ok()?);
    let ground_speed_cms = u16::from_le_bytes(buf[14..16].try_into().ok()?);
    let vertical_speed_cms = i16::from_le_bytes(buf[16..18].try_into().ok()?);
    let track_centideg = u16::from_le_bytes(buf[18..20].try_into().ok()?);
    let timestamp_decisec = u32::from_le_bytes(buf[20..24].try_into().ok()?);
    if track_centideg >= 36_000 || timestamp_decisec >= 864_000 {
        return None;
    }
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
        Location {
            status: OperationalStatus::Airborne,
            latitude_e7: 47_502_345_6,    // 47.5023456°
            longitude_e7: 19_040_123_4,    // 19.0401234°
            altitude_cm: 12_000,           // 120.00 m
            ground_speed_cms: 850,         // 8.5 m/s
            vertical_speed_cms: -25,       // -0.25 m/s
            track_centideg: 18_000,        // 180.00° (due south)
            timestamp_decisec: 432_000,    // 12:00:00.0 UTC
        }
    }

    #[test]
    fn basic_id_round_trip() {
        let msg = sample_basic();
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id(&msg, &mut buf);
        let decoded = decode_basic_id(&buf).expect("decode");
        assert_eq!(decoded, msg);
    }

    #[test]
    fn basic_id_header_byte_is_well_formed() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id(&sample_basic(), &mut buf);
        assert_eq!(buf[0] >> 4, MessageType::BasicId as u8);
        assert_eq!(buf[0] & 0x0F, PROTOCOL_VERSION);
    }

    #[test]
    fn location_round_trip() {
        let msg = sample_location();
        let mut buf = [0u8; FRAME_BYTES];
        encode_location(&msg, &mut buf);
        let decoded = decode_location(&buf).expect("decode");
        assert_eq!(decoded, msg);
    }

    #[test]
    fn location_rejects_wrong_message_type() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id(&sample_basic(), &mut buf);
        assert!(decode_location(&buf).is_none());
    }

    #[test]
    fn location_rejects_out_of_range_track() {
        let mut msg = sample_location();
        msg.track_centideg = 36_000;
        let mut buf = [0u8; FRAME_BYTES];
        encode_location(&msg, &mut buf);
        assert!(decode_location(&buf).is_none());
    }

    #[test]
    fn location_rejects_out_of_range_timestamp() {
        let mut msg = sample_location();
        msg.timestamp_decisec = 864_000;
        let mut buf = [0u8; FRAME_BYTES];
        encode_location(&msg, &mut buf);
        assert!(decode_location(&buf).is_none());
    }

    #[test]
    fn decoder_rejects_unknown_protocol_version() {
        let mut buf = [0u8; FRAME_BYTES];
        encode_basic_id(&sample_basic(), &mut buf);
        buf[0] = (buf[0] & 0xF0) | 0x0E; // unsupported version
        assert!(decode_basic_id(&buf).is_none());
    }

    proptest! {
        #[test]
        fn proptest_basic_id_round_trip(
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
            encode_basic_id(&msg, &mut buf);
            let decoded = decode_basic_id(&buf).expect("decode");
            prop_assert_eq!(decoded, msg);
        }

        #[test]
        fn proptest_location_round_trip_in_range(
            status_code in 0u8..=3,
            lat in -900_000_000i32..=900_000_000,
            lon in -1_800_000_000i32..=1_800_000_000,
            alt in -10_000i32..=1_000_000,
            ground in 0u16..=20_000,
            vert in -10_000i16..=10_000,
            track in 0u16..36_000,
            ts in 0u32..864_000,
        ) {
            let msg = Location {
                status: OperationalStatus::from_code(status_code).unwrap(),
                latitude_e7: lat,
                longitude_e7: lon,
                altitude_cm: alt,
                ground_speed_cms: ground,
                vertical_speed_cms: vert,
                track_centideg: track,
                timestamp_decisec: ts,
            };
            let mut buf = [0u8; FRAME_BYTES];
            encode_location(&msg, &mut buf);
            let decoded = decode_location(&buf).expect("decode");
            prop_assert_eq!(decoded, msg);
        }

        /// Decoder never panics on arbitrary input.
        #[test]
        fn proptest_decoder_never_panics(bytes in proptest::collection::vec(0u8..=255u8, FRAME_BYTES)) {
            let mut buf = [0u8; FRAME_BYTES];
            buf.copy_from_slice(&bytes);
            let _ = decode_basic_id(&buf);
            let _ = decode_location(&buf);
        }
    }
}
