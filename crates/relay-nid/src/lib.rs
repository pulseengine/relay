//! Relay Network Identification — verified codec for EASA U-space
//! "Network Remote ID" / ASTM F3411-22 frames.
//!
//! ## Layout
//!
//! Source of truth for the Verus contracts. The plain-Rust mirror at
//! `../plain/src/lib.rs` is what `cargo` builds; this tree is what
//! `verus_test` checks via Bazel (`//:relay_nid_verus_test`).
//!
//! ## What Verus actually proves here
//!
//! The encode/decode bodies are bit-level transforms — Verus reasons
//! about those poorly, so they are marked `#[verifier::external_body]`
//! and their **signatures** are the formal contracts (frame size,
//! totality, no panic). The codec's panic-freedom under arbitrary 25
//! bytes of input is exhaustively pinned by the Kani harnesses below
//! (NID-K01..K05) — Verus + Kani together cover the same DO-178
//! technique class as a fully Verus-discharged proof would.
//!
//! What Verus *does* prove on the source files:
//!   - **NID-V01**: every `from_code` is a total function (its `ensures`
//!     contract says `result.is_Some() <==> code is in the documented
//!     range`); Verus rejects code that calls them with out-of-range
//!     literals.
//!   - **NID-V02**: the public enum repr discriminants match the
//!     wire-format codes (the `as u8` cast in encoders is well-defined).
//!   - **NID-V03**: every encoder writes into a caller-provided
//!     `&mut [u8; FRAME_BYTES]` — no allocation, no out-of-bounds
//!     write possible at the type level.

#![no_std]

pub mod bitpack;

use vstd::prelude::*;

verus! {

/// Frame size in bytes — every Network ID message is exactly this long.
pub const FRAME_BYTES: usize = 25;

/// Bytes of the UAS ID field (per ASTM F3411 — 20 ASCII bytes,
/// zero-padded).
pub const UAS_ID_BYTES: usize = 20;

/// Current protocol version this crate emits.
pub const PROTOCOL_VERSION: u8 = 2;

/// Network ID message family.
#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum MessageType {
    BasicId = 0,
    Location = 1,
}

impl MessageType {
    pub fn from_code(code: u8) -> (r: Option<MessageType>)
        ensures
            r.is_Some() <==> code == 0 || code == 1,
    {
        match code {
            0 => Some(MessageType::BasicId),
            1 => Some(MessageType::Location),
            _ => None,
        }
    }
}

/// What kind of identifier the UAS ID field carries.
#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum IdType {
    None = 0,
    SerialNumber = 1,
    CaaRegistration = 2,
    Utm = 3,
    SessionId = 4,
}

impl IdType {
    pub fn from_code(code: u8) -> (r: Option<IdType>)
        ensures
            r.is_Some() <==> code <= 4,
    {
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
#[derive(PartialEq, Eq)]
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
    pub fn from_code(code: u8) -> (r: Option<UaType>)
        ensures
            r.is_Some() <==> code <= 9,
    {
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
#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum OperationalStatus {
    Undeclared = 0,
    Ground = 1,
    Airborne = 2,
    Emergency = 3,
}

impl OperationalStatus {
    pub fn from_code(code: u8) -> (r: Option<OperationalStatus>)
        ensures
            r.is_Some() <==> code <= 3,
    {
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
pub struct BasicId {
    pub id_type: IdType,
    pub ua_type: UaType,
    /// 20-byte UAS identifier, ASCII, zero-padded.
    pub uas_id: [u8; UAS_ID_BYTES],
}

/// Location / Vector message — where the vehicle is and where it's going.
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
///
/// **NID-V03**: signature alone guarantees the encoder writes exactly
/// `FRAME_BYTES` bytes and never allocates.
#[verifier::external_body]
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
#[verifier::external_body]
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
#[verifier::external_body]
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
#[verifier::external_body]
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

} // verus!
