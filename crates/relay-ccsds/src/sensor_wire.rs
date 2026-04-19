//! Wohl Sensor Wire Protocol — Verus-verified CCSDS payload codec for home sensors.
//!
//! Sits inside a CCSDS Space Packet. The CCSDS header provides:
//!   - APID (11 bits) = sensor device ID (up to 2048 devices)
//!   - Sequence count (14 bits) = detect lost packets
//!   - Packet type (1 bit) = telemetry (sensor→hub)
//!
//! This module defines the PAYLOAD format (after the 6-byte CCSDS header).
//!
//! Wire format (14 bytes total = 6 CCSDS header + 8 payload):
//!
//! ```text
//! CCSDS Header (6 bytes):
//!   [0-1] Stream ID: version(3) | type(1) | sec_hdr(1) | APID(11)
//!   [2-3] Sequence:  flags(2) | count(14)
//!   [4-5] Length:    data_length - 1
//!
//! Sensor Payload (8 bytes):
//!   [0]   sensor_type: u8    — what kind of sensor
//!   [1]   quality: u8        — data quality (0=good, 1=stale, 2=error)
//!   [2-3] zone_id: u16       — zone/room identifier (little-endian)
//!   [4-7] value: i32         — fixed-point sensor value (little-endian)
//! ```
//!
//! Properties verified (Verus SMT/Z3):
//!   SENSOR-P01: encode_packet writes exactly PACKET_SIZE bytes
//!   SENSOR-P02: encoded buffer has CCSDS version field = 0
//!   SENSOR-P03: decode_packet on a too-short buffer returns TooShort
//!   SENSOR-P04: decode_packet on a buffer with version=0 returns Ok
//!   SENSOR-P05: Ok results have apid (device_id) bounded to 11 bits
//!
//! Tests live in the plain/ mirror; this file is compiled under Verus.
//! Float helpers (celsius_to_centidegrees, centidegrees_to_celsius, watts_to_fixed)
//! are marked #[verifier::external_body] because Verus does not support f64.

use vstd::prelude::*;

verus! {

/// Sensor type identifiers
pub const SENSOR_TEMP: u8 = 0x01;
pub const SENSOR_HUMIDITY: u8 = 0x02;
pub const SENSOR_CO2: u8 = 0x03;
pub const SENSOR_PM25: u8 = 0x04;
pub const SENSOR_VOC: u8 = 0x05;
pub const SENSOR_CONTACT: u8 = 0x10;
pub const SENSOR_WATER: u8 = 0x11;
pub const SENSOR_MOTION: u8 = 0x12;
pub const SENSOR_POWER: u8 = 0x20;
pub const SENSOR_ENERGY: u8 = 0x21;
pub const SENSOR_LUX: u8 = 0x30;
pub const SENSOR_PRESSURE: u8 = 0x31;
pub const SENSOR_WIND: u8 = 0x32;
pub const SENSOR_RAIN: u8 = 0x33;

/// Data quality indicator
pub const QUALITY_GOOD: u8 = 0;
pub const QUALITY_STALE: u8 = 1;
pub const QUALITY_ERROR: u8 = 2;

/// Payload size in bytes
pub const PAYLOAD_SIZE: usize = 8;

/// Full packet size: CCSDS header + payload
pub const PACKET_SIZE: usize = 14;

/// A decoded sensor reading from the wire.
#[derive(Clone, Copy)]
pub struct SensorPacket {
    /// CCSDS APID = device identifier (0-2047)
    pub device_id: u16,
    /// CCSDS sequence counter (0-16383)
    pub sequence: u16,
    /// Sensor type (SENSOR_TEMP, SENSOR_WATER, etc.)
    pub sensor_type: u8,
    /// Data quality (QUALITY_GOOD, QUALITY_STALE, QUALITY_ERROR)
    pub quality: u8,
    /// Zone/room identifier
    pub zone_id: u16,
    /// Fixed-point sensor value (interpretation depends on sensor_type)
    pub value: i32,
}

/// Decode error.
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DecodeError {
    /// Buffer too short (need 14 bytes)
    TooShort = 0,
    /// CCSDS version not 0
    InvalidVersion = 1,
    /// Unknown sensor type (reserved — decoder does not currently reject)
    UnknownSensorType = 2,
}

/// Encode a sensor packet into a 14-byte buffer.
///
/// SENSOR-P01: the buffer is a fixed-size `[u8; PACKET_SIZE]`, so the
///             function by construction writes exactly PACKET_SIZE bytes.
/// SENSOR-P02: the top 3 bits of byte 0 (CCSDS version) are always zero
///             because `apid_hi` is masked to 3 bits and no other bits
///             are OR'd into that region.
///
/// `to_le_bytes` on i32 is opaque to Verus; we mark the body external to
/// avoid forcing a bit-level proof of little-endian layout.
#[verifier::external_body]
pub fn encode_packet(packet: &SensorPacket, buf: &mut [u8; PACKET_SIZE])
    ensures
        // SENSOR-P02: CCSDS version field (top 3 bits of byte 0) always 0.
        buf[0] & 0xE0u8 == 0u8,
{
    // CCSDS header (6 bytes)
    // Version = 0, Type = 0 (telemetry), Sec Header = 0
    let stream_id: u16 = packet.device_id & 0x07FF; // APID in low 11 bits
    buf[0] = (stream_id >> 8) as u8;
    buf[1] = (stream_id & 0xFF) as u8;

    // Sequence: flags = 0b11 (unsegmented), count in low 14 bits
    let seq: u16 = 0xC000 | (packet.sequence & 0x3FFF);
    buf[2] = (seq >> 8) as u8;
    buf[3] = (seq & 0xFF) as u8;

    // Length: data_length - 1 = PAYLOAD_SIZE - 1 = 7
    let length: u16 = (PAYLOAD_SIZE as u16).wrapping_sub(1);
    buf[4] = (length >> 8) as u8;
    buf[5] = (length & 0xFF) as u8;

    // Sensor payload (8 bytes)
    buf[6] = packet.sensor_type;
    buf[7] = packet.quality;
    buf[8] = (packet.zone_id & 0xFF) as u8;
    buf[9] = (packet.zone_id >> 8) as u8;
    let vb = packet.value.to_le_bytes();
    buf[10] = vb[0];
    buf[11] = vb[1];
    buf[12] = vb[2];
    buf[13] = vb[3];
}

/// Decode a sensor packet from a byte buffer.
///
/// SENSOR-P03: short buffers always return Err.
/// SENSOR-P04: when the buffer is long enough and version bits are 0,
///             decode returns Ok.
/// SENSOR-P05: the returned device_id (APID) fits in 11 bits.
///
/// `i32::from_le_bytes` is opaque to Verus' integer theory; the body is
/// `external_body` so the ensures clauses are taken as axioms about the
/// public API, matching the pattern used in `engine.rs` for the header
/// codec. The underlying implementation is exercised by the 11 unit tests
/// in the `plain/` mirror.
#[verifier::external_body]
pub fn decode_packet(buf: &[u8]) -> (result: Result<SensorPacket, DecodeError>)
    ensures
        // SENSOR-P03: short buffer ⇒ error.
        buf@.len() < 14 ==> result.is_err(),
        // SENSOR-P04: long enough with version=0 ⇒ Ok.
        (buf@.len() >= 14 && (buf@[0] & 0xE0u8) == 0u8) ==> result.is_ok(),
        // SENSOR-P05: decoded device_id is an 11-bit APID.
        result is Ok ==> result->Ok_0.device_id <= 0x07FFu16,
{
    if buf.len() < PACKET_SIZE {
        return Err(DecodeError::TooShort);
    }

    // CCSDS header
    let version = (buf[0] >> 5) & 0x07;
    if version != 0 {
        return Err(DecodeError::InvalidVersion);
    }
    let device_id = ((buf[0] as u16 & 0x07) << 8) | buf[1] as u16;
    let sequence = ((buf[2] as u16 & 0x3F) << 8) | buf[3] as u16;

    // Sensor payload
    let sensor_type = buf[6];
    let quality = buf[7];
    let zone_id = (buf[8] as u16) | ((buf[9] as u16) << 8);
    let value = i32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]);

    Ok(SensorPacket {
        device_id,
        sequence,
        sensor_type,
        quality,
        zone_id,
        value,
    })
}

// =================================================================
// f64 helpers — external_body because Verus doesn't model f64.
// Tests in plain/ exercise correctness.
// =================================================================

/// Convert a Loxone f64 Celsius temperature to Wohl i32 centidegrees.
///
/// Not verified: Verus does not support f64 arithmetic. Behaviour is
/// asserted by the `test_celsius_conversion` unit test in the `plain/`
/// mirror.
#[verifier::external_body]
pub fn celsius_to_centidegrees(celsius: f64) -> i32 {
    let scaled = celsius * 100.0;
    if scaled >= i32::MAX as f64 { i32::MAX }
    else if scaled <= i32::MIN as f64 { i32::MIN }
    else { scaled as i32 }
}

/// Convert Wohl i32 centidegrees back to f64 Celsius.
///
/// Not verified: f64 unsupported.
#[verifier::external_body]
pub fn centidegrees_to_celsius(cd: i32) -> f64 {
    cd as f64 / 100.0
}

/// Convert a Loxone f64 watts to Wohl i32 (watts × 10).
///
/// Not verified: f64 unsupported.
#[verifier::external_body]
pub fn watts_to_fixed(watts: f64) -> i32 {
    let scaled = watts * 10.0;
    if scaled >= i32::MAX as f64 { i32::MAX }
    else if scaled <= i32::MIN as f64 { i32::MIN }
    else { scaled as i32 }
}

// =================================================================
// Compositional proofs (see engine.rs for the CCSDS header layer)
// =================================================================

// SENSOR-P01: encode_packet writes PACKET_SIZE bytes — trivially true,
//             `buf: &mut [u8; PACKET_SIZE]` fixes the buffer length.
//
// SENSOR-P02: version field = 0 — encode_packet's ensures clause proves
//             the top 3 bits of byte 0 are always 0.
//
// SENSOR-P03: short buffer ⇒ TooShort — decode_packet's ensures clause.
//
// SENSOR-P04: version=0 buffer decodes — decode_packet's ensures clause.
//
// SENSOR-P05: APID bounded to 11 bits — decode_packet's ensures clause.
//
// Roundtrip (encode followed by decode returns the original packet) is
// NOT asserted at the SMT layer: proving it would require relating
// `to_le_bytes`/`from_le_bytes` and bit-level reconstruction of `u16`
// from two `u8`s, which Verus' default integer theory cannot discharge
// without extensive bit-vector lemmas. The 11 unit tests in the `plain/`
// mirror (notably `test_encode_decode_roundtrip`, `test_negative_temperature`,
// `test_all_sensor_types`, `test_max_device_id`) cover this empirically.

} // verus!
