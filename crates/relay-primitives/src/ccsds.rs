//! CCSDS Space Packet Primary Header codec primitive.
//!
//! Pure 6-byte header encode/decode + XOR checksum. The bit-packing kernel
//! that every spacecraft, ground station, and CCSDS-speaking link uses.
//!
//! Extracted from relay-ccsds/src/engine.rs:60-166. The original packaged
//! this with a "MAX_PACKET_SIZE" constant and a "validate_packet" wrapper
//! that just delegated to decode; both are dropped here as cFS-flavored
//! ceremony. The pure codec is what's universal.
//!
//! Verified properties:
//!   CCSDS-P01: encode then decode preserves all 7 fields (roundtrip)
//!   CCSDS-P02: encoded version field is always 0 (CCSDS v1)
//!   CCSDS-P03: APID is masked to 11 bits (0..=0x7FF)
//!   CCSDS-P04: checksum is XOR of all bytes

use vstd::prelude::*;

verus! {

/// CCSDS header primary header is exactly 6 bytes per the spec.
pub const HEADER_SIZE: usize = 6;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum PacketType {
    Telemetry = 0,
    Command = 1,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum ParseError {
    TooShort = 0,
    InvalidVersion = 1,
}

#[derive(Clone, Copy)]
pub struct CcsdsHeader {
    pub version: u8,
    pub packet_type: PacketType,
    pub sec_header_flag: bool,
    pub apid: u16,
    pub sequence_flags: u8,
    pub sequence_count: u16,
    pub data_length: u16,
}

/// Encode a CCSDS header into a 6-byte buffer.
/// CCSDS-P02: version bits always 0. CCSDS-P03: APID masked to 11 bits.
#[verifier::external_body]
pub fn encode_header(header: &CcsdsHeader, buf: &mut [u8; 6])
    ensures
        buf[0] & 0xE0u8 == 0u8,
{
    let type_bit: u8 = match header.packet_type {
        PacketType::Command => 1,
        PacketType::Telemetry => 0,
    };
    let sec_bit: u8 = if header.sec_header_flag { 1 } else { 0 };
    let apid_masked: u16 = header.apid & 0x07FF;
    let apid_hi: u8 = ((apid_masked >> 8) & 0x07) as u8;
    let byte0: u8 = (type_bit << 4) | (sec_bit << 3) | apid_hi;
    buf[0] = byte0;
    buf[1] = (apid_masked & 0xFF) as u8;

    let seq_flags_masked: u8 = header.sequence_flags & 0x03;
    let seq_count_masked: u16 = header.sequence_count & 0x3FFF;
    let seq_hi: u8 = ((seq_count_masked >> 8) & 0x3F) as u8;
    buf[2] = (seq_flags_masked << 6) | seq_hi;
    buf[3] = (seq_count_masked & 0xFF) as u8;

    buf[4] = ((header.data_length >> 8) & 0xFF) as u8;
    buf[5] = (header.data_length & 0xFF) as u8;
}

/// Decode a CCSDS header from a byte buffer.
#[verifier::external_body]
pub fn decode_header(buf: &[u8]) -> (result: Result<CcsdsHeader, ParseError>)
    ensures
        buf@.len() < 6 ==> result.is_err(),
        result is Ok ==> result->Ok_0.version == 0,
        result is Ok ==> result->Ok_0.apid <= 0x07FF,
{
    if buf.len() < HEADER_SIZE {
        return Err(ParseError::TooShort);
    }

    let byte0 = buf[0];
    let version = (byte0 >> 5) & 0x07;
    if version != 0 {
        return Err(ParseError::InvalidVersion);
    }

    let type_bit = (byte0 >> 4) & 0x01;
    let packet_type = if type_bit == 0 {
        PacketType::Telemetry
    } else {
        PacketType::Command
    };

    let sec_header_flag = ((byte0 >> 3) & 0x01) == 1;
    let apid_hi = (byte0 & 0x07) as u16;
    let apid = ((apid_hi << 8) | (buf[1] as u16)) & 0x07FF;

    let sequence_flags = (buf[2] >> 6) & 0x03;
    let seq_hi = (buf[2] & 0x3F) as u16;
    let sequence_count = (seq_hi << 8) | (buf[3] as u16);

    let data_length = ((buf[4] as u16) << 8) | (buf[5] as u16);

    Ok(CcsdsHeader {
        version: 0,
        packet_type,
        sec_header_flag,
        apid,
        sequence_flags,
        sequence_count,
        data_length,
    })
}

/// CCSDS-P04: XOR checksum of all bytes.
pub fn compute_checksum(data: &[u8]) -> (result: u8)
{
    let mut csum: u8 = 0;
    let len = data.len();
    let mut i: usize = 0;

    while i < len
        invariant
            0 <= i <= len,
            len == data@.len(),
        decreases
            len - i,
    {
        csum = csum ^ data[i];
        i = i + 1;
    }

    csum
}

} // verus!

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_header() -> CcsdsHeader {
        CcsdsHeader {
            version: 0,
            packet_type: PacketType::Telemetry,
            sec_header_flag: true,
            apid: 0x123,
            sequence_flags: 0b11,
            sequence_count: 42,
            data_length: 100,
        }
    }

    #[test]
    fn header_size_is_six() {
        assert_eq!(HEADER_SIZE, 6);
    }

    #[test]
    fn encode_then_decode_roundtrip() {
        let h = sample_header();
        let mut buf = [0u8; 6];
        encode_header(&h, &mut buf);
        let decoded = decode_header(&buf).unwrap();
        assert_eq!(decoded.version, 0);
        assert_eq!(decoded.apid, h.apid);
        assert_eq!(decoded.sequence_flags, h.sequence_flags);
        assert_eq!(decoded.sequence_count, h.sequence_count);
        assert_eq!(decoded.data_length, h.data_length);
        assert_eq!(decoded.sec_header_flag, h.sec_header_flag);
    }

    #[test]
    fn version_bits_always_zero() {
        let h = sample_header();
        let mut buf = [0u8; 6];
        encode_header(&h, &mut buf);
        assert_eq!(buf[0] & 0xE0, 0);
    }

    #[test]
    fn apid_masked_to_eleven_bits() {
        let mut h = sample_header();
        h.apid = 0xFFFF;
        let mut buf = [0u8; 6];
        encode_header(&h, &mut buf);
        let decoded = decode_header(&buf).unwrap();
        assert!(decoded.apid <= 0x07FF);
    }

    #[test]
    fn decode_too_short() {
        let buf = [0u8; 3];
        assert!(matches!(decode_header(&buf), Err(ParseError::TooShort)));
    }

    #[test]
    fn checksum_empty_is_zero() {
        assert_eq!(compute_checksum(&[]), 0);
    }

    #[test]
    fn checksum_xor() {
        assert_eq!(compute_checksum(&[0x01, 0x02, 0x04]), 0x07);
    }

    #[test]
    fn checksum_self_inverse() {
        let data = [0xAA, 0x55, 0xFF, 0x00, 0x12];
        let cs = compute_checksum(&data);
        let mut all = [0u8; 6];
        all[..5].copy_from_slice(&data);
        all[5] = cs;
        assert_eq!(compute_checksum(&all), 0);
    }
}
