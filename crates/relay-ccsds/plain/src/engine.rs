//! Relay CCSDS Packet Codec — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_PACKET_SIZE: usize = 4096;
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
    InvalidType = 2,
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CcsdsHeader {
    pub version: u8,
    pub packet_type: PacketType,
    pub sec_header_flag: bool,
    pub apid: u16,
    pub sequence_flags: u8,
    pub sequence_count: u16,
    pub data_length: u16,
}

pub fn encode_header(header: &CcsdsHeader, buf: &mut [u8; 6]) {
    let type_bit: u8 = if header.packet_type == PacketType::Command { 1 } else { 0 };
    let sec_bit: u8 = if header.sec_header_flag { 1 } else { 0 };
    let apid_masked: u16 = header.apid & 0x07FF;
    let apid_hi: u8 = ((apid_masked >> 8) & 0x07) as u8;
    buf[0] = (type_bit << 4) | (sec_bit << 3) | apid_hi;
    buf[1] = (apid_masked & 0xFF) as u8;

    let seq_flags_masked: u8 = header.sequence_flags & 0x03;
    let seq_count_masked: u16 = header.sequence_count & 0x3FFF;
    let seq_hi: u8 = ((seq_count_masked >> 8) & 0x3F) as u8;
    buf[2] = (seq_flags_masked << 6) | seq_hi;
    buf[3] = (seq_count_masked & 0xFF) as u8;

    buf[4] = ((header.data_length >> 8) & 0xFF) as u8;
    buf[5] = (header.data_length & 0xFF) as u8;
}

pub fn decode_header(buf: &[u8]) -> Result<CcsdsHeader, ParseError> {
    if buf.len() < HEADER_SIZE {
        return Err(ParseError::TooShort);
    }

    let byte0 = buf[0];
    let byte1 = buf[1];
    let byte2 = buf[2];
    let byte3 = buf[3];
    let byte4 = buf[4];
    let byte5 = buf[5];

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
    let apid = (apid_hi << 8) | (byte1 as u16);

    let sequence_flags = (byte2 >> 6) & 0x03;
    let seq_hi = (byte2 & 0x3F) as u16;
    let sequence_count = (seq_hi << 8) | (byte3 as u16);

    let data_length = ((byte4 as u16) << 8) | (byte5 as u16);

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

pub fn compute_checksum(data: &[u8]) -> u8 {
    let mut csum: u8 = 0;
    let mut i: usize = 0;
    while i < data.len() {
        csum = csum ^ data[i];
        i = i + 1;
    }
    csum
}

pub fn validate_packet(buf: &[u8]) -> Result<CcsdsHeader, ParseError> {
    decode_header(buf)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_roundtrip_telemetry() {
        let header = CcsdsHeader {
            version: 0,
            packet_type: PacketType::Telemetry,
            sec_header_flag: false,
            apid: 0x123,
            sequence_flags: 3,
            sequence_count: 0x2ABC,
            data_length: 100,
        };
        let mut buf = [0u8; 6];
        encode_header(&header, &mut buf);
        let decoded = decode_header(&buf).unwrap();
        assert_eq!(decoded.version, 0);
        assert_eq!(decoded.packet_type, PacketType::Telemetry);
        assert_eq!(decoded.sec_header_flag, false);
        assert_eq!(decoded.apid, 0x123);
        assert_eq!(decoded.sequence_flags, 3);
        assert_eq!(decoded.sequence_count, 0x2ABC);
        assert_eq!(decoded.data_length, 100);
    }

    #[test]
    fn test_encode_decode_roundtrip_command() {
        let header = CcsdsHeader {
            version: 0,
            packet_type: PacketType::Command,
            sec_header_flag: true,
            apid: 0x7FF,
            sequence_flags: 1,
            sequence_count: 0,
            data_length: 0xFFFF,
        };
        let mut buf = [0u8; 6];
        encode_header(&header, &mut buf);
        let decoded = decode_header(&buf).unwrap();
        assert_eq!(decoded.packet_type, PacketType::Command);
        assert_eq!(decoded.sec_header_flag, true);
        assert_eq!(decoded.apid, 0x7FF);
        assert_eq!(decoded.sequence_flags, 1);
        assert_eq!(decoded.sequence_count, 0);
        assert_eq!(decoded.data_length, 0xFFFF);
    }

    #[test]
    fn test_invalid_version() {
        // Set version bits to non-zero (e.g., version = 1 => top 3 bits = 001 => byte0 = 0x20)
        let buf: [u8; 6] = [0x20, 0x00, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(decode_header(&buf), Err(ParseError::InvalidVersion));
    }

    #[test]
    fn test_too_short_buffer() {
        let buf: [u8; 3] = [0x00, 0x00, 0x00];
        assert_eq!(decode_header(&buf), Err(ParseError::TooShort));
        assert_eq!(decode_header(&[]), Err(ParseError::TooShort));
    }

    #[test]
    fn test_apid_masking() {
        // APID > 0x7FF should be masked to 11 bits
        let header = CcsdsHeader {
            version: 0,
            packet_type: PacketType::Telemetry,
            sec_header_flag: false,
            apid: 0xFFFF, // will be masked to 0x7FF
            sequence_flags: 0,
            sequence_count: 0,
            data_length: 0,
        };
        let mut buf = [0u8; 6];
        encode_header(&header, &mut buf);
        let decoded = decode_header(&buf).unwrap();
        assert_eq!(decoded.apid, 0x7FF);
    }

    #[test]
    fn test_checksum_empty() {
        assert_eq!(compute_checksum(&[]), 0);
    }

    #[test]
    fn test_checksum_single_byte() {
        assert_eq!(compute_checksum(&[0xAB]), 0xAB);
    }

    #[test]
    fn test_checksum_xor() {
        // 0xFF ^ 0xFF = 0x00
        assert_eq!(compute_checksum(&[0xFF, 0xFF]), 0x00);
        // 0x01 ^ 0x02 ^ 0x04 = 0x07
        assert_eq!(compute_checksum(&[0x01, 0x02, 0x04]), 0x07);
    }

    #[test]
    fn test_validate_packet_too_short() {
        assert_eq!(validate_packet(&[0x00, 0x01]), Err(ParseError::TooShort));
    }

    #[test]
    fn test_validate_packet_valid() {
        let mut buf = [0u8; 6];
        let header = CcsdsHeader {
            version: 0,
            packet_type: PacketType::Telemetry,
            sec_header_flag: false,
            apid: 42,
            sequence_flags: 0,
            sequence_count: 1,
            data_length: 10,
        };
        encode_header(&header, &mut buf);
        let result = validate_packet(&buf);
        assert!(result.is_ok());
        assert_eq!(result.unwrap().apid, 42);
    }

    #[test]
    fn test_version_always_zero_in_output() {
        let header = CcsdsHeader {
            version: 7, // even if input says 7, encode should write 0
            packet_type: PacketType::Telemetry,
            sec_header_flag: false,
            apid: 0,
            sequence_flags: 0,
            sequence_count: 0,
            data_length: 0,
        };
        let mut buf = [0u8; 6];
        encode_header(&header, &mut buf);
        // Top 3 bits of byte 0 should be 0
        assert_eq!(buf[0] & 0xE0, 0);
        let decoded = decode_header(&buf).unwrap();
        assert_eq!(decoded.version, 0);
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// CCSDS-P01: encode then decode is a roundtrip for all valid fields
    #[kani::proof]
    fn verify_encode_decode_roundtrip() {
        let type_bit: bool = kani::any();
        let packet_type = if type_bit { PacketType::Command } else { PacketType::Telemetry };
        let sec_header_flag: bool = kani::any();
        let apid: u16 = kani::any();
        let sequence_flags: u8 = kani::any();
        let sequence_count: u16 = kani::any();
        let data_length: u16 = kani::any();

        let header = CcsdsHeader {
            version: 0,
            packet_type,
            sec_header_flag,
            apid,
            sequence_flags,
            sequence_count,
            data_length,
        };
        let mut buf = [0u8; 6];
        encode_header(&header, &mut buf);
        let decoded = decode_header(&buf).unwrap();
        assert_eq!(decoded.packet_type, packet_type);
        assert_eq!(decoded.sec_header_flag, sec_header_flag);
        assert_eq!(decoded.apid, apid & 0x07FF);
        assert_eq!(decoded.sequence_flags, sequence_flags & 0x03);
        assert_eq!(decoded.sequence_count, sequence_count & 0x3FFF);
        assert_eq!(decoded.data_length, data_length);
    }

    /// CCSDS-P02: no panics for any symbolic input
    #[kani::proof]
    fn verify_no_panic() {
        // Test decode with arbitrary bytes
        let b: [u8; 6] = [
            kani::any(), kani::any(), kani::any(),
            kani::any(), kani::any(), kani::any(),
        ];
        let _ = decode_header(&b);
        let _ = compute_checksum(&b);
    }
}
