//! MAVLink v2 frame envelope — what goes on the wire.
//!
//! Frame layout:
//!
//!   offset 0   magic            u8   = 0xFD for v2
//!   offset 1   payload_len      u8
//!   offset 2   incompat_flags   u8   = 0 for unsigned, 1 for signed
//!   offset 3   compat_flags     u8
//!   offset 4   sequence         u8
//!   offset 5   system_id        u8
//!   offset 6   component_id     u8
//!   offset 7..10  message_id   u24  (3 bytes, little-endian)
//!   offset 10..10+payload_len   payload
//!   offset 10+payload_len..+2   crc16            (little-endian)
//!   offset 10+payload_len+2..   signature (13 bytes, if incompat_flags & 1)
//!
//! For v0.1 we do unsigned packets only — signed-packet support
//! lands when we wire up sigil-style attestation for ground link.
//! That's a v1.0 feature per the falcon roadmap.

use crate::crc::Crc16X25;

/// MAVLink v2 magic byte (start-of-frame).
pub const MAGIC_V2: u8 = 0xFD;

/// MAVLink v1 magic byte (we reject these in v0.1).
pub const MAGIC_V1: u8 = 0xFE;

/// Maximum on-the-wire frame size (header + max payload + CRC,
/// no signature). 255 max payload + 12 header/CRC overhead = 267.
pub const MAX_FRAME_SIZE: usize = 12 + 255;

/// Header overhead before the payload (magic..msgid inclusive).
pub const HEADER_LEN: usize = 10;

/// Frame header — what precedes the payload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct FrameHeader {
    pub magic: u8,
    pub payload_len: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub sequence: u8,
    pub system_id: u8,
    pub component_id: u8,
    pub message_id: u32, // 24-bit on the wire, stored as u32
}

/// Decoded frame: header + payload reference.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Frame<'a> {
    pub header: FrameHeader,
    pub payload: &'a [u8],
}

/// Codec error variants. No allocation, no string formatting.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CodecError {
    /// Buffer too short to contain a valid frame.
    Truncated,
    /// Wrong magic byte at start of frame.
    BadMagic,
    /// CRC validation failed.
    BadCrc,
    /// Message id not implemented in this codec build.
    /// v0.1: only HEARTBEAT (id 0).
    UnsupportedMessage,
    /// Payload length in header disagrees with the per-message
    /// expected length, or with bytes available in buffer.
    BadPayloadLength,
    /// Output buffer too small for encoded frame.
    OutputTooSmall,
}

impl FrameHeader {
    /// Total bytes this frame will occupy on the wire (header +
    /// payload + 2-byte CRC, no signature).
    pub const fn wire_length(self) -> usize {
        HEADER_LEN + self.payload_len as usize + 2
    }
}

/// Encode a frame header + payload + computed CRC into `out`.
///
/// `crc_extra` must be the per-message magic byte from the MAVLink
/// XML (e.g. `HEARTBEAT_CRC_EXTRA = 50`).
///
/// Returns the number of bytes written, or `OutputTooSmall` if
/// `out` is too small.
pub fn encode_frame(
    header: &FrameHeader,
    payload: &[u8],
    crc_extra: u8,
    out: &mut [u8],
) -> Result<usize, CodecError> {
    let n = HEADER_LEN + payload.len() + 2;
    if out.len() < n {
        return Err(CodecError::OutputTooSmall);
    }
    if payload.len() != header.payload_len as usize {
        return Err(CodecError::BadPayloadLength);
    }

    out[0] = header.magic;
    out[1] = header.payload_len;
    out[2] = header.incompat_flags;
    out[3] = header.compat_flags;
    out[4] = header.sequence;
    out[5] = header.system_id;
    out[6] = header.component_id;
    let msgid = header.message_id.to_le_bytes();
    // 24-bit msg id on the wire — drop the top byte.
    out[7] = msgid[0];
    out[8] = msgid[1];
    out[9] = msgid[2];
    let payload_end = HEADER_LEN + payload.len();
    out[HEADER_LEN..payload_end].copy_from_slice(payload);

    // CRC is computed over: len, incompat, compat, seq, sysid, compid,
    // msgid (3 bytes), payload, then CRC_EXTRA.
    // Crucially, the magic byte (out[0]) is NOT included.
    let mut crc = Crc16X25::new();
    crc.accumulate_slice(&out[1..payload_end]);
    crc.accumulate(crc_extra);
    let crc_value = crc.value();

    out[payload_end] = (crc_value & 0xFF) as u8;
    out[payload_end + 1] = (crc_value >> 8) as u8;

    Ok(n)
}

/// Parse a frame from a byte buffer.
///
/// The caller passes the expected `crc_extra` for the message-id
/// it's expecting (typically: peek msg-id from header first, look
/// up CRC_EXTRA, then call this). For HEARTBEAT, it's 50.
///
/// Returns the parsed frame and the number of bytes consumed.
pub fn parse_frame<'a>(
    buf: &'a [u8],
    crc_extra: u8,
) -> Result<(Frame<'a>, usize), CodecError> {
    if buf.len() < HEADER_LEN {
        return Err(CodecError::Truncated);
    }
    let magic = buf[0];
    if magic != MAGIC_V2 {
        return Err(CodecError::BadMagic);
    }
    let payload_len = buf[1] as usize;
    let total = HEADER_LEN + payload_len + 2;
    if buf.len() < total {
        return Err(CodecError::Truncated);
    }
    let header = FrameHeader {
        magic,
        payload_len: buf[1],
        incompat_flags: buf[2],
        compat_flags: buf[3],
        sequence: buf[4],
        system_id: buf[5],
        component_id: buf[6],
        message_id: u32::from_le_bytes([buf[7], buf[8], buf[9], 0]),
    };
    let payload_end = HEADER_LEN + payload_len;
    let payload = &buf[HEADER_LEN..payload_end];

    let mut crc = Crc16X25::new();
    crc.accumulate_slice(&buf[1..payload_end]);
    crc.accumulate(crc_extra);
    let expected = crc.value();
    let actual = u16::from_le_bytes([buf[payload_end], buf[payload_end + 1]]);
    if actual != expected {
        return Err(CodecError::BadCrc);
    }

    Ok((Frame { header, payload }, total))
}

/// Peek the message id from a frame buffer without parsing the rest.
/// Useful when the caller needs to dispatch on msg-id to choose the
/// right `CRC_EXTRA` for the full parse.
pub fn peek_message_id(buf: &[u8]) -> Result<u32, CodecError> {
    if buf.len() < HEADER_LEN {
        return Err(CodecError::Truncated);
    }
    if buf[0] != MAGIC_V2 {
        return Err(CodecError::BadMagic);
    }
    Ok(u32::from_le_bytes([buf[7], buf[8], buf[9], 0]))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::heartbeat::{Heartbeat, HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID,
                            HEARTBEAT_PAYLOAD_LEN};

    fn build_heartbeat_frame(seq: u8, hb: &Heartbeat) -> [u8; HEADER_LEN + HEARTBEAT_PAYLOAD_LEN + 2] {
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: seq,
            system_id: 1,
            component_id: 1,
            message_id: HEARTBEAT_MSG_ID,
        };
        let payload = hb.encode_payload();
        let mut out = [0u8; HEADER_LEN + HEARTBEAT_PAYLOAD_LEN + 2];
        let n = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut out)
            .expect("encode");
        assert_eq!(n, out.len());
        out
    }

    #[test]
    fn frame_starts_with_magic_v2() {
        let hb = Heartbeat::falcon_quad_standby();
        let frame = build_heartbeat_frame(0, &hb);
        assert_eq!(frame[0], MAGIC_V2);
    }

    #[test]
    fn frame_payload_length_byte_matches() {
        let hb = Heartbeat::falcon_quad_standby();
        let frame = build_heartbeat_frame(0, &hb);
        assert_eq!(frame[1] as usize, HEARTBEAT_PAYLOAD_LEN);
    }

    #[test]
    fn frame_msg_id_three_bytes_little_endian() {
        let hb = Heartbeat::falcon_quad_standby();
        let frame = build_heartbeat_frame(0, &hb);
        // HEARTBEAT id is 0.
        assert_eq!(frame[7], 0);
        assert_eq!(frame[8], 0);
        assert_eq!(frame[9], 0);
    }

    #[test]
    fn round_trip_heartbeat_through_frame() {
        let hb_in = Heartbeat::falcon_quad_standby();
        let frame = build_heartbeat_frame(42, &hb_in);
        let (parsed, n) = parse_frame(&frame, HEARTBEAT_CRC_EXTRA)
            .expect("parse heartbeat frame");
        assert_eq!(n, frame.len());
        assert_eq!(parsed.header.magic, MAGIC_V2);
        assert_eq!(parsed.header.payload_len as usize, HEARTBEAT_PAYLOAD_LEN);
        assert_eq!(parsed.header.sequence, 42);
        assert_eq!(parsed.header.system_id, 1);
        assert_eq!(parsed.header.component_id, 1);
        assert_eq!(parsed.header.message_id, HEARTBEAT_MSG_ID);
        let hb_out = Heartbeat::decode_payload(parsed.payload).expect("decode");
        assert_eq!(hb_in, hb_out);
    }

    #[test]
    fn rejects_v1_magic() {
        let mut buf = [0u8; 12];
        buf[0] = MAGIC_V1;
        let err = parse_frame(&buf, HEARTBEAT_CRC_EXTRA).unwrap_err();
        assert_eq!(err, CodecError::BadMagic);
    }

    #[test]
    fn rejects_truncated_header() {
        let buf = [MAGIC_V2; 5];
        let err = parse_frame(&buf, HEARTBEAT_CRC_EXTRA).unwrap_err();
        assert_eq!(err, CodecError::Truncated);
    }

    #[test]
    fn rejects_truncated_payload() {
        let mut buf = [0u8; HEADER_LEN];
        buf[0] = MAGIC_V2;
        buf[1] = 9; // claims 9-byte payload but buffer is just 10 bytes
        let err = parse_frame(&buf, HEARTBEAT_CRC_EXTRA).unwrap_err();
        assert_eq!(err, CodecError::Truncated);
    }

    #[test]
    fn rejects_bad_crc() {
        let hb = Heartbeat::falcon_quad_standby();
        let mut frame = build_heartbeat_frame(0, &hb);
        // Flip last byte (high CRC byte).
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        let err = parse_frame(&frame, HEARTBEAT_CRC_EXTRA).unwrap_err();
        assert_eq!(err, CodecError::BadCrc);
    }

    #[test]
    fn rejects_wrong_crc_extra() {
        let hb = Heartbeat::falcon_quad_standby();
        let frame = build_heartbeat_frame(0, &hb);
        // Parse with the wrong CRC_EXTRA — should fail CRC.
        let err = parse_frame(&frame, 49).unwrap_err();
        assert_eq!(err, CodecError::BadCrc);
    }

    #[test]
    fn encode_rejects_too_small_output() {
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: 0,
            system_id: 1,
            component_id: 1,
            message_id: HEARTBEAT_MSG_ID,
        };
        let payload = Heartbeat::falcon_quad_standby().encode_payload();
        let mut out = [0u8; 5]; // too small
        let err = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut out)
            .unwrap_err();
        assert_eq!(err, CodecError::OutputTooSmall);
    }

    #[test]
    fn encode_rejects_payload_length_mismatch() {
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: 5, // wrong
            incompat_flags: 0,
            compat_flags: 0,
            sequence: 0,
            system_id: 1,
            component_id: 1,
            message_id: HEARTBEAT_MSG_ID,
        };
        let payload = Heartbeat::falcon_quad_standby().encode_payload();
        let mut out = [0u8; MAX_FRAME_SIZE];
        let err = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut out)
            .unwrap_err();
        assert_eq!(err, CodecError::BadPayloadLength);
    }

    #[test]
    fn peek_message_id_finds_heartbeat() {
        let hb = Heartbeat::falcon_quad_standby();
        let frame = build_heartbeat_frame(0, &hb);
        let id = peek_message_id(&frame).expect("peek");
        assert_eq!(id, HEARTBEAT_MSG_ID);
    }

    #[test]
    fn peek_rejects_v1_magic() {
        let mut buf = [0u8; HEADER_LEN];
        buf[0] = MAGIC_V1;
        let err = peek_message_id(&buf).unwrap_err();
        assert_eq!(err, CodecError::BadMagic);
    }

    use proptest::prelude::*;

    proptest! {
        /// Fuzz the parser with arbitrary bytes — it must never panic.
        #[test]
        fn parser_never_panics(bytes in proptest::collection::vec(any::<u8>(), 0..300)) {
            // Result either way; goal is just "no panic".
            let _ = parse_frame(&bytes, HEARTBEAT_CRC_EXTRA);
            let _ = peek_message_id(&bytes);
        }

        /// Encoded frames always round-trip through parse.
        #[test]
        fn encode_parse_round_trip_arbitrary_seq(
            seq in any::<u8>(),
            sysid in 1u8..=255,
            compid in any::<u8>(),
            custom_mode in any::<u32>(),
            mav_type in any::<u8>(),
        ) {
            let hb = Heartbeat {
                custom_mode,
                mav_type,
                autopilot: 0,
                base_mode: 0,
                system_status: 0,
                mavlink_version: 2,
            };
            let header = FrameHeader {
                magic: MAGIC_V2,
                payload_len: HEARTBEAT_PAYLOAD_LEN as u8,
                incompat_flags: 0,
                compat_flags: 0,
                sequence: seq,
                system_id: sysid,
                component_id: compid,
                message_id: HEARTBEAT_MSG_ID,
            };
            let payload = hb.encode_payload();
            let mut buf = [0u8; MAX_FRAME_SIZE];
            let n = encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut buf)
                .expect("encode");
            let (parsed, consumed) = parse_frame(&buf[..n], HEARTBEAT_CRC_EXTRA)
                .expect("parse round trip");
            prop_assert_eq!(consumed, n);
            prop_assert_eq!(parsed.header, header);
            let hb_out = Heartbeat::decode_payload(parsed.payload).expect("decode");
            prop_assert_eq!(hb_out, hb);
        }
    }
}
