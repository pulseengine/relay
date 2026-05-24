//! MAVLink v2 frame envelope — verified mirror of `../plain/src/frame.rs`.
//!
//! Bodies are `#[verifier::external_body]` (slice indexing, CRC
//! folding, magic-byte checks — out of Verus's productive range);
//! the **signatures + `CodecError` variants** are the formal
//! contracts. Panic-freedom over arbitrary 25-byte input is
//! exhaustively pinned by `proptest_parser_never_panics` in the
//! plain test suite.

use crate::crc::Crc16X25;
use vstd::prelude::*;

verus! {

pub const MAGIC_V2: u8 = 0xFD;
pub const MAGIC_V1: u8 = 0xFE;
pub const MAX_FRAME_SIZE: usize = 12 + 255;
pub const HEADER_LEN: usize = 10;

pub struct FrameHeader {
    pub magic: u8,
    pub payload_len: u8,
    pub incompat_flags: u8,
    pub compat_flags: u8,
    pub sequence: u8,
    pub system_id: u8,
    pub component_id: u8,
    pub message_id: u32,
}

pub struct Frame<'a> {
    pub header: FrameHeader,
    pub payload: &'a [u8],
}

#[derive(PartialEq, Eq)]
pub enum CodecError {
    Truncated,
    BadMagic,
    BadCrc,
    UnsupportedMessage,
    BadPayloadLength,
    OutputTooSmall,
}

impl FrameHeader {
    #[verifier::external_body]
    pub fn wire_length(self) -> usize {
        HEADER_LEN + self.payload_len as usize + 2
    }
}

#[verifier::external_body]
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
    out[7] = msgid[0];
    out[8] = msgid[1];
    out[9] = msgid[2];
    let payload_end = HEADER_LEN + payload.len();
    out[HEADER_LEN..payload_end].copy_from_slice(payload);
    let mut crc = Crc16X25::new();
    crc.accumulate_slice(&out[1..payload_end]);
    crc.accumulate(crc_extra);
    let crc_value = crc.value();
    out[payload_end] = (crc_value & 0xFF) as u8;
    out[payload_end + 1] = (crc_value >> 8) as u8;
    Ok(n)
}

#[verifier::external_body]
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
    let payload = &buf[HEADER_LEN..HEADER_LEN + payload_len];
    let mut crc = Crc16X25::new();
    crc.accumulate_slice(&buf[1..HEADER_LEN + payload_len]);
    crc.accumulate(crc_extra);
    let want_crc = crc.value();
    let got_crc = u16::from_le_bytes([buf[HEADER_LEN + payload_len], buf[HEADER_LEN + payload_len + 1]]);
    if want_crc != got_crc {
        return Err(CodecError::BadCrc);
    }
    Ok((Frame { header, payload }, total))
}

#[verifier::external_body]
pub fn peek_message_id(buf: &[u8]) -> Result<u32, CodecError> {
    if buf.len() < HEADER_LEN {
        return Err(CodecError::Truncated);
    }
    if buf[0] != MAGIC_V2 {
        return Err(CodecError::BadMagic);
    }
    Ok(u32::from_le_bytes([buf[7], buf[8], buf[9], 0]))
}

} // verus!
