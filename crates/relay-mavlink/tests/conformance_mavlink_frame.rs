use relay_mavlink::{
    CodecError, FrameHeader, MAGIC_V2, MAX_FRAME_SIZE, encode_frame, parse_frame, peek_message_id,
};

include!("vectors/mavlink_frame.rs");

fn check_parse(i: usize, buf: &[u8], v: &FrameVec) {
    // SWREQ-FALCON-MAVLINK-P07: any incompat flag (including 0x01 signed)
    // is not implemented, so a CRC-valid frame that sets one is rejected.
    if v.incompat != 0 {
        assert_eq!(
            parse_frame(buf, v.crc_extra).map(|(_, n)| n),
            Err(CodecError::UnsupportedIncompatFlags),
            "vector {} incompat {:#04x} must be rejected",
            i,
            v.incompat
        );
        return;
    }
    let (f, n) = parse_frame(buf, v.crc_extra)
        .unwrap_or_else(|e| panic!("vector {} parse failed: {:?}", i, e));
    assert_eq!(n, v.wire_len, "vector {} consumed", i);
    assert_eq!(f.header.magic, MAGIC_V2, "vector {} magic", i);
    assert_eq!(
        f.header.payload_len, v.payload_len,
        "vector {} payload_len",
        i
    );
    assert_eq!(f.header.incompat_flags, v.incompat, "vector {} incompat", i);
    assert_eq!(f.header.compat_flags, v.compat, "vector {} compat", i);
    assert_eq!(f.header.sequence, v.seq, "vector {} seq", i);
    assert_eq!(f.header.system_id, v.sysid, "vector {} sysid", i);
    assert_eq!(f.header.component_id, v.compid, "vector {} compid", i);
    assert_eq!(f.header.message_id, v.msgid, "vector {} msgid", i);
    assert_eq!(f.payload, v.payload, "vector {} payload", i);
}

#[test]
fn conformance_vectors_present() {
    assert!(FRAME_VECTORS.len() >= 150, "too few frame vectors");
    assert!(!NEG_VECTORS.is_empty(), "no negative vectors");
}

#[test]
fn conformance_parse_valid_with_trailing() {
    for (i, v) in FRAME_VECTORS.iter().enumerate() {
        check_parse(i, v.raw, v);
    }
}

#[test]
fn conformance_parse_exact_slice() {
    for (i, v) in FRAME_VECTORS.iter().enumerate() {
        check_parse(i, &v.raw[..v.wire_len], v);
    }
}

#[test]
fn conformance_peek_message_id() {
    for (i, v) in FRAME_VECTORS.iter().enumerate() {
        assert_eq!(peek_message_id(v.raw), Ok(v.msgid), "vector {} peek", i);
    }
}

#[test]
fn conformance_encode_matches_reference() {
    for (i, v) in FRAME_VECTORS.iter().enumerate() {
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: v.payload_len,
            incompat_flags: v.incompat,
            compat_flags: v.compat,
            sequence: v.seq,
            system_id: v.sysid,
            component_id: v.compid,
            message_id: v.msgid,
        };
        let mut out = [0u8; MAX_FRAME_SIZE];
        let n = encode_frame(&header, v.payload, v.crc_extra, &mut out)
            .unwrap_or_else(|e| panic!("vector {} encode failed: {:?}", i, e));
        assert_eq!(n, v.wire_len, "vector {} encoded length", i);
        assert_eq!(
            &out[..n],
            &v.raw[..v.wire_len],
            "vector {} encoded bytes",
            i
        );
        let mut small = vec![0u8; v.wire_len - 1];
        assert_eq!(
            encode_frame(&header, v.payload, v.crc_extra, &mut small),
            Err(CodecError::OutputTooSmall),
            "vector {} small out",
            i
        );
    }
}

#[test]
fn conformance_truncated_valid_frames() {
    for (i, v) in FRAME_VECTORS.iter().enumerate() {
        let short = &v.raw[..v.wire_len - 1];
        assert_eq!(
            parse_frame(short, v.crc_extra),
            Err(CodecError::Truncated),
            "vector {} truncated by one",
            i
        );
        let tiny = &v.raw[..9];
        assert_eq!(
            parse_frame(tiny, v.crc_extra),
            Err(CodecError::Truncated),
            "vector {} tiny",
            i
        );
        assert_eq!(
            peek_message_id(tiny),
            Err(CodecError::Truncated),
            "vector {} peek tiny",
            i
        );
    }
}

#[test]
fn conformance_negative_vectors() {
    for (i, v) in NEG_VECTORS.iter().enumerate() {
        let want = match v.expect {
            0 => CodecError::Truncated,
            1 => CodecError::BadCrc,
            _ => CodecError::BadMagic,
        };
        assert_eq!(
            parse_frame(v.raw, v.crc_extra),
            Err(want),
            "negative vector {} parse",
            i
        );
        let peek_want = if v.raw.len() < 10 {
            Err(CodecError::Truncated)
        } else if v.raw[0] != MAGIC_V2 {
            Err(CodecError::BadMagic)
        } else {
            Ok(v.msgid)
        };
        assert_eq!(
            peek_message_id(v.raw),
            peek_want,
            "negative vector {} peek",
            i
        );
    }
}
