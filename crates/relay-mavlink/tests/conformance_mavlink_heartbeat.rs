#[allow(dead_code)]
mod vectors {
    include!("vectors/mavlink_heartbeat.rs");
}
use relay_mavlink::{
    CodecError, FrameHeader, HEARTBEAT_PAYLOAD_LEN, Heartbeat, encode_frame, parse_frame,
};
use vectors::*;

#[test]
fn conformance_tables_nonempty() {
    assert!(!HB_PAYLOAD_VECTORS.is_empty());
    assert!(!HB_FRAME_VECTORS.is_empty());
    assert!(!HB_NEG_FRAME_VECTORS.is_empty());
    assert!(!HB_NEG_PAYLOAD_VECTORS.is_empty());
}

#[test]
fn conformance_payload_decode() {
    for (i, v) in HB_PAYLOAD_VECTORS.iter().enumerate() {
        assert_eq!(
            v.payload.len(),
            HEARTBEAT_PAYLOAD_LEN,
            "vector {} payload length",
            i
        );
        let hb = Heartbeat::decode_payload(v.payload)
            .unwrap_or_else(|| panic!("vector {} decode returned None", i));
        assert_eq!(hb.custom_mode, v.custom_mode, "vector {} custom_mode", i);
        assert_eq!(hb.mav_type, v.mav_type, "vector {} type", i);
        assert_eq!(hb.autopilot, v.autopilot, "vector {} autopilot", i);
        assert_eq!(hb.base_mode, v.base_mode, "vector {} base_mode", i);
        assert_eq!(
            hb.system_status, v.system_status,
            "vector {} system_status",
            i
        );
        assert_eq!(
            hb.mavlink_version, v.mavlink_version,
            "vector {} mavlink_version",
            i
        );
    }
}

#[test]
fn conformance_payload_encode() {
    for (i, v) in HB_PAYLOAD_VECTORS.iter().enumerate() {
        let hb = Heartbeat {
            custom_mode: v.custom_mode,
            mav_type: v.mav_type,
            autopilot: v.autopilot,
            base_mode: v.base_mode,
            system_status: v.system_status,
            mavlink_version: v.mavlink_version,
        };
        let enc = hb.encode_payload();
        assert_eq!(&enc[..], v.payload, "vector {} encode_payload bytes", i);
    }
}

#[test]
fn conformance_payload_negative() {
    // Empty or over-long is rejected; 1..=8 bytes decodes zero-extended (#675).
    for (i, v) in HB_NEG_PAYLOAD_VECTORS.iter().enumerate() {
        let len = v.bytes.len();
        let got = Heartbeat::decode_payload(v.bytes);
        if len == 0 || len > HEARTBEAT_PAYLOAD_LEN {
            assert!(
                got.is_none(),
                "neg payload vector {} (len {}) must be rejected",
                i,
                len
            );
        } else {
            let mut full = [0u8; HEARTBEAT_PAYLOAD_LEN];
            full[..len].copy_from_slice(v.bytes);
            assert_eq!(
                got,
                Heartbeat::decode_payload(&full),
                "neg payload vector {} (len {}) must zero-extend",
                i,
                len
            );
        }
    }
}

#[test]
fn conformance_frame_parse() {
    for (i, v) in HB_FRAME_VECTORS.iter().enumerate() {
        let (frame, used) = parse_frame(v.raw, CRC_EXTRA_REF)
            .unwrap_or_else(|e| panic!("frame vector {} parse failed: {:?}", i, e));
        assert_eq!(used, v.raw.len(), "frame vector {} consumed", i);
        let h = frame.header;
        assert_eq!(h.magic, v.raw[0], "frame vector {} magic", i);
        assert_eq!(
            h.payload_len, v.payload_len,
            "frame vector {} payload_len",
            i
        );
        assert_eq!(
            h.incompat_flags, v.incompat_flags,
            "frame vector {} incompat",
            i
        );
        assert_eq!(h.compat_flags, v.compat_flags, "frame vector {} compat", i);
        assert_eq!(h.sequence, v.sequence, "frame vector {} sequence", i);
        assert_eq!(h.system_id, v.system_id, "frame vector {} system_id", i);
        assert_eq!(
            h.component_id, v.component_id,
            "frame vector {} component_id",
            i
        );
        assert_eq!(h.message_id, v.message_id, "frame vector {} message_id", i);
        let plen = v.payload_len as usize;
        assert_eq!(
            frame.payload,
            &v.raw[10..10 + plen],
            "frame vector {} payload bytes",
            i
        );
        assert!(
            plen <= HEARTBEAT_PAYLOAD_LEN,
            "frame vector {} payload too long",
            i
        );
        let mut full = [0u8; HEARTBEAT_PAYLOAD_LEN];
        full[..plen].copy_from_slice(frame.payload);
        let hb = Heartbeat::decode_payload(&full)
            .unwrap_or_else(|| panic!("frame vector {} decode None", i));
        // The truncated wire payload decodes directly, zero-extended (#675).
        assert_eq!(
            Heartbeat::decode_payload(frame.payload),
            Some(hb),
            "frame vector {} truncated payload must zero-extend",
            i
        );
        assert_eq!(
            hb.custom_mode, v.custom_mode,
            "frame vector {} custom_mode",
            i
        );
        assert_eq!(hb.mav_type, v.mav_type, "frame vector {} type", i);
        assert_eq!(hb.autopilot, v.autopilot, "frame vector {} autopilot", i);
        assert_eq!(hb.base_mode, v.base_mode, "frame vector {} base_mode", i);
        assert_eq!(
            hb.system_status, v.system_status,
            "frame vector {} system_status",
            i
        );
        assert_eq!(
            hb.mavlink_version, v.mavlink_version,
            "frame vector {} mavlink_version",
            i
        );
        let mut ext = v.raw.to_vec();
        ext.extend_from_slice(&[0xAA, 0x55]);
        let (_f2, used2) = parse_frame(&ext, CRC_EXTRA_REF).unwrap_or_else(|e| {
            panic!(
                "frame vector {} parse with trailing bytes failed: {:?}",
                i, e
            )
        });
        assert_eq!(
            used2,
            v.raw.len(),
            "frame vector {} consumed with trailing bytes",
            i
        );
    }
}

#[test]
fn conformance_frame_encode() {
    for (i, v) in HB_FRAME_VECTORS.iter().enumerate() {
        let plen = v.payload_len as usize;
        let header = FrameHeader {
            magic: v.raw[0],
            payload_len: v.payload_len,
            incompat_flags: v.incompat_flags,
            compat_flags: v.compat_flags,
            sequence: v.sequence,
            system_id: v.system_id,
            component_id: v.component_id,
            message_id: v.message_id,
        };
        let mut out = [0u8; 300];
        let n = encode_frame(&header, &v.raw[10..10 + plen], CRC_EXTRA_REF, &mut out)
            .unwrap_or_else(|e| panic!("frame vector {} encode failed: {:?}", i, e));
        assert_eq!(&out[..n], v.raw, "frame vector {} encoded bytes", i);
    }
}

#[test]
fn conformance_frame_wrong_crc_extra() {
    for (i, v) in HB_FRAME_VECTORS.iter().enumerate() {
        match parse_frame(v.raw, CRC_EXTRA_REF.wrapping_add(1)) {
            Err(e) => assert_eq!(e, CodecError::BadCrc, "frame vector {} wrong crc_extra", i),
            Ok(_) => panic!("frame vector {} parsed with wrong crc_extra", i),
        }
    }
}

#[test]
fn conformance_frame_negative() {
    for (i, v) in HB_NEG_FRAME_VECTORS.iter().enumerate() {
        let expected = match v.kind {
            0 => CodecError::Truncated,
            1 => CodecError::BadCrc,
            _ => CodecError::BadMagic,
        };
        match parse_frame(v.raw, CRC_EXTRA_REF) {
            Err(e) => assert_eq!(e, expected, "neg frame vector {}", i),
            Ok(_) => panic!("neg frame vector {} unexpectedly parsed", i),
        }
    }
}
