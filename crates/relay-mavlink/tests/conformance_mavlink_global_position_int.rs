use relay_mavlink::{CodecError, GLOBAL_POSITION_INT_PAYLOAD_LEN, GlobalPositionInt, parse_frame};

include!("vectors/mavlink_global_position_int.rs");

fn expected(v: &GpiVec) -> GlobalPositionInt {
    GlobalPositionInt {
        time_boot_ms: v.time_boot_ms,
        lat_e7: v.lat,
        lon_e7: v.lon,
        alt_mm: v.alt,
        relative_alt_mm: v.relative_alt,
        vx_cms: v.vx,
        vy_cms: v.vy,
        vz_cms: v.vz,
        hdg_cdeg: v.hdg,
    }
}

fn check_fields(i: usize, got: &GlobalPositionInt, v: &GpiVec) {
    assert_eq!(
        got.time_boot_ms, v.time_boot_ms,
        "vector {}: time_boot_ms",
        i
    );
    assert_eq!(got.lat_e7, v.lat, "vector {}: lat", i);
    assert_eq!(got.lon_e7, v.lon, "vector {}: lon", i);
    assert_eq!(got.alt_mm, v.alt, "vector {}: alt", i);
    assert_eq!(
        got.relative_alt_mm, v.relative_alt,
        "vector {}: relative_alt",
        i
    );
    assert_eq!(got.vx_cms, v.vx, "vector {}: vx", i);
    assert_eq!(got.vy_cms, v.vy, "vector {}: vy", i);
    assert_eq!(got.vz_cms, v.vz, "vector {}: vz", i);
    assert_eq!(got.hdg_cdeg, v.hdg, "vector {}: hdg", i);
}

#[test]
fn conformance_decode_full_payload() {
    assert!(!GPI_VECTORS.is_empty());
    for (i, v) in GPI_VECTORS.iter().enumerate() {
        let d = match GlobalPositionInt::decode_payload(v.payload_full) {
            Some(d) => d,
            None => panic!("vector {}: decode_payload returned None", i),
        };
        check_fields(i, &d, v);
    }
}

#[test]
fn conformance_encode_payload_matches_reference() {
    for (i, v) in GPI_VECTORS.iter().enumerate() {
        let enc = expected(v).encode_payload();
        assert_eq!(
            &enc[..],
            v.payload_full,
            "vector {}: encoded payload bytes",
            i
        );
    }
}

#[test]
fn conformance_frame_with_mavlink2_truncation() {
    for (i, v) in GPI_VECTORS.iter().enumerate() {
        let (frame, consumed) = match parse_frame(v.wire, v.crc_extra) {
            Ok(x) => x,
            Err(e) => panic!("vector {}: parse_frame failed: {:?}", i, e),
        };
        assert_eq!(consumed, v.wire.len(), "vector {}: consumed", i);
        assert_eq!(frame.header.magic, v.wire[0], "vector {}: magic", i);
        assert_eq!(
            frame.header.payload_len as usize, v.trunc_len,
            "vector {}: payload_len",
            i
        );
        assert_eq!(
            frame.header.incompat_flags, v.wire[2],
            "vector {}: incompat",
            i
        );
        assert_eq!(frame.header.compat_flags, v.wire[3], "vector {}: compat", i);
        assert_eq!(frame.header.sequence, v.seq, "vector {}: seq", i);
        assert_eq!(frame.header.system_id, v.sysid, "vector {}: sysid", i);
        assert_eq!(frame.header.component_id, v.compid, "vector {}: compid", i);
        assert_eq!(frame.header.message_id, v.msg_id, "vector {}: msg id", i);
        assert_eq!(
            frame.payload,
            &v.payload_full[..v.trunc_len],
            "vector {}: trimmed payload",
            i
        );
        // Caller-side zero extension of a MAVLink2-truncated payload must decode to the same fields.
        let mut buf = vec![0u8; v.payload_full.len()];
        buf[..frame.payload.len()].copy_from_slice(frame.payload);
        let d = match GlobalPositionInt::decode_payload(&buf) {
            Some(d) => d,
            None => panic!("vector {}: zero-extended decode returned None", i),
        };
        check_fields(i, &d, v);
    }
}

#[test]
fn conformance_truncated_payload_zero_extended_by_decoder() {
    // MAVLink2 truncates trailing zero bytes (#675): the decoder zero-extends
    // a 1..=27 byte payload, so the truncated wire payload decodes to the
    // same message as the full one.
    for (i, v) in GPI_VECTORS.iter().enumerate() {
        if v.trunc_len < v.payload_full.len() {
            let r = GlobalPositionInt::decode_payload(&v.payload_full[..v.trunc_len]);
            let full = GlobalPositionInt::decode_payload(v.payload_full);
            if v.trunc_len == 0 {
                assert!(r.is_none(), "vector {}: empty payload must be rejected", i);
            } else {
                assert_eq!(r, full, "vector {}: truncated payload must zero-extend", i);
            }
        }
    }
}

#[test]
fn conformance_wrong_length_payload_rule() {
    // Empty or over-long is rejected; 1..=27 bytes decodes zero-extended (#675).
    assert!(!GPI_BAD_LEN.is_empty());
    for (i, p) in GPI_BAD_LEN.iter().enumerate() {
        let got = GlobalPositionInt::decode_payload(p);
        if p.is_empty() || p.len() > GLOBAL_POSITION_INT_PAYLOAD_LEN {
            assert!(
                got.is_none(),
                "bad-length case {}: len {} must be rejected",
                i,
                p.len()
            );
        } else {
            let mut full = [0u8; GLOBAL_POSITION_INT_PAYLOAD_LEN];
            full[..p.len()].copy_from_slice(p);
            assert_eq!(
                got,
                GlobalPositionInt::decode_payload(&full),
                "bad-length case {}: len {} must zero-extend",
                i,
                p.len()
            );
        }
    }
}

#[test]
fn conformance_bad_crc_frames_rejected() {
    assert!(!GPI_BAD_CRC.is_empty());
    for (i, n) in GPI_BAD_CRC.iter().enumerate() {
        match parse_frame(n.wire, n.crc_extra) {
            Err(CodecError::BadCrc) => {}
            other => panic!("bad-crc case {}: expected BadCrc, got {:?}", i, other),
        }
    }
}

#[test]
fn conformance_truncated_frames_rejected() {
    assert!(!GPI_TRUNCATED_FRAMES.is_empty());
    for (i, n) in GPI_TRUNCATED_FRAMES.iter().enumerate() {
        match parse_frame(n.wire, n.crc_extra) {
            Err(CodecError::Truncated) => {}
            other => panic!(
                "truncated-frame case {}: expected Truncated, got {:?}",
                i, other
            ),
        }
    }
}
