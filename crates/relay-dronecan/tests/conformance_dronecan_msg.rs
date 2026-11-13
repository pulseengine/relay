use relay_dronecan::msg::{
    MAX_ESC, NodeStatus, decode_node_status, decode_raw_command, encode_node_status,
    encode_raw_command,
};

include!("vectors/dronecan_msg.rs");

#[test]
fn conformance_tables_populated() {
    assert!(
        NS_VECTORS.len() >= 150,
        "node_status table too small: {}",
        NS_VECTORS.len()
    );
    assert!(
        RC_VECTORS.len() >= 150,
        "raw_command table too small: {}",
        RC_VECTORS.len()
    );
    assert!(!NS_SHORT.is_empty());
}

#[test]
fn conformance_node_status_decode() {
    for (i, v) in NS_VECTORS.iter().enumerate() {
        let ns = decode_node_status(v.bytes)
            .unwrap_or_else(|| panic!("vector {i}: decode returned None"));
        assert_eq!(ns.uptime_sec, v.uptime_sec, "vector {i}: uptime_sec");
        assert_eq!(ns.health, v.health, "vector {i}: health");
        assert_eq!(ns.mode, v.mode, "vector {i}: mode");
        assert_eq!(ns.sub_mode, v.sub_mode, "vector {i}: sub_mode");
        assert_eq!(ns.vendor_status, v.vendor, "vector {i}: vendor_status");
    }
}

#[test]
fn conformance_node_status_encode() {
    for (i, v) in NS_VECTORS.iter().enumerate() {
        let s = NodeStatus {
            uptime_sec: v.uptime_sec,
            health: v.health,
            mode: v.mode,
            sub_mode: v.sub_mode,
            vendor_status: v.vendor,
        };
        let enc = encode_node_status(&s);
        assert_eq!(&enc[..], v.bytes, "vector {i}: encoded bytes");
    }
}

#[test]
fn conformance_node_status_trailing_bytes() {
    for (i, v) in NS_VECTORS.iter().enumerate() {
        let mut ext = v.bytes.to_vec();
        ext.extend_from_slice(&[0xFF, 0x00, 0xA5]);
        let ns = decode_node_status(&ext)
            .unwrap_or_else(|| panic!("vector {i}: decode of extended payload returned None"));
        assert_eq!(ns.uptime_sec, v.uptime_sec, "vector {i}: uptime_sec (ext)");
        assert_eq!(ns.health, v.health, "vector {i}: health (ext)");
        assert_eq!(ns.mode, v.mode, "vector {i}: mode (ext)");
        assert_eq!(ns.sub_mode, v.sub_mode, "vector {i}: sub_mode (ext)");
        assert_eq!(
            ns.vendor_status, v.vendor,
            "vector {i}: vendor_status (ext)"
        );
    }
}

#[test]
fn conformance_node_status_short_rejected() {
    for (i, b) in NS_SHORT.iter().enumerate() {
        assert!(b.len() < 7, "short vector {i}: table bug");
        assert_eq!(
            decode_node_status(b),
            None,
            "short vector {i}: len {}",
            b.len()
        );
    }
}

#[test]
fn conformance_raw_command_decode() {
    for (i, v) in RC_VECTORS.iter().enumerate() {
        let rc = decode_raw_command(v.bytes);
        let n = v.cmd.len();
        assert_eq!(rc.count, n, "vector {i}: count");
        assert_eq!(&rc.channels[..n], v.cmd, "vector {i}: channels");
    }
}

#[test]
fn conformance_raw_command_encode() {
    for (i, v) in RC_VECTORS.iter().enumerate() {
        let mut buf = [0xFFu8; 35];
        let n = encode_raw_command(v.cmd, &mut buf);
        assert_eq!(n, v.bytes.len(), "vector {i}: written length");
        assert_eq!(&buf[..n], v.bytes, "vector {i}: encoded bytes");
    }
}

#[test]
fn conformance_raw_command_small_buffer_returns_zero() {
    for (i, v) in RC_VECTORS.iter().enumerate() {
        if v.bytes.is_empty() {
            continue;
        }
        let mut small = vec![0xAAu8; v.bytes.len() - 1];
        assert_eq!(
            encode_raw_command(v.cmd, &mut small),
            0,
            "vector {i}: undersized buffer"
        );
    }
}

#[test]
fn conformance_raw_command_full_count_with_trailing_bytes() {
    for (i, v) in RC_VECTORS.iter().enumerate() {
        if v.cmd.len() != MAX_ESC {
            continue;
        }
        let mut ext = v.bytes.to_vec();
        ext.extend_from_slice(&[0xFFu8; 8]);
        let rc = decode_raw_command(&ext);
        assert_eq!(rc.count, MAX_ESC, "vector {i}: capped count");
        assert_eq!(&rc.channels[..MAX_ESC], v.cmd, "vector {i}: channels (ext)");
    }
}
