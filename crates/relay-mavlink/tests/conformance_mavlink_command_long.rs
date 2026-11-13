include!("vectors/mavlink_command_long.rs");

use relay_mavlink::{
    COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, COMMAND_LONG_PAYLOAD_LEN, CommandLong,
};

fn param_bits(c: &CommandLong) -> [u32; 7] {
    [
        c.param1.to_bits(),
        c.param2.to_bits(),
        c.param3.to_bits(),
        c.param4.to_bits(),
        c.param5.to_bits(),
        c.param6.to_bits(),
        c.param7.to_bits(),
    ]
}

fn from_vec(v: &CmdVec) -> CommandLong {
    CommandLong {
        param1: f32::from_bits(v.param_bits[0]),
        param2: f32::from_bits(v.param_bits[1]),
        param3: f32::from_bits(v.param_bits[2]),
        param4: f32::from_bits(v.param_bits[3]),
        param5: f32::from_bits(v.param_bits[4]),
        param6: f32::from_bits(v.param_bits[5]),
        param7: f32::from_bits(v.param_bits[6]),
        command: v.command,
        target_system: v.target_system,
        target_component: v.target_component,
        confirmation: v.confirmation,
    }
}

#[test]
fn conformance_constants_match_reference() {
    assert_eq!(COMMAND_LONG_MSG_ID, REF_MSG_ID);
    assert_eq!(COMMAND_LONG_CRC_EXTRA, REF_CRC_EXTRA);
    assert_eq!(COMMAND_LONG_PAYLOAD_LEN, REF_PAYLOAD_LEN);
}

#[test]
fn conformance_tables_not_empty() {
    assert!(COMMAND_LONG_VECTORS.len() >= 150);
    assert!(!COMMAND_LONG_NEG.is_empty());
}

#[test]
fn conformance_command_long_decode_fields() {
    for (i, v) in COMMAND_LONG_VECTORS.iter().enumerate() {
        let d = CommandLong::decode_payload(&v.payload)
            .unwrap_or_else(|| panic!("vector {} failed to decode", i));
        let got = param_bits(&d);
        for k in 0..7 {
            assert_eq!(got[k], v.param_bits[k], "vector {} param{}", i, k + 1);
        }
        assert_eq!(d.command, v.command, "vector {} command", i);
        assert_eq!(
            d.target_system, v.target_system,
            "vector {} target_system",
            i
        );
        assert_eq!(
            d.target_component, v.target_component,
            "vector {} target_component",
            i
        );
        assert_eq!(d.confirmation, v.confirmation, "vector {} confirmation", i);
    }
}

#[test]
fn conformance_command_long_encode_matches_reference_bytes() {
    for (i, v) in COMMAND_LONG_VECTORS.iter().enumerate() {
        let c = from_vec(v);
        let enc = c.encode_payload();
        assert_eq!(enc[..], v.payload[..], "vector {} encode bytes", i);
    }
}

#[test]
fn conformance_command_long_decode_encode_roundtrip() {
    for (i, v) in COMMAND_LONG_VECTORS.iter().enumerate() {
        let d = CommandLong::decode_payload(&v.payload)
            .unwrap_or_else(|| panic!("vector {} failed to decode", i));
        let enc = d.encode_payload();
        assert_eq!(enc[..], v.payload[..], "vector {} roundtrip", i);
    }
}

#[test]
fn conformance_command_long_wrong_length_rule() {
    // MAVLink2 truncates trailing zero bytes (#675): an empty or over-long
    // payload is rejected; a 1..=32 byte payload decodes as if zero-extended.
    for (i, n) in COMMAND_LONG_NEG.iter().enumerate() {
        let len = n.bytes.len();
        assert_ne!(
            len, COMMAND_LONG_PAYLOAD_LEN,
            "neg vector {} has valid length",
            i
        );
        let got = CommandLong::decode_payload(n.bytes);
        if len == 0 || len > COMMAND_LONG_PAYLOAD_LEN {
            assert!(got.is_none(), "neg vector {} (len {}) was accepted", i, len);
        } else {
            let mut full = [0u8; COMMAND_LONG_PAYLOAD_LEN];
            full[..len].copy_from_slice(n.bytes);
            let want = CommandLong::decode_payload(&full).expect("zero-extended payload decodes");
            let got =
                got.unwrap_or_else(|| panic!("neg vector {} (len {}) not zero-extended", i, len));
            assert_eq!(
                got.encode_payload(),
                want.encode_payload(),
                "neg vector {} (len {}) zero-extension",
                i,
                len
            );
        }
    }
}
