//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// CCSDS-P01: encode then decode is a roundtrip for all valid fields
#[kani::proof]
fn verify_encode_decode_roundtrip() {
    let type_bit: bool = kani::any();
    let packet_type = if type_bit {
        PacketType::Command
    } else {
        PacketType::Telemetry
    };
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
        kani::any(),
        kani::any(),
        kani::any(),
        kani::any(),
        kani::any(),
        kani::any(),
    ];
    let _ = decode_header(&b);
    let _ = compute_checksum(&b);
}
