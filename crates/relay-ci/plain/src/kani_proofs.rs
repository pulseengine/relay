//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// CI-P01: a properly configured valid header is accepted
#[kani::proof]
fn verify_valid_header_accepted() {
    let mut config = CiConfig::new();
    let stream_id: u16 = kani::any();
    config.valid_stream_ids[0] = stream_id;
    config.stream_id_count = 1;
    let max_code: u8 = kani::any();
    kani::assume(max_code >= 1);
    config.max_cmd_code = max_code;
    config.min_length = 0;
    let max_len: u16 = kani::any();
    kani::assume(max_len >= 1);
    config.max_length = max_len;

    let func_code: u8 = kani::any();
    kani::assume(func_code <= max_code);
    let length: u16 = kani::any();
    kani::assume(length <= max_len);

    let header = CommandHeader {
        stream_id,
        sequence: kani::any(),
        length,
        function_code: func_code,
        checksum: 0,
    };
    assert_eq!(validate_header(&config, &header), CiValidation::Valid);
}

/// CI-P02: no panics for any symbolic input
#[kani::proof]
fn verify_no_panic() {
    let mut config = CiConfig::new();
    let sid_count: u32 = kani::any();
    kani::assume(sid_count <= MAX_STREAM_IDS as u32);
    config.stream_id_count = sid_count;
    config.max_cmd_code = kani::any();
    config.min_length = kani::any();
    config.max_length = kani::any();

    let header = CommandHeader {
        stream_id: kani::any(),
        sequence: kani::any(),
        length: kani::any(),
        function_code: kani::any(),
        checksum: kani::any(),
    };
    let _ = validate_header(&config, &header);
}
