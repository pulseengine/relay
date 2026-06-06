//! Kani bounded-model-checking harnesses for the engine (plain-only).
//!
//! These live OUTSIDE engine.rs because engine.rs is a verus-strip mirror of
//! ../src/engine.rs (see the banner there). Keeping the BMC harnesses here
//! means a strip regen of engine.rs can never wipe them, and engine.rs stays
//! a faithful 1:1 mirror of its Verus source. Declared from lib.rs.
#![cfg(kani)]

use crate::engine::*;

/// FM-P01: empty paths are always rejected by validate_path
#[kani::proof]
fn verify_empty_path_rejected() {
    let p = FilePath::empty();
    assert!(!validate_path(&p));
}

/// FM-P02: no panics for any symbolic input to validate_request
#[kani::proof]
fn verify_no_panic() {
    let cmd_val: u8 = kani::any();
    kani::assume(cmd_val <= 7);
    let command = match cmd_val {
        0 => FmCommand::Copy,
        1 => FmCommand::Move,
        2 => FmCommand::Rename,
        3 => FmCommand::Delete,
        4 => FmCommand::CreateDir,
        5 => FmCommand::DeleteDir,
        6 => FmCommand::Decompress,
        _ => FmCommand::Concat,
    };
    let src_len: u32 = kani::any();
    kani::assume(src_len <= MAX_PATH_LEN as u32 + 2);
    let dest_len: u32 = kani::any();
    kani::assume(dest_len <= MAX_PATH_LEN as u32 + 2);
    let mut source = FilePath::empty();
    source.len = src_len;
    // Fill first byte to avoid null-byte rejection on valid-length paths
    if src_len > 0 && (src_len as usize) <= MAX_PATH_LEN {
        source.bytes[0] = b'/';
    }
    let mut dest = FilePath::empty();
    dest.len = dest_len;
    if dest_len > 0 && (dest_len as usize) <= MAX_PATH_LEN {
        dest.bytes[0] = b'/';
    }
    let req = FmRequest {
        command,
        source,
        dest,
    };
    let _ = validate_request(&req);
}
