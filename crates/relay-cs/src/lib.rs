#![no_std]

//! Relay Checksum — stream transformer: stream<data-region> → stream<check-result>.
//!
//! Architecture (following Verification Guide):
//!
//! ```text
//! core.rs          <- Verus-annotated verified logic (no async, no alloc)
//!   |
//!   +-- verus-strip --> plain/src/core.rs -> cargo test + Kani + coq_of_rust
//!   |
//!   +-- lib.rs     <- P3 async wrapper (NOT verified, thin binding layer)
//! ```
//!
//! The verified core (`core.rs`) contains:
//!   - ChecksumTable: fixed-size region tracking table
//!   - crc32_compute(): CRC32 with standard polynomial
//!   - check_region()/check_batch(): integrity verification
//!
//! This file (`lib.rs`) wraps the verified core with:
//!   - wit-bindgen P3 async stream bindings
//!   - Stream read/write glue (outside verification boundary)

pub mod core;
