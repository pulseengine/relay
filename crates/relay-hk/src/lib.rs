#![no_std]

//! Relay Housekeeping — stream combiner: heartbeats + sensors -> hk packets.
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
//!   - CopyTable: fixed-size field-copy descriptor table
//!   - collect(): copies fields from sources into HK packet
//!
//! This file (`lib.rs`) wraps the verified core with:
//!   - wit-bindgen P3 async stream bindings
//!   - Stream read/write glue (outside verification boundary)

pub mod core;

// TODO: Enable when WIT interfaces are finalized and wit-bindgen P3
// bindings are generated via rules_wasm_component:
//
// wit_bindgen::generate!({
//     world: "housekeeping",
//     path: "../../wit",
// });
