#![no_std]

//! Relay Stored Command — stream transformer: stream<time> → stream<dispatched-command>.
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
//!   - CommandStore: ATS + RTS stored command tables
//!   - process_tick(): dispatches commands whose time has come
//!
//! This file (`lib.rs`) wraps the verified core with:
//!   - wit-bindgen P3 async stream bindings
//!   - Stream read/write glue (outside verification boundary)

pub mod core;

// TODO: Enable when WIT interfaces are finalized and wit-bindgen P3
// bindings are generated via rules_wasm_component:
//
// wit_bindgen::generate!({
//     world: "stored-command",
//     path: "../../wit",
// });
