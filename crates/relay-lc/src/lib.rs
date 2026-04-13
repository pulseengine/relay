#![no_std]

//! Relay Limit Checker — stream transformer: stream<sensor-reading> → stream<limit-violation>.
//!
//! Architecture (following Verification Guide):
//!
//! ```text
//! core.rs          ← Verus-annotated verified logic (no async, no alloc)
//!   │
//!   ├── verus-strip ──→ plain/src/core.rs → cargo test + Kani + coq_of_rust
//!   │
//!   └── lib.rs     ← P3 async wrapper (NOT verified, thin binding layer)
//! ```

pub mod core;
