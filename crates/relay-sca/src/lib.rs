#![no_std]

//! Relay Stored Command Absolute — stream transformer: time events -> dispatched commands.
//!
//! Like relay-sc but commands fire at absolute timestamps (not relative delays).
//! Verified core logic with Verus SMT/Z3 proofs.

pub mod engine;
