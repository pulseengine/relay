#![no_std]

//! Relay CFDP Protocol Core — CCSDS File Delivery Protocol state machine.
//!
//! Protocol logic only: transaction states, ACK/NAK, metadata, EOF.
//! No file I/O. Verified core logic with Verus SMT/Z3 proofs.

pub mod engine;
