#![no_std]

//! Relay CCSDS Packet Codec — CCSDS Space Packet Protocol encoding/decoding.
//!
//! Used by every mission for command/telemetry framing.
//! Verified core logic with Verus SMT/Z3 proofs.

pub mod engine;
pub mod sensor_wire;
