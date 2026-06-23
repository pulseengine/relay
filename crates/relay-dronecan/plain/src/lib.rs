//! Relay DroneCAN v0 — the verified transfer layer for falcon's peripheral bus.
//!
//! DroneCAN v0 (formerly UAVCAN v0): classic CAN 2.0B, 29-bit extended IDs,
//! 8-byte data frames, a 1-byte tail (start/end/toggle/transfer-id), and a
//! CRC-16-CCITT transfer checksum for multi-frame transfers. The 6X-RT's
//! ESC/GPS/mag/power peripherals speak this today (PX4/ArduPilot default); it is
//! the simpler-than-Cyphal protocol, which makes the reassembler Kani-tractable.
//!
//! This crate owns the PURE verified protocol — relay's side of the relay#214 /
//! jess#62 split. jess moves raw CAN frames over the `relay-hal` CanBus seam;
//! this crate decodes the 29-bit ID ([`id`]), the tail byte ([`tail`]), the
//! transfer CRC ([`crc`]), and reassembles multi-frame transfers ([`transfer`])
//! with the buffer-bound + CRC-gating + toggle/transfer-id sequencing the
//! `spar/dronecan.aadl` EMV2 fault model predicts.
//!
//! The architecture (timing + EMV2 fault tree) lives in spar/dronecan.aadl; the
//! node interface WIT is generated from it (wit/dronecan/node.wit). The Kani
//! harnesses in `kani_proofs.rs` are the implementation evidence for the AADL
//! error sinks: corrupt multi-frame -> CRC sink; oversize -> buffer-bound sink.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

pub mod crc;
pub mod id;
pub mod tail;
pub mod transfer;

#[cfg(kani)]
mod kani_proofs;
