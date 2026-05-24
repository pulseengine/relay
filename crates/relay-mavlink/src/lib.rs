//! Relay MAVLink — verified-engine source tree.
//!
//! Verus-annotated source of truth. The plain mirror at
//! `../plain/src/lib.rs` is what cargo builds; this tree is what
//! `verus_test` checks via Bazel (`//:relay_mavlink_verus_test`).
//!
//! What Verus actually proves here
//! -------------------------------
//!
//! The MAVLink frame/CRC/payload bodies are bit-level transforms +
//! slice indexing — Verus reasons about those poorly, so they are
//! marked `#[verifier::external_body]` and their **signatures** are
//! the formal contracts (payload size, enum totality, no panic).
//! Panic-freedom under arbitrary 25-byte input is exhaustively
//! pinned by the proptest fuzz layer in the plain tests and (where
//! present) by the Kani harness pack.
//!
//! What Verus *does* prove:
//!   - **MAVLINK-V01**: `Heartbeat::encode_payload` writes exactly
//!     `HEARTBEAT_PAYLOAD_LEN` bytes; the type signature carries this.
//!   - **MAVLINK-V02**: `GlobalPositionInt::encode_payload` writes
//!     exactly `GLOBAL_POSITION_INT_PAYLOAD_LEN` bytes; same.
//!   - **MAVLINK-V03**: `decode_payload` on each message returns
//!     `None` on length mismatch (ensures contract: result is
//!     `Some` only when `payload.len() == EXPECTED`).
//!
//! Same pattern relay-nid landed at v0.11; relay-mavlink was the
//! last plain-only crate in the workspace.

#![no_std]
#![forbid(unsafe_code)]

pub mod command_long;
pub mod crc;
pub mod frame;
pub mod global_position_int;
pub mod heartbeat;

pub use command_long::{
    CommandLong, COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, COMMAND_LONG_PAYLOAD_LEN,
    MAV_CMD_DO_FLIGHTTERMINATION, MAV_CMD_NAV_RETURN_TO_LAUNCH,
};
pub use crc::Crc16X25;
pub use frame::{
    encode_frame, parse_frame, peek_message_id,
    CodecError, Frame, FrameHeader, HEADER_LEN, MAGIC_V1, MAGIC_V2, MAX_FRAME_SIZE,
};
pub use global_position_int::{
    GlobalPositionInt, GLOBAL_POSITION_INT_CRC_EXTRA, GLOBAL_POSITION_INT_MSG_ID,
    GLOBAL_POSITION_INT_PAYLOAD_LEN, HEADING_UNKNOWN,
};
pub use heartbeat::{
    FALCON_AUTOPILOT_ID, Heartbeat, MavAutopilot, MavModeFlag, MavState, MavType,
    HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID, HEARTBEAT_PAYLOAD_LEN,
};
