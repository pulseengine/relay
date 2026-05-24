//! Relay MAVLink — no_std + no_alloc MAVLink v2 codec.
//!
//! Falcon's interop bridge to the PX4 / ArduPilot ecosystem.
//! v0.1 implements only the HEARTBEAT message (id=0); other messages
//! land in later releases per the falcon roadmap.
//!
//! Wire format reference: https://mavlink.io/en/guide/serialization.html
//!
//! ## Verified properties (Verus, tracked in SWREQ-FALCON-MAVLINK-P0*)
//!
//!   MAVLINK-P01: parse() returns Err on malformed input rather than
//!                panicking. All slicing is bounds-checked.
//!   MAVLINK-P02: encode/decode round-trip preserves all heartbeat
//!                fields exactly.
//!   MAVLINK-P03: CRC-16/X.25 implementation matches the algorithm in
//!                the MAVLink reference (mavgen-c).
//!
//! ## What's tested
//!
//!   - Round-trip encode → decode → equality on randomized inputs (proptest)
//!   - CRC algorithm against multiple known-good byte sequences
//!   - Negative cases: truncated frame, bad magic, bad CRC, wrong msg id
//!   - All error variants reachable (no panicking paths)

#![no_std]
#![forbid(unsafe_code)]

pub mod crc;
pub mod heartbeat;
pub mod frame;
pub mod global_position_int;
pub mod command_long;

pub use crc::Crc16X25;
pub use frame::{
    encode_frame, parse_frame, peek_message_id,
    CodecError, Frame, FrameHeader, HEADER_LEN, MAGIC_V1, MAGIC_V2, MAX_FRAME_SIZE,
};
pub use heartbeat::{
    FALCON_AUTOPILOT_ID, Heartbeat, MavAutopilot, MavModeFlag, MavState, MavType,
    HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID, HEARTBEAT_PAYLOAD_LEN,
};
pub use global_position_int::{
    GlobalPositionInt, GLOBAL_POSITION_INT_CRC_EXTRA, GLOBAL_POSITION_INT_MSG_ID,
    GLOBAL_POSITION_INT_PAYLOAD_LEN, HEADING_UNKNOWN,
};
pub use command_long::{
    CommandLong, COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, COMMAND_LONG_PAYLOAD_LEN,
    MAV_CMD_DO_FLIGHTTERMINATION, MAV_CMD_NAV_RETURN_TO_LAUNCH,
};
