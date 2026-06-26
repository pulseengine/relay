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

pub mod command_long;
pub mod crc;
pub mod frame;
pub mod global_position_int;
pub mod heartbeat;
pub mod mission;
pub mod param;

pub use command_long::{
    COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, COMMAND_LONG_PAYLOAD_LEN, CommandLong,
    MAV_CMD_COMPONENT_ARM_DISARM, MAV_CMD_DO_FLIGHTTERMINATION, MAV_CMD_MISSION_START,
    MAV_CMD_NAV_LAND, MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_TAKEOFF,
};
pub use crc::Crc16X25;
pub use frame::{
    CodecError, Frame, FrameHeader, HEADER_LEN, MAGIC_V1, MAGIC_V2, MAX_FRAME_SIZE, encode_frame,
    parse_frame, peek_message_id,
};
pub use global_position_int::{
    GLOBAL_POSITION_INT_CRC_EXTRA, GLOBAL_POSITION_INT_MSG_ID, GLOBAL_POSITION_INT_PAYLOAD_LEN,
    GlobalPositionInt, HEADING_UNKNOWN,
};
pub use heartbeat::{
    FALCON_AUTOPILOT_ID, HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID, HEARTBEAT_PAYLOAD_LEN, Heartbeat,
    MavAutopilot, MavModeFlag, MavState, MavType,
};
pub use mission::{
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_MISSION_ACCEPTED, MAV_MISSION_ERROR,
    MAV_MISSION_TYPE_MISSION, MISSION_ACK_CRC_EXTRA, MISSION_ACK_MSG_ID, MISSION_ACK_PAYLOAD_LEN,
    MISSION_COUNT_CRC_EXTRA, MISSION_COUNT_MSG_ID, MISSION_COUNT_PAYLOAD_LEN,
    MISSION_ITEM_INT_CRC_EXTRA, MISSION_ITEM_INT_MSG_ID, MISSION_ITEM_INT_PAYLOAD_LEN,
    MISSION_REQUEST_INT_CRC_EXTRA, MISSION_REQUEST_INT_MSG_ID, MISSION_REQUEST_INT_PAYLOAD_LEN,
    MISSION_REQUEST_LIST_CRC_EXTRA, MISSION_REQUEST_LIST_MSG_ID, MISSION_REQUEST_LIST_PAYLOAD_LEN,
    MissionAck, MissionCount, MissionItemInt, MissionRequestInt, MissionRequestList,
};
pub use param::{
    MAV_PARAM_TYPE_REAL32, PARAM_REQUEST_LIST_CRC_EXTRA, PARAM_REQUEST_LIST_MSG_ID,
    PARAM_REQUEST_LIST_PAYLOAD_LEN, PARAM_REQUEST_READ_CRC_EXTRA, PARAM_REQUEST_READ_MSG_ID,
    PARAM_REQUEST_READ_PAYLOAD_LEN, PARAM_SET_CRC_EXTRA, PARAM_SET_MSG_ID, PARAM_SET_PAYLOAD_LEN,
    PARAM_VALUE_CRC_EXTRA, PARAM_VALUE_MSG_ID, PARAM_VALUE_PAYLOAD_LEN, ParamRequestList,
    ParamRequestRead, ParamSet, ParamValue,
};
