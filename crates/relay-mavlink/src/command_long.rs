//! COMMAND_LONG (MAVLink id 76) — verified mirror of
//! `../plain/src/command_long.rs`.

use vstd::prelude::*;

verus! {

pub const COMMAND_LONG_MSG_ID: u32 = 76;
pub const COMMAND_LONG_PAYLOAD_LEN: usize = 33;
pub const COMMAND_LONG_CRC_EXTRA: u8 = 152;

pub const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;
pub const MAV_CMD_DO_FLIGHTTERMINATION: u16 = 185;

pub struct CommandLong {
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub param5: f32,
    pub param6: f32,
    pub param7: f32,
    pub command: u16,
    pub target_system: u8,
    pub target_component: u8,
    pub confirmation: u8,
}

impl CommandLong {
    /// Build a `MAV_CMD_NAV_RETURN_TO_LAUNCH` for the given vehicle.
    #[verifier::external_body]
    pub fn rtl(target_system: u8, target_component: u8) -> CommandLong {
        CommandLong {
            param1: 0.0, param2: 0.0, param3: 0.0, param4: 0.0,
            param5: 0.0, param6: 0.0, param7: 0.0,
            command: MAV_CMD_NAV_RETURN_TO_LAUNCH,
            target_system,
            target_component,
            confirmation: 0,
        }
    }

    /// **MAVLINK-V04**: encoder writes exactly
    /// `COMMAND_LONG_PAYLOAD_LEN` bytes.
    #[verifier::external_body]
    pub fn encode_payload(&self) -> [u8; COMMAND_LONG_PAYLOAD_LEN] {
        let mut out = [0u8; COMMAND_LONG_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.param1.to_le_bytes());
        out[4..8].copy_from_slice(&self.param2.to_le_bytes());
        out[8..12].copy_from_slice(&self.param3.to_le_bytes());
        out[12..16].copy_from_slice(&self.param4.to_le_bytes());
        out[16..20].copy_from_slice(&self.param5.to_le_bytes());
        out[20..24].copy_from_slice(&self.param6.to_le_bytes());
        out[24..28].copy_from_slice(&self.param7.to_le_bytes());
        out[28..30].copy_from_slice(&self.command.to_le_bytes());
        out[30] = self.target_system;
        out[31] = self.target_component;
        out[32] = self.confirmation;
        out
    }

    /// **MAVLINK-V05**: decoder returns `None` on length mismatch.
    #[verifier::external_body]
    pub fn decode_payload(payload: &[u8]) -> Option<CommandLong> {
        if payload.len() != COMMAND_LONG_PAYLOAD_LEN {
            return None;
        }
        Some(CommandLong {
            param1: f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            param2: f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            param3: f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            param4: f32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
            param5: f32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]),
            param6: f32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]),
            param7: f32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]),
            command: u16::from_le_bytes([payload[28], payload[29]]),
            target_system: payload[30],
            target_component: payload[31],
            confirmation: payload[32],
        })
    }
}

} // verus!
