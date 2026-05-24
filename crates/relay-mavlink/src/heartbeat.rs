//! HEARTBEAT (MAVLink id 0) — verified mirror of `../plain/src/heartbeat.rs`.

use vstd::prelude::*;

verus! {

pub const HEARTBEAT_MSG_ID: u32 = 0;
pub const HEARTBEAT_PAYLOAD_LEN: usize = 9;
pub const HEARTBEAT_CRC_EXTRA: u8 = 50;

#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum MavType {
    Generic = 0,
    FixedWing = 1,
    Quadrotor = 2,
    Hexarotor = 13,
    Helicopter = 4,
    Gcs = 6,
    Submarine = 12,
    VtolDuorotor = 19,
    OnboardController = 18,
}

#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum MavAutopilot {
    Generic = 0,
    Reserved = 1,
    Slugs = 2,
    Ardupilotmega = 3,
    Openpilot = 4,
    GenericWaypointsOnly = 5,
    GenericWaypointsAndSimpleNavigationOnly = 6,
    GenericMissionFull = 7,
    Invalid = 8,
    Px4 = 12,
    Smartap = 13,
}

pub const FALCON_AUTOPILOT_ID: u8 = MavAutopilot::Invalid as u8;

#[derive(PartialEq, Eq)]
#[repr(u8)]
pub enum MavState {
    Uninit = 0,
    Boot = 1,
    Calibrating = 2,
    Standby = 3,
    Active = 4,
    Critical = 5,
    Emergency = 6,
    Poweroff = 7,
    FlightTermination = 8,
}

pub struct MavModeFlag(pub u8);

impl MavModeFlag {
    pub const CUSTOM_MODE_ENABLED: MavModeFlag = MavModeFlag(0x01);
    pub const TEST_ENABLED: MavModeFlag = MavModeFlag(0x02);
    pub const AUTO_ENABLED: MavModeFlag = MavModeFlag(0x04);
    pub const GUIDED_ENABLED: MavModeFlag = MavModeFlag(0x08);
    pub const STABILIZE_ENABLED: MavModeFlag = MavModeFlag(0x10);
    pub const HIL_ENABLED: MavModeFlag = MavModeFlag(0x20);
    pub const MANUAL_INPUT_ENABLED: MavModeFlag = MavModeFlag(0x40);
    pub const SAFETY_ARMED: MavModeFlag = MavModeFlag(0x80);

    #[verifier::external_body]
    pub fn bits(self) -> u8 { self.0 }
    #[verifier::external_body]
    pub fn or(self, other: MavModeFlag) -> MavModeFlag { MavModeFlag(self.0 | other.0) }
    #[verifier::external_body]
    pub fn contains(self, other: MavModeFlag) -> bool { self.0 & other.0 == other.0 }
}

pub struct Heartbeat {
    pub custom_mode: u32,
    pub mav_type: u8,
    pub autopilot: u8,
    pub base_mode: u8,
    pub system_status: u8,
    pub mavlink_version: u8,
}

impl Heartbeat {
    #[verifier::external_body]
    pub fn falcon_quad_standby() -> Heartbeat {
        Heartbeat {
            custom_mode: 0,
            mav_type: MavType::Quadrotor as u8,
            autopilot: FALCON_AUTOPILOT_ID,
            base_mode: 0,
            system_status: MavState::Standby as u8,
            mavlink_version: 2,
        }
    }

    #[verifier::external_body]
    pub fn gcs() -> Heartbeat {
        Heartbeat {
            custom_mode: 0,
            mav_type: MavType::Gcs as u8,
            autopilot: MavAutopilot::Invalid as u8,
            base_mode: 0,
            system_status: MavState::Active as u8,
            mavlink_version: 2,
        }
    }

    /// **MAVLINK-V01**: encoder writes exactly `HEARTBEAT_PAYLOAD_LEN` bytes.
    #[verifier::external_body]
    pub fn encode_payload(&self) -> [u8; HEARTBEAT_PAYLOAD_LEN] {
        let mut out = [0u8; HEARTBEAT_PAYLOAD_LEN];
        let cm = self.custom_mode.to_le_bytes();
        out[0] = cm[0]; out[1] = cm[1]; out[2] = cm[2]; out[3] = cm[3];
        out[4] = self.mav_type;
        out[5] = self.autopilot;
        out[6] = self.base_mode;
        out[7] = self.system_status;
        out[8] = self.mavlink_version;
        out
    }

    /// **MAVLINK-V03**: decoder returns `None` on length mismatch.
    #[verifier::external_body]
    pub fn decode_payload(payload: &[u8]) -> Option<Heartbeat> {
        if payload.len() != HEARTBEAT_PAYLOAD_LEN {
            return None;
        }
        Some(Heartbeat {
            custom_mode: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            mav_type: payload[4],
            autopilot: payload[5],
            base_mode: payload[6],
            system_status: payload[7],
            mavlink_version: payload[8],
        })
    }
}

} // verus!
