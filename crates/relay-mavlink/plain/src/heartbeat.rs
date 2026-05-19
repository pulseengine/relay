//! HEARTBEAT message — MAVLink id 0.
//!
//! Sent at 1 Hz by every MAVLink node. Causes the receiving ground
//! control station (QGroundControl, MAVSDK, Auterion Mission Control)
//! to add the vehicle to its list and start tracking it.
//!
//! Payload (9 bytes, little-endian):
//!
//!   offset 0..4   custom_mode      u32
//!   offset 4      type             u8   (MAV_TYPE)
//!   offset 5      autopilot        u8   (MAV_AUTOPILOT)
//!   offset 6      base_mode        u8   (MAV_MODE_FLAG bitfield)
//!   offset 7      system_status    u8   (MAV_STATE)
//!   offset 8      mavlink_version  u8
//!
//! The field order on the wire is the MAVLink-canonical "sorted by
//! size descending" order — `custom_mode` (u32) first, then five
//! u8 fields. This matches the MAVLink spec.
//!
//! CRC_EXTRA for HEARTBEAT: 50 (constant from the message definition).

/// MAVLink message ID for HEARTBEAT.
pub const HEARTBEAT_MSG_ID: u32 = 0;

/// HEARTBEAT payload length in bytes.
pub const HEARTBEAT_PAYLOAD_LEN: usize = 9;

/// HEARTBEAT CRC_EXTRA byte (from MAVLink XML).
pub const HEARTBEAT_CRC_EXTRA: u8 = 50;

/// MAV_TYPE enum — vehicle class.
/// We expose the values that matter for falcon-quad / falcon-hex /
/// falcon-coax. Other types live in the spec; we don't enumerate
/// every entry here.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum MavType {
    /// Generic.
    Generic = 0,
    /// Fixed-wing aircraft.
    FixedWing = 1,
    /// Quadrotor (used for falcon-quad).
    Quadrotor = 2,
    /// Hexarotor (used for falcon-hex).
    Hexarotor = 13,
    /// Helicopter (used for falcon-coax, Ingenuity-class).
    Helicopter = 4,
    /// Ground control station.
    Gcs = 6,
    /// Submarine.
    Submarine = 12,
    /// VTOL Duorotor (one of the VTOL variants).
    VtolDuorotor = 19,
    /// Onboard companion controller.
    OnboardController = 18,
}

/// MAV_AUTOPILOT enum — autopilot identity.
/// falcon claims 12 = MAV_AUTOPILOT_INVALID in v0.1 because we are
/// not (yet) registered. The MAVLink consortium maintains the
/// canonical enum; future falcon release coordinates a real value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

/// Discriminant falcon uses for its autopilot id today. Until a
/// dedicated MAV_AUTOPILOT value is registered upstream, we claim
/// MAV_AUTOPILOT_INVALID (8). When the falcon entry is accepted
/// into the MAVLink XML, change this and re-run the GCS interop
/// tests.
pub const FALCON_AUTOPILOT_ID: u8 = MavAutopilot::Invalid as u8;

/// MAV_STATE enum — system status.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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

/// MAV_MODE_FLAG bitfield — base-mode flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MavModeFlag(pub u8);

impl MavModeFlag {
    pub const CUSTOM_MODE_ENABLED: Self = Self(0x01);
    pub const TEST_ENABLED: Self = Self(0x02);
    pub const AUTO_ENABLED: Self = Self(0x04);
    pub const GUIDED_ENABLED: Self = Self(0x08);
    pub const STABILIZE_ENABLED: Self = Self(0x10);
    pub const HIL_ENABLED: Self = Self(0x20);
    pub const MANUAL_INPUT_ENABLED: Self = Self(0x40);
    pub const SAFETY_ARMED: Self = Self(0x80);

    pub const fn bits(self) -> u8 {
        self.0
    }
    pub const fn or(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
    pub const fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }
}

/// The HEARTBEAT message struct.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Heartbeat {
    pub custom_mode: u32,
    pub mav_type: u8,
    pub autopilot: u8,
    pub base_mode: u8,
    pub system_status: u8,
    pub mavlink_version: u8,
}

impl Heartbeat {
    /// A reasonable default for a falcon-quad sending its first
    /// heartbeat: quadrotor, falcon autopilot (claiming INVALID
    /// until registered), standby, MAVLink v2.
    pub const fn falcon_quad_standby() -> Self {
        Self {
            custom_mode: 0,
            mav_type: MavType::Quadrotor as u8,
            autopilot: FALCON_AUTOPILOT_ID,
            base_mode: 0,
            system_status: MavState::Standby as u8,
            mavlink_version: 2,
        }
    }

    /// A reasonable default GCS heartbeat (e.g., what falcon-hello
    /// emits when running in --mode gcs).
    pub const fn gcs() -> Self {
        Self {
            custom_mode: 0,
            mav_type: MavType::Gcs as u8,
            autopilot: MavAutopilot::Invalid as u8,
            base_mode: 0,
            system_status: MavState::Active as u8,
            mavlink_version: 2,
        }
    }

    /// Encode the heartbeat into its 9-byte payload form.
    /// All fields little-endian per MAVLink.
    pub fn encode_payload(&self) -> [u8; HEARTBEAT_PAYLOAD_LEN] {
        let mut out = [0u8; HEARTBEAT_PAYLOAD_LEN];
        let cm = self.custom_mode.to_le_bytes();
        out[0] = cm[0];
        out[1] = cm[1];
        out[2] = cm[2];
        out[3] = cm[3];
        out[4] = self.mav_type;
        out[5] = self.autopilot;
        out[6] = self.base_mode;
        out[7] = self.system_status;
        out[8] = self.mavlink_version;
        out
    }

    /// Decode from a 9-byte payload slice. Returns None if length
    /// is wrong (the caller has already validated message-id and
    /// frame integrity at this point).
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != HEARTBEAT_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            custom_mode: u32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            mav_type: payload[4],
            autopilot: payload[5],
            base_mode: payload[6],
            system_status: payload[7],
            mavlink_version: payload[8],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn payload_length_constant() {
        let hb = Heartbeat::falcon_quad_standby();
        assert_eq!(hb.encode_payload().len(), HEARTBEAT_PAYLOAD_LEN);
    }

    #[test]
    fn crc_extra_is_fifty() {
        // This is a fixed-by-spec constant from the MAVLink XML.
        // If this fails, somebody edited the wrong number.
        assert_eq!(HEARTBEAT_CRC_EXTRA, 50);
    }

    #[test]
    fn round_trip_zero() {
        let original = Heartbeat {
            custom_mode: 0,
            mav_type: 0,
            autopilot: 0,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 0,
        };
        let bytes = original.encode_payload();
        let decoded = Heartbeat::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn round_trip_falcon_default() {
        let original = Heartbeat::falcon_quad_standby();
        let bytes = original.encode_payload();
        let decoded = Heartbeat::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn round_trip_gcs_default() {
        let original = Heartbeat::gcs();
        let bytes = original.encode_payload();
        let decoded = Heartbeat::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn round_trip_max_values() {
        let original = Heartbeat {
            custom_mode: u32::MAX,
            mav_type: u8::MAX,
            autopilot: u8::MAX,
            base_mode: u8::MAX,
            system_status: u8::MAX,
            mavlink_version: u8::MAX,
        };
        let bytes = original.encode_payload();
        let decoded = Heartbeat::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(Heartbeat::decode_payload(&[0u8; 8]).is_none());
    }

    #[test]
    fn decode_rejects_long_payload() {
        assert!(Heartbeat::decode_payload(&[0u8; 10]).is_none());
    }

    #[test]
    fn decode_empty_payload() {
        assert!(Heartbeat::decode_payload(&[]).is_none());
    }

    #[test]
    fn custom_mode_little_endian() {
        // Make sure encoding really is little-endian as MAVLink requires.
        let hb = Heartbeat {
            custom_mode: 0x12345678,
            mav_type: 0,
            autopilot: 0,
            base_mode: 0,
            system_status: 0,
            mavlink_version: 0,
        };
        let bytes = hb.encode_payload();
        // 0x12345678 LE = 78 56 34 12
        assert_eq!(&bytes[0..4], &[0x78, 0x56, 0x34, 0x12]);
    }

    #[test]
    fn field_offsets_match_spec() {
        // MAVLink-spec field order: custom_mode(u32), type, autopilot,
        // base_mode, system_status, mavlink_version.
        let hb = Heartbeat {
            custom_mode: 0,
            mav_type: 0xAA,
            autopilot: 0xBB,
            base_mode: 0xCC,
            system_status: 0xDD,
            mavlink_version: 0xEE,
        };
        let bytes = hb.encode_payload();
        assert_eq!(bytes[4], 0xAA);
        assert_eq!(bytes[5], 0xBB);
        assert_eq!(bytes[6], 0xCC);
        assert_eq!(bytes[7], 0xDD);
        assert_eq!(bytes[8], 0xEE);
    }

    #[test]
    fn mav_mode_flag_or_and_contains() {
        let combined = MavModeFlag::SAFETY_ARMED.or(MavModeFlag::GUIDED_ENABLED);
        assert!(combined.contains(MavModeFlag::SAFETY_ARMED));
        assert!(combined.contains(MavModeFlag::GUIDED_ENABLED));
        assert!(!combined.contains(MavModeFlag::HIL_ENABLED));
    }

    use proptest::prelude::*;

    proptest! {
        #[test]
        fn round_trip_arbitrary(
            custom_mode in any::<u32>(),
            mav_type in any::<u8>(),
            autopilot in any::<u8>(),
            base_mode in any::<u8>(),
            system_status in any::<u8>(),
            mavlink_version in any::<u8>(),
        ) {
            let original = Heartbeat {
                custom_mode, mav_type, autopilot,
                base_mode, system_status, mavlink_version,
            };
            let bytes = original.encode_payload();
            let decoded = Heartbeat::decode_payload(&bytes)
                .expect("decode arbitrary heartbeat");
            prop_assert_eq!(original, decoded);
        }
    }
}
