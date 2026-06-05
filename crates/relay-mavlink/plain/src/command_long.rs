//! COMMAND_LONG message — MAVLink id 76.
//!
//! Generic command-with-parameters envelope; the workhorse for
//! everything from `MAV_CMD_NAV_RETURN_TO_LAUNCH` to autotune
//! requests. Falcon uses it to deliver the relay-sc-dispatched
//! safety commands back to the flight controller so the FC's nav
//! stack can act on them (v0.14.2: closes the round-trip the
//! v0.14.0 demo stopped one step short of — falcon's RTL command
//! now drives PX4 actively home, not just lands in falcon's own
//! command store).
//!
//! Payload (33 bytes, little-endian, MAVLink-canonical
//! sorted-by-size descending):
//!
//!   offset  0..4    param1            f32
//!   offset  4..8    param2            f32
//!   offset  8..12   param3            f32
//!   offset 12..16   param4            f32
//!   offset 16..20   param5            f32   (x — lat * 1e7 for nav cmds)
//!   offset 20..24   param6            f32   (y — lon * 1e7 for nav cmds)
//!   offset 24..28   param7            f32   (z — alt m for nav cmds)
//!   offset 28..30   command           u16   (MAV_CMD_*)
//!   offset 30       target_system     u8
//!   offset 31       target_component  u8
//!   offset 32       confirmation      u8
//!
//! CRC_EXTRA for COMMAND_LONG: **152** (from MAVLink XML).

/// MAVLink message ID for COMMAND_LONG.
pub const COMMAND_LONG_MSG_ID: u32 = 76;

/// COMMAND_LONG payload length.
pub const COMMAND_LONG_PAYLOAD_LEN: usize = 33;

/// COMMAND_LONG CRC_EXTRA (from MAVLink XML).
pub const COMMAND_LONG_CRC_EXTRA: u8 = 152;

// MAV_CMD enum — only the values falcon actually issues today.
// Full list: https://mavlink.io/en/messages/common.html#MAV_CMD

/// `MAV_CMD_NAV_RETURN_TO_LAUNCH` — fly home, then land.
/// Issued by falcon when relay-sc dispatches RTL command code
/// 0xA17C (the same command code the SITL + StubBench use).
pub const MAV_CMD_NAV_RETURN_TO_LAUNCH: u16 = 20;

/// `MAV_CMD_NAV_LAND` — land at the current (or specified) location.
pub const MAV_CMD_NAV_LAND: u16 = 21;

/// `MAV_CMD_NAV_TAKEOFF` — take off and climb to `param7` altitude.
pub const MAV_CMD_NAV_TAKEOFF: u16 = 22;

/// `MAV_CMD_COMPONENT_ARM_DISARM` — arm (`param1 >= 0.5`) or
/// disarm (`param1 < 0.5`) the vehicle. This is the command QGC /
/// MAVSDK send when the operator hits the Arm / Disarm control.
pub const MAV_CMD_COMPONENT_ARM_DISARM: u16 = 400;

/// `MAV_CMD_MISSION_START` — begin flying the uploaded mission (the GCS
/// "Start Mission" control). Falcon maps it to the Mission FSM event.
pub const MAV_CMD_MISSION_START: u16 = 300;

/// `MAV_CMD_DO_FLIGHTTERMINATION` — terminate flight (last-resort
/// safety command; relay-sc has a slot for it but falcon does not
/// dispatch it today).
pub const MAV_CMD_DO_FLIGHTTERMINATION: u16 = 185;

/// COMMAND_LONG decoded form.
#[derive(Clone, Copy, Debug, PartialEq)]
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
    /// All seven param slots are zero per the spec for this command.
    pub const fn rtl(target_system: u8, target_component: u8) -> Self {
        Self {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MAV_CMD_NAV_RETURN_TO_LAUNCH,
            target_system,
            target_component,
            confirmation: 0,
        }
    }

    /// Build a `MAV_CMD_COMPONENT_ARM_DISARM`. `arm = true` sets
    /// `param1 = 1.0` (arm); `arm = false` sets `param1 = 0.0` (disarm).
    pub const fn arm_disarm(target_system: u8, target_component: u8, arm: bool) -> Self {
        Self {
            param1: if arm { 1.0 } else { 0.0 },
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MAV_CMD_COMPONENT_ARM_DISARM,
            target_system,
            target_component,
            confirmation: 0,
        }
    }

    /// Build a `MAV_CMD_NAV_TAKEOFF` to climb to `alt_m` (param7).
    pub const fn takeoff(target_system: u8, target_component: u8, alt_m: f32) -> Self {
        Self {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: alt_m,
            command: MAV_CMD_NAV_TAKEOFF,
            target_system,
            target_component,
            confirmation: 0,
        }
    }

    /// Build a `MAV_CMD_NAV_LAND` (land in place).
    pub const fn land(target_system: u8, target_component: u8) -> Self {
        Self {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MAV_CMD_NAV_LAND,
            target_system,
            target_component,
            confirmation: 0,
        }
    }

    /// Build a `MAV_CMD_MISSION_START` (begin the uploaded mission).
    pub const fn mission_start(target_system: u8, target_component: u8) -> Self {
        Self {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: MAV_CMD_MISSION_START,
            target_system,
            target_component,
            confirmation: 0,
        }
    }

    /// Encode into the 33-byte payload.
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

    /// Decode from a 33-byte payload. Returns `None` on length mismatch.
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != COMMAND_LONG_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn msg_id_is_76() {
        assert_eq!(COMMAND_LONG_MSG_ID, 76);
    }
    #[test]
    fn payload_length_is_33() {
        assert_eq!(COMMAND_LONG_PAYLOAD_LEN, 33);
    }
    #[test]
    fn crc_extra_is_152() {
        // Fixed by spec; if this fails, somebody edited the MAVLink XML constant.
        assert_eq!(COMMAND_LONG_CRC_EXTRA, 152);
    }
    #[test]
    fn rtl_command_id_is_20() {
        assert_eq!(MAV_CMD_NAV_RETURN_TO_LAUNCH, 20);
    }

    #[test]
    fn rtl_builder_has_zero_params() {
        let c = CommandLong::rtl(1, 1);
        assert_eq!(c.param1, 0.0);
        assert_eq!(c.param7, 0.0);
        assert_eq!(c.command, MAV_CMD_NAV_RETURN_TO_LAUNCH);
        assert_eq!(c.target_system, 1);
        assert_eq!(c.target_component, 1);
        assert_eq!(c.confirmation, 0);
    }

    #[test]
    fn round_trip_rtl() {
        let original = CommandLong::rtl(1, 1);
        let bytes = original.encode_payload();
        let decoded = CommandLong::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn round_trip_with_params() {
        let original = CommandLong {
            param1: 1.5,
            param2: -2.5,
            param3: 3.0,
            param4: 12.34,
            param5: -56.78,
            param6: 90.12,
            param7: 100.0,
            command: 123,
            target_system: 7,
            target_component: 13,
            confirmation: 2,
        };
        let bytes = original.encode_payload();
        let decoded = CommandLong::decode_payload(&bytes).expect("decode");
        assert_eq!(original, decoded);
    }

    #[test]
    fn decode_rejects_short_payload() {
        assert!(CommandLong::decode_payload(&[0u8; 32]).is_none());
    }

    #[test]
    fn decode_rejects_long_payload() {
        assert!(CommandLong::decode_payload(&[0u8; 34]).is_none());
    }

    #[test]
    fn field_offsets_match_spec() {
        // Sorted-by-size descending: 7×f32 (28 bytes) + u16 + 3×u8.
        let c = CommandLong {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            param5: 0.0,
            param6: 0.0,
            param7: 0.0,
            command: 0xAABB,
            target_system: 0xCC,
            target_component: 0xDD,
            confirmation: 0xEE,
        };
        let b = c.encode_payload();
        assert_eq!(&b[28..30], &[0xBB, 0xAA]);
        assert_eq!(b[30], 0xCC);
        assert_eq!(b[31], 0xDD);
        assert_eq!(b[32], 0xEE);
    }

    use proptest::prelude::*;
    proptest! {
        #[test]
        fn round_trip_arbitrary(
            p1 in any::<u32>(), p2 in any::<u32>(), p3 in any::<u32>(),
            p4 in any::<u32>(), p5 in any::<u32>(), p6 in any::<u32>(),
            p7 in any::<u32>(),
            command in any::<u16>(),
            tsys in any::<u8>(), tcomp in any::<u8>(), conf in any::<u8>(),
        ) {
            // Use bit-pattern f32s so NaN payloads also round-trip.
            let original = CommandLong {
                param1: f32::from_bits(p1),
                param2: f32::from_bits(p2),
                param3: f32::from_bits(p3),
                param4: f32::from_bits(p4),
                param5: f32::from_bits(p5),
                param6: f32::from_bits(p6),
                param7: f32::from_bits(p7),
                command, target_system: tsys, target_component: tcomp, confirmation: conf,
            };
            let bytes = original.encode_payload();
            let decoded = CommandLong::decode_payload(&bytes).expect("decode arbitrary");
            // Compare bit patterns (NaN != NaN under PartialEq).
            prop_assert_eq!(original.param1.to_bits(), decoded.param1.to_bits());
            prop_assert_eq!(original.param2.to_bits(), decoded.param2.to_bits());
            prop_assert_eq!(original.command, decoded.command);
            prop_assert_eq!(original.target_system, decoded.target_system);
        }
    }
}
