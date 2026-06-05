//! MAVLink mission-upload protocol messages (v1.33).
//!
//! The four messages a ground station + vehicle exchange to UPLOAD a mission
//! (GCS → vehicle), per <https://mavlink.io/en/services/mission.html>:
//!
//! ```text
//!   1. GCS  -> MISSION_COUNT(count)          "I have N items"
//!   2. veh  -> MISSION_REQUEST_INT(seq=0)    "send me item 0"
//!   3. GCS  -> MISSION_ITEM_INT(seq=0, ...)  the waypoint
//!   ...  (2-3 repeat for each seq)
//!   N. veh  -> MISSION_ACK(ACCEPTED)         "got them all"
//! ```
//!
//! Field order is MAVLink-canonical (sorted by size descending). CRC_EXTRA
//! values are from the MAVLink common.xml.
//!
//! Falcon decodes the uploaded `MISSION_ITEM_INT`s (lat/lon/alt) into NED
//! waypoints for the FlightSupervisor sequencer (v1.30); `MISSION_START`
//! (v1.30) then flies them.

// ── MISSION_COUNT (id 44) ────────────────────────────────────────────────

pub const MISSION_COUNT_MSG_ID: u32 = 44;
pub const MISSION_COUNT_PAYLOAD_LEN: usize = 4;
pub const MISSION_COUNT_CRC_EXTRA: u8 = 221;

/// MAV_MISSION_TYPE_MISSION — the main flight-plan mission.
pub const MAV_MISSION_TYPE_MISSION: u8 = 0;

/// MISSION_COUNT — the GCS announces how many items it will upload.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissionCount {
    pub count: u16,
    pub target_system: u8,
    pub target_component: u8,
}

impl MissionCount {
    pub fn encode_payload(&self) -> [u8; MISSION_COUNT_PAYLOAD_LEN] {
        let mut out = [0u8; MISSION_COUNT_PAYLOAD_LEN];
        out[0..2].copy_from_slice(&self.count.to_le_bytes());
        out[2] = self.target_system;
        out[3] = self.target_component;
        out
    }

    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        // Accept ≥ the base length: later MAVLink adds a mission_type extension
        // byte; falcon ignores it (treats every mission as the main mission).
        if payload.len() < MISSION_COUNT_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            count: u16::from_le_bytes([payload[0], payload[1]]),
            target_system: payload[2],
            target_component: payload[3],
        })
    }
}

// ── MISSION_REQUEST_LIST (id 43) ─────────────────────────────────────────

pub const MISSION_REQUEST_LIST_MSG_ID: u32 = 43;
pub const MISSION_REQUEST_LIST_PAYLOAD_LEN: usize = 2;
pub const MISSION_REQUEST_LIST_CRC_EXTRA: u8 = 132;

/// MISSION_REQUEST_LIST — the GCS asks the vehicle to DOWNLOAD its mission
/// (the vehicle replies with MISSION_COUNT, then serves each MISSION_ITEM_INT).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissionRequestList {
    pub target_system: u8,
    pub target_component: u8,
}

impl MissionRequestList {
    pub fn encode_payload(&self) -> [u8; MISSION_REQUEST_LIST_PAYLOAD_LEN] {
        [self.target_system, self.target_component]
    }

    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() < MISSION_REQUEST_LIST_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            target_system: payload[0],
            target_component: payload[1],
        })
    }
}

// ── MISSION_REQUEST_INT (id 51) ──────────────────────────────────────────

pub const MISSION_REQUEST_INT_MSG_ID: u32 = 51;
pub const MISSION_REQUEST_INT_PAYLOAD_LEN: usize = 4;
pub const MISSION_REQUEST_INT_CRC_EXTRA: u8 = 196;

/// MISSION_REQUEST_INT — the vehicle asks the GCS for item `seq`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissionRequestInt {
    pub seq: u16,
    pub target_system: u8,
    pub target_component: u8,
}

impl MissionRequestInt {
    pub fn encode_payload(&self) -> [u8; MISSION_REQUEST_INT_PAYLOAD_LEN] {
        let mut out = [0u8; MISSION_REQUEST_INT_PAYLOAD_LEN];
        out[0..2].copy_from_slice(&self.seq.to_le_bytes());
        out[2] = self.target_system;
        out[3] = self.target_component;
        out
    }

    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() < MISSION_REQUEST_INT_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            seq: u16::from_le_bytes([payload[0], payload[1]]),
            target_system: payload[2],
            target_component: payload[3],
        })
    }
}

// ── MISSION_ITEM_INT (id 73) ─────────────────────────────────────────────

pub const MISSION_ITEM_INT_MSG_ID: u32 = 73;
pub const MISSION_ITEM_INT_PAYLOAD_LEN: usize = 37;
pub const MISSION_ITEM_INT_CRC_EXTRA: u8 = 38;

/// MAV_FRAME_GLOBAL_RELATIVE_ALT_INT — x/y are lat/lon×1e7, z is metres
/// above the home altitude. The frame falcon expects for uploaded waypoints.
pub const MAV_FRAME_GLOBAL_RELATIVE_ALT_INT: u8 = 6;

/// MISSION_ITEM_INT — one uploaded waypoint. `x` = lat×1e7, `y` = lon×1e7,
/// `z` = altitude (m, per `frame`).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MissionItemInt {
    pub param1: f32,
    pub param2: f32,
    pub param3: f32,
    pub param4: f32,
    pub x: i32,
    pub y: i32,
    pub z: f32,
    pub seq: u16,
    pub command: u16,
    pub target_system: u8,
    pub target_component: u8,
    pub frame: u8,
    pub current: u8,
    pub autocontinue: u8,
}

impl MissionItemInt {
    pub fn encode_payload(&self) -> [u8; MISSION_ITEM_INT_PAYLOAD_LEN] {
        let mut out = [0u8; MISSION_ITEM_INT_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.param1.to_le_bytes());
        out[4..8].copy_from_slice(&self.param2.to_le_bytes());
        out[8..12].copy_from_slice(&self.param3.to_le_bytes());
        out[12..16].copy_from_slice(&self.param4.to_le_bytes());
        out[16..20].copy_from_slice(&self.x.to_le_bytes());
        out[20..24].copy_from_slice(&self.y.to_le_bytes());
        out[24..28].copy_from_slice(&self.z.to_le_bytes());
        out[28..30].copy_from_slice(&self.seq.to_le_bytes());
        out[30..32].copy_from_slice(&self.command.to_le_bytes());
        out[32] = self.target_system;
        out[33] = self.target_component;
        out[34] = self.frame;
        out[35] = self.current;
        out[36] = self.autocontinue;
        out
    }

    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() < MISSION_ITEM_INT_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            param1: f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            param2: f32::from_le_bytes([payload[4], payload[5], payload[6], payload[7]]),
            param3: f32::from_le_bytes([payload[8], payload[9], payload[10], payload[11]]),
            param4: f32::from_le_bytes([payload[12], payload[13], payload[14], payload[15]]),
            x: i32::from_le_bytes([payload[16], payload[17], payload[18], payload[19]]),
            y: i32::from_le_bytes([payload[20], payload[21], payload[22], payload[23]]),
            z: f32::from_le_bytes([payload[24], payload[25], payload[26], payload[27]]),
            seq: u16::from_le_bytes([payload[28], payload[29]]),
            command: u16::from_le_bytes([payload[30], payload[31]]),
            target_system: payload[32],
            target_component: payload[33],
            frame: payload[34],
            current: payload[35],
            autocontinue: payload[36],
        })
    }
}

// ── MISSION_ACK (id 47) ──────────────────────────────────────────────────

pub const MISSION_ACK_MSG_ID: u32 = 47;
pub const MISSION_ACK_PAYLOAD_LEN: usize = 3;
pub const MISSION_ACK_CRC_EXTRA: u8 = 153;

/// MAV_MISSION_ACCEPTED — the upload succeeded.
pub const MAV_MISSION_ACCEPTED: u8 = 0;
/// MAV_MISSION_ERROR — a generic upload failure.
pub const MAV_MISSION_ERROR: u8 = 1;

/// MISSION_ACK — the vehicle confirms the upload result.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MissionAck {
    pub target_system: u8,
    pub target_component: u8,
    pub mav_type: u8,
}

impl MissionAck {
    pub fn encode_payload(&self) -> [u8; MISSION_ACK_PAYLOAD_LEN] {
        [self.target_system, self.target_component, self.mav_type]
    }

    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() < MISSION_ACK_PAYLOAD_LEN {
            return None;
        }
        Some(Self {
            target_system: payload[0],
            target_component: payload[1],
            mav_type: payload[2],
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ids_and_crc_extras() {
        assert_eq!(MISSION_COUNT_MSG_ID, 44);
        assert_eq!(MISSION_COUNT_CRC_EXTRA, 221);
        assert_eq!(MISSION_REQUEST_INT_MSG_ID, 51);
        assert_eq!(MISSION_REQUEST_INT_CRC_EXTRA, 196);
        assert_eq!(MISSION_ITEM_INT_MSG_ID, 73);
        assert_eq!(MISSION_ITEM_INT_CRC_EXTRA, 38);
        assert_eq!(MISSION_ACK_MSG_ID, 47);
        assert_eq!(MISSION_ACK_CRC_EXTRA, 153);
    }

    #[test]
    fn mission_count_round_trip() {
        let c = MissionCount {
            count: 5,
            target_system: 1,
            target_component: 1,
        };
        assert_eq!(MissionCount::decode_payload(&c.encode_payload()), Some(c));
    }

    #[test]
    fn mission_request_int_round_trip() {
        let r = MissionRequestInt {
            seq: 3,
            target_system: 1,
            target_component: 1,
        };
        assert_eq!(
            MissionRequestInt::decode_payload(&r.encode_payload()),
            Some(r)
        );
    }

    #[test]
    fn mission_request_list_round_trip() {
        assert_eq!(MISSION_REQUEST_LIST_MSG_ID, 43);
        assert_eq!(MISSION_REQUEST_LIST_CRC_EXTRA, 132);
        let r = MissionRequestList {
            target_system: 1,
            target_component: 1,
        };
        assert_eq!(
            MissionRequestList::decode_payload(&r.encode_payload()),
            Some(r)
        );
    }

    #[test]
    fn mission_item_int_round_trip() {
        let it = MissionItemInt {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 473_977_123,
            y: 85_456_456,
            z: 12.5,
            seq: 2,
            command: 16,
            target_system: 1,
            target_component: 1,
            frame: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            current: 0,
            autocontinue: 1,
        };
        let d = MissionItemInt::decode_payload(&it.encode_payload()).expect("decode");
        assert_eq!(d.x, it.x);
        assert_eq!(d.y, it.y);
        assert_eq!(d.z.to_bits(), it.z.to_bits());
        assert_eq!(d.seq, it.seq);
        assert_eq!(d.frame, it.frame);
    }

    #[test]
    fn mission_ack_round_trip() {
        let a = MissionAck {
            target_system: 1,
            target_component: 1,
            mav_type: MAV_MISSION_ACCEPTED,
        };
        assert_eq!(MissionAck::decode_payload(&a.encode_payload()), Some(a));
    }

    #[test]
    fn item_offsets_match_spec() {
        // seq is at offset 28 (after 7×4-byte fields), command at 30.
        let it = MissionItemInt {
            param1: 0.0,
            param2: 0.0,
            param3: 0.0,
            param4: 0.0,
            x: 0,
            y: 0,
            z: 0.0,
            seq: 0xBEEF,
            command: 0xAABB,
            target_system: 0xCC,
            target_component: 0xDD,
            frame: 0xEE,
            current: 0x11,
            autocontinue: 0x22,
        };
        let b = it.encode_payload();
        assert_eq!(&b[28..30], &[0xEF, 0xBE]);
        assert_eq!(&b[30..32], &[0xBB, 0xAA]);
        assert_eq!(b[32], 0xCC);
        assert_eq!(b[36], 0x22);
    }
}
