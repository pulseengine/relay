//! MAVLink parameter-protocol messages — the wire framing for falcon's
//! PX4-style parameter system (FEAT-FALCON-v1.96, the keystone gap).
//!
//! Four messages drive the GCS ⇄ vehicle parameter exchange (QGroundControl /
//! MAVSDK / PX4 / ArduPilot all speak this):
//!
//!   PARAM_REQUEST_LIST (21) — GCS: "dump every parameter"
//!   PARAM_REQUEST_READ (20) — GCS: "read this one" (by id, or by index)
//!   PARAM_SET          (23) — GCS: "write this value"
//!   PARAM_VALUE        (22) — vehicle: "here is the value" (read reply / list
//!                             item / write ack)
//!
//! These are the pure wire codecs; the typed, BOUNDED store they drive lives in
//! `relay-param` (a write outside a parameter's declared [min,max] is rejected,
//! Kani PARAM-K01), and the seam that wires the two together is `falcon-param`.
//!
//! ## Wire order — MAVLink "sorted by size descending"
//!
//! MAVLink serializes fields largest-base-type-first (NOT declaration order), so
//! e.g. PARAM_VALUE is `param_value(f32) || param_count(u16) || param_index(u16)
//! || param_id(char[16]) || param_type(u8)`. The exact byte layout + each
//! message's CRC_EXTRA are pinned against **pymavlink reference vectors** in the
//! tests below (the external oracle — a self-referential round-trip cannot prove
//! conformance, the lesson the DroneCAN int14 bit-order bug taught). Regenerate
//! the vectors with `scripts/gen-mavlink-param-vectors.py`.

/// A 16-byte MAVLink parameter id (NUL-padded), as on the wire. Identical to
/// `relay_param::ParamId`; duplicated here to keep relay-mavlink dependency-free.
pub type ParamId = [u8; 16];

/// MAV_PARAM_TYPE_REAL32 — the IEEE-754 f32 parameter encoding falcon uses.
pub const MAV_PARAM_TYPE_REAL32: u8 = 9;

// ───────────────────────── PARAM_REQUEST_LIST (21) ─────────────────────────

/// MAVLink message ID for PARAM_REQUEST_LIST.
pub const PARAM_REQUEST_LIST_MSG_ID: u32 = 21;
/// PARAM_REQUEST_LIST payload length.
pub const PARAM_REQUEST_LIST_PAYLOAD_LEN: usize = 2;
/// PARAM_REQUEST_LIST CRC_EXTRA (from MAVLink XML).
pub const PARAM_REQUEST_LIST_CRC_EXTRA: u8 = 159;

/// PARAM_REQUEST_LIST: dump every parameter. Wire: target_system || target_component.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParamRequestList {
    pub target_system: u8,
    pub target_component: u8,
}

impl ParamRequestList {
    /// Encode into the 2-byte payload.
    pub fn encode_payload(&self) -> [u8; PARAM_REQUEST_LIST_PAYLOAD_LEN] {
        [self.target_system, self.target_component]
    }

    /// Decode from a 2-byte payload. `None` on length mismatch.
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != PARAM_REQUEST_LIST_PAYLOAD_LEN {
            return None;
        }
        Some(Self { target_system: payload[0], target_component: payload[1] })
    }
}

// ───────────────────────── PARAM_REQUEST_READ (20) ─────────────────────────

/// MAVLink message ID for PARAM_REQUEST_READ.
pub const PARAM_REQUEST_READ_MSG_ID: u32 = 20;
/// PARAM_REQUEST_READ payload length.
pub const PARAM_REQUEST_READ_PAYLOAD_LEN: usize = 20;
/// PARAM_REQUEST_READ CRC_EXTRA (from MAVLink XML).
pub const PARAM_REQUEST_READ_CRC_EXTRA: u8 = 214;

/// PARAM_REQUEST_READ: read one parameter. When `param_index >= 0` the GCS reads
/// by index; when `param_index == -1` it reads by `param_id`. Wire:
/// param_index(i16) || target_system || target_component || param_id(char[16]).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ParamRequestRead {
    pub param_index: i16,
    pub target_system: u8,
    pub target_component: u8,
    pub param_id: ParamId,
}

impl ParamRequestRead {
    /// Encode into the 20-byte payload.
    pub fn encode_payload(&self) -> [u8; PARAM_REQUEST_READ_PAYLOAD_LEN] {
        let mut out = [0u8; PARAM_REQUEST_READ_PAYLOAD_LEN];
        out[0..2].copy_from_slice(&self.param_index.to_le_bytes());
        out[2] = self.target_system;
        out[3] = self.target_component;
        out[4..20].copy_from_slice(&self.param_id);
        out
    }

    /// Decode from a 20-byte payload. `None` on length mismatch.
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != PARAM_REQUEST_READ_PAYLOAD_LEN {
            return None;
        }
        let mut param_id = [0u8; 16];
        param_id.copy_from_slice(&payload[4..20]);
        Some(Self {
            param_index: i16::from_le_bytes([payload[0], payload[1]]),
            target_system: payload[2],
            target_component: payload[3],
            param_id,
        })
    }
}

// ───────────────────────────── PARAM_SET (23) ──────────────────────────────

/// MAVLink message ID for PARAM_SET.
pub const PARAM_SET_MSG_ID: u32 = 23;
/// PARAM_SET payload length.
pub const PARAM_SET_PAYLOAD_LEN: usize = 23;
/// PARAM_SET CRC_EXTRA (from MAVLink XML).
pub const PARAM_SET_CRC_EXTRA: u8 = 168;

/// PARAM_SET: write a parameter value. Wire: param_value(f32) || target_system ||
/// target_component || param_id(char[16]) || param_type(u8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamSet {
    pub param_value: f32,
    pub target_system: u8,
    pub target_component: u8,
    pub param_id: ParamId,
    pub param_type: u8,
}

impl ParamSet {
    /// Encode into the 23-byte payload.
    pub fn encode_payload(&self) -> [u8; PARAM_SET_PAYLOAD_LEN] {
        let mut out = [0u8; PARAM_SET_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.param_value.to_le_bytes());
        out[4] = self.target_system;
        out[5] = self.target_component;
        out[6..22].copy_from_slice(&self.param_id);
        out[22] = self.param_type;
        out
    }

    /// Decode from a 23-byte payload. `None` on length mismatch.
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != PARAM_SET_PAYLOAD_LEN {
            return None;
        }
        let mut param_id = [0u8; 16];
        param_id.copy_from_slice(&payload[6..22]);
        Some(Self {
            param_value: f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            target_system: payload[4],
            target_component: payload[5],
            param_id,
            param_type: payload[22],
        })
    }
}

// ──────────────────────────── PARAM_VALUE (22) ─────────────────────────────

/// MAVLink message ID for PARAM_VALUE.
pub const PARAM_VALUE_MSG_ID: u32 = 22;
/// PARAM_VALUE payload length.
pub const PARAM_VALUE_PAYLOAD_LEN: usize = 25;
/// PARAM_VALUE CRC_EXTRA (from MAVLink XML).
pub const PARAM_VALUE_CRC_EXTRA: u8 = 220;

/// PARAM_VALUE: the vehicle's reply — a read result, a list item, or a write ack.
/// `param_index` / `param_count` let the GCS track list progress. Wire:
/// param_value(f32) || param_count(u16) || param_index(u16) || param_id(char[16])
/// || param_type(u8).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ParamValue {
    pub param_value: f32,
    pub param_count: u16,
    pub param_index: u16,
    pub param_id: ParamId,
    pub param_type: u8,
}

impl ParamValue {
    /// Encode into the 25-byte payload.
    pub fn encode_payload(&self) -> [u8; PARAM_VALUE_PAYLOAD_LEN] {
        let mut out = [0u8; PARAM_VALUE_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.param_value.to_le_bytes());
        out[4..6].copy_from_slice(&self.param_count.to_le_bytes());
        out[6..8].copy_from_slice(&self.param_index.to_le_bytes());
        out[8..24].copy_from_slice(&self.param_id);
        out[24] = self.param_type;
        out
    }

    /// Decode from a 25-byte payload. `None` on length mismatch.
    pub fn decode_payload(payload: &[u8]) -> Option<Self> {
        if payload.len() != PARAM_VALUE_PAYLOAD_LEN {
            return None;
        }
        let mut param_id = [0u8; 16];
        param_id.copy_from_slice(&payload[8..24]);
        Some(Self {
            param_value: f32::from_le_bytes([payload[0], payload[1], payload[2], payload[3]]),
            param_count: u16::from_le_bytes([payload[4], payload[5]]),
            param_index: u16::from_le_bytes([payload[6], payload[7]]),
            param_id,
            param_type: payload[24],
        })
    }
}

/// Build a 16-byte `ParamId` from a name, NUL-padded / truncated to 16 bytes.
/// (Mirror of `relay_param::param_id`, kept here so relay-mavlink stays
/// dependency-free; the seam crate uses the relay-param one.)
pub fn param_id(name: &str) -> ParamId {
    let mut id = [0u8; 16];
    let b = name.as_bytes();
    let n = if b.len() < 16 { b.len() } else { 16 };
    id[..n].copy_from_slice(&b[..n]);
    id
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mc_roll_p() -> ParamId {
        // "MC_ROLL_P" NUL-padded — the id used in the pymavlink reference vectors.
        param_id("MC_ROLL_P")
    }

    // ── CONFORMANCE: exact pymavlink reference vectors ──
    // Generated by scripts/gen-mavlink-param-vectors.py against pymavlink 2.4.49
    // (the impl QGC/MAVSDK/PX4/ArduPilot conform to). encode must REPRODUCE these
    // bytes and decode must RECOVER the fields — a round-trip alone cannot catch a
    // wrong size-sorted field order (encode+decode would share the mistake).

    #[test]
    fn param_request_list_matches_pymavlink_reference() {
        // PARAM_REQUEST_LIST id=21 crc_extra=159 len=2 payload=0101
        assert_eq!(PARAM_REQUEST_LIST_MSG_ID, 21);
        assert_eq!(PARAM_REQUEST_LIST_CRC_EXTRA, 159);
        let canonical = [0x01, 0x01];
        let m = ParamRequestList { target_system: 1, target_component: 1 };
        assert_eq!(m.encode_payload(), canonical);
        assert_eq!(ParamRequestList::decode_payload(&canonical), Some(m));
    }

    #[test]
    fn param_request_read_matches_pymavlink_reference() {
        // id=20 crc_extra=214 len=20
        // payload=ffff01014d435f524f4c4c5f5000000000000000
        assert_eq!(PARAM_REQUEST_READ_MSG_ID, 20);
        assert_eq!(PARAM_REQUEST_READ_CRC_EXTRA, 214);
        let canonical = [
            0xff, 0xff, 0x01, 0x01, 0x4d, 0x43, 0x5f, 0x52, 0x4f, 0x4c, 0x4c, 0x5f, 0x50, 0x00,
            0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        ];
        let m = ParamRequestRead {
            param_index: -1,
            target_system: 1,
            target_component: 1,
            param_id: mc_roll_p(),
        };
        assert_eq!(m.encode_payload(), canonical);
        assert_eq!(ParamRequestRead::decode_payload(&canonical), Some(m));
    }

    #[test]
    fn param_set_matches_pymavlink_reference() {
        // id=23 crc_extra=168 len=23 param_value=8.0 type=9
        // payload=0000004101014d435f524f4c4c5f500000000000000009
        assert_eq!(PARAM_SET_MSG_ID, 23);
        assert_eq!(PARAM_SET_CRC_EXTRA, 168);
        let canonical = [
            0x00, 0x00, 0x00, 0x41, 0x01, 0x01, 0x4d, 0x43, 0x5f, 0x52, 0x4f, 0x4c, 0x4c, 0x5f,
            0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
        ];
        let m = ParamSet {
            param_value: 8.0,
            target_system: 1,
            target_component: 1,
            param_id: mc_roll_p(),
            param_type: MAV_PARAM_TYPE_REAL32,
        };
        assert_eq!(m.encode_payload(), canonical);
        assert_eq!(ParamSet::decode_payload(&canonical), Some(m));
    }

    #[test]
    fn param_value_matches_pymavlink_reference() {
        // id=22 crc_extra=220 len=25 value=8.0 count=2 index=0 type=9
        // payload=00000041020000004d435f524f4c4c5f500000000000000009
        assert_eq!(PARAM_VALUE_MSG_ID, 22);
        assert_eq!(PARAM_VALUE_CRC_EXTRA, 220);
        let canonical = [
            0x00, 0x00, 0x00, 0x41, 0x02, 0x00, 0x00, 0x00, 0x4d, 0x43, 0x5f, 0x52, 0x4f, 0x4c,
            0x4c, 0x5f, 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x09,
        ];
        let m = ParamValue {
            param_value: 8.0,
            param_count: 2,
            param_index: 0,
            param_id: mc_roll_p(),
            param_type: MAV_PARAM_TYPE_REAL32,
        };
        assert_eq!(m.encode_payload(), canonical);
        assert_eq!(ParamValue::decode_payload(&canonical), Some(m));
    }

    // ── totality: decode rejects wrong lengths (never panics) ──

    #[test]
    fn decode_rejects_wrong_lengths() {
        assert!(ParamRequestList::decode_payload(&[0u8; 1]).is_none());
        assert!(ParamRequestList::decode_payload(&[0u8; 3]).is_none());
        assert!(ParamRequestRead::decode_payload(&[0u8; 19]).is_none());
        assert!(ParamSet::decode_payload(&[0u8; 22]).is_none());
        assert!(ParamSet::decode_payload(&[0u8; 24]).is_none());
        assert!(ParamValue::decode_payload(&[0u8; 24]).is_none());
    }

    use proptest::prelude::*;
    proptest! {
        /// PARAM_SET round-trips for ANY fields (bit-pattern f32 so NaN payloads
        /// also round-trip), and decode never panics on a 23-byte buffer.
        #[test]
        fn param_set_round_trip(
            vbits in any::<u32>(), tsys in any::<u8>(), tcomp in any::<u8>(),
            idbytes in any::<[u8; 16]>(), ptype in any::<u8>(),
        ) {
            let m = ParamSet {
                param_value: f32::from_bits(vbits),
                target_system: tsys,
                target_component: tcomp,
                param_id: idbytes,
                param_type: ptype,
            };
            let bytes = m.encode_payload();
            let d = ParamSet::decode_payload(&bytes).expect("decode");
            prop_assert_eq!(d.param_value.to_bits(), m.param_value.to_bits());
            prop_assert_eq!(d.target_system, m.target_system);
            prop_assert_eq!(d.param_id, m.param_id);
            prop_assert_eq!(d.param_type, m.param_type);
        }

        /// PARAM_VALUE round-trips for ANY fields.
        #[test]
        fn param_value_round_trip(
            vbits in any::<u32>(), count in any::<u16>(), index in any::<u16>(),
            idbytes in any::<[u8; 16]>(), ptype in any::<u8>(),
        ) {
            let m = ParamValue {
                param_value: f32::from_bits(vbits),
                param_count: count,
                param_index: index,
                param_id: idbytes,
                param_type: ptype,
            };
            let bytes = m.encode_payload();
            let d = ParamValue::decode_payload(&bytes).expect("decode");
            prop_assert_eq!(d.param_value.to_bits(), m.param_value.to_bits());
            prop_assert_eq!(d.param_count, m.param_count);
            prop_assert_eq!(d.param_index, m.param_index);
            prop_assert_eq!(d.param_id, m.param_id);
        }
    }
}
