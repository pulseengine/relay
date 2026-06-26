//! Falcon parameter bridge — the translation seam between the MAVLink PARAM_*
//! wire (QGroundControl / MAVSDK / PX4-ecosystem GCS) and falcon's verified
//! bounded parameter store.
//!
//! The codec (`relay-mavlink::param`) and the store (`relay-param`, whose set is
//! Kani-proven to reject out-of-range writes — PARAM-K01) are both pure and
//! independently verified. This crate adds only the *translation*, in two
//! directions:
//!
//!   inbound   PARAM_SET            ──►  relay_param::ParamStore::set  (bounded)
//!             PARAM_REQUEST_READ   ──►  store lookup (by id or index)
//!             PARAM_REQUEST_LIST   ──►  iterate the store
//!
//!   outbound  the store's value    ──►  PARAM_VALUE  (reply / list item / ack)
//!
//! Keeping the bridge thin is deliberate: the bound-enforcement invariant lives
//! in the verified store, the byte layout lives in the reference-validated codec,
//! and this crate is small enough to read in one sitting.
//!
//! ## What's verified / tested (SWREQ-FALCON-PARAM-P02)
//!
//!   PARAM-P02a: a PARAM_SET whose value is within the parameter's schema bounds
//!               is applied and the PARAM_VALUE ack carries the new value.
//!   PARAM-P02b: a PARAM_SET whose value is OUT of bounds is rejected — the
//!               stored value is unchanged and the ack carries the OLD value, so
//!               the GCS reverts the edit (the end-to-end form of PARAM-K01).
//!   PARAM-P02c: an unknown param-id produces no ack (None); the store is
//!               untouched.
//!   PARAM-P02d: PARAM_REQUEST_READ (by id or index) and PARAM_REQUEST_LIST
//!               iteration return the correct PARAM_VALUE (id, value, index,
//!               count) and never panic.

#![no_std]
#![forbid(unsafe_code)]

use relay_mavlink::{ParamRequestRead, ParamSet, ParamValue, MAV_PARAM_TYPE_REAL32};
use relay_param::{ParamId, ParamStore, SetResult};

/// The (index, count) of a parameter id, by scanning the store — the PARAM_VALUE
/// fields the GCS uses to track list progress. `None` if the id is not present.
fn locate<const N: usize>(store: &ParamStore<N>, id: &ParamId) -> Option<(u16, u16)> {
    let count = store.len();
    let mut i = 0;
    while i < count {
        match store.nth(i) {
            Some((pid, _)) if &pid == id => return Some((i as u16, count as u16)),
            _ => {}
        }
        i += 1;
    }
    None
}

/// The PARAM_VALUE for the parameter at index `i` (REAL32-typed). `None` past end.
fn value_at<const N: usize>(store: &ParamStore<N>, i: usize) -> Option<ParamValue> {
    let (id, v) = store.nth(i)?;
    Some(ParamValue {
        param_value: v,
        param_count: store.len() as u16,
        param_index: i as u16,
        param_id: id,
        param_type: MAV_PARAM_TYPE_REAL32,
    })
}

/// The PARAM_VALUE reply for a parameter looked up by id. `None` if absent.
fn value_by_id<const N: usize>(store: &ParamStore<N>, id: &ParamId) -> Option<ParamValue> {
    let (index, count) = locate(store, id)?;
    Some(ParamValue {
        param_value: store.get(id)?,
        param_count: count,
        param_index: index,
        param_id: *id,
        param_type: MAV_PARAM_TYPE_REAL32,
    })
}

/// Handle an inbound PARAM_SET: apply the BOUNDED write and return the
/// PARAM_VALUE the vehicle acks with. The ack always carries the store's CURRENT
/// value — unchanged when the write was out-of-range (PARAM-K01), so the GCS UI
/// reverts a rejected edit. Returns `None` for an unknown parameter (the
/// reference autopilots stay silent). Total: never panics.
pub fn on_param_set<const N: usize>(
    store: &mut ParamStore<N>,
    msg: &ParamSet,
) -> Option<ParamValue> {
    match store.set(&msg.param_id, msg.param_value) {
        SetResult::Unknown => None,
        SetResult::Applied | SetResult::OutOfRange => value_by_id(store, &msg.param_id),
    }
}

/// Handle an inbound PARAM_REQUEST_READ: read by index when `param_index >= 0`,
/// else by `param_id`. Returns the PARAM_VALUE reply, or `None` if not found.
pub fn on_param_request_read<const N: usize>(
    store: &ParamStore<N>,
    msg: &ParamRequestRead,
) -> Option<ParamValue> {
    if msg.param_index >= 0 {
        value_at(store, msg.param_index as usize)
    } else {
        value_by_id(store, &msg.param_id)
    }
}

/// The PARAM_VALUE for the `i`-th parameter — the GCS dumps the whole table in
/// response to PARAM_REQUEST_LIST by iterating `i` in `0..count`. `None` past the
/// end (the natural loop terminator).
pub fn on_param_request_list_item<const N: usize>(
    store: &ParamStore<N>,
    i: usize,
) -> Option<ParamValue> {
    value_at(store, i)
}

#[cfg(test)]
mod tests {
    use super::*;
    use relay_mavlink::param::param_id;
    use relay_param::ParamDef;

    fn store() -> ParamStore<4> {
        let mut s = ParamStore::new();
        s.register(ParamDef { id: param_id("MC_ROLL_P"), min: 0.0, max: 12.0, default: 6.5 });
        s.register(ParamDef { id: param_id("BAT_LOW_V"), min: 10.0, max: 16.8, default: 14.0 });
        s
    }

    fn set_msg(name: &str, value: f32) -> ParamSet {
        ParamSet {
            param_value: value,
            target_system: 1,
            target_component: 1,
            param_id: param_id(name),
            param_type: MAV_PARAM_TYPE_REAL32,
        }
    }

    /// PARAM-P02a: an in-range PARAM_SET applies; the ack carries the new value.
    #[test]
    fn in_range_set_applies_and_acks_new_value() {
        let mut s = store();
        let ack = on_param_set(&mut s, &set_msg("MC_ROLL_P", 8.0)).expect("ack");
        assert_eq!(ack.param_value, 8.0);
        assert_eq!(ack.param_id, param_id("MC_ROLL_P"));
        assert_eq!(ack.param_index, 0);
        assert_eq!(ack.param_count, 2);
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(8.0));
    }

    /// PARAM-P02b (the safety property, end-to-end): an out-of-range PARAM_SET is
    /// rejected — the store is unchanged and the ack carries the OLD value.
    #[test]
    fn out_of_range_set_rejected_acks_old_value() {
        let mut s = store();
        let ack = on_param_set(&mut s, &set_msg("MC_ROLL_P", 99.0)).expect("ack");
        assert_eq!(ack.param_value, 6.5); // unchanged default, not 99.0
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(6.5));
        // NaN likewise rejected.
        let ack = on_param_set(&mut s, &set_msg("MC_ROLL_P", f32::NAN)).expect("ack");
        assert_eq!(ack.param_value, 6.5);
    }

    /// PARAM-P02c: an unknown param-id produces no ack and touches nothing.
    #[test]
    fn unknown_param_no_ack() {
        let mut s = store();
        assert_eq!(on_param_set(&mut s, &set_msg("NOPE", 1.0)), None);
        assert_eq!(s.len(), 2);
    }

    /// PARAM-P02d: read by id and by index return the right PARAM_VALUE.
    #[test]
    fn request_read_by_id_and_index() {
        let s = store();
        let by_id = on_param_request_read(
            &s,
            &ParamRequestRead {
                param_index: -1,
                target_system: 1,
                target_component: 1,
                param_id: param_id("BAT_LOW_V"),
            },
        )
        .expect("by id");
        assert_eq!(by_id.param_value, 14.0);
        assert_eq!(by_id.param_index, 1);

        let by_index = on_param_request_read(
            &s,
            &ParamRequestRead {
                param_index: 0,
                target_system: 1,
                target_component: 1,
                param_id: [0u8; 16],
            },
        )
        .expect("by index");
        assert_eq!(by_index.param_id, param_id("MC_ROLL_P"));
        assert_eq!(by_index.param_value, 6.5);
    }

    /// PARAM-P02d: PARAM_REQUEST_LIST iteration yields every param then stops.
    #[test]
    fn request_list_iterates_then_terminates() {
        let s = store();
        let mut seen = 0;
        let mut i = 0;
        while let Some(v) = on_param_request_list_item(&s, i) {
            assert_eq!(v.param_count, 2);
            assert_eq!(v.param_index, i as u16);
            seen += 1;
            i += 1;
        }
        assert_eq!(seen, 2);
        assert_eq!(on_param_request_list_item(&s, 99), None);
    }

    /// Full-frame end-to-end: a PARAM_SET frame off the wire drives the bounded
    /// store and the PARAM_VALUE ack re-encodes to a parseable frame. Proves the
    /// codec ⇄ store seam over the real MAVLink v2 envelope, not just payloads.
    #[test]
    fn full_frame_param_set_drives_store() {
        use relay_mavlink::{
            encode_frame, parse_frame, FrameHeader, MAGIC_V2, PARAM_SET_CRC_EXTRA,
            PARAM_SET_MSG_ID, PARAM_SET_PAYLOAD_LEN, PARAM_VALUE_CRC_EXTRA, PARAM_VALUE_MSG_ID,
        };
        let mut s = store();
        // GCS builds + frames a PARAM_SET for MC_ROLL_P = 9.0 (in range).
        let payload = set_msg("MC_ROLL_P", 9.0).encode_payload();
        let hdr = FrameHeader {
            magic: MAGIC_V2,
            payload_len: PARAM_SET_PAYLOAD_LEN as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: 7,
            system_id: 255,
            component_id: 190,
            message_id: PARAM_SET_MSG_ID,
        };
        let mut wire = [0u8; 64];
        let n = encode_frame(&hdr, &payload, PARAM_SET_CRC_EXTRA, &mut wire).expect("encode");

        // Vehicle parses, decodes, applies via the seam.
        let (frame, _) = parse_frame(&wire[..n], PARAM_SET_CRC_EXTRA).expect("parse");
        let msg = ParamSet::decode_payload(frame.payload).expect("decode");
        let ack = on_param_set(&mut s, &msg).expect("ack");
        assert_eq!(s.get(&param_id("MC_ROLL_P")), Some(9.0));

        // The ack re-frames to a parseable PARAM_VALUE.
        let ack_payload = ack.encode_payload();
        let ack_hdr = FrameHeader {
            magic: MAGIC_V2,
            payload_len: ack_payload.len() as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: 8,
            system_id: 1,
            component_id: 1,
            message_id: PARAM_VALUE_MSG_ID,
        };
        let mut ack_wire = [0u8; 64];
        let an = encode_frame(&ack_hdr, &ack_payload, PARAM_VALUE_CRC_EXTRA, &mut ack_wire)
            .expect("ack encode");
        let (ack_frame, _) =
            parse_frame(&ack_wire[..an], PARAM_VALUE_CRC_EXTRA).expect("ack parse");
        let ack_decoded = ParamValue::decode_payload(ack_frame.payload).expect("ack decode");
        assert_eq!(ack_decoded.param_value, 9.0);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;
    use relay_mavlink::param::param_id;
    use relay_param::ParamDef;

    proptest! {
        /// The seam preserves PARAM-K01 end-to-end: for ANY PARAM_SET value, the
        /// stored value stays within the parameter's schema bounds, and the ack
        /// (when present) carries exactly the stored value.
        #[test]
        fn seam_keeps_store_in_bounds(vbits in any::<u32>()) {
            let mut s: ParamStore<2> = ParamStore::new();
            s.register(ParamDef { id: param_id("P"), min: 2.0, max: 8.0, default: 5.0 });
            let msg = ParamSet {
                param_value: f32::from_bits(vbits),
                target_system: 1,
                target_component: 1,
                param_id: param_id("P"),
                param_type: MAV_PARAM_TYPE_REAL32,
            };
            let ack = on_param_set(&mut s, &msg).expect("known param always acks");
            let stored = s.get(&param_id("P")).unwrap();
            prop_assert!((2.0..=8.0).contains(&stored));
            prop_assert_eq!(ack.param_value, stored);
        }
    }
}
