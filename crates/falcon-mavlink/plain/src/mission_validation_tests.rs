//! SWREQ-FALCON-MISSIONUP-P02: an uploaded mission is validated item by item,
//! staged, and committed whole. One test per hostile upload found in the
//! 2026-11-13 codebase review (F2-F5); each failed or panicked before.

use super::*;
use falcon_core::{FlightSupervisor, MISSION_MAX_ALT_AGL_M, MISSION_MIN_ALT_AGL_M};
use relay_mavlink::{
    FrameHeader, MAV_MISSION_DENIED, MISSION_ACK_CRC_EXTRA, MISSION_ACK_MSG_ID,
    MISSION_COUNT_CRC_EXTRA, MISSION_COUNT_MSG_ID, MISSION_ITEM_INT_CRC_EXTRA,
    MISSION_ITEM_INT_MSG_ID, MISSION_REQUEST_INT_CRC_EXTRA, MISSION_REQUEST_INT_MSG_ID, MissionAck,
    MissionCount, MissionItemInt, MissionRequestInt, encode_frame, parse_frame,
};

const HOME_LAT_E7: i32 = 473_977_000;
const HOME_LON_E7: i32 = 85_456_000;
const VEH: u8 = 1;
const GCS: u8 = 255;
const GCS_COMP: u8 = 190;

fn bridge() -> MavBridge {
    MavBridge::new(VEH, 1, HOME_LAT_E7, HOME_LON_E7, 488_000)
}

fn frame(sys: u8, comp: u8, msg_id: u32, payload: &[u8], extra: u8) -> ([u8; 300], usize) {
    let header = FrameHeader {
        magic: MAGIC_V2,
        payload_len: payload.len() as u8,
        incompat_flags: 0,
        compat_flags: 0,
        sequence: 0,
        system_id: sys,
        component_id: comp,
        message_id: msg_id,
    };
    let mut buf = [0u8; 300];
    let n = encode_frame(&header, payload, extra, &mut buf).expect("encode");
    (buf, n)
}

fn count_from(sys: u8, count: u16, mission_type: u8) -> ([u8; 300], usize) {
    let mc = MissionCount {
        count,
        target_system: VEH,
        target_component: 1,
    };
    let base = mc.encode_payload();
    let mut p = [0u8; 5];
    p[..4].copy_from_slice(&base);
    p[4] = mission_type;
    let len = if mission_type == 0 { 4 } else { 5 };
    frame(
        sys,
        GCS_COMP,
        MISSION_COUNT_MSG_ID,
        &p[..len],
        MISSION_COUNT_CRC_EXTRA,
    )
}

fn wp_item(seq: u16, x: i32, y: i32, z: f32) -> MissionItemInt {
    MissionItemInt {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x,
        y,
        z,
        seq,
        command: MAV_CMD_NAV_WAYPOINT,
        target_system: VEH,
        target_component: 1,
        frame: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        current: 0,
        autocontinue: 1,
    }
}

fn item_from(sys: u8, it: &MissionItemInt) -> ([u8; 300], usize) {
    frame(
        sys,
        GCS_COMP,
        MISSION_ITEM_INT_MSG_ID,
        &it.encode_payload(),
        MISSION_ITEM_INT_CRC_EXTRA,
    )
}

/// The MISSION_ACK result code a Rejected/Complete reply carries.
fn ack_code(out: &[u8], n: usize) -> u8 {
    let (f, _) = parse_frame(&out[..n], MISSION_ACK_CRC_EXTRA).expect("ack frame");
    assert_eq!(f.header.message_id, MISSION_ACK_MSG_ID);
    MissionAck::decode_payload(f.payload).expect("ack").mav_type
}

/// Upload a valid one-item mission at 10 m from GCS; returns the bridge.
fn bridge_with_mission() -> MavBridge {
    let mut b = bridge();
    let mut out = [0u8; 300];
    let (c, n) = count_from(GCS, 1, 0);
    assert!(matches!(
        b.ingest_mission(&c[..n], &mut out),
        MissionUpload::Request(_)
    ));
    let (i, n) = item_from(GCS, &wp_item(0, HOME_LAT_E7 + 1000, HOME_LON_E7, 10.0));
    assert!(matches!(
        b.ingest_mission(&i[..n], &mut out),
        MissionUpload::Complete(_)
    ));
    assert_eq!(b.mission_waypoints().len(), 1);
    b
}

/// Start a one-item upload and feed the item; returns the reply.
fn upload_one(b: &mut MavBridge, it: &MissionItemInt, out: &mut [u8; 300]) -> MissionUpload {
    let (c, n) = count_from(GCS, 1, 0);
    assert!(matches!(
        b.ingest_mission(&c[..n], out),
        MissionUpload::Request(_)
    ));
    let (i, n) = item_from(GCS, it);
    b.ingest_mission(&i[..n], out)
}

#[test]
fn hostile_altitudes_are_refused_and_the_mission_kept() {
    for z in [f32::NAN, f32::INFINITY, -50.0, 0.0, 0.5, 120.5, 5000.0] {
        let mut b = bridge_with_mission();
        let before = b.mission_waypoints()[0];
        let mut out = [0u8; 300];
        let r = upload_one(&mut b, &wp_item(0, HOME_LAT_E7, HOME_LON_E7, z), &mut out);
        let MissionUpload::Rejected(n) = r else {
            panic!("z={z}: {r:?}")
        };
        assert_eq!(ack_code(&out, n), MAV_MISSION_INVALID_PARAM7, "z={z}");
        assert_eq!(b.mission_waypoints(), &[before], "z={z}: mission replaced");
    }
}

#[test]
fn altitude_band_edges_are_accepted() {
    for z in [MISSION_MIN_ALT_M, MISSION_MAX_ALT_M] {
        let mut b = bridge();
        let mut out = [0u8; 300];
        let r = upload_one(&mut b, &wp_item(0, HOME_LAT_E7, HOME_LON_E7, z), &mut out);
        assert!(matches!(r, MissionUpload::Complete(_)), "z={z}: {r:?}");
    }
}

/// Before: `item.x - home_lat` overflowed i32 (panic in debug, a waypoint
/// 18,629 km away in release).
#[test]
fn extreme_coordinates_are_refused_without_overflow() {
    let cases = [
        (i32::MIN, HOME_LON_E7, MAV_MISSION_INVALID_PARAM5_X),
        (i32::MAX, HOME_LON_E7, MAV_MISSION_INVALID_PARAM5_X),
        (900_000_001, HOME_LON_E7, MAV_MISSION_INVALID_PARAM5_X),
        (HOME_LAT_E7, i32::MIN, MAV_MISSION_INVALID_PARAM6_Y),
        (HOME_LAT_E7, 1_800_000_001, MAV_MISSION_INVALID_PARAM6_Y),
    ];
    for (x, y, code) in cases {
        let mut b = bridge();
        let mut out = [0u8; 300];
        let r = upload_one(&mut b, &wp_item(0, x, y, 10.0), &mut out);
        let MissionUpload::Rejected(n) = r else {
            panic!("x={x} y={y}: {r:?}")
        };
        assert_eq!(ack_code(&out, n), code, "x={x} y={y}");
        assert!(b.mission_waypoints().is_empty());
    }
    // The valid extremes convert without overflow.
    let mut b = bridge();
    let mut out = [0u8; 300];
    let r = upload_one(
        &mut b,
        &wp_item(0, -900_000_000, 1_800_000_000, 10.0),
        &mut out,
    );
    assert!(matches!(r, MissionUpload::Complete(_)), "{r:?}");
    assert!(b.mission_waypoints()[0].iter().all(|v| v.is_finite()));
}

#[test]
fn unsupported_frame_and_command_are_refused() {
    let mut local = wp_item(0, 5, 5, 10.0);
    local.frame = 1; // MAV_FRAME_LOCAL_NED
    let mut b = bridge_with_mission();
    let mut out = [0u8; 300];
    let MissionUpload::Rejected(n) = upload_one(&mut b, &local, &mut out) else {
        panic!()
    };
    assert_eq!(ack_code(&out, n), MAV_MISSION_UNSUPPORTED_FRAME);
    assert_eq!(b.mission_waypoints().len(), 1);

    let mut loiter = wp_item(0, HOME_LAT_E7, HOME_LON_E7, 10.0);
    loiter.command = 17; // MAV_CMD_NAV_LOITER_UNLIM
    let MissionUpload::Rejected(n) = upload_one(&mut b, &loiter, &mut out) else {
        panic!()
    };
    assert_eq!(ack_code(&out, n), MAV_MISSION_UNSUPPORTED);
    assert_eq!(b.mission_waypoints().len(), 1);
}

/// Before: more than 16 items were silently truncated and ACKed ACCEPTED.
#[test]
fn too_many_items_is_no_space() {
    let mut b = bridge_with_mission();
    let mut out = [0u8; 300];
    let (c, n) = count_from(GCS, MAX_MISSION_ITEMS as u16 + 1, 0);
    let MissionUpload::Rejected(m) = b.ingest_mission(&c[..n], &mut out) else {
        panic!()
    };
    assert_eq!(ack_code(&out, m), MAV_MISSION_NO_SPACE);
    assert_eq!(b.mission_waypoints().len(), 1);
}

/// Before: a fence upload (mission_type 1) replaced the flight mission.
#[test]
fn fence_upload_does_not_replace_the_mission() {
    let mut b = bridge_with_mission();
    let before = b.mission_waypoints()[0];
    let mut out = [0u8; 300];
    let (c, n) = count_from(GCS, 2, 1);
    let MissionUpload::Rejected(m) = b.ingest_mission(&c[..n], &mut out) else {
        panic!()
    };
    assert_eq!(ack_code(&out, m), MAV_MISSION_UNSUPPORTED);
    // Items that follow are ignored: no upload is active.
    let (i, n) = item_from(GCS, &wp_item(0, HOME_LAT_E7, HOME_LON_E7, 10.0));
    assert_eq!(b.ingest_mission(&i[..n], &mut out), MissionUpload::Ignored);
    assert_eq!(b.mission_waypoints(), &[before]);
}

/// Before: a stray MISSION_COUNT from any sender emptied the mission.
#[test]
fn stray_count_does_not_wipe_the_mission() {
    let mut b = bridge_with_mission();
    let before = b.mission_waypoints()[0];
    let mut out = [0u8; 300];
    let (c, n) = count_from(66, 16, 0);
    assert!(matches!(
        b.ingest_mission(&c[..n], &mut out),
        MissionUpload::Request(_)
    ));
    assert_eq!(
        b.mission_waypoints(),
        &[before],
        "staging must not touch the mission"
    );
}

#[test]
fn items_from_another_sender_are_ignored() {
    let mut b = bridge();
    let mut out = [0u8; 300];
    let (c, n) = count_from(GCS, 1, 0);
    assert!(matches!(
        b.ingest_mission(&c[..n], &mut out),
        MissionUpload::Request(_)
    ));
    let it = wp_item(0, HOME_LAT_E7, HOME_LON_E7, 10.0);
    let (i, n) = item_from(66, &it);
    assert_eq!(b.ingest_mission(&i[..n], &mut out), MissionUpload::Ignored);
    let (i, n) = item_from(GCS, &it);
    assert!(matches!(
        b.ingest_mission(&i[..n], &mut out),
        MissionUpload::Complete(_)
    ));
}

/// Before: refreshing the downloadable mission with 16 waypoints during an
/// upload made the next item index out of bounds (panic in debug and release).
#[test]
fn refreshing_the_download_mid_upload_cannot_go_out_of_bounds() {
    let mut b = bridge();
    let mut out = [0u8; 300];
    let (c, n) = count_from(GCS, 2, 0);
    assert!(matches!(
        b.ingest_mission(&c[..n], &mut out),
        MissionUpload::Request(_)
    ));
    b.set_downloadable_mission(&[[1.0, 1.0, -5.0]; MAX_MISSION_ITEMS]);
    for seq in 0..2u16 {
        let (i, n) = item_from(GCS, &wp_item(seq, HOME_LAT_E7, HOME_LON_E7, 10.0));
        let r = b.ingest_mission(&i[..n], &mut out);
        assert!(
            !matches!(r, MissionUpload::Rejected(_) | MissionUpload::Ignored),
            "{r:?}"
        );
    }
    assert_eq!(b.mission_waypoints().len(), 2, "the upload commits whole");
}

#[test]
fn download_request_for_another_vehicle_is_ignored() {
    let mut b = bridge_with_mission();
    let mut out = [0u8; 300];
    let req = MissionRequestInt {
        seq: 0,
        target_system: 9,
        target_component: 1,
    };
    let (r, n) = frame(
        GCS,
        GCS_COMP,
        MISSION_REQUEST_INT_MSG_ID,
        &req.encode_payload(),
        MISSION_REQUEST_INT_CRC_EXTRA,
    );
    assert_eq!(b.serve_mission(&r[..n], &mut out), MissionDownload::Ignored);
    let req = MissionRequestInt {
        seq: 0,
        target_system: VEH,
        target_component: 1,
    };
    let (r, n) = frame(
        GCS,
        GCS_COMP,
        MISSION_REQUEST_INT_MSG_ID,
        &req.encode_payload(),
        MISSION_REQUEST_INT_CRC_EXTRA,
    );
    assert!(matches!(
        b.serve_mission(&r[..n], &mut out),
        MissionDownload::Item(_)
    ));
}

/// A host-supplied download mission far outside any real range projects
/// back without overflow (saturating), instead of panicking.
#[test]
fn download_projection_saturates() {
    let mut b = bridge();
    b.set_downloadable_mission(&[[3.0e9, -3.0e9, -10.0]]);
    let mut out = [0u8; 300];
    let req = MissionRequestInt {
        seq: 0,
        target_system: VEH,
        target_component: 1,
    };
    let (r, n) = frame(
        GCS,
        GCS_COMP,
        MISSION_REQUEST_INT_MSG_ID,
        &req.encode_payload(),
        MISSION_REQUEST_INT_CRC_EXTRA,
    );
    assert!(matches!(
        b.serve_mission(&r[..n], &mut out),
        MissionDownload::Item(_)
    ));
}

/// The bridge's altitude band and the supervisor's are the same numbers.
#[test]
fn bridge_and_supervisor_altitude_band_agree() {
    assert_eq!(MISSION_MIN_ALT_M, MISSION_MIN_ALT_AGL_M);
    assert_eq!(MISSION_MAX_ALT_M, MISSION_MAX_ALT_AGL_M);
}

/// The supervisor refuses a whole mission with any waypoint out of the
/// envelope, keeping the previous one; a valid mission loads.
#[test]
fn supervisor_refuses_out_of_envelope_missions() {
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 50.0, 2.0, 1.0);
    assert!(sup.set_mission_waypoints(&[[10.0, 0.0, -2.0]]));
    let bad: [[f32; 3]; 6] = [
        [f32::NAN, 0.0, -2.0],
        [0.0, 0.0, f32::NAN],
        [0.0, 0.0, 50.0],    // 50 m below home
        [0.0, 0.0, -0.5],    // under the 1 m floor
        [0.0, 0.0, -5000.0], // above the 120 m ceiling
        [60.0, 0.0, -2.0],   // outside the 50 m fence
    ];
    for wp in bad {
        assert!(
            !sup.set_mission_waypoints(&[[5.0, 0.0, -3.0], wp]),
            "{wp:?} accepted"
        );
        assert_eq!(sup.waypoint_count(), 1, "{wp:?} replaced the mission");
    }
}

#[test]
fn denied_code_is_the_mavlink_value() {
    // Pinned against pymavlink 2.4.50 common.xml (MAV_MISSION_RESULT).
    assert_eq!(
        (
            MAV_MISSION_UNSUPPORTED_FRAME,
            MAV_MISSION_UNSUPPORTED,
            MAV_MISSION_NO_SPACE,
            MAV_MISSION_INVALID_PARAM5_X,
            MAV_MISSION_INVALID_PARAM6_Y,
            MAV_MISSION_INVALID_PARAM7,
            MAV_MISSION_DENIED,
        ),
        (2, 3, 4, 10, 11, 12, 14)
    );
}
