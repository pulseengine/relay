//! Bridge tests — the four MAVBRIDGE properties.

use super::*;
use relay_fsm::{FlightFsm, Gates};
use relay_mavlink::{
    COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, COMMAND_LONG_PAYLOAD_LEN, FrameHeader,
    GLOBAL_POSITION_INT_CRC_EXTRA, GlobalPositionInt, HEARTBEAT_CRC_EXTRA, Heartbeat, MAGIC_V2,
    encode_frame, parse_frame,
};

// A PX4-style SITL home: Zürich, ~488 m AMSL.
const HOME_LAT_E7: i32 = 473_977_000;
const HOME_LON_E7: i32 = 85_456_000;
const HOME_ALT_MM: i32 = 488_000;

const GCS_SYS: u8 = 255;
const VEH_SYS: u8 = 1;
const VEH_COMP: u8 = 1;

fn gates(level: bool, throttle_low: bool, have_position: bool) -> Gates {
    Gates {
        level,
        throttle_low,
        have_position,
    }
}

/// Ingest a command through the bridge and apply the resulting event to the
/// FSM under the given gates, returning the new mode.
fn cmd_drives(bridge: &MavBridge, fsm: &mut FlightFsm, cmd: &CommandLong, g: Gates) -> Mode {
    let (buf, n) = encode_command(cmd);
    let ev = bridge
        .ingest(&buf[..n])
        .expect("valid frame")
        .expect("a routed event");
    fsm.on(ev, g)
}

/// Encode a COMMAND_LONG as a GCS would put it on the wire.
fn encode_command(cmd: &CommandLong) -> ([u8; 64], usize) {
    let payload = cmd.encode_payload();
    let header = FrameHeader {
        magic: MAGIC_V2,
        payload_len: COMMAND_LONG_PAYLOAD_LEN as u8,
        incompat_flags: 0,
        compat_flags: 0,
        sequence: 7,
        system_id: GCS_SYS,
        component_id: 190, // MAV_COMP_ID_MISSIONPLANNER
        message_id: COMMAND_LONG_MSG_ID,
    };
    let mut buf = [0u8; 64];
    let n = encode_frame(&header, &payload, COMMAND_LONG_CRC_EXTRA, &mut buf).expect("encode cmd");
    (buf, n)
}

// ---- MAVBRIDGE-P01: command stream drives the FSM lifecycle ----

#[test]
fn command_to_event_mapping_is_exhaustive() {
    assert_eq!(
        command_to_event(&CommandLong::arm_disarm(1, 1, true)),
        Some(Event::Arm)
    );
    assert_eq!(
        command_to_event(&CommandLong::arm_disarm(1, 1, false)),
        Some(Event::RequestDisarm)
    );
    assert_eq!(
        command_to_event(&CommandLong::takeoff(1, 1, 5.0)),
        Some(Event::RequestTakeoff)
    );
    assert_eq!(
        command_to_event(&CommandLong::land(1, 1)),
        Some(Event::RequestLand)
    );
    assert_eq!(
        command_to_event(&CommandLong::rtl(1, 1)),
        Some(Event::RequestRtl)
    );
    assert_eq!(
        command_to_event(&CommandLong::mission_start(1, 1)),
        Some(Event::RequestMission)
    );
    // An unmapped command is ignored, not mistranslated.
    let mut unknown = CommandLong::rtl(1, 1);
    unknown.command = 511; // MAV_CMD_DO_SET_SERVO-ish; falcon doesn't route it
    assert_eq!(command_to_event(&unknown), None);
}

#[test]
fn ingested_commands_drive_full_flight_lifecycle() {
    let bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let mut fsm = FlightFsm::new();

    // arm (ground, level, throttle idle) → Armed
    assert_eq!(
        cmd_drives(
            &bridge,
            &mut fsm,
            &CommandLong::arm_disarm(VEH_SYS, VEH_COMP, true),
            gates(true, true, true)
        ),
        Mode::Armed
    );
    // takeoff (have position) → Takeoff
    assert_eq!(
        cmd_drives(
            &bridge,
            &mut fsm,
            &CommandLong::takeoff(VEH_SYS, VEH_COMP, 5.0),
            gates(true, true, true)
        ),
        Mode::Takeoff
    );
    // sensed milestone (not a command): reached altitude → Loiter
    assert_eq!(
        fsm.on(Event::ReachedAltitude, gates(true, false, true)),
        Mode::Loiter
    );
    // RTL command → Rtl
    assert_eq!(
        cmd_drives(
            &bridge,
            &mut fsm,
            &CommandLong::rtl(VEH_SYS, VEH_COMP),
            gates(true, false, true)
        ),
        Mode::Rtl
    );
    // sensed: reached home → Land
    assert_eq!(
        fsm.on(Event::ReachedHome, gates(true, false, true)),
        Mode::Land
    );
    // sensed: touchdown → Disarmed
    assert_eq!(
        fsm.on(Event::Touchdown, gates(true, true, true)),
        Mode::Disarmed
    );
}

#[test]
fn command_addressed_to_other_vehicle_is_ignored() {
    let bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    // Arm command targeted at system 9, not us.
    let (buf, n) = encode_command(&CommandLong::arm_disarm(9, 1, true));
    assert_eq!(bridge.ingest(&buf[..n]).expect("valid frame"), None);

    // Broadcast (target_system 0) is accepted.
    let (buf, n) = encode_command(&CommandLong::arm_disarm(0, 1, true));
    assert_eq!(
        bridge.ingest(&buf[..n]).expect("valid frame"),
        Some(Event::Arm)
    );
}

#[test]
fn non_command_message_is_not_routed() {
    // A HEARTBEAT frame ingested by the bridge is a valid frame but not a
    // command → Ok(None), no error.
    let mut other = MavBridge::new(GCS_SYS, 1, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let mut buf = [0u8; 64];
    let n = other.heartbeat(Mode::Loiter, &mut buf).expect("encode hb");
    let bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    assert_eq!(bridge.ingest(&buf[..n]).expect("valid frame"), None);
}

// ---- MAVBRIDGE-P02: ingest never panics on arbitrary bytes ----

#[test]
fn ingest_never_panics_on_garbage() {
    let bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    // Empty + all single bytes.
    let _ = bridge.ingest(&[]);
    for b in 0u16..=255 {
        let _ = bridge.ingest(&[b as u8]);
    }
    // Deterministic LCG-driven buffers of varied length (no rand dep).
    let mut state: u32 = 0x1234_5678;
    let mut buf = [0u8; 80];
    for len in 0..buf.len() {
        for byte in buf.iter_mut().take(len) {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            *byte = (state >> 24) as u8;
        }
        // Must return without unwinding; value itself is unconstrained.
        let _ = bridge.ingest(&buf[..len]);
        // Also exercise the valid-magic path explicitly.
        let mut framed = buf;
        framed[0] = MAGIC_V2;
        let _ = bridge.ingest(&framed[..len]);
    }
}

// ---- MAVBRIDGE-P03: outbound telemetry round-trips ----

#[test]
fn heartbeat_round_trips_for_every_mode() {
    let modes = [
        Mode::Disarmed,
        Mode::Armed,
        Mode::Takeoff,
        Mode::Loiter,
        Mode::Mission,
        Mode::Land,
        Mode::Rtl,
    ];
    let mut bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    for mode in modes {
        let mut buf = [0u8; 64];
        let n = bridge.heartbeat(mode, &mut buf).expect("encode hb");
        let (frame, consumed) = parse_frame(&buf[..n], HEARTBEAT_CRC_EXTRA).expect("parse hb");
        assert_eq!(consumed, n);
        assert_eq!(frame.header.system_id, VEH_SYS);
        let hb = Heartbeat::decode_payload(frame.payload).expect("decode hb");
        assert_eq!(hb, mode_to_heartbeat(mode));
        // The custom_mode names the falcon mode 1:1.
        assert_eq!(hb.custom_mode, custom_mode_code(mode));
    }
}

#[test]
fn heartbeat_arms_safety_flag_off_the_ground() {
    // Disarmed: SAFETY_ARMED clear, STANDBY.
    let hb = mode_to_heartbeat(Mode::Disarmed);
    assert_eq!(hb.base_mode & MavModeFlag::SAFETY_ARMED.bits(), 0);
    assert_eq!(hb.system_status, MavState::Standby as u8);
    // Any flying mode: SAFETY_ARMED set, ACTIVE.
    for mode in [
        Mode::Armed,
        Mode::Takeoff,
        Mode::Loiter,
        Mode::Mission,
        Mode::Land,
        Mode::Rtl,
    ] {
        let hb = mode_to_heartbeat(mode);
        assert_ne!(
            hb.base_mode & MavModeFlag::SAFETY_ARMED.bits(),
            0,
            "{mode:?}"
        );
        assert_eq!(hb.system_status, MavState::Active as u8, "{mode:?}");
    }
}

#[test]
fn sequence_number_advances_per_frame() {
    let mut bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let mut buf = [0u8; 64];
    let n0 = bridge.heartbeat(Mode::Loiter, &mut buf).unwrap();
    let seq0 = parse_frame(&buf[..n0], HEARTBEAT_CRC_EXTRA)
        .unwrap()
        .0
        .header
        .sequence;
    let n1 = bridge.heartbeat(Mode::Loiter, &mut buf).unwrap();
    let seq1 = parse_frame(&buf[..n1], HEARTBEAT_CRC_EXTRA)
        .unwrap()
        .0
        .header
        .sequence;
    assert_eq!(seq0, 0);
    assert_eq!(seq1, 1);
}

#[test]
fn global_position_round_trips() {
    let mut bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let p = [12.0_f32, -34.0, -20.0]; // 12 m N, 34 m W, 20 m up
    let v = [1.5_f32, -2.0, 0.5];
    let mut buf = [0u8; 64];
    let n = bridge
        .global_position(p, v, 0.0, 123_456, &mut buf)
        .expect("encode gpi");
    let (frame, consumed) = parse_frame(&buf[..n], GLOBAL_POSITION_INT_CRC_EXTRA).expect("parse");
    assert_eq!(consumed, n);
    let gpi = GlobalPositionInt::decode_payload(frame.payload).expect("decode gpi");

    let expected = ned_to_global_position(
        HOME_LAT_E7,
        HOME_LON_E7,
        HOME_ALT_MM,
        bridge.m_per_deg_lon,
        p,
        v,
        0.0,
        123_456,
    );
    assert_eq!(gpi, expected);
}

// ---- MAVBRIDGE-P04: NED→geodetic projection is correct ----

#[test]
fn projection_places_north_east_and_altitude_correctly() {
    let bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    // One degree of latitude is M_PER_DEG_LAT metres; 1113.2 m north should
    // move lat by ~0.01 deg = 100_000 e7-units.
    let gpi = ned_to_global_position(
        HOME_LAT_E7,
        HOME_LON_E7,
        HOME_ALT_MM,
        bridge.m_per_deg_lon,
        [1113.2, 0.0, 0.0],
        [0.0; 3],
        0.0,
        0,
    );
    let dlat = gpi.lat_e7 - HOME_LAT_E7;
    assert!((dlat - 100_000).abs() <= 50, "dlat_e7 = {dlat}");
    // Pure-north move: longitude unchanged.
    assert_eq!(gpi.lon_e7, HOME_LON_E7);

    // East move uses the (smaller) cos-scaled longitude metre: e metres east
    // → e / m_per_deg_lon degrees. Check the implied metres match.
    let east_m = 500.0_f64;
    let gpi = ned_to_global_position(
        HOME_LAT_E7,
        HOME_LON_E7,
        HOME_ALT_MM,
        bridge.m_per_deg_lon,
        [0.0, east_m as f32, 0.0],
        [0.0; 3],
        0.0,
        0,
    );
    let dlon_deg = (gpi.lon_e7 - HOME_LON_E7) as f64 / 1.0e7;
    let recovered_east_m = dlon_deg * bridge.m_per_deg_lon;
    assert!(
        (recovered_east_m - east_m).abs() < 1.0,
        "recovered {recovered_east_m} m"
    );

    // Altitude sign: NED down = -15 → 15 m up → +15000 mm relative.
    let gpi = ned_to_global_position(
        HOME_LAT_E7,
        HOME_LON_E7,
        HOME_ALT_MM,
        bridge.m_per_deg_lon,
        [0.0, 0.0, -15.0],
        [0.0; 3],
        0.0,
        0,
    );
    assert_eq!(gpi.relative_alt_mm, 15_000);
    assert_eq!(gpi.alt_mm, HOME_ALT_MM + 15_000);
}

#[test]
fn heading_wraps_into_centidegrees() {
    // 0 rad → 0 cdeg (north).
    assert_eq!(yaw_to_cdeg(0.0), 0);
    // +90° = π/2 → 9000 cdeg.
    let east = yaw_to_cdeg(core::f32::consts::FRAC_PI_2);
    assert!((east as i32 - 9000).abs() <= 2, "east heading {east}");
    // -90° wraps to 270° = 27000 cdeg.
    let west = yaw_to_cdeg(-core::f32::consts::FRAC_PI_2);
    assert!((west as i32 - 27000).abs() <= 2, "west heading {west}");
    // Always inside the [0, 35999] contract.
    assert!(yaw_to_cdeg(100.0) <= 35_999);
    assert!(yaw_to_cdeg(-100.0) <= 35_999);
}

// ---- v1.33: MAVLink mission-UPLOAD protocol ----

use relay_mavlink::{
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MAV_MISSION_ACCEPTED, MISSION_ACK_CRC_EXTRA,
    MISSION_COUNT_CRC_EXTRA, MISSION_COUNT_MSG_ID, MISSION_COUNT_PAYLOAD_LEN,
    MISSION_ITEM_INT_CRC_EXTRA, MISSION_ITEM_INT_MSG_ID, MISSION_ITEM_INT_PAYLOAD_LEN,
    MISSION_REQUEST_INT_CRC_EXTRA, MissionAck, MissionCount, MissionItemInt, MissionRequestInt,
};

/// Encode an arbitrary message as a GCS would frame it.
fn encode_gcs(msg_id: u32, payload: &[u8], crc_extra: u8) -> ([u8; 96], usize) {
    let header = FrameHeader {
        magic: MAGIC_V2,
        payload_len: payload.len() as u8,
        incompat_flags: 0,
        compat_flags: 0,
        sequence: 0,
        system_id: GCS_SYS,
        component_id: 190,
        message_id: msg_id,
    };
    let mut buf = [0u8; 96];
    let n = encode_frame(&header, payload, crc_extra, &mut buf).expect("encode gcs msg");
    (buf, n)
}

/// A MISSION_ITEM_INT for a NED waypoint, framed exactly as the GCS would,
/// using the geodetic image of the NED point under the bridge's home.
fn item_for_ned(seq: u16, lat_e7: i32, lon_e7: i32, alt_m: f32) -> ([u8; 96], usize) {
    let item = MissionItemInt {
        param1: 0.0,
        param2: 0.0,
        param3: 0.0,
        param4: 0.0,
        x: lat_e7,
        y: lon_e7,
        z: alt_m,
        seq,
        command: 16, // MAV_CMD_NAV_WAYPOINT
        target_system: VEH_SYS,
        target_component: VEH_COMP,
        frame: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        current: 0,
        autocontinue: 1,
    };
    let p = item.encode_payload();
    assert_eq!(p.len(), MISSION_ITEM_INT_PAYLOAD_LEN);
    encode_gcs(MISSION_ITEM_INT_MSG_ID, &p, MISSION_ITEM_INT_CRC_EXTRA)
}

#[test]
fn mission_upload_handshake_loads_waypoints() {
    let mut bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let mut out = [0u8; 96];

    // 1) GCS announces 2 items → bridge requests item 0.
    let mc = MissionCount {
        count: 2,
        target_system: VEH_SYS,
        target_component: VEH_COMP,
    };
    let p = mc.encode_payload();
    assert_eq!(p.len(), MISSION_COUNT_PAYLOAD_LEN);
    let (buf, n) = encode_gcs(MISSION_COUNT_MSG_ID, &p, MISSION_COUNT_CRC_EXTRA);
    let r = bridge.ingest_mission(&buf[..n], &mut out);
    let req_n = match r {
        MissionUpload::Request(k) => k,
        other => panic!("expected Request after COUNT, got {other:?}"),
    };
    let (rf, _) = parse_frame(&out[..req_n], MISSION_REQUEST_INT_CRC_EXTRA).expect("parse req");
    let req = MissionRequestInt::decode_payload(rf.payload).expect("decode req");
    assert_eq!(req.seq, 0, "must request item 0 first");

    // 2) Item 0 = 0.01° north (≈ 1113.2 m), 100 m east-ish lon, 5 m up.
    let (b0, n0) = item_for_ned(0, HOME_LAT_E7 + 100_000, HOME_LON_E7, 5.0);
    let r0 = bridge.ingest_mission(&b0[..n0], &mut out);
    let req1_n = match r0 {
        MissionUpload::Request(k) => k,
        other => panic!("expected Request for item 1, got {other:?}"),
    };
    let (rf1, _) = parse_frame(&out[..req1_n], MISSION_REQUEST_INT_CRC_EXTRA).unwrap();
    assert_eq!(
        MissionRequestInt::decode_payload(rf1.payload).unwrap().seq,
        1
    );

    // 3) Item 1 = 0.01° east of home, 8 m up → completes → MISSION_ACK(ACCEPTED).
    let (b1, n1) = item_for_ned(1, HOME_LAT_E7, HOME_LON_E7 + 100_000, 8.0);
    let r1 = bridge.ingest_mission(&b1[..n1], &mut out);
    let ack_n = match r1 {
        MissionUpload::Complete(k) => k,
        other => panic!("expected Complete after last item, got {other:?}"),
    };
    let (af, _) = parse_frame(&out[..ack_n], MISSION_ACK_CRC_EXTRA).expect("parse ack");
    let ack = MissionAck::decode_payload(af.payload).expect("decode ack");
    assert_eq!(
        ack.mav_type, MAV_MISSION_ACCEPTED,
        "upload must be accepted"
    );

    // 4) The loaded NED waypoints match the geodetic round-trip.
    let wps = bridge.mission_waypoints();
    assert_eq!(wps.len(), 2, "two waypoints loaded");
    // item 0: 0.01° lat north ⇒ ~1113.2 m north, ~0 east, 5 m up (down = -5).
    assert!(
        (wps[0][0] - 1113.2).abs() < 2.0,
        "wp0 north = {} m",
        wps[0][0]
    );
    assert!(wps[0][1].abs() < 1.0, "wp0 east = {} m", wps[0][1]);
    assert!((wps[0][2] + 5.0).abs() < 0.01, "wp0 down = {} m", wps[0][2]);
    // item 1: 0.01° lon east at this latitude ⇒ ~0 north, several-hundred m east.
    assert!(wps[1][0].abs() < 1.0, "wp1 north = {} m", wps[1][0]);
    let m_per_deg_lon = 111_320.0 * (47.3977_f64.to_radians().cos());
    let expect_east = (0.01 * m_per_deg_lon) as f32;
    assert!(
        (wps[1][1] - expect_east).abs() < 2.0,
        "wp1 east = {} m (expect ~{expect_east})",
        wps[1][1]
    );
    assert!((wps[1][2] + 8.0).abs() < 0.01, "wp1 down = {} m", wps[1][2]);
}

#[test]
fn mission_item_before_count_is_ignored() {
    let mut bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let mut out = [0u8; 96];
    // An ITEM with no upload in progress is ignored (no spurious waypoint).
    let (b, n) = item_for_ned(0, HOME_LAT_E7 + 100_000, HOME_LON_E7, 5.0);
    assert_eq!(
        bridge.ingest_mission(&b[..n], &mut out),
        MissionUpload::Ignored
    );
    assert_eq!(bridge.mission_waypoints().len(), 0);
}

/// END-TO-END (v1.33): a mission UPLOADED over MAVLink is flown by the real
/// FlightSupervisor. The full chain — GCS handshake → NED waypoints →
/// MISSION_START → autonomous sortie — that v1.33 is about.
#[test]
fn uploaded_mission_is_flown_by_the_supervisor() {
    use falcon_core::{FlightSupervisor, SimBackend};
    use relay_fsm::Event;

    // 1) Upload a 3-leg mission over MAVLink (close-in legs, ~3 m each).
    let mut bridge = MavBridge::new(VEH_SYS, VEH_COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);
    let mut out = [0u8; 96];
    let legs = [
        (HOME_LAT_E7 + 280, HOME_LON_E7),       // ~3.1 m N
        (HOME_LAT_E7 + 280, HOME_LON_E7 + 400), // ~3.1 m N, ~3 m E
        (HOME_LAT_E7, HOME_LON_E7 + 400),       // ~3 m E
    ];
    let mc = MissionCount {
        count: 3,
        target_system: VEH_SYS,
        target_component: VEH_COMP,
    };
    let (b, n) = encode_gcs(
        MISSION_COUNT_MSG_ID,
        &mc.encode_payload(),
        MISSION_COUNT_CRC_EXTRA,
    );
    assert!(matches!(
        bridge.ingest_mission(&b[..n], &mut out),
        MissionUpload::Request(_)
    ));
    for (i, (lat, lon)) in legs.iter().enumerate() {
        let (bi, ni) = item_for_ned(i as u16, *lat, *lon, 2.0);
        let r = bridge.ingest_mission(&bi[..ni], &mut out);
        if i < legs.len() - 1 {
            assert!(
                matches!(r, MissionUpload::Request(_)),
                "item {i} requests next"
            );
        } else {
            assert!(
                matches!(r, MissionUpload::Complete(_)),
                "last item completes"
            );
        }
    }
    let wps: [[f32; 3]; 3] = [
        bridge.mission_waypoints()[0],
        bridge.mission_waypoints()[1],
        bridge.mission_waypoints()[2],
    ];

    // 2) Load the uploaded waypoints into the real supervisor and fly.
    let dt = 0.002f32;
    let level = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
    let mut bk = SimBackend::new(level, dt);
    bk.ground_contact = true;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 1.0);
    sup.set_mission_waypoints(bridge.mission_waypoints());
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, true);
    for _ in 0..8000 {
        sup.step(&mut bk);
    }
    assert_eq!(sup.mode(), Mode::Loiter);
    sup.command(Event::RequestMission, true, false);

    let mut min_d = [f32::MAX; 3];
    let mut disarmed = false;
    for _ in 0..160000 {
        sup.step(&mut bk);
        let p = sup.state().p;
        for (i, w) in wps.iter().enumerate() {
            let d = ((p[0] - w[0]).powi(2) + (p[1] - w[1]).powi(2) + (p[2] - w[2]).powi(2)).sqrt();
            if d < min_d[i] {
                min_d[i] = d;
            }
        }
        if sup.mode() == Mode::Disarmed {
            disarmed = true;
            break;
        }
    }
    for (i, d) in min_d.iter().enumerate() {
        assert!(
            *d < 1.5,
            "uploaded waypoint {i} not visited: min dist {d} m"
        );
    }
    assert!(
        disarmed,
        "uploaded mission must complete (mode {:?})",
        sup.mode()
    );
}
