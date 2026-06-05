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
