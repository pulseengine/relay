//! falcon-mavlink-viz — the v1.28 release-video harness.
//!
//! This is the honest core of the "show the functionality" video: a
//! `SimBackend` flight that is **genuinely driven by a MAVLink command
//! timeline**. Each scheduled command is encoded exactly as a ground station
//! (QGroundControl / MAVSDK) would put it on the wire, ingested through the
//! real `falcon-mavlink` bridge, and the resulting `relay-fsm` event is fed to
//! the `FlightSupervisor`. The aircraft you see in the video is the supervisor
//! flying the verified cascade in response — not a scripted animation.
//!
//! Every sampled frame also encodes the outbound HEARTBEAT + GLOBAL_POSITION_INT
//! through the bridge and decodes them back, so the telemetry HUD shows the
//! bridge's actual wire values, not hand-faked numbers.
//!
//! Output: one JSON object per line (JSONL) on stdout — consumed by
//! tools/render-mavlink-viz.py.
//!
//!   cargo run -p falcon-mavlink-viz --release > /tmp/mavlink-viz.jsonl

use falcon_core::{FlightSupervisor, KeepoutZone, SimBackend};
use falcon_mavlink::MavBridge;
use relay_fsm::Mode;
use relay_mavlink::{
    COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, COMMAND_LONG_PAYLOAD_LEN, CommandLong,
    FrameHeader, GLOBAL_POSITION_INT_CRC_EXTRA, GlobalPositionInt, HEARTBEAT_CRC_EXTRA, Heartbeat,
    MAGIC_V2, encode_frame, parse_frame,
};

const HZ: f32 = 1000.0;
const DT: f32 = 1.0 / HZ;
const FRAME_EVERY: usize = 33; // ~30 fps sampling of the 1 kHz loop
const MAX_STEPS: usize = 48_000; // 48 s

// Geodetic home the NED telemetry is projected against (a PX4-SITL-style spot).
const HOME_LAT_E7: i32 = 473_977_000;
const HOME_LON_E7: i32 = 85_456_000;
const HOME_ALT_MM: i32 = 488_000;

const VEH: u8 = 1;
const COMP: u8 = 1;
const GCS: u8 = 255;
const CRUISE: f32 = 3.0; // takeoff/loiter altitude (m AGL)

const IDENTITY: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// Encode a COMMAND_LONG as a GCS would frame it on the wire.
fn encode_cmd(cmd: &CommandLong) -> ([u8; 64], usize) {
    let payload = cmd.encode_payload();
    let header = FrameHeader {
        magic: MAGIC_V2,
        payload_len: COMMAND_LONG_PAYLOAD_LEN as u8,
        incompat_flags: 0,
        compat_flags: 0,
        sequence: 0,
        system_id: GCS,
        component_id: 190,
        message_id: COMMAND_LONG_MSG_ID,
    };
    let mut buf = [0u8; 64];
    let n = encode_frame(&header, &payload, COMMAND_LONG_CRC_EXTRA, &mut buf).expect("encode cmd");
    (buf, n)
}

fn mode_str(m: Mode) -> &'static str {
    match m {
        Mode::Disarmed => "DISARMED",
        Mode::Armed => "ARMED",
        Mode::Takeoff => "TAKEOFF",
        Mode::Loiter => "LOITER",
        Mode::Mission => "MISSION",
        Mode::Land => "LAND",
        Mode::Rtl => "RTL",
    }
}

fn main() {
    // Scenario selector (argv[1]):
    //   "rtl"  (default) — spawn off home, RTL is a visible lateral return then
    //                      land (the v1.28 MAVLink-bridge video).
    //   "land"           — hover at home, NAV_LAND a crisp constant-rate vertical
    //                      descent to touchdown (the v1.29 supervisor-landing video).
    //   "mission"        — MISSION_START flies a stored multi-leg waypoint path,
    //                      then autonomously returns + lands (the v1.30 sequencer).
    //   "avoid"          — a mission whose path runs through a keep-out zone; the
    //                      vehicle arcs AROUND it and back (the v1.31 avoidance).
    let scenario = std::env::args().nth(1).unwrap_or_else(|| "rtl".into());
    let land_demo = scenario == "land";
    let avoid_demo = scenario == "avoid";
    // both mission + avoid fly a MISSION_START sortie from home at 2 m
    let mission_demo = scenario == "mission" || avoid_demo;

    // The v1.30 mission legs (NED, 2 m AGL) — a non-collinear path so the
    // in-order sequencing is visible. Avoid uses one far leg straight across a
    // no-fly zone so the detour is obvious.
    let mission: &[[f32; 3]] = if avoid_demo {
        &[[9.0, 0.0, -2.0]]
    } else {
        &[[3.0, 0.0, -2.0], [3.0, 3.0, -2.0], [0.0, 3.0, -2.0]]
    };
    // v1.31 keep-out zone sitting on the avoid mission's path.
    let zone = KeepoutZone {
        center: [4.5, 0.0, -2.0],
        radius: 1.8,
    };

    let mut sim = SimBackend::new(IDENTITY, DT);
    // RTL demo spawns off home (so the return is visible); land + mission start
    // at home (mission flies out and back, land drops straight down).
    sim.pos = if land_demo || mission_demo {
        [0.0, 0.0, 0.0]
    } else {
        [3.0, -2.0, 0.0]
    };
    sim.ground_contact = true; // let the touchdown settle on the surface
    // Quadratic aerodynamic drag (v1.17): damps the position-loop ring so the
    // motion reads clean, not wobbly. Honest physics.
    sim.drag_quad = 0.3;

    let cruise = if mission_demo { 2.0 } else { CRUISE };
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, cruise, 1.0);
    if mission_demo {
        sup.set_mission_waypoints(mission);
    }
    if avoid_demo {
        sup.set_keepout_zones(&[zone]);
    }
    let mut bridge = MavBridge::new(VEH, COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    // The MAVLink command timeline (step index, ticker label, frame). Auto
    // milestones (ReachedAltitude → Loiter, ReachedHome → Land, Touchdown →
    // Disarmed, and v1.30 waypoint advance) are produced by the supervisor.
    let final_cmd = if mission_demo {
        (
            18_000,
            "MISSION_START",
            CommandLong::mission_start(VEH, COMP),
        )
    } else if land_demo {
        (22_000, "NAV_LAND", CommandLong::land(VEH, COMP))
    } else {
        (22_000, "RTL", CommandLong::rtl(VEH, COMP))
    };
    let takeoff_label = if mission_demo {
        "TAKEOFF 2 m"
    } else {
        "TAKEOFF 3 m"
    };
    let schedule: [(usize, &str, CommandLong); 3] = [
        (800, "ARM", CommandLong::arm_disarm(VEH, COMP, true)),
        (2500, takeoff_label, CommandLong::takeoff(VEH, COMP, cruise)),
        final_cmd,
    ];
    let mut sched_i = 0;

    // The land demo touches down ~27 s in; the mission is a longer sortie. End a
    // few seconds after touchdown so the clip isn't a long disarmed-on-ground tail.
    let max_steps = if mission_demo {
        130_000
    } else if land_demo {
        38_000
    } else {
        MAX_STEPS
    };

    let mut disarm_tail = 0usize;
    let mut flew = false;
    for step in 0..max_steps {
        let mut rx_label = "";
        if sched_i < schedule.len() && schedule[sched_i].0 == step {
            let (_, label, cmd) = schedule[sched_i];
            let (buf, n) = encode_cmd(&cmd);
            // Drive the supervisor through the REAL bridge decode.
            if let Ok(Some(ev)) = bridge.ingest(&buf[..n]) {
                sup.command(ev, true, true);
                rx_label = label;
            }
            sched_i += 1;
        }

        sup.step(&mut sim);

        // End a few seconds after the sortie completes (mission length varies).
        let mode = sup.mode();
        flew |= matches!(
            mode,
            Mode::Takeoff | Mode::Loiter | Mode::Mission | Mode::Rtl
        );
        if flew && mode == Mode::Disarmed {
            disarm_tail += 1;
            if disarm_tail > 2500 {
                break;
            }
        }

        if step % FRAME_EVERY == 0 || !rx_label.is_empty() {
            let st = sup.state();
            let t_ms = (step as f32 * DT * 1000.0) as u32;

            // Outbound telemetry, encoded + decoded through the real bridge.
            let mut hb = [0u8; 64];
            let hn = bridge.heartbeat(mode, &mut hb).expect("hb");
            let (hf, _) = parse_frame(&hb[..hn], HEARTBEAT_CRC_EXTRA).expect("parse hb");
            let hbd = Heartbeat::decode_payload(hf.payload).expect("decode hb");

            let mut gp = [0u8; 64];
            let gn = bridge
                .global_position(st.p, st.v, 0.0, t_ms, &mut gp)
                .expect("gp");
            let (gf, _) = parse_frame(&gp[..gn], GLOBAL_POSITION_INT_CRC_EXTRA).expect("parse gp");
            let gpd = GlobalPositionInt::decode_payload(gf.payload).expect("decode gp");

            println!(
                "{{\"t\":{:.3},\"mode\":\"{}\",\"rx\":\"{}\",\"px\":{:.3},\"py\":{:.3},\"pz\":{:.3},\"vx\":{:.3},\"vy\":{:.3},\"vz\":{:.3},\"hb_custom\":{},\"base_mode\":{},\"sys_status\":{},\"lat_e7\":{},\"lon_e7\":{},\"rel_alt_mm\":{},\"vz_cms\":{},\"hdg_cdeg\":{},\"wp\":{},\"wp_n\":{}}}",
                step as f32 * DT,
                mode_str(mode),
                rx_label,
                st.p[0],
                st.p[1],
                st.p[2],
                st.v[0],
                st.v[1],
                st.v[2],
                hbd.custom_mode,
                hbd.base_mode,
                hbd.system_status,
                gpd.lat_e7,
                gpd.lon_e7,
                gpd.relative_alt_mm,
                gpd.vz_cms,
                gpd.hdg_cdeg,
                sup.waypoint_index(),
                sup.waypoint_count()
            );
        }
    }
}
