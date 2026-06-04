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

use falcon_core::{FlightSupervisor, SimBackend};
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
    // Spawn offset from home so RTL is a visible lateral return-then-land.
    let mut sim = SimBackend::new(IDENTITY, DT);
    sim.pos = [3.0, -2.0, 0.0];
    sim.ground_contact = true; // let the touchdown settle on the surface
    // Quadratic aerodynamic drag (v1.17): damps the position-loop ring on the
    // RTL step so the return + descent read clean, not wobbly. Honest physics.
    sim.drag_quad = 0.3;

    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, CRUISE, 1.0);
    let mut bridge = MavBridge::new(VEH, COMP, HOME_LAT_E7, HOME_LON_E7, HOME_ALT_MM);

    // The MAVLink command timeline (step index, ticker label, frame). Auto
    // milestones (ReachedAltitude → Loiter, ReachedHome → Land, Touchdown →
    // Disarmed) are produced by the supervisor, not scheduled here.
    let schedule: [(usize, &str, CommandLong); 3] = [
        (800, "ARM", CommandLong::arm_disarm(VEH, COMP, true)),
        (2500, "TAKEOFF 3 m", CommandLong::takeoff(VEH, COMP, CRUISE)),
        (22_000, "RTL", CommandLong::rtl(VEH, COMP)),
    ];
    let mut sched_i = 0;

    for step in 0..MAX_STEPS {
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

        if step % FRAME_EVERY == 0 || !rx_label.is_empty() {
            let st = sup.state();
            let mode = sup.mode();
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
                "{{\"t\":{:.3},\"mode\":\"{}\",\"rx\":\"{}\",\"px\":{:.3},\"py\":{:.3},\"pz\":{:.3},\"vx\":{:.3},\"vy\":{:.3},\"vz\":{:.3},\"hb_custom\":{},\"base_mode\":{},\"sys_status\":{},\"lat_e7\":{},\"lon_e7\":{},\"rel_alt_mm\":{},\"vz_cms\":{},\"hdg_cdeg\":{}}}",
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
                gpd.hdg_cdeg
            );
        }
    }
}
