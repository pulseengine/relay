//! MAVLink backend — reads `GLOBAL_POSITION_INT` frames from a real FC.
//!
//! ## Wiring
//!
//! ```text
//!   ┌─────────────────┐   USB/serial / UDP   ┌──────────────────┐
//!   │ Flight Ctrl     │ ───────────────────▶ │  this harness    │
//!   │ (PX4 / ArduPilot│   GLOBAL_POSITION_   │  via FrameSource │
//!   │  / etc.)        │   INT @ 10 Hz        │                  │
//!   └─────────────────┘                       └──────────────────┘
//! ```
//!
//! The harness consumes the FC's *reported* position (the one the
//! navigation stack derived from whatever GPS receiver is in front of
//! it — spoofed or real). The verified `Geofence::check` runs on this
//! reading. When a spoofer pushes the FC outside the fence, the latch
//! trips and `relay-sc` dispatches the RTL — the same chain the SITL
//! and the StubBench exercise.
//!
//! ## What's in software (this file)
//!
//!   * `FrameSource` — the trait that decouples MAVLink-frame supply
//!     from the bench. One implementation is in-memory (for tests);
//!     a real bench wires a UDP or serial source.
//!   * `UdpFrameSource` — a real `std::net::UdpSocket` reader. The
//!     code path is wired end-to-end; a real FC sending MAVLink to a
//!     known port works without further changes.
//!   * `Home` — lat/lon/alt of the launch site; the projection
//!     anchor for NED conversion. Equirectangular approximation is
//!     used (within the few-km HITL bench range the error is ≪ 1 cm).
//!   * `MavlinkBench` — implements `HitlBench` over a `FrameSource`.
//!
//! Hardware run is the user's bench job; everything above the FC's
//! USB/UDP boundary is exercised by `cargo test`.

use crate::harness::{CommandSink, HitlBench};
use relay_mavlink::{
    parse_frame, peek_message_id, GlobalPositionInt,
    GLOBAL_POSITION_INT_CRC_EXTRA, GLOBAL_POSITION_INT_MSG_ID,
    GLOBAL_POSITION_INT_PAYLOAD_LEN,
};
use std::net::{SocketAddr, UdpSocket};

/// Mean WGS-84 earth radius (m). Good enough for the equirectangular
/// projection over a few-km bench range.
const EARTH_R_M: f64 = 6_371_000.0;
const DEG_PER_E7: f64 = 1.0 / 1e7;
const PI: f64 = std::f64::consts::PI;

/// Launch-site anchor for the lat/lon → NED projection.
#[derive(Clone, Copy, Debug)]
pub struct Home {
    pub lat_e7: i32,
    pub lon_e7: i32,
    /// Home altitude in millimetres AMSL. Subtracted from the
    /// `alt_mm` field to recover NED `D` (down) above home.
    pub alt_mm: i32,
}

impl Home {
    /// Equirectangular projection of an `(lat_e7, lon_e7, alt_mm)`
    /// reading to local NED in centimetres. Down is positive.
    pub fn project_ned_cm(&self, lat_e7: i32, lon_e7: i32, alt_mm: i32) -> (i32, i32, i32) {
        // Difference in degrees, then radians.
        let d_lat_deg = (lat_e7 - self.lat_e7) as f64 * DEG_PER_E7;
        let d_lon_deg = (lon_e7 - self.lon_e7) as f64 * DEG_PER_E7;
        let home_lat_rad = (self.lat_e7 as f64 * DEG_PER_E7) * PI / 180.0;

        let north_m = d_lat_deg * (PI / 180.0) * EARTH_R_M;
        let east_m = d_lon_deg * (PI / 180.0) * EARTH_R_M * home_lat_rad.cos();
        // Down is positive — alt above home is negative D.
        let down_m = -((alt_mm - self.alt_mm) as f64 * 0.001);

        let n_cm = (north_m * 100.0) as i32;
        let e_cm = (east_m * 100.0) as i32;
        let d_cm = (down_m * 100.0) as i32;
        (n_cm, e_cm, d_cm)
    }
}

/// What `MavlinkBench` reads frames from. One method, returns a
/// borrowed slice of frame bytes or `None` when no frame is ready
/// for this tick.
pub trait FrameSource {
    /// Return the next available frame, or `None` if nothing is
    /// pending. The returned slice is valid until the next call.
    fn next_frame(&mut self) -> Option<&[u8]>;
    fn name(&self) -> &'static str;
}

/// In-memory `FrameSource` — for tests + the deterministic backend.
/// A pre-built script of MAVLink frame byte vectors.
pub struct InMemoryFrameSource {
    frames: Vec<Vec<u8>>,
    cursor: usize,
}

impl InMemoryFrameSource {
    pub fn new(frames: Vec<Vec<u8>>) -> Self {
        Self { frames, cursor: 0 }
    }
}

impl FrameSource for InMemoryFrameSource {
    fn name(&self) -> &'static str { "mem" }
    fn next_frame(&mut self) -> Option<&[u8]> {
        if self.cursor >= self.frames.len() {
            return None;
        }
        let f = &self.frames[self.cursor];
        self.cursor += 1;
        Some(f.as_slice())
    }
}

/// UDP `FrameSource` — reads non-blocking from a bound socket.
///
/// Important: PX4-SITL (and most MAVLink autopilots) **do not stream
/// telemetry to a passive listener**. The autopilot only starts
/// sending to a peer after it receives at least one MAVLink frame
/// from that peer (it learns the peer's address from the incoming
/// datagram). Without registration the harness's bench-run finishes
/// with `latched: false` + `spoof_first_seen_at_s: None` even though
/// the bind succeeded — diagnosed against PX4 jMAVSim on
/// 2026-05-25.
///
/// Use `new_with_registration(sock, peer)` to send a HEARTBEAT to
/// the autopilot's GCS listen port on construction and then a
/// periodic HEARTBEAT every second from `next_frame()`. The plain
/// `new(sock)` constructor keeps the old "no registration" behaviour
/// for callers wiring their own discovery path (e.g. unit tests
/// using `InMemoryFrameSource`-style scripts).
///
/// Typical use:
///
/// ```no_run
/// use std::net::UdpSocket;
/// use falcon_hitl_rfspoof::mavlink::UdpFrameSource;
///
/// // PX4 sends GCS stream to UDP 14550; bind there to receive it.
/// let sock = UdpSocket::bind("0.0.0.0:14550").unwrap();
/// sock.set_nonblocking(true).unwrap();
/// // PX4-SITL's normal-mode (GCS) MAVLink listens on UDP 18570.
/// // Sending a HEARTBEAT there registers us as a peer; PX4 then
/// // starts streaming GLOBAL_POSITION_INT etc. back to us on 14550.
/// let src = UdpFrameSource::new_with_registration(
///     sock, "127.0.0.1:18570".parse().unwrap()
/// );
/// ```
pub struct UdpFrameSource {
    sock: UdpSocket,
    buf: [u8; 1500],
    last_len: usize,
    /// If `Some`, send a HEARTBEAT here every `HEARTBEAT_INTERVAL`.
    /// Canonical MAVLink GCS behaviour — registers us with the
    /// autopilot and keeps the registration alive.
    register_peer: Option<std::net::SocketAddr>,
    last_heartbeat: Option<std::time::Instant>,
    heartbeat_seq: u8,
}

impl UdpFrameSource {
    /// HEARTBEAT cadence (canonical MAVLink GCS: 1 Hz).
    const HEARTBEAT_INTERVAL: std::time::Duration = std::time::Duration::from_secs(1);

    pub fn new(sock: UdpSocket) -> Self {
        Self {
            sock,
            buf: [0u8; 1500],
            last_len: 0,
            register_peer: None,
            last_heartbeat: None,
            heartbeat_seq: 0,
        }
    }

    /// New `UdpFrameSource` that registers with an autopilot. Sends
    /// a HEARTBEAT to `peer` on construction (so the autopilot learns
    /// our address immediately) and another every second from
    /// `next_frame()`.
    pub fn new_with_registration(sock: UdpSocket, peer: std::net::SocketAddr) -> Self {
        let mut s = Self::new(sock);
        s.register_peer = Some(peer);
        s.send_heartbeat();
        s
    }

    /// Send one MAVLink HEARTBEAT to the registration peer. No-op if
    /// no peer was configured. Errors are eaten silently — a missed
    /// HEARTBEAT just means the next call retries.
    fn send_heartbeat(&mut self) {
        use relay_mavlink::{
            encode_frame, FrameHeader, Heartbeat, HEADER_LEN,
            HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID, HEARTBEAT_PAYLOAD_LEN, MAGIC_V2,
        };
        let Some(peer) = self.register_peer else { return };
        let payload = Heartbeat::gcs().encode_payload();
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: payload.len() as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: self.heartbeat_seq,
            // GCS-conventional sysid/compid — matches QGroundControl's defaults.
            system_id: 255,
            component_id: 190,
            message_id: HEARTBEAT_MSG_ID,
        };
        let mut frame = [0u8; HEADER_LEN + HEARTBEAT_PAYLOAD_LEN + 2];
        if encode_frame(&header, &payload, HEARTBEAT_CRC_EXTRA, &mut frame).is_ok() {
            let _ = self.sock.send_to(&frame, peer);
        }
        self.heartbeat_seq = self.heartbeat_seq.wrapping_add(1);
        self.last_heartbeat = Some(std::time::Instant::now());
    }
}

impl FrameSource for UdpFrameSource {
    fn name(&self) -> &'static str { "udp" }
    fn next_frame(&mut self) -> Option<&[u8]> {
        // Keep the registration alive by sending a HEARTBEAT every
        // second when a peer is configured. PX4-SITL forgets peers
        // that go silent for too long.
        if self.register_peer.is_some() {
            let due = self
                .last_heartbeat
                .map(|t| t.elapsed() >= Self::HEARTBEAT_INTERVAL)
                .unwrap_or(true);
            if due {
                self.send_heartbeat();
            }
        }
        match self.sock.recv(&mut self.buf) {
            Ok(n) => {
                self.last_len = n;
                Some(&self.buf[..n])
            }
            Err(_) => None,
        }
    }
}

/// CommandSink that sends frames back over UDP to a peer address —
/// typically PX4-SITL's GCS listen port. Used to close the v0.14.2
/// round-trip: when relay-sc dispatches RTL, the harness encodes a
/// COMMAND_LONG and writes it here so the FC actually acts on it.
pub struct UdpCommandSink {
    sock: UdpSocket,
    peer: SocketAddr,
}

impl UdpCommandSink {
    pub fn new(sock: UdpSocket, peer: SocketAddr) -> Self {
        Self { sock, peer }
    }
}

impl CommandSink for UdpCommandSink {
    fn name(&self) -> &'static str { "udp" }
    fn send_frame(&mut self, bytes: &[u8]) -> Result<(), &'static str> {
        self.sock
            .send_to(bytes, self.peer)
            .map(|_| ())
            .map_err(|_| "udp send_to failed")
    }
}

/// `HitlBench` over a MAVLink frame source.
///
/// Decodes each new `GLOBAL_POSITION_INT` it sees and caches the
/// projected NED position. If a tick produces no new position, the
/// last cached one is reused — the verified `Geofence::check`
/// tolerates idempotent inputs (LC-P09 monotonicity).
pub struct MavlinkBench<S: FrameSource> {
    source: S,
    home: Home,
    last_n_cm: i32,
    last_e_cm: i32,
    last_d_cm: i32,
    spoof_active: bool,
    /// Time-since-start; the bench treats "any new position outside
    /// the home box" as evidence the spoof is active.
    t: f32,
    /// Initial NED estimate (used while we have no real fix yet).
    seeded: bool,
}

impl<S: FrameSource> MavlinkBench<S> {
    pub fn new(source: S, home: Home, initial_ned_cm: (i32, i32, i32)) -> Self {
        Self {
            source,
            home,
            last_n_cm: initial_ned_cm.0,
            last_e_cm: initial_ned_cm.1,
            last_d_cm: initial_ned_cm.2,
            spoof_active: false,
            t: 0.0,
            seeded: false,
        }
    }

    fn drain_frames(&mut self) {
        // Drain every pending frame this tick so the cached position
        // is the freshest one. The MAVLink rate (10 Hz typ.) is
        // independent of the harness tick rate (100 Hz).
        //
        // We *peek* the message-id first because parse_frame takes
        // crc_extra as input — the protocol is per-message CRC.
        // GLOBAL_POSITION_INT is the only message-id this bench
        // consumes; anything else is silently skipped.
        while let Some(bytes) = self.source.next_frame() {
            let mid = match peek_message_id(bytes) {
                Ok(m) => m,
                Err(_) => continue,
            };
            if mid != GLOBAL_POSITION_INT_MSG_ID {
                continue;
            }
            let (frame, _consumed) = match parse_frame(bytes, GLOBAL_POSITION_INT_CRC_EXTRA) {
                Ok(p) => p,
                Err(_) => continue, // bad CRC, truncated, etc. — silent skip
            };
            if frame.payload.len() != GLOBAL_POSITION_INT_PAYLOAD_LEN {
                continue;
            }
            let Some(msg) = GlobalPositionInt::decode_payload(frame.payload) else {
                continue;
            };
            let (n, e, d) = self.home.project_ned_cm(msg.lat_e7, msg.lon_e7, msg.alt_mm);
            // First fix seeds the bench; subsequent jumps > ~20 m are
            // flagged as "spoof active" — diagnostic only; the safety
            // path doesn't depend on this flag.
            if self.seeded {
                let dn = (n - self.last_n_cm).abs();
                let de = (e - self.last_e_cm).abs();
                if dn > 2_000 || de > 2_000 {
                    self.spoof_active = true;
                }
            }
            self.last_n_cm = n;
            self.last_e_cm = e;
            self.last_d_cm = d;
            self.seeded = true;
        }
    }
}

impl<S: FrameSource> HitlBench for MavlinkBench<S> {
    fn name(&self) -> &'static str { "mavlink" }
    fn step(&mut self, dt: f32) {
        self.t += dt;
        self.drain_frames();
    }
    fn position_cm(&self) -> (i32, i32, i32) {
        (self.last_n_cm, self.last_e_cm, self.last_d_cm)
    }
    fn spoof_active(&self) -> bool { self.spoof_active }
}

/// Build a MAVLink v2 frame carrying a GLOBAL_POSITION_INT payload.
/// Used by tests + by bench tooling to script trajectories.
pub fn build_global_position_frame(seq: u8, msg: &GlobalPositionInt) -> Vec<u8> {
    use relay_mavlink::{encode_frame, FrameHeader, GLOBAL_POSITION_INT_CRC_EXTRA, MAGIC_V2};
    let payload = msg.encode_payload();
    let header = FrameHeader {
        magic: MAGIC_V2,
        payload_len: payload.len() as u8,
        incompat_flags: 0,
        compat_flags: 0,
        sequence: seq,
        system_id: 1,
        component_id: 1,
        message_id: GLOBAL_POSITION_INT_MSG_ID,
    };
    let mut out = vec![0u8; relay_mavlink::HEADER_LEN + payload.len() + 2];
    let n = encode_frame(&header, &payload, GLOBAL_POSITION_INT_CRC_EXTRA, &mut out)
        .expect("encode_frame should fit in pre-sized buffer");
    out.truncate(n);
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::harness::{load_rtl_rts, run_scenario, NullCommandSink};
    use relay_lc::engine::Geofence;
    use relay_sc::engine::CommandStore;

    fn budapest_home() -> Home {
        // ~Budapest centre.
        Home { lat_e7: 475_023_456, lon_e7: 190_401_234, alt_mm: 120_000 }
    }

    fn pos_at(home: Home, n_cm: i32, e_cm: i32, d_cm: i32) -> GlobalPositionInt {
        // Inverse of project_ned_cm — convert NED cm to MAVLink fields.
        let home_lat_rad = (home.lat_e7 as f64 * DEG_PER_E7) * PI / 180.0;
        let north_m = n_cm as f64 / 100.0;
        let east_m = e_cm as f64 / 100.0;
        let d_lat_deg = north_m / EARTH_R_M * 180.0 / PI;
        let d_lon_deg = east_m / (EARTH_R_M * home_lat_rad.cos()) * 180.0 / PI;
        let lat_e7 = home.lat_e7 + (d_lat_deg / DEG_PER_E7) as i32;
        let lon_e7 = home.lon_e7 + (d_lon_deg / DEG_PER_E7) as i32;
        // Down is positive — alt above home is negative D.
        let alt_mm = home.alt_mm + ((-d_cm as f64) * 10.0) as i32;
        GlobalPositionInt {
            time_boot_ms: 0,
            lat_e7, lon_e7, alt_mm,
            relative_alt_mm: -d_cm * 10,
            vx_cms: 0, vy_cms: 0, vz_cms: 0,
            hdg_cdeg: 0,
        }
    }

    #[test]
    fn home_projects_to_origin() {
        let home = budapest_home();
        let (n, e, d) = home.project_ned_cm(home.lat_e7, home.lon_e7, home.alt_mm);
        assert_eq!((n, e, d), (0, 0, 0));
    }

    #[test]
    fn one_hundred_metres_north_projects_to_10000_cm_n() {
        let home = budapest_home();
        let msg = pos_at(home, 10_000, 0, -500);
        let (n, e, _d) = home.project_ned_cm(msg.lat_e7, msg.lon_e7, msg.alt_mm);
        // Equirectangular is exact at zero lon-offset; allow ±1 cm
        // rounding from the lat_e7 truncation.
        assert!((n - 10_000).abs() <= 1, "n_cm = {}", n);
        assert!(e.abs() <= 1, "e_cm = {}", e);
    }

    #[test]
    fn altitude_translates_to_down() {
        let home = budapest_home();
        // 5 m below home (down +5 m).
        let msg = pos_at(home, 0, 0, 500);
        let (_n, _e, d) = home.project_ned_cm(msg.lat_e7, msg.lon_e7, msg.alt_mm);
        assert!((d - 500).abs() <= 1, "d_cm = {}", d);
    }

    /// End-to-end MAVLink → bench → harness: a script of three frames
    /// (hover, hover, snap-east-200 m) must trip the latch + dispatch
    /// RTL within budget. Mirrors the StubBench positive control.
    #[test]
    fn mavlink_bench_trips_and_dispatches_on_spoof() {
        let home = budapest_home();
        let frames: Vec<Vec<u8>> = vec![
            build_global_position_frame(0, &pos_at(home, 0, 0, -500)),     // t=0 in-fence
            build_global_position_frame(1, &pos_at(home, 0, 0, -500)),     // t=1 in-fence
            build_global_position_frame(2, &pos_at(home, 0, 20_000, -500)), // t=2+ outside
        ];
        let src = InMemoryFrameSource::new(frames);
        let mut bench = MavlinkBench::new(src, home, (0, 0, -500));

        let mut fence = Geofence::new(-10_000, 10_000, -10_000, 10_000, -10_000, 10_000);
        let mut sc = CommandStore::new();
        load_rtl_rts(&mut sc, 0, 0xA17C);

        let mut sink = NullCommandSink::new();
        let v = run_scenario(&mut bench, &mut fence, &mut sc, &mut sink, 1.0, 5.0, 0, 2.0);

        assert!(v.pass(), "verdict = {:?}", v);
        assert!(v.latched);
        assert!(v.rtl_dispatched);
        assert!(v.rtl_frame_sent);
        assert_eq!(sink.frames_sent, 1);
    }

    /// Negative control: every frame stays inside the fence — must
    /// never latch. Catches the same class of bug as the stub
    /// negative-control test.
    #[test]
    fn mavlink_bench_no_violation_no_latch() {
        let home = budapest_home();
        let frames: Vec<Vec<u8>> = (0..5u8)
            .map(|i| build_global_position_frame(i, &pos_at(home, 0, 100 * i as i32, -500)))
            .collect();
        let src = InMemoryFrameSource::new(frames);
        let mut bench = MavlinkBench::new(src, home, (0, 0, -500));

        let mut fence = Geofence::new(-10_000, 10_000, -10_000, 10_000, -10_000, 10_000);
        let mut sc = CommandStore::new();
        load_rtl_rts(&mut sc, 0, 0xA17C);

        let mut sink = NullCommandSink::new();
        let v = run_scenario(&mut bench, &mut fence, &mut sc, &mut sink, 1.0, 5.0, 0, 10.0);
        assert!(!v.latched);
        assert!(!v.rtl_dispatched);
        assert!(!v.rtl_frame_sent);
        assert_eq!(sink.frames_sent, 0);
        assert!(v.failure.is_none());
    }

    /// Pin the wire-shape of one of our built frames so silent
    /// MAVLink-encoder drift in relay-mavlink surfaces here.
    #[test]
    fn build_frame_carries_correct_message_id() {
        let home = budapest_home();
        let bytes = build_global_position_frame(7, &pos_at(home, 0, 0, -500));
        // Frame is MAGIC_V2 then payload_len then …
        assert_eq!(bytes[0], 0xFD);
        assert_eq!(bytes[1] as usize, GLOBAL_POSITION_INT_PAYLOAD_LEN);
        // message_id is 3 bytes LE starting at offset 7.
        let mid = u32::from_le_bytes([bytes[7], bytes[8], bytes[9], 0]);
        assert_eq!(mid, GLOBAL_POSITION_INT_MSG_ID);
    }
}
