//! Falcon MAVLink bridge — the translation seam between the MAVLink v2
//! wire (QGroundControl / MAVSDK / PX4-ecosystem GCS) and falcon's flight
//! supervisor.
//!
//! The codec (`relay-mavlink`) and the flight-mode state machine
//! (`relay-fsm`) are both pure and independently verified. This crate adds
//! only the *translation*, in two directions:
//!
//!   inbound   COMMAND_LONG frame  ──►  relay_fsm::Event
//!             (arm/disarm, takeoff, land, RTL, flight-termination)
//!
//!   outbound  flight Mode         ──►  HEARTBEAT frame
//!             NED estimator state ──►  GLOBAL_POSITION_INT frame
//!
//! Keeping the bridge thin is deliberate: every byte-level invariant lives
//! in the verified codec, every safety transition lives in the verified
//! FSM, and this crate is small enough to read in one sitting.
//!
//! ## What's verified / tested (SWREQ-FALCON-MAVBRIDGE-P01)
//!
//!   MAVBRIDGE-P01: a stream of decoded COMMAND_LONGs (arm → takeoff →
//!                  reached-altitude → RTL → reached-home → touchdown)
//!                  drives the FSM through the nominal flight lifecycle.
//!   MAVBRIDGE-P02: `ingest` never panics on arbitrary bytes (it returns
//!                  `Ok(None)` or a `CodecError`, never unwinds).
//!   MAVBRIDGE-P03: HEARTBEAT and GLOBAL_POSITION_INT emitted by the bridge
//!                  re-parse to the values the bridge intended (round-trip).
//!   MAVBRIDGE-P04: the NED→geodetic projection places a point N metres
//!                  north / E metres east at the expected lat/lon to within
//!                  telemetry tolerance, and is sign-correct on altitude
//!                  (NED down is negative up).

#![no_std]
#![forbid(unsafe_code)]

use relay_fsm::{Event, Mode};
use relay_mavlink::{
    COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID, CodecError, CommandLong, FALCON_AUTOPILOT_ID,
    FrameHeader, GLOBAL_POSITION_INT_CRC_EXTRA, GLOBAL_POSITION_INT_MSG_ID, GlobalPositionInt,
    HEARTBEAT_CRC_EXTRA, HEARTBEAT_MSG_ID, Heartbeat, MAGIC_V2, MAV_CMD_COMPONENT_ARM_DISARM,
    MAV_CMD_DO_FLIGHTTERMINATION, MAV_CMD_MISSION_START, MAV_CMD_NAV_LAND,
    MAV_CMD_NAV_RETURN_TO_LAUNCH, MAV_CMD_NAV_TAKEOFF, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    MAV_MISSION_ACCEPTED, MISSION_ACK_CRC_EXTRA, MISSION_ACK_MSG_ID, MISSION_COUNT_CRC_EXTRA,
    MISSION_COUNT_MSG_ID, MISSION_ITEM_INT_CRC_EXTRA, MISSION_ITEM_INT_MSG_ID,
    MISSION_ITEM_INT_PAYLOAD_LEN, MISSION_REQUEST_INT_CRC_EXTRA, MISSION_REQUEST_INT_MSG_ID,
    MISSION_REQUEST_LIST_CRC_EXTRA, MISSION_REQUEST_LIST_MSG_ID, MavModeFlag, MavState, MavType,
    MissionAck, MissionCount, MissionItemInt, MissionRequestInt, MissionRequestList, encode_frame,
    parse_frame, peek_message_id,
};

/// Metres per degree of latitude (WGS-84 mean). Latitude scale is very
/// nearly constant with latitude, so a single mean is fine for the
/// local-tangent telemetry projection.
const M_PER_DEG_LAT: f64 = 111_320.0;

// ----------------------------------------------------------------------
// Falcon custom-mode codes (the `custom_mode` field of HEARTBEAT).
//
// MAVLink leaves `custom_mode` autopilot-defined; falcon publishes a flat
// 1:1 code so a GCS can name the active flight mode. (PX4 packs main/sub
// modes into the high bytes; falcon doesn't need that yet.)
// ----------------------------------------------------------------------

/// `custom_mode` value for each falcon flight mode.
pub const fn custom_mode_code(mode: Mode) -> u32 {
    match mode {
        Mode::Disarmed => 0,
        Mode::Armed => 1,
        Mode::Takeoff => 2,
        Mode::Loiter => 3,
        Mode::Mission => 4,
        Mode::Land => 5,
        Mode::Rtl => 6,
    }
}

// ----------------------------------------------------------------------
// Inbound: COMMAND_LONG → Event
// ----------------------------------------------------------------------

/// Translate a decoded COMMAND_LONG into the flight-FSM event it requests,
/// or `None` if falcon does not act on that command.
///
/// This is intentionally a *total, pure* function over the command id +
/// param1, so it can be exhaustively tested without any framing or I/O.
pub fn command_to_event(cmd: &CommandLong) -> Option<Event> {
    match cmd.command {
        // ARM_DISARM carries the arm/disarm choice in param1 (>=0.5 = arm).
        MAV_CMD_COMPONENT_ARM_DISARM => Some(if cmd.param1 >= 0.5 {
            Event::Arm
        } else {
            Event::RequestDisarm
        }),
        MAV_CMD_NAV_TAKEOFF => Some(Event::RequestTakeoff),
        MAV_CMD_NAV_LAND => Some(Event::RequestLand),
        MAV_CMD_NAV_RETURN_TO_LAUNCH => Some(Event::RequestRtl),
        MAV_CMD_MISSION_START => Some(Event::RequestMission),
        // Flight-termination is a last-resort failsafe; param1>=0.5 = engage.
        MAV_CMD_DO_FLIGHTTERMINATION => {
            if cmd.param1 >= 0.5 {
                Some(Event::Failsafe)
            } else {
                None
            }
        }
        _ => None,
    }
}

// ----------------------------------------------------------------------
// Outbound: Mode → HEARTBEAT
// ----------------------------------------------------------------------

/// Build the HEARTBEAT that advertises the given flight mode.
///
/// `base_mode` carries the standard MAVLink flags a GCS reads at a glance:
/// `CUSTOM_MODE_ENABLED` (always — the real mode is in `custom_mode`),
/// `SAFETY_ARMED` whenever the motors are live, and a coarse
/// guided/auto/stabilize hint. `system_status` is `STANDBY` on the ground
/// disarmed, `ACTIVE` once armed.
pub fn mode_to_heartbeat(mode: Mode) -> Heartbeat {
    let mut base = MavModeFlag::CUSTOM_MODE_ENABLED;
    if !matches!(mode, Mode::Disarmed) {
        base = base.or(MavModeFlag::SAFETY_ARMED);
    }
    base = base.or(match mode {
        // On the ground: no navigation flag.
        Mode::Disarmed | Mode::Armed => MavModeFlag::CUSTOM_MODE_ENABLED,
        // Operator-commanded holds: guided + stabilized.
        Mode::Takeoff | Mode::Loiter => {
            MavModeFlag::GUIDED_ENABLED.or(MavModeFlag::STABILIZE_ENABLED)
        }
        // Autonomous nav (mission / return / autoland): auto + stabilized.
        Mode::Mission | Mode::Rtl | Mode::Land => {
            MavModeFlag::AUTO_ENABLED.or(MavModeFlag::STABILIZE_ENABLED)
        }
    });
    let system_status = if matches!(mode, Mode::Disarmed) {
        MavState::Standby
    } else {
        MavState::Active
    };
    Heartbeat {
        custom_mode: custom_mode_code(mode),
        mav_type: MavType::Quadrotor as u8,
        autopilot: FALCON_AUTOPILOT_ID,
        base_mode: base.bits(),
        system_status: system_status as u8,
        mavlink_version: 2,
    }
}

// ----------------------------------------------------------------------
// Outbound: NED estimator state → GLOBAL_POSITION_INT
// ----------------------------------------------------------------------

/// Saturating f32→i16 cast (cm/s velocities are well inside i16 range in
/// normal flight; the clamp keeps a wild estimate from wrapping).
fn sat_i16(x: f32) -> i16 {
    if x >= i16::MAX as f32 {
        i16::MAX
    } else if x <= i16::MIN as f32 {
        i16::MIN
    } else {
        x as i32 as i16
    }
}

/// Yaw (rad, NED: 0 = north, clockwise +) → heading centidegrees [0, 35999].
fn yaw_to_cdeg(yaw_rad: f32) -> u16 {
    let deg = yaw_rad * (180.0 / core::f32::consts::PI);
    let mut d = deg % 360.0;
    if d < 0.0 {
        d += 360.0;
    }
    let cdeg = (d * 100.0) as u32;
    // Guard the [0, 35999] contract against the 360.0 boundary.
    if cdeg > 35_999 { 35_999 } else { cdeg as u16 }
}

/// Project a local NED state (relative to a geodetic home) into a
/// GLOBAL_POSITION_INT. Flat-earth / local-tangent: exact enough for
/// telemetry over the few-km radius a multirotor flies.
///
/// `m_per_deg_lon` is `M_PER_DEG_LAT * cos(home_lat)`, precomputed once at
/// bridge construction so the hot path stays free of transcendentals.
#[allow(clippy::too_many_arguments)]
pub fn ned_to_global_position(
    home_lat_e7: i32,
    home_lon_e7: i32,
    home_alt_mm: i32,
    m_per_deg_lon: f64,
    p_ned: [f32; 3],
    v_ned: [f32; 3],
    yaw_rad: f32,
    time_boot_ms: u32,
) -> GlobalPositionInt {
    let north = p_ned[0] as f64;
    let east = p_ned[1] as f64;
    let down = p_ned[2] as f64;

    let dlat_e7 = (north / M_PER_DEG_LAT * 1.0e7) as i32;
    let dlon_e7 = (east / m_per_deg_lon * 1.0e7) as i32;
    // NED down is positive-down; altitude-up = -down.
    let rel_alt_mm = (-down * 1000.0) as i32;

    GlobalPositionInt {
        time_boot_ms,
        lat_e7: home_lat_e7.saturating_add(dlat_e7),
        lon_e7: home_lon_e7.saturating_add(dlon_e7),
        alt_mm: home_alt_mm.saturating_add(rel_alt_mm),
        relative_alt_mm: rel_alt_mm,
        vx_cms: sat_i16(v_ned[0] * 100.0),
        vy_cms: sat_i16(v_ned[1] * 100.0),
        vz_cms: sat_i16(v_ned[2] * 100.0),
        hdg_cdeg: yaw_to_cdeg(yaw_rad),
    }
}

// ----------------------------------------------------------------------
// The bridge object
// ----------------------------------------------------------------------

/// Maximum mission items the uploader accumulates (no_std fixed capacity;
/// matches the FlightSupervisor's MAX_WAYPOINTS).
pub const MAX_MISSION_ITEMS: usize = 16;

/// Result of feeding one frame to [`MavBridge::ingest_mission`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionUpload {
    /// Not a mission-upload message (or addressed to another vehicle).
    Ignored,
    /// A `MISSION_REQUEST_INT` was written to `out` (n bytes) — send it to ask
    /// the GCS for the next item.
    Request(usize),
    /// All items received: a `MISSION_ACK(ACCEPTED)` was written to `out` (n
    /// bytes) and the mission is loaded — read it with `mission_waypoints()`.
    Complete(usize),
    /// The upload was rejected: a `MISSION_ACK` with an error was written.
    Rejected(usize),
}

/// Result of feeding one frame to [`MavBridge::serve_mission`] (the DOWNLOAD
/// direction: the vehicle serves its stored mission back to a GCS).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MissionDownload {
    /// Not a mission-download request (or addressed to another vehicle).
    Ignored,
    /// A `MISSION_COUNT` was written to `out` (n bytes) — the reply to a
    /// `MISSION_REQUEST_LIST`.
    Count(usize),
    /// A `MISSION_ITEM_INT` was written to `out` (n bytes) — the reply to a
    /// `MISSION_REQUEST_INT(seq)`.
    Item(usize),
    /// The GCS sent a `MISSION_ACK` — the download is complete.
    Done,
}

/// Stateful MAVLink endpoint for a single falcon vehicle: holds its
/// identity, an outbound frame sequence counter, the geodetic home the NED
/// telemetry is projected against, and the in-progress mission upload.
#[derive(Clone, Copy, Debug)]
pub struct MavBridge {
    system_id: u8,
    component_id: u8,
    seq: u8,
    home_lat_e7: i32,
    home_lon_e7: i32,
    home_alt_mm: i32,
    m_per_deg_lon: f64,
    // ── mission-upload state (v1.33) ──
    up_expected: u16,
    up_next: u16,
    up_count: usize,
    up_active: bool,
    up_gcs_sys: u8,
    up_gcs_comp: u8,
    up_wps: [[f32; 3]; MAX_MISSION_ITEMS],
}

impl MavBridge {
    /// Create a bridge for the vehicle `system_id`/`component_id`, with the
    /// geodetic home (degrees×1e7, mm AMSL) that NED telemetry is relative
    /// to. The longitude metres-per-degree is computed once here (the only
    /// transcendental call) and reused for every outbound position.
    pub fn new(
        system_id: u8,
        component_id: u8,
        home_lat_e7: i32,
        home_lon_e7: i32,
        home_alt_mm: i32,
    ) -> Self {
        let lat_rad = (home_lat_e7 as f64 / 1.0e7) * (core::f64::consts::PI / 180.0);
        let m = M_PER_DEG_LAT * relay_math::cosf(lat_rad as f32) as f64;
        // Near the poles cos→0; clamp so the east projection can't blow up
        // or divide by zero. (A multirotor never flies there, but the bridge
        // stays total.)
        let m_per_deg_lon = if m < 1.0 { 1.0 } else { m };
        Self {
            system_id,
            component_id,
            seq: 0,
            home_lat_e7,
            home_lon_e7,
            home_alt_mm,
            m_per_deg_lon,
            up_expected: 0,
            up_next: 0,
            up_count: 0,
            up_active: false,
            up_gcs_sys: 0,
            up_gcs_comp: 0,
            up_wps: [[0.0; 3]; MAX_MISSION_ITEMS],
        }
    }

    /// This vehicle's MAVLink system id.
    pub fn system_id(&self) -> u8 {
        self.system_id
    }

    /// Decode one inbound frame and, if it is a COMMAND_LONG addressed to
    /// this vehicle (or broadcast, `target_system == 0`), return the flight
    /// event it requests.
    ///
    /// Returns:
    ///   `Ok(Some(ev))` — a command we act on,
    ///   `Ok(None)`     — a valid frame we don't route (other message, other
    ///                    vehicle, or an unmapped command),
    ///   `Err(e)`       — the bytes are not a valid frame.
    ///
    /// Never panics on arbitrary input.
    pub fn ingest(&self, buf: &[u8]) -> Result<Option<Event>, CodecError> {
        let msg_id = peek_message_id(buf)?;
        if msg_id != COMMAND_LONG_MSG_ID {
            return Ok(None);
        }
        let (frame, _consumed) = parse_frame(buf, COMMAND_LONG_CRC_EXTRA)?;
        let cmd = CommandLong::decode_payload(frame.payload).ok_or(CodecError::BadPayloadLength)?;
        if cmd.target_system != 0 && cmd.target_system != self.system_id {
            return Ok(None);
        }
        Ok(command_to_event(&cmd))
    }

    /// Drive the MAVLink mission-UPLOAD protocol (GCS → vehicle). Feed each
    /// inbound frame: on `MISSION_COUNT` the uploader starts and requests item
    /// 0; each `MISSION_ITEM_INT` is decoded into an NED waypoint and the next
    /// item is requested, until all are in and a `MISSION_ACK(ACCEPTED)` is
    /// emitted. The protocol responses are written to `out`; on `Complete` the
    /// uploaded mission is available via [`mission_waypoints`](Self::mission_waypoints).
    ///
    /// Never panics on arbitrary input.
    pub fn ingest_mission(&mut self, buf: &[u8], out: &mut [u8]) -> MissionUpload {
        let msg_id = match peek_message_id(buf) {
            Ok(id) => id,
            Err(_) => return MissionUpload::Ignored,
        };
        match msg_id {
            MISSION_COUNT_MSG_ID => {
                let frame = match parse_frame(buf, MISSION_COUNT_CRC_EXTRA) {
                    Ok((f, _)) => f,
                    Err(_) => return MissionUpload::Ignored,
                };
                let mc = match MissionCount::decode_payload(frame.payload) {
                    Some(m) => m,
                    None => return MissionUpload::Ignored,
                };
                if mc.target_system != 0 && mc.target_system != self.system_id {
                    return MissionUpload::Ignored;
                }
                // Start a fresh upload; remember who to reply to.
                self.up_gcs_sys = frame.header.system_id;
                self.up_gcs_comp = frame.header.component_id;
                self.up_expected = mc.count.min(MAX_MISSION_ITEMS as u16);
                self.up_next = 0;
                self.up_count = 0;
                self.up_active = true;
                if self.up_expected == 0 {
                    self.up_active = false;
                    return self.send_ack(MAV_MISSION_ACCEPTED, out);
                }
                self.send_request(0, out)
            }
            MISSION_ITEM_INT_MSG_ID => {
                if !self.up_active {
                    return MissionUpload::Ignored;
                }
                let frame = match parse_frame(buf, MISSION_ITEM_INT_CRC_EXTRA) {
                    Ok((f, _)) => f,
                    Err(_) => return MissionUpload::Ignored,
                };
                let item = match MissionItemInt::decode_payload(frame.payload) {
                    Some(i) => i,
                    None => return MissionUpload::Ignored,
                };
                // Out-of-sequence: re-request the one we still need (idempotent).
                if item.seq != self.up_next {
                    return self.send_request(self.up_next, out);
                }
                // Geodetic → NED, relative to home (the inverse of the outbound
                // GLOBAL_POSITION_INT projection). z is altitude-up; NED is down.
                let north = (item.x - self.home_lat_e7) as f64 / 1.0e7 * M_PER_DEG_LAT;
                let east = (item.y - self.home_lon_e7) as f64 / 1.0e7 * self.m_per_deg_lon;
                self.up_wps[self.up_count] = [north as f32, east as f32, -item.z];
                self.up_count += 1;
                self.up_next += 1;
                if self.up_next < self.up_expected {
                    self.send_request(self.up_next, out)
                } else {
                    self.up_active = false;
                    self.send_ack(MAV_MISSION_ACCEPTED, out)
                }
            }
            _ => MissionUpload::Ignored,
        }
    }

    /// The NED waypoints from the most recent completed upload — feed straight
    /// to `FlightSupervisor::set_mission_waypoints`.
    pub fn mission_waypoints(&self) -> &[[f32; 3]] {
        &self.up_wps[..self.up_count]
    }

    /// Set the mission the bridge will serve on DOWNLOAD (e.g. the waypoints the
    /// FlightSupervisor is currently flying). Stored as the same NED waypoints an
    /// upload would leave; a download projects them back to geodetic.
    pub fn set_downloadable_mission(&mut self, wps: &[[f32; 3]]) {
        let n = wps.len().min(MAX_MISSION_ITEMS);
        self.up_wps[..n].copy_from_slice(&wps[..n]);
        self.up_count = n;
    }

    /// Drive the MAVLink mission-DOWNLOAD protocol (vehicle → GCS), the mirror of
    /// [`ingest_mission`](Self::ingest_mission). Feed each inbound frame:
    /// `MISSION_REQUEST_LIST` → reply `MISSION_COUNT`; `MISSION_REQUEST_INT(seq)`
    /// → reply `MISSION_ITEM_INT(seq)` (the stored NED waypoint projected back to
    /// geodetic about home); `MISSION_ACK` → the download is complete. Replies
    /// are written to `out`. Never panics on arbitrary input.
    pub fn serve_mission(&mut self, buf: &[u8], out: &mut [u8]) -> MissionDownload {
        let msg_id = match peek_message_id(buf) {
            Ok(id) => id,
            Err(_) => return MissionDownload::Ignored,
        };
        match msg_id {
            MISSION_REQUEST_LIST_MSG_ID => {
                let frame = match parse_frame(buf, MISSION_REQUEST_LIST_CRC_EXTRA) {
                    Ok((f, _)) => f,
                    Err(_) => return MissionDownload::Ignored,
                };
                let req = match MissionRequestList::decode_payload(frame.payload) {
                    Some(r) => r,
                    None => return MissionDownload::Ignored,
                };
                if req.target_system != 0 && req.target_system != self.system_id {
                    return MissionDownload::Ignored;
                }
                self.up_gcs_sys = frame.header.system_id;
                self.up_gcs_comp = frame.header.component_id;
                let count = MissionCount {
                    count: self.up_count as u16,
                    target_system: self.up_gcs_sys,
                    target_component: self.up_gcs_comp,
                };
                match self.frame_out(
                    MISSION_COUNT_MSG_ID,
                    &count.encode_payload(),
                    MISSION_COUNT_CRC_EXTRA,
                    out,
                ) {
                    Ok(n) => MissionDownload::Count(n),
                    Err(_) => MissionDownload::Ignored,
                }
            }
            MISSION_REQUEST_INT_MSG_ID => {
                let frame = match parse_frame(buf, MISSION_REQUEST_INT_CRC_EXTRA) {
                    Ok((f, _)) => f,
                    Err(_) => return MissionDownload::Ignored,
                };
                let req = match MissionRequestInt::decode_payload(frame.payload) {
                    Some(r) => r,
                    None => return MissionDownload::Ignored,
                };
                let seq = req.seq as usize;
                if seq >= self.up_count {
                    return MissionDownload::Ignored;
                }
                let wp = self.up_wps[seq];
                // NED → geodetic (the inverse of the inbound upload projection).
                let lat_e7 = self.home_lat_e7 + (wp[0] as f64 / M_PER_DEG_LAT * 1.0e7) as i32;
                let lon_e7 = self.home_lon_e7 + (wp[1] as f64 / self.m_per_deg_lon * 1.0e7) as i32;
                let item = MissionItemInt {
                    param1: 0.0,
                    param2: 0.0,
                    param3: 0.0,
                    param4: 0.0,
                    x: lat_e7,
                    y: lon_e7,
                    z: -wp[2], // NED down → altitude-up
                    seq: req.seq,
                    command: 16, // MAV_CMD_NAV_WAYPOINT
                    target_system: self.up_gcs_sys,
                    target_component: self.up_gcs_comp,
                    frame: MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                    current: 0,
                    autocontinue: 1,
                };
                let payload = item.encode_payload();
                debug_assert_eq!(payload.len(), MISSION_ITEM_INT_PAYLOAD_LEN);
                match self.frame_out(
                    MISSION_ITEM_INT_MSG_ID,
                    &payload,
                    MISSION_ITEM_INT_CRC_EXTRA,
                    out,
                ) {
                    Ok(n) => MissionDownload::Item(n),
                    Err(_) => MissionDownload::Ignored,
                }
            }
            MISSION_ACK_MSG_ID => MissionDownload::Done,
            _ => MissionDownload::Ignored,
        }
    }

    fn send_request(&mut self, seq: u16, out: &mut [u8]) -> MissionUpload {
        let req = MissionRequestInt {
            seq,
            target_system: self.up_gcs_sys,
            target_component: self.up_gcs_comp,
        };
        match self.frame_out(
            MISSION_REQUEST_INT_MSG_ID,
            &req.encode_payload(),
            MISSION_REQUEST_INT_CRC_EXTRA,
            out,
        ) {
            Ok(n) => MissionUpload::Request(n),
            Err(_) => MissionUpload::Rejected(0),
        }
    }

    fn send_ack(&mut self, mav_type: u8, out: &mut [u8]) -> MissionUpload {
        let ack = MissionAck {
            target_system: self.up_gcs_sys,
            target_component: self.up_gcs_comp,
            mav_type,
        };
        match self.frame_out(
            MISSION_ACK_MSG_ID,
            &ack.encode_payload(),
            MISSION_ACK_CRC_EXTRA,
            out,
        ) {
            Ok(n) if mav_type == MAV_MISSION_ACCEPTED => MissionUpload::Complete(n),
            Ok(n) => MissionUpload::Rejected(n),
            Err(_) => MissionUpload::Rejected(0),
        }
    }

    /// Encode a HEARTBEAT for `mode` into `out`; returns bytes written.
    pub fn heartbeat(&mut self, mode: Mode, out: &mut [u8]) -> Result<usize, CodecError> {
        let payload = mode_to_heartbeat(mode).encode_payload();
        self.frame_out(HEARTBEAT_MSG_ID, &payload, HEARTBEAT_CRC_EXTRA, out)
    }

    /// Encode a GLOBAL_POSITION_INT for the given NED state into `out`;
    /// returns bytes written. `yaw_rad` is the body heading (0 = north).
    pub fn global_position(
        &mut self,
        p_ned: [f32; 3],
        v_ned: [f32; 3],
        yaw_rad: f32,
        time_boot_ms: u32,
        out: &mut [u8],
    ) -> Result<usize, CodecError> {
        let gpi = ned_to_global_position(
            self.home_lat_e7,
            self.home_lon_e7,
            self.home_alt_mm,
            self.m_per_deg_lon,
            p_ned,
            v_ned,
            yaw_rad,
            time_boot_ms,
        );
        let payload = gpi.encode_payload();
        self.frame_out(
            GLOBAL_POSITION_INT_MSG_ID,
            &payload,
            GLOBAL_POSITION_INT_CRC_EXTRA,
            out,
        )
    }

    /// Frame a payload with this endpoint's identity + the next sequence
    /// number, advancing the counter (wrapping at 256 per MAVLink).
    fn frame_out(
        &mut self,
        msg_id: u32,
        payload: &[u8],
        crc_extra: u8,
        out: &mut [u8],
    ) -> Result<usize, CodecError> {
        let header = FrameHeader {
            magic: MAGIC_V2,
            payload_len: payload.len() as u8,
            incompat_flags: 0,
            compat_flags: 0,
            sequence: self.seq,
            system_id: self.system_id,
            component_id: self.component_id,
            message_id: msg_id,
        };
        let n = encode_frame(&header, payload, crc_extra, out)?;
        self.seq = self.seq.wrapping_add(1);
        Ok(n)
    }
}

#[cfg(test)]
mod tests;
