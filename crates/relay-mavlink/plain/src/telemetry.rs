//! MAVLink TELEMETRY encoders (MAVLINK-P06, v1.119) — the operator stream.
//!
//! Six encode-only messages the falcon streams to the GCS over the SiK
//! radio: ATTITUDE, SYS_STATUS, GPS_RAW_INT, SERVO_OUTPUT_RAW, VFR_HUD and
//! STATUSTEXT. Every payload layout and CRC_EXTRA below is pinned against
//! **pymavlink reference vectors** (scripts/gen-mavlink-telemetry-vectors.py)
//! in the conformance tests — the external-oracle discipline the DroneCAN
//! bit-order bug taught: self-round-trips prove robustness, only the
//! reference implementation proves CONFORMANCE. MAVLink2 truncates trailing
//! zero payload bytes ON THE WIRE; these encoders emit the full declared
//! payload and the frame layer owns truncation, so the vectors here are the
//! canonical fixed-length payloads (pymavlink's `unpacker.size`).

// ── ATTITUDE (30) ────────────────────────────────────────────────────────────

pub const ATTITUDE_MSG_ID: u32 = 30;
/// ATTITUDE CRC_EXTRA (pymavlink reference).
pub const ATTITUDE_CRC_EXTRA: u8 = 39;
pub const ATTITUDE_PAYLOAD_LEN: usize = 28;

/// Attitude + body rates (rad, rad/s), the 10 Hz operator-attitude stream.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Attitude {
    pub time_boot_ms: u32,
    pub roll: f32,
    pub pitch: f32,
    pub yaw: f32,
    pub rollspeed: f32,
    pub pitchspeed: f32,
    pub yawspeed: f32,
}

impl Attitude {
    pub fn encode_payload(&self) -> [u8; ATTITUDE_PAYLOAD_LEN] {
        let mut out = [0u8; ATTITUDE_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.time_boot_ms.to_le_bytes());
        out[4..8].copy_from_slice(&self.roll.to_le_bytes());
        out[8..12].copy_from_slice(&self.pitch.to_le_bytes());
        out[12..16].copy_from_slice(&self.yaw.to_le_bytes());
        out[16..20].copy_from_slice(&self.rollspeed.to_le_bytes());
        out[20..24].copy_from_slice(&self.pitchspeed.to_le_bytes());
        out[24..28].copy_from_slice(&self.yawspeed.to_le_bytes());
        out
    }
}

// ── SYS_STATUS (1) ───────────────────────────────────────────────────────────

pub const SYS_STATUS_MSG_ID: u32 = 1;
/// SYS_STATUS CRC_EXTRA (pymavlink reference).
pub const SYS_STATUS_CRC_EXTRA: u8 = 124;
pub const SYS_STATUS_PAYLOAD_LEN: usize = 31;

/// System health + battery for the 2 Hz status stream. Wire order is
/// size-descending per MAVLink: the three u32 masks, then the u16 block,
/// then the i8 remaining.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SysStatus {
    pub sensors_present: u32,
    pub sensors_enabled: u32,
    pub sensors_health: u32,
    /// Main-loop load, 0..1000 (‰).
    pub load: u16,
    /// Battery voltage (mV).
    pub voltage_battery_mv: u16,
    /// Battery current (10 mA units); -1 = unknown.
    pub current_battery_10ma: i16,
    pub drop_rate_comm: u16,
    pub errors_comm: u16,
    pub errors_count1: u16,
    pub errors_count2: u16,
    pub errors_count3: u16,
    pub errors_count4: u16,
    /// Remaining battery (%); -1 = unknown.
    pub battery_remaining_pct: i8,
}

impl SysStatus {
    pub fn encode_payload(&self) -> [u8; SYS_STATUS_PAYLOAD_LEN] {
        let mut out = [0u8; SYS_STATUS_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.sensors_present.to_le_bytes());
        out[4..8].copy_from_slice(&self.sensors_enabled.to_le_bytes());
        out[8..12].copy_from_slice(&self.sensors_health.to_le_bytes());
        out[12..14].copy_from_slice(&self.load.to_le_bytes());
        out[14..16].copy_from_slice(&self.voltage_battery_mv.to_le_bytes());
        out[16..18].copy_from_slice(&self.current_battery_10ma.to_le_bytes());
        out[18..20].copy_from_slice(&self.drop_rate_comm.to_le_bytes());
        out[20..22].copy_from_slice(&self.errors_comm.to_le_bytes());
        out[22..24].copy_from_slice(&self.errors_count1.to_le_bytes());
        out[24..26].copy_from_slice(&self.errors_count2.to_le_bytes());
        out[26..28].copy_from_slice(&self.errors_count3.to_le_bytes());
        out[28..30].copy_from_slice(&self.errors_count4.to_le_bytes());
        out[30] = self.battery_remaining_pct as u8;
        out
    }
}

// ── GPS_RAW_INT (24) ─────────────────────────────────────────────────────────

pub const GPS_RAW_INT_MSG_ID: u32 = 24;
/// GPS_RAW_INT CRC_EXTRA (pymavlink reference).
pub const GPS_RAW_INT_CRC_EXTRA: u8 = 24;
/// Full MAVLink2 payload incl. extension fields (alt_ellipsoid…yaw).
pub const GPS_RAW_INT_PAYLOAD_LEN: usize = 52;

/// Raw GNSS for the 2 Hz stream. Extension fields (h/v/vel/hdg accuracy,
/// ellipsoid alt, GPS yaw) are emitted as 0 = unknown.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GpsRawInt {
    pub time_usec: u64,
    pub lat_e7: i32,
    pub lon_e7: i32,
    /// MSL altitude (mm).
    pub alt_mm: i32,
    pub eph_cm: u16,
    pub epv_cm: u16,
    /// Ground speed (cm/s); u16::MAX = unknown.
    pub vel_cms: u16,
    /// Course over ground (cdeg); u16::MAX = unknown.
    pub cog_cdeg: u16,
    /// 0-1 = no fix, 2 = 2D, 3 = 3D, 4 = DGPS, 5/6 = RTK.
    pub fix_type: u8,
    pub satellites_visible: u8,
}

impl GpsRawInt {
    pub fn encode_payload(&self) -> [u8; GPS_RAW_INT_PAYLOAD_LEN] {
        let mut out = [0u8; GPS_RAW_INT_PAYLOAD_LEN];
        out[0..8].copy_from_slice(&self.time_usec.to_le_bytes());
        out[8..12].copy_from_slice(&self.lat_e7.to_le_bytes());
        out[12..16].copy_from_slice(&self.lon_e7.to_le_bytes());
        out[16..20].copy_from_slice(&self.alt_mm.to_le_bytes());
        out[20..22].copy_from_slice(&self.eph_cm.to_le_bytes());
        out[22..24].copy_from_slice(&self.epv_cm.to_le_bytes());
        out[24..26].copy_from_slice(&self.vel_cms.to_le_bytes());
        out[26..28].copy_from_slice(&self.cog_cdeg.to_le_bytes());
        out[28] = self.fix_type;
        out[29] = self.satellites_visible;
        // bytes 30..52: MAVLink2 extension fields, all 0 = unknown.
        out
    }
}

// ── SERVO_OUTPUT_RAW (36) ────────────────────────────────────────────────────

pub const SERVO_OUTPUT_RAW_MSG_ID: u32 = 36;
/// SERVO_OUTPUT_RAW CRC_EXTRA (pymavlink reference).
pub const SERVO_OUTPUT_RAW_CRC_EXTRA: u8 = 222;
/// Full MAVLink2 payload incl. servo9..16 extension block.
pub const SERVO_OUTPUT_RAW_PAYLOAD_LEN: usize = 37;

/// Actuator outputs for the 2 Hz stream: the quad's four motors as
/// servo1..4 (µs), servo5..16 zero.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoOutputRaw {
    /// Timestamp (µs); the wire field is u32 — truncated by the encoder.
    pub time_usec: u64,
    pub port: u8,
    pub servo_us: [u16; 4],
}

impl ServoOutputRaw {
    pub fn encode_payload(&self) -> [u8; SERVO_OUTPUT_RAW_PAYLOAD_LEN] {
        let mut out = [0u8; SERVO_OUTPUT_RAW_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&(self.time_usec as u32).to_le_bytes());
        for (i, s) in self.servo_us.iter().enumerate() {
            out[4 + i * 2..6 + i * 2].copy_from_slice(&s.to_le_bytes());
        }
        // servo5..8 (bytes 12..20) zero.
        out[20] = self.port;
        // servo9..16 extension block (bytes 21..37) zero.
        out
    }
}

// ── VFR_HUD (74) ─────────────────────────────────────────────────────────────

pub const VFR_HUD_MSG_ID: u32 = 74;
/// VFR_HUD CRC_EXTRA (pymavlink reference).
pub const VFR_HUD_CRC_EXTRA: u8 = 20;
pub const VFR_HUD_PAYLOAD_LEN: usize = 20;

/// The HUD strip: speeds (m/s), MSL altitude (m), climb (m/s), heading
/// (deg 0..360), throttle (%).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VfrHud {
    pub airspeed: f32,
    pub groundspeed: f32,
    pub alt_m: f32,
    pub climb: f32,
    pub heading_deg: i16,
    pub throttle_pct: u16,
}

impl VfrHud {
    pub fn encode_payload(&self) -> [u8; VFR_HUD_PAYLOAD_LEN] {
        let mut out = [0u8; VFR_HUD_PAYLOAD_LEN];
        out[0..4].copy_from_slice(&self.airspeed.to_le_bytes());
        out[4..8].copy_from_slice(&self.groundspeed.to_le_bytes());
        out[8..12].copy_from_slice(&self.alt_m.to_le_bytes());
        out[12..16].copy_from_slice(&self.climb.to_le_bytes());
        out[16..18].copy_from_slice(&self.heading_deg.to_le_bytes());
        out[18..20].copy_from_slice(&self.throttle_pct.to_le_bytes());
        out
    }
}

// ── STATUSTEXT (253) ─────────────────────────────────────────────────────────

pub const STATUSTEXT_MSG_ID: u32 = 253;
/// STATUSTEXT CRC_EXTRA (pymavlink reference).
pub const STATUSTEXT_CRC_EXTRA: u8 = 83;
/// Full MAVLink2 payload incl. id/chunk_seq extension.
pub const STATUSTEXT_PAYLOAD_LEN: usize = 54;

/// MAV_SEVERITY values (subset falcon emits).
pub const SEVERITY_CRITICAL: u8 = 2;
pub const SEVERITY_WARNING: u8 = 4;
pub const SEVERITY_INFO: u8 = 6;

/// Operator event text (failsafes, mode changes, pre-arm reasons). Text is
/// NUL-padded/truncated to 50 bytes; multi-chunk texts are out of scope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StatusText {
    pub severity: u8,
    pub text: [u8; 50],
}

impl StatusText {
    /// Build from a str (truncated to 50 bytes on a char boundary-agnostic
    /// byte cut — operator strings here are ASCII).
    pub fn new(severity: u8, msg: &str) -> StatusText {
        let mut text = [0u8; 50];
        let b = msg.as_bytes();
        let n = if b.len() < 50 { b.len() } else { 50 };
        text[..n].copy_from_slice(&b[..n]);
        StatusText { severity, text }
    }

    pub fn encode_payload(&self) -> [u8; STATUSTEXT_PAYLOAD_LEN] {
        let mut out = [0u8; STATUSTEXT_PAYLOAD_LEN];
        out[0] = self.severity;
        out[1..51].copy_from_slice(&self.text);
        // id (u16) + chunk_seq (u8) extension = 0 (single-chunk).
        out
    }
}

#[cfg(test)]
mod conformance {
    //! pymavlink reference vectors (scripts/gen-mavlink-telemetry-vectors.py,
    //! pymavlink 2.4.49). The encoder must reproduce the hex EXACTLY.
    use super::*;

    fn hex(payload: &[u8]) -> impl core::fmt::Display + '_ {
        struct H<'a>(&'a [u8]);
        impl core::fmt::Display for H<'_> {
            fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
                for b in self.0 {
                    write!(f, "{b:02x}")?;
                }
                Ok(())
            }
        }
        H(payload)
    }

    fn assert_hex(actual: &[u8], expected: &str) {
        extern crate std;
        assert_eq!(std::format!("{}", hex(actual)), expected);
    }

    #[test]
    fn attitude_matches_pymavlink() {
        let m = Attitude {
            time_boot_ms: 123456,
            roll: 0.1,
            pitch: -0.05,
            #[allow(clippy::approx_constant)] // pymavlink vector pinned at 1.5708 exactly
            yaw: 1.5708,
            rollspeed: 0.01,
            pitchspeed: -0.02,
            yawspeed: 0.5,
        };
        assert_hex(
            &m.encode_payload(),
            "40e20100cdcccc3dcdcc4cbdf90fc93f0ad7233c0ad7a3bc0000003f",
        );
    }

    #[test]
    fn sys_status_matches_pymavlink() {
        let m = SysStatus {
            sensors_present: 0x3F,
            sensors_enabled: 0x3F,
            sensors_health: 0x3F,
            load: 250,
            voltage_battery_mv: 15400,
            current_battery_10ma: 1250,
            drop_rate_comm: 0,
            errors_comm: 0,
            errors_count1: 0,
            errors_count2: 0,
            errors_count3: 0,
            errors_count4: 0,
            battery_remaining_pct: 87,
        };
        assert_hex(
            &m.encode_payload(),
            "3f0000003f0000003f000000fa00283ce20400000000000000000000000057",
        );
    }

    #[test]
    fn gps_raw_int_matches_pymavlink() {
        let m = GpsRawInt {
            time_usec: 1234567890,
            lat_e7: 473977000,
            lon_e7: 85456000,
            alt_mm: 488000,
            eph_cm: 120,
            epv_cm: 180,
            vel_cms: 250,
            cog_cdeg: 9000,
            fix_type: 3,
            satellites_visible: 14,
        };
        assert_hex(
            &m.encode_payload(),
            "d202964900000000a850401c80f41705407207007800b400fa002823030e00000000000000000000000000000000000000000000",
        );
    }

    #[test]
    fn servo_output_raw_matches_pymavlink() {
        let m = ServoOutputRaw {
            time_usec: 1234567890,
            port: 0,
            servo_us: [1500, 1520, 1480, 1510],
        };
        assert_hex(
            &m.encode_payload(),
            "d2029649dc05f005c805e60500000000000000000000000000000000000000000000000000",
        );
    }

    #[test]
    fn vfr_hud_matches_pymavlink() {
        let m = VfrHud {
            airspeed: 0.0,
            groundspeed: 2.5,
            alt_m: 2.0,
            climb: -0.5,
            heading_deg: 90,
            throttle_pct: 58,
        };
        assert_hex(&m.encode_payload(), "000000000000204000000040000000bf5a003a00");
    }

    #[test]
    fn statustext_matches_pymavlink() {
        let m = StatusText::new(SEVERITY_CRITICAL, "ROTOR 0 OUT: LANDING");
        assert_hex(
            &m.encode_payload(),
            "02524f544f522030204f55543a204c414e44494e47000000000000000000000000000000000000000000000000000000000000000000",
        );
    }
}
