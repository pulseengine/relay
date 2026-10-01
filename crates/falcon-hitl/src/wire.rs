//! The `pulseengine:falcon-cascade` transport binding (TRANSPORT-P01).
//!
//! WHY THIS EXISTS. We ship a verified component and, until now, no way to
//! talk to it. An integrator drove the published component over UDP against
//! Gazebo and had to invent the wire themselves. This module is the published
//! binding so the next one does not have to — and `falcon-hitl` is where it
//! lives because the crate's genuinely reusable half is exactly this: a
//! fixed-layout, little-endian, `no_std`/no-alloc codec that runs unchanged on
//! the MCU.
//!
//! ## The header is 8 bytes, and the requirement says 6
//!
//! SWREQ-FALCON-TRANSPORT-P01 describes the captured wire as "a 6-byte header
//! — magic `1c fa`, version, message type, little-endian `u32` sequence". That
//! field list sums to **8**: magic 2 + version 1 + type 1 + sequence 4. The
//! label is wrong, not the fields, and the capture's own frame sizes settle it
//! exactly:
//!
//! ```text
//!   captured motors frame   24 B
//!   seam's `motor-pwm`      16 B  (m1..m4: f32)
//!   difference               8 B  <- the header
//! ```
//!
//! So 8 it is, derived rather than chosen. Recorded here because the next
//! person to read the requirement will do the same arithmetic and should find
//! the answer instead of the discrepancy.
//!
//! The sensor side does NOT reconcile as cleanly and is not claimed to: the
//! captured sensor frame is 83 B, leaving a 75 B payload, while a flat
//! encoding of the seam's `sensor-frame` is **76 B** (imu 24 + dt 4 + three
//! `option<vec3>`/`option<f32>` at 13/13/5 + `option<rotor-rpm>` 17). The most
//! likely explanation is that the integrator sent `heading-rad` as a bare
//! `f32` because their rig always has one, dropping its presence byte. We do
//! not know, and we do not need to: this binding is NORMATIVE and the capture
//! is prior art that a wire of this shape works. The one-byte difference is
//! called out in the binding document so an existing host knows precisely what
//! to change.
//!
//! ## What is specified here vs. what the capture specified
//!
//! The header shape comes from the capture, so an existing host needs minimal
//! change. Everything below it is OURS, because the payloads are the seam's own
//! records and the sequence semantics are a contract the capture never had to
//! state.

use core::fmt;

/// Frame magic. Two bytes, so a host that points at the wrong port sees a
/// rejection rather than decoding noise as flight data.
pub const MAGIC: [u8; 2] = [0x1c, 0xfa];

/// Binding version. Bumped when any payload layout changes — NOT when the WIT
/// package version changes, because a host cares about bytes, not names.
pub const VERSION: u8 = 1;

/// magic(2) + version(1) + msg type(1) + sequence(4). See the module note on
/// why this is 8 and not the 6 the requirement says.
pub const HEADER_LEN: usize = 8;

/// `vehicle-config`: nine `f32`s, in WIT declaration order.
pub const CONFIG_PAYLOAD_LEN: usize = 36;
/// An ack carries the sequence it acknowledges, and nothing else.
pub const ACK_PAYLOAD_LEN: usize = 4;
/// A flat `sensor-frame`. See the module note: this is 76, and the capture
/// implies 75.
pub const SENSOR_PAYLOAD_LEN: usize = 76;
/// `motor-pwm`: four `f32`s. Matches the capture exactly (24 - 8).
pub const MOTORS_PAYLOAD_LEN: usize = 16;

/// Message type. Values are wire-visible and must not be renumbered; add new
/// ones and bump [`VERSION`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MsgType {
    /// Host -> component: install `vehicle-config`. See the handshake note.
    Config = 0,
    /// Component -> host: the config was applied, echoing its sequence.
    Ack = 1,
    /// Host -> component: one `sensor-frame` for this tick.
    Sensor = 2,
    /// Component -> host: the resulting `motor-pwm`.
    Motors = 3,
}

impl MsgType {
    /// Exact payload length for this type. A length that disagrees is a
    /// rejection, not a truncation — see [`WireError::BadLength`].
    pub const fn payload_len(self) -> usize {
        match self {
            MsgType::Config => CONFIG_PAYLOAD_LEN,
            MsgType::Ack => ACK_PAYLOAD_LEN,
            MsgType::Sensor => SENSOR_PAYLOAD_LEN,
            MsgType::Motors => MOTORS_PAYLOAD_LEN,
        }
    }

    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(MsgType::Config),
            1 => Some(MsgType::Ack),
            2 => Some(MsgType::Sensor),
            3 => Some(MsgType::Motors),
            _ => None,
        }
    }
}

/// A decoded frame header.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Header {
    pub version: u8,
    pub msg: MsgType,
    pub seq: u32,
}

/// Why a frame was rejected.
///
/// Every variant is a REJECTION. Nothing here is recoverable by guessing: a
/// decoder that salvages a frame it does not understand is how a host ends up
/// flying on misread bytes.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum WireError {
    /// Fewer than [`HEADER_LEN`] bytes.
    TooShort { got: usize },
    /// First two bytes are not [`MAGIC`] — wrong port, or a desynchronised
    /// stream.
    BadMagic { got: [u8; 2] },
    /// A version this build does not implement. The host is told both numbers
    /// so it can report a mismatch instead of a parse failure.
    BadVersion { got: u8, want: u8 },
    /// A message type this build does not know.
    UnknownType { got: u8 },
    /// The payload length disagrees with the type's fixed length.
    BadLength {
        msg: MsgType,
        got: usize,
        want: usize,
    },
}

impl fmt::Display for WireError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            WireError::TooShort { got } => {
                write!(
                    f,
                    "frame shorter than an {HEADER_LEN}-byte header ({got} B)"
                )
            }
            WireError::BadMagic { got } => {
                write!(f, "bad magic {:02x}{:02x} (want 1cfa)", got[0], got[1])
            }
            WireError::BadVersion { got, want } => {
                write!(f, "binding version {got}, this build speaks {want}")
            }
            WireError::UnknownType { got } => write!(f, "unknown message type {got}"),
            WireError::BadLength { msg, got, want } => {
                write!(f, "{msg:?} payload {got} B, want {want} B")
            }
        }
    }
}

/// Write a header into the first [`HEADER_LEN`] bytes of `out`.
pub fn encode_header(out: &mut [u8], msg: MsgType, seq: u32) -> Result<(), WireError> {
    if out.len() < HEADER_LEN {
        return Err(WireError::TooShort { got: out.len() });
    }
    out[0] = MAGIC[0];
    out[1] = MAGIC[1];
    out[2] = VERSION;
    out[3] = msg as u8;
    out[4..8].copy_from_slice(&seq.to_le_bytes());
    Ok(())
}

/// Decode a header and validate the payload length that follows it.
///
/// Returns the header and the payload slice, so a caller cannot accidentally
/// read past the frame.
pub fn decode_frame(buf: &[u8]) -> Result<(Header, &[u8]), WireError> {
    if buf.len() < HEADER_LEN {
        return Err(WireError::TooShort { got: buf.len() });
    }
    if buf[0] != MAGIC[0] || buf[1] != MAGIC[1] {
        return Err(WireError::BadMagic {
            got: [buf[0], buf[1]],
        });
    }
    if buf[2] != VERSION {
        return Err(WireError::BadVersion {
            got: buf[2],
            want: VERSION,
        });
    }
    let msg = MsgType::from_u8(buf[3]).ok_or(WireError::UnknownType { got: buf[3] })?;
    let mut s = [0u8; 4];
    s.copy_from_slice(&buf[4..8]);
    let seq = u32::from_le_bytes(s);
    let payload = &buf[HEADER_LEN..];
    let want = msg.payload_len();
    if payload.len() != want {
        return Err(WireError::BadLength {
            msg,
            got: payload.len(),
            want,
        });
    }
    Ok((
        Header {
            version: buf[2],
            msg,
            seq,
        },
        payload,
    ))
}

// ── the tick contract ────────────────────────────────────────────────────

/// What the sequence number says happened to this tick.
///
/// TRANSPORT-P01's falsification condition is explicit that this distinction
/// must exist: the requirement is "wrong if a host that satisfies the binding
/// cannot tell a dropped tick from a late one". So these are separate variants
/// carrying separate counts, not one `OutOfOrder`.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum TickVerdict {
    /// The first frame seen. Nothing to compare against.
    First,
    /// Exactly the successor of the last sequence.
    InOrder,
    /// The same sequence again — a retransmit, or a host that failed to
    /// increment. The caller must NOT step the controller twice on one tick.
    Duplicate,
    /// A forward jump: `lost` frames never arrived.
    Dropped { lost: u32 },
    /// A sequence OLDER than one already seen — it arrived after its
    /// successor, so it is late rather than lost. Reordering is real on UDP.
    Late { by: u32 },
}

/// Classifies each frame's sequence against the last one accepted.
///
/// WRAPPING IS HANDLED, not assumed away. A `u32` sequence at 250 Hz wraps
/// after ~198 days of continuous flight, which is longer than any mission this
/// airframe will fly and still not a reason to decode it wrongly. The
/// comparison uses the half-space convention — a wrapped difference below
/// 2^31 is "forward", at or above is "backward" — so `seq` 0 arriving after
/// `u32::MAX` reads as `InOrder`, not as a 4-billion-frame drop.
#[derive(Clone, Copy, Debug, Default)]
pub struct SeqTracker {
    last: Option<u32>,
}

impl SeqTracker {
    pub const fn new() -> Self {
        Self { last: None }
    }

    /// The last sequence this tracker accepted as current.
    pub const fn last(&self) -> Option<u32> {
        self.last
    }

    /// Classify `seq`, and advance if it moves the stream forward.
    ///
    /// A `Late` or `Duplicate` frame does NOT advance the tracker — otherwise a
    /// single reordered datagram would make every subsequent in-order frame
    /// look like a drop.
    pub fn observe(&mut self, seq: u32) -> TickVerdict {
        let Some(last) = self.last else {
            self.last = Some(seq);
            return TickVerdict::First;
        };
        let fwd = seq.wrapping_sub(last);
        match fwd {
            0 => TickVerdict::Duplicate,
            1 => {
                self.last = Some(seq);
                TickVerdict::InOrder
            }
            // Forward half-space: a real gap.
            d if d < 1 << 31 => {
                self.last = Some(seq);
                TickVerdict::Dropped { lost: d - 1 }
            }
            // Backward half-space: older than what we have.
            _ => TickVerdict::Late {
                by: last.wrapping_sub(seq),
            },
        }
    }
}

// ── payloads: the seam's own records, flat and little-endian ─────────────
//
// FIXED SIZE EVEN WHEN FIELDS ARE ABSENT. Each optional field is a presence
// byte followed by its value bytes, and the value bytes are present (zeroed)
// whether or not the field is. A compact encoding would be smaller; a fixed
// one lets the MCU use a static buffer, makes every length disagreement a hard
// rejection rather than a short read, and keeps field offsets constant so a
// capture is readable by eye. The seam learned at v0.8 and v0.10 that the cost
// of a frame that cannot express a field (`falcon-hitl`'s own 54-byte frame
// carries no heading, no rotor RPM, and no battery that says ABSENT) is paid in
// flight, not in bytes.

fn put_f32(buf: &mut [u8], at: usize, v: f32) {
    buf[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
fn get_f32(buf: &[u8], at: usize) -> f32 {
    let mut b = [0u8; 4];
    b.copy_from_slice(&buf[at..at + 4]);
    f32::from_le_bytes(b)
}
fn put_i32(buf: &mut [u8], at: usize, v: i32) {
    buf[at..at + 4].copy_from_slice(&v.to_le_bytes());
}
fn get_i32(buf: &[u8], at: usize) -> i32 {
    let mut b = [0u8; 4];
    b.copy_from_slice(&buf[at..at + 4]);
    i32::from_le_bytes(b)
}

/// `sensor-frame` as the binding carries it. Mirrors the WIT record, including
/// which fields are optional — a host that cannot supply one says so rather
/// than sending a zero that the estimator would fuse as a measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WireSensorFrame {
    /// ax ay az gx gy gz, body frame.
    pub imu: [f32; 6],
    /// The host's ACTUAL period for this tick, not its nominal one. v0.8 of the
    /// seam exists because stages disagreed by up to 20x and the hold error went
    /// from 0.00 m to 27.17 m with no error signal anywhere.
    pub dt_s: f32,
    pub position_ned: Option<[f32; 3]>,
    pub mag_body: Option<[f32; 3]>,
    pub heading_rad: Option<f32>,
    pub motor_rpm: Option<[i32; 4]>,
}

/// `vehicle-config`, in WIT declaration order.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WireVehicleConfig {
    pub hover_thrust: f32,
    pub loop_rate_hz: f32,
    pub pos_var: f32,
    pub process_floor_vel: f32,
    pub process_floor_pos: f32,
    pub altitude_kp: f32,
    pub altitude_kd: f32,
    pub altitude_ki: f32,
    pub position_ki: f32,
}

impl WireVehicleConfig {
    pub fn to_array(self) -> [f32; 9] {
        [
            self.hover_thrust,
            self.loop_rate_hz,
            self.pos_var,
            self.process_floor_vel,
            self.process_floor_pos,
            self.altitude_kp,
            self.altitude_kd,
            self.altitude_ki,
            self.position_ki,
        ]
    }
    pub fn from_array(a: [f32; 9]) -> Self {
        Self {
            hover_thrust: a[0],
            loop_rate_hz: a[1],
            pos_var: a[2],
            process_floor_vel: a[3],
            process_floor_pos: a[4],
            altitude_kp: a[5],
            altitude_kd: a[6],
            altitude_ki: a[7],
            position_ki: a[8],
        }
    }
}

/// Encode a whole `Motors` frame (header + payload) into `out`.
pub fn encode_motors(out: &mut [u8; HEADER_LEN + MOTORS_PAYLOAD_LEN], seq: u32, m: [f32; 4]) {
    encode_header(out, MsgType::Motors, seq).expect("buffer is exactly one frame");
    for (i, v) in m.iter().enumerate() {
        put_f32(out, HEADER_LEN + i * 4, *v);
    }
}

/// Decode a `Motors` payload (the slice [`decode_frame`] handed back).
pub fn decode_motors(payload: &[u8]) -> Result<[f32; 4], WireError> {
    if payload.len() != MOTORS_PAYLOAD_LEN {
        return Err(WireError::BadLength {
            msg: MsgType::Motors,
            got: payload.len(),
            want: MOTORS_PAYLOAD_LEN,
        });
    }
    let mut m = [0.0f32; 4];
    for (i, mi) in m.iter_mut().enumerate() {
        *mi = get_f32(payload, i * 4);
    }
    Ok(m)
}

/// Encode a whole `Config` frame.
pub fn encode_config(
    out: &mut [u8; HEADER_LEN + CONFIG_PAYLOAD_LEN],
    seq: u32,
    cfg: &WireVehicleConfig,
) {
    encode_header(out, MsgType::Config, seq).expect("buffer is exactly one frame");
    for (i, v) in cfg.to_array().iter().enumerate() {
        put_f32(out, HEADER_LEN + i * 4, *v);
    }
}

/// Decode a `Config` payload.
pub fn decode_config(payload: &[u8]) -> Result<WireVehicleConfig, WireError> {
    if payload.len() != CONFIG_PAYLOAD_LEN {
        return Err(WireError::BadLength {
            msg: MsgType::Config,
            got: payload.len(),
            want: CONFIG_PAYLOAD_LEN,
        });
    }
    let mut a = [0.0f32; 9];
    for (i, ai) in a.iter_mut().enumerate() {
        *ai = get_f32(payload, i * 4);
    }
    Ok(WireVehicleConfig::from_array(a))
}

/// Encode a whole `Ack` frame. `acked` is the sequence being acknowledged,
/// which is NOT necessarily this frame's own `seq`.
pub fn encode_ack(out: &mut [u8; HEADER_LEN + ACK_PAYLOAD_LEN], seq: u32, acked: u32) {
    encode_header(out, MsgType::Ack, seq).expect("buffer is exactly one frame");
    out[HEADER_LEN..HEADER_LEN + 4].copy_from_slice(&acked.to_le_bytes());
}

/// Decode an `Ack` payload into the sequence it acknowledges.
pub fn decode_ack(payload: &[u8]) -> Result<u32, WireError> {
    if payload.len() != ACK_PAYLOAD_LEN {
        return Err(WireError::BadLength {
            msg: MsgType::Ack,
            got: payload.len(),
            want: ACK_PAYLOAD_LEN,
        });
    }
    let mut b = [0u8; 4];
    b.copy_from_slice(&payload[..4]);
    Ok(u32::from_le_bytes(b))
}

/// Encode a whole `Sensor` frame. Offsets are fixed; see the payload note.
pub fn encode_sensor_frame(
    out: &mut [u8; HEADER_LEN + SENSOR_PAYLOAD_LEN],
    seq: u32,
    f: &WireSensorFrame,
) {
    encode_header(out, MsgType::Sensor, seq).expect("buffer is exactly one frame");
    let b = HEADER_LEN;
    for (i, v) in f.imu.iter().enumerate() {
        put_f32(out, b + i * 4, *v);
    }
    put_f32(out, b + 24, f.dt_s);
    // position-ned
    out[b + 28] = f.position_ned.is_some() as u8;
    for (i, v) in f.position_ned.unwrap_or([0.0; 3]).iter().enumerate() {
        put_f32(out, b + 29 + i * 4, *v);
    }
    // mag-body
    out[b + 41] = f.mag_body.is_some() as u8;
    for (i, v) in f.mag_body.unwrap_or([0.0; 3]).iter().enumerate() {
        put_f32(out, b + 42 + i * 4, *v);
    }
    // heading-rad
    out[b + 54] = f.heading_rad.is_some() as u8;
    put_f32(out, b + 55, f.heading_rad.unwrap_or(0.0));
    // motor-rpm
    out[b + 59] = f.motor_rpm.is_some() as u8;
    for (i, v) in f.motor_rpm.unwrap_or([0; 4]).iter().enumerate() {
        put_i32(out, b + 60 + i * 4, *v);
    }
}

/// Decode a `Sensor` payload.
pub fn decode_sensor_frame(payload: &[u8]) -> Result<WireSensorFrame, WireError> {
    if payload.len() != SENSOR_PAYLOAD_LEN {
        return Err(WireError::BadLength {
            msg: MsgType::Sensor,
            got: payload.len(),
            want: SENSOR_PAYLOAD_LEN,
        });
    }
    let mut f = WireSensorFrame::default();
    for (i, v) in f.imu.iter_mut().enumerate() {
        *v = get_f32(payload, i * 4);
    }
    f.dt_s = get_f32(payload, 24);
    if payload[28] != 0 {
        f.position_ned = Some([
            get_f32(payload, 29),
            get_f32(payload, 33),
            get_f32(payload, 37),
        ]);
    }
    if payload[41] != 0 {
        f.mag_body = Some([
            get_f32(payload, 42),
            get_f32(payload, 46),
            get_f32(payload, 50),
        ]);
    }
    if payload[54] != 0 {
        f.heading_rad = Some(get_f32(payload, 55));
    }
    if payload[59] != 0 {
        f.motor_rpm = Some([
            get_i32(payload, 60),
            get_i32(payload, 64),
            get_i32(payload, 68),
            get_i32(payload, 72),
        ]);
    }
    Ok(f)
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── FIXED BYTE VECTORS, hand-computed ───────────────────────────────
    //
    // Round-trips prove ROBUSTNESS, not CONFORMANCE: an encoder and decoder
    // that share a wrong offset or a swapped endianness agree perfectly with
    // each other. That exact blind spot shipped garbage ESC throttles once
    // (40 tests + 9 Kani harnesses + a clean-room review all missed it), and
    // the rule learned from it is that a wire codec is validated against
    // vectors it did not produce.
    //
    // The TRUE external reference here is the integrator's capture, and it is
    // NOT in this repo (see the module note). So these vectors are hand-computed
    // from the IEEE-754 bit patterns instead — weaker than a capture, strictly
    // stronger than a round-trip, and they do catch the offset and endianness
    // classes. Every field here is byte-aligned, so the bit-misalignment class
    // that bit the DroneCAN decoder cannot arise.

    /// `1.0f32` is `0x3F800000`; little-endian that is `00 00 80 3f`.
    const F1: [u8; 4] = [0x00, 0x00, 0x80, 0x3f];
    /// `0.5f32` is `0x3F000000`.
    const F05: [u8; 4] = [0x00, 0x00, 0x00, 0x3f];
    /// `-1.0f32` is `0xBF800000`.
    const FM1: [u8; 4] = [0x00, 0x00, 0x80, 0xbf];

    #[test]
    fn a_motors_frame_matches_its_hand_computed_bytes() {
        let mut out = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut out, 7, [1.0, 0.5, 0.0, -1.0]);
        let mut want = [0u8; 24];
        want[..4].copy_from_slice(&[0x1c, 0xfa, 0x01, 0x03]); // magic, v1, Motors
        want[4..8].copy_from_slice(&[0x07, 0x00, 0x00, 0x00]); // seq 7, LE
        want[8..12].copy_from_slice(&F1);
        want[12..16].copy_from_slice(&F05);
        // m3 = 0.0 -> already zero
        want[20..24].copy_from_slice(&FM1);
        assert_eq!(out, want, "motors frame bytes");
    }

    #[test]
    fn an_ack_frame_matches_its_hand_computed_bytes() {
        let mut out = [0u8; HEADER_LEN + ACK_PAYLOAD_LEN];
        encode_ack(&mut out, 9, 5);
        assert_eq!(
            out,
            [0x1c, 0xfa, 0x01, 0x01, 0x09, 0, 0, 0, 0x05, 0, 0, 0],
            "ack frame bytes"
        );
    }

    /// The capture's motors frame was 24 B and the seam's `motor-pwm` is 16 B.
    /// That difference is the whole reason this binding's header is 8 and not
    /// the 6 the requirement states, so it is asserted rather than remembered.
    #[test]
    fn the_header_length_is_what_the_captures_frame_sizes_imply() {
        assert_eq!(MOTORS_PAYLOAD_LEN, 16, "motor-pwm is four f32");
        assert_eq!(
            HEADER_LEN + MOTORS_PAYLOAD_LEN,
            24,
            "the captured motors frame"
        );
        assert_eq!(HEADER_LEN, 8, "magic 2 + version 1 + type 1 + seq 4");
    }

    // ── rejection, not salvage ──────────────────────────────────────────

    #[test]
    fn a_frame_from_the_wrong_port_is_rejected_not_decoded() {
        let mut f = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut f, 1, [0.1, 0.2, 0.3, 0.4]);
        f[0] = 0xff;
        assert_eq!(
            decode_frame(&f),
            Err(WireError::BadMagic { got: [0xff, 0xfa] }),
            "bad magic must not decode as flight data"
        );
    }

    #[test]
    fn a_version_mismatch_names_both_versions() {
        let mut f = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut f, 1, [0.0; 4]);
        f[2] = 99;
        assert_eq!(
            decode_frame(&f),
            Err(WireError::BadVersion {
                got: 99,
                want: VERSION
            })
        );
    }

    #[test]
    fn an_unknown_message_type_is_rejected() {
        let mut f = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut f, 1, [0.0; 4]);
        f[3] = 7;
        assert_eq!(decode_frame(&f), Err(WireError::UnknownType { got: 7 }));
    }

    #[test]
    fn a_truncated_payload_is_rejected_rather_than_short_read() {
        let mut f = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut f, 1, [0.0; 4]);
        assert_eq!(
            decode_frame(&f[..HEADER_LEN + 8]),
            Err(WireError::BadLength {
                msg: MsgType::Motors,
                got: 8,
                want: 16
            })
        );
        assert!(matches!(
            decode_frame(&f[..3]),
            Err(WireError::TooShort { got: 3 })
        ));
    }

    // ── round-trips (necessary, not sufficient) ─────────────────────────

    #[test]
    fn every_message_type_round_trips() {
        let mut m = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut m, 11, [0.1, -0.2, 0.3, 0.4]);
        let (h, p) = decode_frame(&m).unwrap();
        assert_eq!((h.msg, h.seq), (MsgType::Motors, 11));
        assert_eq!(decode_motors(p).unwrap(), [0.1, -0.2, 0.3, 0.4]);

        let cfg = WireVehicleConfig {
            hover_thrust: 0.585,
            loop_rate_hz: 250.0,
            pos_var: 0.25,
            process_floor_vel: 0.30,
            process_floor_pos: 0.05,
            altitude_kp: 0.15,
            altitude_kd: 1.0,
            altitude_ki: 0.03,
            position_ki: 0.02,
        };
        let mut c = [0u8; HEADER_LEN + CONFIG_PAYLOAD_LEN];
        encode_config(&mut c, 1, &cfg);
        let (h, p) = decode_frame(&c).unwrap();
        assert_eq!(h.msg, MsgType::Config);
        assert_eq!(decode_config(p).unwrap(), cfg);

        let mut a = [0u8; HEADER_LEN + ACK_PAYLOAD_LEN];
        encode_ack(&mut a, 2, 1);
        let (_, p) = decode_frame(&a).unwrap();
        assert_eq!(decode_ack(p).unwrap(), 1);
    }

    #[test]
    fn a_sensor_frame_round_trips_with_every_field_present() {
        let f = WireSensorFrame {
            imu: [0.1, 0.2, 9.8, 0.01, -0.02, 0.003],
            dt_s: 0.004,
            position_ned: Some([1.0, 2.0, -3.0]),
            mag_body: Some([0.2, 0.0, 0.4]),
            heading_rad: Some(1.57),
            motor_rpm: Some([7000, 7010, 6990, 7001]),
        };
        let mut buf = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut buf, 3, &f);
        let (h, p) = decode_frame(&buf).unwrap();
        assert_eq!((h.msg, h.seq), (MsgType::Sensor, 3));
        assert_eq!(decode_sensor_frame(p).unwrap(), f);
    }

    /// THE FAIL-SAFE PROPERTY. An absent field must come back `None`, never
    /// `Some(0.0)`. This project has shipped the opposite twice — a battery
    /// whose absence read as a healthy pack (#413) and an accelerometer whose
    /// absence read as gravity (#452) — and both were absence-looks-like-data.
    #[test]
    fn an_absent_field_decodes_as_absent_not_as_zero() {
        let f = WireSensorFrame {
            imu: [1.0; 6],
            dt_s: 0.004,
            position_ned: None,
            mag_body: None,
            heading_rad: None,
            motor_rpm: None,
        };
        let mut buf = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut buf, 1, &f);
        let (_, p) = decode_frame(&buf).unwrap();
        let got = decode_sensor_frame(p).unwrap();
        assert_eq!(
            got.position_ned, None,
            "absent position must not read as 0,0,0"
        );
        assert_eq!(got.mag_body, None);
        assert_eq!(
            got.heading_rad, None,
            "absent heading must not read as 0 rad"
        );
        assert_eq!(
            got.motor_rpm, None,
            "absent RPM must not read as a stopped rotor"
        );
        assert_eq!(got.imu, [1.0; 6], "present fields still decode");
    }

    /// The frame is a fixed size whether or not the optional fields are set —
    /// so a host can use one static buffer and a length disagreement is always
    /// an error rather than a legal compact encoding.
    #[test]
    fn the_sensor_frame_is_the_same_size_empty_or_full() {
        let mut a = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        let mut b = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut a, 1, &WireSensorFrame::default());
        encode_sensor_frame(
            &mut b,
            1,
            &WireSensorFrame {
                position_ned: Some([1.0; 3]),
                mag_body: Some([1.0; 3]),
                heading_rad: Some(1.0),
                motor_rpm: Some([1; 4]),
                ..Default::default()
            },
        );
        assert_eq!(a.len(), b.len());
        assert!(decode_frame(&a).is_ok() && decode_frame(&b).is_ok());
    }

    // ── the tick contract ───────────────────────────────────────────────

    /// TRANSPORT-P01's falsification: "wrong if a host that satisfies the
    /// binding cannot tell a dropped tick from a late one". So the two must be
    /// distinguishable, with counts.
    #[test]
    fn a_dropped_tick_is_distinguishable_from_a_late_one() {
        let mut t = SeqTracker::new();
        assert_eq!(t.observe(10), TickVerdict::First);
        assert_eq!(t.observe(11), TickVerdict::InOrder);
        // forward jump: 12 and 13 never arrived
        assert_eq!(t.observe(14), TickVerdict::Dropped { lost: 2 });
        // an older sequence AFTER a newer one: late, not lost
        assert_eq!(t.observe(12), TickVerdict::Late { by: 2 });
        assert_eq!(t.observe(14), TickVerdict::Duplicate);
    }

    /// A reordered datagram must not poison the stream: if `Late` advanced the
    /// tracker, the next in-order frame would read as a large drop.
    #[test]
    fn a_late_or_duplicate_frame_does_not_advance_the_tracker() {
        let mut t = SeqTracker::new();
        t.observe(100);
        t.observe(101);
        assert_eq!(t.last(), Some(101));
        assert_eq!(t.observe(99), TickVerdict::Late { by: 2 });
        assert_eq!(t.last(), Some(101), "a late frame must not move `last`");
        assert_eq!(t.observe(101), TickVerdict::Duplicate);
        assert_eq!(t.last(), Some(101), "a duplicate must not move `last`");
        assert_eq!(
            t.observe(102),
            TickVerdict::InOrder,
            "stream is not poisoned"
        );
    }

    /// A u32 sequence at 250 Hz wraps after ~198 days. The wrap must read as
    /// one in-order tick, not as a 4-billion-frame drop.
    #[test]
    fn the_sequence_wraps_without_reporting_four_billion_drops() {
        let mut t = SeqTracker::new();
        t.observe(u32::MAX);
        assert_eq!(
            t.observe(0),
            TickVerdict::InOrder,
            "MAX -> 0 is the successor"
        );
        assert_eq!(t.observe(1), TickVerdict::InOrder);
        // and a backward step across the wrap is still Late
        assert_eq!(t.observe(u32::MAX - 1), TickVerdict::Late { by: 3 });
    }
}
