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
//! That is CORROBORATION, NOT PROOF. The 24 comes from the same unsourced
//! sentence as the "6" (#503 — the capture is not in this repository), and the
//! sensor figure from that same sentence does NOT reconcile. Trusting the number
//! that fits while setting aside the one that does not is an inference.
//!
//! So **8 is a normative choice**, not a measurement: it is what the
//! requirement's own field enumeration adds up to — which is self-contained and
//! needs no captured number — and it leaves an existing host's header unchanged.
//! Recorded here because the next person to read the requirement will do the
//! same arithmetic and should find this note instead of the discrepancy.
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
///
/// **v2** (relay#609 finding 9, #572, #527 — one revision, so a host's decoder
/// changes once): the `Sensor` payload gains a `battery-v` element with an
/// explicit presence byte (76 -> 81 B); three host -> component frame types
/// arrive (`SupervisorConfig` 4, `Calibration` 5, `Command` 6) so a host can
/// drive `pilot.configure-supervisor`, `pilot.configure-calibration` and
/// `pilot.command` over the binding; and [`decode_frame`] REJECTS a non-finite
/// `f32` in any float slot with [`WireError::NonFinite`] instead of passing it
/// through.
///
/// **Still v2 after the pre-release amendments** (#614, from the integrator
/// review on jess#167): strict 0/1 presence bytes, the reply echo rule, and
/// `Calibration` 60 -> 72 B with `mag-offdiag` appended. v2 has never been
/// tagged, so these are part of v2, not a v3. From the first release that
/// ships v2, a payload change bumps this.
pub const VERSION: u8 = 2;

/// magic(2) + version(1) + msg type(1) + sequence(4). See the module note on
/// why this is 8 and not the 6 the requirement says.
pub const HEADER_LEN: usize = 8;

/// `vehicle-config`: nine `f32`s, in WIT declaration order.
pub const CONFIG_PAYLOAD_LEN: usize = 36;
/// An ack carries the sequence it acknowledges, and nothing else.
pub const ACK_PAYLOAD_LEN: usize = 4;
/// A flat `sensor-frame`. v1 was 76 (the capture implies 75, see the module
/// note); v2 appends `battery-v` as presence byte + `f32` at 76..81 (#572), so
/// every v1 offset is unchanged.
pub const SENSOR_PAYLOAD_LEN: usize = 81;
/// `motor-pwm`: four `f32`s. Matches the capture exactly (24 - 8).
pub const MOTORS_PAYLOAD_LEN: usize = 16;
/// `pilot.supervisor-config`: `home` (vec3) + `fence-radius` + `cruise-alt` +
/// `low-batt-v` — six `f32`s in WIT declaration order.
pub const SUPERVISOR_CONFIG_PAYLOAD_LEN: usize = 24;
/// `pilot.sensor-calibration` (relay#611): six `vec3`s — eighteen `f32`s in
/// WIT declaration order. 60 -> 72 B within the unreleased binding v2:
/// `mag-offdiag` (symmetric soft-iron off-diagonal terms) is APPENDED at
/// payload 60/64/68, so every earlier offset is unchanged.
pub const CALIBRATION_PAYLOAD_LEN: usize = 72;
/// `pilot.command(cmd, level, throttle-low)`: the `pilot-command` discriminant
/// and the two RC interlocks, one byte each.
pub const COMMAND_PAYLOAD_LEN: usize = 3;

/// Message type. Values are wire-visible and must not be renumbered; add new
/// ones and bump [`VERSION`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MsgType {
    /// Host -> component: install `vehicle-config`. See the handshake note.
    Config = 0,
    /// Component -> host: the control frame was RECEIVED. Its header seq
    /// echoes the acknowledged frame's seq (the echo rule), and the payload
    /// repeats it.
    Ack = 1,
    /// Host -> component: one `sensor-frame` for this tick.
    Sensor = 2,
    /// Component -> host: the resulting `motor-pwm`. Its header seq is the
    /// seq of the `Sensor` frame it answers (the echo rule, jess#167).
    Motors = 3,
    /// Host -> component (v2): `pilot.configure-supervisor`. Acked like
    /// `Config`.
    SupervisorConfig = 4,
    /// Host -> component (v2): `pilot.configure-calibration` (relay#611).
    /// Acked like `Config`.
    Calibration = 5,
    /// Host -> component (v2): one `pilot.command`. Acked like `Config`.
    Command = 6,
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
            MsgType::SupervisorConfig => SUPERVISOR_CONFIG_PAYLOAD_LEN,
            MsgType::Calibration => CALIBRATION_PAYLOAD_LEN,
            MsgType::Command => COMMAND_PAYLOAD_LEN,
        }
    }

    /// Payload offsets of every `f32` SLOT in this type — the bytes the
    /// non-finite rule (#527) applies to.
    ///
    /// SLOTS, NOT FIELDS: an optional field's value bytes are a slot whether or
    /// not its presence byte is set, and they are checked either way (the
    /// encoder writes 0.0 there; a host that parks NaN in an absent slot as a
    /// "no reading" marker is using exactly the sentinel habit the presence
    /// byte replaces). What is deliberately NOT here: presence bytes, and the
    /// four `motor-rpm` `s32`s — an rpm of -8388608 is the bytes `00 00 80 ff`,
    /// which is the -inf pattern, so a "check every 4-byte word" rule would
    /// reject a valid integer.
    pub const fn f32_offsets(self) -> &'static [usize] {
        match self {
            MsgType::Config => &CONFIG_F32,
            MsgType::Ack => &[],
            MsgType::Sensor => &SENSOR_F32,
            MsgType::Motors => &MOTORS_F32,
            MsgType::SupervisorConfig => &SUPERVISOR_CONFIG_F32,
            MsgType::Calibration => &CALIBRATION_F32,
            MsgType::Command => &[],
        }
    }

    fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(MsgType::Config),
            1 => Some(MsgType::Ack),
            2 => Some(MsgType::Sensor),
            3 => Some(MsgType::Motors),
            4 => Some(MsgType::SupervisorConfig),
            5 => Some(MsgType::Calibration),
            6 => Some(MsgType::Command),
            _ => None,
        }
    }
}

const CONFIG_F32: [usize; 9] = [0, 4, 8, 12, 16, 20, 24, 28, 32];
const MOTORS_F32: [usize; 4] = [0, 4, 8, 12];
const SUPERVISOR_CONFIG_F32: [usize; 6] = [0, 4, 8, 12, 16, 20];
const CALIBRATION_F32: [usize; 18] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68,
];
/// imu 0..24, dt-s 24, position-ned 29/33/37, mag-body 42/46/50, heading-rad
/// 55, battery-v 77. The presence bytes (28, 41, 54, 59, 76) and the motor-rpm
/// `s32`s (60..76) are not float slots.
const SENSOR_F32: [usize; 15] = [0, 4, 8, 12, 16, 20, 24, 29, 33, 37, 42, 46, 50, 55, 77];
/// The `Sensor` payload's PRESENCE bytes, in field order: position-ned 28,
/// mag-body 41, heading-rad 54, motor-rpm 59, battery-v 76. Each is strictly
/// 0 (ABSENT) or 1 (present) — anything else is [`WireError::BadBool`]
/// (an integrator's encoder probe found a presence byte of 2 decoding as
/// present — jess#167).
pub const SENSOR_PRESENCE: [usize; 5] = [28, 41, 54, 59, 76];

/// Is this little-endian `f32` bit pattern finite?
///
/// An exponent-field test on the raw bits rather than `f32::is_finite`: a
/// single integer mask-and-compare, which needs no float in the decoder at
/// all. The Kani harnesses judge finiteness with a DIFFERENT integer
/// formulation (shifting the exponent out of bits assembled byte by byte), so
/// the proof is not the decoder agreeing with a copy of itself. All-ones
/// exponent = NaN (any payload, either sign) or ±infinity.
fn finite_bits(b: [u8; 4]) -> bool {
    (u32::from_le_bytes(b) & 0x7f80_0000) != 0x7f80_0000
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
    /// An `f32` slot holds NaN or ±infinity (#527). `offset` is the slot's
    /// PAYLOAD offset, so a host can name the field from the layout table.
    ///
    /// Rejected rather than passed through: a NaN motor command or estimator
    /// input does not fail loudly, it propagates. Absence is said with a
    /// presence byte, never with a non-finite value.
    NonFinite { msg: MsgType, offset: usize },
    /// A `Command` byte that is not a `pilot-command` discriminant (0..=7).
    BadCommand { got: u8 },
    /// A boolean byte that is neither 0 nor 1. Two kinds of byte are
    /// boolean on this wire, and both are refused on anything else:
    ///
    /// - `Command`'s `level` / `throttle-low` (offsets 1, 2): ARMING
    ///   INTERLOCKS — reading a stray 0xAB as `true` would be the fail-unsafe
    ///   direction.
    /// - `Sensor`'s five PRESENCE bytes ([`SENSOR_PRESENCE`]: 28, 41, 54, 59,
    ///   76), each the 0/1 discriminant of an `option`. Before this, any
    ///   non-zero value read as present (found by an integrator's probe,
    ///   jess#167), so a corrupted byte became a reading
    ///   instead of a rejection.
    ///
    /// One variant for both, not a separate `BadPresence`: the rule is the
    /// same (0 or 1, nothing else), and `msg` + `offset` already name which
    /// byte it was, so a host that handles `BadBool` handles both.
    BadBool {
        msg: MsgType,
        offset: usize,
        got: u8,
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
            WireError::NonFinite { msg, offset } => {
                write!(f, "{msg:?} payload: non-finite f32 at offset {offset}")
            }
            WireError::BadCommand { got } => write!(f, "unknown pilot-command {got}"),
            WireError::BadBool { msg, offset, got } => {
                write!(
                    f,
                    "{msg:?} payload: bool at offset {offset} is {got}, want 0 or 1"
                )
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

/// Decode a header and validate the payload that follows it — its length, and
/// (v2) its VALUES: every `f32` slot finite (#527), every `Sensor` presence
/// byte 0 or 1, and every `Command` byte in range. A frame this returns `Ok`
/// for is one every `decode_*` reader accepts.
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
    validate_values(msg, payload)?;
    Ok((
        Header {
            version: buf[2],
            msg,
            seq,
        },
        payload,
    ))
}

/// The value rules, given a payload of the right length for `msg`.
///
/// Shared by [`decode_frame`] and every `decode_*` reader, so a caller that
/// holds a bare payload gets the same verdict as one that decoded the frame.
fn validate_values(msg: MsgType, payload: &[u8]) -> Result<(), WireError> {
    for &at in msg.f32_offsets() {
        let mut b = [0u8; 4];
        b.copy_from_slice(&payload[at..at + 4]);
        if !finite_bits(b) {
            return Err(WireError::NonFinite { msg, offset: at });
        }
    }
    if msg == MsgType::Sensor {
        // Presence is 0 or 1, never "non-zero" (jess#167). Checked AFTER the
        // float slots, in ascending offset order, so the first error a frame
        // has is the one reported (the Kani sensor harness pins that order).
        for at in SENSOR_PRESENCE {
            if payload[at] > 1 {
                return Err(WireError::BadBool {
                    msg,
                    offset: at,
                    got: payload[at],
                });
            }
        }
    }
    if msg == MsgType::Command {
        if WirePilotCommand::from_u8(payload[0]).is_none() {
            return Err(WireError::BadCommand { got: payload[0] });
        }
        for at in [1usize, 2] {
            if payload[at] > 1 {
                return Err(WireError::BadBool {
                    msg,
                    offset: at,
                    got: payload[at],
                });
            }
        }
    }
    Ok(())
}

/// Length check plus [`validate_values`], for the payload readers.
fn check_payload(msg: MsgType, payload: &[u8]) -> Result<(), WireError> {
    let want = msg.payload_len();
    if payload.len() != want {
        return Err(WireError::BadLength {
            msg,
            got: payload.len(),
            want,
        });
    }
    validate_values(msg, payload)
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

    /// A tracker whose baseline is `seq`, as if `seq` had already been
    /// accepted.
    ///
    /// For a receiver that KNOWS where the stream starts — a host classifying
    /// replies in its own sequence space, which it numbered from `seq`. An
    /// unseeded tracker reads its first frame as `First` whatever its value,
    /// which hides the loss of every frame sent before it.
    pub const fn starting_at(seq: u32) -> Self {
        Self { last: Some(seq) }
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
    /// v2 (#572). Pack voltage (V), or `None` when the host has no battery
    /// sense — carried with its own presence byte, so a present 0.0 V (a dead
    /// pack, which the battery failsafe must see) and ABSENT (no sense at all,
    /// which the supervisor treats as "not yet declared") stay distinct.
    pub battery_v: Option<f32>,
}

/// `pilot.supervisor-config`, in WIT declaration order. Each field's unit and
/// sign are the WIT's own (read there, not restated loosely here):
/// `home` is NED metres; `fence-radius` metres; `cruise-alt` metres AGL and
/// POSITIVE (the one altitude in the package that is not NED-signed);
/// `low-batt-v` volts for the whole pack.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WireSupervisorConfig {
    pub home: [f32; 3],
    pub fence_radius: f32,
    pub cruise_alt: f32,
    pub low_batt_v: f32,
}

/// `pilot.sensor-calibration` (relay#611), in WIT declaration order. Units
/// per the WIT: gyro-bias rad/s, accel-bias m/s², the scales and the
/// off-diagonal terms unitless, mag-offset in the host's own `mag-body` unit.
///
/// All six fields map field for field onto `relay_calib::CalParams`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct WireCalibration {
    pub gyro_bias: [f32; 3],
    pub accel_bias: [f32; 3],
    pub accel_scale: [f32; 3],
    pub mag_offset: [f32; 3],
    pub mag_scale: [f32; 3],
    /// Symmetric soft-iron OFF-DIAGONAL terms (unitless): with `mag_scale` as
    /// the diagonal, the whole symmetric 3x3 soft-iron matrix (the shape PX4
    /// calibrates as `CAL_MAGn_ODIAG`), ordered `[xy, xz, yz]` as in
    /// `relay_calib::CalParams::mag_offdiag` and PX4's `set_offdiagonal`.
    pub mag_offdiag: [f32; 3],
}

/// `pilot.pilot-command`. The wire value is the WIT DECLARATION ORDER, pinned
/// explicitly here so a reorder of this enum cannot renumber the wire.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum WirePilotCommand {
    Arm = 0,
    RequestTakeoff = 1,
    RequestMission = 2,
    RequestLoiter = 3,
    RequestRtl = 4,
    RequestLand = 5,
    RequestDisarm = 6,
    Terminate = 7,
}

impl WirePilotCommand {
    pub fn from_u8(v: u8) -> Option<Self> {
        Some(match v {
            0 => Self::Arm,
            1 => Self::RequestTakeoff,
            2 => Self::RequestMission,
            3 => Self::RequestLoiter,
            4 => Self::RequestRtl,
            5 => Self::RequestLand,
            6 => Self::RequestDisarm,
            7 => Self::Terminate,
            _ => return None,
        })
    }
}

/// One `pilot.command(cmd, level, throttle-low)` call.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct WireCommand {
    pub cmd: WirePilotCommand,
    /// RC interlock held by the host: the vehicle is level.
    pub level: bool,
    /// RC interlock held by the host: the throttle stick is low.
    pub throttle_low: bool,
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
///
/// THE ECHO RULE (docs/TRANSPORT-BINDING.md §3): a component answering a
/// `Sensor` frame passes THAT frame's seq here, never a counter of its own, so
/// a host can index the reply to its tick under loss.
pub fn encode_motors(out: &mut [u8; HEADER_LEN + MOTORS_PAYLOAD_LEN], seq: u32, m: [f32; 4]) {
    encode_header(out, MsgType::Motors, seq).expect("buffer is exactly one frame");
    for (i, v) in m.iter().enumerate() {
        put_f32(out, HEADER_LEN + i * 4, *v);
    }
}

/// Decode a `Motors` payload (the slice [`decode_frame`] handed back).
pub fn decode_motors(payload: &[u8]) -> Result<[f32; 4], WireError> {
    check_payload(MsgType::Motors, payload)?;
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
    check_payload(MsgType::Config, payload)?;
    let mut a = [0.0f32; 9];
    for (i, ai) in a.iter_mut().enumerate() {
        *ai = get_f32(payload, i * 4);
    }
    Ok(WireVehicleConfig::from_array(a))
}

/// Encode a whole `Ack` frame. `acked` is the sequence being acknowledged.
///
/// Under the echo rule a component passes the acknowledged seq as BOTH `seq`
/// (the header) and `acked` (the payload). They are separate parameters
/// because the payload predates the rule and the layout is unchanged; a
/// receiver correlates on the payload and classifies on the header.
pub fn encode_ack(out: &mut [u8; HEADER_LEN + ACK_PAYLOAD_LEN], seq: u32, acked: u32) {
    encode_header(out, MsgType::Ack, seq).expect("buffer is exactly one frame");
    out[HEADER_LEN..HEADER_LEN + 4].copy_from_slice(&acked.to_le_bytes());
}

/// Decode an `Ack` payload into the sequence it acknowledges.
pub fn decode_ack(payload: &[u8]) -> Result<u32, WireError> {
    check_payload(MsgType::Ack, payload)?;
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
    // battery-v (v2, #572). The presence byte IS the ABSENT bit; the value is
    // never consulted to decide presence, so 0.0 V present stays present.
    out[b + 76] = f.battery_v.is_some() as u8;
    put_f32(out, b + 77, f.battery_v.unwrap_or(0.0));
}

/// Decode a `Sensor` payload. A field is present EXACTLY when its presence
/// byte is 1; `check_payload` has already refused any presence byte above 1.
pub fn decode_sensor_frame(payload: &[u8]) -> Result<WireSensorFrame, WireError> {
    check_payload(MsgType::Sensor, payload)?;
    let mut f = WireSensorFrame::default();
    for (i, v) in f.imu.iter_mut().enumerate() {
        *v = get_f32(payload, i * 4);
    }
    f.dt_s = get_f32(payload, 24);
    if payload[28] == 1 {
        f.position_ned = Some([
            get_f32(payload, 29),
            get_f32(payload, 33),
            get_f32(payload, 37),
        ]);
    }
    if payload[41] == 1 {
        f.mag_body = Some([
            get_f32(payload, 42),
            get_f32(payload, 46),
            get_f32(payload, 50),
        ]);
    }
    if payload[54] == 1 {
        f.heading_rad = Some(get_f32(payload, 55));
    }
    if payload[59] == 1 {
        f.motor_rpm = Some([
            get_i32(payload, 60),
            get_i32(payload, 64),
            get_i32(payload, 68),
            get_i32(payload, 72),
        ]);
    }
    if payload[76] == 1 {
        f.battery_v = Some(get_f32(payload, 77));
    }
    Ok(f)
}

/// Encode a whole `SupervisorConfig` frame (v2).
pub fn encode_supervisor_config(
    out: &mut [u8; HEADER_LEN + SUPERVISOR_CONFIG_PAYLOAD_LEN],
    seq: u32,
    c: &WireSupervisorConfig,
) {
    encode_header(out, MsgType::SupervisorConfig, seq).expect("buffer is exactly one frame");
    let b = HEADER_LEN;
    for (i, v) in c.home.iter().enumerate() {
        put_f32(out, b + i * 4, *v);
    }
    put_f32(out, b + 12, c.fence_radius);
    put_f32(out, b + 16, c.cruise_alt);
    put_f32(out, b + 20, c.low_batt_v);
}

/// Decode a `SupervisorConfig` payload.
pub fn decode_supervisor_config(payload: &[u8]) -> Result<WireSupervisorConfig, WireError> {
    check_payload(MsgType::SupervisorConfig, payload)?;
    Ok(WireSupervisorConfig {
        home: [
            get_f32(payload, 0),
            get_f32(payload, 4),
            get_f32(payload, 8),
        ],
        fence_radius: get_f32(payload, 12),
        cruise_alt: get_f32(payload, 16),
        low_batt_v: get_f32(payload, 20),
    })
}

/// Encode a whole `Calibration` frame (v2).
pub fn encode_calibration(
    out: &mut [u8; HEADER_LEN + CALIBRATION_PAYLOAD_LEN],
    seq: u32,
    c: &WireCalibration,
) {
    encode_header(out, MsgType::Calibration, seq).expect("buffer is exactly one frame");
    let groups = [
        c.gyro_bias,
        c.accel_bias,
        c.accel_scale,
        c.mag_offset,
        c.mag_scale,
        // appended at payload 60 — [xy, xz, yz], see the struct
        c.mag_offdiag,
    ];
    for (g, v3) in groups.iter().enumerate() {
        for (i, v) in v3.iter().enumerate() {
            put_f32(out, HEADER_LEN + g * 12 + i * 4, *v);
        }
    }
}

/// Decode a `Calibration` payload.
pub fn decode_calibration(payload: &[u8]) -> Result<WireCalibration, WireError> {
    check_payload(MsgType::Calibration, payload)?;
    let v3 = |g: usize| {
        [
            get_f32(payload, g * 12),
            get_f32(payload, g * 12 + 4),
            get_f32(payload, g * 12 + 8),
        ]
    };
    Ok(WireCalibration {
        gyro_bias: v3(0),
        accel_bias: v3(1),
        accel_scale: v3(2),
        mag_offset: v3(3),
        mag_scale: v3(4),
        mag_offdiag: v3(5),
    })
}

/// Encode a whole `Command` frame (v2).
pub fn encode_command(out: &mut [u8; HEADER_LEN + COMMAND_PAYLOAD_LEN], seq: u32, c: &WireCommand) {
    encode_header(out, MsgType::Command, seq).expect("buffer is exactly one frame");
    out[HEADER_LEN] = c.cmd as u8;
    out[HEADER_LEN + 1] = c.level as u8;
    out[HEADER_LEN + 2] = c.throttle_low as u8;
}

/// Decode a `Command` payload.
pub fn decode_command(payload: &[u8]) -> Result<WireCommand, WireError> {
    check_payload(MsgType::Command, payload)?;
    Ok(WireCommand {
        // `check_payload` already proved this in range; the `ok_or` is the
        // no-panic path rather than an `unwrap`.
        cmd: WirePilotCommand::from_u8(payload[0])
            .ok_or(WireError::BadCommand { got: payload[0] })?,
        level: payload[1] == 1,
        throttle_low: payload[2] == 1,
    })
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
        want[..4].copy_from_slice(&[0x1c, 0xfa, 0x02, 0x03]); // magic, v2, Motors
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
            [0x1c, 0xfa, 0x02, 0x01, 0x09, 0, 0, 0, 0x05, 0, 0, 0],
            "ack frame bytes"
        );
    }

    /// A REGRESSION LOCK on the frame geometry, not a derivation. The header
    /// length is a NORMATIVE CHOICE (see the module note): the requirement's own
    /// field list sums to 8 while its prose says 6, and the captured 24 B motors
    /// frame is consistent with 8 but comes from a sentence whose evidence is
    /// not in this repository (#503). This test pins the choice so it cannot
    /// drift silently — it does not establish that 8 is what the wire carried.
    #[test]
    fn the_frame_geometry_stays_pinned() {
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
            battery_v: Some(15.2),
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
            battery_v: None,
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
        assert_eq!(
            got.battery_v, None,
            "absent battery must not read as 0 V (#572)"
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
                battery_v: Some(1.0),
                ..Default::default()
            },
        );
        assert_eq!(a.len(), b.len());
        assert!(decode_frame(&a).is_ok() && decode_frame(&b).is_ok());
    }

    // ── binding v2: the new frames, by hand-computed bytes ──────────────
    //
    // Each vector below is written out byte by byte from the IEEE-754 bit
    // patterns, NOT produced by the encoder: little-endian, so the LOW byte of
    // the f32's bit pattern comes first. The patterns used:
    //
    //   0.0    0x00000000  00 00 00 00      2.0    0x40000000  00 00 00 40
    //   0.125  0x3E000000  00 00 00 3e      4.0    0x40800000  00 00 80 40
    //   0.25   0x3E800000  00 00 80 3e     14.0    0x41600000  00 00 60 41
    //   0.5    0x3F000000  00 00 00 3f     16.0    0x41800000  00 00 80 41
    //   0.75   0x3F400000  00 00 40 3f     50.0    0x42480000  00 00 48 42
    //   1.0    0x3F800000  00 00 80 3f     -0.5    0xBF000000  00 00 00 bf
    //   1.5    0x3FC00000  00 00 c0 3f     -1.0    0xBF800000  00 00 80 bf
    //                                      -2.0    0xC0000000  00 00 00 c0
    //
    // A vector is checked BOTH ways: the encoder must emit it, and the decoder
    // must read the intended values back out of it.

    /// `SupervisorConfig` (type 4), seq 0x0102: home (1.0, -2.0, 0.0) NED m,
    /// fence-radius 50.0 m, cruise-alt 2.0 m AGL (positive), low-batt-v 14.0 V.
    const SUPCFG_VECTOR: [u8; 32] = [
        0x1c, 0xfa, 0x02, 0x04, 0x02, 0x01, 0x00, 0x00, // magic, v2, type 4, seq 0x0102
        0x00, 0x00, 0x80, 0x3f, // 0  home.x (north)  1.0
        0x00, 0x00, 0x00, 0xc0, // 4  home.y (east)  -2.0
        0x00, 0x00, 0x00, 0x00, // 8  home.z (down)   0.0
        0x00, 0x00, 0x48, 0x42, // 12 fence-radius   50.0
        0x00, 0x00, 0x00, 0x40, // 16 cruise-alt      2.0
        0x00, 0x00, 0x60, 0x41, // 20 low-batt-v     14.0
    ];

    #[test]
    fn a_supervisor_config_frame_matches_its_hand_computed_bytes() {
        let c = WireSupervisorConfig {
            home: [1.0, -2.0, 0.0],
            fence_radius: 50.0,
            cruise_alt: 2.0,
            low_batt_v: 14.0,
        };
        let mut out = [0u8; HEADER_LEN + SUPERVISOR_CONFIG_PAYLOAD_LEN];
        encode_supervisor_config(&mut out, 0x0102, &c);
        assert_eq!(out, SUPCFG_VECTOR, "supervisor-config frame bytes");
        let (h, p) = decode_frame(&SUPCFG_VECTOR).unwrap();
        assert_eq!((h.msg, h.seq), (MsgType::SupervisorConfig, 0x0102));
        assert_eq!(decode_supervisor_config(p).unwrap(), c);
    }

    /// `Calibration` (type 5), seq 3, in #611's `sensor-calibration` order:
    /// gyro-bias (0.125, 0, -0.5), accel-bias (0.25, 0, 0), accel-scale
    /// (1.0, 1.5, 0.75), mag-offset (4.0, 0, -1.0), mag-scale (1.0, 2.0, 0.5),
    /// mag-offdiag (0.25, -0.5, 0.125) in [xy, xz, yz] order.
    const CAL_VECTOR: [u8; 80] = [
        0x1c, 0xfa, 0x02, 0x05, 0x03, 0x00, 0x00, 0x00, // magic, v2, type 5, seq 3
        0x00, 0x00, 0x00, 0x3e, // 0  gyro-bias.x    0.125
        0x00, 0x00, 0x00, 0x00, // 4  gyro-bias.y    0.0
        0x00, 0x00, 0x00, 0xbf, // 8  gyro-bias.z   -0.5
        0x00, 0x00, 0x80, 0x3e, // 12 accel-bias.x   0.25
        0x00, 0x00, 0x00, 0x00, // 16 accel-bias.y   0.0
        0x00, 0x00, 0x00, 0x00, // 20 accel-bias.z   0.0
        0x00, 0x00, 0x80, 0x3f, // 24 accel-scale.x  1.0
        0x00, 0x00, 0xc0, 0x3f, // 28 accel-scale.y  1.5
        0x00, 0x00, 0x40, 0x3f, // 32 accel-scale.z  0.75
        0x00, 0x00, 0x80, 0x40, // 36 mag-offset.x   4.0
        0x00, 0x00, 0x00, 0x00, // 40 mag-offset.y   0.0
        0x00, 0x00, 0x80, 0xbf, // 44 mag-offset.z  -1.0
        0x00, 0x00, 0x80, 0x3f, // 48 mag-scale.x    1.0
        0x00, 0x00, 0x00, 0x40, // 52 mag-scale.y    2.0
        0x00, 0x00, 0x00, 0x3f, // 56 mag-scale.z    0.5
        0x00, 0x00, 0x80, 0x3e, // 60 mag-offdiag.xy  0.25
        0x00, 0x00, 0x00, 0xbf, // 64 mag-offdiag.xz -0.5
        0x00, 0x00, 0x00, 0x3e, // 68 mag-offdiag.yz  0.125
    ];

    #[test]
    fn a_calibration_frame_matches_its_hand_computed_bytes() {
        let c = WireCalibration {
            gyro_bias: [0.125, 0.0, -0.5],
            accel_bias: [0.25, 0.0, 0.0],
            accel_scale: [1.0, 1.5, 0.75],
            mag_offset: [4.0, 0.0, -1.0],
            mag_scale: [1.0, 2.0, 0.5],
            mag_offdiag: [0.25, -0.5, 0.125],
        };
        let mut out = [0u8; HEADER_LEN + CALIBRATION_PAYLOAD_LEN];
        encode_calibration(&mut out, 3, &c);
        assert_eq!(out, CAL_VECTOR, "calibration frame bytes");
        let (h, p) = decode_frame(&CAL_VECTOR).unwrap();
        assert_eq!((h.msg, h.seq), (MsgType::Calibration, 3));
        assert_eq!(decode_calibration(p).unwrap(), c);
    }

    /// `Command` (type 6), seq 0x01020304: request-takeoff (1), level = true,
    /// throttle-low = false.
    const CMD_VECTOR: [u8; 11] = [
        0x1c, 0xfa, 0x02, 0x06, 0x04, 0x03, 0x02, 0x01, // magic, v2, type 6, seq 0x01020304
        0x01, // 0 cmd           request-takeoff
        0x01, // 1 level         true
        0x00, // 2 throttle-low  false
    ];

    #[test]
    fn a_command_frame_matches_its_hand_computed_bytes() {
        let c = WireCommand {
            cmd: WirePilotCommand::RequestTakeoff,
            level: true,
            throttle_low: false,
        };
        let mut out = [0u8; HEADER_LEN + COMMAND_PAYLOAD_LEN];
        encode_command(&mut out, 0x0102_0304, &c);
        assert_eq!(out, CMD_VECTOR, "command frame bytes");
        let (h, p) = decode_frame(&CMD_VECTOR).unwrap();
        assert_eq!((h.msg, h.seq), (MsgType::Command, 0x0102_0304));
        assert_eq!(decode_command(p).unwrap(), c);
    }

    /// `Sensor` (type 2) at v2, seq 2, every element exercised: position and
    /// heading present, mag ABSENT, rpm present — including -8388608, whose
    /// bytes `00 00 80 ff` are the -inf pattern and must NOT trip the
    /// non-finite rule because rpm is an s32 — and battery-v PRESENT at 16.0 V.
    const SENSOR_VECTOR: [u8; 89] = [
        0x1c, 0xfa, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, // magic, v2, type 2, seq 2
        0x00, 0x00, 0x00, 0x3f, // 0  imu.ax   0.5
        0x00, 0x00, 0x00, 0x00, // 4  imu.ay   0.0
        0x00, 0x00, 0x80, 0xbf, // 8  imu.az  -1.0
        0x00, 0x00, 0x00, 0x00, // 12 imu.gx   0.0
        0x00, 0x00, 0x80, 0x3e, // 16 imu.gy   0.25
        0x00, 0x00, 0x00, 0x00, // 20 imu.gz   0.0
        0x00, 0x00, 0x00, 0x3e, // 24 dt-s     0.125
        0x01, // 28 position-ned present
        0x00, 0x00, 0x80, 0x3f, // 29 position-ned.x  1.0
        0x00, 0x00, 0x00, 0x40, // 33 position-ned.y  2.0
        0x00, 0x00, 0x00, 0xc0, // 37 position-ned.z -2.0
        0x00, // 41 mag-body ABSENT
        0x00, 0x00, 0x00, 0x00, // 42 mag-body.x (zeroed)
        0x00, 0x00, 0x00, 0x00, // 46 mag-body.y (zeroed)
        0x00, 0x00, 0x00, 0x00, // 50 mag-body.z (zeroed)
        0x01, // 54 heading-rad present
        0x00, 0x00, 0x00, 0x3f, // 55 heading-rad 0.5
        0x01, // 59 motor-rpm present
        0x01, 0x00, 0x00, 0x00, // 60 motor-rpm.m1         1
        0xff, 0xff, 0xff, 0xff, // 64 motor-rpm.m2        -1
        0x00, 0x01, 0x00, 0x00, // 68 motor-rpm.m3       256
        0x00, 0x00, 0x80, 0xff, // 72 motor-rpm.m4  -8388608
        0x01, // 76 battery-v present
        0x00, 0x00, 0x80, 0x41, // 77 battery-v 16.0
    ];

    #[test]
    fn a_v2_sensor_frame_matches_its_hand_computed_bytes() {
        let f = WireSensorFrame {
            imu: [0.5, 0.0, -1.0, 0.0, 0.25, 0.0],
            dt_s: 0.125,
            position_ned: Some([1.0, 2.0, -2.0]),
            mag_body: None,
            heading_rad: Some(0.5),
            motor_rpm: Some([1, -1, 256, -8_388_608]),
            battery_v: Some(16.0),
        };
        let mut out = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut out, 2, &f);
        assert_eq!(out, SENSOR_VECTOR, "v2 sensor frame bytes");
        let (h, p) = decode_frame(&SENSOR_VECTOR).expect("an s32 rpm is not a float slot");
        assert_eq!((h.msg, h.seq), (MsgType::Sensor, 2));
        assert_eq!(decode_sensor_frame(p).unwrap(), f);
    }

    /// #572: 0.0 V PRESENT and ABSENT differ ONLY in the presence byte at
    /// payload 76, and they must decode differently. A dead pack is a
    /// measurement the battery failsafe acts on; "no sense" is not.
    #[test]
    fn a_present_zero_volt_battery_is_not_absent() {
        let mut present = SENSOR_VECTOR;
        present[HEADER_LEN + 76..HEADER_LEN + 81].copy_from_slice(&[0x01, 0x00, 0x00, 0x00, 0x00]);
        let mut absent = SENSOR_VECTOR;
        absent[HEADER_LEN + 76..HEADER_LEN + 81].copy_from_slice(&[0x00, 0x00, 0x00, 0x00, 0x00]);

        let (_, p) = decode_frame(&present).unwrap();
        assert_eq!(
            decode_sensor_frame(p).unwrap().battery_v,
            Some(0.0),
            "a present 0.0 V must read as 0.0 V, not as absent"
        );
        let (_, p) = decode_frame(&absent).unwrap();
        assert_eq!(
            decode_sensor_frame(p).unwrap().battery_v,
            None,
            "presence byte 0 must read as ABSENT"
        );

        // and the encoder produces exactly those two byte strings
        let mut f = decode_sensor_frame(&present[HEADER_LEN..]).unwrap();
        let mut out = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut out, 2, &f);
        assert_eq!(out, present);
        f.battery_v = None;
        encode_sensor_frame(&mut out, 2, &f);
        assert_eq!(out, absent);
    }

    #[test]
    fn every_payload_length_is_pinned() {
        // Literals on the right, so a constant that drifts fails here.
        assert_eq!(MsgType::Config.payload_len(), 36);
        assert_eq!(MsgType::Ack.payload_len(), 4);
        assert_eq!(MsgType::Sensor.payload_len(), 81);
        assert_eq!(MsgType::Motors.payload_len(), 16);
        assert_eq!(MsgType::SupervisorConfig.payload_len(), 24);
        assert_eq!(MsgType::Calibration.payload_len(), 72);
        assert_eq!(MsgType::Command.payload_len(), 3);
        assert_eq!(VERSION, 2, "binding v2");
        // ...and decode_frame enforces each one: one byte short and one byte
        // long are both BadLength.
        for (t, n) in [(4u8, 24usize), (5, 72), (6, 3), (2, 81)] {
            let msg = MsgType::from_u8(t).unwrap();
            let mut buf = [0u8; HEADER_LEN + 82];
            encode_header(&mut buf, msg, 1).unwrap();
            if msg == MsgType::Command {
                buf[HEADER_LEN] = 0; // a valid command byte; bools already 0
            }
            assert!(
                decode_frame(&buf[..HEADER_LEN + n]).is_ok(),
                "{msg:?} exact"
            );
            assert_eq!(
                decode_frame(&buf[..HEADER_LEN + n - 1]),
                Err(WireError::BadLength {
                    msg,
                    got: n - 1,
                    want: n
                })
            );
            assert_eq!(
                decode_frame(&buf[..HEADER_LEN + n + 1]),
                Err(WireError::BadLength {
                    msg,
                    got: n + 1,
                    want: n
                })
            );
        }
    }

    #[test]
    fn the_new_frames_round_trip() {
        let s = WireSupervisorConfig {
            home: [10.5, -3.25, 0.0],
            fence_radius: 120.0,
            cruise_alt: 30.0,
            low_batt_v: 21.6,
        };
        let mut b = [0u8; HEADER_LEN + SUPERVISOR_CONFIG_PAYLOAD_LEN];
        encode_supervisor_config(&mut b, 9, &s);
        let (_, p) = decode_frame(&b).unwrap();
        assert_eq!(decode_supervisor_config(p).unwrap(), s);

        let c = WireCalibration {
            gyro_bias: [0.001, -0.002, 0.0005],
            accel_bias: [0.05, -0.03, 0.1],
            accel_scale: [1.01, 0.99, 1.002],
            mag_offset: [12.0, -7.5, 3.0],
            mag_scale: [0.98, 1.03, 1.0],
            mag_offdiag: [0.01, -0.02, 0.003],
        };
        let mut b = [0u8; HEADER_LEN + CALIBRATION_PAYLOAD_LEN];
        encode_calibration(&mut b, 10, &c);
        let (_, p) = decode_frame(&b).unwrap();
        assert_eq!(decode_calibration(p).unwrap(), c);

        for v in 0u8..=7 {
            for (level, throttle_low) in
                [(false, false), (true, false), (false, true), (true, true)]
            {
                let c = WireCommand {
                    cmd: WirePilotCommand::from_u8(v).unwrap(),
                    level,
                    throttle_low,
                };
                let mut b = [0u8; HEADER_LEN + COMMAND_PAYLOAD_LEN];
                encode_command(&mut b, 11, &c);
                let (_, p) = decode_frame(&b).unwrap();
                assert_eq!(decode_command(p).unwrap(), c);
            }
        }
    }

    /// The `pilot-command` wire values are the WIT declaration order, by name.
    #[test]
    fn pilot_command_values_are_the_wit_declaration_order() {
        assert_eq!(WirePilotCommand::Arm as u8, 0);
        assert_eq!(WirePilotCommand::RequestTakeoff as u8, 1);
        assert_eq!(WirePilotCommand::RequestMission as u8, 2);
        assert_eq!(WirePilotCommand::RequestLoiter as u8, 3);
        assert_eq!(WirePilotCommand::RequestRtl as u8, 4);
        assert_eq!(WirePilotCommand::RequestLand as u8, 5);
        assert_eq!(WirePilotCommand::RequestDisarm as u8, 6);
        assert_eq!(WirePilotCommand::Terminate as u8, 7);
        assert_eq!(WirePilotCommand::from_u8(8), None);
    }

    /// An out-of-range command byte, and any interlock byte other than 0/1,
    /// is refused — a stray byte must never read as `level = true`.
    #[test]
    fn a_malformed_command_is_refused() {
        let mut f = CMD_VECTOR;
        f[HEADER_LEN] = 8;
        assert_eq!(decode_frame(&f), Err(WireError::BadCommand { got: 8 }));
        f[HEADER_LEN] = 0xff;
        assert_eq!(decode_frame(&f), Err(WireError::BadCommand { got: 0xff }));
        for (at, got) in [(1usize, 2u8), (1, 0xab), (2, 2), (2, 0xff)] {
            let mut f = CMD_VECTOR;
            f[HEADER_LEN + at] = got;
            assert_eq!(
                decode_frame(&f),
                Err(WireError::BadBool {
                    msg: MsgType::Command,
                    offset: at,
                    got
                }),
                "interlock byte {at} = {got:#x}"
            );
        }
    }

    // ── #527: NON-FINITE IS REJECTED, for every float field of every frame ──

    /// Non-finite bit patterns, not just `f32::NAN`: a decoder that compared
    /// against the canonical NaN would let a signalling or negative NaN by.
    const NON_FINITE: [u32; 6] = [
        0x7fc0_0000, // canonical quiet NaN
        0xffc0_0000, // negative quiet NaN
        0x7f80_0001, // signalling NaN, lowest payload
        0x7fff_ffff, // NaN, all payload bits
        0x7f80_0000, // +inf
        0xff80_0000, // -inf
    ];

    /// One float FIELD of one frame type: its name (as the layout table names
    /// it), its PAYLOAD offset — written here from the WIT field order, not
    /// read from the decoder's own slot list — and a frame builder that puts
    /// `v` in that field THROUGH THE STRUCT, so the encoder places it.
    struct FloatField {
        name: &'static str,
        offset: usize,
        frame: fn(f32) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize),
    }

    fn buf<const N: usize>(b: [u8; N]) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize) {
        let mut out = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        out[..N].copy_from_slice(&b);
        (out, N)
    }
    fn cfg_with(i: usize, v: f32) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize) {
        let mut a = [0.5f32; 9];
        a[i] = v;
        let mut b = [0u8; HEADER_LEN + CONFIG_PAYLOAD_LEN];
        encode_config(&mut b, 1, &WireVehicleConfig::from_array(a));
        buf(b)
    }
    fn motors_with(i: usize, v: f32) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize) {
        let mut m = [0.5f32; 4];
        m[i] = v;
        let mut b = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut b, 1, m);
        buf(b)
    }
    fn sup_with(i: usize, v: f32) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize) {
        let mut c = WireSupervisorConfig {
            home: [1.0, 2.0, 3.0],
            fence_radius: 50.0,
            cruise_alt: 2.0,
            low_batt_v: 14.0,
        };
        match i {
            0..=2 => c.home[i] = v,
            3 => c.fence_radius = v,
            4 => c.cruise_alt = v,
            _ => c.low_batt_v = v,
        }
        let mut b = [0u8; HEADER_LEN + SUPERVISOR_CONFIG_PAYLOAD_LEN];
        encode_supervisor_config(&mut b, 1, &c);
        buf(b)
    }
    fn cal_with(i: usize, v: f32) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize) {
        let mut c = WireCalibration {
            gyro_bias: [0.0; 3],
            accel_bias: [0.0; 3],
            accel_scale: [1.0; 3],
            mag_offset: [0.0; 3],
            mag_scale: [1.0; 3],
            mag_offdiag: [0.0; 3],
        };
        let g = match i / 3 {
            0 => &mut c.gyro_bias,
            1 => &mut c.accel_bias,
            2 => &mut c.accel_scale,
            3 => &mut c.mag_offset,
            4 => &mut c.mag_scale,
            _ => &mut c.mag_offdiag,
        };
        g[i % 3] = v;
        let mut b = [0u8; HEADER_LEN + CALIBRATION_PAYLOAD_LEN];
        encode_calibration(&mut b, 1, &c);
        buf(b)
    }
    fn sensor_with(i: usize, v: f32) -> ([u8; HEADER_LEN + SENSOR_PAYLOAD_LEN], usize) {
        let mut f = WireSensorFrame {
            imu: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0],
            dt_s: 0.004,
            position_ned: Some([0.0; 3]),
            mag_body: Some([0.2, 0.0, 0.4]),
            heading_rad: Some(0.0),
            motor_rpm: Some([0; 4]),
            battery_v: Some(16.0),
        };
        match i {
            0..=5 => f.imu[i] = v,
            6 => f.dt_s = v,
            7..=9 => f.position_ned.as_mut().unwrap()[i - 7] = v,
            10..=12 => f.mag_body.as_mut().unwrap()[i - 10] = v,
            13 => f.heading_rad = Some(v),
            _ => f.battery_v = Some(v),
        }
        let mut b = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut b, 1, &f);
        (b, HEADER_LEN + SENSOR_PAYLOAD_LEN)
    }

    macro_rules! field {
        ($name:literal, $off:expr, $f:ident, $i:expr) => {
            FloatField {
                name: $name,
                offset: $off,
                frame: |v| $f($i, v),
            }
        };
    }

    fn float_fields(msg: MsgType) -> &'static [FloatField] {
        const CONFIG: [FloatField; 9] = [
            field!("hover-thrust", 0, cfg_with, 0),
            field!("loop-rate-hz", 4, cfg_with, 1),
            field!("pos-var", 8, cfg_with, 2),
            field!("process-floor-vel", 12, cfg_with, 3),
            field!("process-floor-pos", 16, cfg_with, 4),
            field!("altitude-kp", 20, cfg_with, 5),
            field!("altitude-kd", 24, cfg_with, 6),
            field!("altitude-ki", 28, cfg_with, 7),
            field!("position-ki", 32, cfg_with, 8),
        ];
        const MOTORS: [FloatField; 4] = [
            field!("m1", 0, motors_with, 0),
            field!("m2", 4, motors_with, 1),
            field!("m3", 8, motors_with, 2),
            field!("m4", 12, motors_with, 3),
        ];
        const SUP: [FloatField; 6] = [
            field!("home.x", 0, sup_with, 0),
            field!("home.y", 4, sup_with, 1),
            field!("home.z", 8, sup_with, 2),
            field!("fence-radius", 12, sup_with, 3),
            field!("cruise-alt", 16, sup_with, 4),
            field!("low-batt-v", 20, sup_with, 5),
        ];
        const CAL: [FloatField; 18] = [
            field!("gyro-bias.x", 0, cal_with, 0),
            field!("gyro-bias.y", 4, cal_with, 1),
            field!("gyro-bias.z", 8, cal_with, 2),
            field!("accel-bias.x", 12, cal_with, 3),
            field!("accel-bias.y", 16, cal_with, 4),
            field!("accel-bias.z", 20, cal_with, 5),
            field!("accel-scale.x", 24, cal_with, 6),
            field!("accel-scale.y", 28, cal_with, 7),
            field!("accel-scale.z", 32, cal_with, 8),
            field!("mag-offset.x", 36, cal_with, 9),
            field!("mag-offset.y", 40, cal_with, 10),
            field!("mag-offset.z", 44, cal_with, 11),
            field!("mag-scale.x", 48, cal_with, 12),
            field!("mag-scale.y", 52, cal_with, 13),
            field!("mag-scale.z", 56, cal_with, 14),
            field!("mag-offdiag.xy", 60, cal_with, 15),
            field!("mag-offdiag.xz", 64, cal_with, 16),
            field!("mag-offdiag.yz", 68, cal_with, 17),
        ];
        const SENSOR: [FloatField; 15] = [
            field!("imu.ax", 0, sensor_with, 0),
            field!("imu.ay", 4, sensor_with, 1),
            field!("imu.az", 8, sensor_with, 2),
            field!("imu.gx", 12, sensor_with, 3),
            field!("imu.gy", 16, sensor_with, 4),
            field!("imu.gz", 20, sensor_with, 5),
            field!("dt-s", 24, sensor_with, 6),
            field!("position-ned.x", 29, sensor_with, 7),
            field!("position-ned.y", 33, sensor_with, 8),
            field!("position-ned.z", 37, sensor_with, 9),
            field!("mag-body.x", 42, sensor_with, 10),
            field!("mag-body.y", 46, sensor_with, 11),
            field!("mag-body.z", 50, sensor_with, 12),
            field!("heading-rad", 55, sensor_with, 13),
            field!("battery-v", 77, sensor_with, 14),
        ];
        match msg {
            MsgType::Config => &CONFIG,
            MsgType::Motors => &MOTORS,
            MsgType::SupervisorConfig => &SUP,
            MsgType::Calibration => &CAL,
            MsgType::Sensor => &SENSOR,
            MsgType::Ack | MsgType::Command => &[],
        }
    }

    const ALL_TYPES: [MsgType; 7] = [
        MsgType::Config,
        MsgType::Ack,
        MsgType::Sensor,
        MsgType::Motors,
        MsgType::SupervisorConfig,
        MsgType::Calibration,
        MsgType::Command,
    ];

    /// TABLE-DRIVEN: every float field of every frame type, every non-finite
    /// pattern, is rejected by `decode_frame` with `NonFinite` naming that
    /// field's offset — and by the payload reader too. On binding v1 every one
    /// of these decoded successfully (the Kani harness proved NaN ACCEPTED).
    #[test]
    fn a_non_finite_value_in_any_float_field_of_any_frame_is_rejected() {
        let mut checked = 0;
        for msg in ALL_TYPES {
            let fields = float_fields(msg);
            // COMPLETENESS: the test's field list and the decoder's slot list
            // agree in count and position — a slot the decoder forgot, or one
            // it checks that is not a float field, fails here.
            let mut want: [usize; 18] = [usize::MAX; 18];
            for (i, f) in fields.iter().enumerate() {
                want[i] = f.offset;
            }
            assert_eq!(
                &want[..fields.len()],
                msg.f32_offsets(),
                "{msg:?}: float slots"
            );
            for field in fields {
                // the control: a finite value in the same field decodes
                let (b, n) = (field.frame)(1.25);
                assert!(
                    decode_frame(&b[..n]).is_ok(),
                    "{msg:?}.{} control",
                    field.name
                );
                for bits in NON_FINITE {
                    let (b, n) = (field.frame)(f32::from_bits(bits));
                    assert_eq!(
                        decode_frame(&b[..n]),
                        Err(WireError::NonFinite {
                            msg,
                            offset: field.offset
                        }),
                        "{msg:?}.{} = {bits:#010x} must be rejected",
                        field.name
                    );
                    let payload = &b[HEADER_LEN..n];
                    let reader = match msg {
                        MsgType::Config => decode_config(payload).err(),
                        MsgType::Sensor => decode_sensor_frame(payload).err(),
                        MsgType::Motors => decode_motors(payload).err(),
                        MsgType::SupervisorConfig => decode_supervisor_config(payload).err(),
                        MsgType::Calibration => decode_calibration(payload).err(),
                        MsgType::Ack | MsgType::Command => unreachable!(),
                    };
                    assert_eq!(
                        reader,
                        Some(WireError::NonFinite {
                            msg,
                            offset: field.offset
                        }),
                        "{msg:?}.{} payload reader",
                        field.name
                    );
                    checked += 1;
                }
            }
        }
        // 9 + 15 + 4 + 6 + 18 = 52 float fields x 6 patterns
        assert_eq!(
            checked,
            52 * 6,
            "every float field of every frame type was exercised"
        );
    }

    /// Finite edge values are ACCEPTED: the rule is "finite", not "normal" or
    /// "non-negative". -0.0, the smallest subnormal and f32::MAX all decode.
    #[test]
    fn finite_edge_values_are_accepted() {
        for v in [
            -0.0f32,
            f32::from_bits(1),
            f32::MIN_POSITIVE,
            f32::MAX,
            f32::MIN,
        ] {
            for msg in ALL_TYPES {
                for field in float_fields(msg) {
                    let (b, n) = (field.frame)(v);
                    assert!(
                        decode_frame(&b[..n]).is_ok(),
                        "{msg:?}.{} = {v:e} is finite and must decode",
                        field.name
                    );
                }
            }
        }
    }

    /// The pinned decision on ABSENT slots: an optional field's value bytes are
    /// checked even when its presence byte is 0. The encoder writes 0.0 there;
    /// NaN-as-"no reading" is the sentinel habit the presence byte replaces.
    #[test]
    fn a_non_finite_value_in_an_absent_slot_is_still_rejected() {
        let mut b = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut b, 1, &WireSensorFrame::default());
        for (presence, slot) in [(28usize, 29usize), (41, 42), (54, 55), (76, 77)] {
            let mut f = b;
            assert_eq!(f[HEADER_LEN + presence], 0, "absent");
            f[HEADER_LEN + slot..HEADER_LEN + slot + 4].copy_from_slice(&f32::NAN.to_le_bytes());
            assert_eq!(
                decode_frame(&f),
                Err(WireError::NonFinite {
                    msg: MsgType::Sensor,
                    offset: slot
                })
            );
        }
    }

    /// The non-float bytes are NOT subject to the non-finite rule: rpm s32s
    /// whose bytes spell NaN / ±inf decode. A presence byte is not a float
    /// either — it has its OWN rule (0 or 1), so a 0xff there is
    /// `BadBool`, never `NonFinite`.
    #[test]
    fn integer_and_presence_bytes_are_not_float_slots() {
        let mut b = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(
            &mut b,
            1,
            &WireSensorFrame {
                motor_rpm: Some([
                    0x7fc0_0000u32 as i32,
                    0xffc0_0000u32 as i32,
                    0x7f80_0000,
                    0xff80_0000u32 as i32,
                ]),
                ..Default::default()
            },
        );
        assert!(decode_frame(&b).is_ok(), "rpm is s32, never a float slot");
        b[HEADER_LEN + 76] = 0xff; // a presence byte is a byte, not a float
        assert_eq!(
            decode_frame(&b),
            Err(WireError::BadBool {
                msg: MsgType::Sensor,
                offset: 76,
                got: 0xff
            })
        );
    }

    /// TABLE-DRIVEN: every `Sensor` presence byte (offsets written
    /// here from the layout table, not read from `SENSOR_PRESENCE`) accepts
    /// exactly 0 and 1, and refuses 2, 0x80 and 0xFF with `BadBool` naming
    /// that offset — through `decode_frame` AND the payload reader. On the
    /// base this amends, a 2 decoded as PRESENT (an integrator's probe measured
    /// it, jess#167).
    #[test]
    fn every_sensor_presence_byte_is_strictly_zero_or_one() {
        let full = WireSensorFrame {
            imu: [0.0, 0.0, -9.81, 0.0, 0.0, 0.0],
            dt_s: 0.004,
            position_ned: Some([1.0, 2.0, 3.0]),
            mag_body: Some([0.2, 0.0, 0.4]),
            heading_rad: Some(0.5),
            motor_rpm: Some([1, 2, 3, 4]),
            battery_v: Some(16.0),
        };
        let mut base = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut base, 1, &full);
        // (name, payload offset) — position-ned, mag-body, heading-rad,
        // motor-rpm, battery-v.
        let table: [(&str, usize); 5] = [
            ("position-ned", 28),
            ("mag-body", 41),
            ("heading-rad", 54),
            ("motor-rpm", 59),
            ("battery-v", 76),
        ];
        assert_eq!(
            table.map(|(_, o)| o),
            SENSOR_PRESENCE,
            "the decoder's presence list"
        );
        let mut checked = 0;
        for (name, at) in table {
            // accepted: 0 reads as absent, 1 as present
            for ok in [0u8, 1] {
                let mut f = base;
                f[HEADER_LEN + at] = ok;
                let (_, p) = decode_frame(&f).unwrap_or_else(|e| panic!("{name}={ok}: {e}"));
                let d = decode_sensor_frame(p).unwrap();
                let present = match at {
                    28 => d.position_ned.is_some(),
                    41 => d.mag_body.is_some(),
                    54 => d.heading_rad.is_some(),
                    59 => d.motor_rpm.is_some(),
                    _ => d.battery_v.is_some(),
                };
                assert_eq!(present, ok == 1, "{name} presence {ok}");
            }
            // refused: anything else, from either reader
            for bad in [2u8, 0x80, 0xff] {
                let mut f = base;
                f[HEADER_LEN + at] = bad;
                let want = Err(WireError::BadBool {
                    msg: MsgType::Sensor,
                    offset: at,
                    got: bad,
                });
                assert_eq!(decode_frame(&f).map(|_| ()), want, "{name} = {bad:#x}");
                assert_eq!(
                    decode_sensor_frame(&f[HEADER_LEN..]).map(|_| ()),
                    want,
                    "{name} = {bad:#x}, payload reader"
                );
                checked += 1;
            }
        }
        assert_eq!(checked, 5 * 3, "every presence byte x every bad value");
    }

    // ── the binding document's offset tables, checked against the encoder ──
    //
    // The tables in docs/TRANSPORT-BINDING.md are what a host implements from,
    // so they are tested, not trusted. Each table sits between
    // `<!-- wire-layout: NAME LEN -->` and `<!-- /wire-layout -->`, one row per
    // scalar: `off size type field`. For every frame type the test encodes ONE
    // frame in which every field holds a DIFFERENT value, then requires, for
    // every row, that the bytes at the documented offset are that field's
    // value — and that the rows are contiguous, cover the payload exactly, and
    // name every field exactly once. The expected values come from the struct
    // fields below, never from the encoder's offsets, so a doc row that is
    // wrong, missing, duplicated or shifted fails.

    const DOC: &str = include_str!("../../../docs/TRANSPORT-BINDING.md");

    struct DocRow {
        off: usize,
        size: usize,
        ty: &'static str,
        field: &'static str,
    }

    fn doc_table(name: &str) -> (usize, [Option<DocRow>; 32]) {
        let open = DOC
            .lines()
            .position(|l| {
                l.trim()
                    .strip_prefix("<!-- wire-layout: ")
                    .and_then(|r| r.split_whitespace().next())
                    == Some(name)
            })
            .unwrap_or_else(|| panic!("doc has no wire-layout table for {name}"));
        let header: &'static str = DOC.lines().nth(open).unwrap();
        let len: usize = header
            .trim()
            .trim_start_matches("<!-- wire-layout: ")
            .split_whitespace()
            .nth(1)
            .and_then(|t| t.parse().ok())
            .unwrap_or_else(|| panic!("{name}: table header must state the payload length"));
        let mut rows: [Option<DocRow>; 32] = core::array::from_fn(|_| None);
        let mut n = 0;
        for l in DOC.lines().skip(open + 1) {
            if l.trim() == "<!-- /wire-layout -->" {
                return (len, rows);
            }
            let t: [&str; 4] = {
                let mut it = l.split_whitespace();
                let a = it.next().unwrap_or("");
                if !a.chars().all(|c| c.is_ascii_digit()) || a.is_empty() {
                    continue;
                }
                [
                    a,
                    it.next().unwrap(),
                    it.next().unwrap(),
                    it.next().unwrap(),
                ]
            };
            rows[n] = Some(DocRow {
                off: t[0].parse().unwrap(),
                size: t[1].parse().unwrap(),
                ty: t[2],
                field: t[3],
            });
            n += 1;
        }
        panic!("{name}: unterminated wire-layout table");
    }

    /// A field's expected bytes: its value in the frame below, by NAME.
    enum Want {
        F32(f32),
        S32(i32),
        U8(u8),
        U32(u32),
    }

    fn check_table(name: &str, frame: &[u8], fields: &[(&str, Want)]) {
        let (len, rows) = doc_table(name);
        let payload = &frame[HEADER_LEN..];
        let rows: &[Option<DocRow>] = &rows;
        let mut next = 0;
        let mut seen = 0;
        for row in rows.iter().flatten() {
            assert_eq!(
                row.off, next,
                "{name}.{}: rows must be contiguous",
                row.field
            );
            next = row.off + row.size;
            let (_, want) = fields
                .iter()
                .find(|(f, _)| *f == row.field)
                .unwrap_or_else(|| panic!("{name}: doc row `{}` is not a field", row.field));
            let got = &payload[row.off..row.off + row.size];
            match (want, row.ty) {
                (Want::F32(v), "f32") => assert_eq!(got, v.to_le_bytes(), "{name}.{}", row.field),
                (Want::S32(v), "s32") => assert_eq!(got, v.to_le_bytes(), "{name}.{}", row.field),
                (Want::U8(v), "u8") => assert_eq!(got, [*v], "{name}.{}", row.field),
                (Want::U32(v), "u32") => assert_eq!(got, v.to_le_bytes(), "{name}.{}", row.field),
                (_, ty) => panic!("{name}.{}: doc says {ty}, which is not its type", row.field),
            }
            seen += 1;
        }
        assert_eq!(next, len, "{name}: rows must cover the stated length");
        assert_eq!(
            len,
            payload.len(),
            "{name}: stated length vs the encoder's payload"
        );
        assert_eq!(
            seen,
            fields.len(),
            "{name}: every field documented exactly once"
        );
    }

    #[test]
    fn the_binding_document_offset_tables_match_the_encoder() {
        // sensor: every field a distinct value, every presence byte set
        let f = WireSensorFrame {
            imu: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            dt_s: 7.0,
            position_ned: Some([8.0, 9.0, 10.0]),
            mag_body: Some([11.0, 12.0, 13.0]),
            heading_rad: Some(14.0),
            motor_rpm: Some([15, 16, 17, 18]),
            battery_v: Some(19.0),
        };
        let mut b = [0u8; HEADER_LEN + SENSOR_PAYLOAD_LEN];
        encode_sensor_frame(&mut b, 1, &f);
        use Want::*;
        check_table(
            "sensor",
            &b,
            &[
                ("imu.ax", F32(1.0)),
                ("imu.ay", F32(2.0)),
                ("imu.az", F32(3.0)),
                ("imu.gx", F32(4.0)),
                ("imu.gy", F32(5.0)),
                ("imu.gz", F32(6.0)),
                ("dt-s", F32(7.0)),
                ("position-ned.present", U8(1)),
                ("position-ned.x", F32(8.0)),
                ("position-ned.y", F32(9.0)),
                ("position-ned.z", F32(10.0)),
                ("mag-body.present", U8(1)),
                ("mag-body.x", F32(11.0)),
                ("mag-body.y", F32(12.0)),
                ("mag-body.z", F32(13.0)),
                ("heading-rad.present", U8(1)),
                ("heading-rad", F32(14.0)),
                ("motor-rpm.present", U8(1)),
                ("motor-rpm.m1", S32(15)),
                ("motor-rpm.m2", S32(16)),
                ("motor-rpm.m3", S32(17)),
                ("motor-rpm.m4", S32(18)),
                ("battery-v.present", U8(1)),
                ("battery-v", F32(19.0)),
            ],
        );

        let c = WireVehicleConfig::from_array([1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]);
        let mut b = [0u8; HEADER_LEN + CONFIG_PAYLOAD_LEN];
        encode_config(&mut b, 1, &c);
        check_table(
            "config",
            &b,
            &[
                ("hover-thrust", F32(1.0)),
                ("loop-rate-hz", F32(2.0)),
                ("pos-var", F32(3.0)),
                ("process-floor-vel", F32(4.0)),
                ("process-floor-pos", F32(5.0)),
                ("altitude-kp", F32(6.0)),
                ("altitude-kd", F32(7.0)),
                ("altitude-ki", F32(8.0)),
                ("position-ki", F32(9.0)),
            ],
        );

        let mut b = [0u8; HEADER_LEN + ACK_PAYLOAD_LEN];
        encode_ack(&mut b, 1, 0xdead_beef);
        check_table("ack", &b, &[("acked-seq", U32(0xdead_beef))]);

        let mut b = [0u8; HEADER_LEN + MOTORS_PAYLOAD_LEN];
        encode_motors(&mut b, 1, [1.0, 2.0, 3.0, 4.0]);
        check_table(
            "motors",
            &b,
            &[
                ("m1", F32(1.0)),
                ("m2", F32(2.0)),
                ("m3", F32(3.0)),
                ("m4", F32(4.0)),
            ],
        );

        let s = WireSupervisorConfig {
            home: [1.0, 2.0, 3.0],
            fence_radius: 4.0,
            cruise_alt: 5.0,
            low_batt_v: 6.0,
        };
        let mut b = [0u8; HEADER_LEN + SUPERVISOR_CONFIG_PAYLOAD_LEN];
        encode_supervisor_config(&mut b, 1, &s);
        check_table(
            "supervisor-config",
            &b,
            &[
                ("home.x", F32(1.0)),
                ("home.y", F32(2.0)),
                ("home.z", F32(3.0)),
                ("fence-radius", F32(4.0)),
                ("cruise-alt", F32(5.0)),
                ("low-batt-v", F32(6.0)),
            ],
        );

        let c = WireCalibration {
            gyro_bias: [1.0, 2.0, 3.0],
            accel_bias: [4.0, 5.0, 6.0],
            accel_scale: [7.0, 8.0, 9.0],
            mag_offset: [10.0, 11.0, 12.0],
            mag_scale: [13.0, 14.0, 15.0],
            mag_offdiag: [16.0, 17.0, 18.0],
        };
        let mut b = [0u8; HEADER_LEN + CALIBRATION_PAYLOAD_LEN];
        encode_calibration(&mut b, 1, &c);
        check_table(
            "calibration",
            &b,
            &[
                ("gyro-bias.x", F32(1.0)),
                ("gyro-bias.y", F32(2.0)),
                ("gyro-bias.z", F32(3.0)),
                ("accel-bias.x", F32(4.0)),
                ("accel-bias.y", F32(5.0)),
                ("accel-bias.z", F32(6.0)),
                ("accel-scale.x", F32(7.0)),
                ("accel-scale.y", F32(8.0)),
                ("accel-scale.z", F32(9.0)),
                ("mag-offset.x", F32(10.0)),
                ("mag-offset.y", F32(11.0)),
                ("mag-offset.z", F32(12.0)),
                ("mag-scale.x", F32(13.0)),
                ("mag-scale.y", F32(14.0)),
                ("mag-scale.z", F32(15.0)),
                ("mag-offdiag.xy", F32(16.0)),
                ("mag-offdiag.xz", F32(17.0)),
                ("mag-offdiag.yz", F32(18.0)),
            ],
        );

        // command: distinct byte values where the type allows it
        let mut b = [0u8; HEADER_LEN + COMMAND_PAYLOAD_LEN];
        encode_command(
            &mut b,
            1,
            &WireCommand {
                cmd: WirePilotCommand::RequestDisarm,
                level: true,
                throttle_low: false,
            },
        );
        check_table(
            "command",
            &b,
            &[("cmd", U8(6)), ("level", U8(1)), ("throttle-low", U8(0))],
        );

        // and the header, against the header table
        let mut b = [0u8; HEADER_LEN + ACK_PAYLOAD_LEN];
        encode_ack(&mut b, 0x0403_0201, 0);
        let mut hdr = [0u8; HEADER_LEN + HEADER_LEN];
        hdr[HEADER_LEN..].copy_from_slice(&b[..HEADER_LEN]);
        check_table(
            "header",
            &hdr,
            &[
                ("magic0", U8(0x1c)),
                ("magic1", U8(0xfa)),
                ("version", U8(VERSION)),
                ("type", U8(MsgType::Ack as u8)),
                ("seq", U32(0x0403_0201)),
            ],
        );
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
