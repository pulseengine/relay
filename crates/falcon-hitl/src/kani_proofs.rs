//! Kani proofs for the TRANSPORT-P01 wire binding's decoder (binding v2).
//!
//! WHY THIS FILE EXISTS. `wire::decode_frame` and the `decode_*` payload
//! readers parse UNTRUSTED BYTES off a socket or a UART. SWREQ-FALCON-TRANSPORT-P01
//! named a decoder without a proof as a promotion blocker, and these harnesses
//! are what the `falcon-hitl` leg of `.github/workflows/kani.yml` runs.
//!
//! A plain sibling, declared `#[cfg(kani)]` from `lib.rs`: harnesses never live
//! in a module a verus strip could regenerate, and `cargo build` / `cargo test`
//! cannot see a break in `cfg(kani)` code, so these are run with `cargo kani`.
//!
//! WHAT CHANGED AT v2 (#527). The v1 dispatch harnesses PROVED THAT NaN WAS
//! ACCEPTED: they `expect`ed every well-formed frame to decode, over symbolic
//! payloads that include every NaN and infinity bit pattern, and two `cover!`s
//! demonstrated a NaN and an infinite motor command coming out of the decoder.
//! The maintainer's decision was to reject non-finite values at the decoder, so
//! those harnesses now prove the NEW contract, as a biconditional per frame
//! type:
//!
//!   decode_frame(frame) is Ok  <=>  every f32 slot of the payload is finite
//!                                   (and, for Command, every byte in range;
//!                                   for Sensor, every presence byte 0 or 1)
//!   ...and when it is not Ok, the error is EXACTLY `NonFinite` (or
//!   `BadCommand` / `BadBool`), naming a slot that really is non-finite.
//!
//! THE SLOT TABLES BELOW ARE THE HARNESSES' OWN, written from the WIT field
//! order, slot bits are assembled byte by byte, and finiteness is judged by
//! shifting out the exponent field — while the decoder uses its own slot
//! table, `from_le_bytes` and a mask-and-compare. So a
//! decoder that forgot a slot, checked a non-float byte (an `s32` rpm whose
//! bytes spell -inf is a valid rpm), or mis-classified a NaN payload fails
//! here rather than agreeing with a copy of itself.
//!
//! WHAT THESE PROVE, AND WHAT THEY DO NOT. They establish ROBUSTNESS and the
//! VALUE CONTRACT. They do NOT establish CONFORMANCE to
//! `docs/TRANSPORT-BINDING.md`; that is what the hand-computed byte vectors and
//! the doc-table test in `wire.rs` are for. Treating a green proof as
//! conformance is the error that let a bit-order defect through 40 tests and 9
//! Kani harnesses on the DroneCAN decoder.
//!
//! HARNESS SPLIT IS FOR TRACTABILITY. One harness takes a SYMBOLIC LENGTH over
//! a small buffer, which is what reaches every rejection branch. The dispatch
//! harnesses take EXACT-LENGTH buffers per message type, so Kani never explores
//! the rejection ladder and a 15-float sensor decode in the same state space —
//! that combination is the SAT heavy-tail shape that stalled a harness at ~31%
//! on CI once already.

use crate::wire::{
    ACK_PAYLOAD_LEN, CALIBRATION_PAYLOAD_LEN, COMMAND_PAYLOAD_LEN, CONFIG_PAYLOAD_LEN, HEADER_LEN,
    MAGIC, MOTORS_PAYLOAD_LEN, MsgType, SENSOR_PAYLOAD_LEN, SUPERVISOR_CONFIG_PAYLOAD_LEN, VERSION,
    WireError, decode_ack, decode_calibration, decode_command, decode_config, decode_frame,
    decode_motors, decode_sensor_frame, decode_supervisor_config,
};

/// The harnesses' OWN float-slot tables (payload offsets), from the WIT field
/// order. Deliberately not `MsgType::f32_offsets`.
const CONFIG_SLOTS: [usize; 9] = [0, 4, 8, 12, 16, 20, 24, 28, 32];
const MOTORS_SLOTS: [usize; 4] = [0, 4, 8, 12];
const SUPCFG_SLOTS: [usize; 6] = [0, 4, 8, 12, 16, 20];
/// Calibration: five vec3s, then `mag-offdiag` appended at 60/64/68.
const CAL_SLOTS: [usize; 18] = [
    0, 4, 8, 12, 16, 20, 24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68,
];
/// imu 6, dt-s, position-ned 3 (after its presence byte at 28), mag-body 3
/// (presence 41), heading-rad (presence 54), battery-v (presence 76). The four
/// motor-rpm s32s at 60..76 are NOT float slots.
const SENSOR_SLOTS: [usize; 15] = [0, 4, 8, 12, 16, 20, 24, 29, 33, 37, 42, 46, 50, 55, 77];
/// The harness's OWN list of the sensor PRESENCE bytes, from the
/// layout table: position-ned, mag-body, heading-rad, motor-rpm, battery-v.
/// Deliberately not `wire::SENSOR_PRESENCE`.
const SENSOR_PRESENCE_BYTES: [usize; 5] = [28, 41, 54, 59, 76];

/// The FIRST presence byte, in field order, that is neither 0 nor 1.
fn first_bad_presence(payload: &[u8]) -> Option<usize> {
    let mut first = None;
    for &at in &SENSOR_PRESENCE_BYTES {
        if first.is_none() && payload[at] > 1 {
            first = Some(at);
        }
    }
    first
}

/// The slot's bits, assembled BYTE BY BYTE (little-endian), not via
/// `from_le_bytes` — a second, independent reading of the wire order.
fn bits_at(payload: &[u8], at: usize) -> u32 {
    (payload[at] as u32)
        | (payload[at + 1] as u32) << 8
        | (payload[at + 2] as u32) << 16
        | (payload[at + 3] as u32) << 24
}

/// IEEE-754 binary32 finiteness, judged from the EXPONENT FIELD by shift: the
/// biased exponent (bits 23..31) is 255 exactly for NaN and ±infinity. A
/// different formulation from the decoder's mask-and-compare, and
/// integer-only, so the harness's oracle involves no floating-point
/// classification of its own.
fn finite_bits(bits: u32) -> bool {
    (bits >> 23) & 0xff != 0xff
}

fn all_finite(payload: &[u8], slots: &[usize]) -> bool {
    first_non_finite(payload, slots).is_none()
}

/// The FIRST slot, in WIT field order, whose value is non-finite. Computed
/// over the concrete slot table, so the harness never indexes the payload by a
/// symbolic offset (that indexing is the expensive shape for CBMC).
fn first_non_finite(payload: &[u8], slots: &[usize]) -> Option<usize> {
    let mut first = None;
    for &at in slots {
        if first.is_none() && !finite_bits(bits_at(payload, at)) {
            first = Some(at);
        }
    }
    first
}

/// The shared biconditional for an exact-length frame of a float-only type:
/// Ok iff every slot is finite; otherwise exactly `NonFinite`, naming the
/// FIRST non-finite slot in field order — a float slot, and one that really is
/// non-finite. Returns whether it decoded.
fn value_contract(buf: &[u8], msg: MsgType, slots: &[usize]) -> bool {
    let payload = &buf[HEADER_LEN..];
    let first_bad = first_non_finite(payload, slots);
    match decode_frame(buf) {
        Ok((h, p)) => {
            assert!(
                first_bad.is_none(),
                "a frame with a non-finite slot was ACCEPTED"
            );
            assert!(h.msg == msg);
            assert!(p.len() == msg.payload_len());
            true
        }
        Err(WireError::NonFinite { msg: m, offset }) => {
            assert!(m == msg);
            assert!(
                first_bad == Some(offset),
                "NonFinite must name the first non-finite float slot"
            );
            false
        }
        Err(_) => {
            // magic, version, type and length are all fixed well-formed here,
            // so any other rejection is a decoder that lies about its reason.
            assert!(
                false,
                "a well-formed frame was rejected for a non-value reason"
            );
            false
        }
    }
}

fn well_formed<const N: usize>(msg: MsgType) -> [u8; N] {
    let mut buf: [u8; N] = kani::any();
    buf[0] = MAGIC[0];
    buf[1] = MAGIC[1];
    buf[2] = VERSION;
    buf[3] = msg as u8;
    buf
}

/// Every structural promise `decode_frame` makes, over an arbitrary byte string
/// of arbitrary length.
///
/// The buffer is sized for the SHORT message types (`Command` = 3, `Ack` = 4,
/// `Motors` = 16) plus one byte, so a symbolic length can reach `TooShort`, an
/// exact match, and an over-long payload — i.e. every arm of the rejection
/// ladder. The long types get their own exact-length harnesses below.
#[kani::proof]
#[kani::unwind(32)]
fn decode_frame_is_total_and_never_lies_about_length() {
    const N: usize = HEADER_LEN + MOTORS_PAYLOAD_LEN + 1;
    let buf: [u8; N] = kani::any();
    let len: usize = kani::any();
    kani::assume(len <= N);
    let slice = &buf[..len];

    match decode_frame(slice) {
        Ok((h, payload)) => {
            // (1) A successful decode pins the WHOLE frame length. This is the
            //     property a host relies on to find the next frame in a stream.
            assert!(slice.len() == HEADER_LEN + h.msg.payload_len());
            // (2) The payload handed back is exactly the tail — the decoder
            //     cannot hand a caller a window that runs past the frame.
            assert!(payload.len() == h.msg.payload_len());
            assert!(payload.as_ptr() == unsafe { slice.as_ptr().add(HEADER_LEN) });
            // (3) The version is the accepted one, never whatever was on the wire.
            assert!(h.version == VERSION);
            // (4) The sequence is the little-endian u32 at bytes 4..8, so a host
            //     can tell a dropped tick from a late one (the requirement's own
            //     falsification condition).
            let mut s = [0u8; 4];
            s.copy_from_slice(&slice[4..8]);
            assert!(h.seq == u32::from_le_bytes(s));
            // (5) Magic matched. Stated as a proof rather than trusted, because
            //     a decoder that resynchronises on bad magic is how a host ends
            //     up flying on misread bytes.
            assert!(slice[0] == MAGIC[0] && slice[1] == MAGIC[1]);
            // (6) v2: an accepted Motors frame carries no non-finite command
            //     (#527), and an accepted Command frame is in range.
            if h.msg == MsgType::Motors {
                assert!(all_finite(payload, &MOTORS_SLOTS));
            }
            if h.msg == MsgType::Command {
                assert!(payload[0] <= 7 && payload[1] <= 1 && payload[2] <= 1);
            }
        }
        Err(_) => {
            // A rejection is always allowed; what matters is that SOME frames
            // are accepted, which the covers below force Kani to demonstrate.
        }
    }
}

/// Anti-vacuity: every accept path and every reject path is REACHABLE.
///
/// Without this a decoder that rejected unconditionally would satisfy the
/// harness above. Each `cover!` must be SATISFIABLE for the proof run to be
/// meaningful — the Kani equivalent of exercising a bar with a deliberately
/// bad input rather than assuming it can fire.
#[kani::proof]
#[kani::unwind(32)]
fn every_branch_of_the_rejection_ladder_is_reachable() {
    const N: usize = HEADER_LEN + MOTORS_PAYLOAD_LEN + 1;
    let buf: [u8; N] = kani::any();
    let len: usize = kani::any();
    kani::assume(len <= N);
    let slice = &buf[..len];
    let r = decode_frame(slice);

    kani::cover!(r.is_ok(), "some byte string decodes");
    kani::cover!(r.is_err(), "some byte string is rejected");
    kani::cover!(
        matches!(r, Ok((h, _)) if h.msg == MsgType::Ack),
        "an Ack frame decodes"
    );
    kani::cover!(
        matches!(r, Ok((h, _)) if h.msg == MsgType::Motors),
        "a Motors frame decodes"
    );
    kani::cover!(
        matches!(r, Ok((h, _)) if h.msg == MsgType::Command),
        "a Command frame decodes"
    );
    // One cover per rejection reason. If any of these is UNSATISFIABLE the
    // corresponding guard is dead code and the error variant is a lie.
    kani::cover!(len < HEADER_LEN && r.is_err(), "a short frame is rejected");
    kani::cover!(
        len >= HEADER_LEN && (slice[0] != MAGIC[0] || slice[1] != MAGIC[1]) && r.is_err(),
        "bad magic is rejected"
    );
    kani::cover!(
        len >= HEADER_LEN
            && slice[0] == MAGIC[0]
            && slice[1] == MAGIC[1]
            && slice[2] != VERSION
            && r.is_err(),
        "a wrong version is rejected"
    );
    kani::cover!(
        len >= HEADER_LEN
            && slice[0] == MAGIC[0]
            && slice[1] == MAGIC[1]
            && slice[2] == VERSION
            && slice[3] > 6
            && r.is_err(),
        "an unknown message type is rejected"
    );
    kani::cover!(
        matches!(r, Err(WireError::BadLength { .. })),
        "a wrong payload length is rejected"
    );
    kani::cover!(
        matches!(r, Err(WireError::NonFinite { .. })),
        "a non-finite float is rejected"
    );
}

/// The COMPOSED surface for `Motors`: header decode, then the payload reader
/// that `decode_frame` hands its slice to. That chain is what actually turns
/// untrusted bytes into a motor command.
///
/// v1 of this harness proved a NaN and an infinite motor command were
/// ACCEPTED. It now proves the opposite, as a biconditional, and that an
/// accepted command is bit-for-bit what was on the wire.
#[kani::proof]
#[kani::unwind(8)]
fn a_well_formed_motors_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + MOTORS_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::Motors);
    let ok = value_contract(&buf, MsgType::Motors, &MOTORS_SLOTS);
    if ok {
        let m = decode_motors(&buf[HEADER_LEN..])
            .expect("the payload reader must accept what decode_frame accepted");
        for i in 0..4 {
            assert!(finite_bits(m[i].to_bits()));
            assert!(m[i].to_bits() == bits_at(&buf[HEADER_LEN..], i * 4));
        }
    } else {
        // The reader agrees with decode_frame on a bare payload.
        assert!(decode_motors(&buf[HEADER_LEN..]).is_err());
    }
    kani::cover!(ok, "a finite motor command is accepted");
    kani::cover!(!ok, "a NaN or infinite motor command is rejected");
}

/// Same composed proof for `Ack`, whose payload is the sequence being
/// acknowledged — the field the configuration handshake turns on. It has no
/// float slot, so every well-formed Ack decodes.
#[kani::proof]
#[kani::unwind(8)]
fn a_well_formed_ack_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + ACK_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::Ack);
    let (h, payload) = decode_frame(&buf).expect("a well-formed Ack frame must decode");
    assert!(h.msg == MsgType::Ack);
    let acked = decode_ack(payload).expect("the payload reader must accept decode_frame's slice");
    let mut b = [0u8; 4];
    b.copy_from_slice(&buf[HEADER_LEN..HEADER_LEN + 4]);
    assert!(acked == u32::from_le_bytes(b));
}

/// `Config` — 36 bytes, the handshake the component resets its core on.
#[kani::proof]
#[kani::unwind(16)]
fn a_well_formed_config_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + CONFIG_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::Config);
    let ok = value_contract(&buf, MsgType::Config, &CONFIG_SLOTS);
    assert!(decode_config(&buf[HEADER_LEN..]).is_ok() == ok);
    kani::cover!(ok, "a finite config is accepted");
    kani::cover!(!ok, "a config with a non-finite field is rejected");
}

/// `Sensor` — 81 bytes at v2, the largest payload and the one that feeds the
/// estimator. Its own harness because it is the most expensive to explore.
///
/// Its own CONTRACT too, because the sensor payload has a second value rule
/// (provenance: jess#167): every presence byte is strictly 0 or 1. The verdict is a
/// three-way biconditional with a pinned precedence —
///
///   first non-finite float slot      -> exactly `NonFinite` at that slot
///   else first presence byte > 1     -> exactly `BadBool` at that byte
///   else                             -> `Ok`
///
/// and on `Ok`, EVERY optional field (all five, not just battery) is present
/// exactly when its presence byte is 1 — never inferred from the value — with
/// a present value bit for bit the wire value. That last clause carries #572
/// (a present 0.0 V can never decode as ABSENT) and the presence rule (a presence byte
/// of 2 can never decode as present, because it never decodes at all).
#[kani::proof]
#[kani::unwind(32)]
fn a_well_formed_sensor_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + SENSOR_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::Sensor);
    let payload = &buf[HEADER_LEN..];
    let first_nan = first_non_finite(payload, &SENSOR_SLOTS);
    let first_presence = first_bad_presence(payload);
    let ok = match decode_frame(&buf) {
        Ok((h, p)) => {
            assert!(
                first_nan.is_none(),
                "a frame with a non-finite slot was ACCEPTED"
            );
            assert!(
                first_presence.is_none(),
                "a frame with a presence byte above 1 was ACCEPTED"
            );
            assert!(h.msg == MsgType::Sensor);
            assert!(p.len() == SENSOR_PAYLOAD_LEN);
            true
        }
        Err(WireError::NonFinite { msg, offset }) => {
            assert!(msg == MsgType::Sensor);
            assert!(
                first_nan == Some(offset),
                "NonFinite must name the first non-finite float slot"
            );
            false
        }
        Err(WireError::BadBool { msg, offset, got }) => {
            assert!(msg == MsgType::Sensor);
            assert!(first_nan.is_none(), "floats are judged first");
            assert!(
                first_presence == Some(offset),
                "BadBool must name the first presence byte above 1"
            );
            assert!(got == payload[offset] && got > 1);
            false
        }
        Err(_) => {
            assert!(
                false,
                "a well-formed sensor frame was rejected for another reason"
            );
            false
        }
    };
    if ok {
        let f = decode_sensor_frame(payload)
            .expect("the payload reader must accept what decode_frame accepted");
        assert!(f.position_ned.is_some() == (payload[28] == 1));
        assert!(f.mag_body.is_some() == (payload[41] == 1));
        assert!(f.heading_rad.is_some() == (payload[54] == 1));
        assert!(f.motor_rpm.is_some() == (payload[59] == 1));
        assert!(f.battery_v.is_some() == (payload[76] == 1));
        if let Some(v) = f.battery_v {
            assert!(v.to_bits() == bits_at(payload, 77));
        }
        if let Some(h) = f.heading_rad {
            assert!(h.to_bits() == bits_at(payload, 55));
        }
        kani::cover!(
            f.battery_v.map(f32::to_bits) == Some(0),
            "a present 0.0 V battery decodes as present"
        );
        kani::cover!(f.battery_v.is_none(), "an absent battery decodes as absent");
    } else {
        assert!(decode_sensor_frame(payload).is_err());
    }
    kani::cover!(ok, "a finite sensor frame is accepted");
    kani::cover!(
        first_nan.is_some() && !ok,
        "a sensor frame with a non-finite field is rejected"
    );
    kani::cover!(
        first_nan.is_none() && first_presence.is_some() && !ok,
        "a sensor frame with a presence byte above 1 is rejected"
    );
    kani::cover!(
        first_nan.is_none() && payload[28] <= 1 && payload[41] <= 1 && payload[54] == 2,
        "a heading presence byte of 2 (the case an integrator's probe measured) is reached"
    );
}

/// `SupervisorConfig` (v2) — 24 bytes, `pilot.configure-supervisor`.
#[kani::proof]
#[kani::unwind(8)]
fn a_well_formed_supervisor_config_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + SUPERVISOR_CONFIG_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::SupervisorConfig);
    let ok = value_contract(&buf, MsgType::SupervisorConfig, &SUPCFG_SLOTS);
    let payload = &buf[HEADER_LEN..];
    if ok {
        let c = decode_supervisor_config(payload)
            .expect("the payload reader must accept what decode_frame accepted");
        // WIT order: home.x home.y home.z fence-radius cruise-alt low-batt-v
        assert!(c.home[0].to_bits() == bits_at(payload, 0));
        assert!(c.fence_radius.to_bits() == bits_at(payload, 12));
        assert!(c.cruise_alt.to_bits() == bits_at(payload, 16));
        assert!(c.low_batt_v.to_bits() == bits_at(payload, 20));
    } else {
        assert!(decode_supervisor_config(payload).is_err());
    }
    kani::cover!(ok, "a finite supervisor-config is accepted");
    kani::cover!(
        !ok,
        "a supervisor-config with a non-finite field is rejected"
    );
}

/// `Calibration` (v2) — 72 bytes, `pilot.configure-calibration` (relay#611):
/// five vec3s, then `mag-offdiag` appended at 60/64/68 (provisional
/// `[xy, xz, yz]` order — must match #611's relay-calib ordering). Every one
/// of the 18 slots is NonFinite-checked; the unwind bound covers the 18-slot
/// loops.
#[kani::proof]
#[kani::unwind(20)]
fn a_well_formed_calibration_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + CALIBRATION_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::Calibration);
    let ok = value_contract(&buf, MsgType::Calibration, &CAL_SLOTS);
    let payload = &buf[HEADER_LEN..];
    if ok {
        let c = decode_calibration(payload)
            .expect("the payload reader must accept what decode_frame accepted");
        // WIT order: gyro-bias accel-bias accel-scale mag-offset mag-scale
        assert!(c.gyro_bias[0].to_bits() == bits_at(payload, 0));
        assert!(c.accel_bias[0].to_bits() == bits_at(payload, 12));
        assert!(c.accel_scale[0].to_bits() == bits_at(payload, 24));
        assert!(c.mag_offset[0].to_bits() == bits_at(payload, 36));
        assert!(c.mag_scale[2].to_bits() == bits_at(payload, 56));
        assert!(c.mag_offdiag[0].to_bits() == bits_at(payload, 60));
        assert!(c.mag_offdiag[1].to_bits() == bits_at(payload, 64));
        assert!(c.mag_offdiag[2].to_bits() == bits_at(payload, 68));
    } else {
        assert!(decode_calibration(payload).is_err());
    }
    kani::cover!(ok, "a finite calibration is accepted");
    kani::cover!(!ok, "a calibration with a non-finite field is rejected");
    kani::cover!(
        first_non_finite(&buf[HEADER_LEN..], &CAL_SLOTS) == Some(68),
        "a non-finite mag-offdiag.yz (the last appended slot) is rejected"
    );
}

/// `Command` (v2) — 3 bytes, `pilot.command`. Accepted EXACTLY when the
/// command byte is a `pilot-command` (0..=7) and both interlocks are 0 or 1;
/// a decoded interlock is `true` only for the byte 1. A stray byte must never
/// read as `level = true` — that is the fail-unsafe direction.
#[kani::proof]
#[kani::unwind(8)]
fn a_well_formed_command_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + COMMAND_PAYLOAD_LEN;
    let buf: [u8; N] = well_formed(MsgType::Command);
    let p = [buf[HEADER_LEN], buf[HEADER_LEN + 1], buf[HEADER_LEN + 2]];
    let in_range = p[0] <= 7 && p[1] <= 1 && p[2] <= 1;
    match decode_frame(&buf) {
        Ok((h, payload)) => {
            assert!(in_range);
            assert!(h.msg == MsgType::Command);
            let c = decode_command(payload).expect("accepted by decode_frame");
            assert!(c.cmd as u8 == p[0]);
            assert!(c.level == (p[1] == 1));
            assert!(c.throttle_low == (p[2] == 1));
        }
        Err(WireError::BadCommand { got }) => {
            assert!(p[0] > 7 && got == p[0]);
        }
        Err(WireError::BadBool { msg, offset, got }) => {
            assert!(msg == MsgType::Command);
            assert!((offset == 1 || offset == 2) && got > 1 && got == p[offset]);
        }
        Err(_) => assert!(
            false,
            "a well-formed Command was rejected for another reason"
        ),
    }
    kani::cover!(in_range, "an in-range command is accepted");
    kani::cover!(p[0] > 7, "an unknown pilot-command is rejected");
    kani::cover!(
        p[0] <= 7 && p[1] > 1,
        "a non-0/1 interlock byte is rejected"
    );
}
