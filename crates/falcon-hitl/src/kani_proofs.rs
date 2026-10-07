//! Kani proofs for the TRANSPORT-P01 wire binding's decoder.
//!
//! WHY THIS FILE EXISTS. `wire::decode_frame` and the four `decode_*` payload
//! readers parse UNTRUSTED BYTES off a socket or a UART, and until now
//! `falcon-hitl` carried ZERO `kani::proof` harnesses and did not appear in
//! `.github/workflows/kani.yml` at all. SWREQ-FALCON-TRANSPORT-P01 names that
//! as a promotion blocker in its own text, not a caveat:
//!
//!   "NO PROOF ON THE DECODER ... `wire::decode_frame` parses UNTRUSTED BYTES
//!    and has no Kani harness, and `falcon-hitl` is not in the kani.yml matrix
//!    at all ... Named now so it does not become the next orphaned leaf — the
//!    pattern this release line keeps finding."
//!
//! A plain sibling, declared `#[cfg(kani)]` from `lib.rs`: harnesses never live
//! in a module a verus strip could regenerate, and `cargo build` / `cargo test`
//! cannot see a break in `cfg(kani)` code, so these are run with `cargo kani`.
//!
//! WHAT THESE PROVE, AND WHAT THEY DO NOT. They establish ROBUSTNESS — the
//! decoder cannot panic on any byte string and cannot misreport a length. They
//! do NOT establish CONFORMANCE to `docs/TRANSPORT-BINDING.md`, because a
//! self-referential proof cannot: that needs vectors this repo did not produce,
//! which `wire.rs`'s own test module already says it does not have. Treating a
//! green proof as conformance is the error that let a bit-order defect through
//! 40 tests and 9 Kani harnesses on the DroneCAN decoder.
//!
//! HARNESS SPLIT IS FOR TRACTABILITY. One harness takes a SYMBOLIC LENGTH over
//! a small buffer, which is what reaches every rejection branch. The dispatch
//! harnesses take EXACT-LENGTH buffers per message type, so Kani never explores
//! the rejection paths and a 19-float sensor decode in the same state space —
//! that combination is the SAT heavy-tail shape that stalled a harness at ~31%
//! on CI once already.

use crate::wire::{
    decode_ack, decode_config, decode_frame, decode_motors, decode_sensor_frame, MsgType,
    ACK_PAYLOAD_LEN, CONFIG_PAYLOAD_LEN, HEADER_LEN, MAGIC, MOTORS_PAYLOAD_LEN,
    SENSOR_PAYLOAD_LEN, VERSION,
};

/// Every structural promise `decode_frame` makes, over an arbitrary byte string
/// of arbitrary length.
///
/// The buffer is sized for the two SHORT message types (`Ack` = 4,
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
    // One cover per rejection reason. If any of these is UNSATISFIABLE the
    // corresponding guard is dead code and the error variant is a lie.
    kani::cover!(len < HEADER_LEN && r.is_err(), "a short frame is rejected");
    kani::cover!(
        len >= HEADER_LEN && (slice[0] != MAGIC[0] || slice[1] != MAGIC[1]) && r.is_err(),
        "bad magic is rejected"
    );
    kani::cover!(
        len >= HEADER_LEN && slice[0] == MAGIC[0] && slice[1] == MAGIC[1] && slice[2] != VERSION
            && r.is_err(),
        "a wrong version is rejected"
    );
    kani::cover!(
        len >= HEADER_LEN && slice[0] == MAGIC[0] && slice[1] == MAGIC[1] && slice[2] == VERSION
            && slice[3] > 3
            && r.is_err(),
        "an unknown message type is rejected"
    );
}

/// The COMPOSED surface for `Motors`: header decode, then the payload reader
/// that `decode_frame` hands its slice to. That chain is what actually turns
/// untrusted bytes into a motor command.
///
/// Exact length, so this harness proves the dispatch rather than re-exploring
/// the rejection ladder already covered above.
#[kani::proof]
#[kani::unwind(8)]
fn a_well_formed_motors_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + MOTORS_PAYLOAD_LEN;
    let mut buf: [u8; N] = kani::any();
    buf[0] = MAGIC[0];
    buf[1] = MAGIC[1];
    buf[2] = VERSION;
    buf[3] = MsgType::Motors as u8;

    let (h, payload) = decode_frame(&buf).expect("a well-formed Motors frame must decode");
    assert!(h.msg == MsgType::Motors);
    // The length already matched in decode_frame, so the payload reader cannot
    // disagree with it. A `BadLength` here would mean the two disagree about
    // the same constant.
    let m = decode_motors(payload).expect("the payload reader must accept decode_frame's slice");

    // A FINDING, PROVEN RATHER THAN ASSERTED: the decoder accepts NON-FINITE
    // motor commands. `get_f32` is a bare `f32::from_le_bytes`, and there is no
    // `is_finite` check anywhere in `wire.rs`. These covers are SATISFIABLE,
    // which is the proof that arbitrary wire bytes become NaN and infinity in a
    // motor command. `docs/TRANSPORT-BINDING.md` specifies non-finite handling
    // for `dt-s` ("falls back to 0.001 s") and says NOTHING about these.
    //
    // Deliberately recorded as a cover and NOT fixed here: rejecting non-finite
    // payload values changes what the published binding ACCEPTS, which is a
    // seam decision rather than a proof fix. Raised on the requirement instead.
    kani::cover!(m[0].is_nan(), "a NaN motor command is accepted");
    kani::cover!(m[0].is_infinite(), "an infinite motor command is accepted");
    // What IS proven unconditionally: four commands out, no panic, no overflow.
    assert!(m.len() == 4);
}

/// Same composed proof for `Ack`, whose payload is the sequence being
/// acknowledged — the field the configuration handshake turns on.
#[kani::proof]
#[kani::unwind(8)]
fn a_well_formed_ack_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + ACK_PAYLOAD_LEN;
    let mut buf: [u8; N] = kani::any();
    buf[0] = MAGIC[0];
    buf[1] = MAGIC[1];
    buf[2] = VERSION;
    buf[3] = MsgType::Ack as u8;

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
    let mut buf: [u8; N] = kani::any();
    buf[0] = MAGIC[0];
    buf[1] = MAGIC[1];
    buf[2] = VERSION;
    buf[3] = MsgType::Config as u8;

    let (h, payload) = decode_frame(&buf).expect("a well-formed Config frame must decode");
    assert!(h.msg == MsgType::Config);
    let _ = decode_config(payload).expect("the payload reader must accept decode_frame's slice");
}

/// `Sensor` — 76 bytes, the largest payload and the one that feeds the
/// estimator. Its own harness because it is the most expensive to explore.
#[kani::proof]
#[kani::unwind(32)]
fn a_well_formed_sensor_frame_dispatches_without_panicking() {
    const N: usize = HEADER_LEN + SENSOR_PAYLOAD_LEN;
    let mut buf: [u8; N] = kani::any();
    buf[0] = MAGIC[0];
    buf[1] = MAGIC[1];
    buf[2] = VERSION;
    buf[3] = MsgType::Sensor as u8;

    let (h, payload) = decode_frame(&buf).expect("a well-formed Sensor frame must decode");
    assert!(h.msg == MsgType::Sensor);
    let _ =
        decode_sensor_frame(payload).expect("the payload reader must accept decode_frame's slice");
}
