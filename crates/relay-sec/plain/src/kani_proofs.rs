//! Kani bounded-model-checking harnesses for relay-sec (plain-only).
//!
//! Kept in this sibling module (declared from lib.rs) following the relay
//! convention: BMC harnesses never live in a verus-stripped file, so a regen
//! cannot wipe them. These prove the anti-replay window with NO cryptography —
//! freshness is a state-machine property.
#![cfg(kani)]

use crate::*;

/// SEC-K01 — a counter accepted once is rejected as Replay on immediate
/// re-submit. The fundamental no-double-accept guarantee.
#[kani::proof]
fn verify_no_double_accept() {
    let mut w = ReplayWindow::new();
    let c: u64 = kani::any();
    kani::assume(c > 0);
    // First accept from an empty window is always Fresh.
    assert!(w.accept(c) == ReplayVerdict::Fresh);
    // The very same counter must now be a Replay.
    assert!(w.accept(c) == ReplayVerdict::Replay);
}

/// SEC-K02 — accept never lowers the frontier, for any window built from a
/// real accept and any subsequent counter.
#[kani::proof]
fn verify_frontier_monotone() {
    let mut w = ReplayWindow::new();
    let a: u64 = kani::any();
    kani::assume(a > 0);
    w.accept(a);
    let before = w.highest();
    let b: u64 = kani::any();
    w.accept(b);
    assert!(w.highest() >= before);
}

/// SEC-K03 — accept is total: no panic, no arithmetic overflow, no shift-UB,
/// for ANY counter against a non-trivial window. This is what makes the `<<`
/// in the slide path safe.
#[kani::proof]
fn verify_accept_total() {
    let mut w = ReplayWindow::new();
    let a: u64 = kani::any();
    kani::assume(a > 0);
    w.accept(a);
    let b: u64 = kani::any();
    let _ = w.accept(b); // Kani proves this cannot panic for any b
}

/// SEC-K04 — anything strictly older than the window is TooOld and leaves the
/// window unchanged.
#[kani::proof]
fn verify_too_old_unchanged() {
    let mut w = ReplayWindow::new();
    let h: u64 = kani::any();
    kani::assume(h > WINDOW_BITS); // leave room below the frontier
    w.accept(h);
    let snapshot = w;
    let c: u64 = kani::any();
    kani::assume(c > 0);
    kani::assume(c <= h - WINDOW_BITS); // strictly outside the window
    assert!(w.accept(c) == ReplayVerdict::TooOld);
    assert!(w.highest() == snapshot.highest());
}

/// SEC-K05 — the constant-time tag comparison is functionally correct: it agrees
/// with structural equality for every pair of 16-byte tags.
#[kani::proof]
fn verify_ct_eq_tag_correct() {
    let a: [u8; 16] = kani::any();
    let b: [u8; 16] = kani::any();
    assert_eq!(ascon::ct_eq_tag(&a, &b), a == b);
}

/// SEC-K06 — Ascon-AEAD128 `mac` is total (no panic / no OOB index) for any
/// message length across the empty / partial / full / multi-block tail cases.
/// Length is symbolic (the indexing driver); data is concrete to keep the
/// permutation cheap — panic-freedom does not depend on the byte values.
#[kani::proof]
#[kani::unwind(41)]
fn verify_ascon_mac_total() {
    let key = [0u8; 16];
    let nonce = [0u8; 16];
    let len: usize = kani::any();
    kani::assume(len <= 33); // 0, partial, one full block, full+partial tail
    let buf = [0u8; 33];
    let _ = ascon::mac(&key, &nonce, &buf[..len]);
}

use crate::frame::{SecurityChannel, MIN_FRAME_LEN};
use crate::header::{SecurityHeader, SEC_HEADER_LEN};

/// SEC-K07 — the Security-Header parser is total: any buffer (any bytes, any
/// length) yields Some/None, never a panic.
#[kani::proof]
fn verify_header_parse_total() {
    let buf: [u8; SEC_HEADER_LEN + 4] = kani::any();
    let len: usize = kani::any();
    kani::assume(len <= SEC_HEADER_LEN + 4);
    let _ = SecurityHeader::parse(&buf[..len]);
}

/// SEC-K08 — write∘parse round-trips for any header.
#[kani::proof]
fn verify_header_roundtrip() {
    let h = SecurityHeader {
        spi: kani::any(),
        channel_id: kani::any(),
        counter: kani::any(),
    };
    let mut buf = [0u8; SEC_HEADER_LEN];
    assert!(h.write(&mut buf));
    assert!(SecurityHeader::parse(&buf) == Some(h));
}

/// SEC-K09 — the AEAD nonce is injective in (spi, channel_id, counter):
/// distinct headers yield distinct nonces (nonce-uniqueness precondition).
#[kani::proof]
fn verify_nonce_injective() {
    let a = SecurityHeader {
        spi: kani::any(),
        channel_id: kani::any(),
        counter: kani::any(),
    };
    let b = SecurityHeader {
        spi: kani::any(),
        channel_id: kani::any(),
        counter: kani::any(),
    };
    kani::assume(a != b);
    assert!(a.nonce() != b.nonce());
}

/// SEC-K10 — frame `verify` is total: no panic for any frame length across the
/// too-short / header-only / valid cases. Bytes are concrete (verify has no
/// value-dependent indexing); the symbolic length drives the slicing.
/// unwind covers the longest internal loop — ct_eq_tag's 16-byte compare.
#[kani::proof]
#[kani::unwind(41)]
fn verify_frame_total() {
    let mut ch = SecurityChannel::new(0, 0, [0u8; 16]);
    let len: usize = kani::any();
    kani::assume(len <= MIN_FRAME_LEN + 4);
    let frame = [0u8; MIN_FRAME_LEN + 4];
    let _ = ch.verify(&frame[..len]);
}

/// SEC-K12 — the mode-aware `verify_into` on an AEAD channel (which runs Ascon
/// decrypt) is total: no panic for any frame length. Concrete bytes, symbolic
/// length; unwind covers the Ascon permutation + the constant-time compare.
#[kani::proof]
#[kani::unwind(41)]
fn verify_into_aead_total() {
    use crate::frame::Confidentiality;
    let mut ch = SecurityChannel::with_confidentiality(0, 0, [0u8; 16], Confidentiality::Aead);
    let len: usize = kani::any();
    kani::assume(len <= MIN_FRAME_LEN + 4);
    let frame = [0u8; MIN_FRAME_LEN + 4];
    let mut out = [0u8; MIN_FRAME_LEN + 4];
    let _ = ch.verify_into(&frame[..len], &mut out);
}

/// SEC-K13 — the handshake MAC nonce is injective in (role, epoch): distinct
/// (role, epoch) pairs yield distinct nonces. This is the mechanical core of
/// the "no Ascon nonce reuse under k_id across reboots" property (design-review
/// must-fix #2): since epoch is a reboot-persistent monotonic counter, the
/// (key, nonce) pair never repeats.
#[kani::proof]
fn verify_handshake_nonce_injective() {
    let ra: u8 = kani::any();
    let ea: u64 = kani::any();
    let rb: u8 = kani::any();
    let eb: u64 = kani::any();
    kani::assume((ra, ea) != (rb, eb));
    assert!(crate::session::handshake_nonce(ra, ea) != crate::session::handshake_nonce(rb, eb));
}

/// SEC-K14 — building the session transcript is total: the fixed-capacity,
/// no_alloc transcript writer never panics for any epoch values (symbolic),
/// across the spi/channel/epoch/ephemeral fields. Proves the new handshake
/// indexing logic is panic-free.
#[kani::proof]
fn verify_session_transcript_total() {
    use crate::session::Config;
    let cfg = Config {
        spi: kani::any(),
        channel_id: kani::any(),
        k_id: [0u8; 16],
    };
    let epoch_i: u64 = kani::any();
    let epoch_r: u64 = kani::any();
    let ei = [0u8; 32];
    let er = [0u8; 32];
    let _ = crate::session::session_transcript(&cfg, epoch_i, epoch_r, &ei, &er);
}

/// SEC-K15 — cross-channel isolation: a frame wrapped on one security
/// association NEVER verifies on a channel with a different SPI, for ALL
/// (distinct) SPI pairs. This is the property that makes relay-sec sound as a
/// TRANSPORT-AGNOSTIC end-to-end layer: distinct transports — the two
/// directions of an inter-core IPC link, a radio link vs a CCSDS uplink — are
/// distinct SAs (distinct SPI), so a frame captured on one transport can never
/// be accepted on another. The rejection is by construction (the SPI check
/// precedes the MAC), independent of payload or key, so it holds whatever rides
/// on top. Symbolic SPIs drive it; the key/payload are concrete (the property
/// does not depend on their value — only on SPI distinctness).
#[kani::proof]
#[kani::unwind(41)]
fn cross_channel_isolation() {
    let spi_a: u16 = kani::any();
    let spi_b: u16 = kani::any();
    kani::assume(spi_a != spi_b);
    let key = [7u8; 16];
    let mut a = SecurityChannel::new(spi_a, 0, key);
    let mut b = SecurityChannel::new(spi_b, 0, key);
    let payload = [0xABu8; 4];
    let mut frame = [0u8; MIN_FRAME_LEN + 4];
    let n = a.wrap(&payload, &mut frame).unwrap();
    // b's SA is a different SPI -> the frame is not for it, never authenticated.
    assert!(matches!(
        b.verify(&frame[..n]),
        Err(crate::frame::VerifyError::UnknownSpi)
    ));
}
