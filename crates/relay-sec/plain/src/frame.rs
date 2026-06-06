//! F2/F3: per-channel authenticated frame wrap/verify.
//!
//! Frame layout: `SecurityHeader(11) ‖ payload ‖ Ascon-tag(16)`.
//!
//! A [`SecurityChannel`] holds ONE channel's security association — a distinct
//! SPI and key. Because the key is per channel, a frame recorded on one channel
//! (radio) cannot be replayed on another (cellular): the MAC check fails under
//! the wrong key. The replay window is keyed by the authenticated channel, never
//! the physical arrival link.
//!
//! `verify` is the **authenticate-before-parse** crux: parse the fixed header
//! (the only pre-auth step) → confirm the SPI is ours → recompute and
//! constant-time-compare the Ascon tag → and ONLY THEN judge freshness on the
//! AUTHENTICATED counter. The payload is returned only after all of that, so a
//! spoofed or replayed frame never reaches the downstream (post-auth) command
//! parser. F3's wiring is exactly this composition at the call site:
//! `let payload = channel.verify(frame)?; /* then validate_header(payload) */`.
//!
//! Verified: Kani SEC-K10 verify is total (no panic) on arbitrary bytes;
//! SEC-K11 / proptest a wrapped frame verifies and returns its payload;
//! tampered tag → BadMac, wrong SPI → UnknownSpi, replay → Replay.

use crate::ascon::{self, KEY_LEN, TAG_LEN};
use crate::header::{SecurityHeader, SEC_HEADER_LEN};
use crate::{ReplayVerdict, ReplayWindow};

/// Smallest possible frame: header + empty payload + tag.
pub const MIN_FRAME_LEN: usize = SEC_HEADER_LEN + TAG_LEN;

/// Why a frame was rejected. `verify` is total — it returns one of these for any
/// malformed or inauthentic input rather than panicking.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum VerifyError {
    /// Shorter than a header + tag.
    TooShort,
    /// The SPI does not match this channel's association.
    UnknownSpi,
    /// The Ascon tag did not authenticate (tampered / forged / wrong key).
    BadMac,
    /// Already-seen counter (within the replay window).
    Replay,
    /// Counter older than the replay window remembers.
    TooOld,
}

/// One channel's security association: a distinct SPI + key, a send counter for
/// `wrap`, and an anti-replay window for `verify`.
#[derive(Clone, Copy)]
pub struct SecurityChannel {
    spi: u16,
    channel_id: u8,
    key: [u8; KEY_LEN],
    send_counter: u64,
    window: ReplayWindow,
}

impl SecurityChannel {
    /// A fresh channel. The key comes from the cold-path session establishment
    /// (ephemeral X25519 + Ed25519/sigil); on reboot a new key + fresh counter
    /// are established (rekey-on-reboot), so the nonce is never reused.
    pub fn new(spi: u16, channel_id: u8, key: [u8; KEY_LEN]) -> Self {
        SecurityChannel {
            spi,
            channel_id,
            key,
            send_counter: 0,
            window: ReplayWindow::new(),
        }
    }

    /// Wrap `payload` into `out` as `header ‖ payload ‖ tag`, advancing the send
    /// counter. Returns the frame length, or `None` if `out` is too small to
    /// hold `SEC_HEADER_LEN + payload.len() + TAG_LEN` bytes.
    pub fn wrap(&mut self, payload: &[u8], out: &mut [u8]) -> Option<usize> {
        let body_end = SEC_HEADER_LEN + payload.len();
        let frame_len = body_end + TAG_LEN;
        if out.len() < frame_len {
            return None;
        }
        self.send_counter = self.send_counter.wrapping_add(1);
        let hdr = SecurityHeader {
            spi: self.spi,
            channel_id: self.channel_id,
            counter: self.send_counter,
        };
        hdr.write(out);
        out[SEC_HEADER_LEN..body_end].copy_from_slice(payload);
        // MAC-floor over (header ‖ payload).
        let tag = ascon::mac(&self.key, &hdr.nonce(), &out[..body_end]);
        out[body_end..frame_len].copy_from_slice(&tag);
        Some(frame_len)
    }

    /// Verify `frame` and, on success, return the authenticated payload slice.
    ///
    /// TOTAL: a typed [`VerifyError`] for any input, never a panic. Order is the
    /// safety crux — header parse, SPI, MAC, THEN replay on the authenticated
    /// counter.
    pub fn verify<'f>(&mut self, frame: &'f [u8]) -> Result<&'f [u8], VerifyError> {
        // 1. parse the fixed header (pre-auth, total)
        let hdr = SecurityHeader::parse(frame).ok_or(VerifyError::TooShort)?;
        // 2. need room for header + (>=0 payload) + tag
        if frame.len() < MIN_FRAME_LEN {
            return Err(VerifyError::TooShort);
        }
        // 3. per-channel key — a wrong SPI is not for us (defends cross-link too)
        if hdr.spi != self.spi {
            return Err(VerifyError::UnknownSpi);
        }
        let body_end = frame.len() - TAG_LEN; // >= SEC_HEADER_LEN by step 2
        let ad = &frame[..body_end]; // header ‖ payload
        // 4. recompute the MAC and compare in constant time (AUTHENTICATE)
        let expected = ascon::mac(&self.key, &hdr.nonce(), ad);
        let mut got = [0u8; TAG_LEN];
        got.copy_from_slice(&frame[body_end..]);
        if !ascon::ct_eq_tag(&expected, &got) {
            return Err(VerifyError::BadMac);
        }
        // 5. ONLY NOW judge freshness on the AUTHENTICATED counter
        match self.window.accept(hdr.counter) {
            ReplayVerdict::Fresh | ReplayVerdict::Reordered => Ok(&frame[SEC_HEADER_LEN..body_end]),
            ReplayVerdict::Replay => Err(VerifyError::Replay),
            ReplayVerdict::TooOld => Err(VerifyError::TooOld),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn channels() -> (SecurityChannel, SecurityChannel) {
        // sender and receiver share SPI + key for one channel.
        let key = [0x42u8; KEY_LEN];
        (
            SecurityChannel::new(0xABCD, 2, key),
            SecurityChannel::new(0xABCD, 2, key),
        )
    }

    #[test]
    fn wrap_then_verify_roundtrips() {
        let (mut tx, mut rx) = channels();
        let payload = b"ARM 0";
        let mut buf = [0u8; 64];
        let n = tx.wrap(payload, &mut buf).unwrap();
        assert_eq!(rx.verify(&buf[..n]), Ok(&payload[..]));
    }

    #[test]
    fn empty_payload_roundtrips() {
        let (mut tx, mut rx) = channels();
        let mut buf = [0u8; MIN_FRAME_LEN];
        let n = tx.wrap(&[], &mut buf).unwrap();
        assert_eq!(n, MIN_FRAME_LEN);
        assert_eq!(rx.verify(&buf[..n]), Ok(&[][..]));
    }

    #[test]
    fn tampered_tag_is_bad_mac() {
        let (mut tx, mut rx) = channels();
        let mut buf = [0u8; 64];
        let n = tx.wrap(b"LAND", &mut buf).unwrap();
        buf[n - 1] ^= 1; // flip a tag bit
        assert_eq!(rx.verify(&buf[..n]), Err(VerifyError::BadMac));
    }

    #[test]
    fn tampered_payload_is_bad_mac() {
        let (mut tx, mut rx) = channels();
        let mut buf = [0u8; 64];
        let n = tx.wrap(b"LAND", &mut buf).unwrap();
        buf[SEC_HEADER_LEN] ^= 0xFF; // flip a payload bit
        assert_eq!(rx.verify(&buf[..n]), Err(VerifyError::BadMac));
    }

    #[test]
    fn wrong_spi_is_rejected() {
        let key = [0x42u8; KEY_LEN];
        let mut tx = SecurityChannel::new(0x1111, 2, key);
        let mut rx = SecurityChannel::new(0x2222, 2, key); // different SPI
        let mut buf = [0u8; 64];
        let n = tx.wrap(b"x", &mut buf).unwrap();
        assert_eq!(rx.verify(&buf[..n]), Err(VerifyError::UnknownSpi));
    }

    #[test]
    fn replay_is_rejected() {
        let (mut tx, mut rx) = channels();
        let mut buf = [0u8; 64];
        let n = tx.wrap(b"RTL", &mut buf).unwrap();
        assert!(rx.verify(&buf[..n]).is_ok());
        // the very same frame, replayed, is rejected on the authenticated counter
        assert_eq!(rx.verify(&buf[..n]), Err(VerifyError::Replay));
    }

    #[test]
    fn garbage_never_panics() {
        let (_, mut rx) = channels();
        for len in 0..40usize {
            let mut buf = [0u8; 40];
            for (i, b) in buf.iter_mut().enumerate().take(len) {
                *b = (i as u8).wrapping_mul(37).wrapping_add(11);
            }
            let _ = rx.verify(&buf[..len]); // must not panic
        }
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// SEC-K11 (breadth): any payload wrapped by the sender verifies on the
        /// receiver and returns exactly that payload.
        #[test]
        fn wrap_verify_roundtrip(
            spi in any::<u16>(),
            ch in any::<u8>(),
            key in any::<[u8; KEY_LEN]>(),
            payload in proptest::collection::vec(any::<u8>(), 0..48),
        ) {
            let mut tx = SecurityChannel::new(spi, ch, key);
            let mut rx = SecurityChannel::new(spi, ch, key);
            let mut buf = [0u8; 11 + 48 + 16];
            let n = tx.wrap(&payload, &mut buf).unwrap();
            prop_assert_eq!(rx.verify(&buf[..n]), Ok(&payload[..]));
        }

        /// verify never panics for arbitrary bytes and a random association.
        #[test]
        fn verify_is_total(
            spi in any::<u16>(),
            key in any::<[u8; KEY_LEN]>(),
            frame in proptest::collection::vec(any::<u8>(), 0..64),
        ) {
            let mut rx = SecurityChannel::new(spi, 0, key);
            let _ = rx.verify(&frame); // must not panic
        }
    }
}
