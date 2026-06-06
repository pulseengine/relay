//! F1: the Security-Header parser — the minimal PRE-authentication TCB.
//!
//! "Authenticate before you parse" cannot cover this header itself: you must
//! read SPI ∥ channel_id ∥ counter to look up the security association before
//! the MAC can be verified. So this fixed-layout parser is the one piece of
//! attacker-controlled input handled BEFORE authentication — it must be TOTAL
//! (a typed result, never a panic) on arbitrary garbage. The payload parser
//! runs strictly post-auth.
//!
//! Layout (little-endian): SPI(2) ‖ channel_id(1) ‖ counter(8) = 11 bytes.
//!
//! Verified (Kani): SEC-K07 parse is total; SEC-K08 write∘parse round-trips;
//! SEC-K09 the AEAD nonce is injective in (spi, channel_id, counter).

use crate::ascon::NONCE_LEN;

/// Wire length of the fixed Security Header.
pub const SEC_HEADER_LEN: usize = 11;

/// The authenticated, fixed-layout frame header.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SecurityHeader {
    /// Security Parameter Index — selects the security association (per channel).
    pub spi: u16,
    /// Logical channel (radio / cellular / …) — authenticated, blocks cross-link replay.
    pub channel_id: u8,
    /// Monotonic frame counter — the anti-replay sequence and nonce input.
    pub counter: u64,
}

impl SecurityHeader {
    /// Parse the fixed header from the front of `buf`. TOTAL: returns `None` for
    /// any buffer shorter than [`SEC_HEADER_LEN`]; never panics for any input.
    pub fn parse(buf: &[u8]) -> Option<SecurityHeader> {
        if buf.len() < SEC_HEADER_LEN {
            return None;
        }
        Some(SecurityHeader {
            spi: u16::from_le_bytes([buf[0], buf[1]]),
            channel_id: buf[2],
            counter: u64::from_le_bytes([
                buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10],
            ]),
        })
    }

    /// Write the header into the front of `buf`. Returns false if `buf` is too
    /// short (total — no panic).
    pub fn write(&self, buf: &mut [u8]) -> bool {
        if buf.len() < SEC_HEADER_LEN {
            return false;
        }
        buf[0..2].copy_from_slice(&self.spi.to_le_bytes());
        buf[2] = self.channel_id;
        buf[3..11].copy_from_slice(&self.counter.to_le_bytes());
        true
    }

    /// Derive the 128-bit AEAD nonce from the authenticated header fields. The
    /// (spi, channel_id, counter) prefix is embedded injectively, so distinct
    /// headers yield distinct nonces — the uniqueness precondition for AEAD
    /// (with per-channel keys + a monotonic counter + rekey-on-reboot, the
    /// nonce is never reused).
    pub fn nonce(&self) -> [u8; NONCE_LEN] {
        let mut n = [0u8; NONCE_LEN];
        n[0..2].copy_from_slice(&self.spi.to_le_bytes());
        n[2] = self.channel_id;
        n[3..11].copy_from_slice(&self.counter.to_le_bytes());
        // bytes 11..16 reserved (zero); the 11-byte prefix is already injective.
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn round_trip() {
        let h = SecurityHeader {
            spi: 0xBEEF,
            channel_id: 7,
            counter: 0x0123_4567_89AB_CDEF,
        };
        let mut buf = [0u8; SEC_HEADER_LEN];
        assert!(h.write(&mut buf));
        assert_eq!(SecurityHeader::parse(&buf), Some(h));
    }

    #[test]
    fn parse_too_short_is_none() {
        for n in 0..SEC_HEADER_LEN {
            assert_eq!(SecurityHeader::parse(&[0u8; SEC_HEADER_LEN][..n]), None);
        }
    }

    #[test]
    fn write_too_short_is_false() {
        let h = SecurityHeader { spi: 1, channel_id: 1, counter: 1 };
        let mut tiny = [0u8; SEC_HEADER_LEN - 1];
        assert!(!h.write(&mut tiny));
    }

    #[test]
    fn nonce_embeds_fields() {
        let h = SecurityHeader { spi: 0x0201, channel_id: 3, counter: 0x0A09_0807_0605_0403 };
        let n = h.nonce();
        assert_eq!(&n[0..2], &0x0201u16.to_le_bytes());
        assert_eq!(n[2], 3);
        assert_eq!(&n[3..11], &0x0A09_0807_0605_0403u64.to_le_bytes());
        assert_eq!(&n[11..16], &[0u8; 5]);
    }

    #[test]
    fn distinct_headers_distinct_nonces() {
        let a = SecurityHeader { spi: 1, channel_id: 0, counter: 5 }.nonce();
        let b = SecurityHeader { spi: 1, channel_id: 1, counter: 5 }.nonce(); // channel differs
        let c = SecurityHeader { spi: 1, channel_id: 0, counter: 6 }.nonce(); // counter differs
        assert_ne!(a, b);
        assert_ne!(a, c);
    }
}
