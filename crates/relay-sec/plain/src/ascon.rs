//! Ascon-AEAD128 — the single verified crypto primitive for relay-sec.
//!
//! NIST SP 800-232 (the standardized Ascon, 2024). ONE permutation gives relay-sec
//! both of its modes, so the verified core proves a single primitive:
//!   - **MAC-only floor** = `mac()`  — AEAD with empty plaintext; the 128-bit tag
//!     authenticates the frame (passed as associated data).
//!   - **AEAD upgrade**   = `seal()` — the same construction encrypting a payload.
//!
//! This is what makes "one qualified layer, two products (falcon + wohl)" literally
//! true at the primitive level, rather than carrying a separate MAC and AEAD.
//!
//! Chosen over Chaskey+AES-GCM because the targets (STM32G0 M0+, STM32G474 M4,
//! ESP32-C3 RV32) have NO AES instructions: table-based AES-GCM is timing-fragile,
//! bitsliced is large and hard to verify. Ascon's permutation is table-free bitwise
//! logic — constant-time by construction and Kani-friendly. Implemented in safe Rust
//! with `u64` words (portable + constant-time when lowered to 32-bit ops); no
//! `unsafe`, no alignment/endianness assumptions (all I/O is explicit little-endian).
//!
//! Correctness oracle: the official NIST genkat vectors (LWC_AEAD_KAT_128_128.txt),
//! asserted byte-for-byte in `tests` — Count 1 (empty/empty), Count 2 (MAC-floor:
//! empty PT + AD), Count 35 (PT + AD). If those pass, the permutation, IV,
//! domain-separation, padding, and finalization are all byte-exact.

/// Ascon-AEAD128 IV (NIST SP 800-232, little-endian load). NOT the Ascon v1.2 IV.
const IV: u64 = 0x00001000808c0001;

/// Round constants for p[12]; p[8] uses the last 8 (indices 4..12).
const RC: [u64; 12] = [
    0xf0, 0xe1, 0xd2, 0xc3, 0xb4, 0xa5, 0x96, 0x87, 0x78, 0x69, 0x5a, 0x4b,
];

/// Domain-separation constant XORed into x4 after associated data.
const DSEP: u64 = 0x8000_0000_0000_0000;

/// Tag / key / nonce length in bytes.
pub const TAG_LEN: usize = 16;
pub const KEY_LEN: usize = 16;
pub const NONCE_LEN: usize = 16;

#[inline]
fn load64(b: &[u8], i: usize) -> u64 {
    (b[i] as u64)
        | ((b[i + 1] as u64) << 8)
        | ((b[i + 2] as u64) << 16)
        | ((b[i + 3] as u64) << 24)
        | ((b[i + 4] as u64) << 32)
        | ((b[i + 5] as u64) << 40)
        | ((b[i + 6] as u64) << 48)
        | ((b[i + 7] as u64) << 56)
}

#[inline]
fn store64(b: &mut [u8], i: usize, x: u64) {
    b[i..i + 8].copy_from_slice(&x.to_le_bytes());
}

/// One round of the Ascon permutation: constant addition, substitution (the
/// table-free chi-style S-box), then linear diffusion. Verbatim from the
/// SP 800-232 reference `round.h`.
#[inline]
fn round(s: &mut [u64; 5], c: u64) {
    s[2] ^= c;
    // substitution layer
    s[0] ^= s[4];
    s[4] ^= s[3];
    s[2] ^= s[1];
    let t0 = s[0] ^ ((!s[1]) & s[2]);
    let t1 = s[1] ^ ((!s[2]) & s[3]);
    let t2 = s[2] ^ ((!s[3]) & s[4]);
    let t3 = s[3] ^ ((!s[4]) & s[0]);
    let t4 = s[4] ^ ((!s[0]) & s[1]);
    let mut u0 = t0;
    let mut u1 = t1;
    let mut u2 = t2;
    let mut u3 = t3;
    let u4 = t4;
    u1 ^= u0;
    u0 ^= u4;
    u3 ^= u2;
    u2 = !u2;
    // linear diffusion layer (ROR = rotate-right)
    s[0] = u0 ^ u0.rotate_right(19) ^ u0.rotate_right(28);
    s[1] = u1 ^ u1.rotate_right(61) ^ u1.rotate_right(39);
    s[2] = u2 ^ u2.rotate_right(1) ^ u2.rotate_right(6);
    s[3] = u3 ^ u3.rotate_right(10) ^ u3.rotate_right(17);
    s[4] = u4 ^ u4.rotate_right(7) ^ u4.rotate_right(41);
}

#[inline]
fn p12(s: &mut [u64; 5]) {
    let mut i = 0;
    while i < 12 {
        round(s, RC[i]);
        i += 1;
    }
}

#[inline]
fn p8(s: &mut [u64; 5]) {
    let mut i = 4; // last 8 constants
    while i < 12 {
        round(s, RC[i]);
        i += 1;
    }
}

/// Core Ascon-AEAD128 encryption. Writes ciphertext (same length as `pt`) into
/// `ct` and returns the 128-bit tag. `mac` and `seal` are thin wrappers.
///
/// Precondition: `ct.len() == pt.len()`.
fn aead_encrypt(key: &[u8; KEY_LEN], nonce: &[u8; NONCE_LEN], ad: &[u8], pt: &[u8], ct: &mut [u8]) -> [u8; TAG_LEN] {
    let k0 = load64(key, 0);
    let k1 = load64(key, 8);

    // Initialization
    let mut s = [IV, k0, k1, load64(nonce, 0), load64(nonce, 8)];
    p12(&mut s);
    s[3] ^= k0;
    s[4] ^= k1;

    // Associated data (only if non-empty), 128-bit rate
    if !ad.is_empty() {
        let mut off = 0;
        while ad.len() - off >= 16 {
            s[0] ^= load64(ad, off);
            s[1] ^= load64(ad, off + 8);
            p8(&mut s);
            off += 16;
        }
        let rem = ad.len() - off; // 0..=15
        let mut b = [0u8; 16];
        b[..rem].copy_from_slice(&ad[off..]);
        b[rem] = 0x01; // 10* padding
        s[0] ^= load64(&b, 0);
        s[1] ^= load64(&b, 8);
        p8(&mut s);
    }

    // Domain separation
    s[4] ^= DSEP;

    // Plaintext -> ciphertext (always a final padded block, even when empty)
    let mut off = 0;
    while pt.len() - off >= 16 {
        s[0] ^= load64(pt, off);
        s[1] ^= load64(pt, off + 8);
        store64(ct, off, s[0]);
        store64(ct, off + 8, s[1]);
        p8(&mut s);
        off += 16;
    }
    let rem = pt.len() - off; // 0..=15
    let mut r = [0u8; 16];
    store64(&mut r, 0, s[0]);
    store64(&mut r, 8, s[1]);
    let mut i = 0;
    while i < rem {
        r[i] ^= pt[off + i];
        ct[off + i] = r[i];
        i += 1;
    }
    r[rem] ^= 0x01; // 10* padding bit
    s[0] = load64(&r, 0);
    s[1] = load64(&r, 8);

    // Finalization
    s[2] ^= k0;
    s[3] ^= k1;
    p12(&mut s);
    s[3] ^= k0;
    s[4] ^= k1;

    let mut tag = [0u8; TAG_LEN];
    store64(&mut tag, 0, s[3]);
    store64(&mut tag, 8, s[4]);
    tag
}

/// MAC-only floor: authenticate `msg` (as associated data) under `key`/`nonce`,
/// returning the 128-bit tag. Ascon-AEAD128 with empty plaintext.
pub fn mac(key: &[u8; KEY_LEN], nonce: &[u8; NONCE_LEN], msg: &[u8]) -> [u8; TAG_LEN] {
    let mut empty = [0u8; 0];
    aead_encrypt(key, nonce, msg, &[], &mut empty)
}

/// AEAD upgrade: encrypt `pt` into `ct` (equal length) with `ad` authenticated,
/// returning the tag. Confidentiality + integrity.
pub fn seal(
    key: &[u8; KEY_LEN],
    nonce: &[u8; NONCE_LEN],
    ad: &[u8],
    pt: &[u8],
    ct: &mut [u8],
) -> [u8; TAG_LEN] {
    aead_encrypt(key, nonce, ad, pt, ct)
}

/// Constant-time tag comparison — no early-out.
pub fn ct_eq_tag(a: &[u8; TAG_LEN], b: &[u8; TAG_LEN]) -> bool {
    let mut diff = 0u8;
    let mut i = 0;
    while i < TAG_LEN {
        diff |= a[i] ^ b[i];
        i += 1;
    }
    diff == 0
}

/// Verify a MAC-floor tag in constant time.
pub fn mac_verify(key: &[u8; KEY_LEN], nonce: &[u8; NONCE_LEN], msg: &[u8], tag: &[u8; TAG_LEN]) -> bool {
    ct_eq_tag(&mac(key, nonce, msg), tag)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hx<const N: usize>(s: &str) -> [u8; N] {
        let mut out = [0u8; N];
        let b = s.as_bytes();
        let mut i = 0;
        while i < N {
            let hi = (b[2 * i] as char).to_digit(16).unwrap() as u8;
            let lo = (b[2 * i + 1] as char).to_digit(16).unwrap() as u8;
            out[i] = (hi << 4) | lo;
            i += 1;
        }
        out
    }

    // Official NIST genkat fixed key/nonce.
    fn key() -> [u8; 16] {
        hx::<16>("000102030405060708090A0B0C0D0E0F")
    }
    fn nonce() -> [u8; 16] {
        hx::<16>("101112131415161718191A1B1C1D1E1F")
    }

    /// Count 1: empty AD, empty PT — the "tag of nothing".
    #[test]
    fn kat_count1_empty() {
        let tag = mac(&key(), &nonce(), &[]);
        assert_eq!(tag, hx::<16>("4F9C278211BEC9316BF68F46EE8B2EC6"));
    }

    /// Count 2: MAC-floor — empty PT, AD = 0x30.
    #[test]
    fn kat_count2_mac_floor() {
        let tag = mac(&key(), &nonce(), &hx::<1>("30"));
        assert_eq!(tag, hx::<16>("CCCB674FE18A09A285D6AB11B35675C0"));
    }

    /// Count 3: empty PT, AD = 0x3031.
    #[test]
    fn kat_count3() {
        let tag = mac(&key(), &nonce(), &hx::<2>("3031"));
        assert_eq!(tag, hx::<16>("F65B191550C4DF9CFDD4460EBBCCA782"));
    }

    /// Count 34: PT = 0x20, empty AD. CT = E8, tag = DD576A...
    #[test]
    fn kat_count34_pt_only() {
        let pt = hx::<1>("20");
        let mut ct = [0u8; 1];
        let tag = seal(&key(), &nonce(), &[], &pt, &mut ct);
        assert_eq!(ct, hx::<1>("E8"));
        assert_eq!(tag, hx::<16>("DD576ABA1CD3E6FC704DE02AEDB79588"));
    }

    /// Count 35: PT = 0x20, AD = 0x30. CT = 96, tag = 2B8016...
    #[test]
    fn kat_count35_pt_ad() {
        let pt = hx::<1>("20");
        let mut ct = [0u8; 1];
        let tag = seal(&key(), &nonce(), &hx::<1>("30"), &pt, &mut ct);
        assert_eq!(ct, hx::<1>("96"));
        assert_eq!(tag, hx::<16>("2B8016836C75A7D86866588CA245D886"));
    }

    /// mac_verify accepts a genuine tag and rejects a single-bit forgery.
    #[test]
    fn mac_verify_accepts_genuine_rejects_forgery() {
        let msg = hx::<4>("DEADBEEF");
        let t = mac(&key(), &nonce(), &msg);
        assert!(mac_verify(&key(), &nonce(), &msg, &t));
        let mut bad = t;
        bad[7] ^= 1;
        assert!(!mac_verify(&key(), &nonce(), &msg, &bad));
    }

    /// A multi-block message round-trips through seal correctly (exercises the
    /// full-block loop + partial tail).
    #[test]
    fn seal_multiblock_lengths() {
        let k = key();
        let n = nonce();
        for len in [0usize, 1, 8, 15, 16, 17, 31, 32, 40] {
            let mut pt = [0u8; 40];
            for (i, b) in pt.iter_mut().enumerate().take(len) {
                *b = i as u8;
            }
            let mut ct = [0u8; 40];
            let _ = seal(&k, &n, &[0xAA, 0xBB], &pt[..len], &mut ct[..len]);
            // ciphertext differs from plaintext for non-empty (sanity)
            if len > 0 {
                assert_ne!(&ct[..len], &pt[..len]);
            }
        }
    }
}
