//! X25519 (RFC 7748) — ephemeral ECDH for relay-sec session establishment.
//!
//! Field arithmetic + Montgomery ladder ported from TweetNaCl (public domain),
//! the compact, audited reference. Representation: a field element is `[i64; 16]`
//! limbs of radix 2^16 over GF(2^255 - 19). One scalar multiplication per
//! handshake, so the table-free constant-time ladder is the right cost.
//!
//! Verified posture (matches the Ascon primitive): byte-exact against the
//! official RFC 7748 §5.2 / §6.1 known-answer vectors, Kani-total. Clamping is
//! built into the ladder (RFC 7748 §5). Per the v1.37 design review, callers
//! MUST treat an all-zero shared secret as a hard failure — [`x25519`] returns
//! `false` in that case (low-order / contributory-zero rejection, RFC 7748 §6.1).

#![allow(clippy::needless_range_loop)]

/// A field element: 16 limbs, radix 2^16, over GF(2^255 - 19).
type Gf = [i64; 16];

/// 121665 = 0x1_DB41 — the Montgomery curve constant (a24).
const A24: Gf = [0xDB41, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

/// The u-coordinate of the X25519 base point (u = 9).
pub const BASE_POINT: [u8; 32] = [
    9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
];

#[inline]
fn car25519(o: &mut Gf) {
    for i in 0..16 {
        o[i] += 1 << 16;
        let c = o[i] >> 16;
        if i < 15 {
            o[i + 1] += c - 1;
        } else {
            o[0] += 38 * (c - 1);
        }
        o[i] -= c << 16;
    }
}

/// Constant-time conditional swap of `p` and `q` when `b == 1`.
#[inline]
fn sel25519(p: &mut Gf, q: &mut Gf, b: i64) {
    let c = !(b - 1);
    for i in 0..16 {
        let t = c & (p[i] ^ q[i]);
        p[i] ^= t;
        q[i] ^= t;
    }
}

#[inline]
fn add(o: &mut Gf, a: &Gf, b: &Gf) {
    for i in 0..16 {
        o[i] = a[i] + b[i];
    }
}

#[inline]
fn sub(o: &mut Gf, a: &Gf, b: &Gf) {
    for i in 0..16 {
        o[i] = a[i] - b[i];
    }
}

#[inline]
fn mul(o: &mut Gf, a: &Gf, b: &Gf) {
    let mut t = [0i64; 31];
    for i in 0..16 {
        for j in 0..16 {
            t[i + j] += a[i] * b[j];
        }
    }
    for i in 0..15 {
        t[i] += 38 * t[i + 16];
    }
    o[..16].copy_from_slice(&t[..16]);
    car25519(o);
    car25519(o);
}

#[inline]
fn sqr(o: &mut Gf, a: &Gf) {
    let a2 = *a;
    mul(o, &a2, &a2);
}

/// Field inversion via Fermat: x^(p-2), p = 2^255 - 19.
fn inv25519(o: &mut Gf, i: &Gf) {
    let mut c = *i;
    for a in (0..=253).rev() {
        let c2 = c;
        sqr(&mut c, &c2);
        if a != 2 && a != 4 {
            let c3 = c;
            mul(&mut c, &c3, i);
        }
    }
    *o = c;
}

fn unpack25519(o: &mut Gf, n: &[u8; 32]) {
    for i in 0..16 {
        o[i] = n[2 * i] as i64 + ((n[2 * i + 1] as i64) << 8);
    }
    o[15] &= 0x7fff; // ignore the high bit of the u-coordinate (RFC 7748 §5)
}

fn pack25519(o: &mut [u8; 32], n: &Gf) {
    let mut t = *n;
    car25519(&mut t);
    car25519(&mut t);
    car25519(&mut t);
    for _ in 0..2 {
        let mut m: Gf = [0; 16];
        m[0] = t[0] - 0xffed;
        for i in 1..15 {
            m[i] = t[i] - 0xffff - ((m[i - 1] >> 16) & 1);
            m[i - 1] &= 0xffff;
        }
        m[15] = t[15] - 0x7fff - ((m[14] >> 16) & 1);
        let b = (m[15] >> 16) & 1;
        m[14] &= 0xffff;
        sel25519(&mut t, &mut m, 1 - b);
    }
    for i in 0..16 {
        o[2 * i] = (t[i] & 0xff) as u8;
        o[2 * i + 1] = (t[i] >> 8) as u8;
    }
}

/// X25519 scalar multiplication: `out = scalar * point` (RFC 7748).
///
/// `scalar` is clamped internally (RFC 7748 §5), so the caller may pass raw
/// random bytes as the private key. Returns `false` — and leaves `out` zeroed —
/// iff the result is the all-zero field element, i.e. `point` was a low-order /
/// contributory-zero point (RFC 7748 §6.1). A `false` return MUST abort the
/// handshake: the shared secret is unusable and an attacker may have forced it.
#[must_use]
pub fn x25519(out: &mut [u8; 32], scalar: &[u8; 32], point: &[u8; 32]) -> bool {
    let mut z = *scalar;
    z[0] &= 248;
    z[31] = (z[31] & 127) | 64;

    let mut x: Gf = [0; 16];
    unpack25519(&mut x, point);

    let mut a: Gf = [0; 16];
    let mut b: Gf = x;
    let mut c: Gf = [0; 16];
    let mut d: Gf = [0; 16];
    let mut e: Gf;
    let mut f: Gf;
    a[0] = 1;
    d[0] = 1;

    for i in (0..=254).rev() {
        let r = ((z[i >> 3] >> (i & 7)) & 1) as i64;
        sel25519(&mut a, &mut b, r);
        sel25519(&mut c, &mut d, r);
        e = [0; 16];
        add(&mut e, &a, &c);
        let a2 = a;
        sub(&mut a, &a2, &c);
        let c2 = c;
        add(&mut c, &b, &d);
        let b2 = b;
        sub(&mut b, &b2, &d);
        sqr(&mut d, &e);
        f = [0; 16];
        sqr(&mut f, &a);
        let c3 = c;
        let a3 = a;
        mul(&mut a, &c3, &a3);
        let b3 = b;
        mul(&mut c, &b3, &e);
        let _ = c2;
        let _ = b2;
        e = [0; 16];
        add(&mut e, &a, &c);
        let a4 = a;
        sub(&mut a, &a4, &c);
        let a5 = a;
        sqr(&mut b, &a5);
        let d2 = d;
        sub(&mut c, &d2, &f);
        let c4 = c;
        mul(&mut a, &c4, &A24);
        let a6 = a;
        add(&mut a, &a6, &d);
        let c5 = c;
        let a7 = a;
        mul(&mut c, &c5, &a7);
        let d3 = d;
        mul(&mut a, &d3, &f);
        let b4 = b;
        mul(&mut d, &b4, &x);
        let e2 = e;
        sqr(&mut b, &e2);
        sel25519(&mut a, &mut b, r);
        sel25519(&mut c, &mut d, r);
    }

    let mut c_inv: Gf = [0; 16];
    inv25519(&mut c_inv, &c);
    let a_final = a;
    mul(&mut a, &a_final, &c_inv);
    pack25519(out, &a);

    // Low-order / contributory-zero rejection (RFC 7748 §6.1), constant time.
    let mut acc = 0u8;
    for &byte in out.iter() {
        acc |= byte;
    }
    acc != 0
}

/// `out = scalar * BasePoint` — the public key for a private `scalar`.
#[must_use]
pub fn x25519_base(out: &mut [u8; 32], scalar: &[u8; 32]) -> bool {
    x25519(out, scalar, &BASE_POINT)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn hx(s: &str) -> [u8; 32] {
        let b = s.as_bytes();
        let mut o = [0u8; 32];
        let v = |c: u8| -> u8 {
            match c {
                b'0'..=b'9' => c - b'0',
                b'a'..=b'f' => c - b'a' + 10,
                _ => panic!("bad hex"),
            }
        };
        for i in 0..32 {
            o[i] = (v(b[2 * i]) << 4) | v(b[2 * i + 1]);
        }
        o
    }

    /// RFC 7748 §5.2, test vector 1.
    #[test]
    fn rfc7748_vector_1() {
        let k = hx("a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4");
        let u = hx("e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c");
        let want = hx("c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552");
        let mut out = [0u8; 32];
        assert!(x25519(&mut out, &k, &u));
        assert_eq!(out, want);
    }

    /// RFC 7748 §5.2, test vector 2.
    #[test]
    fn rfc7748_vector_2() {
        let k = hx("4b66e9d4d1b4673c5ad22691957d6af5c11b6421e0ea01d42ca4169e7918ba0d");
        let u = hx("e5210f12786811d3f4b7959d0538ae2c31dbe7106fc03c3efc4cd549c715a493");
        let want = hx("95cbde9476e8907d7aade45cb4b873f88b595a68799fa152e6f8f7647aac7957");
        let mut out = [0u8; 32];
        assert!(x25519(&mut out, &k, &u));
        assert_eq!(out, want);
    }

    /// RFC 7748 §6.1 — Alice/Bob public keys and the shared secret.
    #[test]
    fn rfc7748_diffie_hellman() {
        let a_priv = hx("77076d0a7318a57d3c16c17251b26645df4c2f87ebc0992ab177fba51db92c2a");
        let a_pub_want = hx("8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a");
        let b_priv = hx("5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2f8b27ff88e0eb");
        let b_pub_want = hx("de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f");
        let shared_want = hx("4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742");

        let mut a_pub = [0u8; 32];
        let mut b_pub = [0u8; 32];
        assert!(x25519_base(&mut a_pub, &a_priv));
        assert!(x25519_base(&mut b_pub, &b_priv));
        assert_eq!(a_pub, a_pub_want);
        assert_eq!(b_pub, b_pub_want);

        // ECDH from both sides must agree and match the RFC shared secret.
        let mut s_ab = [0u8; 32];
        let mut s_ba = [0u8; 32];
        assert!(x25519(&mut s_ab, &a_priv, &b_pub));
        assert!(x25519(&mut s_ba, &b_priv, &a_pub));
        assert_eq!(s_ab, s_ba);
        assert_eq!(s_ab, shared_want);
    }

    /// Low-order point (all-zero u) yields the all-zero secret and is REJECTED
    /// (RFC 7748 §6.1 contributory-zero rejection — design-review must-fix #4).
    #[test]
    fn low_order_point_rejected() {
        let k = hx("a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4");
        let zero = [0u8; 32];
        let mut out = [0xAAu8; 32];
        assert!(!x25519(&mut out, &k, &zero));
        assert_eq!(out, [0u8; 32]); // out is zeroed on rejection
    }
}
