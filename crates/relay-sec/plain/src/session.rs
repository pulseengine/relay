//! Session establishment — ephemeral X25519 key agreement with forward secrecy
//! and a rekey-on-reboot invariant (relay-sec, feature F4 / falcon v1.37).
//!
//! ## Construction (per the v1.37 design review)
//!
//! A 3-flight, PSK-authenticated, ephemeral-ephemeral Diffie-Hellman handshake.
//! This is **NNpsk2-style** — NOT Noise-KK: authentication rests on a *shared*
//! long-term per-channel key `k_id` (the security-association key), so it gives
//! mutual authentication + forward secrecy but, being symmetric-PSK, has **no
//! KCI resistance** — compromise of `k_id` is a total break of that one channel
//! (active impersonation included). `k_id` is per-channel, so a leak is
//! contained. This boundary is intentional and documented in SWREQ-SEC-P06.
//!
//! ```text
//!   I -> R : Hello    { e_pub_i, epoch_i, tag_i = MAC_kid(n(INIT,epoch_i), T1) }
//!   R -> I : Response { e_pub_r, epoch_r, tag_r = MAC_kid(n(RESP,epoch_r), T2) }
//!   I -> R : Confirm  { tag_c   = MAC_skmac(N_CONFIRM, T_session) }
//! ```
//!
//! Forward secrecy: the session key derives from `Z = X25519(e_self, e_peer)`
//! over **ephemeral** keys discarded after the handshake; `k_id` never enters the
//! KDF, so a later `k_id` compromise does not expose past sessions.
//!
//! ## Reviewer must-fixes, realised here
//!
//! 1. **Responder freshness** — `epoch_r` is the responder's fresh challenge,
//!    bound into `tag_r` and the confirmation, so the initiator's authenticator
//!    proves liveness in *this* session (defeats handshake replay).
//! 2. **No Ascon nonce reuse across reboots** — handshake MAC nonces are
//!    `(role ‖ epoch)`. `epoch` is a caller-supplied **reboot-persistent
//!    monotonic counter** (NVM-backed); never reset, so `(k_id, nonce)` never
//!    repeats. Caller contract on [`Config`].
//! 4. **Low-order rejection** — `Z == 0` aborts the handshake ([`x25519`]).
//! 5. **Rekey-on-reboot independent of RNG** — both epochs bind into the KDF
//!    transcript, so a post-reboot session key is unconditionally distinct from
//!    any pre-reboot one even if the RNG repeats an ephemeral.
//! 6. **Self/duplicate-share rejection** — a received share equal to our own
//!    aborts ([`HandshakeError::SelfShare`]).
//! 7. **Domain-separated keys** — `SK_enc` and `SK_mac` come from distinct KDF
//!    labels; never the same key for two roles.
//!
//! The responder rejects a non-increasing initiator epoch ([`EpochRegression`]),
//! and a data key is only ever *returned* once the handshake reaches Established
//! (key confirmation verified), so no authenticated frame is accepted earlier.

use crate::ascon;
use crate::x25519;

/// A fixed-capacity, panic-free transcript writer (no_alloc). Writes saturate at
/// `N` rather than panicking; all call sites size `N` exactly, so no truncation
/// occurs in practice — the saturation only exists to keep the type total.
pub(crate) struct Tw<const N: usize> {
    b: [u8; N],
    n: usize,
}

impl<const N: usize> Tw<N> {
    fn new() -> Self {
        Tw { b: [0u8; N], n: 0 }
    }
    fn put(&mut self, s: &[u8]) {
        let end = core::cmp::min(self.n + s.len(), N);
        let k = end - self.n;
        self.b[self.n..end].copy_from_slice(&s[..k]);
        self.n = end;
    }
    fn slice(&self) -> &[u8] {
        &self.b[..self.n]
    }
}

const PROTO: &[u8] = b"relay-sec/x25519/v1";

const ROLE_INIT: u8 = 1;
const ROLE_RESP: u8 = 2;

// KDF salt + fixed expand/extract nonces. Distinct leading bytes keep these from
// ever colliding with a handshake MAC nonce (which leads with ROLE_INIT/RESP).
const KDF_SALT: [u8; 16] = *b"relay-sec/kdf\0\0\0";
const N_EXTRACT: [u8; 16] = [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
const N_ENC: [u8; 16] = [11, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
const N_MAC: [u8; 16] = [12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
const N_CONFIRM: [u8; 16] = [13, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
const LABEL_ENC: &[u8] = b"relay-sec/enc";
const LABEL_MAC: &[u8] = b"relay-sec/mac";

/// What can go wrong in a handshake. Every path is one of these — no panics.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum HandshakeError {
    /// A long-term-key (`k_id`) MAC did not verify — wrong key or tampered.
    BadTag,
    /// The key-confirmation MAC did not verify — the peer derived a different
    /// session key (or tampered confirmation).
    BadConfirm,
    /// X25519 produced the all-zero shared secret — a low-order/contributory
    /// point was injected. Abort (RFC 7748 §6.1).
    LowOrderPoint,
    /// The peer's ephemeral share equals our own — reflection/self-session.
    SelfShare,
    /// The initiator epoch did not strictly increase past the last one seen —
    /// a replayed or stale handshake.
    EpochRegression,
}

/// The two domain-separated session keys produced by a completed handshake.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct SessionKeys {
    /// AEAD/encryption subkey.
    pub enc: [u8; 16],
    /// Authentication subkey (also used for key confirmation).
    pub mac: [u8; 16],
}

/// Per-channel handshake configuration.
///
/// **Caller contract:** `epoch` values passed to [`Initiator::start`] /
/// [`Responder::receive`] MUST come from a reboot-persistent monotonic counter
/// (e.g. NVM-backed) that never resets and strictly increases per handshake.
/// This is what guarantees Ascon nonce uniqueness under `k_id` across reboots
/// and the rekey-on-reboot key distinctness (review must-fixes #2, #5).
#[derive(Clone, Copy)]
pub struct Config {
    /// Security Parameter Index — identifies the security association.
    pub spi: u16,
    /// Channel id within the association.
    pub channel_id: u8,
    /// Long-term shared per-channel key (the SA key).
    pub k_id: [u8; 16],
}

/// Flight 1: initiator → responder.
#[derive(Clone, Copy, Debug)]
pub struct Hello {
    /// Initiator ephemeral public key.
    pub e_pub: [u8; 32],
    /// Initiator persistent-monotonic epoch.
    pub epoch: u64,
    /// MAC over the init transcript under `k_id`.
    pub tag: [u8; 16],
}

/// Flight 2: responder → initiator.
#[derive(Clone, Copy, Debug)]
pub struct Response {
    /// Responder ephemeral public key.
    pub e_pub: [u8; 32],
    /// Responder persistent-monotonic epoch (its freshness challenge).
    pub epoch: u64,
    /// MAC over the response transcript under `k_id`.
    pub tag: [u8; 16],
}

/// Flight 3: initiator → responder (key confirmation).
#[derive(Clone, Copy, Debug)]
pub struct Confirm {
    /// MAC over the session transcript under the derived `SK_mac`.
    pub tag: [u8; 16],
}

pub(crate) fn handshake_nonce(role: u8, epoch: u64) -> [u8; 16] {
    let mut n = [0u8; 16];
    n[0] = role;
    n[8..16].copy_from_slice(&epoch.to_le_bytes());
    n
}

/// The session transcript both sides bind into the KDF and confirmation:
/// `spi ‖ channel_id ‖ epoch_i ‖ epoch_r ‖ e_pub_i ‖ e_pub_r`. Binding both
/// epochs makes the derived key unconditionally distinct across reboots (#5),
/// and binding both ephemerals defeats unknown-key-share.
pub(crate) fn session_transcript(
    cfg: &Config,
    epoch_i: u64,
    epoch_r: u64,
    e_pub_i: &[u8; 32],
    e_pub_r: &[u8; 32],
) -> Tw<96> {
    let mut t = Tw::<96>::new();
    t.put(&cfg.spi.to_le_bytes());
    t.put(&[cfg.channel_id]);
    t.put(&epoch_i.to_le_bytes());
    t.put(&epoch_r.to_le_bytes());
    t.put(e_pub_i);
    t.put(e_pub_r);
    t
}

fn init_transcript(cfg: &Config, epoch_i: u64, e_pub_i: &[u8; 32]) -> Tw<64> {
    let mut t = Tw::<64>::new();
    t.put(PROTO);
    t.put(&cfg.spi.to_le_bytes());
    t.put(&[cfg.channel_id]);
    t.put(&epoch_i.to_le_bytes());
    t.put(e_pub_i);
    t
}

fn resp_transcript(
    cfg: &Config,
    epoch_i: u64,
    epoch_r: u64,
    e_pub_i: &[u8; 32],
    e_pub_r: &[u8; 32],
) -> Tw<128> {
    let mut t = Tw::<128>::new();
    t.put(PROTO);
    t.put(&cfg.spi.to_le_bytes());
    t.put(&[cfg.channel_id]);
    t.put(&epoch_i.to_le_bytes());
    t.put(&epoch_r.to_le_bytes());
    t.put(e_pub_i);
    t.put(e_pub_r);
    t
}

/// Extract-then-expand KDF (HKDF-shaped) over Ascon-MAC. `Z` and the session
/// transcript are extracted into a PRK; `SK_enc`/`SK_mac` expand from it under
/// distinct labels + nonces (domain separation, review must-fix #7).
fn derive_keys(z: &[u8; 32], transcript: &[u8]) -> SessionKeys {
    let mut ex = Tw::<160>::new();
    ex.put(z);
    ex.put(transcript);
    let prk = ascon::mac(&KDF_SALT, &N_EXTRACT, ex.slice());
    let enc = ascon::mac(&prk, &N_ENC, LABEL_ENC);
    let mac = ascon::mac(&prk, &N_MAC, LABEL_MAC);
    SessionKeys { enc, mac }
}

/// Initiator handshake state (holds the ephemeral private key until completion).
pub struct Initiator {
    cfg: Config,
    e_priv: [u8; 32],
    e_pub: [u8; 32],
    epoch: u64,
}

impl Initiator {
    /// Begin a handshake. `e_priv` is the freshly-generated ephemeral private key
    /// (caller supplies CSPRNG bytes — review must-fix #3); `epoch` is the
    /// persistent-monotonic counter (must-fix #2).
    pub fn start(cfg: Config, e_priv: [u8; 32], epoch: u64) -> (Self, Hello) {
        let mut e_pub = [0u8; 32];
        // Base-point mult of a clamped non-zero scalar is never all-zero.
        let _ = x25519::x25519_base(&mut e_pub, &e_priv);
        let t1 = init_transcript(&cfg, epoch, &e_pub);
        let tag = ascon::mac(&cfg.k_id, &handshake_nonce(ROLE_INIT, epoch), t1.slice());
        let hello = Hello { e_pub, epoch, tag };
        (
            Initiator {
                cfg,
                e_priv,
                e_pub,
                epoch,
            },
            hello,
        )
    }

    /// Complete the handshake against the responder's reply: verify the
    /// responder, derive the session keys, and emit the confirmation.
    pub fn finish(self, resp: &Response) -> Result<(SessionKeys, Confirm), HandshakeError> {
        if resp.e_pub == self.e_pub {
            return Err(HandshakeError::SelfShare);
        }
        let t2 = resp_transcript(&self.cfg, self.epoch, resp.epoch, &self.e_pub, &resp.e_pub);
        if !ascon::mac_verify(
            &self.cfg.k_id,
            &handshake_nonce(ROLE_RESP, resp.epoch),
            t2.slice(),
            &resp.tag,
        ) {
            return Err(HandshakeError::BadTag);
        }
        let mut z = [0u8; 32];
        if !x25519::x25519(&mut z, &self.e_priv, &resp.e_pub) {
            return Err(HandshakeError::LowOrderPoint);
        }
        let ts = session_transcript(&self.cfg, self.epoch, resp.epoch, &self.e_pub, &resp.e_pub);
        let keys = derive_keys(&z, ts.slice());
        let tag = ascon::mac(&keys.mac, &N_CONFIRM, ts.slice());
        Ok((keys, Confirm { tag }))
    }
}

/// Responder handshake state (holds the derived keys until confirmation).
pub struct Responder {
    cfg: Config,
    epoch_i: u64,
    epoch_r: u64,
    e_pub_i: [u8; 32],
    e_pub_r: [u8; 32],
    keys: SessionKeys,
}

impl Responder {
    /// Receive flight 1, authenticate the initiator, derive keys, and reply.
    ///
    /// `last_peer_epoch` is the highest initiator epoch previously accepted on
    /// this channel; a `hello.epoch` not strictly greater is rejected as a
    /// replay. `epoch_r` is the responder's own persistent-monotonic epoch.
    pub fn receive(
        cfg: Config,
        hello: &Hello,
        e_priv: [u8; 32],
        epoch_r: u64,
        last_peer_epoch: u64,
    ) -> Result<(Self, Response), HandshakeError> {
        if hello.epoch <= last_peer_epoch {
            return Err(HandshakeError::EpochRegression);
        }
        let t1 = init_transcript(&cfg, hello.epoch, &hello.e_pub);
        if !ascon::mac_verify(
            &cfg.k_id,
            &handshake_nonce(ROLE_INIT, hello.epoch),
            t1.slice(),
            &hello.tag,
        ) {
            return Err(HandshakeError::BadTag);
        }
        let mut e_pub_r = [0u8; 32];
        let _ = x25519::x25519_base(&mut e_pub_r, &e_priv);
        if hello.e_pub == e_pub_r {
            return Err(HandshakeError::SelfShare);
        }
        let mut z = [0u8; 32];
        if !x25519::x25519(&mut z, &e_priv, &hello.e_pub) {
            return Err(HandshakeError::LowOrderPoint);
        }
        let ts = session_transcript(&cfg, hello.epoch, epoch_r, &hello.e_pub, &e_pub_r);
        let keys = derive_keys(&z, ts.slice());
        let t2 = resp_transcript(&cfg, hello.epoch, epoch_r, &hello.e_pub, &e_pub_r);
        let tag = ascon::mac(&cfg.k_id, &handshake_nonce(ROLE_RESP, epoch_r), t2.slice());
        let response = Response {
            e_pub: e_pub_r,
            epoch: epoch_r,
            tag,
        };
        Ok((
            Responder {
                cfg,
                epoch_i: hello.epoch,
                epoch_r,
                e_pub_i: hello.e_pub,
                e_pub_r,
                keys,
            },
            response,
        ))
    }

    /// Verify the initiator's key confirmation. Only on success are the session
    /// keys released (Established) — no data key exists before this point.
    pub fn finish(self, confirm: &Confirm) -> Result<SessionKeys, HandshakeError> {
        let ts = session_transcript(
            &self.cfg,
            self.epoch_i,
            self.epoch_r,
            &self.e_pub_i,
            &self.e_pub_r,
        );
        if !ascon::mac_verify(&self.keys.mac, &N_CONFIRM, ts.slice(), &confirm.tag) {
            return Err(HandshakeError::BadConfirm);
        }
        Ok(self.keys)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn cfg() -> Config {
        Config {
            spi: 0x1234,
            channel_id: 7,
            k_id: [0x11; 16],
        }
    }

    // Distinct, deterministic "ephemerals" for the tests (NOT how production
    // gets randomness — production supplies CSPRNG bytes).
    fn priv_i() -> [u8; 32] {
        [0x42; 32]
    }
    fn priv_r() -> [u8; 32] {
        [0x99; 32]
    }

    // Extract the error without requiring Debug on the secret-bearing Responder
    // (we deliberately do NOT derive Debug on key-bearing handshake state).
    fn recv_err(r: Result<(Responder, Response), HandshakeError>) -> HandshakeError {
        match r {
            Ok(_) => panic!("expected a handshake error"),
            Err(e) => e,
        }
    }

    /// The happy path: both sides reach Established with identical keys.
    #[test]
    fn handshake_roundtrip_agrees_on_keys() {
        let (init, hello) = Initiator::start(cfg(), priv_i(), 100);
        let (resp, response) = Responder::receive(cfg(), &hello, priv_r(), 200, 0).unwrap();
        let (ikeys, confirm) = init.finish(&response).unwrap();
        let rkeys = resp.finish(&confirm).unwrap();
        assert_eq!(ikeys, rkeys);
        // The two subkeys are independent (domain separation actually happened).
        assert_ne!(ikeys.enc, ikeys.mac);
    }

    /// Forward-secrecy smoke: different ephemerals ⇒ different session key,
    /// even with the SAME long-term k_id and the same epochs.
    #[test]
    fn distinct_ephemerals_distinct_keys() {
        let (init1, h1) = Initiator::start(cfg(), [0x01; 32], 1);
        let (r1, resp1) = Responder::receive(cfg(), &h1, [0x02; 32], 1, 0).unwrap();
        let (k1, _) = init1.finish(&resp1).unwrap();
        let _ = r1;

        let (init2, h2) = Initiator::start(cfg(), [0x03; 32], 1);
        let (r2, resp2) = Responder::receive(cfg(), &h2, [0x04; 32], 1, 0).unwrap();
        let (k2, _) = init2.finish(&resp2).unwrap();
        let _ = r2;

        assert_ne!(k1, k2);
    }

    /// REKEY-ON-REBOOT (the headline invariant): same k_id, same peer, but a
    /// bumped epoch yields a different session key — UNCONDITIONALLY, even if the
    /// ephemerals were reused (RNG repeat after reboot). And a MAC made under the
    /// pre-reboot key is rejected under the post-reboot key.
    #[test]
    fn rekey_on_reboot_changes_keys_even_if_rng_repeats() {
        // Pre-reboot session at epochs (10, 20) with fixed ephemerals.
        let (i_a, h_a) = Initiator::start(cfg(), [0x07; 32], 10);
        let (r_a, resp_a) = Responder::receive(cfg(), &h_a, [0x08; 32], 20, 0).unwrap();
        let (k_a, _) = i_a.finish(&resp_a).unwrap();
        let _ = r_a;

        // Post-reboot: epochs bumped to (11, 21) but the SAME ephemerals reused.
        let (i_b, h_b) = Initiator::start(cfg(), [0x07; 32], 11);
        let (r_b, resp_b) = Responder::receive(cfg(), &h_b, [0x08; 32], 21, 10).unwrap();
        let (k_b, _) = i_b.finish(&resp_b).unwrap();
        let _ = r_b;

        assert_ne!(k_a, k_b, "epoch must change the key independent of the RNG");

        // A frame authenticated under the old session key fails under the new one.
        let frame = b"disarm";
        let nonce = [0u8; 16];
        let old_tag = ascon::mac(&k_a.mac, &nonce, frame);
        assert!(!ascon::mac_verify(&k_b.mac, &nonce, frame, &old_tag));
    }

    /// Replay of a captured initiator Hello is rejected by epoch monotonicity.
    #[test]
    fn replayed_hello_rejected_by_epoch() {
        let (_init, hello) = Initiator::start(cfg(), priv_i(), 100);
        // First acceptance moves last_peer_epoch to 100.
        let (_r, _resp) = Responder::receive(cfg(), &hello, priv_r(), 200, 0).unwrap();
        // Replaying the SAME hello with last_peer_epoch=100 is rejected.
        let err = recv_err(Responder::receive(cfg(), &hello, priv_r(), 201, 100));
        assert_eq!(err, HandshakeError::EpochRegression);
    }

    /// A tampered responder tag is rejected (BadTag), not silently accepted.
    #[test]
    fn tampered_response_rejected() {
        let (init, hello) = Initiator::start(cfg(), priv_i(), 5);
        let (_r, mut response) = Responder::receive(cfg(), &hello, priv_r(), 5, 0).unwrap();
        response.tag[0] ^= 1;
        assert_eq!(init.finish(&response).unwrap_err(), HandshakeError::BadTag);
    }

    /// A wrong long-term key makes the responder reject the initiator (BadTag) —
    /// only a holder of k_id can open the channel.
    #[test]
    fn wrong_kid_rejected() {
        let (_init, hello) = Initiator::start(cfg(), priv_i(), 9);
        let mut bad = cfg();
        bad.k_id = [0x22; 16];
        assert_eq!(
            recv_err(Responder::receive(bad, &hello, priv_r(), 9, 0)),
            HandshakeError::BadTag
        );
    }

    /// A low-order peer share (all-zero) aborts with LowOrderPoint, not a
    /// degenerate zero shared secret.
    #[test]
    fn low_order_share_rejected() {
        let (_init, mut hello) = Initiator::start(cfg(), priv_i(), 3);
        hello.e_pub = [0u8; 32]; // low-order point
        // Re-MAC so the tag is valid and we reach the X25519 step.
        let t1 = init_transcript(&cfg(), 3, &hello.e_pub);
        hello.tag = ascon::mac(&cfg().k_id, &handshake_nonce(ROLE_INIT, 3), t1.slice());
        assert_eq!(
            recv_err(Responder::receive(cfg(), &hello, priv_r(), 3, 0)),
            HandshakeError::LowOrderPoint
        );
    }

    /// A reflected/self share (peer share == our own) is rejected.
    #[test]
    fn self_share_rejected() {
        // Force responder to use the same ephemeral as the initiator, so the
        // initiator sees its own e_pub coming back.
        let (init, hello) = Initiator::start(cfg(), priv_i(), 4);
        // Responder uses the SAME private key, producing the same e_pub.
        let (_r, response) = match Responder::receive(cfg(), &hello, priv_i(), 4, 0) {
            Ok(v) => v,
            // receive itself rejects self-share when it computes its own e_pub.
            Err(e) => {
                assert_eq!(e, HandshakeError::SelfShare);
                return;
            }
        };
        assert_eq!(
            init.finish(&response).unwrap_err(),
            HandshakeError::SelfShare
        );
    }

    /// A tampered confirmation is rejected by the responder (BadConfirm).
    #[test]
    fn tampered_confirm_rejected() {
        let (init, hello) = Initiator::start(cfg(), priv_i(), 6);
        let (resp, response) = Responder::receive(cfg(), &hello, priv_r(), 6, 0).unwrap();
        let (_k, mut confirm) = init.finish(&response).unwrap();
        confirm.tag[0] ^= 1;
        assert_eq!(
            resp.finish(&confirm).unwrap_err(),
            HandshakeError::BadConfirm
        );
    }
}
