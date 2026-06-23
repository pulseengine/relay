//! Relay RC — pilot stick input → manual flight-mode setpoints.
//!
//! The everyday PX4 manual modes, missing until now (falcon was offboard/MAVLink
//! only). Maps a normalized RC stick input to either:
//!
//!  * **Stabilized** — sticks command a bounded ATTITUDE setpoint (roll/pitch
//!    tilt, yaw rate); centre stick = level, so releasing the sticks auto-levels.
//!    Feeds the verified attitude controller (relay-geo/relay-att).
//!  * **Acro** — sticks command a bounded BODY-RATE setpoint directly, feeding
//!    the verified rate controller (relay-adrc/relay-rate). No auto-level.
//!
//! The safety-relevant property — proven by Kani — is that the OUTPUT IS ALWAYS
//! BOUNDED: for ANY stick input (including NaN/∞ from a glitching receiver) the
//! commanded tilt never exceeds `max_tilt`, the rates never exceed their max, and
//! thrust stays in [0, 1]. A bad RC frame can never command an unbounded angle or
//! rate.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// Normalized pilot stick input. Each axis in [-1, 1]; throttle in [0, 1].
/// Out-of-range / NaN values are sanitised by the mapping functions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RcInput {
    /// Roll stick (right positive).
    pub roll: f32,
    /// Pitch stick (forward/nose-down positive).
    pub pitch: f32,
    /// Yaw stick (clockwise positive).
    pub yaw: f32,
    /// Throttle (0 = idle, 1 = full).
    pub throttle: f32,
}

/// Stabilized-mode output: a bounded attitude setpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AttitudeCmd {
    /// Roll angle setpoint (rad).
    pub roll_rad: f32,
    /// Pitch angle setpoint (rad).
    pub pitch_rad: f32,
    /// Yaw-rate setpoint (rad/s).
    pub yaw_rate: f32,
    /// Collective thrust [0, 1].
    pub thrust: f32,
}

/// Acro-mode output: a bounded body-rate setpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RateCmd {
    /// Roll-rate setpoint (rad/s).
    pub roll_rate: f32,
    /// Pitch-rate setpoint (rad/s).
    pub pitch_rate: f32,
    /// Yaw-rate setpoint (rad/s).
    pub yaw_rate: f32,
    /// Collective thrust [0, 1].
    pub thrust: f32,
}

/// Sanitise a stick axis to [-1, 1]; NaN/∞ ⇒ 0 (centre = safe).
#[inline]
pub(crate) fn unit(x: f32) -> f32 {
    if x.is_nan() {
        0.0
    } else if x > 1.0 {
        1.0
    } else if x < -1.0 {
        -1.0
    } else {
        x
    }
}

/// Sanitise throttle to [0, 1]; NaN ⇒ 0 (idle = safe).
#[inline]
pub(crate) fn throttle01(x: f32) -> f32 {
    if x.is_nan() {
        0.0
    } else if x > 1.0 {
        1.0
    } else if x < 0.0 {
        0.0
    } else {
        x
    }
}

/// Clamp a positive limit to a finite non-negative value (NaN/neg ⇒ 0).
#[inline]
fn limit(x: f32) -> f32 {
    if x.is_finite() && x > 0.0 {
        x
    } else {
        0.0
    }
}

/// Stabilized mode: sticks → a bounded attitude setpoint. Centre roll/pitch sticks
/// command level (0 rad), so releasing auto-levels. `max_tilt_rad` bounds roll and
/// pitch; `max_yaw_rate` bounds the yaw rate.
pub fn stabilized(rc: RcInput, max_tilt_rad: f32, max_yaw_rate: f32) -> AttitudeCmd {
    let tilt = limit(max_tilt_rad);
    let yawr = limit(max_yaw_rate);
    AttitudeCmd {
        roll_rad: unit(rc.roll) * tilt,
        pitch_rad: unit(rc.pitch) * tilt,
        yaw_rate: unit(rc.yaw) * yawr,
        thrust: throttle01(rc.throttle),
    }
}

/// Acro mode: sticks → a bounded body-rate setpoint. `max_rate` bounds every axis.
pub fn acro(rc: RcInput, max_rate: f32) -> RateCmd {
    let r = limit(max_rate);
    RateCmd {
        roll_rate: unit(rc.roll) * r,
        pitch_rate: unit(rc.pitch) * r,
        yaw_rate: unit(rc.yaw) * r,
        thrust: throttle01(rc.throttle),
    }
}

// ---------------------------------------------------------------------------
// SBUS wire decode (RC-P02) — the verified-decode lane upstream of RcInput.
//
// Futaba/FrSky S.BUS: a fixed 25-byte frame carrying 16 channels of 11 bits
// each (LSB-first, packed across 22 bytes so channels straddle byte
// boundaries), a flags byte, and an end byte. NO CRC — integrity is the
// start/end markers + the fixed length. The decode splits along the codebase's
// verification line: the INTEGER bit-unpacking (every channel <= 0x7FF,
// total/panic-free over ALL frames) is Kani-proven (RC-K03); the f32
// channel->stick normalisation (bounded output) is proptest-gated, since Kani
// on f32 multiply/divide is intractable. Failsafe is EXPOSE-ONLY: the flags are
// surfaced for the relay-fsafe arbiter to act on; the decode never overrides
// channels (single-responsibility — policy is not the decoder's job).
// ---------------------------------------------------------------------------

/// SBUS frame length in bytes (fixed).
pub const SBUS_FRAME_LEN: usize = 25;
/// SBUS start-of-frame marker (byte 0).
pub const SBUS_START: u8 = 0x0F;

/// Nominal SBUS channel endpoints (Futaba): 172 = -100%, 992 = centre,
/// 1811 = +100%. Span is ~819 counts per 100%.
const SBUS_CENTRE: f32 = 992.0;
const SBUS_SPAN: f32 = 819.0;
const SBUS_MIN: f32 = 172.0;
const SBUS_RANGE: f32 = 1811.0 - 172.0; // 1639 counts, idle..full throttle

/// The 16 decoded SBUS channels plus the receiver status flags. Each channel is
/// an 11-bit count in `0..=2047`; `failsafe`/`frame_lost` are exposed for the
/// arbiter (the decode applies no policy of its own).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SbusChannels {
    /// 16 channels, each an 11-bit raw count (0..=2047).
    pub ch: [u16; 16],
    /// Receiver failsafe latched (lost the transmitter).
    pub failsafe: bool,
    /// At least one frame was lost since the last good frame.
    pub frame_lost: bool,
}

/// Decode a 25-byte SBUS frame into its 16 channels + status flags.
///
/// Returns `None` if the start marker is wrong (the only structural check SBUS
/// affords without a CRC). Total: never panics for ANY 25 bytes, and every
/// returned channel is guaranteed `<= 0x7FF` (11-bit) — proven by Kani (RC-K03).
pub fn decode_sbus(frame: &[u8; SBUS_FRAME_LEN]) -> Option<SbusChannels> {
    if frame[0] != SBUS_START {
        return None;
    }
    // 22 payload bytes (frame[1..23]) carry 16 * 11 = 176 bits, LSB-first.
    let ch = unpack_channels_11bit(&frame[1..23]);
    let flags = frame[23];
    Some(SbusChannels {
        ch,
        frame_lost: flags & 0x04 != 0,
        failsafe: flags & 0x08 != 0,
    })
}

/// Unpack 16 LSB-first 11-bit channels from a packed payload — the wire layout
/// SHARED by SBUS and CRSF RC_CHANNELS_PACKED (only the surrounding framing
/// differs). Every read is bounds-guarded, so the unpack is panic-free for ANY
/// payload length and every returned channel is guaranteed `<= 0x7FF` (proven
/// by Kani RC-K03 over SBUS and RC-K04 over CRSF).
fn unpack_channels_11bit(payload: &[u8]) -> [u16; 16] {
    let mut ch = [0u16; 16];
    let mut i = 0;
    while i < 16 {
        let bit = i * 11;
        let byte = bit / 8;
        let shift = (bit % 8) as u32;
        // The 11-bit field spans at most three payload bytes; guard every read
        // so the unpack cannot index out of bounds regardless of slice length.
        let b0 = *payload.get(byte).unwrap_or(&0) as u32;
        let b1 = *payload.get(byte + 1).unwrap_or(&0) as u32;
        let b2 = *payload.get(byte + 2).unwrap_or(&0) as u32;
        let word = b0 | (b1 << 8) | (b2 << 16);
        ch[i] = ((word >> shift) & 0x07FF) as u16;
        i += 1;
    }
    ch
}

/// Map an 11-bit RC channel count to a centred stick axis in [-1, 1], sanitised
/// through the same clamp the manual modes use. SBUS and CRSF share the
/// 172/992/1811 endpoint convention, so this maps both.
#[inline]
fn rc_axis(raw: u16) -> f32 {
    unit((raw as f32 - SBUS_CENTRE) / SBUS_SPAN)
}

/// Map an 11-bit throttle channel count to [0, 1] (172..1811 -> 0..1), sanitised.
#[inline]
fn rc_throttle(raw: u16) -> f32 {
    throttle01((raw as f32 - SBUS_MIN) / SBUS_RANGE)
}

/// Map 16 raw 11-bit channel counts to a normalised [`RcInput`] using AETR order
/// (the Pixhawk/PX4 default: ch0 roll, ch1 pitch, ch2 throttle, ch3 yaw). The
/// output is always bounded (roll/pitch/yaw in [-1, 1], throttle in [0, 1]) for
/// ANY channel counts — proptest-gated, since it feeds the verified manual modes.
fn channels_to_rc(ch: &[u16; 16]) -> RcInput {
    RcInput {
        roll: rc_axis(ch[0]),
        pitch: rc_axis(ch[1]),
        throttle: rc_throttle(ch[2]),
        yaw: rc_axis(ch[3]),
    }
}

/// Map decoded SBUS channels to a normalised [`RcInput`] (AETR order). Bounded
/// for any channel counts (RC-P02).
pub fn sbus_to_rc(s: &SbusChannels) -> RcInput {
    channels_to_rc(&s.ch)
}

// ---------------------------------------------------------------------------
// CRSF wire decode (RC-P03) — TBS Crossfire / ExpressLRS, the other dominant RC
// link. Same 11-bit channel packing + 172/992/1811 endpoints as SBUS (so the
// unpack + stick mapping are shared), but different framing: [addr][len][type]
// [22-byte payload][CRC-8]. Unlike SBUS, CRSF has a real CRC-8 (DVB-S2, poly
// 0xD5) integrity check; link status (failsafe/RSSI) rides a SEPARATE
// LinkStatistics frame, so the RC frame exposes no inline flags.
// ---------------------------------------------------------------------------

/// CRSF RC_CHANNELS_PACKED frame length: addr + len + type + 22 payload + crc.
pub const CRSF_RC_FRAME_LEN: usize = 26;
/// CRSF sync/destination address for the flight controller.
pub const CRSF_ADDR_FC: u8 = 0xC8;
/// CRSF frame type for the packed 16-channel RC payload.
pub const CRSF_TYPE_RC_CHANNELS: u8 = 0x16;
/// CRSF `len` byte for an RC frame: type(1) + payload(22) + crc(1) = 24.
const CRSF_RC_LEN_FIELD: u8 = 24;

/// 16 decoded CRSF channels (11-bit counts, 0..=2047). CRSF carries link status
/// (failsafe/RSSI/link-quality) in a SEPARATE LinkStatistics frame, so the RC
/// frame alone exposes no flags — the arbiter reads link health from that frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CrsfChannels {
    /// 16 channels, each an 11-bit raw count (0..=2047).
    pub ch: [u16; 16],
}

/// CRSF CRC-8 (DVB-S2: poly 0xD5, init 0x00, no reflection) over the given bytes
/// (type + payload). Bounded loop — Kani-tractable.
fn crsf_crc8(data: &[u8]) -> u8 {
    let mut crc = 0u8;
    for &b in data {
        crc ^= b;
        let mut bit = 0;
        while bit < 8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0xD5 } else { crc << 1 };
            bit += 1;
        }
    }
    crc
}

/// Decode a 26-byte CRSF RC_CHANNELS_PACKED frame into its 16 channels.
///
/// Returns `None` on a wrong address/length/type or a CRC-8 mismatch — CRSF's
/// integrity check, the structural guarantee SBUS lacks. Total: never panics for
/// ANY 26 bytes, and every returned channel is guaranteed `<= 0x7FF` (Kani
/// RC-K04).
pub fn decode_crsf_rc(frame: &[u8; CRSF_RC_FRAME_LEN]) -> Option<CrsfChannels> {
    if frame[0] != CRSF_ADDR_FC
        || frame[1] != CRSF_RC_LEN_FIELD
        || frame[2] != CRSF_TYPE_RC_CHANNELS
    {
        return None;
    }
    // CRC-8 over type + payload (bytes 2..25); compare with the trailing byte.
    if crsf_crc8(&frame[2..25]) != frame[25] {
        return None;
    }
    Some(CrsfChannels {
        ch: unpack_channels_11bit(&frame[3..25]),
    })
}

/// Map decoded CRSF channels to a normalised [`RcInput`] (AETR order, the shared
/// endpoints). Bounded for any channel counts (RC-P03).
pub fn crsf_to_rc(c: &CrsfChannels) -> RcInput {
    channels_to_rc(&c.ch)
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    fn rc(roll: f32, pitch: f32, yaw: f32, throttle: f32) -> RcInput {
        RcInput { roll, pitch, yaw, throttle }
    }

    #[test]
    fn centre_stick_auto_levels() {
        let c = stabilized(rc(0.0, 0.0, 0.0, 0.5), 0.5, 1.0);
        assert_eq!(c.roll_rad, 0.0);
        assert_eq!(c.pitch_rad, 0.0);
        assert_eq!(c.yaw_rate, 0.0);
        assert_eq!(c.thrust, 0.5);
    }

    #[test]
    fn full_stick_hits_the_limit() {
        let c = stabilized(rc(1.0, -1.0, 1.0, 1.0), 0.5, 2.0);
        assert!((c.roll_rad - 0.5).abs() < 1e-6);
        assert!((c.pitch_rad + 0.5).abs() < 1e-6);
        assert!((c.yaw_rate - 2.0).abs() < 1e-6);
    }

    #[test]
    fn over_range_and_nan_are_bounded() {
        // glitching receiver: out-of-range + NaN must clamp, not blow up.
        let c = stabilized(rc(5.0, f32::NAN, -9.0, 2.0), 0.4, 1.5);
        assert!((c.roll_rad - 0.4).abs() < 1e-6);
        assert_eq!(c.pitch_rad, 0.0); // NaN → centre
        assert!((c.yaw_rate + 1.5).abs() < 1e-6);
        assert_eq!(c.thrust, 1.0);
    }

    #[test]
    fn acro_rates_bounded() {
        let c = acro(rc(2.0, -2.0, 0.5, 0.7), 3.0);
        assert!((c.roll_rate - 3.0).abs() < 1e-6);
        assert!((c.pitch_rate + 3.0).abs() < 1e-6);
        assert!((c.yaw_rate - 1.5).abs() < 1e-6);
        assert_eq!(c.thrust, 0.7);
    }

    // --- SBUS wire decode (RC-P02) ---

    /// Pack 16 channel counts into a 25-byte SBUS frame (the inverse of
    /// decode_sbus) so tests can round-trip without a captured byte log.
    fn pack_sbus(ch: &[u16; 16], flags: u8) -> [u8; SBUS_FRAME_LEN] {
        let mut f = [0u8; SBUS_FRAME_LEN];
        f[0] = SBUS_START;
        let mut bit = 0usize;
        for &c in ch.iter() {
            let v = (c & 0x07FF) as u32;
            for b in 0..11 {
                if v & (1 << b) != 0 {
                    let pos = bit + b;
                    f[1 + pos / 8] |= 1 << (pos % 8);
                }
            }
            bit += 11;
        }
        f[23] = flags;
        f
    }

    #[test]
    fn sbus_rejects_bad_start_byte() {
        let mut f = pack_sbus(&[992; 16], 0);
        f[0] = 0x00;
        assert_eq!(decode_sbus(&f), None);
    }

    #[test]
    fn sbus_round_trips_channels() {
        // distinct per-channel values, all 11-bit
        let mut ch = [0u16; 16];
        for (i, c) in ch.iter_mut().enumerate() {
            *c = (i as u16 * 113 + 7) & 0x07FF;
        }
        let f = pack_sbus(&ch, 0);
        let s = decode_sbus(&f).expect("valid start byte");
        assert_eq!(s.ch, ch);
        assert!(!s.failsafe && !s.frame_lost);
    }

    #[test]
    fn sbus_flags_are_exposed_not_applied() {
        // failsafe + frame_lost set; channels must STILL reflect the wire bytes
        // (expose-only: the decode applies no policy).
        let ch = [1811u16; 16];
        let f = pack_sbus(&ch, 0x04 | 0x08);
        let s = decode_sbus(&f).unwrap();
        assert!(s.failsafe && s.frame_lost);
        assert_eq!(s.ch, ch); // not overridden to centre/idle
    }

    #[test]
    fn sbus_endpoints_map_to_stick_extremes() {
        // centre count -> level; min/max -> stick extremes; throttle idle/full.
        let ch = [992, 172, 1811, 172, /* rest */ 992, 992, 992, 992, 992, 992,
                  992, 992, 992, 992, 992, 992];
        let rc = sbus_to_rc(&decode_sbus(&pack_sbus(&ch, 0)).unwrap());
        assert!(rc.roll.abs() < 1e-3); // 992 -> centre
        assert!((rc.pitch + 1.0).abs() < 2e-2); // 172 -> -100%
        assert!((rc.throttle - 1.0).abs() < 1e-6); // 1811 -> full
        assert!(rc.yaw < -0.9); // 172 -> near -100%
    }

    // --- CRSF wire decode (RC-P03) ---

    /// Pack 16 channels into a valid 26-byte CRSF RC frame (correct CRC-8).
    fn pack_crsf(ch: &[u16; 16]) -> [u8; CRSF_RC_FRAME_LEN] {
        let mut f = [0u8; CRSF_RC_FRAME_LEN];
        f[0] = CRSF_ADDR_FC;
        f[1] = CRSF_RC_LEN_FIELD;
        f[2] = CRSF_TYPE_RC_CHANNELS;
        let mut bit = 0usize;
        for &c in ch.iter() {
            let v = (c & 0x07FF) as u32;
            for b in 0..11 {
                if v & (1 << b) != 0 {
                    let pos = bit + b;
                    f[3 + pos / 8] |= 1 << (pos % 8);
                }
            }
            bit += 11;
        }
        f[25] = crsf_crc8(&f[2..25]);
        f
    }

    #[test]
    fn crsf_rejects_bad_address() {
        let mut f = pack_crsf(&[992; 16]);
        f[0] = 0xEE; // transmitter-module address, not the FC
        assert_eq!(decode_crsf_rc(&f), None);
    }

    #[test]
    fn crsf_rejects_bad_type() {
        let mut f = pack_crsf(&[992; 16]);
        f[2] = 0x14; // LinkStatistics, not RC_CHANNELS_PACKED
        f[25] = {
            // recompute CRC so ONLY the type check (not CRC) rejects it
            let mut crc = 0u8;
            for &b in &f[2..25] {
                crc ^= b;
                for _ in 0..8 {
                    crc = if crc & 0x80 != 0 { (crc << 1) ^ 0xD5 } else { crc << 1 };
                }
            }
            crc
        };
        assert_eq!(decode_crsf_rc(&f), None);
    }

    #[test]
    fn crsf_rejects_bad_crc() {
        let mut f = pack_crsf(&[992; 16]);
        f[25] ^= 0xFF; // corrupt the CRC byte
        assert_eq!(decode_crsf_rc(&f), None);
    }

    #[test]
    fn crsf_round_trips_channels() {
        let mut ch = [0u16; 16];
        for (i, c) in ch.iter_mut().enumerate() {
            *c = (i as u16 * 97 + 13) & 0x07FF;
        }
        let s = decode_crsf_rc(&pack_crsf(&ch)).expect("valid frame");
        assert_eq!(s.ch, ch);
    }

    #[test]
    fn crsf_and_sbus_share_the_unpack() {
        // The SAME 11-bit channels packed into each protocol's frame decode to
        // identical channel counts — proving the shared unpack core.
        let mut ch = [0u16; 16];
        for (i, c) in ch.iter_mut().enumerate() {
            *c = (i as u16 * 113 + 7) & 0x07FF;
        }
        let from_sbus = decode_sbus(&pack_sbus(&ch, 0)).unwrap().ch;
        let from_crsf = decode_crsf_rc(&pack_crsf(&ch)).unwrap().ch;
        assert_eq!(from_sbus, from_crsf);
    }

    #[test]
    fn crsf_endpoints_map_to_stick_extremes() {
        let ch = [992, 172, 1811, 1811, 992, 992, 992, 992, 992, 992, 992, 992,
                  992, 992, 992, 992];
        let rc = crsf_to_rc(&decode_crsf_rc(&pack_crsf(&ch)).unwrap());
        assert!(rc.roll.abs() < 1e-3); // 992 -> centre
        assert!((rc.pitch + 1.0).abs() < 2e-2); // 172 -> -100%
        assert!((rc.throttle - 1.0).abs() < 1e-6); // 1811 -> full
        assert!(rc.yaw > 0.9); // 1811 -> near +100%
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// For ANY stick input + limits, the stabilized attitude command is
        /// bounded by the tilt/yaw limits and thrust in [0,1].
        #[test]
        fn stabilized_output_bounded(
            r in -10.0f32..10.0, p in -10.0f32..10.0, y in -10.0f32..10.0,
            t in -1.0f32..3.0, tilt in 0.0f32..1.5, yawr in 0.0f32..5.0,
        ) {
            let c = stabilized(RcInput { roll: r, pitch: p, yaw: y, throttle: t }, tilt, yawr);
            prop_assert!(c.roll_rad.abs() <= tilt + 1e-4);
            prop_assert!(c.pitch_rad.abs() <= tilt + 1e-4);
            prop_assert!(c.yaw_rate.abs() <= yawr + 1e-4);
            prop_assert!((0.0..=1.0).contains(&c.thrust));
        }

        /// Acro rates are bounded by max_rate for any stick input.
        #[test]
        fn acro_output_bounded(
            r in -10.0f32..10.0, p in -10.0f32..10.0, y in -10.0f32..10.0,
            t in -1.0f32..3.0, mr in 0.0f32..8.0,
        ) {
            let c = acro(RcInput { roll: r, pitch: p, yaw: y, throttle: t }, mr);
            prop_assert!(c.roll_rate.abs() <= mr + 1e-4);
            prop_assert!(c.pitch_rate.abs() <= mr + 1e-4);
            prop_assert!(c.yaw_rate.abs() <= mr + 1e-4);
            prop_assert!((0.0..=1.0).contains(&c.thrust));
        }

        /// RC-P02: the SBUS stick mapping is bounded for ANY 16 channel counts
        /// (incl. out-of-nominal-range raw values) — roll/pitch/yaw in [-1, 1],
        /// throttle in [0, 1]. The f32 floor under the Kani integer-unpack proof.
        #[test]
        fn sbus_to_rc_output_bounded(raw in proptest::array::uniform16(0u16..=0xFFFF)) {
            let s = SbusChannels { ch: raw, failsafe: false, frame_lost: false };
            let rc = sbus_to_rc(&s);
            prop_assert!((-1.0..=1.0).contains(&rc.roll));
            prop_assert!((-1.0..=1.0).contains(&rc.pitch));
            prop_assert!((-1.0..=1.0).contains(&rc.yaw));
            prop_assert!((0.0..=1.0).contains(&rc.throttle));
        }

        /// decode_sbus never panics for ANY 25 bytes, and every returned channel
        /// is an 11-bit count (the proptest companion to Kani RC-K03).
        #[test]
        fn sbus_decode_never_panics(bytes in proptest::array::uniform25(0u8..=0xFF)) {
            if let Some(s) = decode_sbus(&bytes) {
                for c in s.ch {
                    prop_assert!(c <= 0x07FF);
                }
            }
        }

        /// RC-P03: the CRSF stick mapping is bounded for ANY 16 channel counts.
        #[test]
        fn crsf_to_rc_output_bounded(raw in proptest::array::uniform16(0u16..=0xFFFF)) {
            let rc = crsf_to_rc(&CrsfChannels { ch: raw });
            prop_assert!((-1.0..=1.0).contains(&rc.roll));
            prop_assert!((-1.0..=1.0).contains(&rc.pitch));
            prop_assert!((-1.0..=1.0).contains(&rc.yaw));
            prop_assert!((0.0..=1.0).contains(&rc.throttle));
        }

        /// decode_crsf_rc never panics for ANY 26 bytes (incl. the CRC-8 loop),
        /// and every returned channel is 11-bit (proptest companion to RC-K04).
        #[test]
        fn crsf_decode_never_panics(bytes in proptest::array::uniform26(0u8..=0xFF)) {
            if let Some(c) = decode_crsf_rc(&bytes) {
                for v in c.ch {
                    prop_assert!(v <= 0x07FF);
                }
            }
        }
    }
}
