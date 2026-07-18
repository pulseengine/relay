//! Benewake TF02-Pro UART frame decode (RANGEDRV-P01, v1.121).
//!
//! The standard Benewake 9-byte data frame, cross-validated against TWO
//! independent open-source parsers (the external-reference rule for wire
//! decoders — self-round-trip proves robustness, not conformance):
//!
//! - ArduPilot `AP_RangeFinder_Benewake.cpp`: dist LE cm at bytes 2–3
//!   (`(linebuf[3] << 8) | linebuf[2]`), checksum = sum of bytes 0..7
//!   compared to byte 8.
//! - PX4 `tfmini_parser.cpp`: sync `'Y','Y'` (0x59 0x59), dist LE cm,
//!   strength at bytes 4–5, `cksm += parserbuf[i]` over the first 8
//!   bytes, 0xFFFF distance = invalid sentinel.
//! - Benewake TF02-Pro manual: strength < 60 or == 65535 ⇒ the distance
//!   is unreliable; device envelope 0.1–40 m.
//!
//! Decode is TOTAL over arbitrary bytes (Kani FR-K03/K04) and the quality
//! gate is part of the decoder: a low-strength or out-of-envelope return
//! NEVER reaches the caller as a range.

/// One decoded, quality-gated TF02-Pro measurement.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Tf02Sample {
    /// Gated distance, metres (within the 0.1–40 m device envelope).
    pub distance_m: f32,
    /// Signal strength (device units, 60..65534 after gating).
    pub strength: u16,
}

/// Why a syntactically valid frame was rejected by the quality gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Tf02Reject {
    /// Header or checksum wrong — not a frame (resync and move on).
    NotAFrame,
    /// The device's own invalid-measurement sentinel (dist == 0xFFFF).
    InvalidSentinel,
    /// Strength below 60 or the 65535 unreliable marker.
    LowStrength,
    /// Outside the 0.1–40 m device envelope (spurious return).
    OutOfEnvelope,
}

/// TF02-Pro frame length.
pub const TF02_FRAME_LEN: usize = 9;
/// Frame header byte (twice).
pub const TF02_HEADER: u8 = 0x59;
/// Minimum reliable strength (Benewake manual).
pub const TF02_STRENGTH_MIN: u16 = 60;
/// Device envelope, centimetres.
pub const TF02_DIST_MIN_CM: u16 = 10;
pub const TF02_DIST_MAX_CM: u16 = 4000;

/// Decode ONE 9-byte frame. Total: any byte pattern yields `Ok` or a
/// specific reject — never a panic, never a NaN/out-of-envelope range.
pub fn decode_tf02_frame(frame: &[u8; TF02_FRAME_LEN]) -> Result<Tf02Sample, Tf02Reject> {
    if frame[0] != TF02_HEADER || frame[1] != TF02_HEADER {
        return Err(Tf02Reject::NotAFrame);
    }
    let mut sum: u8 = 0;
    for b in frame.iter().take(TF02_FRAME_LEN - 1) {
        sum = sum.wrapping_add(*b);
    }
    if sum != frame[TF02_FRAME_LEN - 1] {
        return Err(Tf02Reject::NotAFrame);
    }
    let dist_cm = u16::from_le_bytes([frame[2], frame[3]]);
    let strength = u16::from_le_bytes([frame[4], frame[5]]);
    if dist_cm == 0xFFFF {
        return Err(Tf02Reject::InvalidSentinel);
    }
    if strength < TF02_STRENGTH_MIN || strength == 0xFFFF {
        return Err(Tf02Reject::LowStrength);
    }
    if !(TF02_DIST_MIN_CM..=TF02_DIST_MAX_CM).contains(&dist_cm) {
        return Err(Tf02Reject::OutOfEnvelope);
    }
    // Division, not ×0.01: the f32 quotient is correctly rounded, so the
    // envelope bounds are EXACT (10/100 == 0.1f32; 4000/100 == 40.0f32) —
    // Kani caught 10 × 0.01f32 = 0.099999994 escaping the envelope.
    Ok(Tf02Sample { distance_m: dist_cm as f32 / 100.0, strength })
}

/// Streaming resync scanner: find and decode the first valid frame in
/// `bytes`, returning `(result, bytes_consumed)`. On a valid or gated
/// frame, consumes through its end; with no decodable frame, consumes
/// up to the last possible header start so the caller can append more
/// bytes and retry. Total over arbitrary input.
pub fn scan_tf02(bytes: &[u8]) -> (Option<Result<Tf02Sample, Tf02Reject>>, usize) {
    let mut i = 0usize;
    while i + TF02_FRAME_LEN <= bytes.len() {
        if bytes[i] == TF02_HEADER && bytes[i + 1] == TF02_HEADER {
            let mut frame = [0u8; TF02_FRAME_LEN];
            frame.copy_from_slice(&bytes[i..i + TF02_FRAME_LEN]);
            let r = decode_tf02_frame(&frame);
            if r != Err(Tf02Reject::NotAFrame) {
                return (Some(r), i + TF02_FRAME_LEN);
            }
            // checksum-failed header: skip ONE byte (a real frame may
            // start inside the corrupt span), keep scanning.
        }
        i += 1;
    }
    (None, i)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a frame with the documented checksum (sum of bytes 0..7,
    /// low 8 bits) — the SAME arithmetic both external parsers quote.
    fn frame(dist_cm: u16, strength: u16, temp_raw: u16) -> [u8; 9] {
        let d = dist_cm.to_le_bytes();
        let s = strength.to_le_bytes();
        let t = temp_raw.to_le_bytes();
        let mut f = [0x59, 0x59, d[0], d[1], s[0], s[1], t[0], t[1], 0];
        f[8] = f[..8].iter().fold(0u8, |a, b| a.wrapping_add(*b));
        f
    }

    /// Conformance against the externally documented arithmetic:
    /// ArduPilot's example decode `(linebuf[3]<<8)|linebuf[2]` in cm.
    /// 5.56 m, strength 1080 (a mid-range concrete case worked by hand):
    /// dist 556 = 0x022C → bytes 2C 02; strength 1080 = 0x0438 → 38 04;
    /// checksum = low byte of 0x59+0x59+0x2C+0x02+0x38+0x04+0+0 = 0x11C → 0x1C.
    #[test]
    fn decodes_documented_frame_arithmetic() {
        let f = [0x59, 0x59, 0x2C, 0x02, 0x38, 0x04, 0x00, 0x00, 0x1C];
        assert_eq!(
            decode_tf02_frame(&f),
            Ok(Tf02Sample { distance_m: 5.56, strength: 1080 })
        );
        // and the constructor agrees with the hand-worked bytes:
        assert_eq!(frame(556, 1080, 0), f);
    }

    #[test]
    fn rejects_bad_checksum_and_header() {
        let mut f = frame(556, 1080, 0);
        f[8] ^= 0xFF;
        assert_eq!(decode_tf02_frame(&f), Err(Tf02Reject::NotAFrame));
        let mut g = frame(556, 1080, 0);
        g[0] = 0x58;
        assert_eq!(decode_tf02_frame(&g), Err(Tf02Reject::NotAFrame));
    }

    /// The gate: low strength, unreliable marker, sentinel distance, and
    /// out-of-envelope returns NEVER reach the caller as a range.
    #[test]
    fn quality_gate_rejects() {
        assert_eq!(decode_tf02_frame(&frame(556, 59, 0)), Err(Tf02Reject::LowStrength));
        assert_eq!(decode_tf02_frame(&frame(556, 0xFFFF, 0)), Err(Tf02Reject::LowStrength));
        assert_eq!(
            decode_tf02_frame(&frame(0xFFFF, 1080, 0)),
            Err(Tf02Reject::InvalidSentinel)
        );
        assert_eq!(decode_tf02_frame(&frame(5, 1080, 0)), Err(Tf02Reject::OutOfEnvelope));
        assert_eq!(decode_tf02_frame(&frame(4050, 1080, 0)), Err(Tf02Reject::OutOfEnvelope));
        // envelope boundaries are inclusive:
        assert!(decode_tf02_frame(&frame(10, 1080, 0)).is_ok());
        assert!(decode_tf02_frame(&frame(4000, 1080, 0)).is_ok());
    }

    /// Resync: garbage → frame → garbage; the scanner finds the frame and
    /// reports the right consumed count.
    #[test]
    fn scanner_resyncs_through_garbage() {
        let f = frame(200, 500, 0);
        let mut stream = [0u8; 25];
        stream[..7].copy_from_slice(&[0x00, 0x59, 0x12, 0xFF, 0x59, 0x00, 0xAB]);
        stream[7..16].copy_from_slice(&f);
        let (r, used) = scan_tf02(&stream);
        assert_eq!(r, Some(Ok(Tf02Sample { distance_m: 2.0, strength: 500 })));
        assert_eq!(used, 16);
        // no frame at all: consumed leaves a potential partial header tail.
        let (r, used) = scan_tf02(&stream[..7]);
        assert_eq!(r, None);
        assert!(used <= 7);
    }

    /// A frame straddling a corrupt double-header: the scanner must not
    /// get stuck (skip-one-byte resync) and still find the real frame.
    #[test]
    fn scanner_skips_false_header() {
        let f = frame(300, 800, 0);
        let mut stream = [0u8; 20];
        stream[0] = 0x59;
        stream[1] = 0x59; // false start, checksum will fail
        stream[2..11].copy_from_slice(&f);
        let (r, _used) = scan_tf02(&stream);
        assert_eq!(r, Some(Ok(Tf02Sample { distance_m: 3.0, strength: 800 })));
    }

    mod proptests {
        use super::super::*;
        use proptest::prelude::*;

        proptest! {
            /// Totality + gate soundness over arbitrary bytes: never a
            /// panic, and any accepted sample is inside the envelope with
            /// reliable strength.
            #[test]
            fn decode_total_and_gate_sound(bytes in proptest::array::uniform9(any::<u8>())) {
                if let Ok(s) = decode_tf02_frame(&bytes) {
                    prop_assert!(s.distance_m >= 0.10 && s.distance_m <= 40.0);
                    prop_assert!(s.strength >= 60 && s.strength != 0xFFFF);
                    prop_assert!(s.distance_m.is_finite());
                }
            }

            /// Scanner totality: consumed never exceeds input length.
            #[test]
            fn scan_total(bytes in proptest::collection::vec(any::<u8>(), 0..64)) {
                let (_, used) = scan_tf02(&bytes);
                prop_assert!(used <= bytes.len());
            }
        }
    }
}
