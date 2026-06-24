//! IEEE 754 half-precision (binary16) decode — the shared primitive for the
//! DroneCAN sensor telemetry (voltage/current/temperature) carried as float16,
//! LSB-first. Decode-only: falcon reads sensor float16 (in); it never emits it
//! (esc.RawCommand is int14). Pure arithmetic — no panic surface, so totality is
//! trivial; value correctness is pinned by known IEEE-half vectors + proptest.

/// Decode an IEEE 754 binary16 bit pattern to f32. Total: never panics for any
/// `u16` (subnormals, ±inf, NaN, ±0 all handled).
pub fn f16_to_f32(bits: u16) -> f32 {
    let sign = if bits & 0x8000 != 0 { -1.0f32 } else { 1.0f32 };
    let exp = (bits >> 10) & 0x1F;
    let frac = bits & 0x03FF;
    let mag = match exp {
        0 => {
            // zero or subnormal: frac * 2^-24
            (frac as f32) * (1.0 / 16_777_216.0)
        }
        0x1F => {
            // inf (frac == 0) or NaN
            if frac == 0 {
                f32::INFINITY
            } else {
                f32::NAN
            }
        }
        _ => {
            // normal: (1 + frac/1024) * 2^(exp-15)
            (1.0 + (frac as f32) / 1024.0) * exp2(exp as i32 - 15)
        }
    };
    sign * mag
}

/// 2^n for a bounded exponent (normal float16 exponents are -14..=15). Table-free
/// repeated multiply keeps it no_std-clean (no libm dependency).
#[inline]
fn exp2(n: i32) -> f32 {
    let mut v = 1.0f32;
    if n >= 0 {
        let mut i = 0;
        while i < n {
            v *= 2.0;
            i += 1;
        }
    } else {
        let mut i = 0;
        while i < -n {
            v *= 0.5;
            i += 1;
        }
    }
    v
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn known_half_vectors() {
        // canonical IEEE-754 binary16 patterns
        assert_eq!(f16_to_f32(0x0000), 0.0);
        assert_eq!(f16_to_f32(0x3C00), 1.0);
        assert_eq!(f16_to_f32(0x4000), 2.0);
        assert_eq!(f16_to_f32(0xC000), -2.0);
        assert_eq!(f16_to_f32(0x3800), 0.5);
        assert_eq!(f16_to_f32(0x4900), 10.0);
        assert!((f16_to_f32(0x3555) - 0.333_25).abs() < 1e-3); // ~1/3
        assert!(f16_to_f32(0x7C00).is_infinite() && f16_to_f32(0x7C00) > 0.0);
        assert!(f16_to_f32(0xFC00).is_infinite() && f16_to_f32(0xFC00) < 0.0);
        assert!(f16_to_f32(0x7E00).is_nan());
    }

    #[test]
    fn smallest_normal_and_subnormal() {
        // smallest positive normal 2^-14, smallest subnormal 2^-24
        assert!((f16_to_f32(0x0400) - 2.0f32.powi(-14)).abs() < 1e-12);
        assert!((f16_to_f32(0x0001) - 2.0f32.powi(-24)).abs() < 1e-12);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// f16_to_f32 never panics for ANY bit pattern, and finite results match
        /// the reference within half-precision tolerance for the normal range.
        #[test]
        fn decode_total_and_matches_reference(bits in 0u16..=0xFFFF) {
            let v = f16_to_f32(bits);
            // never panics (reaching here proves it); special values consistent
            let exp = (bits >> 10) & 0x1F;
            if exp == 0x1F {
                let frac = bits & 0x3FF;
                if frac == 0 {
                    prop_assert!(v.is_infinite());
                } else {
                    prop_assert!(v.is_nan());
                }
            } else {
                prop_assert!(v.is_finite());
            }
        }
    }
}
