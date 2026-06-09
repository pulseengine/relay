//! Falcon ESC + battery driver bodies (F7).
//!
//! Two output-side driver pieces the readiness dossier flagged as missing:
//!
//!  * **DShot** — the digital ESC throttle protocol. A DShot frame is 16 bits:
//!    an 11-bit throttle, a 1-bit telemetry request, and a 4-bit CRC. This
//!    builds the frame from the verified mixer's `[0,1]` motor command, with the
//!    throttle SATURATING into the valid DShot range (never wrapping), and
//!    computes the CRC4 exactly per the DShot spec.
//!  * **Battery** — maps a raw ADC reading to pack voltage through a divider and
//!    classifies it against the low / critical thresholds that drive the v1.41
//!    failsafe arbiter (relay-fsafe).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`. Verified by protocol KAT-style
//! tests + Kani totality (any mixer command → an in-range frame; the throttle
//! map never panics).

#![no_std]
#![forbid(unsafe_code)]

/// DShot throttle command (0 = motor stop / disarmed; 48..=2047 = active range;
/// 1..=47 are reserved special commands this driver does not emit from a flight
/// command). 11-bit value.
pub const DSHOT_MIN: u16 = 48;
/// Maximum DShot throttle (11-bit).
pub const DSHOT_MAX: u16 = 2047;

/// Map a verified-mixer motor command in `[0, 1]` to a DShot throttle value,
/// SATURATING: `≤ 0` ⇒ 0 (stop), `≥ 1` ⇒ `DSHOT_MAX`, otherwise linearly into
/// `[DSHOT_MIN, DSHOT_MAX]`. NaN ⇒ 0 (fail safe to stopped). Total: the result
/// is always a valid 11-bit throttle, never wraps.
pub fn throttle_from_unit(cmd: f32) -> u16 {
    if !(cmd > 0.0) {
        // covers NaN and ≤ 0
        return 0;
    }
    if cmd >= 1.0 {
        return DSHOT_MAX;
    }
    let span = (DSHOT_MAX - DSHOT_MIN) as f32;
    let v = DSHOT_MIN as f32 + cmd * span;
    // v ∈ (DSHOT_MIN, DSHOT_MAX); round to nearest, clamp for f32 safety.
    let r = v + 0.5;
    if r >= DSHOT_MAX as f32 {
        DSHOT_MAX
    } else if r <= DSHOT_MIN as f32 {
        DSHOT_MIN
    } else {
        r as u16
    }
}

/// Build a 16-bit DShot frame: `throttle` (11-bit, masked), a telemetry-request
/// bit, and the 4-bit DShot CRC. `frame = (throttle<<1 | telem) << 4 | crc`.
pub fn dshot_frame(throttle: u16, telemetry: bool) -> u16 {
    let value = ((throttle & 0x07FF) << 1) | telemetry as u16; // 12-bit data
    let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    (value << 4) | crc
}

/// Convenience: a flight-ready DShot frame straight from a mixer `[0,1]` command
/// (no telemetry request).
pub fn dshot_from_unit(cmd: f32) -> u16 {
    dshot_frame(throttle_from_unit(cmd), false)
}

/// Verify a received/own DShot frame's CRC4 (e.g. loopback / self-check).
pub fn dshot_crc_ok(frame: u16) -> bool {
    let value = frame >> 4;
    let crc = frame & 0x0F;
    crc == ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F)
}

/// Battery state classified against thresholds (feeds relay-fsafe).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BatteryState {
    /// Above the low threshold.
    Ok,
    /// Below low, above critical — low-battery failsafe (→ RTL).
    Low,
    /// Below critical — critical-battery failsafe (→ Land).
    Critical,
}

/// Battery monitor: raw ADC → pack voltage → threshold classification.
#[derive(Clone, Copy, Debug)]
pub struct BatteryMonitor {
    /// Volts per ADC count (divider gain / full-scale).
    volts_per_count: f32,
    low_v: f32,
    critical_v: f32,
}

impl BatteryMonitor {
    /// `volts_per_count` calibrates the ADC + divider; `low_v`/`critical_v` are
    /// the failsafe thresholds (critical ≤ low). Degenerate args are sanitised.
    pub fn new(volts_per_count: f32, low_v: f32, critical_v: f32) -> Self {
        let vpc = if volts_per_count.is_finite() && volts_per_count > 0.0 { volts_per_count } else { 0.0 };
        let low = if low_v.is_finite() { low_v } else { 0.0 };
        let crit = if critical_v.is_finite() { critical_v.min(low) } else { 0.0 };
        Self { volts_per_count: vpc, low_v: low, critical_v: crit }
    }

    /// Pack voltage from a raw ADC count.
    pub fn voltage(&self, adc_count: u16) -> f32 {
        adc_count as f32 * self.volts_per_count
    }

    /// Classify a raw ADC reading against the thresholds.
    pub fn classify(&self, adc_count: u16) -> BatteryState {
        let v = self.voltage(adc_count);
        if v <= self.critical_v {
            BatteryState::Critical
        } else if v <= self.low_v {
            BatteryState::Low
        } else {
            BatteryState::Ok
        }
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    /// Independent DShot CRC for cross-checking (computed a different way).
    fn ref_crc(value12: u16) -> u16 {
        let mut csum = 0u16;
        let mut v = value12;
        for _ in 0..3 {
            csum ^= v & 0x0F;
            v >>= 4;
        }
        csum & 0x0F
    }

    #[test]
    fn dshot_frame_crc_matches_reference() {
        for throttle in [0u16, 48, 1000, 1046, 2047] {
            for telem in [false, true] {
                let frame = dshot_frame(throttle, telem);
                let value = ((throttle & 0x7FF) << 1) | telem as u16;
                assert_eq!(frame >> 4, value, "data field");
                assert_eq!(frame & 0x0F, ref_crc(value), "CRC for throttle {throttle}");
                assert!(dshot_crc_ok(frame));
            }
        }
    }

    #[test]
    fn known_dshot_frame_value() {
        // throttle 1046, no telemetry: value = 1046<<1 = 2092 (0x82C);
        // crc = (0x82C ^ 0x82 ^ 0x8) & 0xF. 0x82C^0x082 = 0x8AE; ^0x008 = 0x8A6;
        // &0xF = 0x6. frame = 0x82C<<4 | 6 = 0x82C6.
        assert_eq!(dshot_frame(1046, false), 0x82C6);
    }

    #[test]
    fn throttle_map_saturates() {
        assert_eq!(throttle_from_unit(-0.5), 0); // ≤0 ⇒ stop
        assert_eq!(throttle_from_unit(0.0), 0);
        assert_eq!(throttle_from_unit(f32::NAN), 0); // fail safe
        assert_eq!(throttle_from_unit(2.0), DSHOT_MAX); // ≥1 ⇒ max, no wrap
        let mid = throttle_from_unit(0.5);
        assert!((DSHOT_MIN..=DSHOT_MAX).contains(&mid));
        // monotone: more command ⇒ not-less throttle.
        assert!(throttle_from_unit(0.75) >= throttle_from_unit(0.25));
    }

    #[test]
    fn battery_classifies_against_thresholds() {
        // 12-cell-ish: volts_per_count chosen so count 1000 ⇒ 16.0 V.
        let bm = BatteryMonitor::new(0.016, 14.0, 13.2);
        assert_eq!(bm.classify(1000), BatteryState::Ok); // 16.0 V
        assert_eq!(bm.classify(860), BatteryState::Low); // 13.76 V
        assert_eq!(bm.classify(800), BatteryState::Critical); // 12.8 V
    }
}
