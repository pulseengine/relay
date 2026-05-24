//! HackRF backend — real RF transmission via `gps-sdr-sim` + `hackrf_transfer`.
//!
//! ## How this is supposed to work
//!
//! On a bench:
//!
//! ```text
//!   ┌──────────────┐  IQ samples  ┌────────────────┐    L1 C/A    ┌──────────┐
//!   │ gps-sdr-sim  │ ───────────▶ │ hackrf_transfer│ ───────────▶ │  GPS RX  │
//!   │  (synthesise │              │  (transmit on  │   (1575.42   │  on FC   │
//!   │   spoofed    │              │   1575.42 MHz) │     MHz)     │          │
//!   │   trajectory)│              │                │              │          │
//!   └──────────────┘              └────────────────┘              └────┬─────┘
//!                                                                      │
//!                                                                      │ NMEA / MAVLink
//!                                                                      ▼
//!                                                              ┌─────────────────┐
//!                                                              │  this harness   │
//!                                                              │ via serial/USB  │
//!                                                              └─────────────────┘
//! ```
//!
//! The harness drives the `gps-sdr-sim → hackrf_transfer` chain to
//! synthesize a trajectory that walks the spoofed position out of the
//! fence, then reads the FC's reported position back over a telemetry
//! link and asserts the latch trips.
//!
//! ## What this module actually delivers in software
//!
//! The IO surface to the real bench (GPS module, FC telemetry) is the
//! piece that needs hardware to test end-to-end. This module:
//!
//!   * builds the `gps-sdr-sim` and `hackrf_transfer` command lines
//!     the harness would invoke,
//!   * lets the test infrastructure observe what *would* be sent,
//!   * exposes `HackRfBench` that returns the *planned* trajectory
//!     when no hardware is connected (so the rest of the harness can
//!     be exercised at desk-time).
//!
//! On a real bench the user wires `read_fc_position_cm()` to whatever
//! telemetry source they have (MAVLink GLOBAL_POSITION_INT, NMEA GGA,
//! etc.). That single function is the one piece this crate cannot
//! deliver in software.
//!
//! Tools required on `$PATH` for live operation:
//!   * `gps-sdr-sim` — <https://github.com/osqzss/gps-sdr-sim>
//!   * `hackrf_transfer` — from `hackrf` package
//!
//! See `README.md` for the bench wiring and calibration recipe.

use crate::harness::HitlBench;

/// Configuration for an `hackrf_transfer` invocation.
pub struct HackRfConfig {
    /// Path to the IQ file `gps-sdr-sim` will produce.
    pub iq_path: String,
    /// Centre frequency in Hz. GPS L1 C/A is 1_575_420_000.
    pub freq_hz: u64,
    /// Sample rate in Hz. `gps-sdr-sim` defaults to 2_600_000.
    pub sample_rate_hz: u32,
    /// TX gain in dB (`hackrf_transfer -x`). Bench-calibrated.
    pub gain_db: u8,
}

impl HackRfConfig {
    pub fn l1_ca_default(iq_path: impl Into<String>) -> Self {
        HackRfConfig {
            iq_path: iq_path.into(),
            freq_hz: 1_575_420_000,
            sample_rate_hz: 2_600_000,
            gain_db: 0,
        }
    }

    /// The exact `hackrf_transfer` argv this config would invoke.
    /// Exposed so unit tests can pin the command-line shape without
    /// actually shelling out.
    pub fn hackrf_transfer_argv(&self) -> [String; 9] {
        [
            "hackrf_transfer".into(),
            "-t".into(), self.iq_path.clone(),
            "-f".into(), self.freq_hz.to_string(),
            "-s".into(), self.sample_rate_hz.to_string(),
            "-x".into(), self.gain_db.to_string(),
        ]
    }
}

/// Configuration for `gps-sdr-sim` — the planned spoofed trajectory.
pub struct GpsSdrSimConfig {
    /// Path to the RINEX nav file.
    pub nav_path: String,
    /// Output IQ path (matches `HackRfConfig::iq_path`).
    pub out_iq_path: String,
    /// Static spoofed coordinate — lat, lon, alt (m).
    pub spoof_lat_deg: f64,
    pub spoof_lon_deg: f64,
    pub spoof_alt_m: f64,
    /// Duration in seconds.
    pub duration_s: u32,
}

impl GpsSdrSimConfig {
    /// The `gps-sdr-sim` argv this config would invoke.
    pub fn argv(&self) -> [String; 9] {
        [
            "gps-sdr-sim".into(),
            "-e".into(), self.nav_path.clone(),
            "-l".into(), format!("{},{},{}", self.spoof_lat_deg, self.spoof_lon_deg, self.spoof_alt_m),
            "-o".into(), self.out_iq_path.clone(),
            "-d".into(), self.duration_s.to_string(),
        ]
    }
}

/// HackRF-backed HITL bench. Until a real telemetry source is wired,
/// `position_cm()` returns the spoofer's *target* coordinate translated
/// to local NED — i.e. the FC is assumed to track the spoofed signal.
/// On a real bench the user replaces this with telemetry from the FC.
pub struct HackRfBench {
    label: &'static str,
    spoof_start_s: f32,
    /// Pre-spoof NED position (cm).
    pre_n_cm: i32,
    pre_e_cm: i32,
    pre_d_cm: i32,
    /// Post-spoof NED target (cm) — where the spoofer wants the FC to think it is.
    spoof_n_cm: i32,
    spoof_e_cm: i32,
    spoof_d_cm: i32,
    t: f32,
}

impl HackRfBench {
    pub fn new(
        spoof_start_s: f32,
        pre_n_cm: i32,
        pre_e_cm: i32,
        pre_d_cm: i32,
        spoof_n_cm: i32,
        spoof_e_cm: i32,
        spoof_d_cm: i32,
    ) -> Self {
        HackRfBench {
            label: "hackrf",
            spoof_start_s,
            pre_n_cm,
            pre_e_cm,
            pre_d_cm,
            spoof_n_cm,
            spoof_e_cm,
            spoof_d_cm,
            t: 0.0,
        }
    }
}

impl HitlBench for HackRfBench {
    fn name(&self) -> &'static str { self.label }

    fn step(&mut self, dt: f32) {
        self.t += dt;
        // Real bench: drive hackrf_transfer + read FC telemetry here.
        // Off-bench (cargo test, CI): no-op — the trajectory is pre-planned.
    }

    fn position_cm(&self) -> (i32, i32, i32) {
        if self.t < self.spoof_start_s {
            (self.pre_n_cm, self.pre_e_cm, self.pre_d_cm)
        } else {
            (self.spoof_n_cm, self.spoof_e_cm, self.spoof_d_cm)
        }
    }

    fn spoof_active(&self) -> bool {
        self.t >= self.spoof_start_s
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hackrf_argv_shape_is_stable() {
        let cfg = HackRfConfig::l1_ca_default("/tmp/spoof.iq");
        let argv = cfg.hackrf_transfer_argv();
        assert_eq!(argv[0], "hackrf_transfer");
        assert_eq!(argv[1], "-t");
        assert_eq!(argv[2], "/tmp/spoof.iq");
        assert_eq!(argv[3], "-f");
        assert_eq!(argv[4], "1575420000");
    }

    #[test]
    fn gps_sdr_sim_argv_shape_is_stable() {
        let cfg = GpsSdrSimConfig {
            nav_path: "brdc0010.24n".into(),
            out_iq_path: "/tmp/spoof.iq".into(),
            spoof_lat_deg: 47.5023,
            spoof_lon_deg: 19.0401,
            spoof_alt_m: 120.0,
            duration_s: 30,
        };
        let argv = cfg.argv();
        assert_eq!(argv[0], "gps-sdr-sim");
        assert_eq!(argv[1], "-e");
        assert_eq!(argv[3], "-l");
        assert!(argv[4].starts_with("47.5023"));
    }
}
