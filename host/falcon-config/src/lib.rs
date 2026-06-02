//! HOST-ONLY tuning configuration for the falcon cascade.
//!
//! # No-std / no-alloc carve-out (deliberate)
//!
//! This crate pulls in `serde`, `serde_json`, `std`, and `alloc`. It is a
//! **host-side tuning tool only** — for the bench / SITL, where fast JSON
//! parameter sweeps beat rebuild-per-tweak. **Flight and embedded code
//! MUST NOT depend on this crate.** They consume the plain
//! `no_std`/`no_alloc` config structs directly (`relay_iekf::IekfConfig`,
//! `relay_geo::GeoGains`, `relay_adrc::AdrcGains`) — as `const` defaults or
//! a `no_std` binary format (e.g. postcard) later. All external config
//! machinery is carved out here so the flight path stays clean.
//!
//! The flow: `falcon-quad.json` → [`FalconConfig`] (serde) → the plain
//! flight structs via the `to_*` converters.

use serde::{Deserialize, Serialize};

/// IEKF tuning. Mirrors `relay_iekf::IekfConfig` plus the per-update
/// measurement variances the bench passes to update_* (those are call
/// arguments, not part of IekfConfig).
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct IekfCfg {
    /// Per-axis attitude/gyro process noise [roll, pitch, yaw].
    pub q_gyro: [f32; 3],
    pub q_accel: f32,
    pub q_bias_gyro: f32,
    pub q_bias_accel: f32,
    /// Initial 1σ [att, vel, pos, gyro_bias, accel_bias].
    pub p0: [f32; 5],
    /// Position measurement variance (m²).
    pub pos_var: f32,
    /// Heading (compass) measurement variance (rad²).
    pub yaw_var: f32,
    /// Gravity/accel-tilt base measurement variance.
    pub grav_var: f32,
}

/// Geometric SE(3) attitude tuning. Mirrors `relay_geo::GeoGains`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct GeoCfg {
    pub k_r: [f32; 3],
    pub k_omega: [f32; 3],
    pub j: [f32; 3],
}

/// One ADRC axis. Mirrors `relay_adrc::AdrcGains`.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct AdrcCfg {
    pub omega_o: f32,
    pub omega_c: f32,
    pub b0: f32,
    /// Actuator time constant τ (s) modelled in the ESO (0 = none).
    #[serde(default)]
    pub tau: f32,
}

/// Which mixer policy the bench uses.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum MixerMode {
    /// Thrust-priority floor (MIX-P05): scales all torque uniformly.
    ThrustFloor,
    /// Sequential desaturation (MIX-P06): yaw sacrificed first.
    Priority,
    /// Airmode (MIX-P07): shift collective, preserve attitude authority.
    Airmode,
}

/// Yaw control mode for the geo-hover position-hold.
#[derive(Clone, Copy, Debug, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum YawMode {
    /// No yaw torque (rely on natural yaw stability + estimated heading).
    Off,
    /// ADRC regulates yaw RATE to 0 (no heading tracking).
    RateHold,
    /// Track the heading setpoint (geometric attitude → yaw rate).
    HeadingHold,
}

/// Bench-local position/altitude P-D + mixer tuning for geo-hover.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct PosCfg {
    pub kp_pos: f32,
    pub kd_vel: f32,
    pub a_cmd_max: f32,
    pub hover_thrust: f32,
    pub kp_alt: f32,
    pub kd_alt: f32,
    /// Mixer thrust floor.
    pub mixer_floor: f32,
    /// Outer→inner: use the ADRC inner loop (true) or direct geometric
    /// torque (false).
    pub use_adrc: bool,
    pub yaw_mode: YawMode,
    /// Inner-loop period (s). 0.001 = 1 kHz (Track A: the rate loop must
    /// run fast vs the 25 ms yaw actuator pole).
    #[serde(default = "default_loop_dt")]
    pub loop_dt: f32,
    /// Outer-loop (estimator updates + geometric/position) decimation: run
    /// the outer loop every Nth inner tick. 10 → 100 Hz outer at 1 kHz inner.
    #[serde(default = "default_outer_decim")]
    pub outer_decim: u32,
    /// Gyro low-pass cutoff (Hz) before the ADRC. 0 disables.
    #[serde(default = "default_gyro_lpf_hz")]
    pub gyro_lpf_hz: f32,
    /// Mixer policy (`mixer_floor` is the floor/idle).
    #[serde(default = "default_mixer_mode")]
    pub mixer_mode: MixerMode,
}

// Priority for now: airmode gives the yaw loop FULL authority, which the
// negative-b0 yaw WORKAROUND can't yet handle (it diverges). Airmode
// becomes the default once the yaw sign is properly fixed (Track A #4).
fn default_mixer_mode() -> MixerMode { MixerMode::Priority }

fn default_loop_dt() -> f32 { 0.001 }
fn default_outer_decim() -> u32 { 10 }
fn default_gyro_lpf_hz() -> f32 { 60.0 }

/// The whole falcon-quad tuning set.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub struct FalconConfig {
    pub iekf: IekfCfg,
    pub geo: GeoCfg,
    pub adrc: [AdrcCfg; 3],
    pub pos: PosCfg,
}

impl FalconConfig {
    /// Load from a JSON file (host-side; panics-free Result).
    pub fn from_json_path(path: &str) -> Result<Self, String> {
        let s = std::fs::read_to_string(path).map_err(|e| format!("read {path}: {e}"))?;
        serde_json::from_str(&s).map_err(|e| format!("parse {path}: {e}"))
    }

    /// Serialise to pretty JSON (for writing the default).
    pub fn to_json(&self) -> String {
        serde_json::to_string_pretty(self).unwrap_or_default()
    }

    // ── Converters to the plain no_std flight structs ──

    pub fn to_iekf_config(&self) -> relay_iekf::IekfConfig {
        relay_iekf::IekfConfig {
            q_gyro: self.iekf.q_gyro,
            q_accel: self.iekf.q_accel,
            q_bias_gyro: self.iekf.q_bias_gyro,
            q_bias_accel: self.iekf.q_bias_accel,
            p0: self.iekf.p0,
        }
    }

    pub fn to_geo_gains(&self) -> relay_geo::GeoGains {
        relay_geo::GeoGains { k_r: self.geo.k_r, k_omega: self.geo.k_omega, j: self.geo.j }
    }

    pub fn to_adrc(&self) -> relay_adrc::AdrcRate {
        let mk = |a: &AdrcCfg| relay_adrc::AdrcGains::with_tau(a.omega_o, a.omega_c, a.b0, a.tau);
        relay_adrc::AdrcRate::new([mk(&self.adrc[0]), mk(&self.adrc[1]), mk(&self.adrc[2])])
    }
}

impl Default for FalconConfig {
    /// Best-known falcon-quad tuning — the A/B-investigation result that
    /// achieves reliable position-hold (Track B: 6/6 PASS; reproduced
    /// 3/4+ here, ~0.3 m typical). See
    /// docs/research/v0.25-position-hold-AB-synthesis.md.
    fn default() -> Self {
        FalconConfig {
            iekf: IekfCfg {
                q_gyro: [1e-4, 1e-4, 1e-2],
                q_accel: 1e-2,
                q_bias_gyro: 1e-6,
                q_bias_accel: 1e-5,
                p0: [0.1, 0.5, 1.0, 0.01, 0.1],
                // Tight position measurement: the horizontal loop damps on
                // the IEKF velocity; loose pos_var lagged it → a ~6 s-period
                // circular limit cycle. 0.005 cut the lag.
                pos_var: 0.005,
                yaw_var: 0.0005,
                grav_var: 0.5,
            },
            geo: GeoCfg {
                k_r: [0.6, 0.6, 0.15],
                k_omega: [0.12, 0.12, 0.35],
                j: [0.0217, 0.0217, 0.04],
            },
            adrc: [
                AdrcCfg { omega_o: 40.0, omega_c: 12.0, b0: 30.0, tau: 0.0125 },
                AdrcCfg { omega_o: 40.0, omega_c: 12.0, b0: 30.0, tau: 0.0125 },
                // Yaw: ω_o high / ω_c low (control bw below the motor pole)
                // + actuator lag τ=0.025 in the ESO. b0 is NEGATIVE (−6),
                // and that is now *validated*, not a fudge: the frame-yaw
                // oracle reads RELIABLY OPPOSE (+0.15 N·m → −1.1 rad/s,
                // deterministic: −1.14/−1.04/−1.13). The yaw channel
                // (controller-convention → mixer → gz) genuinely has
                // NEGATIVE control effectiveness, so a negative b0 is the
                // physically correct value (ADRC: "robust to a wrong b0").
                // The equivalent representation (frame seam SIGN[2]=−1 with
                // b0=+6) is mathematically identical (u→−u, b0·u_act and the
                // body torque unchanged) but showed WORSE in limited gz runs
                // (0/6 vs 3/4) — unresolved (gz startup nondeterminism vs a
                // hidden asymmetry), so we keep the verified-good b0=−6.
                AdrcCfg { omega_o: 30.0, omega_c: 3.0, b0: -6.0, tau: 0.025 },
            ],
            pos: PosCfg {
                // Gentle horizontal gains: high gains excited the limit
                // cycle; a small kp centres the drone without exciting it.
                kp_pos: 0.08,
                kd_vel: 0.6,
                a_cmd_max: 1.0,
                // √-PWM hover point; the altitude loop has no integrator so
                // this must match gravity (0.52 sank, >0.60 climbed).
                hover_thrust: 0.555,
                kp_alt: 0.05,
                kd_alt: 0.30,
                // Low idle gives yaw headroom in the priority mixer.
                mixer_floor: 0.2,
                use_adrc: true,
                yaw_mode: YawMode::RateHold,
                loop_dt: default_loop_dt(),       // 1 kHz inner loop
                outer_decim: default_outer_decim(), // 100 Hz outer loop
                gyro_lpf_hz: default_gyro_lpf_hz(), // 60 Hz gyro LPF
                mixer_mode: default_mixer_mode(),   // priority
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_roundtrips_through_json() {
        let c = FalconConfig::default();
        let json = c.to_json();
        let back: FalconConfig = serde_json::from_str(&json).unwrap();
        assert_eq!(back.geo.k_r, c.geo.k_r);
        assert_eq!(back.pos.yaw_mode, c.pos.yaw_mode);
        assert_eq!(back.adrc[2].omega_c, c.adrc[2].omega_c);
    }

    #[test]
    fn converts_to_flight_structs() {
        let c = FalconConfig::default();
        let iekf = c.to_iekf_config();
        assert_eq!(iekf.q_gyro, [1e-4, 1e-4, 1e-2]);
        let geo = c.to_geo_gains();
        assert_eq!(geo.k_r[2], 0.15);
        let _adrc = c.to_adrc();
    }
}
