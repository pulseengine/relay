use super::*;
use relay_calib::CalParams;
use relay_fsm::{Event, Mode};

struct SplitMix64(u64);
impl SplitMix64 {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }
    fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * ((self.next_u64() >> 40) as f32 / (1u64 << 24) as f32)
    }
}
fn trial_rng(seed: u64, i: u32) -> SplitMix64 {
    let mut s = SplitMix64(seed ^ (i as u64).wrapping_mul(0x9E37_79B9_7F4A_7C15));
    let _ = s.next_u64();
    s
}

/// Feeds an IMU consistent with a body tilt of `tilt` rad about body-y, so
/// the IEKF's estimate reaches ~tilt and the runaway/wind detectors act on
/// it — exactly the point-tests' mechanism.
///
/// HOW THE TILT REACHES THE ESTIMATE (#452). The first version leaned the
/// gravity vector in the accelerometer and reported ZERO gyro: a vehicle
/// that is tilted without ever having rotated. That only worked while the
/// estimator read attitude off the accelerometer. It does not any more
/// while position aiding is fresh, and it was never how a vehicle tells an
/// IMU it is tilted: a hovering multirotor's accelerometer reads the thrust
/// axis (−g on body z) at ANY tilt, and the rotation shows up in the GYRO.
/// So this backend now ROTATES to the commanded tilt at a fixed rate and
/// reports that rate, then holds. The estimate gets there by integrating
/// the gyro, which is what happens on a real airframe.
struct TiltBackend {
    tilt: f32,
    pos: Vec3,
    /// Attitude actually reached so far (rad), ramped toward `tilt`.
    tilt_now: f32,
}
impl TiltBackend {
    /// Rotation rate while slewing to the commanded tilt (rad/s). A
    /// runaway's own rate; fast enough that 800 steps of 0.004 s reach any
    /// tilt in the campaigns, slow enough to stay a plausible body rate.
    const SLEW: f32 = 1.0;
}
impl FlightBackend for TiltBackend {
    fn read_imu(&mut self) -> ImuSample {
        let dt = 0.004;
        let err = self.tilt - self.tilt_now;
        let step = Self::SLEW * dt;
        let rate = if err > step {
            self.tilt_now += step;
            Self::SLEW
        } else if err < -step {
            self.tilt_now -= step;
            -Self::SLEW
        } else {
            self.tilt_now = self.tilt;
            err / dt
        };
        ImuSample {
            // A hovering vehicle's accelerometer reads the thrust axis
            // whatever its tilt — the tilt is in the gyro, not here.
            accel: [0.0, 0.0, -GRAVITY],
            gyro: [0.0, rate, 0.0],
        }
    }
    fn read_position(&mut self) -> Option<Vec3> {
        Some(self.pos)
    }
    fn read_mag(&mut self) -> Option<Vec3> {
        None
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        Some(16.0)
    }
    fn write_motors(&mut self, _: &[f32]) {}
    fn dt(&self) -> f32 {
        0.004
    }
}

/// Converge level, arm, take off → airborne (mode Takeoff).
fn airborne(sup: &mut FlightSupervisor, b: &mut TiltBackend) {
    b.tilt = 0.0;
    b.pos = [0.0, 0.0, 0.0];
    for _ in 0..1500 {
        sup.step(b);
    }
    sup.command(Event::Arm, true, true);
    sup.command(Event::RequestTakeoff, true, false);
}

/// **Runaway → terminate, three-way on tilt vs the runaway limit.** Half the
/// trials sit a sustained tilt ABOVE the limit (must terminate + latch); half
/// sit a tilt BELOW it (must NOT terminate — an RTL via the wind path is not
/// a false terminate). Both regimes randomised.
const RUNAWAY_SEED: u64 = 0x0FA1_C0DE_5AFE_0001;

#[test]
fn failsafe_runaway_terminate_campaign() {
    // no_std crate: no alloc ⇒ no Vec/String/format!; fail-fast with core
    // panic formatting (index + tilt + mode is the reproducer).
    let trials = 120u32; // deterministic tilt sweep: 60/regime densely covers the range (noise dispersion is the next slice)
    let (mut term_fires, mut no_false_term) = (0u32, 0u32);
    for i in 0..trials {
        let mut rng = trial_rng(RUNAWAY_SEED, i);
        let runaway = i % 2 == 0;
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 1.0e9, 2.0, 14.0);
        sup.set_calibration(CalParams {
            gyro_bias: [0.001, 0.0, 0.0],
            ..CalParams::identity()
        });
        let mut b = TiltBackend {
            tilt: 0.0,
            pos: [0.0, 0.0, 0.0],
            tilt_now: 0.0,
        };
        airborne(&mut sup, &mut b);
        assert_ne!(
            sup.mode(),
            Mode::Terminated,
            "trial {i}: terminated on level takeoff"
        );
        if runaway {
            b.tilt = rng.range(1.15, 1.50); // strictly > TILT_RUNAWAY_LIMIT
            for _ in 0..4000 {
                sup.step(&mut b);
            }
            assert_eq!(
                sup.mode(),
                Mode::Terminated,
                "trial {i}: tilt {:.3} did not terminate",
                b.tilt
            );
            // latched: return level, must STAY terminated.
            b.tilt = 0.0;
            for _ in 0..500 {
                sup.step(&mut b);
            }
            assert_eq!(
                sup.mode(),
                Mode::Terminated,
                "trial {i}: terminate did not latch"
            );
            term_fires += 1;
        } else {
            b.tilt = rng.range(0.0, 0.95); // strictly < TILT_RUNAWAY_LIMIT
            for _ in 0..4000 {
                sup.step(&mut b);
            }
            assert_ne!(
                sup.mode(),
                Mode::Terminated,
                "trial {i}: FALSE terminate at tilt {:.3}",
                b.tilt
            );
            no_false_term += 1;
        }
    }
    // Both regimes well-sampled (≈200 each) ⇒ the pass covers "terminates
    // when it must" AND "never false-terminates"; a one-sided pass trips this.
    assert!(
        term_fires > 40 && no_false_term > 40,
        "both regimes must be well-sampled ({term_fires}/{no_false_term})"
    );
}

/// **Wind → RTL, three-way on tilt in/out of the saturation band.** Half the
/// trials sit a sustained tilt INSIDE [0.30,0.70] away from home (the RTL
/// failsafe must latch); half sit a tilt BELOW the band (must NOT latch RTL).
const WIND_SEED: u64 = 0x0FA1_C0DE_5AFE_0002;

#[test]
fn failsafe_wind_rtl_campaign() {
    let trials = 120u32; // deterministic tilt sweep: 60/regime densely covers the range (noise dispersion is the next slice)
    let (mut rtl_fires, mut no_false_rtl) = (0u32, 0u32);
    for i in 0..trials {
        let mut rng = trial_rng(WIND_SEED, i);
        let windy = i % 2 == 0;
        let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 1.0e9, 2.0, 14.0);
        sup.set_calibration(CalParams {
            gyro_bias: [0.001, 0.0, 0.0],
            ..CalParams::identity()
        });
        let mut b = TiltBackend {
            tilt: 0.0,
            pos: [0.0, 0.0, 0.0],
            tilt_now: 0.0,
        };
        airborne(&mut sup, &mut b);
        if windy {
            b.tilt = rng.range(0.38, 0.62); // inside the band with margin
            b.pos = [rng.range(80.0, 200.0), 0.0, 0.0]; // away from home
            for _ in 0..800 {
                sup.step(&mut b);
            }
            assert!(
                sup.rtl_latched,
                "trial {i}: wind tilt {:.3} did NOT latch RTL (mode {:?})",
                b.tilt,
                sup.mode()
            );
            rtl_fires += 1;
        } else {
            b.tilt = rng.range(0.0, 0.20); // below the band
            for _ in 0..800 {
                sup.step(&mut b);
            }
            assert!(
                !sup.rtl_latched,
                "trial {i}: FALSE wind RTL at tilt {:.3}",
                b.tilt
            );
            no_false_rtl += 1;
        }
    }
    // Both regimes well-sampled ⇒ the pass covers "RTL latches when it must"
    // AND "never false-latches"; a one-sided pass trips this.
    assert!(
        rtl_fires > 40 && no_false_rtl > 40,
        "both regimes must be well-sampled ({rtl_fires}/{no_false_rtl})"
    );
}
