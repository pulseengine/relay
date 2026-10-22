//! BATTERY-P03: "a single dropped sample cannot trip it either."
//!
//! WHY THIS FILE EXISTS. The v1.140 clean-room review (#497) found that this
//! clause had NO test: setting `BATT_SILENCE_S` from 1.0 to 0.0 left all 66
//! falcon-core tests green. A clause no test can falsify is an assertion, not a
//! verified property — and this one guards against a real failure mode: a
//! battery monitor that misses ONE CAN/I2C reading would otherwise latch an RTL
//! mid-mission.
//!
//! The rule under test (falcon-core): a pack that HAS reported is declared
//! silent only after `batt_missing_s > BATT_SILENCE_S` (declare-on-first-sight,
//! #475/#476). So one missing reading — one control tick — must not raise
//! `BATT_SILENT`, and a silence longer than `BATT_SILENCE_S` must.

use falcon_core::failsafe_flags::BATT_SILENT;
use falcon_core::{BATT_SILENCE_S, FlightBackend, FlightSupervisor, ImuSample};

const G: f32 = 9.80665;
const DT: f32 = 0.0025; // 400 Hz — one tick is far shorter than BATT_SILENCE_S

/// A backend whose battery reading follows a script: `Some(v)` except on the
/// ticks listed as dropped.
struct ScriptedPack {
    tick: u32,
    dropped_from: u32,
    dropped_ticks: u32,
}

impl FlightBackend for ScriptedPack {
    fn read_imu(&mut self) -> ImuSample {
        ImuSample {
            accel: [0.0, 0.0, -G],
            gyro: [0.0; 3],
        }
    }
    fn read_position(&mut self) -> Option<[f32; 3]> {
        Some([0.0, 0.0, 0.0])
    }
    fn read_mag(&mut self) -> Option<[f32; 3]> {
        None
    }
    fn write_motors(&mut self, _: &[f32]) {}
    fn dt(&self) -> f32 {
        DT
    }
    fn read_battery_v(&mut self) -> Option<f32> {
        let t = self.tick;
        self.tick += 1;
        let dropped = t >= self.dropped_from && t < self.dropped_from + self.dropped_ticks;
        if dropped { None } else { Some(16.8) }
    }
}

/// Step the supervisor through `n` ticks and report whether BATT_SILENT was
/// raised on ANY of them — a latch that blinked on for one tick still counts.
fn silent_ever(sup: &mut FlightSupervisor, b: &mut ScriptedPack, n: u32) -> bool {
    let mut seen = false;
    for _ in 0..n {
        sup.step(b);
        seen |= sup.failsafe_flags() & BATT_SILENT != 0;
    }
    seen
}

#[test]
fn a_single_dropped_battery_sample_does_not_raise_batt_silent() {
    // 400 ticks of a healthy pack (so `batt_seen` is true), ONE dropped
    // reading, then the pack reports again.
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 14.0);
    let mut b = ScriptedPack {
        tick: 0,
        dropped_from: 400,
        dropped_ticks: 1,
    };
    assert!(
        !silent_ever(&mut sup, &mut b, 800),
        "ONE dropped battery reading ({DT} s) raised BATT_SILENT; the pack is \
         only silent after BATT_SILENCE_S = {BATT_SILENCE_S} s of no readings"
    );
}

#[test]
fn a_silence_longer_than_batt_silence_s_does_raise_batt_silent() {
    // THE CONTROL. Without it, the test above passes for a supervisor that can
    // never raise BATT_SILENT at all. A silence comfortably past the threshold
    // must raise it — so the first test's "no" is attributable to the
    // threshold, not to a dead detector.
    let silence_ticks = ((BATT_SILENCE_S * 1.5) / DT) as u32 + 1;
    let mut sup = FlightSupervisor::new([0.0, 0.0, 0.0], 100.0, 2.0, 14.0);
    let mut b = ScriptedPack {
        tick: 0,
        dropped_from: 400,
        dropped_ticks: silence_ticks,
    };
    assert!(
        silent_ever(&mut sup, &mut b, 400 + silence_ticks),
        "a {:.2} s silence after the pack had reported did NOT raise BATT_SILENT",
        silence_ticks as f32 * DT
    );
}
