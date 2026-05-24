//! Stub HITL backend — deterministic in-process "RF spoofer".
//!
//! This is the backend everything below `cargo test` exercises. The
//! geometry is intentionally crude so the harness verdict is easy to
//! reason about:
//!
//! * Before `spoof_start_s` the bench reports a true in-fence position.
//! * At `spoof_start_s` the spoofer flips on and the reported position
//!   *snaps* to a pre-configured out-of-fence coordinate — i.e. the
//!   FC "believes" it is somewhere else. Real GPS spoofers behave
//!   exactly like this (the receiver locks onto the stronger fake
//!   signal and the position estimate jumps to wherever the spoofer
//!   broadcasts).
//!
//! Whatever the harness observes here, it would observe on real
//! hardware too — same `HitlBench` trait, same `run_scenario` driver,
//! same verdict shape. The only thing the stub *can't* exercise is
//! actual RF physics; that's what the `hackrf` backend is for.

use crate::harness::HitlBench;

pub struct StubBench {
    /// Pre-spoof NED position, cm.
    pre_n_cm: i32,
    pre_e_cm: i32,
    pre_d_cm: i32,
    /// Spoofed (post-`spoof_start_s`) NED position, cm.
    spoof_n_cm: i32,
    spoof_e_cm: i32,
    spoof_d_cm: i32,
    /// Time at which the spoof flips on.
    spoof_start_s: f32,
    /// Internal clock.
    t: f32,
}

impl StubBench {
    pub fn new(
        pre_n_cm: i32,
        pre_e_cm: i32,
        pre_d_cm: i32,
        spoof_n_cm: i32,
        spoof_e_cm: i32,
        spoof_d_cm: i32,
        spoof_start_s: f32,
    ) -> Self {
        StubBench {
            pre_n_cm, pre_e_cm, pre_d_cm,
            spoof_n_cm, spoof_e_cm, spoof_d_cm,
            spoof_start_s,
            t: 0.0,
        }
    }
}

impl HitlBench for StubBench {
    fn name(&self) -> &'static str { "stub" }

    fn step(&mut self, dt: f32) { self.t += dt; }

    fn position_cm(&self) -> (i32, i32, i32) {
        if self.t < self.spoof_start_s {
            (self.pre_n_cm, self.pre_e_cm, self.pre_d_cm)
        } else {
            (self.spoof_n_cm, self.spoof_e_cm, self.spoof_d_cm)
        }
    }

    fn spoof_active(&self) -> bool { self.t >= self.spoof_start_s }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::harness::{load_rtl_rts, run_scenario};
    use relay_lc::engine::Geofence;
    use relay_sc::engine::CommandStore;

    /// The stub-bench scenario should: trip the latch *after* the spoof
    /// becomes active (never before) and dispatch RTL within the
    /// latency budget. Mirrors the v0.10 SITL geofence scenario.
    #[test]
    fn stub_bench_trips_and_dispatches() {
        // 100 m × 100 m × 100 m fence centred on the origin.
        let mut fence = Geofence::new(-10_000, 10_000, -10_000, 10_000, -10_000, 10_000);
        let mut sc = CommandStore::new();
        load_rtl_rts(&mut sc, 0, 0xA17C); // same RTL_CMD_CODE as the SITL

        // Pre-spoof: hovering at origin, -5 m altitude.
        // Spoof: yanked to 200 m east — well outside the 100 m fence.
        let mut bench = StubBench::new(0, 0, -500, 0, 20_000, -500, 2.0);

        let v = run_scenario(
            &mut bench,
            &mut fence,
            &mut sc,
            0.01,   // 100 Hz tick
            5.0,    // 5-second scenario
            0,      // RTL RTS id
            1.0,    // must latch within 1 s of spoof going active
        );

        assert!(v.pass(), "verdict = {:?}", v);
        assert!(v.latched);
        assert!(v.rtl_dispatched);
        let latched_at = v.latched_at_s.unwrap();
        let spoof_at = v.spoof_first_seen_at_s.unwrap();
        assert!(latched_at >= spoof_at, "latch before spoof: {} < {}", latched_at, spoof_at);
        // One-tick latency since the spoof is a step-jump.
        assert!(latched_at - spoof_at < 0.05);
    }

    /// Negative-control: if the bench never moves outside the fence,
    /// the latch must not trip — guards against a harness that "passes"
    /// because the fence always trips.
    #[test]
    fn stub_bench_no_violation_no_latch() {
        let mut fence = Geofence::new(-10_000, 10_000, -10_000, 10_000, -10_000, 10_000);
        let mut sc = CommandStore::new();
        load_rtl_rts(&mut sc, 0, 0xA17C);

        // "Spoof on" but the spoofed coordinate is still inside the fence —
        // models a benign denial-of-service spoof that doesn't push the
        // vehicle out. The verified `check()` should stay silent.
        let mut bench = StubBench::new(0, 0, -500, 100, 200, -500, 2.0);

        let v = run_scenario(
            &mut bench,
            &mut fence,
            &mut sc,
            0.01,
            5.0,
            0,
            10.0,  // generous budget so we don't fail-stop on the missing latch
        );

        assert!(!v.latched);
        assert!(!v.rtl_dispatched);
        assert!(v.failure.is_none(), "harness fail-stopped on a benign run: {:?}", v.failure);
    }
}
