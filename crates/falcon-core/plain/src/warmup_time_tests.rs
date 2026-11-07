extern crate std;
use super::*;

const LEVEL: [[f32; 3]; 3] = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// The supervisor's core: built at the nominal 1 kHz, stepped at `hz`.
fn hovering(hz: f32) -> (SimBackend, FlightCore) {
    let mut b = SimBackend::new(LEVEL, 1.0 / hz);
    b.rpm_telemetry = true;
    b.pos = [0.0, 0.0, -2.0];
    let mut core = FlightCore::new(SUPERVISOR_CORE_HOVER_THRUST, SUPERVISOR_CORE_LOOP_HZ);
    core.set_position([0.0, 0.0, -2.0]);
    (b, core)
}

/// Both halves derive the SAME count, ROUNDED: in f32, 1/(1/1000) is
/// 999.99994, and truncating 0.2 x that gives 199, not 200.
#[test]
fn the_warm_up_is_a_fixed_time_at_the_resynced_rate() {
    for (hz, want) in [(250.0f32, 50u32), (400.0, 80), (500.0, 100), (1000.0, 200)] {
        let (mut b, mut core) = hovering(hz);
        core.step(&mut b);
        assert_eq!(
            (core.est.fdi_warmup_steps, core.casc.fdi_warmup_steps),
            (want, want),
            "at {hz} Hz the warm-up must be 0.2 s = {want} ticks"
        );
    }
    assert_eq!(FlightCore::warmup_steps(1.0 / (1.0f32 / 1000.0)), 200);
    assert_eq!(FlightCore::warmup_steps(1.0 / (1.0f32 / 250.0)), 50);
}

/// #609 finding 18: the FDI spin-up guard is its own 0.3 s, ROUNDED at
/// the resynced rate, and the lag model's gain follows the rate too.
#[test]
fn the_fdi_guard_is_0_3_s_at_the_resynced_rate() {
    for (hz, want) in [(250.0f32, 75u32), (400.0, 120), (500.0, 150), (1000.0, 300)] {
        let (mut b, mut core) = hovering(hz);
        core.step(&mut b);
        assert_eq!(
            core.casc.fdi_guard_steps, want,
            "at {hz} Hz the guard must be 0.3 s"
        );
        let alpha = (1.0 / hz) / 0.05;
        assert!(
            (core.casc.fdi_lag_alpha - alpha).abs() < 1e-4,
            "at {hz} Hz the lag gain is {} not {alpha}",
            core.casc.fdi_lag_alpha
        );
    }
    assert_eq!(FlightCore::fdi_guard_steps(1.0 / (1.0f32 / 1000.0)), 300);
    assert_eq!(FlightCore::fdi_guard_steps(1.0 / (1.0f32 / 250.0)), 75);
}

/// An AIRBORNE start with a rotor dead from the first tick (#609 finding
/// 18), at 250 Hz and at 1 kHz: the spin-up STALL CHECK isolates the
/// correct rotor once the stall window has elapsed (not before it, and
/// within 2 ticks of it: the first tick has no previous command above
/// FDI_STALL_CMD_MIN yet), with the vehicle still near level.
///
/// This test used to be `the_fdi_engages_after_a_fixed_time_at_250_hz_
/// and_1_khz`, asserting isolation in [guard, guard + 5] ticks. Under the
/// 0.3 s guard that expectation was unreachable: the roll/pitch rate gate
/// shuts at ~0.26 s, the rotor was NEVER isolated and the vehicle
/// tumbled (left red on 3c7bbd8). The new design catches a FULLY stalled
/// rotor inside the guard, so the invariant is now tighter, not looser:
/// isolated at the stall window (0.1 s) instead of 0.2-0.3 s, AND bounded
/// tilt, which the old test never checked. The guard's own timing (the
/// residual path for a rotor that is weak but not stalled) moved to
/// tests/fdi_spinup_guard.rs `a_weak_rotor_is_left_to_the_guard`.
#[test]
fn an_airborne_rotor_dead_from_the_first_tick_is_caught_by_the_stall_check() {
    for hz in [250.0f32, 1000.0] {
        let window = (0.1 * hz + 0.5) as u32;
        let (mut b, mut core) = hovering(hz);
        b.fail_rotor(1);
        let mut isolated = None;
        let mut tilt = 0.0f32;
        for i in 1..=(2.0 * hz) as u32 {
            core.step(&mut b);
            tilt = tilt.max(b.tilt());
            if core.failed_motor().is_some() {
                isolated = Some(i);
                break;
            }
        }
        std::eprintln!(
            "{hz} Hz: stall window {window} ticks, isolated at tick {isolated:?}, tilt {tilt:.3} rad"
        );
        let i = isolated.unwrap_or_else(|| panic!("{hz} Hz: never isolated"));
        assert_eq!(core.failed_motor(), Some(1), "{hz} Hz: wrong rotor");
        assert!(
            i >= window && i <= window + 2,
            "{hz} Hz: isolated at tick {i}, the stall window is {window} ticks"
        );
        assert!(tilt < 0.1, "{hz} Hz: tilted {tilt:.3} rad before isolation");
    }
}

/// The stall window is 0.1 s ROUNDED at the resynced rate (#609 finding 18).
#[test]
fn the_stall_window_is_0_1_s_at_the_resynced_rate() {
    for (hz, want) in [(250.0f32, 25u32), (400.0, 40), (500.0, 50), (1000.0, 100)] {
        let (mut b, mut core) = hovering(hz);
        core.step(&mut b);
        assert_eq!(core.casc.fdi_stall_steps, want, "at {hz} Hz");
    }
    assert_eq!(FlightCore::fdi_stall_steps(1.0 / (1.0f32 / 1000.0)), 100);
}
/// REVIEW OF v1.141, FINDING 3: `resync_loop_rate` rescales the
/// IN-PROGRESS counters to the new rate, so they stay a measure of
/// elapsed TIME against thresholds that are fixed times. 1 kHz, rotor 1
/// dead from the first tick, 55 ticks, then ONE 2 ms frame (the core
/// resyncs to 500 Hz for it), then 1 ms frames again. On the late frame
/// every counter must read the elapsed time at 500 Hz (+-1 tick): the
/// spin-up guard counter and both step counts 57 ms, the stall run 56 ms
/// (it starts on tick 2: the first tick has no previous command). The
/// stalled rotor must not be isolated early by the late frame, and must
/// still be isolated once 0.1 s of stall has elapsed (+2 ticks).
///
/// FAILS ON 5020977: the counters stay in old-rate ticks against a
/// 50-tick window, and rotor 1 is isolated at 57 ms.
#[test]
fn a_late_frame_rescales_the_in_progress_counters() {
    let (mut b, mut core) = hovering(1000.0);
    b.fail_rotor(1);
    for _ in 0..55 {
        core.step(&mut b);
    }
    assert_eq!(core.failed_motor(), None, "isolated before the late frame");
    b.dt = 0.002;
    core.step(&mut b);
    b.dt = 0.001;
    let near = |got: u32, ms: f32| (got as f32 - ms * 0.5).abs() <= 1.0;
    let c = (
        core.casc.fdi_spinup,
        core.casc.step_count,
        core.est.step_count,
        core.casc.fdi_stall_run[1],
    );
    std::eprintln!("on the 2 ms frame: spinup/casc step/est step/stall run = {c:?}");
    assert!(
        near(c.0, 57.0) && near(c.1, 57.0) && near(c.2, 57.0) && near(c.3, 56.0),
        "on the 2 ms frame (500 Hz) the counters must read 57/57/57/56 ms = ~28 ticks, got {c:?}"
    );
    assert_eq!(
        core.failed_motor(),
        None,
        "the late frame isolated rotor 1 at 57 ms"
    );
    let mut elapsed_ms = 57u32;
    while core.failed_motor().is_none() && elapsed_ms < 400 {
        core.step(&mut b);
        elapsed_ms += 1;
    }
    std::eprintln!("rotor 1 isolated at {elapsed_ms} ms");
    assert_eq!(core.failed_motor(), Some(1));
    assert!(
        (100..=103).contains(&elapsed_ms),
        "isolated at {elapsed_ms} ms; the stall window is 100 ms"
    );
}
