//! HITL harness — the host-side test driver.
//!
//! ## Boundary
//!
//! The harness sits between two verified subsystems and a hardware
//! bench. It does *not* itself touch RF or hardware; it is a state
//! machine that:
//!
//!   1. Asks the bench for the position the FC's navigation stack is
//!      currently reporting.
//!   2. Feeds that position into the formally verified
//!      [`relay_lc::engine::Geofence::check`].
//!   3. On rising edge, dispatches the RTL RTS through the formally
//!      verified [`relay_sc::engine::CommandStore`].
//!   4. Asserts that the latch (a) tripped at all during the window the
//!      spoof was active and (b) did so within an expected latency.
//!
//! Anything that actually drives RF, USB or serial lives behind a
//! [`HitlBench`] backend — see `stub.rs` (deterministic, always
//! available) and `hackrf.rs` (real hardware, requires `gps-sdr-sim` +
//! `hackrf_transfer` on `$PATH`).
//!
//! The harness is what an EASA evidence reviewer reads first: it is
//! the runnable contract between the spec ("geofence trips when the
//! true position leaves the fence") and the lab ("here is what we
//! observed when an RF spoofer pushed the FC outside its fence").

use relay_lc::engine::Geofence;
use relay_sc::engine::{CommandStore, RtsCommand};

/// What a HITL backend has to provide.
///
/// Implementations may block on hardware (USB ack, SDR transmit
/// pacing) — they own their own timing. The harness assumes nothing
/// beyond "after `step(dt)` returns, `position_cm()` reflects the
/// bench's view at `t + dt`".
pub trait HitlBench {
    /// Backend name for the verdict log.
    fn name(&self) -> &'static str;

    /// Advance the bench by `dt` seconds. May block on hardware.
    fn step(&mut self, dt: f32);

    /// Current bench-reported NED position in centimetres.
    fn position_cm(&self) -> (i32, i32, i32);

    /// `true` iff the RF spoofer is transmitting for this step.
    /// The harness uses this only for diagnostic correlation —
    /// nothing in the verified path depends on it.
    fn spoof_active(&self) -> bool;
}

/// Outcome of one HITL run — what an evidence reviewer reads.
#[derive(Debug, Clone, Copy)]
pub struct HitlVerdict {
    pub backend: &'static str,
    /// Total simulated/wall-clock seconds the harness ran.
    pub duration_s: f32,
    /// Number of `step()` ticks issued.
    pub steps: u32,
    /// Did the geofence latch trip during the run?
    pub latched: bool,
    /// Time (s) at which the latch tripped, if it did.
    pub latched_at_s: Option<f32>,
    /// Did the harness dispatch an RTL RTS via relay-sc?
    pub rtl_dispatched: bool,
    /// First step at which `spoof_active()` was observed `true`.
    pub spoof_first_seen_at_s: Option<f32>,
    /// `Some(reason)` iff the harness produced a fail-stop verdict.
    pub failure: Option<&'static str>,
}

impl HitlVerdict {
    pub fn pass(&self) -> bool {
        self.failure.is_none() && self.latched && self.rtl_dispatched
    }
}

/// Drive one scenario end-to-end.
///
/// Arguments:
///   - `bench`     — RF source (stub or HackRF)
///   - `fence`     — pre-configured Geofence (caller picks NED bounds)
///   - `sc`        — pre-loaded CommandStore with the RTL RTS
///   - `dt`        — time per step (s)
///   - `duration_s` — total scenario length (s)
///   - `rtl_rts_id` — the RTS id to fire on the rising edge
///   - `max_latch_latency_s` — fail-stop if no latch within this many
///     seconds *after* the spoof first goes active
pub fn run_scenario(
    bench: &mut dyn HitlBench,
    fence: &mut Geofence,
    sc: &mut CommandStore,
    dt: f32,
    duration_s: f32,
    rtl_rts_id: u32,
    max_latch_latency_s: f32,
) -> HitlVerdict {
    let mut t = 0.0_f32;
    let mut steps = 0u32;
    let mut latched_at_s: Option<f32> = None;
    let mut rtl_dispatched = false;
    let mut spoof_first_seen_at_s: Option<f32> = None;
    let mut failure: Option<&'static str> = None;

    while t < duration_s {
        bench.step(dt);
        if spoof_first_seen_at_s.is_none() && bench.spoof_active() {
            spoof_first_seen_at_s = Some(t);
        }
        let (n, e, d) = bench.position_cm();
        let rising = fence.check(n, e, d);
        if rising {
            latched_at_s = Some(t);
            // The same cFS-DNA RTL RTS the SITL fires — see
            // examples/falcon-sitl-hover for the in-loop equivalent.
            let ok = sc.start_rts(rtl_rts_id, t as u64);
            rtl_dispatched = ok;
            if !ok {
                failure = Some("relay-sc rejected start_rts (RTS not loaded?)");
            }
        }
        // Tick the command store so the RTS commands actually dispatch.
        let _ = sc.process_tick(t as u64);

        // Fail-stop on spoof-without-latch.
        if let (Some(spoof_t), None) = (spoof_first_seen_at_s, latched_at_s) {
            if t - spoof_t > max_latch_latency_s {
                failure = Some("spoof active but geofence did not latch within budget");
                break;
            }
        }

        t += dt;
        steps += 1;
    }

    HitlVerdict {
        backend: bench.name(),
        duration_s,
        steps,
        latched: latched_at_s.is_some(),
        latched_at_s,
        rtl_dispatched,
        spoof_first_seen_at_s,
        failure,
    }
}

/// Construct an RTL RTS sequence at the given id. Mirrors the SITL
/// setup — the RTL command code is the same constant the SITL uses.
pub fn load_rtl_rts(sc: &mut CommandStore, rts_id: u32, rtl_command_code: u16) {
    sc.load_rts_command(
        rts_id,
        RtsCommand {
            delay_sec: 0,
            command_code: rtl_command_code,
            payload_offset: 0,
            payload_len: 0,
        },
    );
}
