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
use relay_mavlink::{CommandLong, MAV_CMD_NAV_RETURN_TO_LAUNCH};
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

    /// Iff `true`, `run_scenario` paces the loop in real time
    /// (sleeps `dt` between ticks) so external real-time IO
    /// (PX4-SITL UDP, HackRF) has wall-clock time to deliver
    /// frames. Defaults to `false` — preserves the "complete a
    /// 60-tick test in microseconds" contract for `StubBench`
    /// and `InMemoryFrameSource`-backed `MavlinkBench` tests.
    /// `UdpFrameSource`-backed `MavlinkBench` overrides to true.
    fn real_time(&self) -> bool { false }

    /// `true` iff the RF spoofer is transmitting for this step.
    /// The harness uses this only for diagnostic correlation —
    /// nothing in the verified path depends on it.
    fn spoof_active(&self) -> bool;
}

/// What the harness does with the safety command relay-sc dispatches.
///
/// On a real bench this is a MAVLink `COMMAND_LONG` writer that
/// pushes the command back to the flight controller (closing the
/// round-trip — falcon's RTL trip becomes a real PX4 RTL command);
/// on cargo-tests this is a `NullCommandSink` that just counts.
///
/// One method, takes the encoded MAVLink frame; returns ok/err.
/// Decoupled from the verified path so the safety logic doesn't
/// know about IO.
pub trait CommandSink {
    fn send_frame(&mut self, frame_bytes: &[u8]) -> Result<(), &'static str>;
    fn name(&self) -> &'static str;
}

/// Null sink — accepts frames silently, used when the harness is
/// not configured with a real sink (every existing test path).
pub struct NullCommandSink {
    pub frames_sent: u32,
}

impl NullCommandSink {
    pub fn new() -> Self { Self { frames_sent: 0 } }
}

impl CommandSink for NullCommandSink {
    fn send_frame(&mut self, _bytes: &[u8]) -> Result<(), &'static str> {
        self.frames_sent += 1;
        Ok(())
    }
    fn name(&self) -> &'static str { "null" }
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
    /// Did the harness push a COMMAND_LONG / RTL frame to the
    /// downstream CommandSink? (v0.14.2 round-trip evidence.)
    pub rtl_frame_sent: bool,
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

/// Build the MAVLink COMMAND_LONG frame bytes for the RTL command
/// the harness pushes back to the FC. The `target_system` /
/// `target_component` are PX4 defaults (1/1).
fn build_rtl_frame(seq: u8) -> Vec<u8> {
    use relay_mavlink::{
        encode_frame, FrameHeader, COMMAND_LONG_CRC_EXTRA, COMMAND_LONG_MSG_ID,
        COMMAND_LONG_PAYLOAD_LEN, HEADER_LEN, MAGIC_V2,
    };
    let cmd = CommandLong::rtl(1, 1);
    debug_assert_eq!(cmd.command, MAV_CMD_NAV_RETURN_TO_LAUNCH);
    let payload = cmd.encode_payload();
    let header = FrameHeader {
        magic: MAGIC_V2,
        payload_len: COMMAND_LONG_PAYLOAD_LEN as u8,
        incompat_flags: 0,
        compat_flags: 0,
        sequence: seq,
        system_id: 255,        // GCS-style sender id
        component_id: 190,
        message_id: COMMAND_LONG_MSG_ID,
    };
    let mut out = vec![0u8; HEADER_LEN + COMMAND_LONG_PAYLOAD_LEN + 2];
    let n = encode_frame(&header, &payload, COMMAND_LONG_CRC_EXTRA, &mut out)
        .expect("encode_frame should fit in pre-sized buffer");
    out.truncate(n);
    out
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
    sink: &mut dyn CommandSink,
    dt: f32,
    duration_s: f32,
    rtl_rts_id: u32,
    max_latch_latency_s: f32,
) -> HitlVerdict {
    let mut t = 0.0_f32;
    let mut steps = 0u32;
    let mut latched_at_s: Option<f32> = None;
    let mut rtl_dispatched = false;
    let mut rtl_frame_sent = false;
    let mut spoof_first_seen_at_s: Option<f32> = None;
    let mut failure: Option<&'static str> = None;
    let mut mavlink_seq: u8 = 0;

    // Real-time pacing: bench's UDP/RF sources need wall-clock
    // time between polls so the OS delivers frames. The default
    // `t += dt` loop completes 6 000 ticks in microseconds, way
    // ahead of any real source. Diagnosed 2026-05-25: with no
    // sleep, MavlinkBench against live PX4-SITL got 0 frames in
    // 30 s of wall time even though `nc` saw 327 packets/s on
    // the same port.
    let pace_real_time = bench.real_time();
    let tick_period = std::time::Duration::from_secs_f32(dt);

    while t < duration_s {
        let tick_start = std::time::Instant::now();
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
            } else {
                // v0.14.2 round-trip: also encode and push the
                // MAVLink COMMAND_LONG so a real FC acts on it.
                // Sink failure is non-fatal (failure to push the
                // frame doesn't invalidate the latch + dispatch).
                let frame = build_rtl_frame(mavlink_seq);
                mavlink_seq = mavlink_seq.wrapping_add(1);
                match sink.send_frame(&frame) {
                    Ok(()) => rtl_frame_sent = true,
                    Err(reason) => {
                        // Record but don't fail-stop — the verified
                        // safety chain has already done its job.
                        if failure.is_none() {
                            failure = Some(reason);
                        }
                    }
                }
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

        // Real-time pacing — sleep what's left of this tick's
        // budget. Bench work that already exceeded the budget
        // produces no sleep (catches up by skipping).
        if pace_real_time {
            let used = tick_start.elapsed();
            if used < tick_period {
                std::thread::sleep(tick_period - used);
            }
        }
    }

    HitlVerdict {
        backend: bench.name(),
        duration_s,
        steps,
        latched: latched_at_s.is_some(),
        latched_at_s,
        rtl_dispatched,
        rtl_frame_sent,
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
