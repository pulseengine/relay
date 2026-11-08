//! The single-rotor-out FDI of [`CascadePartition`]: its tuning constants,
//! the rate-derived tick counts, the re-arm hook and the per-tick detector
//! (spin-up stall check, lag-modelled residual). Moved out of `lib.rs`; every
//! public item is re-exported at the crate root, so `falcon_core::FDI_*` and the
//! existing methods resolve unchanged.

use super::*;

/// Rotor-out FDI CUSUM level and per-step drift, matching the SITL-verified
/// fault-tolerance chain. Named so a re-armed detector is built identically
/// to the original (#609 finding 16).
pub(crate) const FDI_CUSUM_THRESHOLD: f32 = 0.5;
pub(crate) const FDI_CUSUM_DRIFT: f32 = 0.1;

/// The rotor-out FDI SPIN-UP GUARD, in seconds after every motors-off ->
/// flying edge (#609 finding 18, maintainer decision 2026-11-04 "T300+M50").
/// A fixed TIME, counted in ticks of the RESYNCED loop rate and rounded
/// ([`FlightCore::fdi_guard_steps`]), exactly like finding 7's warm-up.
///
/// It was 0.2 s (the shared [`FlightCore::warmup_steps`]). One ESC whose
/// output ramps up more slowly than the other three (a soft-start) leaves a
/// DIFFERENTIAL residual that finding 17's common-mode subtraction cannot
/// remove, and a 0.2 s guard released while it was still large: measured on
/// SimBackend, 48/60/76 of 192 (ramp x lag x rotor) spin-ups at
/// 250/500/1000 Hz isolated the slow, HEALTHY rotor. With this guard and the
/// lag model below: 4/8/12, every one of them a 300 ms ramp (the named
/// known limit in `tests/fdi_spinup_guard.rs`).
///
/// THE CEILING IS CLOSE. A rotor dead from the first tick is caught when
/// the guard releases, so the guard IS its detection latency: at 0.3 s it is
/// isolated in <= 0.3 s with tilt <= 0.33 rad; at 0.4 s the vehicle flips
/// before isolation. The margin to that ceiling is ~0.1 s on the GROUND
/// (SimBackend's ground contact holds the airframe; not checked on gz or
/// hardware); do not lengthen this without re-measuring the dead-at-spin-up
/// case. AIRBORNE the margin is NEGATIVE: a core started hovering with a
/// rotor dead from tick 1 shuts the 2 rad/s roll/pitch rate gate at
/// ~0.26 s, before 0.3 s releases the guard. For a FULLY stalled rotor
/// that is covered by the spin-up stall check ([`FDI_STALL_RPM_FRAC`]),
/// which isolates it in ~0.1 s inside the guard; a rotor that spins with
/// reduced effectiveness from an airborne start is not, and still waits
/// for this guard.
pub const FDI_SPINUP_GUARD_S: f32 = 0.3;

/// First-order time constant (s) of the ROTOR LAG MODEL the FDI residual is
/// computed against (#609 finding 18). The residual compares the achieved
/// effectiveness (ESC RPM / [`ESC_RPM_FULL`]) with the command passed
/// through `y += (u - y) * dt / tau` (forward Euler, `dt / tau` capped at 1),
/// instead of with the raw previous command: a healthy rotor tracking its
/// command with this lag then reads ~0 even while it is still catching up.
///
/// ASSUMPTION, NOT A MEASUREMENT. 50 ms is the lag the SimBackend sweep was
/// run at; it is not derived from the first vehicle's ESCs. Replace it from
/// real ESC data for the BOM's ESCs: the spin-UP and spin-DOWN time
/// constants (they differ; this model uses one), the ESC soft-start profile,
/// the unit-to-unit spread, and the RPM telemetry latency. A plant slower
/// than the model leaves a residual on every transient (removed by the
/// common-mode subtraction only when all four lag alike); a faster one
/// leaves the model behind instead. Not a configuration field on purpose:
/// it would be another integrator-facing seam change.
pub const FDI_ROTOR_LAG_TAU_S: f32 = 0.05;

/// SPIN-UP STALL CHECK (#609 finding 18, maintainer decision 2026-11-04
/// "(c)"): DURING the spin-up guard only, a rotor whose achieved
/// effectiveness (ESC RPM / [`ESC_RPM_FULL`]) stays below this fraction
/// while its command is above [`FDI_STALL_CMD_MIN`] for
/// [`FDI_STALL_WINDOW_S`] of consecutive ticks is isolated at once, without
/// waiting for the guard. A rotor dead from the first tick is then caught
/// in ~0.1 s (tilt ~0.05 rad) instead of 0.3 s, and an AIRBORNE start, whose
/// roll/pitch rate gate shuts at ~0.26 s, is caught before that (it never
/// was with the 0.3 s guard alone).
///
/// ASSUMPTION, NOT A MEASUREMENT, like [`FDI_ROTOR_LAG_TAU_S`]. 5 % / 20 % /
/// 100 ms were chosen from the SimBackend sweep: the longest run any
/// HEALTHY rotor spent below 5 % with its command above 20 % was 68 ms (a
/// 300 ms ESC soft-start behind an 80 ms lag), so 100 ms leaves 32 ms.
/// Replace them from BOM ESC startup data. THE RISK: a SENSORLESS ESC can
/// report zero RPM, or no RPM at all, while it aligns and starts
/// commutating, and that phase can outlast 100 ms at low throttle; a real
/// ESC doing so would be isolated as dead on a healthy spin-up.
///
/// WHAT IT CATCHES: only a FULLY stalled rotor (RPM ~ 0). A rotor that
/// spins with reduced effectiveness at spin-up is left to the residual and
/// still waits for the guard ([`FDI_SPINUP_GUARD_S`]).
///
/// MISSING RPM DOES NOT COUNT: a tick with no RPM telemetry (the seam's
/// `motor-rpm: none`) is not evidence of a stall. It RESETS every rotor's run
/// to zero, so only an unbroken window of present, near-zero readings can
/// isolate; a backend without RPM never trips this.
///
/// A SHARED STALL DOES NOT COUNT EITHER (review of v1.141, finding 2): a
/// rotor is isolated only while it is the only rotor stalled on that tick.
/// Two or more stalled at once (zero RPM on every rotor from a telemetry bus
/// that is up but not populated, or ESCs that have not commutated yet) is
/// not a single-rotor fault and isolates nothing; see the stall check in
/// `CascadePartition::step_fdi` for the rule and its justification.
pub const FDI_STALL_RPM_FRAC: f32 = 0.05;
/// The command a rotor must be above for its near-zero RPM to count as a
/// stall (#609 finding 18; see [`FDI_STALL_RPM_FRAC`]). Below it, a low RPM
/// is just a low command. Assumption pending BOM ESC startup data.
pub const FDI_STALL_CMD_MIN: f32 = 0.20;
/// The unbroken time a rotor must read stalled to be isolated (#609 finding
/// 18; see [`FDI_STALL_RPM_FRAC`]), rounded to ticks of the resynced rate.
/// Assumption pending BOM ESC startup data.
pub const FDI_STALL_WINDOW_S: f32 = 0.1;

// FDI tick-count helpers, shared by the fused core and both halves of a
// split (see `FlightCore::warmup_steps`).
impl FlightCore {
    /// The rotor-out FDI spin-up guard at `loop_hz`: [`FDI_SPINUP_GUARD_S`]
    /// of ticks, ROUNDED like [`Self::warmup_steps`] (1/(1/1000) is
    /// 999.99994 in f32), same >= 10 floor (#609 finding 18). ONE definition
    /// for the fused core and both halves of a split.
    pub fn fdi_guard_steps(loop_hz: f32) -> u32 {
        ((loop_hz * FDI_SPINUP_GUARD_S + 0.5) as u32).max(10)
    }

    /// The stall check's window at `loop_hz`: [`FDI_STALL_WINDOW_S`] of
    /// ticks, ROUNDED, at least 1 (#609 finding 18). One definition for the
    /// fused core and both halves of a split.
    pub fn fdi_stall_steps(loop_hz: f32) -> u32 {
        ((loop_hz * FDI_STALL_WINDOW_S + 0.5) as u32).max(1)
    }

    /// The rotor lag model's per-tick gain at `loop_hz`:
    /// `(1 / loop_hz) / FDI_ROTOR_LAG_TAU_S`, capped at 1 (#609 finding 18).
    /// Forward Euler, no libm on the flight path.
    pub fn fdi_lag_alpha(loop_hz: f32) -> f32 {
        ((1.0 / loop_hz) / FDI_ROTOR_LAG_TAU_S).min(1.0)
    }
}

impl CascadePartition {
    /// FDI observability (v1.115): `(rate2, tilt_cos, gate_open, resid[4])` from
    /// the last single-rotor-out detector evaluation. `gate_open` is the
    /// near-level/low-rate gate; when it stays false under sensor noise the
    /// detector never sees the residual — the FAULT-P03 failure mode.
    pub fn fdi_diag(&self) -> (f32, f32, bool, [f32; 4]) {
        self.dbg_fdi
    }
    /// RE-ARM the rotor-out FDI for a new spin-up (#609 finding 16): restart the
    /// spin-up guard, forget the previous flight's last motor command and the
    /// rotor lag model's state (the residual's reference, which nothing else
    /// resets on the ground) and clear
    /// the CUSUM accumulators. ONLY while no rotor is isolated: a rotor the FDI
    /// HAS isolated stays isolated, so the `MotorsHealthy` pre-arm row keeps
    /// refusing to arm a vehicle with a dead motor (#565) and the degraded law
    /// stays engaged. Called by the supervisor on every motors-off -> flying
    /// edge.
    pub fn rearm_fdi(&mut self) {
        if self.failed_motor.is_none() {
            self.fdi_spinup = 0;
            self.last_motor_cmd = [0.0; 4];
            self.fdi_lag_model = [0.0; 4];
            self.fdi_stall_run = [0; 4];
            self.fdi_stall_common = false;
            self.fdi = RotorFaultDetector::new(FDI_CUSUM_THRESHOLD, FDI_CUSUM_DRIFT);
        }
    }
    /// The rotor-out FDI's per-tick work, the tail of [`Self::step`]: called
    /// once per control tick, after the allocator and before the command is
    /// recorded, with that tick's estimate, filtered gyro and RPM telemetry.
    pub(super) fn step_fdi(&mut self, est: &NavState, gyro_f: Vec3, rpm_now: Option<[i32; 4]>) {
        self.fdi_spinup = self.fdi_spinup.saturating_add(1);
        // ROTOR LAG MODEL (#609 finding 18): advance the first-order model of
        // what a healthy rotor reports now, driven by the command that
        // produced this tick's telemetry (`last_motor_cmd`, the v1.115
        // alignment). Every tick, gate open or shut, so it never goes stale.
        let mut k = 0;
        while k < 4 {
            self.fdi_lag_model[k] +=
                (self.last_motor_cmd[k] - self.fdi_lag_model[k]) * self.fdi_lag_alpha;
            k += 1;
        }
        // ATTITUDE/RATE GATE: a rotor-out residual is only DISTINGUISHABLE in
        // near-level, low-rate flight. During an attitude transient the
        // controller commands large asymmetric motors, so |commanded − achieved|
        // spikes on the throttled/saturated rotors and would false-trip the CUSUM
        // — you cannot diagnose a dead rotor while the airframe is tumbling. Only
        // run the detector when the vehicle is roughly level and not spinning
        // (v1.113 — caught a phantom rotor-out during an attitude transient on gz).
        let tilt_cos = 1.0 - 2.0 * (est.q[1] * est.q[1] + est.q[2] * est.q[2]); // R[2][2]
        // ROLL/PITCH rate only — NOT yaw (v1.115, FAULT-P03). The gate exists to
        // avoid diagnosing a dead rotor mid-TUMBLE (a roll/pitch upset spikes
        // the residual on saturated rotors). But a single-rotor-out RELINQUISHES
        // yaw: the body then spins freely about its near-vertical axis at several
        // rad/s — normal, not tumbling. Including yaw here slammed the gate shut
        // for the rest of the flight, so any fault not caught in the brief window
        // before the spin built was NEVER caught (26/300 misses + 43 mis-isolations
        // under heavy sensor noise). Roll/pitch rate stays low through the spin,
        // keeping detection available while still blocking a real tumble.
        let rp_rate2 = gyro_f[0] * gyro_f[0] + gyro_f[1] * gyro_f[1];
        // Rate gate at 2 rad/s (was 1): the REAL gz plant hovers with an
        // attitude limit-cycle near 1 rad/s roll/pitch, which held this gate
        // chronically shut — the FDI then lost the race against the runaway
        // detector on a genuine rotor-out (gz, v1.117: Terminate fired at
        // kill+0.6 s before isolation ever happened). A true tumble is
        // > 3 rad/s, so 2 rad/s keeps the you-cannot-diagnose-mid-tumble
        // purpose; the false-trip protection is carried by the v1.115
        // command-ALIGNED residual, not this gate.
        let fdi_steady = tilt_cos > 0.90 && rp_rate2 < 4.0; // ≲26° tilt, ≲2 rad/s roll+pitch
        let mut dbg_resid = [0.0f32; 4];
        // ── SPIN-UP STALL CHECK (#609 finding 18) ── inside the guard only,
        // and deliberately NOT behind `fdi_steady`: an airborne start shuts
        // that gate before the guard releases. A rotor reading near-zero RPM
        // with its (previous, aligned) command above FDI_STALL_CMD_MIN for
        // FDI_STALL_WINDOW_S unbroken is isolated now. A tick WITHOUT RPM is
        // not evidence: it resets every run (see FDI_STALL_RPM_FRAC).
        //
        // COMMON-MODE REJECTION (review of v1.141, finding 2), the stall
        // path's analogue of the residual path's min-subtraction (#609
        // finding 17). All four runs are updated FIRST, then judged: the
        // loop used to isolate inside the update, so when every rotor read
        // 0 RPM (a telemetry bus up but not yet populated, bidirectional
        // DShot before the ESCs commutate) rotor 0, checked first, was
        // isolated at the window, permanently, and the `MotorsHealthy`
        // pre-arm row then refused every re-arm.
        //
        // THE RULE: a rotor is isolated only while it is the ONLY rotor
        // stalled on this tick. Two or more stalled at once is not a
        // single-rotor fault, and why ">= 2" rather than "all four":
        //  - a multirotor with two dead rotors is not recoverable by the
        //    single-rotor reallocation this isolation engages, so refusing
        //    to isolate one of them loses nothing, while isolating a rotor
        //    on a telemetry fault latches a healthy vehicle as dead;
        //  - "stalled NOW", not "has filled the window": runs start only
        //    once a rotor's command passes FDI_STALL_CMD_MIN, so a shared
        //    stall can be staggered by a few ticks, and the first rotor
        //    would then fill its window alone (and three ESCs commutating
        //    while one is still silent is the same pattern).
        // A real dead rotor's healthy siblings read near zero only for the
        // first few ms of the spin-up (the rotor lag; a 100 ms soft-start
        // for longer), so its own run keeps counting through that co-stall
        // and it is isolated at the window IF every sibling has left the
        // stall band before the window fills
        // (tests/fdi_spinup_guard.rs
        // `a_stalled_rotor_beside_a_slow_healthy_rotor_is_still_isolated_in_the_window`).
        // But a window that FILLS while shared is discarded: from then on
        // every co-stalled tick resets the runs (`fdi_stall_common`), so a
        // rotor left silent when the others recover must earn a fresh
        // window on its own.
        //
        // KNOWN LIMIT (c), a DOUBLE fault (review of v1.140, defect C): a
        // dead rotor whose healthy SIBLING's RPM telemetry stays at 0 past
        // the window (an unpopulated bus slot, a dead telemetry wire) is
        // co-stalled on every tick, so this check never isolates it. Started
        // AIRBORNE it is never isolated at all: the residual path is shut by
        // the rate gate once the guard releases, and the vehicle falls (every
        // dead/silent pair, 250 Hz and 1 kHz; pinned by tests/fdi_spinup_guard.rs
        // `known_limit_c_dead_rotor_beside_a_silent_sibling_is_not_isolated_airborne`).
        // On a ground spin-up the isolation is deferred to the guard's
        // release and the residual path (measured, SimBackend, 50 ms lag:
        // isolated on the guard's own tick, 0.3 s, at 0.26 rad of tilt,
        // peak 1.02 rad, the supervisor already in Land). v1.143 (#638): disambiguate co-stalled
        // rotors by the body-rate / attitude response, which a dead rotor
        // produces and a dead telemetry channel does not.
        if self.failed_motor.is_none() && self.fdi_spinup < self.fdi_guard_steps {
            match rpm_now {
                Some(rpm) => {
                    let mut stalled = 0u32;
                    let mut longest = 0u32;
                    let mut i = 0;
                    while i < 4 {
                        let achieved = (rpm[i] as f32 / ESC_RPM_FULL).clamp(0.0, 2.0);
                        if achieved < FDI_STALL_RPM_FRAC
                            && self.last_motor_cmd[i] > FDI_STALL_CMD_MIN
                        {
                            self.fdi_stall_run[i] = self.fdi_stall_run[i].saturating_add(1);
                            stalled += 1;
                            if self.fdi_stall_run[i] > longest {
                                longest = self.fdi_stall_run[i];
                            }
                        } else {
                            self.fdi_stall_run[i] = 0;
                        }
                        i += 1;
                    }
                    if stalled >= 2 {
                        if longest >= self.fdi_stall_steps {
                            self.fdi_stall_common = true;
                        }
                        if self.fdi_stall_common {
                            self.fdi_stall_run = [0; 4];
                        }
                    } else {
                        self.fdi_stall_common = false;
                        if stalled == 1 && longest >= self.fdi_stall_steps {
                            let mut j = 0;
                            while j < 4 {
                                if self.fdi_stall_run[j] > 0 {
                                    self.failed_motor = Some(j);
                                }
                                j += 1;
                            }
                        }
                    }
                }
                None => {
                    self.fdi_stall_run = [0; 4];
                    self.fdi_stall_common = false;
                }
            }
        }
        if self.failed_motor.is_none() && self.fdi_spinup >= self.fdi_guard_steps && fdi_steady {
            if let Some(rpm) = rpm_now {
                let mut resid = [0.0f32; 4];
                let mut i = 0;
                while i < 4 {
                    let achieved = (rpm[i] as f32 / ESC_RPM_FULL).clamp(0.0, 2.0);
                    // Compare achieved against the LAG-MODELLED command (#609
                    // finding 18): the previous command that produced it
                    // (v1.115 — the telemetry lags one tick, so the current
                    // command spikes every residual on an abrupt step), passed
                    // through FDI_ROTOR_LAG_TAU_S. A healthy rotor still
                    // catching up reads ~0; a dead one reads the model.
                    resid[i] = (self.fdi_lag_model[i] - achieved).abs();
                    i += 1;
                }
                // COMMON-MODE REJECTION (#609 finding 17): subtract what all
                // four rotors share. The command-aligned residual above assumes
                // ONE tick of telemetry lag; a real ESC/rotor lags by a
                // first-order time constant of tens of ms, so any abrupt
                // COLLECTIVE step leaves all four behind their commands by the
                // same amount. The supervised landing makes that step when
                // the velocity law engages (hover ~0.5 -> ~0.3): at 50 ms lag
                // every residual read 0.19, equal to 1e-6, the CUSUM crossed
                // on tick 7 and rotor 0 was isolated only because it is
                // checked first. A single rotor out is DIFFERENTIAL (one
                // residual rises, three stay near zero), so removing the
                // per-tick minimum leaves it intact: Loiter isolation stays
                // at 3 ticks; isolation at the landing step goes 4 -> 8-12
                // ticks (tests/fdi_common_mode.rs).
                //
                // KEPT with the lag model (#609 finding 18): the model is
                // one assumed time constant, so a fleet of ESCs lagging
                // faster or slower than it, all alike, still leaves a shared
                // residual on every collective step, and this removes it.
                let common = resid[0].min(resid[1]).min(resid[2]).min(resid[3]);
                for r in resid.iter_mut() {
                    *r -= common;
                }
                dbg_resid = resid;
                self.failed_motor = self.fdi.update(resid);
            }
        }
        self.dbg_fdi = (rp_rate2, tilt_cos, fdi_steady, dbg_resid);
    }
}
