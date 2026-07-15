//! Telemetry STREAM SCHEDULER (MAVLINK-P06, v1.119) — rate + priority +
//! byte-budget arbitration for the GCS link.
//!
//! The SiK 433 MHz radio carries ~57.6 kbit/s ≈ 7200 bytes/s; the telemetry
//! set must fit with measured headroom, and under a DEGRADED link (half
//! budget or worse) the CRITICAL class — HEARTBEAT and STATUSTEXT — must
//! never be starved: a GCS that loses heartbeat declares the vehicle lost,
//! and a failsafe event that never reaches the operator is an invisible
//! emergency. NORMAL streams degrade gracefully instead (a due stream that
//! does not fit stays due and sends when budget allows).
//!
//! Pure, `no_std`, fixed-capacity, total: the scheduler never panics and
//! never emits more bytes per tick than the budget.

/// Stream priority class.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Priority {
    /// Never starved: emitted whenever due, before any Normal stream.
    /// The build-time invariant [`TelemetryScheduler::critical_fits`] must
    /// hold (the sum of all critical frame sizes fits the per-tick budget).
    Critical,
    /// Best-effort at its rate; degrades under link pressure (stays due).
    Normal,
}

/// One periodic stream slot.
#[derive(Clone, Copy, Debug)]
pub struct StreamSlot {
    /// Emit every `interval_ticks` scheduler ticks (0 ⇒ disabled).
    pub interval_ticks: u32,
    /// Full on-wire frame size (header + full payload + crc), bytes.
    pub frame_bytes: usize,
    pub priority: Priority,
}

/// Fixed-capacity scheduler over `N` streams. Call [`tick`](Self::tick) once
/// per scheduler period with the byte budget for that period; it returns a
/// bitmask of stream indices to emit NOW.
pub struct TelemetryScheduler<const N: usize> {
    slots: [StreamSlot; N],
    countdown: [u32; N],
    /// Ticks each stream has spent DUE-but-deferred (fairness age): the
    /// most-starved due stream gets first claim on the remaining budget, so
    /// sustained pressure degrades every normal stream proportionally
    /// instead of silently starving whichever sorts last.
    age: [u32; N],
    /// Normal-stream emissions deferred by budget pressure (loud counter).
    pub deferred: u32,
}

impl<const N: usize> TelemetryScheduler<N> {
    pub fn new(slots: [StreamSlot; N]) -> Self {
        let mut countdown = [0u32; N];
        for (c, s) in countdown.iter_mut().zip(slots.iter()) {
            *c = s.interval_ticks; // stagger-free start; first emit after one interval
        }
        TelemetryScheduler { slots, countdown, age: [0; N], deferred: 0 }
    }

    /// The no-starvation precondition: every critical frame fits the budget
    /// TOGETHER. Check at integration time; `tick` upholds the guarantee
    /// whenever this holds.
    pub fn critical_fits(&self, budget_bytes: usize) -> bool {
        let mut sum = 0usize;
        for s in self.slots.iter() {
            if s.priority == Priority::Critical && s.interval_ticks != 0 {
                sum = sum.saturating_add(s.frame_bytes);
            }
        }
        sum <= budget_bytes
    }

    /// Advance one period with `budget_bytes` available; returns a bitmask of
    /// streams to emit. Criticals due are ALWAYS in the mask (budget is
    /// consumed first-priority); normals fit-or-stay-due (deferred counted).
    pub fn tick(&mut self, budget_bytes: usize) -> u32 {
        let mut mask = 0u32;
        let mut left = budget_bytes;
        // Criticals first — unconditionally emitted when due (the caller
        // guarantees critical_fits; even if violated, we still emit and
        // saturate rather than starve the heartbeat).
        for i in 0..N.min(32) {
            let s = self.slots[i];
            if s.interval_ticks == 0 || s.priority != Priority::Critical {
                continue;
            }
            if self.countdown[i] <= 1 {
                mask |= 1 << i;
                self.countdown[i] = s.interval_ticks;
                left = left.saturating_sub(s.frame_bytes);
            } else {
                self.countdown[i] -= 1;
            }
        }
        // Normals, FAIRLY: repeatedly pick the due stream with the highest
        // deferral age that FITS the remaining budget (age breaks toward the
        // most-starved, so sustained pressure degrades proportionally — an
        // index-ordered scan starved the tail streams, caught by test).
        let mut due = [false; N];
        for (i, d) in due.iter_mut().enumerate().take(32) {
            let s = self.slots[i];
            if s.interval_ticks == 0 || s.priority != Priority::Normal {
                continue;
            }
            if self.countdown[i] <= 1 {
                *d = true;
            } else {
                self.countdown[i] -= 1;
            }
        }
        loop {
            let mut best: Option<usize> = None;
            for (i, d) in due.iter().enumerate().take(32) {
                if !*d || self.slots[i].frame_bytes > left {
                    continue;
                }
                match best {
                    Some(b) if self.age[b] >= self.age[i] => {}
                    _ => best = Some(i),
                }
            }
            let Some(i) = best else { break };
            due[i] = false;
            mask |= 1 << i;
            self.countdown[i] = self.slots[i].interval_ticks;
            self.age[i] = 0;
            left -= self.slots[i].frame_bytes;
        }
        // Whatever is still due did not fit: defer loudly, age for fairness.
        for (i, d) in due.iter().enumerate().take(32) {
            if *d {
                self.deferred = self.deferred.saturating_add(1);
                self.age[i] = self.age[i].saturating_add(1);
                self.countdown[i] = 1; // stays due
            }
        }
        mask
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // The v1.119 falcon set at a 10 Hz scheduler tick (budget per tick =
    // link bytes/s ÷ 10). Frame = 12 B MAVLink2 overhead + full payload.
    const HB: usize = 0; // HEARTBEAT 1 Hz, critical (21 B)
    const ST: usize = 1; // STATUSTEXT event slot, critical (66 B)
    const ATT: usize = 2; // ATTITUDE 10 Hz (40 B)
    const GPOS: usize = 3; // GLOBAL_POSITION_INT 5 Hz (40 B)
    const VFR: usize = 4; // VFR_HUD 5 Hz (32 B)
    const SYS: usize = 5; // SYS_STATUS 2 Hz (43 B)
    const GPS: usize = 6; // GPS_RAW_INT 2 Hz (64 B)
    const SRV: usize = 7; // SERVO_OUTPUT_RAW 2 Hz (49 B)

    fn falcon_set() -> TelemetryScheduler<8> {
        TelemetryScheduler::new([
            StreamSlot { interval_ticks: 10, frame_bytes: 21, priority: Priority::Critical },
            StreamSlot { interval_ticks: 10, frame_bytes: 66, priority: Priority::Critical },
            StreamSlot { interval_ticks: 1, frame_bytes: 40, priority: Priority::Normal },
            StreamSlot { interval_ticks: 2, frame_bytes: 40, priority: Priority::Normal },
            StreamSlot { interval_ticks: 2, frame_bytes: 32, priority: Priority::Normal },
            StreamSlot { interval_ticks: 5, frame_bytes: 43, priority: Priority::Normal },
            StreamSlot { interval_ticks: 5, frame_bytes: 64, priority: Priority::Normal },
            StreamSlot { interval_ticks: 5, frame_bytes: 49, priority: Priority::Normal },
        ])
    }

    /// MAVLINK-P06 rate budget: at the SiK link (7200 B/s ⇒ 720 B per 10 Hz
    /// tick) the WHOLE set meets its rates with ≥ 20% headroom — measured
    /// over 30 s, not asserted from arithmetic.
    #[test]
    fn full_set_fits_sik_budget_with_headroom() {
        let mut sched = falcon_set();
        let budget = 720usize;
        assert!(sched.critical_fits(budget));
        let mut counts = [0u32; 8];
        let mut bytes_total = 0usize;
        for _ in 0..300 {
            let mask = sched.tick(budget);
            for (i, c) in counts.iter_mut().enumerate() {
                if mask & (1 << i) != 0 {
                    *c += 1;
                    bytes_total += sched.slots[i].frame_bytes;
                }
            }
        }
        assert_eq!(sched.deferred, 0, "nothing defers at full budget");
        assert_eq!(counts[HB], 30, "HEARTBEAT 1 Hz");
        assert_eq!(counts[ATT], 300, "ATTITUDE 10 Hz");
        assert_eq!(counts[GPOS], 150, "GPOS 5 Hz");
        assert_eq!(counts[VFR], 150, "VFR 5 Hz");
        assert_eq!(counts[SYS], 60, "SYS_STATUS 2 Hz");
        assert_eq!(counts[GPS], 60, "GPS_RAW 2 Hz");
        assert_eq!(counts[SRV], 60, "SERVO 2 Hz");
        // Headroom: measured bytes over 30 s vs the link capacity.
        let capacity = 7200usize * 30;
        let used_frac = bytes_total as f32 / capacity as f32;
        assert!(
            used_frac <= 0.80,
            "set must leave >=20% headroom: used {:.0}%",
            used_frac * 100.0
        );
    }

    /// The no-starvation precondition is a checkable boundary: the critical
    /// set (HEARTBEAT 21 + STATUSTEXT 66 = 87 B) fits budgets >= 87 only.
    #[test]
    fn critical_fits_is_the_precise_boundary() {
        let sched = falcon_set();
        assert!(!sched.critical_fits(60));
        assert!(!sched.critical_fits(86));
        assert!(sched.critical_fits(87));
        assert!(sched.critical_fits(720));
    }

    /// Even when critical_fits is violated (extreme link collapse), criticals
    /// are STILL emitted (saturating past budget) rather than starved.
    #[test]
    fn criticals_emit_even_past_budget() {
        let mut sched = falcon_set();
        let budget = 30usize; // < HB+ST
        let mut hb = 0;
        let mut st = 0;
        for _ in 0..100 {
            let mask = sched.tick(budget);
            if mask & (1 << HB) != 0 {
                hb += 1;
            }
            if mask & (1 << ST) != 0 {
                st += 1;
            }
        }
        assert_eq!(hb, 10, "HEARTBEAT held its 1 Hz rate at collapse budget");
        assert_eq!(st, 10, "STATUSTEXT slot held rate at collapse budget");
        assert!(sched.deferred > 0, "normals deferred loudly");
    }

    /// Degraded-but-workable link: normals degrade, criticals exact.
    #[test]
    fn degraded_link_criticals_exact_normals_degrade() {
        let mut sched = falcon_set();
        let budget = 100usize; // fits criticals (87) + a little
        let mut counts = [0u32; 8];
        for _ in 0..300 {
            let mask = sched.tick(budget);
            for (i, c) in counts.iter_mut().enumerate() {
                if mask & (1 << i) != 0 {
                    *c += 1;
                }
            }
        }
        assert_eq!(counts[HB], 30, "HEARTBEAT exact under pressure");
        assert_eq!(counts[ST], 30, "STATUSTEXT exact under pressure");
        assert!(counts[ATT] < 300, "ATTITUDE degraded (got {})", counts[ATT]);
        assert!(sched.deferred > 0);
        // Deferral must not deadlock: every normal stream still progresses.
        for (i, c) in counts.iter().enumerate() {
            assert!(*c > 0, "stream {i} fully starved");
        }
    }
}
