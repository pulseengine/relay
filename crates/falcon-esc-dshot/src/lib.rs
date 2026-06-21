//! Falcon ESC + battery driver bodies (F7).
//!
//! Two output-side driver pieces the readiness dossier flagged as missing:
//!
//!  * **DShot** — the digital ESC throttle protocol. A DShot frame is 16 bits:
//!    an 11-bit throttle, a 1-bit telemetry request, and a 4-bit CRC. This
//!    builds the frame from the verified mixer's `[0,1]` motor command, with the
//!    throttle SATURATING into the valid DShot range (never wrapping), and
//!    computes the CRC4 exactly per the DShot spec.
//!  * **Battery** — maps a raw ADC reading to pack voltage through a divider and
//!    classifies it against the low / critical thresholds that drive the v1.41
//!    failsafe arbiter (relay-fsafe).
//!
//! no_std / no_alloc / `forbid(unsafe_code)`. Verified by protocol KAT-style
//! tests + Kani totality (any mixer command → an in-range frame; the throttle
//! map never panics).

#![no_std]
#![forbid(unsafe_code)]

/// DShot throttle command (0 = motor stop / disarmed; 48..=2047 = active range;
/// 1..=47 are reserved special commands this driver does not emit from a flight
/// command). 11-bit value.
pub const DSHOT_MIN: u16 = 48;
/// Maximum DShot throttle (11-bit).
pub const DSHOT_MAX: u16 = 2047;

/// Map a verified-mixer motor command in `[0, 1]` to a DShot throttle value,
/// SATURATING: `≤ 0` ⇒ 0 (stop), `≥ 1` ⇒ `DSHOT_MAX`, otherwise linearly into
/// `[DSHOT_MIN, DSHOT_MAX]`. NaN ⇒ 0 (fail safe to stopped). Total: the result
/// is always a valid 11-bit throttle, never wraps.
pub fn throttle_from_unit(cmd: f32) -> u16 {
    if !(cmd > 0.0) {
        // covers NaN and ≤ 0
        return 0;
    }
    if cmd >= 1.0 {
        return DSHOT_MAX;
    }
    let span = (DSHOT_MAX - DSHOT_MIN) as f32;
    let v = DSHOT_MIN as f32 + cmd * span;
    // v ∈ (DSHOT_MIN, DSHOT_MAX); round to nearest, clamp for f32 safety.
    let r = v + 0.5;
    if r >= DSHOT_MAX as f32 {
        DSHOT_MAX
    } else if r <= DSHOT_MIN as f32 {
        DSHOT_MIN
    } else {
        r as u16
    }
}

/// Build a 16-bit DShot frame: `throttle` (11-bit, masked), a telemetry-request
/// bit, and the 4-bit DShot CRC. `frame = (throttle<<1 | telem) << 4 | crc`.
pub fn dshot_frame(throttle: u16, telemetry: bool) -> u16 {
    let value = ((throttle & 0x07FF) << 1) | telemetry as u16; // 12-bit data
    let crc = (value ^ (value >> 4) ^ (value >> 8)) & 0x0F;
    (value << 4) | crc
}

/// Convenience: a flight-ready DShot frame straight from a mixer `[0,1]` command
/// (no telemetry request).
pub fn dshot_from_unit(cmd: f32) -> u16 {
    dshot_frame(throttle_from_unit(cmd), false)
}

/// Verify a received/own DShot frame's CRC4 (e.g. loopback / self-check).
pub fn dshot_crc_ok(frame: u16) -> bool {
    let value = frame >> 4;
    let crc = frame & 0x0F;
    crc == ((value ^ (value >> 4) ^ (value >> 8)) & 0x0F)
}

/// Battery state classified against thresholds (feeds relay-fsafe).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BatteryState {
    /// Above the low threshold.
    Ok,
    /// Below low, above critical — low-battery failsafe (→ RTL).
    Low,
    /// Below critical — critical-battery failsafe (→ Land).
    Critical,
}

/// Battery monitor: raw ADC → pack voltage → threshold classification.
#[derive(Clone, Copy, Debug)]
pub struct BatteryMonitor {
    /// Volts per ADC count (divider gain / full-scale).
    volts_per_count: f32,
    low_v: f32,
    critical_v: f32,
}

impl BatteryMonitor {
    /// `volts_per_count` calibrates the ADC + divider; `low_v`/`critical_v` are
    /// the failsafe thresholds (critical ≤ low). Degenerate args are sanitised.
    pub fn new(volts_per_count: f32, low_v: f32, critical_v: f32) -> Self {
        let vpc = if volts_per_count.is_finite() && volts_per_count > 0.0 { volts_per_count } else { 0.0 };
        let low = if low_v.is_finite() { low_v } else { 0.0 };
        let crit = if critical_v.is_finite() { critical_v.min(low) } else { 0.0 };
        Self { volts_per_count: vpc, low_v: low, critical_v: crit }
    }

    /// Pack voltage from a raw ADC count.
    pub fn voltage(&self, adc_count: u16) -> f32 {
        adc_count as f32 * self.volts_per_count
    }

    /// Classify a raw ADC reading against the thresholds.
    pub fn classify(&self, adc_count: u16) -> BatteryState {
        let v = self.voltage(adc_count);
        if v <= self.critical_v {
            BatteryState::Critical
        } else if v <= self.low_v {
            BatteryState::Low
        } else {
            BatteryState::Ok
        }
    }
}

/// A 4-motor DShot ESC bank over a **relay-hal [`PwmOut`](relay_hal::PwmOut)**
/// frame sink — the hardware path (relay#214 / jess#62). The frame *encoding*
/// ([`dshot_from_unit`], CRC4, throttle saturation) is the pure verified body in
/// this crate; this just maps the mixer's `[0,1]` commands to frames and emits
/// them. jess binds FlexIO+eDMA under `PwmOut`. Fallible: a sink error returns
/// `P::Error`.
pub struct DShotEsc<P> {
    pwm: P,
}

impl<P: relay_hal::PwmOut> DShotEsc<P> {
    /// Wrap a `PwmOut` actuator-frame sink.
    pub fn new(pwm: P) -> Self {
        Self { pwm }
    }

    /// Encode four mixer `[0,1]` commands as DShot frames and emit them — one
    /// frame per motor channel.
    pub async fn send(&mut self, motors: &[f32; 4]) -> Result<(), P::Error> {
        let frames = [
            dshot_from_unit(motors[0]),
            dshot_from_unit(motors[1]),
            dshot_from_unit(motors[2]),
            dshot_from_unit(motors[3]),
        ];
        self.pwm.send(&frames).await
    }

    /// Consume the bank, returning the `PwmOut` sink.
    pub fn release(self) -> P {
        self.pwm
    }
}

/// A battery monitor reading its raw per-rail count over a relay-hal
/// [`AdcIn`](relay_hal::AdcIn) sink — the hardware path (relay#214 / jess#62).
/// jess binds the i.MX RT1176 LPADC / INA228 under `AdcIn`; the voltage +
/// classification ([`BatteryMonitor`]) stays the pure verified body above the
/// seam. **Monitor-only** — power failover is hardware-OR'd (DD-014), so no
/// failover logic crosses this seam. Fallible: an ADC error returns `A::Error`.
pub struct BatteryAdc<A> {
    adc: A,
    channel: u8,
    monitor: BatteryMonitor,
}

impl<A: relay_hal::AdcIn> BatteryAdc<A> {
    /// Wrap an `AdcIn` reading rail `channel`, with the given monitor calibration.
    pub fn new(adc: A, channel: u8, monitor: BatteryMonitor) -> Self {
        Self { adc, channel, monitor }
    }

    /// Async read of the battery state: pull the raw count via `AdcIn`, then
    /// classify it with the same verified [`BatteryMonitor::classify`].
    pub async fn read_state(&mut self) -> Result<BatteryState, A::Error> {
        let count = self.adc.read(self.channel).await?;
        Ok(self.monitor.classify(count))
    }

    /// Async read of the battery voltage (volts).
    pub async fn read_voltage(&mut self) -> Result<f32, A::Error> {
        let count = self.adc.read(self.channel).await?;
        Ok(self.monitor.voltage(count))
    }

    /// Consume the monitor, returning the `AdcIn` sink.
    pub fn release(self) -> A {
        self.adc
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::*;

    /// Independent DShot CRC for cross-checking (computed a different way).
    fn ref_crc(value12: u16) -> u16 {
        let mut csum = 0u16;
        let mut v = value12;
        for _ in 0..3 {
            csum ^= v & 0x0F;
            v >>= 4;
        }
        csum & 0x0F
    }

    #[test]
    fn dshot_frame_crc_matches_reference() {
        for throttle in [0u16, 48, 1000, 1046, 2047] {
            for telem in [false, true] {
                let frame = dshot_frame(throttle, telem);
                let value = ((throttle & 0x7FF) << 1) | telem as u16;
                assert_eq!(frame >> 4, value, "data field");
                assert_eq!(frame & 0x0F, ref_crc(value), "CRC for throttle {throttle}");
                assert!(dshot_crc_ok(frame));
            }
        }
    }

    #[test]
    fn known_dshot_frame_value() {
        // throttle 1046, no telemetry: value = 1046<<1 = 2092 (0x82C);
        // crc = (0x82C ^ 0x82 ^ 0x8) & 0xF. 0x82C^0x082 = 0x8AE; ^0x008 = 0x8A6;
        // &0xF = 0x6. frame = 0x82C<<4 | 6 = 0x82C6.
        assert_eq!(dshot_frame(1046, false), 0x82C6);
    }

    #[test]
    fn throttle_map_saturates() {
        assert_eq!(throttle_from_unit(-0.5), 0); // ≤0 ⇒ stop
        assert_eq!(throttle_from_unit(0.0), 0);
        assert_eq!(throttle_from_unit(f32::NAN), 0); // fail safe
        assert_eq!(throttle_from_unit(2.0), DSHOT_MAX); // ≥1 ⇒ max, no wrap
        let mid = throttle_from_unit(0.5);
        assert!((DSHOT_MIN..=DSHOT_MAX).contains(&mid));
        // monotone: more command ⇒ not-less throttle.
        assert!(throttle_from_unit(0.75) >= throttle_from_unit(0.25));
    }

    #[test]
    fn battery_classifies_against_thresholds() {
        // 12-cell-ish: volts_per_count chosen so count 1000 ⇒ 16.0 V.
        let bm = BatteryMonitor::new(0.016, 14.0, 13.2);
        assert_eq!(bm.classify(1000), BatteryState::Ok); // 16.0 V
        assert_eq!(bm.classify(860), BatteryState::Low); // 13.76 V
        assert_eq!(bm.classify(800), BatteryState::Critical); // 12.8 V
    }

    // ── async actuator path (relay-hal PwmOut sink) ──────────────────────────

    /// A mock `PwmOut` recording the last frame batch it was sent (or failing).
    struct MockPwm {
        last: Option<[u16; 4]>,
        fail: bool,
    }

    #[derive(Debug, PartialEq)]
    struct MockPwmError;
    impl relay_hal::PwmOut for MockPwm {
        type Error = MockPwmError;
        async fn send(&mut self, frames: &[u16]) -> Result<(), MockPwmError> {
            if self.fail {
                return Err(MockPwmError);
            }
            let mut f = [0u16; 4];
            for (i, &x) in frames.iter().take(4).enumerate() {
                f[i] = x;
            }
            self.last = Some(f);
            Ok(())
        }
    }

    fn block_on<F: core::future::Future>(fut: F) -> F::Output {
        use core::task::{Context, Poll, Waker};
        let mut fut = core::pin::pin!(fut);
        let mut cx = Context::from_waker(Waker::noop());
        loop {
            if let Poll::Ready(v) = fut.as_mut().poll(&mut cx) {
                return v;
            }
        }
    }

    /// `DShotEsc::send` emits exactly the frames `dshot_from_unit` produces for
    /// the same commands — the actuator re-target keeps the verified encoding;
    /// only the transport (the PwmOut sink) differs.
    #[test]
    fn esc_send_emits_dshot_frames() {
        let motors = [0.0_f32, 0.25, 0.5, 1.0];
        let mut esc = DShotEsc::new(MockPwm { last: None, fail: false });
        block_on(esc.send(&motors)).unwrap();
        let sent = esc.release().last.expect("a frame batch was sent");
        let expected = [
            dshot_from_unit(0.0),
            dshot_from_unit(0.25),
            dshot_from_unit(0.5),
            dshot_from_unit(1.0),
        ];
        assert_eq!(sent, expected);
    }

    /// The async path is fallible: a sink error propagates as Err.
    #[test]
    fn esc_send_propagates_sink_error() {
        let mut esc = DShotEsc::new(MockPwm { last: None, fail: true });
        assert_eq!(block_on(esc.send(&[0.0; 4])), Err(MockPwmError));
    }

    // ── async battery path (relay-hal AdcIn) + inter-core carrier ────────────

    /// A mock `AdcIn` returning a fixed count (or failing).
    struct MockAdc {
        count: u16,
        fail: bool,
    }
    impl relay_hal::AdcIn for MockAdc {
        type Error = MockPwmError;
        async fn read(&mut self, _channel: u8) -> Result<u16, MockPwmError> {
            if self.fail {
                Err(MockPwmError)
            } else {
                Ok(self.count)
            }
        }
    }

    /// `BatteryAdc::read_state` over an `AdcIn` returning count C equals the sync
    /// `BatteryMonitor::classify(C)`, and `read_voltage` equals `voltage(C)` —
    /// the ADC re-target keeps the verified classify; only the transport differs.
    #[test]
    fn battery_adc_matches_sync_classify() {
        let monitor = BatteryMonitor::new(0.016, 14.0, 13.2);
        for count in [1000_u16, 860, 800, 0] {
            let mut bat = BatteryAdc::new(MockAdc { count, fail: false }, 3, monitor);
            assert_eq!(block_on(bat.read_state()).unwrap(), monitor.classify(count));
            let mut bat2 = BatteryAdc::new(MockAdc { count, fail: false }, 3, monitor);
            let v = block_on(bat2.read_voltage()).unwrap();
            assert!((v - monitor.voltage(count)).abs() < 1e-6);
        }
    }

    /// The async battery path is fallible: an ADC error propagates as Err.
    #[test]
    fn battery_adc_propagates_error() {
        let monitor = BatteryMonitor::new(0.016, 14.0, 13.2);
        let mut bat = BatteryAdc::new(MockAdc { count: 0, fail: true }, 3, monitor);
        assert_eq!(block_on(bat.read_state()), Err(MockPwmError));
    }

    /// relay#177 — a BatteryState (the HAL output) rides the relay-bus ring
    /// across the M4→M7 seam, byte-identical + FIFO.
    #[test]
    fn battery_state_rides_the_intercore_ring() {
        use relay_bus::{MessageTransport, SpscRing};
        let monitor = BatteryMonitor::new(0.016, 14.0, 13.2);
        let s0 = monitor.classify(1000); // Ok
        let s1 = monitor.classify(800); // Critical
        let mut ring: SpscRing<BatteryState, 2> = SpscRing::new();
        assert!(ring.push(s0));
        assert!(ring.push(s1));
        assert_eq!(ring.pop(), Some(s0));
        assert_eq!(ring.pop(), Some(s1));
        assert!(ring.pop().is_none());
    }
}
