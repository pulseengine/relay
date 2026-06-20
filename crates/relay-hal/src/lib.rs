//! relay-hal — the `no_std` peripheral-abstraction seam.
//!
//! relay owns the hardware *abstraction*; jess binds it to the real silicon
//! (jess#62 / relay#214; jess DD-014/015/018). The driver bodies + the register
//! decode + the estimator's redundancy voting all stay *above* this seam, in
//! portable Kani-proven Rust; the host (jess's native TCB, or the meld-fused
//! wasm HAL component) provides only the transport, raw monitor counts, and the
//! enumeration of what is physically present.
//!
//! ## What the seam carries
//!
//! - **bus transport** — SPI / I2C, anchored on `embedded-hal-async`
//!   (`SpiDevice` / `I2c`) at the *transaction* level so zero register
//!   semantics cross the seam. (Wired once the crate-universe dep lands —
//!   relay#214; this v1 ships the three relay-owned pieces below.)
//! - [`PwmOut`] — async actuator-frame sink (DShot frame on T1/T2 via
//!   FlexIO+eDMA; duty on T3's failsafe PWM).
//! - [`AdcIn`] — async per-rail monitor read (INA228 V/I), **monitor-only**.
//! - [`Presence`] — the enumerated device set with per-device health: how
//!   board-variance and the voter's selectable set are expressed.
//!
//! ## Why these shapes (frozen design, jess#62)
//!
//! - **Async everywhere** (DD-018): jess is wasm-async on all three cores, so
//!   the transport traits are async.
//! - **Per-instance + enumerable, never multiplexed** (DD-014): each isolated
//!   bus is its own handle; board variance (DD-013) lives in the *enumerated
//!   set*, not in fixed trait arity.
//! - **Fallible = the health source**: a wedged isolated bus surfaces as a
//!   transport error, which the driver folds into the per-device [`Health`] the
//!   estimator votes on. Decode + voting stay in relay; this seam never votes.

#![no_std]
#![forbid(unsafe_code)]

#[cfg(kani)]
mod kani_proofs;

use core::future::Future;

/// Async actuator-frame sink.
///
/// The frame *encoding* (DShot bit pattern + CRC, throttle saturation into
/// `[48, 2047]`, or a raw duty value) stays in the driver crate
/// (`falcon-esc-dshot`); this seam transports already-encoded per-channel
/// frames. On T1/T2 the binding is FlexIO+eDMA; on T3 it degenerates to classic
/// failsafe duty-PWM (eFlexPWM).
pub trait PwmOut {
    /// Transport error (e.g. a stalled DMA / bus fault).
    type Error;

    /// Emit one frame per actuator channel. `frames[i]` drives channel `i`.
    fn send(&mut self, frames: &[u16]) -> impl Future<Output = Result<(), Self::Error>>;
}

/// Async per-rail ADC monitor read.
///
/// **Monitor-only.** Power failover is hardware-OR'd (three sources
/// diode/priority-ORed in hardware — DD-014), so *no failover logic crosses
/// this seam*; relay reads per-rail V/I/health for monitoring + logging only.
/// The raw-count → engineering-units decode stays in the driver crate.
pub trait AdcIn {
    /// Transport error.
    type Error;

    /// Read the raw count from monitor `channel` (per-rail INA228).
    fn read(&mut self, channel: u8) -> impl Future<Output = Result<u16, Self::Error>>;
}

/// What kind of device a [`DeviceDescriptor`] refers to.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum DeviceKind {
    Imu,
    Baro,
    Mag,
    Gnss,
    PowerMonitor,
    Other,
}

/// Per-device health — the value the estimator's voter selects on. Sourced from
/// the bus transport's fallibility (a wedged isolated bus → `Failed`).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Health {
    Ok,
    Degraded,
    Failed,
}

/// One physical device the host enumerated as present on the board.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct DeviceDescriptor {
    /// Which **isolated** bus instance this device sits on. Per-instance, never
    /// multiplexed — a wedged bus contains its fault to one device (DD-014).
    pub bus: u8,
    /// Chip identity (WHO_AM_I / part id) the driver confirmed.
    pub part_id: u16,
    pub kind: DeviceKind,
    pub health: Health,
}

/// The fixed-capacity set of devices present on **this** board.
///
/// `no_alloc`. Board variance (DD-013) is expressed by the *enumerated set* a
/// host reports, not by changing the trait arity — the same seam shape serves
/// every board and all three cores; only the contents differ.
#[derive(Clone, Copy, Debug)]
pub struct Presence<const N: usize> {
    devices: [Option<DeviceDescriptor>; N],
    count: usize,
}

impl<const N: usize> Default for Presence<N> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const N: usize> Presence<N> {
    /// An empty device set.
    pub const fn new() -> Self {
        Self {
            devices: [None; N],
            count: 0,
        }
    }

    /// Append a present device. Returns `false` (and leaves the set unchanged)
    /// when already at capacity — never panics, never grows past `N`.
    pub fn push(&mut self, device: DeviceDescriptor) -> bool {
        if self.count >= N {
            return false;
        }
        self.devices[self.count] = Some(device);
        self.count += 1;
        true
    }

    /// How many devices are present.
    pub fn count(&self) -> usize {
        self.count
    }

    /// The descriptor at slot `i`, or `None` if `i` is outside the present
    /// range. Any index — including `>= N` — returns `None` rather than panic.
    pub fn get(&self, i: usize) -> Option<DeviceDescriptor> {
        if i < self.count {
            self.devices[i]
        } else {
            None
        }
    }

    /// How many present devices of `kind` are healthy ([`Health::Ok`]) — the
    /// estimator's selectable voter set. Always `<= count()`.
    pub fn count_healthy(&self, kind: DeviceKind) -> usize {
        let mut n = 0;
        let mut i = 0;
        while i < self.count {
            if let Some(d) = self.devices[i] {
                if d.kind == kind && matches!(d.health, Health::Ok) {
                    n += 1;
                }
            }
            i += 1;
        }
        n
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn dev(bus: u8, kind: DeviceKind, health: Health) -> DeviceDescriptor {
        DeviceDescriptor {
            bus,
            part_id: 0x42,
            kind,
            health,
        }
    }

    #[test]
    fn push_fills_then_refuses() {
        let mut p: Presence<3> = Presence::new();
        assert!(p.push(dev(0, DeviceKind::Imu, Health::Ok)));
        assert!(p.push(dev(1, DeviceKind::Imu, Health::Ok)));
        assert!(p.push(dev(2, DeviceKind::Imu, Health::Degraded)));
        assert_eq!(p.count(), 3);
        // full — refuses, unchanged.
        assert!(!p.push(dev(3, DeviceKind::Imu, Health::Ok)));
        assert_eq!(p.count(), 3);
    }

    #[test]
    fn get_only_in_present_range() {
        let mut p: Presence<4> = Presence::new();
        p.push(dev(0, DeviceKind::Baro, Health::Ok));
        assert!(p.get(0).is_some());
        assert!(p.get(1).is_none()); // present-range, not capacity
        assert!(p.get(99).is_none()); // way out of range, no panic
    }

    #[test]
    fn healthy_set_is_the_voter_set() {
        // Triple-IMU domain (DD-014): one degraded, one failed → 1 selectable.
        let mut p: Presence<4> = Presence::new();
        p.push(dev(0, DeviceKind::Imu, Health::Ok));
        p.push(dev(1, DeviceKind::Imu, Health::Degraded));
        p.push(dev(2, DeviceKind::Imu, Health::Failed));
        p.push(dev(3, DeviceKind::Baro, Health::Ok));
        assert_eq!(p.count_healthy(DeviceKind::Imu), 1);
        assert_eq!(p.count_healthy(DeviceKind::Baro), 1);
        assert_eq!(p.count_healthy(DeviceKind::Mag), 0);
        assert!(p.count_healthy(DeviceKind::Imu) <= p.count());
    }
}
