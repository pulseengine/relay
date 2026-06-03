//! # falcon-imu-icm42688 — a reference IMU driver behind the HAL seam (v1.13.0)
//!
//! A concrete [`ImuDriver`] for the TDK InvenSense **ICM-42688-P** 6-axis IMU,
//! the first real driver written against the v1.11 hardware seam. It shows what
//! "implement five drivers and the same verified core flies your board" means
//! in practice: this is one of the five.
//!
//! ## What is proven here (against a mock bus, no silicon)
//!
//! - the **register protocol** — `WHO_AM_I` identity check, power-up and
//!   full-scale configuration, and the big-endian burst read of the
//!   accel+gyro data registers;
//! - the **unit-scaling math** — raw `i16` counts → SI body-frame
//!   (m/s², rad/s) at the configured full-scale ranges;
//! - that it satisfies the [`ImuDriver`] contract, so a
//!   `HardwareBackend { imu: Icm42688::new(bus), .. }` type-checks and drives
//!   the verified `FlightCore` unchanged.
//!
//! ## The documented hardware GAPS (need the real chip — not faked)
//!
//! - a real [`RegBus`] over `embedded-hal` SPI (CS timing, mode 3, the
//!   read/write address-MSB convention) and its **on-silicon validation**;
//! - sensor **calibration** (bias, scale, axis/board-mounting remap — the
//!   identity remap here is a placeholder; real mounting needs a 3×3);
//! - data-ready/FIFO timing, error/NaN handling on bus faults, temperature
//!   compensation.
//!
//! Everything above the bus is real driver logic; the bus and its validation
//! are yours.

#![no_std]

use falcon_core::{ImuDriver, ImuSample};

/// A minimal, dependency-free register bus: the thin contract your board's SPI
/// (or I²C) peripheral satisfies. `read_burst` reads `buf.len()` sequential
/// registers starting at `reg` (auto-increment), exactly as the ICM-42688
/// supports for its data block. A real impl wraps `embedded-hal`.
pub trait RegBus {
    /// Read one register.
    fn read_reg(&mut self, reg: u8) -> u8;
    /// Write one register.
    fn write_reg(&mut self, reg: u8, val: u8);
    /// Burst-read sequential registers starting at `reg` into `buf`.
    fn read_burst(&mut self, reg: u8, buf: &mut [u8]);
}

/// Driver initialisation failures.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriverError {
    /// `WHO_AM_I` did not return the expected ICM-42688 identity.
    WrongWhoAmI(u8),
}

// ── ICM-42688-P register map (user bank 0) ───────────────────────────────
const REG_PWR_MGMT0: u8 = 0x4E;
const REG_GYRO_CONFIG0: u8 = 0x4F;
const REG_ACCEL_CONFIG0: u8 = 0x50;
const REG_WHO_AM_I: u8 = 0x75;
/// Start of the 12-byte accel+gyro data block (ACCEL_DATA_X1).
const REG_ACCEL_DATA_X1: u8 = 0x1F;
/// The ICM-42688-P identity.
pub const WHO_AM_I_VALUE: u8 = 0x47;

const GRAVITY: f32 = 9.80665;
const PI: f32 = core::f32::consts::PI;

/// The ICM-42688-P driver, generic over the register bus. Configured at
/// power-up to ±16 g / ±2000 dps full scale (the scales below match).
pub struct Icm42688<B> {
    bus: B,
    accel_scale: f32, // m/s² per LSB
    gyro_scale: f32,  // rad/s per LSB
}

impl<B: RegBus> Icm42688<B> {
    /// Wrap a bus. Call [`init`](Self::init) before reading.
    pub fn new(bus: B) -> Self {
        // ±16 g → 2048 LSB/g; ±2000 dps → 16.4 LSB/dps (datasheet sensitivities).
        Icm42688 {
            bus,
            accel_scale: GRAVITY / 2048.0,
            gyro_scale: (PI / 180.0) / 16.4,
        }
    }

    /// Verify identity and bring the sensor up: gyro + accel in low-noise mode,
    /// ±2000 dps and ±16 g full scale. Returns the chip's `WHO_AM_I` mismatch
    /// if it is not an ICM-42688.
    pub fn init(&mut self) -> Result<(), DriverError> {
        let who = self.bus.read_reg(REG_WHO_AM_I);
        if who != WHO_AM_I_VALUE {
            return Err(DriverError::WrongWhoAmI(who));
        }
        // PWR_MGMT0: GYRO_MODE=11 (low-noise) | ACCEL_MODE=11 (low-noise).
        self.bus.write_reg(REG_PWR_MGMT0, 0x0F);
        // GYRO_CONFIG0: GYRO_FS_SEL=000 (±2000 dps) | ODR=0110 (1 kHz).
        self.bus.write_reg(REG_GYRO_CONFIG0, 0x06);
        // ACCEL_CONFIG0: ACCEL_FS_SEL=000 (±16 g) | ODR=0110 (1 kHz).
        self.bus.write_reg(REG_ACCEL_CONFIG0, 0x06);
        Ok(())
    }

    /// Burst-read the data block and assemble the SI body-frame sample.
    pub fn read_sample(&mut self) -> ImuSample {
        let mut raw = [0u8; 12];
        self.bus.read_burst(REG_ACCEL_DATA_X1, &mut raw);
        // big-endian i16 pairs: AX AY AZ GX GY GZ
        let be = |hi: u8, lo: u8| ((hi as i16) << 8 | lo as i16) as f32;
        let accel = [
            be(raw[0], raw[1]) * self.accel_scale,
            be(raw[2], raw[3]) * self.accel_scale,
            be(raw[4], raw[5]) * self.accel_scale,
        ];
        let gyro = [
            be(raw[6], raw[7]) * self.gyro_scale,
            be(raw[8], raw[9]) * self.gyro_scale,
            be(raw[10], raw[11]) * self.gyro_scale,
        ];
        // NB: identity axis remap. Real board mounting needs a 3×3 here.
        ImuSample { accel, gyro }
    }

    /// Consume the driver, returning the bus (for shutdown / re-use).
    pub fn release(self) -> B {
        self.bus
    }
}

impl<B: RegBus> ImuDriver for Icm42688<B> {
    fn read(&mut self) -> ImuSample {
        self.read_sample()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A mock register bus with a programmable register file: `WHO_AM_I` and a
    /// 12-byte data block. Records the config writes so the test can assert the
    /// driver programmed the expected full-scale ranges.
    struct MockBus {
        who: u8,
        data: [u8; 12],
        writes: [(u8, u8); 8],
        n_writes: usize,
    }
    impl MockBus {
        fn new(who: u8, data: [u8; 12]) -> Self {
            MockBus { who, data, writes: [(0, 0); 8], n_writes: 0 }
        }
    }
    impl RegBus for MockBus {
        fn read_reg(&mut self, reg: u8) -> u8 {
            if reg == REG_WHO_AM_I {
                self.who
            } else {
                0
            }
        }
        fn write_reg(&mut self, reg: u8, val: u8) {
            if self.n_writes < self.writes.len() {
                self.writes[self.n_writes] = (reg, val);
                self.n_writes += 1;
            }
        }
        fn read_burst(&mut self, reg: u8, buf: &mut [u8]) {
            assert_eq!(reg, REG_ACCEL_DATA_X1, "driver must burst from the data block");
            buf.copy_from_slice(&self.data);
        }
    }

    /// Big-endian encode an i16 into (hi, lo).
    fn be(v: i16) -> (u8, u8) {
        ((v >> 8) as u8, (v & 0xFF) as u8)
    }

    /// The driver identifies the chip, configures ±16 g / ±2000 dps, and decodes
    /// the data block into correct SI units. AZ = 2048 LSB = +1 g → 9.80665
    /// m/s²; GX = 164 LSB = +10 dps → 0.174533 rad/s.
    #[test]
    fn decodes_register_block_to_si() {
        let (azh, azl) = be(2048); // +1 g on Z
        let (gxh, gxl) = be(164); // +10 dps on X
        let data = [
            0, 0, // AX = 0
            0, 0, // AY = 0
            azh, azl, // AZ = 2048
            gxh, gxl, // GX = 164
            0, 0, // GY = 0
            0, 0, // GZ = 0
        ];
        let mut imu = Icm42688::new(MockBus::new(WHO_AM_I_VALUE, data));
        imu.init().expect("identity ok");

        // configured the documented full-scale registers
        let bus = &imu.bus;
        assert!(bus.writes[..bus.n_writes].contains(&(REG_PWR_MGMT0, 0x0F)));
        assert!(bus.writes[..bus.n_writes].contains(&(REG_GYRO_CONFIG0, 0x06)));
        assert!(bus.writes[..bus.n_writes].contains(&(REG_ACCEL_CONFIG0, 0x06)));

        let s = imu.read();
        assert!((s.accel[0]).abs() < 1e-4);
        assert!((s.accel[1]).abs() < 1e-4);
        assert!((s.accel[2] - 9.80665).abs() < 1e-3, "AZ → 1 g: {}", s.accel[2]);
        assert!((s.gyro[0] - 0.174533).abs() < 1e-4, "GX → 10 dps: {}", s.gyro[0]);
        assert!((s.gyro[1]).abs() < 1e-6 && (s.gyro[2]).abs() < 1e-6);
    }

    /// A wrong `WHO_AM_I` is rejected (not silently flown).
    #[test]
    fn rejects_wrong_chip() {
        let mut imu = Icm42688::new(MockBus::new(0x12, [0u8; 12]));
        assert_eq!(imu.init(), Err(DriverError::WrongWhoAmI(0x12)));
    }

    /// Negative counts decode through the sign bit (AZ = −1 g).
    #[test]
    fn decodes_negative_counts() {
        let (azh, azl) = be(-2048);
        let data = [0, 0, 0, 0, azh, azl, 0, 0, 0, 0, 0, 0];
        let mut imu = Icm42688::new(MockBus::new(WHO_AM_I_VALUE, data));
        imu.init().unwrap();
        let s = imu.read();
        assert!((s.accel[2] + 9.80665).abs() < 1e-3, "AZ → −1 g: {}", s.accel[2]);
    }

    /// The driver satisfies the seam: it plugs into a `HardwareBackend` as the
    /// IMU and reports through the `ImuDriver` trait object boundary.
    #[test]
    fn satisfies_imu_driver_seam() {
        let data = [0, 0, 0, 0, 0x08, 0x00, 0, 0, 0, 0, 0, 0];
        let mut imu = Icm42688::new(MockBus::new(WHO_AM_I_VALUE, data));
        imu.init().unwrap();
        fn use_driver<D: ImuDriver>(d: &mut D) -> ImuSample {
            d.read()
        }
        let s = use_driver(&mut imu);
        assert!((s.accel[2] - 9.80665).abs() < 1e-3);
    }
}
