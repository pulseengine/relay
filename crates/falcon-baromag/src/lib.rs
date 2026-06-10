//! Falcon magnetometer + barometer driver bodies (F6 sensor trio).
//!
//! Two register-bus sensor drivers that complete the input side of the F6
//! driver suite (GNSS landed v1.42; the ICM-42688 IMU is the v1.13 reference):
//!
//!  * **`mag`** — an IST8310 3-axis magnetometer over I2C: `WHO_AM_I` identity
//!    check, read the 6 data bytes (3×i16 LE), scale to microtesla, and a flat
//!    (level) heading helper. Feeds the IEKF's existing magnetometer-heading
//!    update (`relay_iekf::Iekf::update_magnetometer`).
//!  * **`baro`** — a BMP388 barometer: `CHIP_ID` check, read the 24-bit pressure
//!    + temperature words, apply a linear calibration to Pa / °C. Feeds the IEKF
//!    baro z-anchor.
//!
//! These pin the **byte-level register protocol** (the verifiable driver part),
//! the same posture as the ICM-42688 and UBX drivers: mock-bus protocol tests +
//! Kani totality (the decode never panics on any register contents). On-silicon
//! I2C/SPI validation stays the v1.55 open item.
//!
//! Honest limits: the BMP388 per-chip NVM trim compensation (the 21-coefficient
//! polynomial) is a calibration follow-on — this driver applies a simple linear
//! scale/offset. Pressure→altitude needs `powf` (not in the relay-math seam yet)
//! so it is left to the consumer. The mag heading helper is flat (not
//! tilt-compensated); the IEKF's `update_magnetometer` does tilt compensation.
//!
//! no_std / no_alloc / `forbid(unsafe_code)`.

#![no_std]
#![forbid(unsafe_code)]

/// A minimal register bus (I2C/SPI register transactions). Mirrors the
/// `falcon-imu-icm42688` `RegBus` so all sensor drivers share the seam shape; a
/// real impl wraps `embedded-hal`.
pub trait RegBus {
    /// Read one register.
    fn read_reg(&mut self, reg: u8) -> u8;
    /// Write one register.
    fn write_reg(&mut self, reg: u8, val: u8);
    /// Read `buf.len()` bytes starting at `reg` (auto-increment).
    fn read_burst(&mut self, reg: u8, buf: &mut [u8]);
}

/// Driver error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriverError {
    /// The identity register did not return the expected device id.
    WrongIdentity { got: u8, want: u8 },
}

#[inline]
fn le_i16(lo: u8, hi: u8) -> i16 {
    i16::from_le_bytes([lo, hi])
}

/// IST8310 magnetometer.
pub mod mag {
    use super::*;

    const REG_WHO_AM_I: u8 = 0x00;
    const WHO_AM_I_VALUE: u8 = 0x10;
    const REG_CNTL1: u8 = 0x0A;
    const MODE_CONTINUOUS: u8 = 0x01;
    const REG_DATA: u8 = 0x03; // X_L, X_H, Y_L, Y_H, Z_L, Z_H
    /// IST8310 sensitivity: ~0.3 microtesla / LSB.
    const UT_PER_LSB: f32 = 0.3;

    /// A magnetometer reading (microtesla, sensor axes).
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct MagField {
        /// X field, µT.
        pub x_ut: f32,
        /// Y field, µT.
        pub y_ut: f32,
        /// Z field, µT.
        pub z_ut: f32,
    }

    /// IST8310 driver over a [`RegBus`].
    pub struct Ist8310<B: RegBus> {
        bus: B,
    }

    impl<B: RegBus> Ist8310<B> {
        /// Wrap a bus (does not touch the device until [`init`](Self::init)).
        pub fn new(bus: B) -> Self {
            Self { bus }
        }

        /// Verify identity and put the device into continuous-measurement mode.
        pub fn init(&mut self) -> Result<(), DriverError> {
            let id = self.bus.read_reg(REG_WHO_AM_I);
            if id != WHO_AM_I_VALUE {
                return Err(DriverError::WrongIdentity { got: id, want: WHO_AM_I_VALUE });
            }
            self.bus.write_reg(REG_CNTL1, MODE_CONTINUOUS);
            Ok(())
        }

        /// Read the field vector (µT). Total: any register contents decode to a
        /// finite vector, no panic.
        pub fn read_field(&mut self) -> MagField {
            let mut b = [0u8; 6];
            self.bus.read_burst(REG_DATA, &mut b);
            MagField {
                x_ut: le_i16(b[0], b[1]) as f32 * UT_PER_LSB,
                y_ut: le_i16(b[2], b[3]) as f32 * UT_PER_LSB,
                z_ut: le_i16(b[4], b[5]) as f32 * UT_PER_LSB,
            }
        }
    }

    /// Flat (level) magnetic heading from a field vector, rad in (−π, π].
    /// NOT tilt-compensated — the IEKF `update_magnetometer` does that; this is
    /// for a quick level-heading readout. Degenerate (no horizontal field) ⇒ 0.
    pub fn flat_heading(field: MagField) -> f32 {
        if (field.x_ut * field.x_ut + field.y_ut * field.y_ut) < 1e-12 {
            return 0.0;
        }
        relay_math::atan2f(field.y_ut, field.x_ut)
    }
}

/// BMP388 barometer.
pub mod baro {
    use super::*;

    const REG_CHIP_ID: u8 = 0x00;
    const CHIP_ID_VALUE: u8 = 0x50;
    const REG_PWR_CTRL: u8 = 0x1B;
    const PWR_PRESS_TEMP_NORMAL: u8 = 0x33; // press_en|temp_en|mode=normal
    const REG_DATA: u8 = 0x04; // P[0..3], T[0..3] (24-bit LE each)

    /// A barometer reading.
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub struct BaroSample {
        /// Raw 24-bit pressure word.
        pub raw_pressure: u32,
        /// Raw 24-bit temperature word.
        pub raw_temp: u32,
        /// Pressure, Pa (linear calibration applied).
        pub pressure_pa: f32,
        /// Temperature, °C (linear calibration applied).
        pub temp_c: f32,
    }

    /// Linear calibration (placeholder for the full BMP388 NVM trim, which is the
    /// documented follow-on). `pressure_pa = raw_pressure * p_scale`,
    /// `temp_c = raw_temp * t_scale + t_offset`.
    #[derive(Clone, Copy, Debug)]
    pub struct BaroCal {
        /// Pa per pressure LSB.
        pub p_scale: f32,
        /// °C per temp LSB.
        pub t_scale: f32,
        /// °C offset.
        pub t_offset: f32,
    }

    impl Default for BaroCal {
        fn default() -> Self {
            // Nominal scales: BMP388 raw is ~24-bit over the sensor range; these
            // map a mid-range raw to ~standard sea-level pressure. Replaced by
            // the per-chip NVM trim at calibration time.
            Self { p_scale: 0.012_5, t_scale: 0.005, t_offset: 0.0 }
        }
    }

    /// BMP388 driver over a [`RegBus`].
    pub struct Bmp388<B: RegBus> {
        bus: B,
        cal: BaroCal,
    }

    impl<B: RegBus> Bmp388<B> {
        /// Wrap a bus with the default calibration.
        pub fn new(bus: B) -> Self {
            Self { bus, cal: BaroCal::default() }
        }

        /// Wrap a bus with an explicit calibration.
        pub fn with_cal(bus: B, cal: BaroCal) -> Self {
            Self { bus, cal }
        }

        /// Verify identity and enable pressure+temperature normal mode.
        pub fn init(&mut self) -> Result<(), DriverError> {
            let id = self.bus.read_reg(REG_CHIP_ID);
            if id != CHIP_ID_VALUE {
                return Err(DriverError::WrongIdentity { got: id, want: CHIP_ID_VALUE });
            }
            self.bus.write_reg(REG_PWR_CTRL, PWR_PRESS_TEMP_NORMAL);
            Ok(())
        }

        /// Read pressure + temperature. Total for any register contents.
        pub fn read(&mut self) -> BaroSample {
            let mut b = [0u8; 6];
            self.bus.read_burst(REG_DATA, &mut b);
            let raw_pressure = b[0] as u32 | (b[1] as u32) << 8 | (b[2] as u32) << 16;
            let raw_temp = b[3] as u32 | (b[4] as u32) << 8 | (b[5] as u32) << 16;
            BaroSample {
                raw_pressure,
                raw_temp,
                pressure_pa: raw_pressure as f32 * self.cal.p_scale,
                temp_c: raw_temp as f32 * self.cal.t_scale + self.cal.t_offset,
            }
        }
    }
}

#[cfg(kani)]
mod kani_proofs;

#[cfg(test)]
mod tests {
    use super::baro::*;
    use super::mag::*;
    use super::*;

    /// A mock register bus: a 256-byte register file + a recorded data block for
    /// the burst region.
    struct MockBus {
        regs: [u8; 256],
    }
    impl MockBus {
        fn new() -> Self {
            MockBus { regs: [0; 256] }
        }
        fn set(&mut self, reg: u8, val: u8) {
            self.regs[reg as usize] = val;
        }
    }
    impl RegBus for MockBus {
        fn read_reg(&mut self, reg: u8) -> u8 {
            self.regs[reg as usize]
        }
        fn write_reg(&mut self, reg: u8, val: u8) {
            self.regs[reg as usize] = val;
        }
        fn read_burst(&mut self, reg: u8, buf: &mut [u8]) {
            for (i, b) in buf.iter_mut().enumerate() {
                *b = self.regs[(reg as usize + i) & 0xff];
            }
        }
    }

    #[test]
    fn mag_identity_and_decode() {
        let mut bus = MockBus::new();
        bus.set(0x00, 0x10); // WHO_AM_I
        // Data block X_L,X_H,Y_L,Y_H,Z_L,Z_H at 0x03..0x08.
        // X = +1000 (0x03E8), Y = -500 (0xFE0C), Z = +250 (0x00FA).
        bus.set(0x03, 0xE8);
        bus.set(0x04, 0x03); // X = 1000
        bus.set(0x05, 0x0C);
        bus.set(0x06, 0xFE); // Y = -500
        bus.set(0x07, 0xFA);
        bus.set(0x08, 0x00); // Z = 250
        let mut m = Ist8310::new(bus);
        m.init().expect("identity ok");
        let f = m.read_field();
        assert!((f.x_ut - 300.0).abs() < 1e-3); // 1000 * 0.3
        assert!((f.y_ut - (-150.0)).abs() < 1e-3); // -500 * 0.3
        assert!((f.z_ut - 75.0).abs() < 1e-3); // 250 * 0.3
    }

    #[test]
    fn mag_wrong_identity_rejected() {
        let mut bus = MockBus::new();
        bus.set(0x00, 0xAB);
        let mut m = Ist8310::new(bus);
        assert_eq!(m.init(), Err(DriverError::WrongIdentity { got: 0xAB, want: 0x10 }));
    }

    #[test]
    fn flat_heading_cardinal() {
        // field pointing +Y (east) ⇒ heading +π/2.
        let h = flat_heading(MagField { x_ut: 0.0, y_ut: 10.0, z_ut: 0.0 });
        assert!((h - core::f32::consts::FRAC_PI_2).abs() < 1e-4);
    }

    #[test]
    fn baro_identity_and_decode() {
        let mut bus = MockBus::new();
        bus.set(0x00, 0x50); // CHIP_ID
        // pressure raw = 0x051234, temp raw = 0x008000 at 0x04..0x0A
        bus.set(0x04, 0x34);
        bus.set(0x05, 0x12);
        bus.set(0x06, 0x05); // 0x051234 = 332340
        bus.set(0x07, 0x00);
        bus.set(0x08, 0x80);
        bus.set(0x09, 0x00); // 0x008000 = 32768
        let mut b = Bmp388::new(bus);
        b.init().expect("chip id ok");
        let s = b.read();
        assert_eq!(s.raw_pressure, 0x051234);
        assert_eq!(s.raw_temp, 0x008000);
        assert!((s.pressure_pa - 332340.0 * 0.0125).abs() < 1.0);
    }

    #[test]
    fn baro_wrong_identity_rejected() {
        let mut bus = MockBus::new();
        bus.set(0x00, 0x99);
        let mut b = Bmp388::new(bus);
        assert_eq!(b.init(), Err(DriverError::WrongIdentity { got: 0x99, want: 0x50 }));
    }
}
