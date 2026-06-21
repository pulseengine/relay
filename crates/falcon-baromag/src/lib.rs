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

use embedded_hal_async::i2c::I2c;

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
            decode_field(&b)
        }
    }

    /// Pure decode of the 6-byte IST8310 data block (LE i16 X,Y,Z) into a
    /// [`MagField`]. The verifiable body — shared by the sync [`Ist8310`] sim
    /// path and the async [`Ist8310I2c`] hardware path, so the protocol is
    /// decoded once and only the transport differs (relay#214 / jess#62).
    pub fn decode_field(b: &[u8; 6]) -> MagField {
        MagField {
            x_ut: le_i16(b[0], b[1]) as f32 * UT_PER_LSB,
            y_ut: le_i16(b[2], b[3]) as f32 * UT_PER_LSB,
            z_ut: le_i16(b[4], b[5]) as f32 * UT_PER_LSB,
        }
    }

    /// IST8310 over an **`embedded-hal-async` `I2c`** — the hardware path. jess
    /// binds the i.MX RT1176 LPI2C (DMA + completion-IRQ); this driver decodes
    /// above it with the same [`decode_field`]. Fallible: an I2C error returns
    /// `I::Error`, the per-device health source the estimator votes on.
    pub struct Ist8310I2c<I> {
        i2c: I,
        addr: u8,
    }

    impl<I: I2c> Ist8310I2c<I> {
        /// The IST8310's default 7-bit I2C address.
        pub const DEFAULT_ADDR: u8 = 0x0E;

        /// Wrap an I2C bus at the default address.
        pub fn new(i2c: I) -> Self {
            Self { i2c, addr: Self::DEFAULT_ADDR }
        }

        /// Wrap an I2C bus at an explicit address.
        pub fn with_addr(i2c: I, addr: u8) -> Self {
            Self { i2c, addr }
        }

        /// Async field read: one I2C write-read (register pointer → 6 bytes),
        /// then [`decode_field`].
        pub async fn read_field(&mut self) -> Result<MagField, I::Error> {
            let mut b = [0u8; 6];
            self.i2c.write_read(self.addr, &[REG_DATA], &mut b).await?;
            Ok(decode_field(&b))
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
            decode_sample(&b, &self.cal)
        }
    }

    /// Pure decode of the 6-byte BMP388 data block (P[0..3], T[0..3], 24-bit LE)
    /// plus the linear calibration into a [`BaroSample`]. Shared by the sync
    /// [`Bmp388`] sim path and the async [`Bmp388I2c`] hardware path.
    pub fn decode_sample(b: &[u8; 6], cal: &BaroCal) -> BaroSample {
        let raw_pressure = b[0] as u32 | (b[1] as u32) << 8 | (b[2] as u32) << 16;
        let raw_temp = b[3] as u32 | (b[4] as u32) << 8 | (b[5] as u32) << 16;
        BaroSample {
            raw_pressure,
            raw_temp,
            pressure_pa: raw_pressure as f32 * cal.p_scale,
            temp_c: raw_temp as f32 * cal.t_scale + cal.t_offset,
        }
    }

    /// BMP388 over an **`embedded-hal-async` `I2c`** — the hardware path. jess
    /// binds the LPI2C; this driver decodes above it with the same
    /// [`decode_sample`]. Fallible (`I::Error` → per-device health).
    pub struct Bmp388I2c<I> {
        i2c: I,
        addr: u8,
        cal: BaroCal,
    }

    impl<I: I2c> Bmp388I2c<I> {
        /// The BMP388's default 7-bit I2C address (SDO low).
        pub const DEFAULT_ADDR: u8 = 0x76;

        /// Wrap an I2C bus at the default address + default calibration.
        pub fn new(i2c: I) -> Self {
            Self { i2c, addr: Self::DEFAULT_ADDR, cal: BaroCal::default() }
        }

        /// Wrap an I2C bus at an explicit address + calibration.
        pub fn with_cal(i2c: I, addr: u8, cal: BaroCal) -> Self {
            Self { i2c, addr, cal }
        }

        /// Async pressure + temperature read: one I2C write-read, then
        /// [`decode_sample`].
        pub async fn read(&mut self) -> Result<BaroSample, I::Error> {
            let mut b = [0u8; 6];
            self.i2c.write_read(self.addr, &[REG_DATA], &mut b).await?;
            Ok(decode_sample(&b, &self.cal))
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

    // ── async hardware path (embedded-hal-async I2c) ─────────────────────────

    /// A mock `embedded-hal-async` I2c whose write-read returns a canned 6-byte
    /// block (or fails, to exercise the fallible path).
    struct MockI2c {
        block: [u8; 6],
        fail: bool,
    }

    #[derive(Debug)]
    struct MockI2cError;
    impl embedded_hal::i2c::Error for MockI2cError {
        fn kind(&self) -> embedded_hal::i2c::ErrorKind {
            embedded_hal::i2c::ErrorKind::Other
        }
    }
    impl embedded_hal::i2c::ErrorType for MockI2c {
        type Error = MockI2cError;
    }
    impl embedded_hal_async::i2c::I2c for MockI2c {
        async fn transaction(
            &mut self,
            _address: u8,
            operations: &mut [embedded_hal::i2c::Operation<'_>],
        ) -> Result<(), MockI2cError> {
            if self.fail {
                return Err(MockI2cError);
            }
            // write_read lowers to [Write(reg ptr), Read(buf)] — fill the Read.
            for op in operations {
                if let embedded_hal::i2c::Operation::Read(buf) = op {
                    let n = buf.len().min(6);
                    buf[..n].copy_from_slice(&self.block[..n]);
                }
            }
            Ok(())
        }
    }

    /// Drive a future that completes synchronously (the mock never pends).
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

    /// The async I2c mag path decodes a block to the SAME MagField the sync
    /// RegBus path does — the re-target preserves the verified decode.
    #[test]
    fn mag_async_i2c_matches_sync_decode() {
        let block = [0xE8, 0x03, 0x0C, 0xFE, 0xFA, 0x00];
        // sync RegBus path (block at the IST8310 data register 0x03)
        let mut bus = MockBus::new();
        bus.set(0x00, 0x10);
        for (i, &v) in block.iter().enumerate() {
            bus.set(0x03 + i as u8, v);
        }
        let mut sync = Ist8310::new(bus);
        sync.init().unwrap();
        let sync_field = sync.read_field();
        // async I2c path
        let mut hw = Ist8310I2c::new(MockI2c { block, fail: false });
        let async_field = block_on(hw.read_field()).unwrap();
        assert_eq!(sync_field, async_field);
    }

    /// Same equivalence for the baro.
    #[test]
    fn baro_async_i2c_matches_sync_decode() {
        let block = [0x34, 0x12, 0x05, 0x00, 0x80, 0x00];
        let mut bus = MockBus::new();
        bus.set(0x00, 0x50);
        for (i, &v) in block.iter().enumerate() {
            bus.set(0x04 + i as u8, v);
        }
        let mut sync = Bmp388::new(bus);
        sync.init().unwrap();
        let sync_sample = sync.read();
        let mut hw = Bmp388I2c::new(MockI2c { block, fail: false });
        let async_sample = block_on(hw.read()).unwrap();
        assert_eq!(sync_sample, async_sample);
    }

    /// The async paths are fallible: an I2C transport error propagates as Err.
    #[test]
    fn baromag_async_propagates_transport_error() {
        let mut mag = Ist8310I2c::new(MockI2c { block: [0; 6], fail: true });
        assert!(block_on(mag.read_field()).is_err());
        let mut baro = Bmp388I2c::new(MockI2c { block: [0; 6], fail: true });
        assert!(block_on(baro.read()).is_err());
    }

    /// relay#177 — a decoded sensor sample (the HAL output) rides the relay-bus
    /// MessageTransport ring across the M4→M7 inter-core seam, byte-identical
    /// and FIFO. Proves the verified carrier transports the verified HAL output.
    #[test]
    fn mag_sample_rides_the_intercore_ring() {
        use relay_bus::{MessageTransport, SpscRing};

        // M4 (sensor-I/O core): decode two field samples.
        let f0 = decode_field(&[0xE8, 0x03, 0x0C, 0xFE, 0xFA, 0x00]);
        let f1 = decode_field(&[0x10, 0x00, 0x20, 0x00, 0x30, 0x00]);

        // Push them onto the inter-core ring (the #177 carrier seam).
        let mut ring: SpscRing<MagField, 4> = SpscRing::new();
        assert!(ring.push(f0));
        assert!(ring.push(f1));

        // M7 (flight core): pop them back — identical values, FIFO order.
        assert_eq!(ring.pop(), Some(f0));
        assert_eq!(ring.pop(), Some(f1));
        assert!(ring.pop().is_none());
    }
}
