//! DroneCAN v0 sensor-ingest decoders (DC-P03) — the SPI/I2C drivers' CAN-bus
//! counterparts. Pure fixed-offset DSDL decoders over reassembled transfer
//! payloads (multi-frame via the v1.92 reassembler); the float16 telemetry uses
//! the shared [`crate::float16`] primitive.
//!
//! Decode-in only (falcon reads these): esc.Status (FDI: RPM/voltage/current/
//! temp), MagneticFieldStrength (external mag over CAN), StaticPressure (external
//! baro), BatteryInfo prefix (power monitor over CAN). gnss.Fix2 (variable
//! covariance tail) is the v1.95 follow-on.

use crate::dsdl;
use crate::float16::f16_to_f32;

/// Data-type ids of the sensor messages this module decodes.
pub const DTID_ESC_STATUS: u16 = 1034;
pub const DTID_MAG: u16 = 1002; // uavcan.equipment.ahrs.MagneticFieldStrength
pub const DTID_BARO: u16 = 1028; // uavcan.equipment.air_data.StaticPressure
pub const DTID_BATTERY: u16 = 1092; // uavcan.equipment.power.BatteryInfo

/// uavcan.equipment.esc.Status (DTID 1034) — per-ESC telemetry; the FDI source.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EscStatus {
    pub error_count: u32,
    pub voltage: f32,
    pub current: f32,
    pub temperature: f32,
    pub rpm: i32,
    pub power_rating_pct: u8,
    pub esc_index: u8,
}

/// Decode esc.Status (14-byte fixed layout). `None` if shorter. Total: never
/// panics; bit-fields masked to width (power_rating <= 127, esc_index <= 31).
pub fn decode_esc_status(p: &[u8]) -> Option<EscStatus> {
    if p.len() < 14 {
        return None;
    }
    Some(EscStatus {
        // byte-aligned: little-endian (== DSDL). Bit-misaligned: the DSDL codec.
        error_count: u32::from_le_bytes([p[0], p[1], p[2], p[3]]),
        voltage: f16_to_f32(u16::from_le_bytes([p[4], p[5]])),
        current: f16_to_f32(u16::from_le_bytes([p[6], p[7]])),
        temperature: f16_to_f32(u16::from_le_bytes([p[8], p[9]])),
        rpm: dsdl::read_int(p, 80, 18) as i32,
        power_rating_pct: dsdl::read_uint(p, 98, 7) as u8,
        esc_index: dsdl::read_uint(p, 105, 5) as u8,
    })
}

/// uavcan.equipment.ahrs.MagneticFieldStrength (DTID 1002) — external mag over CAN.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagField {
    /// Magnetic field in gauss, [x, y, z].
    pub field_ga: [f32; 3],
}

/// Decode MagneticFieldStrength (6-byte fixed prefix: 3x float16). `None` if
/// shorter. Total.
///
/// CONFORMANCE FIX: the DSDL has NO `ahrs_id` (that is MagneticFieldStrength2,
/// DTID 1003); `magnetic_field_ga[3]` starts at byte 0. The earlier decoder read
/// a phantom ahrs_id at byte 0 and shifted the field by one byte.
pub fn decode_mag(p: &[u8]) -> Option<MagField> {
    if p.len() < 6 {
        return None;
    }
    Some(MagField {
        field_ga: [
            f16_to_f32(u16::from_le_bytes([p[0], p[1]])),
            f16_to_f32(u16::from_le_bytes([p[2], p[3]])),
            f16_to_f32(u16::from_le_bytes([p[4], p[5]])),
        ],
    })
}

/// uavcan.equipment.air_data.StaticPressure (DTID 1028) — external baro over CAN.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StaticPressure {
    pub pressure_pa: f32,
    pub variance: f32,
}

/// Decode StaticPressure (6-byte single-frame: float32 + float16). `None` if
/// shorter. Total.
pub fn decode_baro(p: &[u8]) -> Option<StaticPressure> {
    if p.len() < 6 {
        return None;
    }
    Some(StaticPressure {
        pressure_pa: f32::from_le_bytes([p[0], p[1], p[2], p[3]]),
        variance: f16_to_f32(u16::from_le_bytes([p[4], p[5]])),
    })
}

/// uavcan.equipment.power.BatteryInfo (DTID 1092) flight-relevant PREFIX —
/// temperature/voltage/current (the BatteryMonitor inputs over CAN). The SoC /
/// capacity / model-name tail is not decoded.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BatteryTelemetry {
    pub temperature: f32,
    pub voltage: f32,
    pub current: f32,
}

/// Decode the BatteryInfo prefix (3x float16). `None` if shorter than 6 bytes.
/// Total.
pub fn decode_battery_info(p: &[u8]) -> Option<BatteryTelemetry> {
    if p.len() < 6 {
        return None;
    }
    Some(BatteryTelemetry {
        temperature: f16_to_f32(u16::from_le_bytes([p[0], p[1]])),
        voltage: f16_to_f32(u16::from_le_bytes([p[2], p[3]])),
        current: f16_to_f32(u16::from_le_bytes([p[4], p[5]])),
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    /// CONFORMANCE: pydronecan canonical esc.Status(error_count=7, voltage=10.0,
    /// current=1.0, temperature=290.0, rpm=5000, power_rating=50, esc_index=3) =
    /// `070000000049003c885c8813190c`. The rpm/power_rating/esc_index bit-fields
    /// use the DSDL codec (the LSB-first read_bits was wrong).
    #[test]
    fn esc_status_decodes_fields() {
        let p = [0x07, 0, 0, 0, 0x00, 0x49, 0x00, 0x3c, 0x88, 0x5c, 0x88, 0x13, 0x19, 0x0c];
        let s = decode_esc_status(&p).unwrap();
        assert_eq!(s.error_count, 7);
        assert_eq!(s.voltage, 10.0);
        assert_eq!(s.current, 1.0);
        assert!((s.temperature - 290.0).abs() < 1.0); // float16 precision
        assert_eq!(s.rpm, 5000);
        assert_eq!(s.power_rating_pct, 50);
        assert_eq!(s.esc_index, 3);
    }

    /// CONFORMANCE: pydronecan esc.Status with rpm=-1 = `...ffffc000`.
    #[test]
    fn esc_status_rpm_negative_sign_extends() {
        let p = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0xff, 0xff, 0xc0, 0x00];
        assert_eq!(decode_esc_status(&p).unwrap().rpm, -1);
    }

    #[test]
    fn esc_status_rejects_short() {
        assert_eq!(decode_esc_status(&[0; 13]), None);
    }

    /// CONFORMANCE: pydronecan MagneticFieldStrength([1.0,-2.0,0.5]) =
    /// `003c00c00038` — field_ga starts at byte 0, NO ahrs_id.
    #[test]
    fn mag_decodes_three_axes() {
        let p = [0x00, 0x3c, 0x00, 0xc0, 0x00, 0x38];
        let m = decode_mag(&p).unwrap();
        assert_eq!(m.field_ga, [1.0, -2.0, 0.5]);
    }

    /// CONFORMANCE: pydronecan StaticPressure(101325.0, variance 1.0) = `80e6c547003c`.
    #[test]
    fn baro_decodes_pressure_f32() {
        let p = [0x80, 0xe6, 0xc5, 0x47, 0x00, 0x3c];
        let b = decode_baro(&p).unwrap();
        assert_eq!(b.pressure_pa, 101325.0);
        assert_eq!(b.variance, 1.0);
    }

    /// CONFORMANCE: pydronecan BatteryInfo(temp=300, voltage=22.2, current=5.0)
    /// prefix = `b05c8d4d0045...`.
    #[test]
    fn battery_prefix_decodes_voltage_current() {
        let p = [0xb0, 0x5c, 0x8d, 0x4d, 0x00, 0x45];
        let bt = decode_battery_info(&p).unwrap();
        assert!((bt.temperature - 300.0).abs() < 1.0);
        assert!((bt.voltage - 22.2).abs() < 0.05);
        assert!((bt.current - 5.0).abs() < 0.05);
    }

    #[test]
    fn short_payloads_reject_not_panic() {
        assert_eq!(decode_mag(&[0; 5]), None); // mag now needs 6 bytes (no ahrs_id)
        assert_eq!(decode_baro(&[0; 5]), None);
        assert_eq!(decode_battery_info(&[0; 5]), None);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// All sensor decoders never panic for ANY payload (the float16 + DSDL
        /// bit-codec paths included); the bit-fields stay in range. The full-path
        /// totality companion to the Kani dsdl-read proof.
        #[test]
        fn sensor_decoders_never_panic(bytes in proptest::collection::vec(any::<u8>(), 0..40)) {
            if let Some(s) = decode_esc_status(&bytes) {
                prop_assert!(s.power_rating_pct <= 127);
                prop_assert!(s.esc_index <= 31);
            }
            let _ = decode_mag(&bytes);
            let _ = decode_baro(&bytes);
            let _ = decode_battery_info(&bytes);
        }
    }
}
