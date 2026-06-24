//! DroneCAN v0 sensor-ingest decoders (DC-P03) — the SPI/I2C drivers' CAN-bus
//! counterparts. Pure fixed-offset DSDL decoders over reassembled transfer
//! payloads (multi-frame via the v1.92 reassembler); the float16 telemetry uses
//! the shared [`crate::float16`] primitive.
//!
//! Decode-in only (falcon reads these): esc.Status (FDI: RPM/voltage/current/
//! temp), MagneticFieldStrength (external mag over CAN), StaticPressure (external
//! baro), BatteryInfo prefix (power monitor over CAN). gnss.Fix2 (variable
//! covariance tail) is the v1.95 follow-on.

use crate::float16::f16_to_f32;

/// Data-type ids of the sensor messages this module decodes.
pub const DTID_ESC_STATUS: u16 = 1034;
pub const DTID_MAG: u16 = 1002; // uavcan.equipment.ahrs.MagneticFieldStrength
pub const DTID_BARO: u16 = 1028; // uavcan.equipment.air_data.StaticPressure
pub const DTID_BATTERY: u16 = 1092; // uavcan.equipment.power.BatteryInfo

/// Read `nbits` (<= 32) starting at LSB-first `bit_off` from `payload`. Bounded:
/// bits past the end read as 0, so it never panics and the result is always
/// `< 2^nbits` (Kani DC-K08). Used for the non-byte-aligned esc.Status fields.
pub(crate) fn read_bits(payload: &[u8], bit_off: usize, nbits: u32) -> u32 {
    let mut v = 0u32;
    let mut i = 0u32;
    while i < nbits && i < 32 {
        let pos = bit_off + i as usize;
        if pos / 8 < payload.len() && (payload[pos / 8] >> (pos % 8)) & 1 != 0 {
            v |= 1 << i;
        }
        i += 1;
    }
    v
}

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
    let rpm_raw = read_bits(p, 80, 18);
    // sign-extend the 18-bit two's-complement rpm to i32
    let rpm = if rpm_raw & 0x0002_0000 != 0 {
        (rpm_raw | 0xFFFC_0000) as i32
    } else {
        rpm_raw as i32
    };
    Some(EscStatus {
        error_count: u32::from_le_bytes([p[0], p[1], p[2], p[3]]),
        voltage: f16_to_f32(u16::from_le_bytes([p[4], p[5]])),
        current: f16_to_f32(u16::from_le_bytes([p[6], p[7]])),
        temperature: f16_to_f32(u16::from_le_bytes([p[8], p[9]])),
        rpm,
        power_rating_pct: read_bits(p, 98, 7) as u8,
        esc_index: read_bits(p, 105, 5) as u8,
    })
}

/// uavcan.equipment.ahrs.MagneticFieldStrength (DTID 1002) — external mag over CAN.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MagField {
    pub ahrs_id: u8,
    /// Magnetic field in gauss, [x, y, z].
    pub field_ga: [f32; 3],
}

/// Decode MagneticFieldStrength (7-byte single-frame: u8 + 3x float16). `None`
/// if shorter. Total.
pub fn decode_mag(p: &[u8]) -> Option<MagField> {
    if p.len() < 7 {
        return None;
    }
    Some(MagField {
        ahrs_id: p[0],
        field_ga: [
            f16_to_f32(u16::from_le_bytes([p[1], p[2]])),
            f16_to_f32(u16::from_le_bytes([p[3], p[4]])),
            f16_to_f32(u16::from_le_bytes([p[5], p[6]])),
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

    #[test]
    fn esc_status_decodes_fields() {
        let mut p = [0u8; 14];
        p[..4].copy_from_slice(&7u32.to_le_bytes()); // error_count = 7
        p[4..6].copy_from_slice(&0x4900u16.to_le_bytes()); // voltage = 10.0
        p[6..8].copy_from_slice(&0x3C00u16.to_le_bytes()); // current = 1.0
        p[8..10].copy_from_slice(&0x4000u16.to_le_bytes()); // temperature = 2.0
        // rpm at bit 80 (byte 10) = 5000; power_rating bit 98; esc_index bit 105
        let rpm: u32 = 5000;
        for b in 0..18 {
            if rpm & (1 << b) != 0 {
                let pos = 80 + b;
                p[pos / 8] |= 1 << (pos % 8);
            }
        }
        let s = decode_esc_status(&p).unwrap();
        assert_eq!(s.error_count, 7);
        assert_eq!(s.voltage, 10.0);
        assert_eq!(s.current, 1.0);
        assert_eq!(s.temperature, 2.0);
        assert_eq!(s.rpm, 5000);
    }

    #[test]
    fn esc_status_rpm_negative_sign_extends() {
        let mut p = [0u8; 14];
        let neg18 = 0x3FFFFu32; // 18-bit -1
        for b in 0..18 {
            if neg18 & (1 << b) != 0 {
                let pos = 80 + b;
                p[pos / 8] |= 1 << (pos % 8);
            }
        }
        assert_eq!(decode_esc_status(&p).unwrap().rpm, -1);
    }

    #[test]
    fn esc_status_rejects_short() {
        assert_eq!(decode_esc_status(&[0; 13]), None);
    }

    #[test]
    fn mag_decodes_three_axes() {
        let mut p = [0u8; 7];
        p[0] = 1; // ahrs_id
        p[1..3].copy_from_slice(&0x3C00u16.to_le_bytes()); // 1.0
        p[3..5].copy_from_slice(&0xC000u16.to_le_bytes()); // -2.0
        p[5..7].copy_from_slice(&0x3800u16.to_le_bytes()); // 0.5
        let m = decode_mag(&p).unwrap();
        assert_eq!(m.ahrs_id, 1);
        assert_eq!(m.field_ga, [1.0, -2.0, 0.5]);
    }

    #[test]
    fn baro_decodes_pressure_f32() {
        let mut p = [0u8; 6];
        p[..4].copy_from_slice(&101325.0f32.to_le_bytes()); // sea-level Pa
        p[4..6].copy_from_slice(&0x3C00u16.to_le_bytes()); // variance 1.0
        let b = decode_baro(&p).unwrap();
        assert_eq!(b.pressure_pa, 101325.0);
        assert_eq!(b.variance, 1.0);
    }

    #[test]
    fn battery_prefix_decodes_voltage_current() {
        let mut p = [0u8; 6];
        p[0..2].copy_from_slice(&0x4900u16.to_le_bytes()); // temp 10.0
        p[2..4].copy_from_slice(&0x4D00u16.to_le_bytes()); // voltage ~20.0
        p[4..6].copy_from_slice(&0x4000u16.to_le_bytes()); // current 2.0
        let bt = decode_battery_info(&p).unwrap();
        assert_eq!(bt.temperature, 10.0);
        assert_eq!(bt.voltage, 20.0);
        assert_eq!(bt.current, 2.0);
    }

    #[test]
    fn short_payloads_reject_not_panic() {
        assert_eq!(decode_mag(&[0; 6]), None);
        assert_eq!(decode_baro(&[0; 5]), None);
        assert_eq!(decode_battery_info(&[0; 5]), None);
    }
}

#[cfg(test)]
mod proptests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        /// All sensor decoders never panic for ANY payload (the float16 path
        /// included); read_bits stays in range. The full-path totality companion
        /// to the Kani read_bits proof.
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
