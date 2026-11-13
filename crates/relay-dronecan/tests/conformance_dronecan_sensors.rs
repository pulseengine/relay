#![allow(dead_code)]
use relay_dronecan::sensors::{decode_baro, decode_battery_info, decode_esc_status, decode_mag};

mod vectors {
    include!("vectors/dronecan_sensors.rs");
}
use vectors::*;

#[test]
fn conformance_esc_status_fields() {
    assert!(!ESC_STATUS.is_empty());
    for (i, v) in ESC_STATUS.iter().enumerate() {
        let s = decode_esc_status(v.bytes)
            .unwrap_or_else(|| panic!("vector {i}: decode returned None"));
        assert_eq!(s.error_count, v.error_count, "vector {i}: error_count");
        assert_eq!(s.voltage.to_bits(), v.voltage_bits, "vector {i}: voltage");
        assert_eq!(s.current.to_bits(), v.current_bits, "vector {i}: current");
        assert_eq!(
            s.temperature.to_bits(),
            v.temperature_bits,
            "vector {i}: temperature"
        );
        assert_eq!(s.rpm, v.rpm, "vector {i}: rpm");
        assert_eq!(
            s.power_rating_pct, v.power_rating_pct,
            "vector {i}: power_rating_pct"
        );
        assert_eq!(s.esc_index, v.esc_index, "vector {i}: esc_index");
    }
}

#[test]
fn conformance_mag_fields() {
    assert!(!MAG.is_empty());
    for (i, v) in MAG.iter().enumerate() {
        let m = decode_mag(v.bytes).unwrap_or_else(|| panic!("vector {i}: decode returned None"));
        assert_eq!(m.field_ga[0].to_bits(), v.ga_x_bits, "vector {i}: ga_x");
        assert_eq!(m.field_ga[1].to_bits(), v.ga_y_bits, "vector {i}: ga_y");
        assert_eq!(m.field_ga[2].to_bits(), v.ga_z_bits, "vector {i}: ga_z");
    }
}

#[test]
fn conformance_baro_fields() {
    assert!(!BARO.is_empty());
    for (i, v) in BARO.iter().enumerate() {
        let b = decode_baro(v.bytes).unwrap_or_else(|| panic!("vector {i}: decode returned None"));
        assert_eq!(
            b.pressure_pa.to_bits(),
            v.pressure_bits,
            "vector {i}: pressure_pa"
        );
        assert_eq!(
            b.variance.to_bits(),
            v.variance_bits,
            "vector {i}: variance"
        );
    }
}

#[test]
fn conformance_battery_prefix_fields() {
    assert!(!BATTERY.is_empty());
    for (i, v) in BATTERY.iter().enumerate() {
        let b = decode_battery_info(v.bytes)
            .unwrap_or_else(|| panic!("vector {i}: decode returned None"));
        assert_eq!(
            b.temperature.to_bits(),
            v.temperature_bits,
            "vector {i}: temperature"
        );
        assert_eq!(b.voltage.to_bits(), v.voltage_bits, "vector {i}: voltage");
        assert_eq!(b.current.to_bits(), v.current_bits, "vector {i}: current");
    }
}

#[test]
fn conformance_short_payloads_rejected() {
    assert!(!SHORT_ESC_STATUS.is_empty());
    for (i, b) in SHORT_ESC_STATUS.iter().enumerate() {
        assert!(
            decode_esc_status(b).is_none(),
            "short esc vector {i} (len {})",
            b.len()
        );
    }
    for (i, b) in SHORT_MAG.iter().enumerate() {
        assert!(
            decode_mag(b).is_none(),
            "short mag vector {i} (len {})",
            b.len()
        );
    }
    for (i, b) in SHORT_BARO.iter().enumerate() {
        assert!(
            decode_baro(b).is_none(),
            "short baro vector {i} (len {})",
            b.len()
        );
    }
    for (i, b) in SHORT_BATTERY.iter().enumerate() {
        assert!(
            decode_battery_info(b).is_none(),
            "short battery vector {i} (len {})",
            b.len()
        );
    }
}
