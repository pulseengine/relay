//! Wohl Sensor Wire Protocol — CCSDS payload format for home sensors.
//!
//! Sits inside a CCSDS Space Packet. The CCSDS header provides:
//!   - APID (11 bits) = sensor device ID (up to 2048 devices)
//!   - Sequence count (14 bits) = detect lost packets
//!   - Packet type (1 bit) = telemetry (sensor→hub)
//!
//! This module defines the PAYLOAD format (after the 6-byte CCSDS header).
//!
//! Wire format (14 bytes total = 6 CCSDS header + 8 payload):
//!
//! ```text
//! CCSDS Header (6 bytes):
//!   [0-1] Stream ID: version(3) | type(1) | sec_hdr(1) | APID(11)
//!   [2-3] Sequence:  flags(2) | count(14)
//!   [4-5] Length:    data_length - 1
//!
//! Sensor Payload (8 bytes):
//!   [0]   sensor_type: u8    — what kind of sensor
//!   [1]   quality: u8        — data quality (0=good, 1=stale, 2=error)
//!   [2-3] zone_id: u16       — zone/room identifier (little-endian)
//!   [4-7] value: i32         — fixed-point sensor value (little-endian)
//! ```
//!
//! Value encoding by sensor_type:
//!   TEMP (0x01):     centidegrees Celsius (2150 = 21.50°C)
//!   HUMIDITY (0x02): percent × 100 (6500 = 65.00%)
//!   CO2 (0x03):      ppm (1200 = 1200 ppm)
//!   PM25 (0x04):     µg/m³ × 10 (250 = 25.0 µg/m³)
//!   VOC (0x05):      index 0-500
//!   CONTACT (0x10):  0 = closed, 1 = open
//!   WATER (0x11):    0 = dry, 1 = wet
//!   MOTION (0x12):   0 = no motion, 1 = motion detected
//!   POWER (0x20):    watts × 10 (15230 = 1523.0W)
//!   ENERGY (0x21):   watt-hours
//!   LUX (0x30):      lux
//!   PRESSURE (0x31): hPa × 10 (10132 = 1013.2 hPa)
//!   WIND (0x32):     m/s × 100 (350 = 3.50 m/s)
//!   RAIN (0x33):     mm × 10 (25 = 2.5 mm)
//!
//! Total packet: 14 bytes. Fits in one Zigbee frame (max 127 bytes),
//! one BLE characteristic (max 512 bytes), one LoRa payload (max 222 bytes).

/// Sensor type identifiers
pub const SENSOR_TEMP: u8 = 0x01;
pub const SENSOR_HUMIDITY: u8 = 0x02;
pub const SENSOR_CO2: u8 = 0x03;
pub const SENSOR_PM25: u8 = 0x04;
pub const SENSOR_VOC: u8 = 0x05;
pub const SENSOR_CONTACT: u8 = 0x10;
pub const SENSOR_WATER: u8 = 0x11;
pub const SENSOR_MOTION: u8 = 0x12;
pub const SENSOR_POWER: u8 = 0x20;
pub const SENSOR_ENERGY: u8 = 0x21;
pub const SENSOR_LUX: u8 = 0x30;
pub const SENSOR_PRESSURE: u8 = 0x31;
pub const SENSOR_WIND: u8 = 0x32;
pub const SENSOR_RAIN: u8 = 0x33;

/// Data quality indicator
pub const QUALITY_GOOD: u8 = 0;
pub const QUALITY_STALE: u8 = 1;
pub const QUALITY_ERROR: u8 = 2;

/// Payload size in bytes
pub const PAYLOAD_SIZE: usize = 8;

/// Full packet size: CCSDS header + payload
pub const PACKET_SIZE: usize = 6 + PAYLOAD_SIZE;

/// A decoded sensor reading from the wire
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorPacket {
    /// CCSDS APID = device identifier (0-2047)
    pub device_id: u16,
    /// CCSDS sequence counter (0-16383)
    pub sequence: u16,
    /// Sensor type (SENSOR_TEMP, SENSOR_WATER, etc.)
    pub sensor_type: u8,
    /// Data quality (QUALITY_GOOD, QUALITY_STALE, QUALITY_ERROR)
    pub quality: u8,
    /// Zone/room identifier
    pub zone_id: u16,
    /// Fixed-point sensor value (interpretation depends on sensor_type)
    pub value: i32,
}

/// Encode a sensor packet into a 14-byte buffer.
/// Returns the number of bytes written (always PACKET_SIZE = 14).
pub fn encode_packet(packet: &SensorPacket, buf: &mut [u8; PACKET_SIZE]) {
    // CCSDS header (6 bytes)
    // Version = 0, Type = 0 (telemetry), Sec Header = 0
    let stream_id: u16 = packet.device_id & 0x07FF; // APID in low 11 bits
    buf[0] = (stream_id >> 8) as u8;
    buf[1] = (stream_id & 0xFF) as u8;

    // Sequence: flags = 0b11 (unsegmented), count in low 14 bits
    let seq: u16 = 0xC000 | (packet.sequence & 0x3FFF);
    buf[2] = (seq >> 8) as u8;
    buf[3] = (seq & 0xFF) as u8;

    // Length: data_length - 1 = PAYLOAD_SIZE - 1 = 7
    let length: u16 = (PAYLOAD_SIZE as u16).wrapping_sub(1);
    buf[4] = (length >> 8) as u8;
    buf[5] = (length & 0xFF) as u8;

    // Sensor payload (8 bytes)
    buf[6] = packet.sensor_type;
    buf[7] = packet.quality;
    buf[8] = (packet.zone_id & 0xFF) as u8;
    buf[9] = (packet.zone_id >> 8) as u8;
    let vb = packet.value.to_le_bytes();
    buf[10] = vb[0];
    buf[11] = vb[1];
    buf[12] = vb[2];
    buf[13] = vb[3];
}

/// Decode error
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Buffer too short (need 14 bytes)
    TooShort,
    /// CCSDS version not 0
    InvalidVersion,
    /// Unknown sensor type
    UnknownSensorType,
}

/// Decode a sensor packet from a byte buffer.
pub fn decode_packet(buf: &[u8]) -> Result<SensorPacket, DecodeError> {
    if buf.len() < PACKET_SIZE {
        return Err(DecodeError::TooShort);
    }

    // CCSDS header
    let version = (buf[0] >> 5) & 0x07;
    if version != 0 {
        return Err(DecodeError::InvalidVersion);
    }
    let device_id = ((buf[0] as u16 & 0x07) << 8) | buf[1] as u16;
    let sequence = ((buf[2] as u16 & 0x3F) << 8) | buf[3] as u16;

    // Sensor payload
    let sensor_type = buf[6];
    let quality = buf[7];
    let zone_id = (buf[8] as u16) | ((buf[9] as u16) << 8);
    let value = i32::from_le_bytes([buf[10], buf[11], buf[12], buf[13]]);

    Ok(SensorPacket {
        device_id,
        sequence,
        sensor_type,
        quality,
        zone_id,
        value,
    })
}

/// Convert a Loxone f64 Celsius temperature to Wohl i32 centidegrees
pub fn celsius_to_centidegrees(celsius: f64) -> i32 {
    let scaled = celsius * 100.0;
    if scaled >= i32::MAX as f64 { i32::MAX }
    else if scaled <= i32::MIN as f64 { i32::MIN }
    else { scaled as i32 }
}

/// Convert Wohl i32 centidegrees back to f64 Celsius
pub fn centidegrees_to_celsius(cd: i32) -> f64 {
    cd as f64 / 100.0
}

/// Convert a Loxone f64 watts to Wohl i32 (watts × 10)
pub fn watts_to_fixed(watts: f64) -> i32 {
    let scaled = watts * 10.0;
    if scaled >= i32::MAX as f64 { i32::MAX }
    else if scaled <= i32::MIN as f64 { i32::MIN }
    else { scaled as i32 }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_decode_roundtrip() {
        let packet = SensorPacket {
            device_id: 42,
            sequence: 1000,
            sensor_type: SENSOR_TEMP,
            quality: QUALITY_GOOD,
            zone_id: 1,
            value: 2150, // 21.50°C
        };
        let mut buf = [0u8; PACKET_SIZE];
        encode_packet(&packet, &mut buf);
        let decoded = decode_packet(&buf).unwrap();
        assert_eq!(decoded.device_id, 42);
        assert_eq!(decoded.sequence, 1000);
        assert_eq!(decoded.sensor_type, SENSOR_TEMP);
        assert_eq!(decoded.quality, QUALITY_GOOD);
        assert_eq!(decoded.zone_id, 1);
        assert_eq!(decoded.value, 2150);
    }

    #[test]
    fn test_too_short() {
        assert_eq!(decode_packet(&[0; 5]), Err(DecodeError::TooShort));
    }

    #[test]
    fn test_invalid_version() {
        let mut buf = [0u8; PACKET_SIZE];
        buf[0] = 0x20; // version = 1
        assert_eq!(decode_packet(&buf), Err(DecodeError::InvalidVersion));
    }

    #[test]
    fn test_water_sensor() {
        let packet = SensorPacket {
            device_id: 100,
            sequence: 0,
            sensor_type: SENSOR_WATER,
            quality: QUALITY_GOOD,
            zone_id: 3,
            value: 1, // wet
        };
        let mut buf = [0u8; PACKET_SIZE];
        encode_packet(&packet, &mut buf);
        let decoded = decode_packet(&buf).unwrap();
        assert_eq!(decoded.sensor_type, SENSOR_WATER);
        assert_eq!(decoded.value, 1);
    }

    #[test]
    fn test_contact_sensor() {
        let packet = SensorPacket {
            device_id: 200,
            sequence: 5,
            sensor_type: SENSOR_CONTACT,
            quality: QUALITY_GOOD,
            zone_id: 1,
            value: 1, // open
        };
        let mut buf = [0u8; PACKET_SIZE];
        encode_packet(&packet, &mut buf);
        let decoded = decode_packet(&buf).unwrap();
        assert_eq!(decoded.sensor_type, SENSOR_CONTACT);
        assert_eq!(decoded.value, 1);
    }

    #[test]
    fn test_negative_temperature() {
        let packet = SensorPacket {
            device_id: 1,
            sequence: 0,
            sensor_type: SENSOR_TEMP,
            quality: QUALITY_GOOD,
            zone_id: 4,
            value: -1500, // -15.00°C
        };
        let mut buf = [0u8; PACKET_SIZE];
        encode_packet(&packet, &mut buf);
        let decoded = decode_packet(&buf).unwrap();
        assert_eq!(decoded.value, -1500);
    }

    #[test]
    fn test_celsius_conversion() {
        assert_eq!(celsius_to_centidegrees(21.5), 2150);
        assert_eq!(celsius_to_centidegrees(-15.0), -1500);
        assert_eq!(celsius_to_centidegrees(0.0), 0);
        assert_eq!(centidegrees_to_celsius(2150), 21.5);
    }

    #[test]
    fn test_watts_conversion() {
        assert_eq!(watts_to_fixed(1523.0), 15230);
        assert_eq!(watts_to_fixed(0.0), 0);
    }

    #[test]
    fn test_all_sensor_types() {
        for st in [SENSOR_TEMP, SENSOR_HUMIDITY, SENSOR_CO2, SENSOR_PM25, SENSOR_VOC,
                   SENSOR_CONTACT, SENSOR_WATER, SENSOR_MOTION,
                   SENSOR_POWER, SENSOR_ENERGY,
                   SENSOR_LUX, SENSOR_PRESSURE, SENSOR_WIND, SENSOR_RAIN] {
            let packet = SensorPacket {
                device_id: 1, sequence: 0, sensor_type: st,
                quality: QUALITY_GOOD, zone_id: 1, value: 42,
            };
            let mut buf = [0u8; PACKET_SIZE];
            encode_packet(&packet, &mut buf);
            let decoded = decode_packet(&buf).unwrap();
            assert_eq!(decoded.sensor_type, st);
            assert_eq!(decoded.value, 42);
        }
    }

    #[test]
    fn test_packet_size() {
        assert_eq!(PACKET_SIZE, 14);
        // Fits in: Zigbee (127), BLE (512), LoRa (222), WiFi (∞)
    }

    #[test]
    fn test_max_device_id() {
        let packet = SensorPacket {
            device_id: 2047, // max APID
            sequence: 16383, // max sequence
            sensor_type: SENSOR_TEMP,
            quality: QUALITY_GOOD,
            zone_id: 0xFFFF,
            value: i32::MAX,
        };
        let mut buf = [0u8; PACKET_SIZE];
        encode_packet(&packet, &mut buf);
        let decoded = decode_packet(&buf).unwrap();
        assert_eq!(decoded.device_id, 2047);
        assert_eq!(decoded.sequence, 16383);
        assert_eq!(decoded.zone_id, 0xFFFF);
        assert_eq!(decoded.value, i32::MAX);
    }
}
