//! Relay Command Ingest — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_VALID_CMD_CODES: usize = 256;
pub const MAX_STREAM_IDS: usize = 16;

#[derive(Clone, Copy)]
pub struct CommandHeader {
    pub stream_id: u16,
    pub sequence: u16,
    pub length: u16,
    pub function_code: u8,
    pub checksum: u8,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum CiValidation {
    Valid = 0,
    InvalidStreamId = 1,
    BadChecksum = 2,
    InvalidCmdCode = 3,
    LengthMismatch = 4,
}

#[derive(Clone, Copy)]
pub struct CiConfig {
    pub valid_stream_ids: [u16; MAX_STREAM_IDS],
    pub stream_id_count: u32,
    pub max_cmd_code: u8,
    pub min_length: u16,
    pub max_length: u16,
}

impl CommandHeader {
    pub const fn empty() -> Self {
        CommandHeader { stream_id: 0, sequence: 0, length: 0, function_code: 0, checksum: 0 }
    }
}

impl CiConfig {
    pub fn new() -> Self {
        CiConfig {
            valid_stream_ids: [0u16; MAX_STREAM_IDS],
            stream_id_count: 0,
            max_cmd_code: 0,
            min_length: 0,
            max_length: 0,
        }
    }
}

/// Compute XOR checksum over a data slice (cFS-style).
pub fn compute_checksum(data: &[u8]) -> u8 {
    let mut csum: u8 = 0;
    let mut i: usize = 0;
    while i < data.len() {
        csum = csum ^ data[i];
        i = i + 1;
    }
    csum
}

/// Returns true only for configured stream IDs.
pub fn is_valid_stream_id(config: &CiConfig, stream_id: u16) -> bool {
    let count = config.stream_id_count;
    let mut i: u32 = 0;
    while i < count {
        if config.valid_stream_ids[i as usize] == stream_id {
            return true;
        }
        i = i + 1;
    }
    false
}

/// Validate a command header against the configuration.
pub fn validate_header(config: &CiConfig, header: &CommandHeader) -> CiValidation {
    // Check stream ID
    if !is_valid_stream_id(config, header.stream_id) {
        return CiValidation::InvalidStreamId;
    }

    // Check checksum (expected: 0 for valid packets)
    if header.checksum != 0 {
        return CiValidation::BadChecksum;
    }

    // Check command code range
    if header.function_code > config.max_cmd_code {
        return CiValidation::InvalidCmdCode;
    }

    // Check length
    if header.length < config.min_length || header.length > config.max_length {
        return CiValidation::LengthMismatch;
    }

    CiValidation::Valid
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> CiConfig {
        let mut config = CiConfig::new();
        config.valid_stream_ids[0] = 0x1880;
        config.valid_stream_ids[1] = 0x1881;
        config.stream_id_count = 2;
        config.max_cmd_code = 10;
        config.min_length = 8;
        config.max_length = 256;
        config
    }

    #[test]
    fn test_valid_command() {
        let config = test_config();
        let header = CommandHeader {
            stream_id: 0x1880,
            sequence: 1,
            length: 64,
            function_code: 5,
            checksum: 0,
        };
        assert_eq!(validate_header(&config, &header), CiValidation::Valid);
    }

    #[test]
    fn test_bad_stream_id() {
        let config = test_config();
        let header = CommandHeader {
            stream_id: 0xFFFF,
            sequence: 1,
            length: 64,
            function_code: 5,
            checksum: 0,
        };
        assert_eq!(validate_header(&config, &header), CiValidation::InvalidStreamId);
    }

    #[test]
    fn test_bad_checksum() {
        let config = test_config();
        let header = CommandHeader {
            stream_id: 0x1880,
            sequence: 1,
            length: 64,
            function_code: 5,
            checksum: 0xAB,
        };
        assert_eq!(validate_header(&config, &header), CiValidation::BadChecksum);
    }

    #[test]
    fn test_invalid_cmd_code() {
        let config = test_config();
        let header = CommandHeader {
            stream_id: 0x1880,
            sequence: 1,
            length: 64,
            function_code: 99,
            checksum: 0,
        };
        assert_eq!(validate_header(&config, &header), CiValidation::InvalidCmdCode);
    }

    #[test]
    fn test_length_mismatch() {
        let config = test_config();
        let header = CommandHeader {
            stream_id: 0x1880,
            sequence: 1,
            length: 2,
            function_code: 5,
            checksum: 0,
        };
        assert_eq!(validate_header(&config, &header), CiValidation::LengthMismatch);
    }

    #[test]
    fn test_valid_stream_id_lookup() {
        let config = test_config();
        assert!(is_valid_stream_id(&config, 0x1880));
        assert!(is_valid_stream_id(&config, 0x1881));
        assert!(!is_valid_stream_id(&config, 0x0000));
        assert!(!is_valid_stream_id(&config, 0xFFFF));
    }

    #[test]
    fn test_compute_checksum_xor() {
        let data = [0x01u8, 0x02, 0x03];
        assert_eq!(compute_checksum(&data), 0x01 ^ 0x02 ^ 0x03);
    }

    #[test]
    fn test_compute_checksum_empty() {
        assert_eq!(compute_checksum(&[]), 0);
    }
}
