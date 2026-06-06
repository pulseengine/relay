//! Relay Memory Manager — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum MmOperation {
    Peek = 0,
    Poke = 1,
    LoadFromFile = 2,
    DumpToFile = 3,
    Fill = 4,
}

#[derive(Clone, Copy)]
pub struct MmRequest {
    pub operation: MmOperation,
    pub address: u32,
    pub size: u32,
    pub value: u64,
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum MmValidation {
    Valid = 0,
    AddressOutOfRange = 1,
    SizeTooLarge = 2,
    SizeZero = 3,
    AlignmentError = 4,
}

#[derive(Clone, Copy)]
pub struct MmConfig {
    pub ram_start: u32,
    pub ram_end: u32,
    pub max_operation_size: u32,
}

impl MmRequest {
    pub const fn empty() -> Self {
        MmRequest {
            operation: MmOperation::Peek,
            address: 0,
            size: 0,
            value: 0,
        }
    }
}

/// Check alignment -- address must be aligned to size for peek/poke.
pub fn is_aligned(address: u32, size: u32) -> bool {
    if size <= 1 {
        true
    } else {
        (address % size) == 0
    }
}

/// Validate a memory operation request against the configuration.
pub fn validate_request(config: &MmConfig, req: &MmRequest) -> MmValidation {
    // Check size > 0
    if req.size == 0 {
        return MmValidation::SizeZero;
    }

    // Check size <= max
    if req.size > config.max_operation_size {
        return MmValidation::SizeTooLarge;
    }

    // Check address in range
    if req.address < config.ram_start || req.address >= config.ram_end {
        return MmValidation::AddressOutOfRange;
    }

    // Check end address doesn't overflow past RAM end
    let end_addr: u64 = req.address as u64 + req.size as u64;
    if end_addr > config.ram_end as u64 {
        return MmValidation::AddressOutOfRange;
    }

    // Check alignment for peek/poke operations
    match req.operation {
        MmOperation::Peek | MmOperation::Poke => {
            if !is_aligned(req.address, req.size) {
                return MmValidation::AlignmentError;
            }
        },
        _ => {},
    }

    MmValidation::Valid
}

#[cfg(test)]
mod tests {
    use super::*;

    fn test_config() -> MmConfig {
        MmConfig {
            ram_start: 0x2000_0000,
            ram_end: 0x2001_0000,
            max_operation_size: 4096,
        }
    }

    #[test]
    fn test_valid_request() {
        let config = test_config();
        let req = MmRequest {
            operation: MmOperation::Peek,
            address: 0x2000_0000,
            size: 4,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::Valid);
    }

    #[test]
    fn test_out_of_range() {
        let config = test_config();
        let req = MmRequest {
            operation: MmOperation::Peek,
            address: 0x1000_0000,
            size: 4,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::AddressOutOfRange);
    }

    #[test]
    fn test_size_too_large() {
        let config = test_config();
        let req = MmRequest {
            operation: MmOperation::Poke,
            address: 0x2000_0000,
            size: 8192,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::SizeTooLarge);
    }

    #[test]
    fn test_size_zero() {
        let config = test_config();
        let req = MmRequest {
            operation: MmOperation::Peek,
            address: 0x2000_0000,
            size: 0,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::SizeZero);
    }

    #[test]
    fn test_alignment_error() {
        let config = test_config();
        let req = MmRequest {
            operation: MmOperation::Peek,
            address: 0x2000_0003,
            size: 4,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::AlignmentError);
    }

    #[test]
    fn test_boundary_address() {
        let config = test_config();
        // Address at very end of range: ram_end - size should be valid
        let req = MmRequest {
            operation: MmOperation::Peek,
            address: 0x2000_FFFC,
            size: 4,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::Valid);

        // Address at ram_end itself should be out of range
        let req2 = MmRequest {
            operation: MmOperation::Peek,
            address: 0x2001_0000,
            size: 4,
            value: 0,
        };
        assert_eq!(validate_request(&config, &req2), MmValidation::AddressOutOfRange);
    }

    #[test]
    fn test_fill_no_alignment_check() {
        let config = test_config();
        // Fill operations don't require alignment
        let req = MmRequest {
            operation: MmOperation::Fill,
            address: 0x2000_0001,
            size: 4,
            value: 0xFF,
        };
        assert_eq!(validate_request(&config, &req), MmValidation::Valid);
    }

    #[test]
    fn test_is_aligned() {
        assert!(is_aligned(0x1000, 4));
        assert!(!is_aligned(0x1001, 4));
        assert!(is_aligned(0x1000, 1));
        assert!(is_aligned(0x1001, 1));
        assert!(is_aligned(0x0, 8));
    }
}
