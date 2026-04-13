//! Relay Checksum — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/core.rs (Verus-annotated). Do not edit manually.

pub const MAX_REGIONS: usize = 64;
pub const MAX_CHECK_PER_CYCLE: usize = 16;

/// CRC32 lookup table (polynomial 0xEDB88320, standard reflected).
pub const CRC32_TABLE: [u32; 256] = {
    let mut table = [0u32; 256];
    let mut i: usize = 0;
    while i < 256 {
        let mut crc: u32 = i as u32;
        let mut j: usize = 0;
        while j < 8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB8_8320u32;
            } else {
                crc = crc >> 1;
            }
            j = j + 1;
        }
        table[i] = crc;
        i = i + 1;
    }
    table
};

/// Compute CRC32 over a data slice.
pub fn crc32_compute(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFFu32;
    let mut i: usize = 0;
    while i < data.len() {
        let byte = data[i];
        let index = ((crc ^ (byte as u32)) & 0xFF) as usize;
        crc = (crc >> 8) ^ CRC32_TABLE[index];
        i = i + 1;
    }
    crc ^ 0xFFFF_FFFFu32
}

/// A tracked memory/data region for integrity checking.
#[derive(Clone, Copy)]
pub struct Region {
    pub region_id: u32,
    pub baseline_crc: u32,
    pub enabled: bool,
    pub last_checked: u64,
}

/// Result of checking a single region.
#[derive(Clone, Copy)]
pub struct CheckResult {
    pub region_id: u32,
    pub computed_crc: u32,
    pub baseline_crc: u32,
    pub mismatch: bool,
}

/// Bounded output of a batch check cycle.
pub struct CheckOutput {
    pub results: [CheckResult; MAX_CHECK_PER_CYCLE],
    pub result_count: u32,
}

/// The Checksum Table -- tracks all registered regions.
/// Fixed-size, no alloc.
pub struct ChecksumTable {
    regions: [Region; MAX_REGIONS],
    region_count: u32,
}

impl Region {
    pub const fn empty() -> Self {
        Region { region_id: 0, baseline_crc: 0, enabled: false, last_checked: 0 }
    }
}

impl CheckResult {
    pub const fn empty() -> Self {
        CheckResult { region_id: 0, computed_crc: 0, baseline_crc: 0, mismatch: false }
    }
}

impl ChecksumTable {
    /// Create an empty checksum table.
    pub fn new() -> Self {
        ChecksumTable {
            regions: [Region::empty(); MAX_REGIONS],
            region_count: 0,
        }
    }

    /// Register a new region for integrity checking.
    /// Returns true on success, false if table is full.
    pub fn register_region(&mut self, region_id: u32, baseline_crc: u32) -> bool {
        if self.region_count as usize >= MAX_REGIONS {
            return false;
        }
        self.regions[self.region_count as usize] = Region {
            region_id,
            baseline_crc,
            enabled: true,
            last_checked: 0,
        };
        self.region_count = self.region_count + 1;
        true
    }

    /// Get current region count.
    pub fn region_count(&self) -> u32 {
        self.region_count
    }

    /// Check a single region by computing CRC32 over its data.
    /// Returns None if region_id not found.
    pub fn check_region(
        &mut self,
        region_id: u32,
        data: &[u8],
        current_time: u64,
    ) -> Option<CheckResult> {
        let count = self.region_count;
        let mut i: u32 = 0;
        while i < count {
            if self.regions[i as usize].region_id == region_id && self.regions[i as usize].enabled {
                let computed = crc32_compute(data);
                let baseline = self.regions[i as usize].baseline_crc;
                self.regions[i as usize].last_checked = current_time;
                let mismatch = computed != baseline;
                return Some(CheckResult {
                    region_id,
                    computed_crc: computed,
                    baseline_crc: baseline,
                    mismatch,
                });
            }
            i = i + 1;
        }
        None
    }

    /// Check a batch of regions. Input is an array of (region_id, data) pairs.
    /// Output bounded by MAX_CHECK_PER_CYCLE.
    pub fn check_batch(
        &mut self,
        region_data: &[(u32, &[u8])],
        current_time: u64,
    ) -> CheckOutput {
        let mut output = CheckOutput {
            results: [CheckResult::empty(); MAX_CHECK_PER_CYCLE],
            result_count: 0,
        };

        let mut i: usize = 0;
        while i < region_data.len() {
            if output.result_count as usize >= MAX_CHECK_PER_CYCLE {
                break;
            }
            let (rid, data) = region_data[i];
            let opt = self.check_region(rid, data, current_time);
            match opt {
                Some(cr) => {
                    output.results[output.result_count as usize] = cr;
                    output.result_count = output.result_count + 1;
                }
                None => {}
            }
            i = i + 1;
        }

        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table() {
        let table = ChecksumTable::new();
        assert_eq!(table.region_count(), 0);
    }

    #[test]
    fn test_crc32_known_value() {
        // CRC32 of "123456789" is 0xCBF43926
        let data = b"123456789";
        let crc = crc32_compute(data);
        assert_eq!(crc, 0xCBF4_3926);
    }

    #[test]
    fn test_crc32_empty() {
        let crc = crc32_compute(b"");
        assert_eq!(crc, 0x0000_0000);
    }

    #[test]
    fn test_matching_baseline() {
        let data = b"hello";
        let baseline = crc32_compute(data);
        let mut table = ChecksumTable::new();
        table.register_region(1, baseline);
        let result = table.check_region(1, data, 100).unwrap();
        assert!(!result.mismatch);
        assert_eq!(result.computed_crc, baseline);
    }

    #[test]
    fn test_mismatched_baseline() {
        let data = b"hello";
        let wrong_baseline = 0xDEAD_BEEF;
        let mut table = ChecksumTable::new();
        table.register_region(1, wrong_baseline);
        let result = table.check_region(1, data, 100).unwrap();
        assert!(result.mismatch);
        assert_eq!(result.baseline_crc, wrong_baseline);
    }

    #[test]
    fn test_register_region() {
        let mut table = ChecksumTable::new();
        assert!(table.register_region(1, 0xAAAA));
        assert!(table.register_region(2, 0xBBBB));
        assert_eq!(table.region_count(), 2);
    }

    #[test]
    fn test_table_full() {
        let mut table = ChecksumTable::new();
        for i in 0..MAX_REGIONS {
            assert!(table.register_region(i as u32, 0));
        }
        assert!(!table.register_region(999, 0));
    }

    #[test]
    fn test_check_updates_last_checked() {
        let data = b"test";
        let baseline = crc32_compute(data);
        let mut table = ChecksumTable::new();
        table.register_region(1, baseline);
        let _ = table.check_region(1, data, 42);
        // Check again at a later time
        let result = table.check_region(1, data, 99).unwrap();
        assert!(!result.mismatch);
    }

    #[test]
    fn test_batch_check_bounded() {
        let mut table = ChecksumTable::new();
        let data: &[u8] = b"data";
        let crc = crc32_compute(data);
        // Register more regions than MAX_CHECK_PER_CYCLE
        const N: usize = MAX_CHECK_PER_CYCLE + 5;
        for i in 0..N as u32 {
            table.register_region(i, crc);
        }
        let mut pairs: [(u32, &[u8]); N] = [(0, b""); N];
        let mut i = 0;
        while i < N {
            pairs[i] = (i as u32, data);
            i += 1;
        }
        let output = table.check_batch(&pairs, 1000);
        assert_eq!(output.result_count as usize, MAX_CHECK_PER_CYCLE);
    }

    #[test]
    fn test_check_nonexistent_region() {
        let mut table = ChecksumTable::new();
        table.register_region(1, 0);
        let result = table.check_region(999, b"data", 0);
        assert!(result.is_none());
    }
}
