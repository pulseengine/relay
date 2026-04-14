//! Relay Memory Dwell — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_DWELL_ENTRIES: usize = 32;
pub const MAX_SAMPLES_PER_CYCLE: usize = 16;

#[derive(Clone, Copy)]
pub struct DwellEntry {
    pub address: u32,
    pub size: u8,
    pub rate_divisor: u32,
    pub enabled: bool,
}

#[derive(Clone, Copy)]
pub struct DwellRequest {
    pub address: u32,
    pub size: u8,
}

pub struct DwellResult {
    pub requests: [DwellRequest; MAX_SAMPLES_PER_CYCLE],
    pub request_count: u32,
}

impl DwellEntry {
    pub const fn empty() -> Self {
        DwellEntry { address: 0, size: 0, rate_divisor: 1, enabled: false }
    }
}

impl DwellRequest {
    pub const fn empty() -> Self {
        DwellRequest { address: 0, size: 0 }
    }
}

pub struct DwellTable {
    entries: [DwellEntry; MAX_DWELL_ENTRIES],
    entry_count: u32,
}

impl DwellTable {
    pub fn new() -> Self {
        DwellTable {
            entries: [DwellEntry::empty(); MAX_DWELL_ENTRIES],
            entry_count: 0,
        }
    }

    /// Add a dwell entry. Returns true on success, false if table full.
    pub fn add_entry(&mut self, entry: DwellEntry) -> bool {
        if self.entry_count as usize >= MAX_DWELL_ENTRIES {
            return false;
        }
        let idx = self.entry_count as usize;
        self.entries[idx] = entry;
        self.entry_count = self.entry_count + 1;
        true
    }

    /// Compute which dwell addresses to sample this cycle.
    /// For each enabled entry: if cycle_count % rate_divisor == 0, add to requests.
    pub fn get_samples(&self, cycle_count: u32) -> DwellResult {
        let mut result = DwellResult {
            requests: [DwellRequest::empty(); MAX_SAMPLES_PER_CYCLE],
            request_count: 0,
        };
        let count = self.entry_count;
        let mut i: u32 = 0;

        while i < count {
            if result.request_count as usize >= MAX_SAMPLES_PER_CYCLE {
                break;
            }

            let idx = i as usize;
            let entry = self.entries[idx];

            if entry.enabled && entry.rate_divisor > 0 {
                let remainder = cycle_count % entry.rate_divisor;
                if remainder == 0 {
                    let ridx = result.request_count as usize;
                    result.requests[ridx] = DwellRequest {
                        address: entry.address,
                        size: entry.size,
                    };
                    result.request_count = result.request_count + 1;
                }
            }

            i = i + 1;
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table() {
        let table = DwellTable::new();
        let result = table.get_samples(0);
        assert_eq!(result.request_count, 0);
    }

    #[test]
    fn test_single_dwell() {
        let mut table = DwellTable::new();
        table.add_entry(DwellEntry {
            address: 0x2000_0000,
            size: 4,
            rate_divisor: 1,
            enabled: true,
        });
        let result = table.get_samples(0);
        assert_eq!(result.request_count, 1);
        assert_eq!(result.requests[0].address, 0x2000_0000);
        assert_eq!(result.requests[0].size, 4);
    }

    #[test]
    fn test_rate_divisor_filtering() {
        let mut table = DwellTable::new();
        table.add_entry(DwellEntry {
            address: 0x2000_0000,
            size: 4,
            rate_divisor: 4,
            enabled: true,
        });
        // cycle 0: 0 % 4 == 0 => sampled
        assert_eq!(table.get_samples(0).request_count, 1);
        // cycle 1: 1 % 4 != 0 => not sampled
        assert_eq!(table.get_samples(1).request_count, 0);
        // cycle 2: 2 % 4 != 0 => not sampled
        assert_eq!(table.get_samples(2).request_count, 0);
        // cycle 4: 4 % 4 == 0 => sampled
        assert_eq!(table.get_samples(4).request_count, 1);
    }

    #[test]
    fn test_disabled_entry() {
        let mut table = DwellTable::new();
        table.add_entry(DwellEntry {
            address: 0x2000_0000,
            size: 4,
            rate_divisor: 1,
            enabled: false,
        });
        assert_eq!(table.get_samples(0).request_count, 0);
    }

    #[test]
    fn test_bounded_output() {
        let mut table = DwellTable::new();
        // Add more entries than MAX_SAMPLES_PER_CYCLE
        for i in 0..(MAX_SAMPLES_PER_CYCLE + 5) {
            table.add_entry(DwellEntry {
                address: (0x2000_0000 + i * 4) as u32,
                size: 4,
                rate_divisor: 1,
                enabled: true,
            });
        }
        let result = table.get_samples(0);
        assert_eq!(result.request_count as usize, MAX_SAMPLES_PER_CYCLE);
    }

    #[test]
    fn test_multiple_entries() {
        let mut table = DwellTable::new();
        table.add_entry(DwellEntry {
            address: 0x1000,
            size: 1,
            rate_divisor: 1,
            enabled: true,
        });
        table.add_entry(DwellEntry {
            address: 0x2000,
            size: 2,
            rate_divisor: 2,
            enabled: true,
        });
        table.add_entry(DwellEntry {
            address: 0x3000,
            size: 4,
            rate_divisor: 3,
            enabled: true,
        });
        // cycle 0: all three fire (0 % 1 == 0, 0 % 2 == 0, 0 % 3 == 0)
        assert_eq!(table.get_samples(0).request_count, 3);
        // cycle 1: only first fires (1 % 1 == 0)
        assert_eq!(table.get_samples(1).request_count, 1);
        // cycle 6: all three fire (6 % 1 == 0, 6 % 2 == 0, 6 % 3 == 0)
        assert_eq!(table.get_samples(6).request_count, 3);
    }

    #[test]
    fn test_table_full() {
        let mut table = DwellTable::new();
        for i in 0..MAX_DWELL_ENTRIES {
            assert!(table.add_entry(DwellEntry {
                address: i as u32,
                size: 1,
                rate_divisor: 1,
                enabled: true,
            }));
        }
        assert!(!table.add_entry(DwellEntry {
            address: 999,
            size: 1,
            rate_divisor: 1,
            enabled: true,
        }));
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// MD-P01: request_count never exceeds MAX_SAMPLES_PER_CYCLE
    #[kani::proof]
    fn verify_sample_bounded() {
        let mut table = DwellTable::new();
        let address: u32 = kani::any();
        let size: u8 = kani::any();
        let rate_divisor: u32 = kani::any();
        kani::assume(rate_divisor >= 1);
        table.add_entry(DwellEntry {
            address,
            size,
            rate_divisor,
            enabled: true,
        });
        let cycle: u32 = kani::any();
        let result = table.get_samples(cycle);
        assert!(result.request_count as usize <= MAX_SAMPLES_PER_CYCLE);
    }

    /// MD-P02: no panics for any symbolic input
    #[kani::proof]
    fn verify_no_panic() {
        let mut table = DwellTable::new();
        let address: u32 = kani::any();
        let size: u8 = kani::any();
        let rate_divisor: u32 = kani::any();
        kani::assume(rate_divisor >= 1);
        let enabled: bool = kani::any();
        table.add_entry(DwellEntry {
            address,
            size,
            rate_divisor,
            enabled,
        });
        let cycle: u32 = kani::any();
        let _ = table.get_samples(cycle);
    }
}
