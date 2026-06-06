//! Relay Data Storage — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_FILTERS: usize = 64;
pub const MAX_DESTINATIONS: usize = 8;
pub const MAX_DECISIONS_PER_CHECK: usize = 16;

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum FileType { Sequence = 0, Time = 1, Count = 2 }

#[derive(Clone, Copy)]
pub struct FilterEntry {
    pub data_id: u32,
    pub destination: u32,
    pub enabled: bool,
    pub file_type: FileType,
}

#[derive(Clone, Copy)]
pub struct StorageDecision {
    pub data_id: u32,
    pub destination: u32,
    pub file_type: FileType,
}

pub struct FilterTable {
    filters: [FilterEntry; MAX_FILTERS],
    filter_count: u32,
}

pub struct FilterResult {
    pub decisions: [StorageDecision; MAX_DECISIONS_PER_CHECK],
    pub decision_count: u32,
}

impl FilterEntry {
    pub const fn empty() -> Self {
        FilterEntry {
            data_id: 0,
            destination: 0,
            enabled: false,
            file_type: FileType::Sequence,
        }
    }
}

impl StorageDecision {
    pub const fn empty() -> Self {
        StorageDecision {
            data_id: 0,
            destination: 0,
            file_type: FileType::Sequence,
        }
    }
}

impl FilterTable {
    pub fn new() -> Self {
        FilterTable {
            filters: [FilterEntry::empty(); MAX_FILTERS],
            filter_count: 0,
        }
    }

    pub fn add_filter(&mut self, entry: FilterEntry) -> bool {
        if self.filter_count as usize >= MAX_FILTERS { return false; }
        let idx = self.filter_count as usize;
        self.filters[idx] = entry;
        self.filter_count = self.filter_count + 1;
        true
    }

    pub fn filter_count(&self) -> u32 { self.filter_count }

    pub fn evaluate(&self, data_id: u32) -> FilterResult {
        let mut result = FilterResult {
            decisions: [StorageDecision::empty(); MAX_DECISIONS_PER_CHECK],
            decision_count: 0,
        };

        let count = self.filter_count;
        let mut i: u32 = 0;
        while i < count {
            if result.decision_count as usize >= MAX_DECISIONS_PER_CHECK { break; }
            let idx = i as usize;
            let f = self.filters[idx];

            if f.enabled && f.data_id == data_id {
                let didx = result.decision_count as usize;
                result.decisions[didx] = StorageDecision {
                    data_id: f.data_id,
                    destination: f.destination,
                    file_type: f.file_type,
                };
                result.decision_count = result.decision_count + 1;
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
        let t = FilterTable::new();
        let r = t.evaluate(42);
        assert_eq!(r.decision_count, 0);
    }

    #[test]
    fn test_matching_filter() {
        let mut t = FilterTable::new();
        t.add_filter(FilterEntry {
            data_id: 100,
            destination: 1,
            enabled: true,
            file_type: FileType::Time,
        });
        let r = t.evaluate(100);
        assert_eq!(r.decision_count, 1);
        assert_eq!(r.decisions[0].data_id, 100);
        assert_eq!(r.decisions[0].destination, 1);
        assert!(r.decisions[0].file_type == FileType::Time);
    }

    #[test]
    fn test_no_match() {
        let mut t = FilterTable::new();
        t.add_filter(FilterEntry {
            data_id: 100,
            destination: 1,
            enabled: true,
            file_type: FileType::Sequence,
        });
        let r = t.evaluate(999);
        assert_eq!(r.decision_count, 0);
    }

    #[test]
    fn test_disabled_filter() {
        let mut t = FilterTable::new();
        t.add_filter(FilterEntry {
            data_id: 100,
            destination: 1,
            enabled: false,
            file_type: FileType::Sequence,
        });
        let r = t.evaluate(100);
        assert_eq!(r.decision_count, 0);
    }

    #[test]
    fn test_multiple_destinations() {
        let mut t = FilterTable::new();
        t.add_filter(FilterEntry {
            data_id: 42,
            destination: 1,
            enabled: true,
            file_type: FileType::Sequence,
        });
        t.add_filter(FilterEntry {
            data_id: 42,
            destination: 2,
            enabled: true,
            file_type: FileType::Time,
        });
        t.add_filter(FilterEntry {
            data_id: 42,
            destination: 3,
            enabled: true,
            file_type: FileType::Count,
        });
        let r = t.evaluate(42);
        assert_eq!(r.decision_count, 3);
        assert_eq!(r.decisions[0].destination, 1);
        assert_eq!(r.decisions[1].destination, 2);
        assert_eq!(r.decisions[2].destination, 3);
    }

    #[test]
    fn test_bounded_output() {
        let mut t = FilterTable::new();
        for i in 0..(MAX_DECISIONS_PER_CHECK as u32 + 10) {
            t.add_filter(FilterEntry {
                data_id: 1,
                destination: i,
                enabled: true,
                file_type: FileType::Sequence,
            });
        }
        let r = t.evaluate(1);
        assert_eq!(r.decision_count, MAX_DECISIONS_PER_CHECK as u32);
    }

    #[test]
    fn test_table_full() {
        let mut t = FilterTable::new();
        for i in 0..MAX_FILTERS as u32 {
            assert!(t.add_filter(FilterEntry {
                data_id: i,
                destination: 0,
                enabled: true,
                file_type: FileType::Sequence,
            }));
        }
        assert!(!t.add_filter(FilterEntry::empty()));
    }

    #[test]
    fn test_filter_count_bounded() {
        let mut t = FilterTable::new();
        for _ in 0..10 {
            t.add_filter(FilterEntry {
                data_id: 1,
                destination: 0,
                enabled: true,
                file_type: FileType::Sequence,
            });
        }
        assert_eq!(t.filter_count(), 10);
        assert!(t.filter_count() as usize <= MAX_FILTERS);
    }
}
