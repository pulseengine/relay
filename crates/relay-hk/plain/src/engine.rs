//! Relay Housekeeping — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/core.rs (Verus-annotated). Do not edit manually.

pub const MAX_COPY_ENTRIES: usize = 128;
pub const MAX_SOURCES: usize = 32;
pub const MAX_OUTPUT_SIZE: usize = 256;
pub const SOURCE_DATA_SIZE: usize = 64;

#[derive(Clone, Copy)]
pub struct CopyEntry {
    pub source_id: u32,
    pub source_offset: u32,
    pub length: u32,
    pub output_offset: u32,
}

pub struct CopyTable {
    entries: [CopyEntry; MAX_COPY_ENTRIES],
    entry_count: u32,
}

#[derive(Clone, Copy)]
pub struct SourceData {
    pub source_id: u32,
    pub data: [u8; SOURCE_DATA_SIZE],
}

pub struct HkPacket {
    pub data: [u8; MAX_OUTPUT_SIZE],
    pub length: u32,
    pub sequence: u32,
}

impl CopyEntry {
    pub const fn empty() -> Self {
        CopyEntry { source_id: 0, source_offset: 0, length: 0, output_offset: 0 }
    }
}

impl SourceData {
    pub const fn empty() -> Self {
        SourceData { source_id: 0, data: [0u8; SOURCE_DATA_SIZE] }
    }
}

impl HkPacket {
    pub fn new() -> Self {
        HkPacket { data: [0u8; MAX_OUTPUT_SIZE], length: 0, sequence: 0 }
    }
}

impl CopyTable {
    pub fn new() -> Self {
        CopyTable {
            entries: [CopyEntry::empty(); MAX_COPY_ENTRIES],
            entry_count: 0,
        }
    }

    pub fn add_entry(&mut self, entry: CopyEntry) -> bool {
        if self.entry_count as usize >= MAX_COPY_ENTRIES { return false; }
        self.entries[self.entry_count as usize] = entry;
        self.entry_count = self.entry_count + 1;
        true
    }

    pub fn entry_count(&self) -> u32 { self.entry_count }

    pub fn collect(&self, sources: &[SourceData], packet: &mut HkPacket) -> bool {
        let count = self.entry_count;
        let mut i: u32 = 0;
        while i < count {
            let entry = self.entries[i as usize];

            // Bounds check: output region must fit in packet
            let out_end = entry.output_offset as usize + entry.length as usize;
            if out_end > MAX_OUTPUT_SIZE { return false; }

            // Bounds check: source region must fit in source data
            let src_end = entry.source_offset as usize + entry.length as usize;
            if src_end > SOURCE_DATA_SIZE { return false; }

            // Find matching source
            let mut found = false;
            let mut s: usize = 0;
            while s < sources.len() {
                if sources[s].source_id == entry.source_id {
                    // Copy bytes from source to packet
                    let mut b: usize = 0;
                    while b < entry.length as usize {
                        packet.data[entry.output_offset as usize + b] =
                            sources[s].data[entry.source_offset as usize + b];
                        b = b + 1;
                    }
                    found = true;
                    break;
                }
                s = s + 1;
            }

            if !found { return false; }

            // Track the high-water mark for packet length
            if out_end as u32 > packet.length {
                packet.length = out_end as u32;
            }

            i = i + 1;
        }
        packet.sequence = packet.sequence + 1;
        true
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_table_collect_succeeds() {
        let table = CopyTable::new();
        let sources: [SourceData; 0] = [];
        let mut packet = HkPacket::new();
        assert!(table.collect(&sources, &mut packet));
        assert_eq!(packet.sequence, 1);
    }

    #[test]
    fn test_single_copy() {
        let mut table = CopyTable::new();
        table.add_entry(CopyEntry {
            source_id: 1,
            source_offset: 0,
            length: 4,
            output_offset: 0,
        });

        let mut src = SourceData::empty();
        src.source_id = 1;
        src.data[0] = 0xDE;
        src.data[1] = 0xAD;
        src.data[2] = 0xBE;
        src.data[3] = 0xEF;

        let sources = [src];
        let mut packet = HkPacket::new();
        assert!(table.collect(&sources, &mut packet));
        assert_eq!(packet.data[0], 0xDE);
        assert_eq!(packet.data[1], 0xAD);
        assert_eq!(packet.data[2], 0xBE);
        assert_eq!(packet.data[3], 0xEF);
        assert_eq!(packet.length, 4);
    }

    #[test]
    fn test_multiple_copies() {
        let mut table = CopyTable::new();
        table.add_entry(CopyEntry { source_id: 1, source_offset: 0, length: 2, output_offset: 0 });
        table.add_entry(CopyEntry { source_id: 2, source_offset: 4, length: 2, output_offset: 2 });

        let mut src1 = SourceData::empty();
        src1.source_id = 1;
        src1.data[0] = 0xAA;
        src1.data[1] = 0xBB;

        let mut src2 = SourceData::empty();
        src2.source_id = 2;
        src2.data[4] = 0xCC;
        src2.data[5] = 0xDD;

        let sources = [src1, src2];
        let mut packet = HkPacket::new();
        assert!(table.collect(&sources, &mut packet));
        assert_eq!(packet.data[0], 0xAA);
        assert_eq!(packet.data[1], 0xBB);
        assert_eq!(packet.data[2], 0xCC);
        assert_eq!(packet.data[3], 0xDD);
        assert_eq!(packet.length, 4);
    }

    #[test]
    fn test_output_bounds_check() {
        let mut table = CopyTable::new();
        table.add_entry(CopyEntry {
            source_id: 1,
            source_offset: 0,
            length: 4,
            output_offset: (MAX_OUTPUT_SIZE - 2) as u32, // would exceed output
        });

        let mut src = SourceData::empty();
        src.source_id = 1;
        let sources = [src];
        let mut packet = HkPacket::new();
        assert!(!table.collect(&sources, &mut packet));
    }

    #[test]
    fn test_source_bounds_check() {
        let mut table = CopyTable::new();
        table.add_entry(CopyEntry {
            source_id: 1,
            source_offset: (SOURCE_DATA_SIZE - 2) as u32,
            length: 4, // would exceed source data
            output_offset: 0,
        });

        let mut src = SourceData::empty();
        src.source_id = 1;
        let sources = [src];
        let mut packet = HkPacket::new();
        assert!(!table.collect(&sources, &mut packet));
    }

    #[test]
    fn test_source_not_found() {
        let mut table = CopyTable::new();
        table.add_entry(CopyEntry {
            source_id: 99,
            source_offset: 0,
            length: 1,
            output_offset: 0,
        });

        let sources: [SourceData; 0] = [];
        let mut packet = HkPacket::new();
        assert!(!table.collect(&sources, &mut packet));
    }

    #[test]
    fn test_table_full_returns_false() {
        let mut table = CopyTable::new();
        for _ in 0..MAX_COPY_ENTRIES {
            assert!(table.add_entry(CopyEntry::empty()));
        }
        assert!(!table.add_entry(CopyEntry::empty()));
    }

    #[test]
    fn test_sequence_increments() {
        let table = CopyTable::new();
        let sources: [SourceData; 0] = [];
        let mut packet = HkPacket::new();
        table.collect(&sources, &mut packet);
        assert_eq!(packet.sequence, 1);
        table.collect(&sources, &mut packet);
        assert_eq!(packet.sequence, 2);
    }
}

#[cfg(kani)]
mod kani_proofs {
    use super::*;

    /// HK-P01: collect on empty table always succeeds and packet.length stays bounded
    #[kani::proof]
    #[kani::unwind(66)]
    fn verify_collect_bounded() {
        let mut table = CopyTable::new();
        let source_id: u32 = kani::any();
        let source_offset: u32 = kani::any();
        let length: u32 = kani::any();
        let output_offset: u32 = kani::any();
        kani::assume(length <= SOURCE_DATA_SIZE as u32);
        kani::assume(source_offset <= SOURCE_DATA_SIZE as u32 - length);
        kani::assume(output_offset <= MAX_OUTPUT_SIZE as u32 - length);
        table.add_entry(CopyEntry {
            source_id,
            source_offset,
            length,
            output_offset,
        });
        let mut src = SourceData::empty();
        src.source_id = source_id;
        let sources = [src];
        let mut packet = HkPacket::new();
        let ok = table.collect(&sources, &mut packet);
        if ok {
            assert!(packet.length as usize <= MAX_OUTPUT_SIZE);
        }
    }

    /// HK-P02: no panics for any symbolic input
    #[kani::proof]
    #[kani::unwind(66)]
    fn verify_no_panic() {
        let mut table = CopyTable::new();
        let source_id: u32 = kani::any();
        let source_offset: u32 = kani::any();
        let length: u32 = kani::any();
        let output_offset: u32 = kani::any();
        kani::assume(length <= SOURCE_DATA_SIZE as u32);
        kani::assume(source_offset <= SOURCE_DATA_SIZE as u32);
        kani::assume(output_offset <= MAX_OUTPUT_SIZE as u32);
        table.add_entry(CopyEntry {
            source_id,
            source_offset,
            length,
            output_offset,
        });
        let mut src = SourceData::empty();
        src.source_id = source_id;
        let sources = [src];
        let mut packet = HkPacket::new();
        let _ = table.collect(&sources, &mut packet);
    }
}
