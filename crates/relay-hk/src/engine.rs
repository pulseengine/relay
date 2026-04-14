//! Relay Housekeeping — verified core logic.
//!
//! Formally verified Rust replacement for NASA cFS Housekeeping (HK).
//! Stream combiner: heartbeats + sensor data → housekeeping packets.
//!
//! Properties verified (Verus SMT/Z3):
//!   HK-P01: Invariant holds after init (table empty, count = 0)
//!   HK-P02: collect never writes past MAX_OUTPUT_SIZE
//!   HK-P03: collect never reads past source data bounds
//!   HK-P04: entry_count bounded by MAX_COPY_ENTRIES
//!
//! Source mapping: NASA cFS HK app (hk_app.c, hk_utils.c)
//! Omitted: cFS message IDs (replaced by source_id), CCSDS headers
//!
//! NO async, NO alloc, NO trait objects, NO closures.

use vstd::prelude::*;

verus! {

/// Maximum number of copy table entries.
pub const MAX_COPY_ENTRIES: usize = 128;

/// Maximum number of data sources.
pub const MAX_SOURCES: usize = 32;

/// Maximum size of an output HK packet in bytes.
pub const MAX_OUTPUT_SIZE: usize = 256;

/// Size of each source data buffer in bytes.
pub const SOURCE_DATA_SIZE: usize = 64;

/// A single entry in the copy table: describes one field to copy.
#[derive(Clone, Copy)]
pub struct CopyEntry {
    /// Which source to read from.
    pub source_id: u32,
    /// Byte offset within the source data.
    pub source_offset: u32,
    /// Number of bytes to copy.
    pub length: u32,
    /// Byte offset in the output packet.
    pub output_offset: u32,
}

/// The Copy Table: a list of field-copy descriptors.
pub struct CopyTable {
    pub entries: [CopyEntry; MAX_COPY_ENTRIES],
    pub entry_count: u32,
}

/// A source data buffer with its identifier.
#[derive(Clone, Copy)]
pub struct SourceData {
    pub source_id: u32,
    pub data: [u8; SOURCE_DATA_SIZE],
}

/// An assembled housekeeping packet.
pub struct HkPacket {
    pub data: [u8; MAX_OUTPUT_SIZE],
    pub length: u32,
    pub sequence: u32,
}

impl CopyEntry {
    pub const fn empty() -> Self {
        CopyEntry {
            source_id: 0,
            source_offset: 0,
            length: 0,
            output_offset: 0,
        }
    }
}

impl SourceData {
    pub const fn empty() -> Self {
        SourceData {
            source_id: 0,
            data: [0u8; SOURCE_DATA_SIZE],
        }
    }
}

impl HkPacket {
    #[verifier::external_body]
    pub fn new() -> (result: Self)
        ensures
            result.length == 0,
    {
        HkPacket {
            data: [0u8; MAX_OUTPUT_SIZE],
            length: 0,
            sequence: 0,
        }
    }
}

impl CopyTable {
    // =================================================================
    // Specification functions
    // =================================================================

    /// The fundamental copy table invariant (HK-P01, HK-P04).
    pub open spec fn inv(&self) -> bool {
        &&& self.entry_count as usize <= MAX_COPY_ENTRIES
    }

    /// Ghost view: number of entries.
    pub open spec fn count_spec(&self) -> nat {
        self.entry_count as nat
    }

    /// Ghost view: is the table full?
    pub open spec fn is_full_spec(&self) -> bool {
        self.entry_count as usize >= MAX_COPY_ENTRIES
    }

    // =================================================================
    // init (HK-P01)
    // =================================================================

    /// Create an empty copy table (HK-P01).
    #[verifier::external_body]
    pub fn new() -> (result: Self)
        ensures
            result.inv(),
            result.count_spec() == 0,
            !result.is_full_spec(),
    {
        CopyTable {
            entries: [CopyEntry::empty(); MAX_COPY_ENTRIES],
            entry_count: 0,
        }
    }

    // =================================================================
    // add_entry (HK-P04)
    // =================================================================

    /// Add an entry to the copy table.
    /// Returns true on success, false if table is full.
    pub fn add_entry(&mut self, entry: CopyEntry) -> (result: bool)
        requires
            old(self).inv(),
        ensures
            self.inv(),
            // HK-P04: returns false iff table was full
            result == !old(self).is_full_spec(),
            result ==> self.count_spec() == old(self).count_spec() + 1,
            !result ==> self.count_spec() == old(self).count_spec(),
    {
        if self.entry_count as usize >= MAX_COPY_ENTRIES {
            return false;
        }
        self.entries.set(self.entry_count as usize, entry);
        self.entry_count = self.entry_count + 1;
        true
    }

    // =================================================================
    // entry_count
    // =================================================================

    /// Get current entry count.
    pub fn entry_count(&self) -> (result: u32)
        requires
            self.inv(),
        ensures
            result == self.entry_count,
            result as usize <= MAX_COPY_ENTRIES,
    {
        self.entry_count
    }

    // =================================================================
    // collect (HK-P02, HK-P03)
    // =================================================================

    /// Collect data from sources into a housekeeping packet.
    ///
    /// For each copy entry: find the matching source, copy the specified
    /// bytes from source[offset..offset+length] to packet[output_offset..].
    /// All accesses are bounds-checked.
    ///
    /// Returns false if any bounds check fails or a source is not found.
    pub fn collect(
        &self,
        sources: &[SourceData],
        packet: &mut HkPacket,
    ) -> (result: bool)
        requires
            self.inv(),
            old(packet).length as usize <= MAX_OUTPUT_SIZE,
        ensures
            // HK-P02: packet length never exceeds MAX_OUTPUT_SIZE
            packet.length as usize <= MAX_OUTPUT_SIZE,
    {
        let count = self.entry_count;
        let mut i: u32 = 0;

        while i < count
            invariant
                self.inv(),
                0 <= i <= count,
                count == self.entry_count,
                count as usize <= MAX_COPY_ENTRIES,
                packet.length as usize <= MAX_OUTPUT_SIZE,
            decreases
                count - i,
        {
            let entry = self.entries[i as usize];

            // Guard against usize overflow on output offset + length
            if entry.output_offset as usize > MAX_OUTPUT_SIZE || entry.length as usize > MAX_OUTPUT_SIZE {
                return false;
            }

            // HK-P02: output bounds check
            let out_end = entry.output_offset as usize + entry.length as usize;
            if out_end > MAX_OUTPUT_SIZE {
                return false;
            }

            // Guard against usize overflow on source offset + length
            if entry.source_offset as usize > SOURCE_DATA_SIZE || entry.length as usize > SOURCE_DATA_SIZE {
                return false;
            }

            // HK-P03: source bounds check
            let src_end = entry.source_offset as usize + entry.length as usize;
            if src_end > SOURCE_DATA_SIZE {
                return false;
            }

            // Find matching source
            let mut found: bool = false;
            let mut s: usize = 0;

            let out_start = entry.output_offset as usize;
            let src_start = entry.source_offset as usize;

            while s < sources.len()
                invariant
                    0 <= s <= sources.len(),
                    packet.length as usize <= MAX_OUTPUT_SIZE,
                    out_end <= MAX_OUTPUT_SIZE,
                    src_end <= SOURCE_DATA_SIZE,
                    out_start <= out_end,
                    src_start <= src_end,
                    out_end - out_start == src_end - src_start,
                decreases
                    sources.len() - s,
            {
                if sources[s].source_id == entry.source_id {
                    let copy_len = out_end - out_start;
                    let mut idx: usize = 0;
                    while idx < copy_len
                        invariant
                            0 <= idx <= copy_len,
                            out_start + copy_len <= MAX_OUTPUT_SIZE,
                            src_start + copy_len <= SOURCE_DATA_SIZE,
                            copy_len == out_end - out_start,
                            s < sources.len(),
                            packet.length as usize <= MAX_OUTPUT_SIZE,
                        decreases
                            copy_len - idx,
                    {
                        packet.data.set(
                            out_start + idx,
                            sources[s].data[src_start + idx],
                        );
                        idx = idx + 1;
                    }
                    found = true;
                    break;
                }
                s = s + 1;
            }

            if !found {
                return false;
            }

            // Track high-water mark for packet length
            if out_end as u32 > packet.length {
                packet.length = out_end as u32;
            }

            i = i + 1;
        }

        if packet.sequence < u32::MAX {
            packet.sequence = packet.sequence + 1;
        }
        true
    }
}

// =================================================================
// Compositional proofs
// =================================================================

// HK-P01: init establishes invariant — proven by new()'s ensures clause.
// (Proof functions cannot call exec functions; new()'s postcondition
//  guarantees inv() directly.)

// HK-P03: The invariant is inductive across all operations.
// init establishes inv (from new's ensures)
// add_entry preserves inv (from add_entry's ensures)
// collect preserves inv (read-only on &self)

} // verus!

// ── Tests (run on plain Rust via verus-strip) ────────────────

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
        assert_eq!(packet.data[3], 0xDD);
    }

    #[test]
    fn test_output_bounds_check() {
        let mut table = CopyTable::new();
        table.add_entry(CopyEntry {
            source_id: 1, source_offset: 0, length: 4,
            output_offset: (MAX_OUTPUT_SIZE - 2) as u32,
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
            source_id: 99, source_offset: 0, length: 1, output_offset: 0,
        });
        let sources: [SourceData; 0] = [];
        let mut packet = HkPacket::new();
        assert!(!table.collect(&sources, &mut packet));
    }
}
