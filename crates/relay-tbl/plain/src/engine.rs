//! Relay Table Services — plain Rust (generated from Verus source via verus-strip).
//! Source of truth: ../src/engine.rs (Verus-annotated). Do not edit manually.

pub const MAX_TABLES: usize = 32;
pub const MAX_TABLE_SIZE: usize = 256;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
#[repr(u8)]
pub enum TblResult {
    Success = 0,
    NotFound = 1,
    ValidationFailed = 2,
    Full = 3,
    SizeMismatch = 4,
}

#[derive(Clone, Copy)]
pub struct TableEntry {
    pub name_hash: u32,
    pub active_buf: u8,
    pub buffers: [[u8; MAX_TABLE_SIZE]; 2],
    pub sizes: [u32; 2],
    pub validated: [bool; 2],
    pub enabled: bool,
}

pub struct TableRegistry {
    pub tables: [TableEntry; MAX_TABLES],
    pub table_count: u32,
}

impl TableEntry {
    pub fn empty() -> Self {
        TableEntry {
            name_hash: 0,
            active_buf: 0,
            buffers: [[0u8; MAX_TABLE_SIZE]; 2],
            sizes: [0, 0],
            validated: [false, false],
            enabled: false,
        }
    }
}

impl TableRegistry {
    pub fn new() -> Self {
        TableRegistry {
            tables: [TableEntry::empty(); MAX_TABLES],
            table_count: 0,
        }
    }

    fn find_table(&self, name_hash: u32) -> u32 {
        let mut i: u32 = 0;
        while i < self.table_count {
            if self.tables[i as usize].name_hash == name_hash {
                return i;
            }
            i = i + 1;
        }
        MAX_TABLES as u32
    }

    pub fn register(&mut self, name_hash: u32, size: u32) -> TblResult {
        if self.table_count as usize >= MAX_TABLES {
            return TblResult::Full;
        }
        let idx = self.table_count as usize;
        let mut entry = TableEntry::empty();
        entry.name_hash = name_hash;
        entry.sizes = [size, size];
        entry.enabled = true;
        self.tables[idx] = entry;
        self.table_count = self.table_count + 1;
        TblResult::Success
    }

    pub fn load(&mut self, name_hash: u32, data: &[u8]) -> TblResult {
        let table_idx = self.find_table(name_hash);
        if table_idx as usize >= MAX_TABLES {
            return TblResult::NotFound;
        }
        if table_idx >= self.table_count {
            return TblResult::NotFound;
        }

        let data_len = data.len();
        if data_len > MAX_TABLE_SIZE {
            return TblResult::SizeMismatch;
        }

        let active = self.tables[table_idx as usize].active_buf;
        let inactive: usize = if active == 0 { 1 } else { 0 };

        // Copy data into inactive buffer
        let mut j: usize = 0;
        while j < data_len {
            self.tables[table_idx as usize].buffers[inactive][j] = data[j];
            j = j + 1;
        }

        self.tables[table_idx as usize].sizes[inactive] = data_len as u32;
        self.tables[table_idx as usize].validated[inactive] = true;

        TblResult::Success
    }

    pub fn activate(&mut self, name_hash: u32) -> TblResult {
        let table_idx = self.find_table(name_hash);
        if table_idx as usize >= MAX_TABLES {
            return TblResult::NotFound;
        }
        if table_idx >= self.table_count {
            return TblResult::NotFound;
        }

        let active = self.tables[table_idx as usize].active_buf;
        let inactive: usize = if active == 0 { 1 } else { 0 };

        if !self.tables[table_idx as usize].validated[inactive] {
            return TblResult::ValidationFailed;
        }

        self.tables[table_idx as usize].active_buf = inactive as u8;
        TblResult::Success
    }

    pub fn get_active(&self, name_hash: u32) -> Option<(u8, u32)> {
        let table_idx = self.find_table(name_hash);
        if table_idx as usize >= MAX_TABLES {
            return None;
        }
        if table_idx >= self.table_count {
            return None;
        }

        let entry = &self.tables[table_idx as usize];
        let buf = entry.active_buf;
        let size = entry.sizes[buf as usize];
        Some((buf, size))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_empty_registry() {
        let reg = TableRegistry::new();
        assert_eq!(reg.table_count, 0);
        assert_eq!(reg.get_active(0x1234), None);
    }

    #[test]
    fn test_register_and_get() {
        let mut reg = TableRegistry::new();
        assert_eq!(reg.register(0xAABB, 64), TblResult::Success);
        let active = reg.get_active(0xAABB);
        assert!(active.is_some());
        let (buf, size) = active.unwrap();
        assert_eq!(buf, 0);
        assert_eq!(size, 64);
    }

    #[test]
    fn test_load_and_activate() {
        let mut reg = TableRegistry::new();
        reg.register(0x1234, 8);
        let data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
        assert_eq!(reg.load(0x1234, &data), TblResult::Success);
        assert_eq!(reg.activate(0x1234), TblResult::Success);
        let (buf, size) = reg.get_active(0x1234).unwrap();
        assert_eq!(buf, 1); // swapped to buffer 1
        assert_eq!(size, 4);
    }

    #[test]
    fn test_activate_without_load_fails() {
        let mut reg = TableRegistry::new();
        reg.register(0x5678, 16);
        assert_eq!(reg.activate(0x5678), TblResult::ValidationFailed);
    }

    #[test]
    fn test_not_found() {
        let mut reg = TableRegistry::new();
        assert_eq!(reg.load(0xFFFF, &[1, 2, 3]), TblResult::NotFound);
        assert_eq!(reg.activate(0xFFFF), TblResult::NotFound);
    }

    #[test]
    fn test_registry_full() {
        let mut reg = TableRegistry::new();
        for i in 0..MAX_TABLES {
            assert_eq!(reg.register(i as u32, 4), TblResult::Success);
        }
        assert_eq!(reg.register(0xDEAD, 4), TblResult::Full);
    }

    #[test]
    fn test_size_mismatch() {
        let mut reg = TableRegistry::new();
        reg.register(0xAAAA, 4);
        let big_data = [0u8; MAX_TABLE_SIZE + 1];
        assert_eq!(reg.load(0xAAAA, &big_data), TblResult::SizeMismatch);
    }

    #[test]
    fn test_double_buffer_swap() {
        let mut reg = TableRegistry::new();
        reg.register(0x1111, 8);

        // Load first version into inactive (buffer 1)
        let data1: [u8; 3] = [1, 2, 3];
        reg.load(0x1111, &data1);
        reg.activate(0x1111);
        let (buf1, size1) = reg.get_active(0x1111).unwrap();
        assert_eq!(buf1, 1);
        assert_eq!(size1, 3);

        // Load second version into new inactive (buffer 0)
        let data2: [u8; 2] = [4, 5];
        reg.load(0x1111, &data2);
        reg.activate(0x1111);
        let (buf2, size2) = reg.get_active(0x1111).unwrap();
        assert_eq!(buf2, 0);
        assert_eq!(size2, 2);
    }
}
