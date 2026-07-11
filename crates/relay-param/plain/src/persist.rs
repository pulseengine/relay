//! Parameter PERSISTENCE (PARAM-P03, v1.117) — the store survives power cycles.
//!
//! Design per the agreed hardware seam (jess#144 / jess DD-016): the target is
//! the Pixhawk 6X-RT's FM25V02A **FRAM** — byte-addressable, no erase pages,
//! ~10^12 write endurance — so the seam is a plain BYTE seam ([`NvmBytes`]) and
//! no wear-leveling layer exists on this path (it would return as a requirement
//! only if parameters ever move to NOR flash). relay owns this record codec +
//! commit protocol; jess owns the FRAM binding below the trait.
//!
//! ## Torn-write safety (the commit protocol)
//!
//! Power can fail mid-`save`. FRAM writes are byte-atomic but a multi-byte
//! image write is not. Classic TWO-SLOT commit:
//!
//! ```text
//! [0]                  selector: 0xA5 => slot A active, 0x5A => slot B
//! [1..16]              reserved
//! [16          ..16+S] slot A   (S = 16-byte header + 24 bytes per record)
//! [16+S        ..16+2S] slot B
//! ```
//!
//! `save` writes the full image to the INACTIVE slot, then flips the selector
//! with a single-byte write (atomic on FRAM). A crash before the flip leaves
//! the previously-active slot untouched — `load` still finds a valid image; a
//! crash after the flip means the new image was already complete. There is no
//! window in which the only copy is torn.
//!
//! ## Load safety (never silent, never out-of-schema)
//!
//! Every record carries a CRC-32; the header carries a magic, a SCHEMA VERSION
//! and its own CRC. `load` applies values through [`ParamStore::set`] — the
//! Kani-proven schema gate — so a corrupt or hostile image **cannot** land an
//! out-of-bounds value (PARAM-K03 proves this for arbitrary NVM bytes). Every
//! degraded outcome is LOUD: the [`LoadReport`] says exactly what happened
//! (fresh defaults, schema mismatch, corrupt image, per-record skips) so the
//! pre-arm gate and telemetry can surface it — a store silently reverting to
//! defaults is the failure mode this module exists to prevent.
//!
//! Schema MIGRATION policy: a stored image whose `schema_version` differs from
//! the firmware's is NOT reinterpreted — it loads as defaults with
//! [`LoadOutcome::SchemaMismatch`] (explicit reset, PX4/ArduPilot param-
//! migration discipline; a translating migrator can layer on top later).

use crate::{ParamStore, SetResult};

/// Byte-addressable non-volatile memory — the FRAM seam.
///
/// Implementations must be plain byte semantics: no erase, no alignment
/// constraints (the FM25V02A contract). Errors are for the binding to surface
/// bus faults; the mock in tests never fails.
pub trait NvmBytes {
    /// Total usable capacity in bytes.
    fn capacity(&self) -> usize;
    /// Read `buf.len()` bytes starting at `offset`. Must fill `buf` entirely.
    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), NvmError>;
    /// Write all of `data` starting at `offset`.
    fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), NvmError>;
}

/// A bus/bounds fault surfaced by the NVM binding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct NvmError;

/// Why `save` failed. The store in NVM is unchanged on any error (the active
/// slot is only superseded by the final selector flip).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SaveError {
    /// The NVM is too small for two slots of `max_records`.
    Capacity,
    /// The store holds more parameters than `max_records`.
    TooManyParams,
    /// The NVM binding reported a fault.
    Nvm,
}

/// How a `load` concluded. Everything except `Loaded` means the store now
/// holds SCHEMA DEFAULTS — loudly, for pre-arm/telemetry to surface.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LoadOutcome {
    /// A valid image was applied.
    Loaded,
    /// No valid image present (fresh device / never saved) — defaults.
    FreshDefaults,
    /// Image header valid but from a DIFFERENT schema version — defaults
    /// (explicit reset, never silent reinterpretation).
    SchemaMismatch,
    /// Header or selector invalid / CRC-failed — defaults.
    Corrupt,
    /// The NVM binding reported a fault during load — defaults.
    NvmFault,
}

/// The loud, complete story of a `load`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LoadReport {
    pub outcome: LoadOutcome,
    /// Records whose value was applied through the schema gate.
    pub applied: u32,
    /// Records whose id is not registered in this firmware's store.
    pub skipped_unknown: u32,
    /// Records rejected by the schema gate (CRC-valid but out-of-bounds for
    /// the CURRENT schema — e.g. a bound tightened between saves) or with a
    /// failed record CRC.
    pub rejected: u32,
}

const SELECTOR_A: u8 = 0xA5;
const SELECTOR_B: u8 = 0x5A;
const RESERVED: usize = 16; // selector byte + 15 spare
const HEADER_LEN: usize = 16;
const RECORD_LEN: usize = 24;
const MAGIC: u32 = 0x5052_4D31; // "PRM1"

/// Fixed geometry for images holding up to `max_records` records.
#[derive(Clone, Copy, Debug)]
pub struct Layout {
    pub max_records: usize,
}

impl Layout {
    pub const fn new(max_records: usize) -> Self {
        Layout { max_records }
    }
    /// Bytes per slot.
    pub const fn slot_len(&self) -> usize {
        HEADER_LEN + RECORD_LEN * self.max_records
    }
    /// Total NVM bytes required (selector + two slots).
    pub const fn required_capacity(&self) -> usize {
        RESERVED + 2 * self.slot_len()
    }
    const fn slot_offset(&self, slot: u8) -> usize {
        RESERVED + (slot as usize) * self.slot_len()
    }
}

/// CRC-32 (IEEE, bitwise — small, table-free, no_std).
fn crc32(data: &[u8]) -> u32 {
    let mut crc: u32 = 0xFFFF_FFFF;
    for &b in data {
        crc ^= b as u32;
        let mut i = 0;
        while i < 8 {
            crc = if crc & 1 != 0 { (crc >> 1) ^ 0xEDB8_8320 } else { crc >> 1 };
            i += 1;
        }
    }
    !crc
}

/// Serialize the store into the inactive slot, then atomically flip the
/// selector. On any error the previously-committed image remains active.
pub fn save<const N: usize, M: NvmBytes>(
    store: &ParamStore<N>,
    nvm: &mut M,
    layout: Layout,
    schema_version: u32,
) -> Result<(), SaveError> {
    if nvm.capacity() < layout.required_capacity() {
        return Err(SaveError::Capacity);
    }
    if store.len() > layout.max_records {
        return Err(SaveError::TooManyParams);
    }
    // Which slot is currently active? Write the OTHER one.
    let mut sel = [0u8; 1];
    nvm.read(0, &mut sel).map_err(|_| SaveError::Nvm)?;
    let (target_slot, new_selector) =
        if sel[0] == SELECTOR_A { (1u8, SELECTOR_B) } else { (0u8, SELECTOR_A) };
    let base = layout.slot_offset(target_slot);

    // Header: magic | schema_version | count | crc(over first 12 bytes).
    let count = store.len() as u32;
    let mut header = [0u8; HEADER_LEN];
    header[0..4].copy_from_slice(&MAGIC.to_le_bytes());
    header[4..8].copy_from_slice(&schema_version.to_le_bytes());
    header[8..12].copy_from_slice(&count.to_le_bytes());
    let hcrc = crc32(&header[0..12]);
    header[12..16].copy_from_slice(&hcrc.to_le_bytes());
    nvm.write(base, &header).map_err(|_| SaveError::Nvm)?;

    // Records: id(16) | value(4, LE bits) | crc(4, over id+value).
    let mut i = 0;
    while let Some((id, value)) = store.nth(i) {
        let mut rec = [0u8; RECORD_LEN];
        rec[0..16].copy_from_slice(&id);
        rec[16..20].copy_from_slice(&value.to_le_bytes());
        let rcrc = crc32(&rec[0..20]);
        rec[20..24].copy_from_slice(&rcrc.to_le_bytes());
        nvm.write(base + HEADER_LEN + i * RECORD_LEN, &rec)
            .map_err(|_| SaveError::Nvm)?;
        i += 1;
    }

    // COMMIT POINT: single-byte selector flip (atomic on FRAM).
    nvm.write(0, &[new_selector]).map_err(|_| SaveError::Nvm)
}

/// Load the active image into the store (which must already hold the
/// firmware's registered schema — values arrive through the bounded `set`).
/// The store is left at DEFAULTS unless a valid record applies over them.
pub fn load<const N: usize, M: NvmBytes>(
    store: &mut ParamStore<N>,
    nvm: &M,
    layout: Layout,
    schema_version: u32,
) -> LoadReport {
    let report = |outcome| LoadReport { outcome, applied: 0, skipped_unknown: 0, rejected: 0 };
    if nvm.capacity() < layout.required_capacity() {
        return report(LoadOutcome::FreshDefaults);
    }
    let mut sel = [0u8; 1];
    if nvm.read(0, &mut sel).is_err() {
        return report(LoadOutcome::NvmFault);
    }
    let slot = match sel[0] {
        SELECTOR_A => 0u8,
        SELECTOR_B => 1u8,
        // Never-committed device (erased FRAM reads 0x00/0xFF) — fresh.
        _ => return report(LoadOutcome::FreshDefaults),
    };
    let base = layout.slot_offset(slot);

    let mut header = [0u8; HEADER_LEN];
    if nvm.read(base, &mut header).is_err() {
        return report(LoadOutcome::NvmFault);
    }
    let magic = u32::from_le_bytes([header[0], header[1], header[2], header[3]]);
    let version = u32::from_le_bytes([header[4], header[5], header[6], header[7]]);
    let count = u32::from_le_bytes([header[8], header[9], header[10], header[11]]);
    let hcrc = u32::from_le_bytes([header[12], header[13], header[14], header[15]]);
    if magic != MAGIC || hcrc != crc32(&header[0..12]) {
        return report(LoadOutcome::Corrupt);
    }
    if version != schema_version {
        // Explicit reset, never silent reinterpretation.
        return report(LoadOutcome::SchemaMismatch);
    }
    if count as usize > layout.max_records {
        return report(LoadOutcome::Corrupt);
    }

    let (mut applied, mut skipped_unknown, mut rejected) = (0u32, 0u32, 0u32);
    let mut i = 0usize;
    while i < count as usize {
        let mut rec = [0u8; RECORD_LEN];
        if nvm.read(base + HEADER_LEN + i * RECORD_LEN, &mut rec).is_err() {
            return LoadReport { outcome: LoadOutcome::NvmFault, applied, skipped_unknown, rejected };
        }
        let rcrc = u32::from_le_bytes([rec[20], rec[21], rec[22], rec[23]]);
        if rcrc != crc32(&rec[0..20]) {
            rejected += 1;
            i += 1;
            continue;
        }
        let mut id = [0u8; 16];
        id.copy_from_slice(&rec[0..16]);
        let value = f32::from_le_bytes([rec[16], rec[17], rec[18], rec[19]]);
        // THE SCHEMA GATE: the Kani-proven bounded set. A CRC-valid record
        // whose value is out of the CURRENT schema's bounds is rejected and
        // the default stands — NVM content can never widen a bound.
        match store.set(&id, value) {
            SetResult::Applied => applied += 1,
            SetResult::Unknown => skipped_unknown += 1,
            SetResult::OutOfRange => rejected += 1,
        }
        i += 1;
    }
    LoadReport { outcome: LoadOutcome::Loaded, applied, skipped_unknown, rejected }
}

/// A fixed-size in-memory NVM — the test/Kani mock AND the Renode-stage
/// stand-in until jess's FRAM binding lands (jess#144).
pub struct ArrayNvm<const CAP: usize> {
    pub bytes: [u8; CAP],
}

impl<const CAP: usize> Default for ArrayNvm<CAP> {
    fn default() -> Self {
        Self::new()
    }
}

impl<const CAP: usize> ArrayNvm<CAP> {
    /// Fresh device: FRAM ships zeroed (selector 0x00 = never committed).
    pub fn new() -> Self {
        ArrayNvm { bytes: [0u8; CAP] }
    }
}

impl<const CAP: usize> NvmBytes for ArrayNvm<CAP> {
    fn capacity(&self) -> usize {
        CAP
    }
    fn read(&self, offset: usize, buf: &mut [u8]) -> Result<(), NvmError> {
        let end = offset.checked_add(buf.len()).ok_or(NvmError)?;
        if end > CAP {
            return Err(NvmError);
        }
        buf.copy_from_slice(&self.bytes[offset..end]);
        Ok(())
    }
    fn write(&mut self, offset: usize, data: &[u8]) -> Result<(), NvmError> {
        let end = offset.checked_add(data.len()).ok_or(NvmError)?;
        if end > CAP {
            return Err(NvmError);
        }
        self.bytes[offset..end].copy_from_slice(data);
        Ok(())
    }
}
