//! Relay BP — a fixed-capacity, `no_std`, no-alloc Bundle Protocol v7 codec.
//!
//! Stage S1 of relay-bp (Refs #626): the BPv7 wire codec only. There is no
//! store, no routing and no convergence layer here.
//!
//! ## Profile (maintainer decisions, 2026-11-03, recorded in the release plan)
//!
//! * RFC 9171 BPv7 with **no fragmentation**. A bundle whose fragment flag is
//!   set is rejected with [`DecodeError::FragmentRejected`]; the encoder refuses
//!   to emit one ([`EncodeError::Invalid`]). Reassembly is not implemented
//!   (SWREQ-RELAY-BP-P07).
//! * Relay does **not** claim CCSDS 734.20 profile conformance.
//! * The CBOR layer ([`cbor`]) is this crate's own fixed-capacity reader and
//!   writer for the canonical subset BPv7 uses. No third-party CBOR or CRC crate
//!   is a runtime dependency (SWREQ-RELAY-BP-P05).
//!
//! ## Verification layout (S1 is plain-only)
//!
//! Relay engines normally carry a Verus `src/` tree that is stripped into
//! `plain/src/`. **S1 has no Verus source yet**: `plain/src/` is the only
//! source, Cargo builds it, and the Kani harnesses live in the plain-only
//! [`kani_proofs`](self) sibling declared below. A Verus track is a later slice.
//!
//! ## Shape of the API
//!
//! * [`decode`] is total over `&[u8]`: it returns a borrowed [`BundleView`] or a
//!   typed [`DecodeError`], never panics and never allocates (SWREQ-RELAY-BP-P01).
//!   It is non-recursive; inner indefinite-length arrays are accepted up to
//!   [`MAX_INNER_INDEF_DEPTH`] (SWREQ-RELAY-BP-P02).
//! * [`Mode::Strict`] additionally applies the RFC 9171 4.3.1 primary-CRC rule
//!   and the 4.4.2 Bundle Age rule (SWREQ-RELAY-BP-P04).
//! * [`encode`] always applies the strict rules before it writes a byte, so it
//!   cannot emit a primary-CRC or Bundle Age violation (SWREQ-RELAY-BP-P04).
//! * CRC-16/X-25 and CRC-32C ([`crc`]) zero only the CRC value bytes and keep the
//!   CBOR byte-string header (RFC 9171 erratum 8043; SWREQ-RELAY-BP-P03).
//! * EIDs ([`eid`]): `dtn` (including `dtn:none`) and two-element `ipn`. The RFC
//!   9758 three-element `ipn` form and unknown schemes get a typed reject
//!   (owner decision 2026-11-25 on #626, SWREQ-RELAY-BP-P06).
#![no_std]
#![forbid(unsafe_code)]

#[cfg(test)]
extern crate std;

pub mod cbor;
pub mod crc;
pub mod decode;
pub mod eid;
pub mod encode;

#[cfg(test)]
mod proptests;
#[cfg(test)]
mod vectors;

#[cfg(kani)]
mod kani_proofs;

pub use decode::decode;
pub use eid::{Eid, OPEN_EID_VERDICT, OpenEidForm, OpenEidVerdict};
pub use encode::encode;

/// Maximum number of canonical (non-primary) blocks a [`BundleView`] holds.
/// A bundle with more is rejected with [`DecodeError::TooManyBlocks`].
pub const MAX_BLOCKS: usize = 16;

/// Maximum nesting depth of INNER indefinite-length arrays (the outer bundle
/// array, which RFC 9171 4.1 requires to be indefinite, is not counted).
///
/// RFC 9171 4.1 permits indefinite-length items. The BPv7 structure nests at
/// most three inner arrays (block, EID, ipn SSP), so an indefinite block holding
/// an indefinite EID (depth 2) is accepted, and a third indefinite level inside
/// it is rejected with [`DecodeError::IndefiniteDepthExceeded`].
pub const MAX_INNER_INDEF_DEPTH: usize = 2;

/// The BPv7 version number (RFC 9171 4.3.1).
pub const BP_VERSION: u64 = 7;

/// Bundle processing control flag bit 0: "the bundle is a fragment" (RFC 9171 4.2.3).
pub const FLAG_IS_FRAGMENT: u64 = 0x01;

/// Block type code of the Bundle Payload Block (RFC 9171 4.3.2).
pub const BLOCK_PAYLOAD: u64 = 1;
/// Block type code of the Previous Node Block (RFC 9171 4.4.1).
pub const BLOCK_PREVIOUS_NODE: u64 = 6;
/// Block type code of the Bundle Age Block (RFC 9171 4.4.2).
pub const BLOCK_BUNDLE_AGE: u64 = 7;
/// Block type code of the Hop Count Block (RFC 9171 4.4.3).
pub const BLOCK_HOP_COUNT: u64 = 10;
/// Block type code of the BPSec Block Integrity Block (RFC 9172 3.7).
pub const BLOCK_BIB: u64 = 11;

/// CRC type of a block (RFC 9171 4.2.1). No other values are valid.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrcType {
    /// 0: no CRC is present.
    None,
    /// 1: CRC-16/X-25.
    Crc16,
    /// 2: CRC-32C (Castagnoli).
    Crc32c,
}

impl CrcType {
    /// Parses a wire CRC type code.
    pub const fn from_code(code: u64) -> Result<Self, DecodeError> {
        match code {
            0 => Ok(CrcType::None),
            1 => Ok(CrcType::Crc16),
            2 => Ok(CrcType::Crc32c),
            other => Err(DecodeError::InvalidCrcType(other)),
        }
    }

    /// The wire CRC type code.
    pub const fn code(self) -> u64 {
        match self {
            CrcType::None => 0,
            CrcType::Crc16 => 1,
            CrcType::Crc32c => 2,
        }
    }

    /// Length in bytes of the CRC value on the wire (RFC 9171 4.2.2).
    pub const fn value_len(self) -> usize {
        match self {
            CrcType::None => 0,
            CrcType::Crc16 => 2,
            CrcType::Crc32c => 4,
        }
    }
}

/// Decoding strictness.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Mode {
    /// RFC 9171 structure, canonical CBOR, CRCs, EIDs and the profile's
    /// fragment rejection.
    Structural,
    /// [`Mode::Structural`] plus the RFC 9171 4.3.1 primary-CRC rule and the
    /// 4.4.2 Bundle Age rule. This is the verdict the encoder enforces.
    Strict,
}

/// A rule [`Mode::Strict`] adds on top of the structural checks (SWREQ-RELAY-BP-P04).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StrictViolation {
    /// Primary CRC type is 0 and no Block Integrity Block targets block 0
    /// (RFC 9171 4.3.1).
    PrimaryCrcMissing,
    /// Creation time is 0 and the bundle does not carry exactly one Bundle Age
    /// block (RFC 9171 4.4.2).
    BundleAgeMissing,
}

/// Why a byte slice is not an accepted bundle. Every rejection is typed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeError {
    /// The input ended inside a data item.
    Truncated,
    /// An argument was not encoded in its shortest form (RFC 8949 4.2.1).
    NonShortestForm,
    /// Additional information 28..30, or 31 on a major type that forbids it.
    MalformedHead,
    /// An indefinite-length byte or text string (block data must be a single
    /// definite-length byte string, RFC 9171 4.3.2).
    IndefiniteString,
    /// A data item of the wrong CBOR type for its field.
    UnexpectedType,
    /// The bundle is not a CBOR indefinite-length array (RFC 9171 4.1).
    OuterArrayNotIndefinite,
    /// Bytes follow the bundle's break code.
    TrailingBytes,
    /// Inner indefinite arrays nest deeper than [`MAX_INNER_INDEF_DEPTH`].
    IndefiniteDepthExceeded,
    /// The primary block's item count is not in 8..=11 or does not match its
    /// fragment flag and CRC type (RFC 9171 4.3.1).
    PrimaryItemCount,
    /// The primary block's version is not 7.
    UnsupportedVersion(u64),
    /// The fragment flag is set. The profile never accepts fragments
    /// (SWREQ-RELAY-BP-P07).
    FragmentRejected,
    /// A CRC type code other than 0, 1, 2.
    InvalidCrcType(u64),
    /// The CRC field is not a byte string of the length its type requires.
    CrcLength,
    /// The CRC carried by block `block_number` (0 = primary) does not match.
    CrcMismatch {
        /// Block number of the failing block; 0 is the primary block.
        block_number: u64,
    },
    /// A canonical block's item count is not 5 or 6 or does not match its CRC type.
    BlockItemCount,
    /// A canonical block numbered 0 (reserved for the primary block).
    InvalidBlockNumber,
    /// Two blocks carry the same block number.
    DuplicateBlockNumber(u64),
    /// More than [`MAX_BLOCKS`] canonical blocks.
    TooManyBlocks,
    /// No payload block.
    MissingPayload,
    /// More than one payload block.
    MultiplePayloadBlocks,
    /// The payload block is not the last block.
    PayloadNotLast,
    /// The payload block's number is not 1.
    PayloadBlockNumber(u64),
    /// More than one Bundle Age block.
    MultipleBundleAgeBlocks,
    /// More than one Previous Node or Hop Count block (carries the block type).
    DuplicateExtensionBlock(u64),
    /// A Bundle Age block whose data is not exactly one canonical CBOR uint.
    InvalidBundleAge,
    /// An EID that is not a two-item array headed by a uint scheme code.
    InvalidEid,
    /// A dtn SSP that is neither uint 0 nor an ASCII `//node/demux` text string.
    InvalidDtnSsp,
    /// An ipn SSP that is not an array of two (or, see below, three) uints.
    InvalidIpnSsp,
    /// An EID form relay-bp rejects (#626); see [`OPEN_EID_VERDICT`].
    OpenEidForm(OpenEidForm),
    /// The creation timestamp is not a two-item array of uints.
    InvalidTimestamp,
    /// A [`Mode::Strict`] rule failed.
    Strict(StrictViolation),
}

/// Why [`encode`] refused to write.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EncodeError {
    /// The output buffer is too small.
    BufferTooSmall,
    /// The bundle fails the strict checks the decoder would apply.
    Invalid(DecodeError),
}

/// The primary block (RFC 9171 4.3.1). Version is always 7 and the bundle is
/// never a fragment, so neither the version nor the fragment fields are stored.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PrimaryBlock<'a> {
    /// Bundle processing control flags.
    pub flags: u64,
    /// CRC type of the primary block.
    pub crc_type: CrcType,
    /// Destination EID.
    pub destination: Eid<'a>,
    /// Source node ID.
    pub source: Eid<'a>,
    /// Report-to EID.
    pub report_to: Eid<'a>,
    /// Creation timestamp: DTN time in ms (0 = unknown clock).
    pub creation_time: u64,
    /// Creation timestamp: sequence number.
    pub sequence: u64,
    /// Lifetime in ms past the creation time.
    pub lifetime: u64,
}

/// A canonical block (RFC 9171 4.3.2). `data` borrows the input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CanonicalBlock<'a> {
    /// Block type code.
    pub block_type: u64,
    /// Block number (never 0; the payload block is 1).
    pub number: u64,
    /// Block processing control flags.
    pub flags: u64,
    /// CRC type of this block.
    pub crc_type: CrcType,
    /// Block-type-specific data (the content of the definite byte string).
    pub data: &'a [u8],
}

impl CanonicalBlock<'_> {
    const EMPTY: CanonicalBlock<'static> = CanonicalBlock {
        block_type: 0,
        number: 0,
        flags: 0,
        crc_type: CrcType::None,
        data: &[],
    };
}

/// A decoded bundle, borrowing its block data from the input. Also the input
/// type of [`encode`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BundleView<'a> {
    /// The primary block.
    pub primary: PrimaryBlock<'a>,
    blocks: [CanonicalBlock<'a>; MAX_BLOCKS],
    len: usize,
}

impl<'a> BundleView<'a> {
    /// A bundle with the given primary block and no canonical blocks yet.
    pub const fn new(primary: PrimaryBlock<'a>) -> Self {
        Self {
            primary,
            blocks: [CanonicalBlock::EMPTY; MAX_BLOCKS],
            len: 0,
        }
    }

    /// Appends a canonical block in wire order. Errors when full.
    pub fn push(&mut self, block: CanonicalBlock<'a>) -> Result<(), DecodeError> {
        match self.blocks.get_mut(self.len) {
            Some(slot) => {
                *slot = block;
                self.len += 1;
                Ok(())
            }
            None => Err(DecodeError::TooManyBlocks),
        }
    }

    /// The canonical blocks in wire order.
    pub fn blocks(&self) -> &[CanonicalBlock<'a>] {
        // `len <= MAX_BLOCKS` is maintained by `push`; `get` keeps this total.
        self.blocks.get(..self.len).unwrap_or(&[])
    }

    /// The payload bytes, if a payload block is present.
    pub fn payload(&self) -> Option<&'a [u8]> {
        self.blocks()
            .iter()
            .find(|b| b.block_type == BLOCK_PAYLOAD)
            .map(|b| b.data)
    }

    /// The Bundle Age in ms, if exactly decodable from the first age block.
    pub fn bundle_age(&self) -> Option<u64> {
        self.blocks()
            .iter()
            .find(|b| b.block_type == BLOCK_BUNDLE_AGE)
            .and_then(|b| decode::parse_bundle_age(b.data).ok())
    }

    /// True when some Block Integrity Block lists block 0 (the primary block)
    /// among its security targets (RFC 9172 3.6, first ASB field). A BIB whose
    /// data cannot be read as a target array (for example, one encrypted by a
    /// BCB) does not count.
    pub fn bib_targets_primary(&self) -> bool {
        self.blocks()
            .iter()
            .any(|b| b.block_type == BLOCK_BIB && decode::bib_lists_target(b.data, 0))
    }
}
