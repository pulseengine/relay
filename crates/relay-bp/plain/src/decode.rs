//! The total bundle decoder (SWREQ-RELAY-BP-P01, P02, P03, P06, P07) and the
//! bundle-level validation shared with the encoder (P04).
//!
//! The decoder is a fixed-shape, non-recursive walk over the input. Every read
//! is bounds-checked by [`Reader`]; every loop either consumes at least one
//! byte per iteration or is bounded by [`MAX_BLOCKS`]. It never allocates.

use crate::cbor::{Head, Reader};
use crate::crc::block_crc;
use crate::eid;
use crate::{
    BLOCK_BUNDLE_AGE, BLOCK_HOP_COUNT, BLOCK_PAYLOAD, BLOCK_PREVIOUS_NODE, BP_VERSION, BundleView,
    CanonicalBlock, CrcType, DecodeError, FLAG_IS_FRAGMENT, Mode, PrimaryBlock, StrictViolation,
};

/// Decodes one bundle from `input`, which must hold exactly that bundle.
///
/// Total: returns `Ok` or a typed [`DecodeError`] for every input.
pub fn decode(input: &[u8], mode: Mode) -> Result<BundleView<'_>, DecodeError> {
    let mut r = Reader::new(input);
    match r.head()? {
        Head::Array(None) => {}
        _ => return Err(DecodeError::OuterArrayNotIndefinite),
    }
    let primary = read_primary(&mut r)?;
    let mut view = BundleView::new(primary);
    // Each block consumes at least one byte, and `push` fails past MAX_BLOCKS,
    // so this loop runs at most MAX_BLOCKS + 1 times.
    loop {
        if r.peek_break() {
            r.expect_break(DecodeError::Truncated)?;
            break;
        }
        let block = read_canonical(&mut r)?;
        view.push(block)?;
    }
    if !r.is_empty() {
        return Err(DecodeError::TrailingBytes);
    }
    validate(&view, mode)?;
    Ok(view)
}

/// Reads a CRC field of `crc_type`; returns (offset of the value bytes, value).
fn read_crc_field(r: &mut Reader<'_>, crc_type: CrcType) -> Result<(usize, u32), DecodeError> {
    let want = crc_type.value_len() as u64;
    match r.head()? {
        Head::Bytes(len) if len == want => {}
        _ => return Err(DecodeError::CrcLength),
    }
    let at = r.pos();
    let v = r.take(want)?;
    let mut value: u32 = 0;
    for &b in v {
        value = (value << 8) | u32::from(b);
    }
    Ok((at, value))
}

fn check_crc(
    r: &Reader<'_>,
    start: usize,
    end: usize,
    crc_type: CrcType,
    field: Option<(usize, u32)>,
    block_number: u64,
) -> Result<(), DecodeError> {
    if let Some((at, value)) = field {
        let block = r.slice(start, end)?;
        let rel = at.checked_sub(start).ok_or(DecodeError::Truncated)?;
        if block_crc(crc_type, block, rel) != Some(value) {
            return Err(DecodeError::CrcMismatch { block_number });
        }
    }
    Ok(())
}

fn read_primary<'a>(r: &mut Reader<'a>) -> Result<PrimaryBlock<'a>, DecodeError> {
    let start = r.pos();
    let (kind, depth) = r.open_array(0)?;
    if matches!(kind, Some(n) if !(8..=11).contains(&n)) {
        return Err(DecodeError::PrimaryItemCount);
    }
    let version = r.uint()?;
    if version != BP_VERSION {
        return Err(DecodeError::UnsupportedVersion(version));
    }
    let flags = r.uint()?;
    // SWREQ-RELAY-BP-P07: the profile never accepts a fragment. Rejected as
    // soon as the flag is read, before anything else of the bundle is kept.
    if flags & FLAG_IS_FRAGMENT != 0 {
        return Err(DecodeError::FragmentRejected);
    }
    let crc_type = CrcType::from_code(r.uint()?)?;
    if let Some(n) = kind {
        // Not a fragment: 8 items without a CRC, 9 with one. 10 and 11 (the
        // fragment fields present while the flag is clear) are malformed.
        let want = if crc_type == CrcType::None { 8 } else { 9 };
        if n != want {
            return Err(DecodeError::PrimaryItemCount);
        }
    }
    let destination = eid::read(r, depth)?;
    let source = eid::read(r, depth)?;
    let report_to = eid::read(r, depth)?;
    let (ts_kind, _) = r.open_array(depth)?;
    if matches!(ts_kind, Some(n) if n != 2) {
        return Err(DecodeError::InvalidTimestamp);
    }
    let creation_time = r.uint().map_err(|_| DecodeError::InvalidTimestamp)?;
    let sequence = r.uint().map_err(|_| DecodeError::InvalidTimestamp)?;
    r.close_array(ts_kind, DecodeError::InvalidTimestamp)?;
    let lifetime = r.uint()?;
    let field = if crc_type == CrcType::None {
        None
    } else {
        Some(read_crc_field(r, crc_type)?)
    };
    r.close_array(kind, DecodeError::PrimaryItemCount)?;
    check_crc(r, start, r.pos(), crc_type, field, 0)?;
    Ok(PrimaryBlock {
        flags,
        crc_type,
        destination,
        source,
        report_to,
        creation_time,
        sequence,
        lifetime,
    })
}

pub(crate) fn read_canonical<'a>(r: &mut Reader<'a>) -> Result<CanonicalBlock<'a>, DecodeError> {
    let start = r.pos();
    let (kind, _) = r.open_array(0)?;
    if matches!(kind, Some(n) if n != 5 && n != 6) {
        return Err(DecodeError::BlockItemCount);
    }
    let block_type = r.uint()?;
    let number = r.uint()?;
    let flags = r.uint()?;
    let crc_type = CrcType::from_code(r.uint()?)?;
    if let Some(n) = kind {
        let want = if crc_type == CrcType::None { 5 } else { 6 };
        if n != want {
            return Err(DecodeError::BlockItemCount);
        }
    }
    // A single definite-length byte string (RFC 9171 4.3.2). An indefinite one
    // fails inside `head` with IndefiniteString.
    let data = r.bytes()?;
    let field = if crc_type == CrcType::None {
        None
    } else {
        Some(read_crc_field(r, crc_type)?)
    };
    r.close_array(kind, DecodeError::BlockItemCount)?;
    check_crc(r, start, r.pos(), crc_type, field, number)?;
    Ok(CanonicalBlock {
        block_type,
        number,
        flags,
        crc_type,
        data,
    })
}

/// Parses Bundle Age block data: exactly one canonical CBOR uint (RFC 9171 4.4.2).
pub(crate) fn parse_bundle_age(data: &[u8]) -> Result<u64, DecodeError> {
    let mut r = Reader::new(data);
    let age = r.uint().map_err(|_| DecodeError::InvalidBundleAge)?;
    if !r.is_empty() {
        return Err(DecodeError::InvalidBundleAge);
    }
    Ok(age)
}

/// True when `data`, read as a BPSec abstract security block, starts with a
/// security-target array (RFC 9172 3.6) that lists `target`. Any data that
/// does not start with an array of uints yields `false`.
pub(crate) fn bib_lists_target(data: &[u8], target: u64) -> bool {
    let mut r = Reader::new(data);
    match r.head() {
        Ok(Head::Array(Some(n))) => {
            let mut i: u64 = 0;
            while i < n {
                match r.uint() {
                    Ok(t) if t == target => return true,
                    Ok(_) => {}
                    Err(_) => return false,
                }
                i += 1;
            }
            false
        }
        Ok(Head::Array(None)) => {
            while !r.peek_break() {
                match r.uint() {
                    Ok(t) if t == target => return true,
                    Ok(_) => {}
                    Err(_) => return false,
                }
            }
            false
        }
        _ => false,
    }
}

/// Bundle-level rules, shared by [`decode`] and [`crate::encode`]. The encoder
/// always calls it with [`Mode::Strict`].
pub(crate) fn validate(v: &BundleView<'_>, mode: Mode) -> Result<(), DecodeError> {
    let p = &v.primary;
    if p.flags & FLAG_IS_FRAGMENT != 0 {
        return Err(DecodeError::FragmentRejected);
    }
    eid::validate(&p.destination)?;
    eid::validate(&p.source)?;
    eid::validate(&p.report_to)?;
    let blocks = v.blocks();
    // Pass 1: block multiplicity (RFC 9171 4.1, 4.4.1-4.4.3).
    let mut payloads = 0u32;
    let mut ages = 0u32;
    let mut prev_nodes = 0u32;
    let mut hop_counts = 0u32;
    for b in blocks {
        match b.block_type {
            BLOCK_PAYLOAD => {
                payloads += 1;
                if payloads > 1 {
                    return Err(DecodeError::MultiplePayloadBlocks);
                }
            }
            BLOCK_BUNDLE_AGE => {
                ages += 1;
                if ages > 1 {
                    return Err(DecodeError::MultipleBundleAgeBlocks);
                }
                parse_bundle_age(b.data)?;
            }
            BLOCK_PREVIOUS_NODE => {
                prev_nodes += 1;
                if prev_nodes > 1 {
                    return Err(DecodeError::DuplicateExtensionBlock(BLOCK_PREVIOUS_NODE));
                }
            }
            BLOCK_HOP_COUNT => {
                hop_counts += 1;
                if hop_counts > 1 {
                    return Err(DecodeError::DuplicateExtensionBlock(BLOCK_HOP_COUNT));
                }
            }
            _ => {}
        }
    }
    if payloads == 0 {
        return Err(DecodeError::MissingPayload);
    }
    // Pass 2: block numbers are non-zero and unique; the payload is block 1.
    for (i, b) in blocks.iter().enumerate() {
        if b.number == 0 {
            return Err(DecodeError::InvalidBlockNumber);
        }
        if blocks[..i].iter().any(|e| e.number == b.number) {
            return Err(DecodeError::DuplicateBlockNumber(b.number));
        }
        if b.block_type == BLOCK_PAYLOAD && b.number != 1 {
            return Err(DecodeError::PayloadBlockNumber(b.number));
        }
    }
    // The payload block is last.
    match blocks.last() {
        Some(last) if last.block_type == BLOCK_PAYLOAD => {}
        _ => return Err(DecodeError::PayloadNotLast),
    }
    if mode == Mode::Strict {
        // RFC 9171 4.3.1: primary CRC type MAY be 0 only if a BIB targets block 0.
        if p.crc_type == CrcType::None && !v.bib_targets_primary() {
            return Err(DecodeError::Strict(StrictViolation::PrimaryCrcMissing));
        }
        // RFC 9171 4.4.2: creation time 0 => exactly one Bundle Age block.
        if p.creation_time == 0 && ages != 1 {
            return Err(DecodeError::Strict(StrictViolation::BundleAgeMissing));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::vectors::{RFC9173_A1_ORIGINAL, RFC9173_A3_ORIGINAL, hex};
    use crate::{Eid, MAX_BLOCKS};
    use std::format;
    use std::string::String;
    use std::vec::Vec;

    /// RFC 9173 A.1.1.1 primary block (8 items, CRC type 0).
    const PRIMARY_A1: &str = "88070000820282010282028202018202820201820018281a000f4240";
    /// Payload block, 1-byte payload, CRC type 0.
    const PAYLOAD_1B: &str = "850101000041aa";

    fn bundle(parts: &[&str]) -> Vec<u8> {
        let mut v = std::vec![0x9f];
        for p in parts {
            v.extend(hex(p));
        }
        v.push(0xff);
        v
    }

    fn structural(bytes: &[u8]) -> Result<BundleView<'_>, DecodeError> {
        decode(bytes, Mode::Structural)
    }

    #[test]
    fn rfc9173_a1_original_decodes_fields() {
        // RFC 9173 Appendix A.1.1.3 (fields per A.1.1.1 and A.1.1.2).
        let b = hex(RFC9173_A1_ORIGINAL);
        let v = structural(&b).unwrap();
        let p = v.primary;
        assert_eq!(p.flags, 0);
        assert_eq!(p.crc_type, CrcType::None);
        assert_eq!(
            p.destination,
            Eid::Ipn {
                node: 1,
                service: 2
            }
        );
        assert_eq!(
            p.source,
            Eid::Ipn {
                node: 2,
                service: 1
            }
        );
        assert_eq!(p.report_to, p.source);
        assert_eq!(
            (p.creation_time, p.sequence, p.lifetime),
            (0, 40, 1_000_000)
        );
        assert_eq!(v.blocks().len(), 1);
        assert_eq!(
            v.payload(),
            Some(&b"Ready to generate a 32-byte payload"[..])
        );
    }

    #[test]
    fn rfc9173_a3_original_carries_bundle_age() {
        // RFC 9173 Appendix A.3.1.2: Bundle Age block <<300>>.
        let b = hex(RFC9173_A3_ORIGINAL);
        let v = structural(&b).unwrap();
        assert_eq!(v.bundle_age(), Some(300));
        assert_eq!(v.blocks()[0].block_type, BLOCK_BUNDLE_AGE);
        assert_eq!(v.blocks()[0].number, 2);
    }

    #[test]
    fn rejects_more_than_one_payload_block() {
        // Checked before block numbers, so it is reported whatever the numbers.
        let b = bundle(&[PRIMARY_A1, PAYLOAD_1B, PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::MultiplePayloadBlocks));
        let b = bundle(&[PRIMARY_A1, "850102000041aa", PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::MultiplePayloadBlocks));
    }

    #[test]
    fn rejects_payload_not_last() {
        // Payload, then a hop-count block (type 10, number 2, data [30, 0]).
        let b = bundle(&[PRIMARY_A1, PAYLOAD_1B, "850a02000043821e00"]);
        assert_eq!(structural(&b), Err(DecodeError::PayloadNotLast));
    }

    #[test]
    fn rejects_more_than_one_bundle_age_block() {
        let b = bundle(&[
            PRIMARY_A1,
            "85070200004319012c",
            "85070300004319012c",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::MultipleBundleAgeBlocks));
    }

    #[test]
    fn rejects_duplicate_hop_count_block() {
        let b = bundle(&[
            PRIMARY_A1,
            "850a02000043821e00",
            "850a03000043821e00",
            PAYLOAD_1B,
        ]);
        assert_eq!(
            structural(&b),
            Err(DecodeError::DuplicateExtensionBlock(BLOCK_HOP_COUNT))
        );
    }

    #[test]
    fn rejects_invalid_bundle_age_data() {
        // Age data holds a byte string instead of a uint, then trailing data.
        let b = bundle(&[PRIMARY_A1, "8507020000424100", PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::InvalidBundleAge));
        let b = bundle(&[PRIMARY_A1, "8507020000420300", PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::InvalidBundleAge));
    }

    #[test]
    fn rejects_primary_item_count_out_of_range() {
        // 7 items and 12 items.
        let b = bundle(&[
            "87070000820282010282028202018202820201820018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::PrimaryItemCount));
        let b = bundle(&[
            "8c070000820282010282028202018202820201820018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::PrimaryItemCount));
    }

    #[test]
    fn rejects_primary_item_count_crc_mismatch() {
        // 9 items but CRC type 0.
        let b = bundle(&[
            "89070000820282010282028202018202820201820018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::PrimaryItemCount));
        // 8 items but CRC type 1.
        let b = bundle(&[
            "88070001820282010282028202018202820201820018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::PrimaryItemCount));
    }

    #[test]
    fn rejects_indefinite_byte_string_in_block_data() {
        // Payload data as an indefinite byte string 5f 41 aa ff.
        let b = bundle(&[PRIMARY_A1, "85010100005f41aaff"]);
        assert_eq!(structural(&b), Err(DecodeError::IndefiniteString));
    }

    #[test]
    fn rejects_non_shortest_form_in_block() {
        // Block number 1 encoded as 0x18 0x01.
        let b = bundle(&[PRIMARY_A1, "8501180100 0041aa"]);
        assert_eq!(structural(&b), Err(DecodeError::NonShortestForm));
        // Lifetime 1_000_000 encoded with an 8-byte argument.
        let p = "88070000820282010282028202018202820201820018281b00000000000f4240";
        assert_eq!(
            structural(&bundle(&[p, PAYLOAD_1B])),
            Err(DecodeError::NonShortestForm)
        );
    }

    #[test]
    fn rejects_outer_definite_array() {
        let mut b = bundle(&[PRIMARY_A1, PAYLOAD_1B]);
        b[0] = 0x82;
        b.pop();
        assert_eq!(structural(&b), Err(DecodeError::OuterArrayNotIndefinite));
    }

    #[test]
    fn rejects_trailing_bytes() {
        let mut b = bundle(&[PRIMARY_A1, PAYLOAD_1B]);
        b.push(0x00);
        assert_eq!(structural(&b), Err(DecodeError::TrailingBytes));
    }

    #[test]
    fn rejects_missing_payload() {
        assert_eq!(
            structural(&bundle(&[PRIMARY_A1])),
            Err(DecodeError::MissingPayload)
        );
        let b = bundle(&[PRIMARY_A1, "85070200004319012c"]);
        assert_eq!(structural(&b), Err(DecodeError::MissingPayload));
    }

    #[test]
    fn rejects_duplicate_block_number() {
        let b = bundle(&[PRIMARY_A1, "8518c0010000 40", PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::DuplicateBlockNumber(1)));
    }

    #[test]
    fn rejects_block_number_zero() {
        let b = bundle(&[PRIMARY_A1, "8518c0000000 40", PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::InvalidBlockNumber));
    }

    #[test]
    fn rejects_payload_block_number_not_one() {
        let b = bundle(&[PRIMARY_A1, "850102000041aa"]);
        assert_eq!(structural(&b), Err(DecodeError::PayloadBlockNumber(2)));
    }

    #[test]
    fn rejects_unsupported_version() {
        let b = bundle(&[
            "88060000820282010282028202018202820201820018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::UnsupportedVersion(6)));
    }

    #[test]
    fn rejects_invalid_crc_type() {
        let b = bundle(&[
            "89070003820282010282028202018202820201820018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert_eq!(structural(&b), Err(DecodeError::InvalidCrcType(3)));
        let b = bundle(&[PRIMARY_A1, "8601010003 41aa 4100"]);
        assert_eq!(structural(&b), Err(DecodeError::InvalidCrcType(3)));
    }

    #[test]
    fn rejects_wrong_crc_length() {
        // CRC type 1 with a 4-byte value.
        let b = bundle(&[PRIMARY_A1, "860101000141aa4400000000"]);
        assert_eq!(structural(&b), Err(DecodeError::CrcLength));
    }

    #[test]
    fn rejects_block_item_count() {
        let b = bundle(&[PRIMARY_A1, "840101000041aa"]);
        assert_eq!(structural(&b), Err(DecodeError::BlockItemCount));
        // 6 items with CRC type 0.
        let b = bundle(&[PRIMARY_A1, "860101000041aa00"]);
        assert_eq!(structural(&b), Err(DecodeError::BlockItemCount));
    }

    #[test]
    fn accepts_inner_indefinite_arrays_within_depth() {
        // Primary block as an indefinite array (depth 1) holding an indefinite
        // destination EID (depth 2) = MAX_INNER_INDEF_DEPTH.
        let p = "9f07000 09f0282010 2ff8202820201820282020182001828 1a000f4240ff";
        let b = bundle(&[p, "9f0101000041aaff"]);
        let v = structural(&b).unwrap();
        assert_eq!(
            v.primary.destination,
            Eid::Ipn {
                node: 1,
                service: 2
            }
        );
        assert_eq!(v.payload(), Some(&[0xaa][..]));
    }

    #[test]
    fn rejects_inner_indefinite_arrays_beyond_depth() {
        // Indefinite primary -> indefinite EID -> indefinite ipn SSP = depth 3.
        let p = "9f070000 9f029f0102ffff 8202820201820282020182001828 1a000f4240ff";
        let b = bundle(&[p, PAYLOAD_1B]);
        assert_eq!(structural(&b), Err(DecodeError::IndefiniteDepthExceeded));
    }

    #[test]
    fn rejects_too_many_blocks() {
        let mut parts: Vec<String> = Vec::new();
        parts.push(PRIMARY_A1.into());
        for n in 2..(MAX_BLOCKS as u64 + 2) {
            parts.push(format!("8518c0{n:02x}000040"));
        }
        parts.push(PAYLOAD_1B.into());
        let refs: Vec<&str> = parts.iter().map(String::as_str).collect();
        assert_eq!(structural(&bundle(&refs)), Err(DecodeError::TooManyBlocks));
    }

    #[test]
    fn rejects_every_truncated_prefix() {
        let b = hex(crate::vectors::RFC9173_A3_FINAL);
        assert!(structural(&b).is_ok());
        for cut in 0..b.len() {
            assert!(structural(&b[..cut]).is_err(), "prefix {cut} accepted");
        }
    }

    #[test]
    fn rejects_crc_mismatch() {
        let b = crate::encode::tests::crc_bundle(CrcType::Crc32c, CrcType::Crc16);
        assert!(structural(&b).is_ok());
        // Flip a payload byte: the payload block's CRC-16 no longer matches.
        let mut bad = b.clone();
        let n = bad.len();
        bad[n - 5] ^= 0x01;
        assert_eq!(
            structural(&bad),
            Err(DecodeError::CrcMismatch { block_number: 1 })
        );
        // Flip the lifetime: the primary CRC-32C no longer matches.
        let mut bad = b.clone();
        let pos = bad
            .windows(5)
            .position(|w| w == [0x1a, 0x00, 0x0f, 0x42, 0x40])
            .unwrap();
        bad[pos + 4] ^= 0x01;
        assert_eq!(
            structural(&bad),
            Err(DecodeError::CrcMismatch { block_number: 0 })
        );
    }

    #[test]
    fn strict_rejects_missing_primary_crc() {
        let b = bundle(&[
            "88070000820282010282028202018202820201821a0001e24018281a000f4240",
            PAYLOAD_1B,
        ]);
        assert!(structural(&b).is_ok());
        assert_eq!(
            decode(&b, Mode::Strict),
            Err(DecodeError::Strict(StrictViolation::PrimaryCrcMissing))
        );
    }

    #[test]
    fn strict_rejects_creation_time_zero_without_age() {
        let b = crate::encode::tests::crc_bundle_at(0, CrcType::Crc16, CrcType::None);
        assert!(structural(&b).is_ok());
        assert_eq!(
            decode(&b, Mode::Strict),
            Err(DecodeError::Strict(StrictViolation::BundleAgeMissing))
        );
    }

    /// Primary fields after the flags/CRC type: dest ipn:1.2, src ipn:2.1,
    /// report-to ipn:2.1, timestamp [123456, 40], lifetime 1_000_000.
    const PRIMARY_TAIL: &str = "820282010282028202018202820201821a0001e24018281a000f4240";

    #[test]
    fn rejects_fragment_flag_primary_10_items() {
        // RFC 9171 4.3.1: 10 items = fragment without CRC; offset 0, total 35.
        let p = std::format!("8a070100{PRIMARY_TAIL}001823");
        assert_eq!(
            structural(&bundle(&[&p, PAYLOAD_1B])),
            Err(DecodeError::FragmentRejected)
        );
        assert_eq!(
            decode(&bundle(&[&p, PAYLOAD_1B]), Mode::Strict),
            Err(DecodeError::FragmentRejected)
        );
    }

    #[test]
    fn rejects_fragment_flag_primary_11_items() {
        // 11 items = fragment with a CRC-16 (value not checked: rejected first).
        let p = std::format!("8b070501{PRIMARY_TAIL}0a1823420000");
        assert_eq!(
            structural(&bundle(&[&p, PAYLOAD_1B])),
            Err(DecodeError::FragmentRejected)
        );
    }

    #[test]
    fn rejects_fragment_flag_any_item_count() {
        // The flag alone decides: an 8-item primary with bit 0 set, and an
        // indefinite primary with bit 0 set among other flags, are rejected.
        let p = std::format!("88070100{PRIMARY_TAIL}");
        assert_eq!(
            structural(&bundle(&[&p, PAYLOAD_1B])),
            Err(DecodeError::FragmentRejected)
        );
        let p = std::format!("9f071a00044005 00{PRIMARY_TAIL}ff");
        assert_eq!(
            structural(&bundle(&[&p, PAYLOAD_1B])),
            Err(DecodeError::FragmentRejected)
        );
    }

    #[test]
    fn rejects_fragment_fields_without_flag() {
        // 10 items while the fragment flag is clear: malformed, not a fragment.
        let p = std::format!("8a070000{PRIMARY_TAIL}001823");
        assert_eq!(
            structural(&bundle(&[&p, PAYLOAD_1B])),
            Err(DecodeError::PrimaryItemCount)
        );
    }
}
