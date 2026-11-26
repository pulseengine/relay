//! Kani bounded-model-checking harnesses for relay-bp (plain-only).
//!
//! S1 has no Verus source, so these sit next to the plain sources; they are
//! declared from lib.rs under `#[cfg(kani)]` and gated by the kani.yml matrix
//! (relay-bp entry).
//!
//! Harness map:
//!   BP-K01 `verify_cbor_head_total`              P01 per item: CBOR head reader, 9 B
//!   BP-K02 `verify_eid_read_total`               P01 per item: EID reader, 12 B
//!   BP-K03 `verify_canonical_block_read_total`   P01 per item: canonical block reader incl. CRC, 8 B
//!   BP-K11 `verify_bundle_age_parse_total`       P01 per item: Bundle Age data parser, 9 B
//!   BP-K12 `verify_bib_target_scan_total`        P01 per item: BIB security-target scan, 8 B
//!   BP-K04 `verify_crc16_table_matches_bitwise`  P03 table step == bitwise step
//!   BP-K05 `verify_crc32c_table_matches_bitwise` P03 table step == bitwise step
//!   BP-K06 `verify_block_crc_zeroes_only_value`  P03 erratum 8043, 8 B block
//!   BP-K07 `verify_encoder_never_violates_strict` P04 Ok => strict rules hold
//!   BP-K09 `verify_fragment_never_validates`     P07 flag set => validate Err (decode gate)
//!   BP-K10 `verify_encoder_refuses_fragment`     P07 flag set => encode Err
//!
//! Tractability note (measured locally, Kani 0.67, loaded host): a WHOLE-decoder
//! totality harness over fully symbolic bytes did not finish (12 B / unwind 18:
//! >1200 s; 8 B / unwind 10: >1500 s), because a symbolic read position makes
//! CBMC unwind every decoder loop to the bound. Totality is therefore argued per
//! item (K01-K03, K11, K12), with the whole decoder covered by proptest. The
//! encoder harnesses (K07, K08) pin each symbolic value to one CBOR head width,
//! so the byte layout is concrete; their first, fully symbolic-width versions
//! did not finish in 800-1800 s either.
//!
//! NO Kani round-trip harness (P05): `verify_round_trip_small` did not finish
//! locally in 1200-1500 s in any form tried, including fixed header values with
//! only four symbolic payload bytes (symbolic CRC-16 and CRC-32C table lookups
//! over three encode/decode passes). It was removed rather than left in the
//! matrix, where a non-terminating leg fails the Kani gate. P05 rests on the
//! external A.3.5 byte round trip, the unit tests and the proptest.
#![cfg(kani)]

use crate::cbor::Reader;
use crate::crc::{bitwise16, bitwise32, block_crc, crc16_x25, step16, step32};
use crate::decode::{bib_lists_target, parse_bundle_age, read_canonical, validate};
use crate::{
    BLOCK_BIB, BLOCK_BUNDLE_AGE, BLOCK_PAYLOAD, BundleView, CanonicalBlock, CrcType, DecodeError,
    Eid, EncodeError, FLAG_IS_FRAGMENT, Mode, PrimaryBlock, decode, eid, encode,
};

/// BP-K01 (P01, per item): the CBOR head reader is total on any 0..=9 bytes
/// and never reports a position past the input.
#[kani::proof]
#[kani::unwind(10)]
fn verify_cbor_head_total() {
    let buf: [u8; 9] = kani::any();
    // Every length 0..=buf.len() is reachable; bounded by construction, not
    // by an assumption (the assume ledger is shrink-only, North Star O3).
    let len = kani::any::<u8>() as usize % (buf.len() + 1);
    let mut r = Reader::new(&buf[..len]);
    let res = r.head();
    assert!(r.pos() <= len);
    if res.is_ok() {
        assert!(r.pos() >= 1);
    }
}

/// BP-K02 (P01, per item): the EID reader is total on any 0..=12 bytes at any
/// starting indefinite depth, and never reads past the input.
#[kani::proof]
#[kani::unwind(13)]
fn verify_eid_read_total() {
    let buf: [u8; 12] = kani::any();
    // Every length 0..=buf.len() is reachable; bounded by construction, not
    // by an assumption (the assume ledger is shrink-only, North Star O3).
    let len = kani::any::<u8>() as usize % (buf.len() + 1);
    let depth = kani::any::<u8>() as usize % (crate::MAX_INNER_INDEF_DEPTH + 1);
    let mut r = Reader::new(&buf[..len]);
    let _ = eid::read(&mut r, depth);
    assert!(r.pos() <= len);
}

/// BP-K03 (P01, per item): the canonical block reader, including the CRC
/// field and the erratum-8043 CRC check over the block, is total on any 0..=8
/// bytes and never reads past the input.
#[kani::proof]
#[kani::unwind(10)]
fn verify_canonical_block_read_total() {
    let buf: [u8; 8] = kani::any();
    // Every length 0..=buf.len() is reachable; bounded by construction, not
    // by an assumption (the assume ledger is shrink-only, North Star O3).
    let len = kani::any::<u8>() as usize % (buf.len() + 1);
    let mut r = Reader::new(&buf[..len]);
    let _ = read_canonical(&mut r);
    assert!(r.pos() <= len);
}

/// BP-K11 (P01, per item): the Bundle Age data parser is total on any 0..=9 bytes.
#[kani::proof]
#[kani::unwind(10)]
fn verify_bundle_age_parse_total() {
    let buf: [u8; 9] = kani::any();
    // Every length 0..=buf.len() is reachable; bounded by construction, not
    // by an assumption (the assume ledger is shrink-only, North Star O3).
    let len = kani::any::<u8>() as usize % (buf.len() + 1);
    let _ = parse_bundle_age(&buf[..len]);
}

/// BP-K12 (P01, per item): the BIB security-target scan used by the strict
/// primary-CRC rule is total on any 0..=8 bytes, including a definite array
/// head that claims more targets than the data holds.
#[kani::proof]
#[kani::unwind(10)]
fn verify_bib_target_scan_total() {
    let buf: [u8; 8] = kani::any();
    // Every length 0..=buf.len() is reachable; bounded by construction, not
    // by an assumption (the assume ledger is shrink-only, North Star O3).
    let len = kani::any::<u8>() as usize % (buf.len() + 1);
    let target: u64 = kani::any();
    let _ = bib_lists_target(&buf[..len], target);
}

/// BP-K04 (P03): the table-driven CRC-16/X-25 step equals the bitwise
/// reference step for every register state and input byte.
#[kani::proof]
#[kani::unwind(9)]
fn verify_crc16_table_matches_bitwise() {
    let state: u16 = kani::any();
    let byte: u8 = kani::any();
    assert_eq!(step16(state, byte), bitwise16(state, byte));
}

/// BP-K05 (P03): the table-driven CRC-32C step equals the bitwise reference.
#[kani::proof]
#[kani::unwind(9)]
fn verify_crc32c_table_matches_bitwise() {
    let state: u32 = kani::any();
    let byte: u8 = kani::any();
    assert_eq!(step32(state, byte), bitwise32(state, byte));
}

/// BP-K06 (P03, erratum 8043): over any 8-byte block and any in-range value
/// offset, `block_crc` equals the plain CRC of a copy in which exactly the two
/// value bytes are zero and every other byte (including the header byte just
/// before them) is unchanged. The block's own value bytes do not matter.
#[kani::proof]
#[kani::unwind(10)]
fn verify_block_crc_zeroes_only_value() {
    let block: [u8; 8] = kani::any();
    // Every offset 0..=block.len()-2 is reachable, by construction.
    let at = kani::any::<u8>() as usize % (block.len() - 1);
    let mut zeroed = block;
    zeroed[at] = 0;
    zeroed[at + 1] = 0;
    let got = block_crc(CrcType::Crc16, &block, at);
    assert_eq!(got, Some(u32::from(crc16_x25(&zeroed))));
}

fn sym_crc() -> CrcType {
    match kani::any::<u8>() % 3 {
        0 => CrcType::None,
        1 => CrcType::Crc16,
        _ => CrcType::Crc32c,
    }
}

/// A symbolic value whose CBOR head is one byte (0..=23).
fn small() -> u64 {
    // Every value 0..=23 is reachable, by construction.
    u64::from(kani::any::<u8>() % 24)
}

fn sym_primary(crc_type: CrcType) -> PrimaryBlock<'static> {
    PrimaryBlock {
        flags: kani::any(),
        crc_type,
        destination: Eid::Ipn {
            node: kani::any(),
            service: kani::any(),
        },
        source: Eid::DtnNone,
        report_to: Eid::DtnNone,
        creation_time: kani::any(),
        sequence: kani::any(),
        lifetime: kani::any(),
    }
}

/// BP-K07 (P04): for symbolic primary CRC type (0/1/2), flags with or without
/// the fragment bit, creation time 0 or non-zero, 0..=2 Bundle Age blocks, and
/// a BIB on block 0 present or absent, `encode` returns Ok ONLY when the RFC
/// 9171 4.3.1 primary-CRC rule and the 4.4.2 Bundle Age rule hold (and the
/// bundle is not a fragment), and refuses with Invalid otherwise: the encoder's
/// gate decision is EXACTLY the rules. Checked through `encode` itself with an
/// empty output buffer (a 64 B buffer, i.e. symbolic byte emission + CRC, did
/// not finish in 1200 s locally); that the written bytes then decode strictly
/// is shown by the unit tests and proptests::canonical_round_trip.
#[kani::proof]
#[kani::unwind(12)]
fn verify_encoder_never_violates_strict() {
    let creation_time = if kani::any() { 0 } else { 812_345_678_901 };
    let p = PrimaryBlock {
        flags: small(),
        crc_type: sym_crc(),
        destination: Eid::Ipn {
            node: small(),
            service: small(),
        },
        source: Eid::DtnNone,
        report_to: Eid::DtnNone,
        creation_time,
        sequence: small(),
        lifetime: 1_000_000,
    };
    let mut v = BundleView::new(p);
    // Four blocks are always present with concrete data, so the block count
    // and every data slice are concrete; only the block TYPES are symbolic.
    // (Pushing blocks conditionally made the slice length symbolic and the
    // harness did not finish in 1200 s.) A non-BIB / non-age slot gets a
    // private-use type (192, 193, 194), which the rules ignore.
    let bib_on_primary: bool = kani::any();
    let age_b: bool = kani::any();
    let age_c: bool = kani::any();
    let ages: u8 = u8::from(age_b) + u8::from(age_c);
    v.push(CanonicalBlock {
        block_type: if bib_on_primary { BLOCK_BIB } else { 192 },
        number: 2,
        flags: 0,
        crc_type: CrcType::None,
        data: &[0x81, 0x00],
    })
    .unwrap();
    v.push(CanonicalBlock {
        block_type: if age_b { BLOCK_BUNDLE_AGE } else { 193 },
        number: 3,
        flags: 0,
        crc_type: CrcType::None,
        data: &[0x05],
    })
    .unwrap();
    v.push(CanonicalBlock {
        block_type: if age_c { BLOCK_BUNDLE_AGE } else { 194 },
        number: 4,
        flags: 0,
        crc_type: CrcType::None,
        data: &[0x05],
    })
    .unwrap();
    v.push(CanonicalBlock {
        block_type: BLOCK_PAYLOAD,
        number: 1,
        flags: 0,
        crc_type: CrcType::None,
        data: &[0xaa],
    })
    .unwrap();

    // Empty output: `encode` can only reach BufferTooSmall after its strict
    // gate has passed (it writes nothing before the gate), and returns
    // Invalid(_) when the gate fails. Ok on a real buffer goes through the
    // same gate. This keeps byte emission and CRC out of the formula.
    let res = encode(&v, &mut []);
    let rules_hold = (v.primary.crc_type != CrcType::None || bib_on_primary)
        && (v.primary.creation_time != 0 || ages == 1)
        && ages <= 1
        && v.primary.flags & FLAG_IS_FRAGMENT == 0;
    match res {
        Err(EncodeError::BufferTooSmall) => assert!(rules_hold),
        Err(EncodeError::Invalid(_)) => assert!(!rules_hold),
        Ok(_) => panic!("an empty buffer cannot hold a bundle"),
    }
    kani::cover!(rules_hold);
    kani::cover!(!rules_hold);
}

/// BP-K09 (P07): for EVERY flags value with bit 0 set (any u64) and any primary
/// CRC type, the bundle-level validation rejects the bundle with
/// FragmentRejected in both modes. `decode` returns `Ok(view)` only after
/// `validate(&view, mode)` has passed on that very view, so no decode returns a
/// bundle whose fragment flag is set; and `read_primary` rejects the flag even
/// earlier (unit tests `rejects_fragment_flag_*`). The byte-level version of
/// this harness (symbolic flags byte inside a bundle) did not finish in
/// 1200-1800 s locally.
#[kani::proof]
#[kani::unwind(4)]
fn verify_fragment_never_validates() {
    let mut p = sym_primary(sym_crc());
    p.flags |= FLAG_IS_FRAGMENT;
    let mut v = BundleView::new(p);
    v.push(CanonicalBlock {
        block_type: BLOCK_PAYLOAD,
        number: 1,
        flags: 0,
        crc_type: CrcType::None,
        data: &[0xaa],
    })
    .unwrap();
    let strict: bool = kani::any();
    let mode = if strict {
        Mode::Strict
    } else {
        Mode::Structural
    };
    assert!(validate(&v, mode) == Err(DecodeError::FragmentRejected));
}

/// BP-K10 (P07): the encoder refuses every bundle whose fragment flag is set,
/// whatever the other flags and fields are.
#[kani::proof]
#[kani::unwind(20)]
fn verify_encoder_refuses_fragment() {
    let mut p = sym_primary(sym_crc());
    p.flags |= FLAG_IS_FRAGMENT;
    let mut v = BundleView::new(p);
    v.push(CanonicalBlock {
        block_type: BLOCK_PAYLOAD,
        number: 1,
        flags: 0,
        crc_type: CrcType::None,
        data: &[0xaa],
    })
    .unwrap();
    let mut out = [0u8; 96];
    assert!(encode(&v, &mut out) == Err(EncodeError::Invalid(DecodeError::FragmentRejected)));
}
