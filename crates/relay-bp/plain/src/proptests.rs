//! Property tests (test-only): decoder totality over arbitrary and mutated
//! bytes (SWREQ-RELAY-BP-P01) and the canonical round trip over generated
//! bundles (SWREQ-RELAY-BP-P05).
//!
//! The generated bundles are produced by this crate's encoder, so the round
//! trip here is a self-consistency property, not an external conformance
//! check; the external anchor is `encode::tests::round_trip_rfc9173_a3_final_bytes`.

use crate::vectors::{RFC9173_A3_FINAL, hex};
use crate::{
    BLOCK_BUNDLE_AGE, BLOCK_PAYLOAD, BundleView, CanonicalBlock, CrcType, Eid, Mode, PrimaryBlock,
    decode, encode,
};
use proptest::prelude::*;
use std::string::String;
use std::vec::Vec;

fn crc_type() -> impl Strategy<Value = CrcType> {
    prop_oneof![
        Just(CrcType::None),
        Just(CrcType::Crc16),
        Just(CrcType::Crc32c)
    ]
}

#[derive(Debug, Clone)]
enum EidSpec {
    None,
    Dtn(String),
    Ipn(u64, u64),
}

fn eid_spec() -> impl Strategy<Value = EidSpec> {
    prop_oneof![
        Just(EidSpec::None),
        "//[a-z0-9.-]{1,12}/[ -~]{0,12}".prop_map(EidSpec::Dtn),
        (any::<u64>(), any::<u64>()).prop_map(|(n, s)| EidSpec::Ipn(n, s)),
    ]
}

fn eid(spec: &EidSpec) -> Eid<'_> {
    match spec {
        EidSpec::None => Eid::DtnNone,
        EidSpec::Dtn(s) => Eid::Dtn(s.as_str()),
        EidSpec::Ipn(n, s) => Eid::Ipn {
            node: *n,
            service: *s,
        },
    }
}

#[derive(Debug, Clone)]
struct Spec {
    flags: u64,
    primary_crc: CrcType,
    eids: [EidSpec; 3],
    creation_time: u64,
    sequence: u64,
    lifetime: u64,
    age: u64,
    ext: Vec<(u64, u64, CrcType, Vec<u8>)>,
    payload_crc: CrcType,
    payload: Vec<u8>,
}

fn spec() -> impl Strategy<Value = Spec> {
    (
        any::<u64>().prop_map(|f| f & !crate::FLAG_IS_FRAGMENT),
        prop_oneof![Just(CrcType::Crc16), Just(CrcType::Crc32c)],
        [eid_spec(), eid_spec(), eid_spec()],
        prop_oneof![Just(0u64), any::<u64>()],
        any::<u64>(),
        any::<u64>(),
        any::<u64>(),
        proptest::collection::vec(
            (
                192u64..=255,
                any::<u64>(),
                crc_type(),
                proptest::collection::vec(any::<u8>(), 0..40),
            ),
            0..4,
        ),
        crc_type(),
        proptest::collection::vec(any::<u8>(), 0..300),
    )
        .prop_map(
            |(flags, primary_crc, eids, ct, sequence, lifetime, age, ext, payload_crc, payload)| {
                Spec {
                    flags,
                    primary_crc,
                    eids,
                    creation_time: ct,
                    sequence,
                    lifetime,
                    age,
                    ext,
                    payload_crc,
                    payload,
                }
            },
        )
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(512))]

    #[test]
    fn decode_total_on_arbitrary_bytes(bytes in proptest::collection::vec(any::<u8>(), 0..512)) {
        // Must return, never panic. Both modes.
        let _ = decode(&bytes, Mode::Structural);
        let _ = decode(&bytes, Mode::Strict);
    }

    #[test]
    fn decode_total_on_mutated_valid_bundle(
        edits in proptest::collection::vec((any::<usize>(), any::<u8>()), 1..8),
        cut in any::<usize>(),
    ) {
        let mut b = hex(RFC9173_A3_FINAL);
        for (i, v) in edits {
            let n = b.len();
            b[i % n] = v;
        }
        let cut = cut % (b.len() + 1);
        let _ = decode(&b[..cut], Mode::Structural);
        let _ = decode(&b, Mode::Strict);
    }

    #[test]
    fn canonical_round_trip(s in spec()) {
        let age_data = {
            let mut buf = [0u8; 9];
            let mut w = crate::cbor::Writer::new(&mut buf);
            w.uint(s.age).unwrap();
            let n = w.pos();
            buf[..n].to_vec()
        };
        let primary = PrimaryBlock {
            flags: s.flags,
            crc_type: s.primary_crc,
            destination: eid(&s.eids[0]),
            source: eid(&s.eids[1]),
            report_to: eid(&s.eids[2]),
            creation_time: s.creation_time,
            sequence: s.sequence,
            lifetime: s.lifetime,
        };
        let mut v = BundleView::new(primary);
        if s.creation_time == 0 {
            v.push(CanonicalBlock {
                block_type: BLOCK_BUNDLE_AGE,
                number: 2,
                flags: 0,
                crc_type: CrcType::None,
                data: &age_data,
            }).unwrap();
        }
        for (i, (ty, flags, crc, data)) in s.ext.iter().enumerate() {
            v.push(CanonicalBlock {
                block_type: *ty,
                number: 10 + i as u64,
                flags: *flags,
                crc_type: *crc,
                data,
            }).unwrap();
        }
        v.push(CanonicalBlock {
            block_type: BLOCK_PAYLOAD,
            number: 1,
            flags: 0,
            crc_type: s.payload_crc,
            data: &s.payload,
        }).unwrap();

        let mut out = std::vec![0u8; 2048];
        let n = encode(&v, &mut out).unwrap();
        let x = &out[..n];
        let d = decode(x, Mode::Strict).unwrap();
        prop_assert_eq!(d, v);
        let mut again = std::vec![0u8; 2048];
        let m = encode(&d, &mut again).unwrap();
        prop_assert_eq!(&again[..m], x);
    }
}
