//! The canonical encoder (SWREQ-RELAY-BP-P04, P05).
//!
//! [`encode`] runs the decoder's own bundle-level validation in
//! [`Mode::Strict`] BEFORE it writes a byte. It therefore cannot emit a bundle
//! whose primary CRC type is 0 without a BIB targeting block 0 (RFC 9171 4.3.1),
//! a bundle with creation time 0 that lacks exactly one Bundle Age block
//! (4.4.2), or a fragment (P07). There is no non-strict encoder.
//!
//! Output is the canonical subset: outer array indefinite, every inner array
//! definite, shortest-form heads. CRCs are computed over the written block with
//! only the value bytes zero (erratum 8043) and patched in big-endian. For any
//! input in that subset that decodes under [`Mode::Strict`], encoding the view
//! reproduces the input exactly (P05).

use crate::cbor::{BREAK, Full, INDEF_ARRAY, MAJOR_BYTES, Writer};
use crate::crc::block_crc;
use crate::decode::validate;
use crate::eid;
use crate::{BP_VERSION, BundleView, CrcType, EncodeError, Mode};

impl From<Full> for EncodeError {
    fn from(_: Full) -> Self {
        EncodeError::BufferTooSmall
    }
}

/// Writes the CRC field for the block that started at `start`, then patches
/// the computed value in.
fn finish_crc(w: &mut Writer<'_>, start: usize, crc_type: CrcType) -> Result<(), EncodeError> {
    let len = crc_type.value_len();
    if len == 0 {
        return Ok(());
    }
    w.head(MAJOR_BYTES, len as u64)?;
    let at = w.pos();
    w.raw_slice(&[0u8; 4][..len])?;
    let end = w.pos();
    let block = w.written(start, end).ok_or(EncodeError::BufferTooSmall)?;
    let rel = at - start;
    let value = block_crc(crc_type, block, rel).ok_or(EncodeError::BufferTooSmall)?;
    match crc_type {
        CrcType::Crc16 => w.patch(at, &(value as u16).to_be_bytes())?,
        CrcType::Crc32c => w.patch(at, &value.to_be_bytes())?,
        CrcType::None => {}
    }
    Ok(())
}

/// Encodes `bundle` into `out` and returns the number of bytes written.
///
/// Refuses (writing nothing meaningful) any bundle that fails the strict
/// checks: see the module documentation.
pub fn encode(bundle: &BundleView<'_>, out: &mut [u8]) -> Result<usize, EncodeError> {
    validate(bundle, Mode::Strict).map_err(EncodeError::Invalid)?;
    let mut w = Writer::new(out);
    w.raw(INDEF_ARRAY)?;

    let p = &bundle.primary;
    let start = w.pos();
    w.array(if p.crc_type == CrcType::None { 8 } else { 9 })?;
    w.uint(BP_VERSION)?;
    w.uint(p.flags)?;
    w.uint(p.crc_type.code())?;
    eid::write(&mut w, &p.destination)?;
    eid::write(&mut w, &p.source)?;
    eid::write(&mut w, &p.report_to)?;
    w.array(2)?;
    w.uint(p.creation_time)?;
    w.uint(p.sequence)?;
    w.uint(p.lifetime)?;
    finish_crc(&mut w, start, p.crc_type)?;

    for b in bundle.blocks() {
        let start = w.pos();
        w.array(if b.crc_type == CrcType::None { 5 } else { 6 })?;
        w.uint(b.block_type)?;
        w.uint(b.number)?;
        w.uint(b.flags)?;
        w.uint(b.crc_type.code())?;
        w.bytes(b.data)?;
        finish_crc(&mut w, start, b.crc_type)?;
    }
    w.raw(BREAK)?;
    Ok(w.pos())
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;
    use crate::decode::decode;
    use crate::{
        BLOCK_BIB, BLOCK_BUNDLE_AGE, BLOCK_PAYLOAD, CanonicalBlock, DecodeError, Eid,
        FLAG_IS_FRAGMENT, PrimaryBlock, StrictViolation,
    };
    use std::vec::Vec;

    /// A DTN time well past 2^32 ms, so it encodes with an 8-byte argument.
    pub(crate) const CT: u64 = 812_345_678_901;

    pub(crate) fn primary(creation_time: u64, crc_type: CrcType) -> PrimaryBlock<'static> {
        PrimaryBlock {
            flags: 0,
            crc_type,
            destination: Eid::Ipn {
                node: 1,
                service: 2,
            },
            source: Eid::Dtn("//falcon/bp"),
            report_to: Eid::DtnNone,
            creation_time,
            sequence: 40,
            lifetime: 1_000_000,
        }
    }

    pub(crate) fn payload(crc_type: CrcType, data: &'static [u8]) -> CanonicalBlock<'static> {
        CanonicalBlock {
            block_type: BLOCK_PAYLOAD,
            number: 1,
            flags: 0,
            crc_type,
            data,
        }
    }

    fn enc(v: &BundleView<'_>) -> Result<Vec<u8>, EncodeError> {
        let mut out = [0u8; 512];
        let n = encode(v, &mut out)?;
        Ok(out[..n].to_vec())
    }

    /// An encoded bundle with the given creation time and CRC types.
    pub(crate) fn crc_bundle_at(ct: u64, primary_crc: CrcType, payload_crc: CrcType) -> Vec<u8> {
        // Built through the decoder-facing structure, but bypassing the strict
        // checks so that negative strict tests can use it: encode a strictly
        // valid twin, then patch the creation time if asked for 0.
        let mut v = BundleView::new(primary(CT, primary_crc));
        v.push(payload(payload_crc, b"relay-bp")).unwrap();
        let bytes = enc(&v).unwrap();
        if ct == CT {
            return bytes;
        }
        assert_eq!(ct, 0, "only CT or 0 supported");
        // Re-encode by hand with creation time 0 and no age block: the strict
        // encoder refuses that, which is the point of the strict tests.
        let mut v0 = BundleView::new(primary(0, primary_crc));
        v0.push(payload(payload_crc, b"relay-bp")).unwrap();
        assert_eq!(
            encode(&v0, &mut [0u8; 256]),
            Err(EncodeError::Invalid(DecodeError::Strict(
                StrictViolation::BundleAgeMissing
            )))
        );
        let mut out = std::vec![0u8; 256];
        let n = encode_unchecked_for_tests(&v0, &mut out);
        out.truncate(n);
        out
    }

    /// Same bytes the encoder writes, without the strict gate. Test-only: lets
    /// strict-decoder tests build violating inputs without a second encoder.
    fn encode_unchecked_for_tests(v: &BundleView<'_>, out: &mut [u8]) -> usize {
        let mut age = BundleView::new(v.primary);
        age.push(CanonicalBlock {
            block_type: BLOCK_BUNDLE_AGE,
            number: 2,
            flags: 0,
            crc_type: CrcType::None,
            data: &[0x00],
        })
        .unwrap();
        for b in v.blocks() {
            age.push(*b).unwrap();
        }
        let n = encode(&age, out).unwrap();
        // Remove the age block (85 07 02 00 00 41 00 = 7 bytes) after the primary.
        let at = out[..n]
            .windows(7)
            .position(|w| w == [0x85, 0x07, 0x02, 0x00, 0x00, 0x41, 0x00])
            .unwrap();
        out.copy_within(at + 7..n, at);
        n - 7
    }

    /// An encoded, strictly valid bundle with the given CRC types.
    pub(crate) fn crc_bundle(primary_crc: CrcType, payload_crc: CrcType) -> Vec<u8> {
        crc_bundle_at(CT, primary_crc, payload_crc)
    }

    #[test]
    fn encoder_refuses_primary_crc_none_without_bib() {
        let mut v = BundleView::new(primary(CT, CrcType::None));
        v.push(payload(CrcType::None, b"x")).unwrap();
        assert_eq!(
            enc(&v),
            Err(EncodeError::Invalid(DecodeError::Strict(
                StrictViolation::PrimaryCrcMissing
            )))
        );
        // A BIB that targets only the payload does not lift the rule.
        let mut v = BundleView::new(primary(CT, CrcType::None));
        v.push(CanonicalBlock {
            block_type: BLOCK_BIB,
            number: 2,
            flags: 0,
            crc_type: CrcType::None,
            data: &[0x81, 0x01],
        })
        .unwrap();
        v.push(payload(CrcType::None, b"x")).unwrap();
        assert_eq!(
            enc(&v),
            Err(EncodeError::Invalid(DecodeError::Strict(
                StrictViolation::PrimaryCrcMissing
            )))
        );
    }

    #[test]
    fn encoder_accepts_primary_crc_none_with_bib_targeting_primary() {
        let mut v = BundleView::new(primary(CT, CrcType::None));
        // ASB starting with security targets [0] (RFC 9172 3.6).
        v.push(CanonicalBlock {
            block_type: BLOCK_BIB,
            number: 2,
            flags: 0,
            crc_type: CrcType::None,
            data: &[0x81, 0x00],
        })
        .unwrap();
        v.push(payload(CrcType::None, b"x")).unwrap();
        let bytes = enc(&v).unwrap();
        assert_eq!(decode(&bytes, Mode::Strict).unwrap(), v);
    }

    #[test]
    fn encoder_refuses_creation_time_zero_without_age_block() {
        let mut v = BundleView::new(primary(0, CrcType::Crc16));
        v.push(payload(CrcType::None, b"x")).unwrap();
        assert_eq!(
            enc(&v),
            Err(EncodeError::Invalid(DecodeError::Strict(
                StrictViolation::BundleAgeMissing
            )))
        );
        // With exactly one Bundle Age block it is accepted.
        let mut v = BundleView::new(primary(0, CrcType::Crc16));
        v.push(CanonicalBlock {
            block_type: BLOCK_BUNDLE_AGE,
            number: 2,
            flags: 0,
            crc_type: CrcType::None,
            data: &[0x19, 0x01, 0x2c],
        })
        .unwrap();
        v.push(payload(CrcType::None, b"x")).unwrap();
        let bytes = enc(&v).unwrap();
        assert_eq!(
            decode(&bytes, Mode::Strict).unwrap().bundle_age(),
            Some(300)
        );
    }

    #[test]
    fn encoder_refuses_fragment_flag() {
        let mut p = primary(CT, CrcType::Crc32c);
        p.flags = FLAG_IS_FRAGMENT | 0x04;
        let mut v = BundleView::new(p);
        v.push(payload(CrcType::None, b"x")).unwrap();
        assert_eq!(
            enc(&v),
            Err(EncodeError::Invalid(DecodeError::FragmentRejected))
        );
    }

    #[test]
    fn encoder_refuses_structurally_invalid_bundle() {
        // No payload.
        let v = BundleView::new(primary(CT, CrcType::Crc16));
        assert_eq!(
            enc(&v),
            Err(EncodeError::Invalid(DecodeError::MissingPayload))
        );
        // Invalid dtn SSP.
        let mut p = primary(CT, CrcType::Crc16);
        p.destination = Eid::Dtn("node");
        let mut v = BundleView::new(p);
        v.push(payload(CrcType::None, b"x")).unwrap();
        assert_eq!(
            enc(&v),
            Err(EncodeError::Invalid(DecodeError::InvalidDtnSsp))
        );
    }

    #[test]
    fn encoder_reports_buffer_too_small() {
        let mut v = BundleView::new(primary(CT, CrcType::Crc32c));
        v.push(payload(CrcType::Crc32c, b"relay-bp")).unwrap();
        let full = enc(&v).unwrap();
        for cap in 0..full.len() {
            let mut out = std::vec![0u8; cap];
            assert_eq!(
                encode(&v, &mut out),
                Err(EncodeError::BufferTooSmall),
                "cap {cap}"
            );
        }
    }

    #[test]
    fn crc_value_is_big_endian_on_wire() {
        let bytes = crc_bundle(CrcType::Crc32c, CrcType::Crc16);
        // Payload block (17 bytes): 86 01 01 00 01 48 "relay-bp" 42 <hi> <lo>, then ff.
        let n = bytes.len();
        let block = &bytes[n - 1 - 17..n - 1];
        assert_eq!(&block[..6], &[0x86, 0x01, 0x01, 0x00, 0x01, 0x48]);
        let mut zeroed = block.to_vec();
        zeroed[15] = 0;
        zeroed[16] = 0;
        let crc = crate::crc::crc16_x25(&zeroed);
        assert_eq!(&block[14..], &[0x42, (crc >> 8) as u8, crc as u8]);
    }

    #[test]
    fn round_trip_crc16_and_crc32c_bundles() {
        for pc in [CrcType::Crc16, CrcType::Crc32c] {
            for bc in [CrcType::None, CrcType::Crc16, CrcType::Crc32c] {
                let x = crc_bundle(pc, bc);
                let v = decode(&x, Mode::Strict).unwrap();
                assert_eq!(enc(&v).unwrap(), x, "primary {pc:?}, payload {bc:?}");
            }
        }
    }

    #[test]
    fn round_trip_rfc9173_a3_final_bytes() {
        // RFC 9173 Appendix A.3.5 (an external encoder's bytes): its BIB targets
        // block 0 and it carries one Bundle Age block, so it passes the strict
        // rules this crate implements, and encode(decode(x)) == x byte for byte.
        // A.3 is DISPUTED for its strict verdict (see vectors.rs); this test
        // only claims the canonical byte round trip.
        let x = crate::vectors::hex(crate::vectors::RFC9173_A3_FINAL);
        let v = decode(&x, Mode::Strict).unwrap();
        assert_eq!(enc(&v).unwrap(), x);
    }

    #[test]
    fn non_canonical_input_reencodes_canonically() {
        // An indefinite inner block is accepted but is outside the canonical
        // subset: re-encoding yields the definite form, not the input.
        let mut x = crc_bundle(CrcType::Crc16, CrcType::None);
        let n = x.len();
        // Last block 85 01 01 00 00 48 <8 bytes> -> 9f ... ff.
        let start = n - 1 - 14;
        assert_eq!(x[start], 0x85);
        x[start] = 0x9f;
        x.insert(n - 1, 0xff);
        let v = decode(&x, Mode::Strict).unwrap();
        let y = enc(&v).unwrap();
        assert_ne!(y, x);
        assert_eq!(y, crc_bundle(CrcType::Crc16, CrcType::None));
    }
}
