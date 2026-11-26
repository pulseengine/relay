//! Endpoint IDs (RFC 9171 4.2.5.1).
//!
//! Decoded and encoded: the `dtn` scheme (code 1), including the null endpoint
//! `dtn:none` = `[1, 0]`, and the two-element `ipn` scheme (code 2),
//! `[2, [node, service]]`.
//!
//! ## Rejected: the RFC 9758 three-element ipn form and unknown schemes
//!
//! Owner decision 2026-11-25 (issue #626, SWREQ-RELAY-BP-P06): the RFC 9758
//! three-element `ipn` SSP `[2, [authority, node, service]]` and EIDs of
//! unknown schemes are rejected with a typed error and the bundle is dropped,
//! to be revisited when an RFC 9758 peer exists. Both forms get the verdict
//! [`OPEN_EID_VERDICT`] = [`OpenEidVerdict::Reject`], reported as
//! [`DecodeError::OpenEidForm`](crate::DecodeError::OpenEidForm). Neither form
//! is ever parsed as something else. Changing the verdict means changing that
//! one constant (and adding the accept path), nothing else.

use crate::DecodeError;
use crate::cbor::{Head, Reader, Writer};

/// URI scheme code of the dtn scheme.
pub const SCHEME_DTN: u64 = 1;
/// URI scheme code of the ipn scheme.
pub const SCHEME_IPN: u64 = 2;

/// A decoded endpoint ID. Text borrows the input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Eid<'a> {
    /// `dtn:none`, the null endpoint, encoded `[1, 0]`.
    DtnNone,
    /// A `dtn://node/demux` EID; holds the SSP text, which starts with `//`.
    Dtn(&'a str),
    /// A two-element `ipn:node.service` EID.
    Ipn {
        /// Node number.
        node: u64,
        /// Service number.
        service: u64,
    },
}

/// An EID form whose verdict is open on #626.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpenEidForm {
    /// The RFC 9758 three-element ipn SSP `[authority, node, service]`.
    IpnThreeElement,
    /// A scheme code other than 1 (dtn) and 2 (ipn); carries the code.
    UnknownScheme(u64),
}

/// The verdict applied to an [`OpenEidForm`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OpenEidVerdict {
    /// Reject with a typed error. The fail-safe default.
    Reject,
}

/// Owner decision 2026-11-25 (#626): open EID forms are rejected. See the
/// module documentation.
pub const OPEN_EID_VERDICT: OpenEidVerdict = OpenEidVerdict::Reject;

fn open_form(form: OpenEidForm) -> DecodeError {
    match OPEN_EID_VERDICT {
        OpenEidVerdict::Reject => DecodeError::OpenEidForm(form),
    }
}

/// True when `ssp` is an ASCII `//node-name/demux` dtn SSP (RFC 9171
/// 4.2.5.1.1): it starts with `//` and has a `/` name delimiter after the node
/// name. `dtn:none` is never text; it is uint 0.
pub fn dtn_ssp_is_valid(ssp: &[u8]) -> bool {
    if ssp.len() < 3 || ssp[0] != b'/' || ssp[1] != b'/' || !ssp.is_ascii() {
        return false;
    }
    ssp[2..].contains(&b'/')
}

/// Checks an [`Eid`] the encoder is asked to write.
pub fn validate(eid: &Eid<'_>) -> Result<(), DecodeError> {
    match eid {
        Eid::Dtn(s) if !dtn_ssp_is_valid(s.as_bytes()) => Err(DecodeError::InvalidDtnSsp),
        _ => Ok(()),
    }
}

/// Reads one EID. `depth` counts the enclosing inner indefinite arrays.
pub fn read<'a>(r: &mut Reader<'a>, depth: usize) -> Result<Eid<'a>, DecodeError> {
    let (kind, depth) = r.open_array(depth)?;
    if matches!(kind, Some(n) if n != 2) {
        return Err(DecodeError::InvalidEid);
    }
    let scheme = match r.head()? {
        Head::Uint(s) => s,
        _ => return Err(DecodeError::InvalidEid),
    };
    let eid = match scheme {
        SCHEME_DTN => match r.head()? {
            Head::Uint(0) => Eid::DtnNone,
            Head::Text(len) => {
                let ssp = r.take(len)?;
                if !dtn_ssp_is_valid(ssp) {
                    return Err(DecodeError::InvalidDtnSsp);
                }
                match core::str::from_utf8(ssp) {
                    Ok(s) => Eid::Dtn(s),
                    Err(_) => return Err(DecodeError::InvalidDtnSsp),
                }
            }
            _ => return Err(DecodeError::InvalidDtnSsp),
        },
        SCHEME_IPN => read_ipn_ssp(r, depth)?,
        other => return Err(open_form(OpenEidForm::UnknownScheme(other))),
    };
    r.close_array(kind, DecodeError::InvalidEid)?;
    Ok(eid)
}

fn read_ipn_ssp<'a>(r: &mut Reader<'a>, depth: usize) -> Result<Eid<'a>, DecodeError> {
    let (kind, _) = r.open_array(depth)?;
    let mut nums = [0u64; 3];
    let count = match kind {
        Some(n) => {
            if n != 2 && n != 3 {
                return Err(DecodeError::InvalidIpnSsp);
            }
            let mut i = 0;
            while i < n as usize {
                nums[i] = r.uint().map_err(|_| DecodeError::InvalidIpnSsp)?;
                i += 1;
            }
            n as usize
        }
        None => {
            let mut i = 0;
            while !r.peek_break() {
                if i == 3 {
                    return Err(DecodeError::InvalidIpnSsp);
                }
                nums[i] = r.uint().map_err(|_| DecodeError::InvalidIpnSsp)?;
                i += 1;
            }
            r.expect_break(DecodeError::InvalidIpnSsp)?;
            i
        }
    };
    match count {
        2 => Ok(Eid::Ipn {
            node: nums[0],
            service: nums[1],
        }),
        3 => Err(open_form(OpenEidForm::IpnThreeElement)),
        _ => Err(DecodeError::InvalidIpnSsp),
    }
}

/// Writes one EID in canonical form (definite arrays).
pub fn write(w: &mut Writer<'_>, eid: &Eid<'_>) -> Result<(), crate::cbor::Full> {
    w.array(2)?;
    match eid {
        Eid::DtnNone => {
            w.uint(SCHEME_DTN)?;
            w.uint(0)
        }
        Eid::Dtn(s) => {
            w.uint(SCHEME_DTN)?;
            w.text(s)
        }
        Eid::Ipn { node, service } => {
            w.uint(SCHEME_IPN)?;
            w.array(2)?;
            w.uint(*node)?;
            w.uint(*service)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::MAX_INNER_INDEF_DEPTH;

    fn rd(bytes: &[u8]) -> Result<Eid<'_>, DecodeError> {
        let mut r = Reader::new(bytes);
        let e = read(&mut r, 0)?;
        assert!(r.is_empty(), "EID did not consume its input");
        Ok(e)
    }

    fn wr(eid: &Eid<'_>, out: &mut [u8]) -> usize {
        let mut w = Writer::new(out);
        write(&mut w, eid).unwrap();
        w.pos()
    }

    #[test]
    fn dtn_none_decodes_and_encodes() {
        assert_eq!(rd(&[0x82, 0x01, 0x00]), Ok(Eid::DtnNone));
        let mut out = [0u8; 8];
        let n = wr(&Eid::DtnNone, &mut out);
        assert_eq!(&out[..n], &[0x82, 0x01, 0x00]);
    }

    #[test]
    fn dtn_text_ssp_round_trips() {
        // [1, "//node/svc"]
        let bytes = b"\x82\x01\x6a//node/svc";
        assert_eq!(rd(bytes), Ok(Eid::Dtn("//node/svc")));
        let mut out = [0u8; 16];
        let n = wr(&Eid::Dtn("//node/svc"), &mut out);
        assert_eq!(&out[..n], bytes);
        // Administrative endpoint: zero-length demux.
        assert_eq!(rd(b"\x82\x01\x67//node/"), Ok(Eid::Dtn("//node/")));
    }

    #[test]
    fn rejects_dtn_text_not_hier_part() {
        // "none" as text is not dtn:none (that is uint 0) and is not `//...`.
        assert_eq!(rd(b"\x82\x01\x64none"), Err(DecodeError::InvalidDtnSsp));
        assert_eq!(rd(b"\x82\x01\x68node/svc"), Err(DecodeError::InvalidDtnSsp));
        // No name delimiter after the node name.
        assert_eq!(rd(b"\x82\x01\x66//node"), Err(DecodeError::InvalidDtnSsp));
        // Non-ASCII.
        assert_eq!(
            rd(b"\x82\x01\x65//\xc3\xa9/"),
            Err(DecodeError::InvalidDtnSsp)
        );
        // dtn SSP that is a uint other than 0, or a byte string.
        assert_eq!(rd(&[0x82, 0x01, 0x01]), Err(DecodeError::InvalidDtnSsp));
        assert_eq!(rd(&[0x82, 0x01, 0x40]), Err(DecodeError::InvalidDtnSsp));
        assert_eq!(validate(&Eid::Dtn("none")), Err(DecodeError::InvalidDtnSsp));
    }

    #[test]
    fn ipn_two_element_round_trips() {
        // [2, [1, 2]] as in RFC 9173 A.1.1.1 (destination ipn:1.2).
        assert_eq!(
            rd(&[0x82, 0x02, 0x82, 0x01, 0x02]),
            Ok(Eid::Ipn {
                node: 1,
                service: 2
            })
        );
        let big = Eid::Ipn {
            node: u64::MAX,
            service: 0x1_0000,
        };
        let mut out = [0u8; 32];
        let n = wr(&big, &mut out);
        assert_eq!(rd(&out[..n]), Ok(big));
    }

    #[test]
    fn ipn_three_element_rejected() {
        // Decided (#626): the RFC 9758 form [2, [authority, node, service]].
        assert_eq!(OPEN_EID_VERDICT, OpenEidVerdict::Reject);
        assert_eq!(
            rd(&[0x82, 0x02, 0x83, 0x05, 0x01, 0x02]),
            Err(DecodeError::OpenEidForm(OpenEidForm::IpnThreeElement))
        );
        // Same verdict when the SSP array is indefinite.
        assert_eq!(
            rd(&[0x82, 0x02, 0x9f, 0x05, 0x01, 0x02, 0xff]),
            Err(DecodeError::OpenEidForm(OpenEidForm::IpnThreeElement))
        );
    }

    #[test]
    fn unknown_scheme_rejected() {
        // Decided (#626): unknown schemes are rejected, never mis-parsed.
        assert_eq!(OPEN_EID_VERDICT, OpenEidVerdict::Reject);
        assert_eq!(
            rd(&[0x82, 0x03, 0x00]),
            Err(DecodeError::OpenEidForm(OpenEidForm::UnknownScheme(3)))
        );
        assert_eq!(
            rd(&[0x82, 0x00, 0x00]),
            Err(DecodeError::OpenEidForm(OpenEidForm::UnknownScheme(0)))
        );
    }

    #[test]
    fn rejects_malformed_ipn_ssp() {
        assert_eq!(
            rd(&[0x82, 0x02, 0x81, 0x01]),
            Err(DecodeError::InvalidIpnSsp)
        );
        assert_eq!(
            rd(&[0x82, 0x02, 0x84, 0x01, 0x02, 0x03, 0x04]),
            Err(DecodeError::InvalidIpnSsp)
        );
        assert_eq!(
            rd(&[0x82, 0x02, 0x82, 0x01, 0x40]),
            Err(DecodeError::InvalidIpnSsp)
        );
        assert_eq!(
            rd(&[0x82, 0x02, 0x9f, 0x01, 0x02, 0x03, 0x04, 0xff]),
            Err(DecodeError::InvalidIpnSsp)
        );
        // ipn SSP that is not an array at all.
        assert_eq!(rd(&[0x82, 0x02, 0x01]), Err(DecodeError::UnexpectedType));
    }

    #[test]
    fn rejects_eid_not_two_items() {
        assert_eq!(rd(&[0x83, 0x01, 0x00, 0x00]), Err(DecodeError::InvalidEid));
        assert_eq!(rd(&[0x81, 0x01]), Err(DecodeError::InvalidEid));
        // Indefinite EID with a third item.
        assert_eq!(
            rd(&[0x9f, 0x01, 0x00, 0x00, 0xff]),
            Err(DecodeError::InvalidEid)
        );
        // Scheme is not a uint.
        assert_eq!(rd(&[0x82, 0x40, 0x00]), Err(DecodeError::InvalidEid));
    }

    #[test]
    fn eid_indefinite_nesting_is_bounded() {
        // Indefinite EID holding an indefinite ipn SSP: depth 2 from the top.
        let two = [0x9f, 0x02, 0x9f, 0x01, 0x02, 0xff, 0xff];
        let mut r = Reader::new(&two);
        assert!(read(&mut r, 0).is_ok());
        // The same EID inside one more indefinite level exceeds the bound.
        let mut r = Reader::new(&two);
        assert_eq!(
            read(&mut r, MAX_INNER_INDEF_DEPTH - 1),
            Err(DecodeError::IndefiniteDepthExceeded)
        );
    }
}
