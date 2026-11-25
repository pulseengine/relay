//! Fixed-capacity CBOR reader and writer for the BPv7 canonical subset.
//!
//! RFC 9171 4.1 requires the core deterministic encoding of RFC 8949 4.2.1
//! except that indefinite-length items are allowed. BPv7 uses only unsigned
//! integers, byte strings, text strings and arrays, so that is all this layer
//! handles; every other major type reads as [`Head::Other`]. The reader rejects
//! non-shortest arguments and indefinite strings. The writer emits only
//! shortest-form heads and definite inner arrays. Nothing here recurses or
//! allocates.

use crate::{DecodeError, MAX_INNER_INDEF_DEPTH};

/// CBOR major type 0: unsigned integer.
pub const MAJOR_UINT: u8 = 0;
/// CBOR major type 2: byte string.
pub const MAJOR_BYTES: u8 = 2;
/// CBOR major type 3: text string.
pub const MAJOR_TEXT: u8 = 3;
/// CBOR major type 4: array.
pub const MAJOR_ARRAY: u8 = 4;
/// Initial byte of an indefinite-length array.
pub const INDEF_ARRAY: u8 = 0x9f;
/// The "break" stop code.
pub const BREAK: u8 = 0xff;

/// One decoded CBOR head.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Head {
    /// Major 0 with its value.
    Uint(u64),
    /// Major 2 with its length.
    Bytes(u64),
    /// Major 3 with its length.
    Text(u64),
    /// Major 4: `Some(n)` definite, `None` indefinite.
    Array(Option<u64>),
    /// The break stop code.
    Break,
    /// Any other well-formed head (negative int, map, tag, simple, float).
    Other,
}

/// A bounds-checked cursor over a borrowed byte slice.
#[derive(Debug, Clone, Copy)]
pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    /// A reader at the start of `buf`.
    pub const fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Current offset into the input.
    pub const fn pos(&self) -> usize {
        self.pos
    }

    /// True when every byte has been consumed.
    pub const fn is_empty(&self) -> bool {
        self.pos >= self.buf.len()
    }

    /// The bytes in `start..end` of the input, or `Truncated`.
    pub fn slice(&self, start: usize, end: usize) -> Result<&'a [u8], DecodeError> {
        self.buf.get(start..end).ok_or(DecodeError::Truncated)
    }

    fn byte(&mut self) -> Result<u8, DecodeError> {
        match self.buf.get(self.pos) {
            Some(&b) => {
                self.pos += 1;
                Ok(b)
            }
            None => Err(DecodeError::Truncated),
        }
    }

    fn be(&mut self, n: usize) -> Result<u64, DecodeError> {
        let mut v: u64 = 0;
        let mut i = 0;
        while i < n {
            v = (v << 8) | u64::from(self.byte()?);
            i += 1;
        }
        Ok(v)
    }

    /// True when the next byte is the break code (does not consume it).
    pub fn peek_break(&self) -> bool {
        self.buf.get(self.pos) == Some(&BREAK)
    }

    /// Consumes a break code, or returns `err`.
    pub fn expect_break(&mut self, err: DecodeError) -> Result<(), DecodeError> {
        if self.peek_break() {
            self.pos += 1;
            Ok(())
        } else {
            Err(err)
        }
    }

    /// Reads one head, rejecting non-shortest arguments and indefinite strings.
    pub fn head(&mut self) -> Result<Head, DecodeError> {
        let ib = self.byte()?;
        let major = ib >> 5;
        let ai = ib & 0x1f;
        let arg = match ai {
            0..=23 => u64::from(ai),
            24 => {
                let v = self.be(1)?;
                if v < 24 {
                    return Err(DecodeError::NonShortestForm);
                }
                v
            }
            25 => {
                let v = self.be(2)?;
                if v <= 0xff {
                    return Err(DecodeError::NonShortestForm);
                }
                v
            }
            26 => {
                let v = self.be(4)?;
                if v <= 0xffff {
                    return Err(DecodeError::NonShortestForm);
                }
                v
            }
            27 => {
                let v = self.be(8)?;
                if v <= 0xffff_ffff {
                    return Err(DecodeError::NonShortestForm);
                }
                v
            }
            28..=30 => return Err(DecodeError::MalformedHead),
            _ => {
                // ai == 31: indefinite length / break.
                return match major {
                    MAJOR_BYTES | MAJOR_TEXT => Err(DecodeError::IndefiniteString),
                    MAJOR_ARRAY => Ok(Head::Array(None)),
                    5 => Ok(Head::Other),
                    7 => Ok(Head::Break),
                    _ => Err(DecodeError::MalformedHead),
                };
            }
        };
        Ok(match major {
            MAJOR_UINT => Head::Uint(arg),
            MAJOR_BYTES => Head::Bytes(arg),
            MAJOR_TEXT => Head::Text(arg),
            MAJOR_ARRAY => Head::Array(Some(arg)),
            _ => Head::Other,
        })
    }

    /// Reads an unsigned integer.
    pub fn uint(&mut self) -> Result<u64, DecodeError> {
        match self.head()? {
            Head::Uint(v) => Ok(v),
            _ => Err(DecodeError::UnexpectedType),
        }
    }

    /// Takes `len` raw bytes.
    pub fn take(&mut self, len: u64) -> Result<&'a [u8], DecodeError> {
        let len = usize::try_from(len).map_err(|_| DecodeError::Truncated)?;
        let end = self.pos.checked_add(len).ok_or(DecodeError::Truncated)?;
        let s = self.buf.get(self.pos..end).ok_or(DecodeError::Truncated)?;
        self.pos = end;
        Ok(s)
    }

    /// Reads a definite-length byte string and returns its content.
    pub fn bytes(&mut self) -> Result<&'a [u8], DecodeError> {
        match self.head()? {
            Head::Bytes(len) => self.take(len),
            _ => Err(DecodeError::UnexpectedType),
        }
    }

    /// Opens an inner array. `depth` is the number of enclosing inner
    /// indefinite arrays; the returned depth includes this one if it is
    /// indefinite. Fails with `IndefiniteDepthExceeded` past
    /// [`MAX_INNER_INDEF_DEPTH`].
    pub fn open_array(&mut self, depth: usize) -> Result<(Option<u64>, usize), DecodeError> {
        match self.head()? {
            Head::Array(Some(n)) => Ok((Some(n), depth)),
            Head::Array(None) => {
                let d = depth.saturating_add(1);
                if d > MAX_INNER_INDEF_DEPTH {
                    Err(DecodeError::IndefiniteDepthExceeded)
                } else {
                    Ok((None, d))
                }
            }
            _ => Err(DecodeError::UnexpectedType),
        }
    }

    /// Closes an array opened with [`Reader::open_array`] once its expected
    /// items are read: a definite array needs nothing, an indefinite one must
    /// be followed by break, else `err`.
    pub fn close_array(&mut self, kind: Option<u64>, err: DecodeError) -> Result<(), DecodeError> {
        match kind {
            Some(_) => Ok(()),
            None => self.expect_break(err),
        }
    }
}

/// A bounds-checked writer into a caller-owned buffer.
#[derive(Debug)]
pub struct Writer<'b> {
    buf: &'b mut [u8],
    pos: usize,
}

/// The output buffer is full.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Full;

impl<'b> Writer<'b> {
    /// A writer at the start of `buf`.
    pub fn new(buf: &'b mut [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    /// Bytes written so far.
    pub const fn pos(&self) -> usize {
        self.pos
    }

    /// The written bytes in `start..end`.
    pub fn written(&self, start: usize, end: usize) -> Option<&[u8]> {
        if end > self.pos {
            return None;
        }
        self.buf.get(start..end)
    }

    /// Overwrites already-written bytes at `at`.
    pub fn patch(&mut self, at: usize, bytes: &[u8]) -> Result<(), Full> {
        let end = at.checked_add(bytes.len()).ok_or(Full)?;
        if end > self.pos {
            return Err(Full);
        }
        let dst = self.buf.get_mut(at..end).ok_or(Full)?;
        dst.copy_from_slice(bytes);
        Ok(())
    }

    /// Writes one raw byte.
    pub fn raw(&mut self, b: u8) -> Result<(), Full> {
        let slot = self.buf.get_mut(self.pos).ok_or(Full)?;
        *slot = b;
        self.pos += 1;
        Ok(())
    }

    /// Writes raw bytes.
    pub fn raw_slice(&mut self, bytes: &[u8]) -> Result<(), Full> {
        let end = self.pos.checked_add(bytes.len()).ok_or(Full)?;
        let dst = self.buf.get_mut(self.pos..end).ok_or(Full)?;
        dst.copy_from_slice(bytes);
        self.pos = end;
        Ok(())
    }

    /// Writes a head in shortest form.
    pub fn head(&mut self, major: u8, arg: u64) -> Result<(), Full> {
        let m = major << 5;
        if arg < 24 {
            self.raw(m | arg as u8)
        } else if arg <= 0xff {
            self.raw(m | 24)?;
            self.raw(arg as u8)
        } else if arg <= 0xffff {
            self.raw(m | 25)?;
            self.raw_slice(&(arg as u16).to_be_bytes())
        } else if arg <= 0xffff_ffff {
            self.raw(m | 26)?;
            self.raw_slice(&(arg as u32).to_be_bytes())
        } else {
            self.raw(m | 27)?;
            self.raw_slice(&arg.to_be_bytes())
        }
    }

    /// Writes an unsigned integer.
    pub fn uint(&mut self, v: u64) -> Result<(), Full> {
        self.head(MAJOR_UINT, v)
    }

    /// Writes a definite-length byte string.
    pub fn bytes(&mut self, b: &[u8]) -> Result<(), Full> {
        self.head(MAJOR_BYTES, b.len() as u64)?;
        self.raw_slice(b)
    }

    /// Writes a definite-length text string.
    pub fn text(&mut self, s: &str) -> Result<(), Full> {
        self.head(MAJOR_TEXT, s.len() as u64)?;
        self.raw_slice(s.as_bytes())
    }

    /// Writes a definite array head.
    pub fn array(&mut self, n: u64) -> Result<(), Full> {
        self.head(MAJOR_ARRAY, n)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn head_of(bytes: &[u8]) -> Result<Head, DecodeError> {
        Reader::new(bytes).head()
    }

    #[test]
    fn rejects_non_shortest_uint_each_width() {
        assert_eq!(head_of(&[0x18, 0x05]), Err(DecodeError::NonShortestForm));
        assert_eq!(
            head_of(&[0x19, 0x00, 0xff]),
            Err(DecodeError::NonShortestForm)
        );
        assert_eq!(
            head_of(&[0x1a, 0x00, 0x00, 0xff, 0xff]),
            Err(DecodeError::NonShortestForm)
        );
        assert_eq!(
            head_of(&[0x1b, 0, 0, 0, 0, 0xff, 0xff, 0xff, 0xff]),
            Err(DecodeError::NonShortestForm)
        );
        // Non-shortest lengths are rejected the same way for strings and arrays.
        assert_eq!(head_of(&[0x58, 0x01]), Err(DecodeError::NonShortestForm));
        assert_eq!(head_of(&[0x98, 0x02]), Err(DecodeError::NonShortestForm));
    }

    #[test]
    fn accepts_shortest_form_boundaries() {
        let cases: [(&[u8], u64); 8] = [
            (&[0x17], 23),
            (&[0x18, 0x18], 24),
            (&[0x18, 0xff], 255),
            (&[0x19, 0x01, 0x00], 256),
            (&[0x19, 0xff, 0xff], 65535),
            (&[0x1a, 0x00, 0x01, 0x00, 0x00], 65536),
            (&[0x1a, 0xff, 0xff, 0xff, 0xff], 0xffff_ffff),
            (&[0x1b, 0, 0, 0, 1, 0, 0, 0, 0], 0x1_0000_0000),
        ];
        for (bytes, v) in cases {
            assert_eq!(head_of(bytes), Ok(Head::Uint(v)), "{bytes:02x?}");
            let mut out = [0u8; 9];
            let mut w = Writer::new(&mut out);
            w.uint(v).unwrap();
            assert_eq!(&out[..bytes.len()], bytes);
        }
    }

    #[test]
    fn rejects_reserved_additional_info() {
        for ai in 28u8..=30 {
            assert_eq!(head_of(&[ai]), Err(DecodeError::MalformedHead));
        }
        // ai 31 on major 0 is not well-formed.
        assert_eq!(head_of(&[0x1f]), Err(DecodeError::MalformedHead));
    }

    #[test]
    fn rejects_indefinite_strings() {
        assert_eq!(head_of(&[0x5f]), Err(DecodeError::IndefiniteString));
        assert_eq!(head_of(&[0x7f]), Err(DecodeError::IndefiniteString));
    }

    #[test]
    fn truncated_argument_is_error() {
        assert_eq!(head_of(&[]), Err(DecodeError::Truncated));
        assert_eq!(head_of(&[0x19, 0x01]), Err(DecodeError::Truncated));
        assert_eq!(
            Reader::new(&[0x45, 1, 2]).bytes(),
            Err(DecodeError::Truncated)
        );
        // A length that cannot fit the input never indexes out of bounds.
        assert_eq!(
            Reader::new(&[0x5b, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff]).bytes(),
            Err(DecodeError::Truncated)
        );
    }
}
