//! Python string semantics the baseline method depended on.
//!
//! The baseline scripts ran `subprocess.run(..., text=True, errors='replace')`
//! and then used `str.splitlines()`, `str.strip()` and `str.split()`. Each of
//! those differs from the nearest Rust std call on some input, so the frozen
//! instrument re-implements the Python behaviour explicitly rather than
//! approximating it.

/// Decode git's stdout as Python's text-mode pipe did: UTF-8 with
/// `errors='replace'`, then universal-newline translation (`\r\n` and lone
/// `\r` both become `\n`).
pub fn decode_stdout(bytes: &[u8]) -> String {
    let s = String::from_utf8_lossy(bytes);
    let mut out = String::with_capacity(s.len());
    let mut chars = s.chars().peekable();
    while let Some(c) = chars.next() {
        if c == '\r' {
            if chars.peek() == Some(&'\n') {
                chars.next();
            }
            out.push('\n');
        } else {
            out.push(c);
        }
    }
    out
}

/// Line boundaries recognised by Python's `str.splitlines()`.
fn is_py_line_boundary(c: char) -> bool {
    matches!(
        c,
        '\n' | '\r'
            | '\u{0b}'
            | '\u{0c}'
            | '\u{1c}'
            | '\u{1d}'
            | '\u{1e}'
            | '\u{85}'
            | '\u{2028}'
            | '\u{2029}'
    )
}

/// Python `str.splitlines()` (no `keepends`): splits on every boundary in
/// [`is_py_line_boundary`] (`\r\n` counts once), and a trailing boundary does
/// not produce an empty final element. `""` gives `[]`.
pub fn splitlines(s: &str) -> Vec<&str> {
    let mut out = Vec::new();
    let mut start = 0;
    let bytes = s.as_bytes();
    let mut iter = s.char_indices().peekable();
    while let Some((i, c)) = iter.next() {
        if is_py_line_boundary(c) {
            out.push(&s[start..i]);
            let mut next = i + c.len_utf8();
            if c == '\r' && bytes.get(next) == Some(&b'\n') {
                iter.next();
                next += 1;
            }
            start = next;
        }
    }
    if start < s.len() {
        out.push(&s[start..]);
    }
    out
}

/// Python `str.isspace()` for one character. Rust's `char::is_whitespace`
/// (Unicode White_Space) lacks the four ASCII information separators that
/// Python treats as whitespace.
pub fn is_py_space(c: char) -> bool {
    c.is_whitespace() || matches!(c, '\u{1c}'..='\u{1f}')
}

/// Python `str.strip()` with no argument.
pub fn strip(s: &str) -> &str {
    s.trim_matches(is_py_space)
}

/// Python `str.split()` with no argument.
pub fn split_ws(s: &str) -> Vec<&str> {
    s.split(is_py_space).filter(|p| !p.is_empty()).collect()
}

/// Python `s[:n]` on a `str` (code points, not bytes).
pub fn prefix_chars(s: &str, n: usize) -> &str {
    match s.char_indices().nth(n) {
        Some((i, _)) => &s[..i],
        None => s,
    }
}

/// Python `f"{x:.0f}"`: round-half-to-even on the exact binary value.
/// Rust's `{:.0}` uses the same rule; this wrapper exists so the property is
/// pinned by a unit test rather than assumed.
pub fn fmt_0f(x: f64) -> String {
    format!("{x:.0}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn splitlines_matches_python() {
        // Python: "a\nb\r\nc\rd\x0ce\x1cf\n".splitlines()
        //   == ['a', 'b', 'c', 'd', 'e', 'f']
        assert_eq!(
            splitlines("a\nb\r\nc\rd\x0ce\x1cf\n"),
            vec!["a", "b", "c", "d", "e", "f"]
        );
        assert_eq!(splitlines(""), Vec::<&str>::new());
        assert_eq!(splitlines("\n"), vec![""]);
        assert_eq!(splitlines("a\n\nb"), vec!["a", "", "b"]);
        assert_eq!(splitlines("x\u{2028}y"), vec!["x", "y"]);
    }

    #[test]
    fn decode_translates_universal_newlines() {
        assert_eq!(decode_stdout(b"a\r\nb\rc\n"), "a\nb\nc\n");
        assert_eq!(decode_stdout(b"\xffok"), "\u{fffd}ok");
    }

    #[test]
    fn strip_and_split_include_info_separators() {
        assert_eq!(strip("\x1f  body \n\x1c"), "body");
        assert_eq!(split_ws("a \x1fb\n c"), vec!["a", "b", "c"]);
    }

    #[test]
    fn prefix_is_by_code_point() {
        assert_eq!(prefix_chars("a—b", 2), "a—");
        assert_eq!(prefix_chars("ab", 5), "ab");
    }

    #[test]
    fn zero_decimals_round_half_even_like_python() {
        // Python: f"{0.5:.0f}" '0', f"{1.5:.0f}" '2', f"{2.5:.0f}" '2',
        // f"{12.5:.0f}" '12', f"{100*7/27:.0f}" '26'
        assert_eq!(fmt_0f(0.5), "0");
        assert_eq!(fmt_0f(1.5), "2");
        assert_eq!(fmt_0f(2.5), "2");
        assert_eq!(fmt_0f(12.5), "12");
        assert_eq!(fmt_0f(100.0 * 7.0 / 27.0), "26");
    }
}
