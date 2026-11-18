//! check-engine-mirrors — each plain engine mirror is the strip of its Verus source.
//!
//! relay#253. Every `crates/<c>/plain/src/engine.rs` says it is "generated from
//! Verus source via verus-strip ... Do not edit manually", and on 2026-11-17 none
//! of the 16 was (relay#253 comment: 0-105 lines of hand-written residue each).
//! This check holds the converged ones to their source until generation replaces
//! the hand mirrors outright.
//!
//! For each crate listed in scripts/engine-mirror-converged.txt:
//!   1. run `verus-strip` on `src/engine.rs` (the Verus source);
//!   2. run `verus-strip` on `plain/src/engine.rs` too, so both sides go through
//!      the same printer (prettyplease) and layout cannot differ;
//!   3. normalise five MECHANICAL classes on both sides, and only these:
//!        - `//` comments, including `//!` and `///` doc comments;
//!        - `Debug` in a derive list (the mirror adds it for test output);
//!        - `pub` on a struct field (Verus needs it for specs);
//!        - a trailing `#[cfg(test)] mod tests` / `#[cfg(kani)] mod kani_proofs`
//!          (plain-only test code);
//!        - an empty inherent `impl X {}` (left when the strip removes an
//!          impl's spec fns, e.g. relay-mm's `impl MmConfig { spec fn inv }`);
//!      plus vstd's `arr.set(i, v)`, rewritten to `arr[i] = v` (rules_verus#34:
//!      verus-strip passes it through, and plain arrays have no `set`), also
//!      when the printer puts the receiver on the line before `.set(`;
//!   4. compare. Any remaining difference fails, and the diff is printed.
//!
//! A listed crate that is missing either file fails. An empty list fails: the
//! check would have compared nothing. Unlisted engines are reported with their
//! residual line count, so convergence is visible, but they do not fail.
//!
//! Build and run:
//!   rustc --edition 2021 -O scripts/check-engine-mirrors.rs -o cem
//!   VERUS_STRIP=/path/to/verus-strip ./cem
//! Tests (no verus-strip needed; they cover the normaliser and the verdict):
//!   rustc --edition 2021 --test scripts/check-engine-mirrors.rs -o cem-test && ./cem-test
//! Exit: 0 ok, 1 drift or nothing compared, 2 could not run verus-strip or read.

use std::fs;
use std::path::{Path, PathBuf};
use std::process::{Command, ExitCode};

const LIST: &str = "scripts/engine-mirror-converged.txt";

/// Removes `//` comments (line-level: a line whose first non-space token is
/// `//`). verus-strip prints doc comments as `///` / `//!` lines.
fn drop_comment_lines(s: &str) -> String {
    s.lines().filter(|l| !l.trim_start().starts_with("//")).map(|l| format!("{l}\n")).collect()
}

/// Cuts a trailing plain-only test or Kani module.
fn drop_test_modules(s: &str) -> String {
    let mut out = s.to_string();
    for head in ["#[cfg(test)]\nmod tests", "#[cfg(kani)]\nmod kani_proofs"] {
        if let Some(i) = out.find(head) {
            out.truncate(i);
        }
    }
    out
}

fn drop_debug_derive(s: &str) -> String {
    s.replace(", Debug)", ")").replace("(Debug, ", "(").replace("#[derive(Debug)]\n", "")
}

/// `    pub name: Type,` -> `    name: Type,` (an indented field line only).
fn drop_field_pub(s: &str) -> String {
    s.lines()
        .map(|l| {
            let ind = l.len() - l.trim_start().len();
            let t = l.trim_start();
            let is_field = ind > 0
                && t.starts_with("pub ")
                && t.ends_with(',')
                && t[4..].split(':').next().is_some_and(|n| {
                    !n.is_empty() && n.chars().all(|c| c.is_ascii_alphanumeric() || c == '_')
                });
            if is_field {
                format!("{}{}\n", &l[..ind], &t[4..])
            } else {
                format!("{l}\n")
            }
        })
        .collect()
}

/// `place.set(idx, val)` -> `place[idx] = val`, balancing brackets so nested
/// calls and struct literals survive. `place` is the run of
/// `[A-Za-z0-9_.]` / balanced `[..]` characters before `.set(`.
fn set_to_index(s: &str) -> String {
    let mut s = s.to_string();
    let mut from = 0;
    while let Some(rel) = s[from..].find(".set(") {
        let dot = from + rel;
        // receiver end: the printer may break a method chain, putting the
        // receiver on the line before `.set(` (relay-to's `self.entries`).
        let b = s.as_bytes();
        let mut end = dot;
        while end > 0 && (b[end - 1] as char).is_ascii_whitespace() {
            end -= 1;
        }
        // receiver start
        let mut st = end;
        let mut depth = 0i32;
        while st > 0 {
            let c = b[st - 1] as char;
            if c == ']' {
                depth += 1;
            } else if c == '[' {
                if depth == 0 {
                    break;
                }
                depth -= 1;
            } else if !(c.is_ascii_alphanumeric() || c == '_' || c == '.' || depth > 0) {
                break;
            }
            st -= 1;
        }
        if st == end {
            from = dot + 5;
            continue;
        }
        // argument span
        let open = dot + 4;
        let mut d = 0i32;
        let mut comma = None;
        let mut close = None;
        for (i, c) in s[open..].char_indices() {
            match c {
                '(' | '[' | '{' => d += 1,
                ')' | ']' | '}' => {
                    d -= 1;
                    if d == 0 {
                        close = Some(open + i);
                        break;
                    }
                }
                ',' if d == 1 && comma.is_none() => comma = Some(open + i),
                _ => {}
            }
        }
        let (Some(comma), Some(close)) = (comma, close) else {
            from = dot + 5;
            continue;
        };
        let place = s[st..end].to_string();
        let idx = s[open + 1..comma].trim().to_string();
        let val = s[comma + 1..close].trim().trim_end_matches(',').trim().to_string();
        let rep = format!("{place}[{idx}] = {val}");
        s.replace_range(st..=close, &rep);
        from = st + rep.len();
    }
    s
}

/// Drops an empty `impl X {}` line. verus-strip removes spec and proof fns,
/// so an impl that held only `spec fn inv` (relay-mm's `MmConfig`) prints as
/// an empty block the mirror has no reason to carry. It has no behaviour.
fn drop_empty_impls(s: &str) -> String {
    s.lines()
        .filter(|l| {
            let t = l.trim();
            !(t.starts_with("impl ") && t.ends_with(" {}") && !t.contains(" for ") && !t.contains('('))
        })
        .map(|l| format!("{l}\n"))
        .collect()
}

/// The mechanical normalisation, minus the reprint (done by verus-strip).
fn normalise(printed: &str) -> String {
    let s = drop_comment_lines(printed);
    let s = drop_empty_impls(&s);
    let s = drop_test_modules(&s);
    let s = drop_debug_derive(&s);
    let s = drop_field_pub(&s);
    set_to_index(&s)
}

/// Collapses all whitespace runs so the post-normalisation comparison does not
/// depend on how the printer wrapped a rewritten `.set` statement.
fn squash(s: &str) -> String {
    s.split_whitespace().collect::<Vec<_>>().join(" ")
}

/// Line-level diff summary for the report (first differing lines only).
fn first_diff(a: &str, b: &str) -> String {
    let (la, lb): (Vec<_>, Vec<_>) = (a.lines().collect(), b.lines().collect());
    let n = la.len().max(lb.len());
    let mut out = String::new();
    let mut shown = 0;
    for i in 0..n {
        let (x, y) = (la.get(i).copied().unwrap_or(""), lb.get(i).copied().unwrap_or(""));
        if x.split_whitespace().ne(y.split_whitespace()) {
            out += &format!("    line {}:\n      source: {x}\n      mirror: {y}\n", i + 1);
            shown += 1;
            if shown == 6 {
                break;
            }
        }
    }
    out
}

fn strip(bin: &Path, file: &Path, out: &Path) -> Result<String, String> {
    let st = Command::new(bin)
        .arg(file)
        .arg("-o")
        .arg(out)
        .output()
        .map_err(|e| format!("cannot run {}: {e}", bin.display()))?;
    if !st.status.success() {
        return Err(format!(
            "verus-strip failed on {}: {}",
            file.display(),
            String::from_utf8_lossy(&st.stderr).trim()
        ));
    }
    fs::read_to_string(out).map_err(|e| format!("cannot read {}: {e}", out.display()))
}

/// Strips `text` through verus-strip again (the reprint after normalising).
fn reprint(bin: &Path, text: &str, tmp: &Path, tag: &str) -> Result<String, String> {
    let inp = tmp.join(format!("{tag}.in.rs"));
    fs::write(&inp, text).map_err(|e| e.to_string())?;
    strip(bin, &inp, &tmp.join(format!("{tag}.out.rs")))
}

/// Both sides stripped, normalised and reprinted, comments dropped.
fn normalised_pair(bin: &Path, krate: &str, tmp: &Path) -> Result<(String, String), String> {
    let src = PathBuf::from(format!("crates/{krate}/src/engine.rs"));
    let plain = PathBuf::from(format!("crates/{krate}/plain/src/engine.rs"));
    for p in [&src, &plain] {
        if !p.is_file() {
            return Err(format!("{}: missing", p.display()));
        }
    }
    let a = strip(bin, &src, &tmp.join(format!("{krate}.src.rs")))?;
    let b = strip(bin, &plain, &tmp.join(format!("{krate}.plain.rs")))?;
    let a = reprint(bin, &normalise(&a), tmp, &format!("{krate}.a"))?;
    let b = reprint(bin, &normalise(&b), tmp, &format!("{krate}.b"))?;
    Ok((drop_comment_lines(&a), drop_comment_lines(&b)))
}

fn compare(bin: &Path, krate: &str, tmp: &Path) -> Result<Option<String>, String> {
    let (a, b) = normalised_pair(bin, krate, tmp)?;
    Ok(if squash(&a) == squash(&b) { None } else { Some(first_diff(&a, &b)) })
}

/// Count of differing lines (whitespace-insensitive), for the progress report.
fn count_diff(a: &str, b: &str) -> usize {
    let (la, lb): (Vec<_>, Vec<_>) = (a.lines().collect(), b.lines().collect());
    (0..la.len().max(lb.len()))
        .filter(|&i| {
            let (x, y) = (la.get(i).copied().unwrap_or(""), lb.get(i).copied().unwrap_or(""));
            x.split_whitespace().ne(y.split_whitespace())
        })
        .count()
}

fn residual_lines(bin: &Path, krate: &str, tmp: &Path) -> Option<usize> {
    let (a, b) = normalised_pair(bin, krate, tmp).ok()?;
    Some(if squash(&a) == squash(&b) { 0 } else { count_diff(&a, &b) })
}

/// The verdict over the per-crate results, separated out so it is testable.
fn verdict(listed: &[String], results: &[(String, Result<Option<String>, String>)]) -> (bool, Vec<String>) {
    let mut lines = Vec::new();
    let mut ok = !listed.is_empty();
    if listed.is_empty() {
        lines.push(format!("FAIL  {LIST} lists no crate: this check compared nothing"));
    }
    for (k, r) in results {
        match r {
            Ok(None) => lines.push(format!("ok    {k}: plain mirror is the strip of the Verus source")),
            Ok(Some(d)) => {
                ok = false;
                lines.push(format!("FAIL  {k}: plain mirror drifted from the Verus source\n{d}"));
            }
            Err(e) => {
                ok = false;
                lines.push(format!("FAIL  {k}: {e}"));
            }
        }
    }
    (ok, lines)
}

fn main() -> ExitCode {
    let Ok(bin) = std::env::var("VERUS_STRIP") else {
        eprintln!("set VERUS_STRIP to a verus-strip binary (rules_verus tools/verus-strip)");
        return ExitCode::from(2);
    };
    let bin = PathBuf::from(bin);
    let listed: Vec<String> = match fs::read_to_string(LIST) {
        Ok(t) => t
            .lines()
            .map(|l| l.split('#').next().unwrap_or("").trim().to_string())
            .filter(|l| !l.is_empty())
            .collect(),
        Err(e) => {
            eprintln!("cannot read {LIST}: {e}");
            return ExitCode::from(2);
        }
    };
    let tmp = std::env::temp_dir().join(format!("cem-{}", std::process::id()));
    if let Err(e) = fs::create_dir_all(&tmp) {
        eprintln!("cannot create {}: {e}", tmp.display());
        return ExitCode::from(2);
    }
    let mut results = Vec::new();
    for k in &listed {
        let r = compare(&bin, k, &tmp);
        if let Err(e) = &r {
            if e.starts_with("cannot run") {
                eprintln!("{e}");
                return ExitCode::from(2);
            }
        }
        results.push((k.clone(), r));
    }
    let (ok, lines) = verdict(&listed, &results);
    for l in lines {
        println!("{l}");
    }
    // Progress report on the engines not yet held (informational).
    let mut pending = Vec::new();
    if let Ok(rd) = fs::read_dir("crates") {
        let mut names: Vec<_> = rd.filter_map(|e| e.ok()).map(|e| e.file_name().to_string_lossy().into_owned()).collect();
        names.sort();
        for n in names {
            if listed.contains(&n) || !Path::new(&format!("crates/{n}/plain/src/engine.rs")).is_file() || !Path::new(&format!("crates/{n}/src/engine.rs")).is_file() {
                continue;
            }
            match residual_lines(&bin, &n, &tmp) {
                Some(r) => pending.push(format!("{n} ({r} lines differ by position)")),
                None => pending.push(format!("{n} (not comparable)")),
            }
        }
    }
    if !pending.is_empty() {
        println!("not yet held: {}", pending.join(", "));
    }
    let _ = fs::remove_dir_all(&tmp);
    if ok {
        println!("engine mirrors: OK ({} held to their Verus source)", listed.len());
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_becomes_an_index_write() {
        let s = "        self.entries.set(idx, Sub { a: f(1, 2), b });\n";
        assert_eq!(set_to_index(s), "        self.entries[idx] = Sub { a: f(1, 2), b };\n");
        assert_eq!(set_to_index("p.data.set(o + i, s[k].data[j + i]);"), "p.data[o + i] = s[k].data[j + i];");
        assert_eq!(set_to_index("a[i].v.set(0, 1);"), "a[i].v[0] = 1;");
    }

    #[test]
    fn set_with_the_receiver_on_the_previous_line() {
        // relay-to as the printer lays it out.
        let s = "        self.entries\n            .set(\n                idx,\n                Sub { a, b },\n            );\n";
        assert_eq!(squash(&set_to_index(s)), "self.entries[idx] = Sub { a, b };");
    }

    #[test]
    fn empty_inherent_impls_are_dropped_and_nothing_else() {
        assert_eq!(drop_empty_impls("struct C {}\nimpl C {}\nfn f() {}\n"), "struct C {}\nfn f() {}\n");
        // A trait impl, or an impl with a body, says something; keep it.
        for keep in ["impl Default for C {}\n", "impl C {\n    fn f() {}\n}\n"] {
            assert_eq!(drop_empty_impls(keep), keep);
        }
    }

    #[test]
    fn a_method_in_a_mirror_impl_is_not_normalised_away() {
        // The planted bad input for the new class: the mirror grows a method
        // in an impl the source leaves empty after the strip.
        let src = "impl C {}\nfn f() {}\n";
        let bad = "impl C {\n    fn g() {}\n}\nfn f() {}\n";
        assert_ne!(squash(&normalise(src)), squash(&normalise(bad)));
    }

    #[test]
    fn mechanical_classes_normalise_away() {
        let src = "//! verus source\n#[derive(Clone, Copy)]\npub struct T {\n    pub x: u32,\n}\nfn f(t: &mut [u32; 4]) {\n    t.set(0, 1);\n}\n";
        let mirror = "//! plain mirror\n#[derive(Clone, Copy, Debug)]\npub struct T {\n    x: u32,\n}\nfn f(t: &mut [u32; 4]) {\n    t[0] = 1;\n}\n#[cfg(test)]\nmod tests {\n    #[test]\n    fn t() {}\n}\n";
        assert_eq!(squash(&normalise(src)), squash(&normalise(mirror)));
    }

    #[test]
    fn a_changed_comparison_is_not_normalised_away() {
        // The planted bad input: the mirror flips `<` to `<=`.
        let src = "fn full(n: u32) -> bool {\n    n < 8\n}\n";
        let bad = "fn full(n: u32) -> bool {\n    n <= 8\n}\n";
        assert_ne!(squash(&normalise(src)), squash(&normalise(bad)));
    }

    #[test]
    fn a_dropped_guard_is_not_normalised_away() {
        let src = "fn g(i: usize) {\n    if i < N {\n        a.set(i, 0);\n    }\n}\n";
        let bad = "fn g(i: usize) {\n    a[i] = 0;\n}\n";
        assert_ne!(squash(&normalise(src)), squash(&normalise(bad)));
    }

    #[test]
    fn pub_is_kept_outside_struct_fields() {
        assert_eq!(drop_field_pub("pub fn f() {}\n    pub fn g(&self) {}\n"), "pub fn f() {}\n    pub fn g(&self) {}\n");
    }

    #[test]
    fn verdict_fails_on_drift_on_error_and_on_an_empty_list() {
        let l = vec!["relay-ci".to_string()];
        assert!(verdict(&l, &[("relay-ci".into(), Ok(None))]).0);
        assert!(!verdict(&l, &[("relay-ci".into(), Ok(Some("d".into())))]).0);
        assert!(!verdict(&l, &[("relay-ci".into(), Err("missing".into()))]).0);
        assert!(!verdict(&[], &[]).0);
    }
}
