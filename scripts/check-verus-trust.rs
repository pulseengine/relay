#!/usr/bin/env rust-script
//! Ceiling on what Verus TRUSTS instead of proving.
//!
//! WHY THIS EXISTS. A `#[verifier::external_body]` function is not verified:
//! Verus takes its body on trust and assumes its `ensures`. The 2026-11-13
//! codebase review counted 69 of them across 17 crates, including every
//! relay-mavlink codec function, and found claims such as "all 4 properties
//! proven" resting on them. The maintainer decided the same day: reword those
//! claims now, prove the bodies under relay#253, and put a per-crate ceiling in
//! CI so the trusted count cannot grow unnoticed.
//!
//! WHAT IT CHECKS. For every crate, the number of lines in `crates/<c>/src/`
//! (recursively, `.rs`) that are a `#[verifier::external_body]` attribute
//! (leading whitespace allowed; comments and doc text do not count) must EQUAL
//! the crate's entry in `scripts/verus-trust-ceiling.txt`:
//!   - more than the ledger  => FAIL: new trusted code; prove it or raise the
//!     ledger in a reviewed change that says why;
//!   - fewer than the ledger => FAIL: lower the ledger to the new count, so the
//!     ceiling ratchets down and the file stays the true number;
//!   - a crate with trusted code and no ledger entry => FAIL.
//! Exit 0 only when every count matches.
//!
//! Run:  rustc --edition 2021 -O scripts/check-verus-trust.rs -o /tmp/cvt && /tmp/cvt
//!       (or `rust-script scripts/check-verus-trust.rs`). Tests: `rustc --test`.

use std::collections::BTreeMap;
use std::fs;
use std::path::Path;
use std::process::ExitCode;

const LEDGER: &str = "scripts/verus-trust-ceiling.txt";
const ATTR: &str = "#[verifier::external_body]";

/// Count attribute lines in one source text.
fn count_in(text: &str) -> usize {
    text.lines().filter(|l| l.trim_start().starts_with(ATTR)).count()
}

fn count_dir(dir: &Path) -> usize {
    let mut n = 0;
    if let Ok(rd) = fs::read_dir(dir) {
        for e in rd.flatten() {
            let p = e.path();
            if p.is_dir() {
                n += count_dir(&p);
            } else if p.extension().is_some_and(|x| x == "rs") {
                n += count_in(&fs::read_to_string(&p).unwrap_or_default());
            }
        }
    }
    n
}

/// Parse `crate count` lines; `#` comments and blank lines ignored.
fn parse_ledger(text: &str) -> Result<BTreeMap<String, usize>, String> {
    let mut m = BTreeMap::new();
    for (i, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut it = line.split_whitespace();
        let (Some(name), Some(n), None) = (it.next(), it.next(), it.next()) else {
            return Err(format!("{LEDGER}:{}: expected `<crate> <count>`", i + 1));
        };
        let n: usize = n
            .parse()
            .map_err(|_| format!("{LEDGER}:{}: `{n}` is not a count", i + 1))?;
        if m.insert(name.to_string(), n).is_some() {
            return Err(format!("{LEDGER}:{}: duplicate entry for {name}", i + 1));
        }
    }
    Ok(m)
}

/// Compare observed counts with the ledger; returns one message per mismatch.
fn compare(observed: &BTreeMap<String, usize>, ledger: &BTreeMap<String, usize>) -> Vec<String> {
    let mut errs = Vec::new();
    for (c, &n) in observed {
        match ledger.get(c) {
            None if n > 0 => errs.push(format!(
                "{c}: {n} external_body, no ledger entry; prove them or add `{c} {n}` with a reason"
            )),
            Some(&cap) if n > cap => errs.push(format!(
                "{c}: {n} external_body > ceiling {cap}; trusted code grew"
            )),
            Some(&cap) if n < cap => errs.push(format!(
                "{c}: {n} external_body < ceiling {cap}; lower the ledger to {n}"
            )),
            _ => {}
        }
    }
    for (c, &cap) in ledger {
        if !observed.contains_key(c) && cap > 0 {
            errs.push(format!("{c}: ledger says {cap} but the crate has no src/; remove the entry"));
        }
    }
    errs
}

fn main() -> ExitCode {
    let ledger = match fs::read_to_string(LEDGER).map_err(|e| format!("{LEDGER}: {e}")).and_then(|t| parse_ledger(&t)) {
        Ok(l) => l,
        Err(e) => {
            eprintln!("verus trust ceiling: FAIL\n  {e}");
            return ExitCode::FAILURE;
        }
    };
    let mut observed = BTreeMap::new();
    let Ok(rd) = fs::read_dir("crates") else {
        eprintln!("verus trust ceiling: FAIL\n  no crates/ directory (run from the repo root)");
        return ExitCode::FAILURE;
    };
    for e in rd.flatten() {
        let src = e.path().join("src");
        if src.is_dir() {
            let name = e.file_name().to_string_lossy().into_owned();
            observed.insert(name, count_dir(&src));
        }
    }
    let total: usize = observed.values().sum();
    let crates = observed.values().filter(|&&n| n > 0).count();
    let errs = compare(&observed, &ledger);
    if errs.is_empty() {
        println!("verus trust ceiling: OK ({total} external_body in {crates} crates, all at ceiling)");
        ExitCode::SUCCESS
    } else {
        eprintln!("verus trust ceiling: FAIL");
        for e in errs {
            eprintln!("  {e}");
        }
        ExitCode::FAILURE
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn m(v: &[(&str, usize)]) -> BTreeMap<String, usize> {
        v.iter().map(|(k, n)| (k.to_string(), *n)).collect()
    }

    #[test]
    fn counts_attribute_lines_only() {
        let src = "#[verifier::external_body]\nfn a() {}\n    #[verifier::external_body]\n// #[verifier::external_body] in a comment\n/// mentions #[verifier::external_body]\n";
        assert_eq!(count_in(src), 2);
    }

    #[test]
    fn equal_counts_pass() {
        assert!(compare(&m(&[("a", 3), ("b", 0)]), &m(&[("a", 3)])).is_empty());
    }

    #[test]
    fn growth_fails() {
        let e = compare(&m(&[("a", 4)]), &m(&[("a", 3)]));
        assert!(e.len() == 1 && e[0].contains("grew"), "{e:?}");
    }

    #[test]
    fn shrink_fails_until_the_ledger_is_lowered() {
        let e = compare(&m(&[("a", 2)]), &m(&[("a", 3)]));
        assert!(e.len() == 1 && e[0].contains("lower the ledger to 2"), "{e:?}");
    }

    #[test]
    fn unlisted_crate_with_trust_fails() {
        let e = compare(&m(&[("new", 1)]), &m(&[]));
        assert!(e.len() == 1 && e[0].contains("no ledger entry"), "{e:?}");
    }

    #[test]
    fn stale_ledger_entry_fails() {
        let e = compare(&m(&[]), &m(&[("gone", 2)]));
        assert!(e.len() == 1 && e[0].contains("remove the entry"), "{e:?}");
    }

    #[test]
    fn ledger_parse_rejects_garbage_and_duplicates() {
        assert!(parse_ledger("a 1\n# c\n\nb 2 # note\n").is_ok());
        assert!(parse_ledger("a x\n").is_err());
        assert!(parse_ledger("a 1 2\n").is_err());
        assert!(parse_ledger("a 1\na 2\n").is_err());
    }
}
