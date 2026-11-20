#!/usr/bin/env rust-script
//! Trust ledgers: what the proofs TRUST or ASSUME instead of proving.
//!
//! WHY THIS EXISTS. A `#[verifier::external_body]` function is not verified:
//! Verus takes its body on trust and assumes its `ensures`. The 2026-11-13
//! codebase review counted 69 of them across 17 crates, including every
//! relay-mavlink codec function, and found claims such as "all 4 properties
//! proven" resting on them. The maintainer decided the same day: reword those
//! claims now, prove the bodies under relay#253, and put a per-crate ceiling in
//! CI so the trusted count cannot grow unnoticed.
//!
//! A `kani::assume` narrows a harness's input range: a bug outside that range
//! is invisible to the proof. North Star O3 counts these in the same ledger
//! and its guard says entries may only shrink (SWREQ-RELAY-LEDGER-P01). 125 of
//! them in 32 files were unledgered on 2026-11-20.
//!
//! WHAT IT CHECKS, per ledger:
//!   1. EXACT. Each key's observed count must EQUAL its ledger entry:
//!      - external_body: lines in `crates/<c>/src/` (recursive `.rs`) that are a
//!        `#[verifier::external_body]` attribute, keyed by crate, in
//!        `scripts/verus-trust-ceiling.txt`;
//!      - kani::assume: occurrences of `kani::assume(` on non-comment lines of
//!        every `.rs` file in the tree (build outputs skipped), keyed by
//!        repo-relative path, in `scripts/kani-assume-ledger.txt`.
//!      More than the ledger, an unlisted key, fewer than the ledger, or a stale
//!      entry each FAIL, so the ledger stays the true number.
//!   2. SHRINK-ONLY. Each ledger's TOTAL may not exceed the same ledger's total
//!      on the base commit (`--base DIR`, a directory holding the base's copies
//!      at the same relative paths). A rename moves an entry and keeps the
//!      total; a new assumption raises it and fails, even with a ledger entry.
//!      A ledger the base does not have yet is reported as introduced.
//!
//! The base comparison is never skipped silently: the caller passes either
//! `--base DIR` or `--no-base REASON`, and the reason is printed.
//!
//! NOT COVERED, named so the count is not read as complete: `#[kani::unwind]`
//! loop bounds, `kani::stub` replacements, Lean `native_decide` (trusts the
//! compiler through the `Lean.ofReduceBool` axiom), and the Rocq and Lean kernel
//! axioms, which need a semantic audit (Print Assumptions / #print axioms), not
//! a text count.
//!
//! Run:  rustc --edition 2021 -O scripts/check-verus-trust.rs -o /tmp/cvt
//!       /tmp/cvt --base <dir-with-base-ledgers> | --no-base "<why>"
//! Tests: rustc --edition 2021 --test scripts/check-verus-trust.rs

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

const EXT_LEDGER: &str = "scripts/verus-trust-ceiling.txt";
const KANI_LEDGER: &str = "scripts/kani-assume-ledger.txt";
const ATTR: &str = "#[verifier::external_body]";
/// Built from two halves so this file carries no occurrence of its own; the
/// tests write `KA(` and substitute it.
const ASSUME: &str = concat!("kani", "::assume(");

/// Directories never scanned: build outputs and vendored trees.
const SKIP_DIRS: &[&str] = &["target", ".git", "node_modules", ".lake", "dist"];

/// Count external_body attribute lines in one source text.
fn count_in(text: &str) -> usize {
    text.lines()
        .filter(|l| l.trim_start().starts_with(ATTR))
        .count()
}

/// Count `kani::assume(` occurrences on lines that are not `//` comments.
fn count_assume(text: &str) -> usize {
    text.lines()
        .filter(|l| !l.trim_start().starts_with("//"))
        .map(|l| l.matches(ASSUME).count())
        .sum()
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

fn skipped(name: &str) -> bool {
    SKIP_DIRS.contains(&name) || name.starts_with("bazel-")
}

/// Every `.rs` file under `root` with at least one assume, keyed by its path
/// relative to `root` with `/` separators.
fn scan_assumes(root: &Path, dir: &Path, out: &mut BTreeMap<String, usize>) {
    let Ok(rd) = fs::read_dir(dir) else { return };
    for e in rd.flatten() {
        let p = e.path();
        let name = e.file_name().to_string_lossy().into_owned();
        if p.is_dir() {
            if !skipped(&name) {
                scan_assumes(root, &p, out);
            }
        } else if p.extension().is_some_and(|x| x == "rs") {
            let n = count_assume(&fs::read_to_string(&p).unwrap_or_default());
            if n > 0 {
                let rel = p.strip_prefix(root).unwrap_or(&p);
                let key = rel
                    .components()
                    .map(|c| c.as_os_str().to_string_lossy())
                    .collect::<Vec<_>>()
                    .join("/");
                out.insert(key, n);
            }
        }
    }
}

/// Parse `key count` lines; `#` comments and blank lines ignored.
fn parse_ledger(path: &str, text: &str) -> Result<BTreeMap<String, usize>, String> {
    let mut m = BTreeMap::new();
    for (i, raw) in text.lines().enumerate() {
        let line = raw.split('#').next().unwrap_or("").trim();
        if line.is_empty() {
            continue;
        }
        let mut it = line.split_whitespace();
        let (Some(name), Some(n), None) = (it.next(), it.next(), it.next()) else {
            return Err(format!("{path}:{}: expected `<key> <count>`", i + 1));
        };
        let n: usize = n
            .parse()
            .map_err(|_| format!("{path}:{}: `{n}` is not a count", i + 1))?;
        if m.insert(name.to_string(), n).is_some() {
            return Err(format!("{path}:{}: duplicate entry for {name}", i + 1));
        }
    }
    Ok(m)
}

/// Compare observed counts with the ledger; one message per mismatch. `what`
/// names the counted form, `gone` says why a stale key has no count.
fn compare(
    what: &str,
    gone: &str,
    observed: &BTreeMap<String, usize>,
    ledger: &BTreeMap<String, usize>,
) -> Vec<String> {
    let mut errs = Vec::new();
    for (c, &n) in observed {
        match ledger.get(c) {
            None if n > 0 => errs.push(format!(
                "{c}: {n} {what}, no ledger entry; prove or remove them, or ledger `{c} {n}` (the total may not grow)"
            )),
            Some(&cap) if n > cap => errs.push(format!(
                "{c}: {n} {what} > ledger {cap}; trusted or assumed code grew"
            )),
            Some(&cap) if n < cap => errs.push(format!(
                "{c}: {n} {what} < ledger {cap}; lower the ledger to {n}"
            )),
            _ => {}
        }
    }
    for (c, &cap) in ledger {
        if !observed.contains_key(c) && cap > 0 {
            errs.push(format!("{c}: ledger says {cap} but {gone}; remove the entry"));
        }
    }
    errs
}

/// The shrink-only rule: `Ok(note)` or `Err(message)`.
fn shrink_only(
    path: &str,
    current: &BTreeMap<String, usize>,
    base: Option<&BTreeMap<String, usize>>,
) -> Result<String, String> {
    let now: usize = current.values().sum();
    match base {
        None => Ok(format!("{path}: introduced by this change ({now})")),
        Some(b) => {
            let was: usize = b.values().sum();
            if now > was {
                Err(format!(
                    "{path}: total {now} > base total {was}; ledger entries may only shrink (North Star O3)"
                ))
            } else {
                Ok(format!("{path}: total {now} <= base {was}"))
            }
        }
    }
}

enum Base {
    Dir(PathBuf),
    None(String),
}

fn parse_args(args: &[String]) -> Result<Base, String> {
    match args {
        [flag, dir] if flag == "--base" => Ok(Base::Dir(PathBuf::from(dir))),
        [flag, why] if flag == "--no-base" && !why.trim().is_empty() => Ok(Base::None(why.clone())),
        _ => Err("usage: check-verus-trust --base <dir> | --no-base \"<reason>\"".into()),
    }
}

fn read_ledger(path: &str) -> Result<BTreeMap<String, usize>, String> {
    fs::read_to_string(path)
        .map_err(|e| format!("{path}: {e}"))
        .and_then(|t| parse_ledger(path, &t))
}

/// The base copy of a ledger: `Ok(None)` when the base does not have it.
fn read_base(dir: &Path, rel: &str) -> Result<Option<BTreeMap<String, usize>>, String> {
    let p = dir.join(rel);
    if !p.exists() {
        return Ok(None);
    }
    let shown = p.display().to_string();
    let t = fs::read_to_string(&p).map_err(|e| format!("{shown}: {e}"))?;
    parse_ledger(&shown, &t).map(Some)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    if args.iter().any(|a| a == "--help" || a == "-h") {
        println!("check-verus-trust --base <dir> | --no-base \"<reason>\"\n\nExternal_body per crate and kani::assume per file must equal their ledgers;\neach ledger's total may not exceed the base commit's.");
        return ExitCode::SUCCESS;
    }
    let base = match parse_args(&args) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("{e}");
            return ExitCode::from(2);
        }
    };

    let mut errs = Vec::new();
    let mut notes = Vec::new();

    // external_body, per crate.
    let ext_ledger = read_ledger(EXT_LEDGER);
    let mut ext_obs = BTreeMap::new();
    match fs::read_dir("crates") {
        Ok(rd) => {
            for e in rd.flatten() {
                let src = e.path().join("src");
                if src.is_dir() {
                    let name = e.file_name().to_string_lossy().into_owned();
                    ext_obs.insert(name, count_dir(&src));
                }
            }
        }
        Err(_) => errs.push("no crates/ directory (run from the repo root)".to_string()),
    }

    // kani::assume, per file.
    let kani_ledger = read_ledger(KANI_LEDGER);
    let mut kani_obs = BTreeMap::new();
    scan_assumes(Path::new("."), Path::new("."), &mut kani_obs);

    for (path, what, gone, ledger, obs) in [
        (EXT_LEDGER, "external_body", "the crate has no src/", &ext_ledger, &ext_obs),
        (KANI_LEDGER, "kani::assume", "the file has none", &kani_ledger, &kani_obs),
    ] {
        match ledger {
            Err(e) => errs.push(e.clone()),
            Ok(l) => {
                errs.extend(compare(what, gone, obs, l));
                match &base {
                    Base::None(why) => notes.push(format!("{path}: shrink-only NOT checked: {why}")),
                    Base::Dir(d) => match read_base(d, path) {
                        Err(e) => errs.push(format!("base ledger unreadable: {e}")),
                        Ok(b) => match shrink_only(path, l, b.as_ref()) {
                            Ok(n) => notes.push(n),
                            Err(e) => errs.push(e),
                        },
                    },
                }
            }
        }
    }

    let ext_total: usize = ext_obs.values().sum();
    let ext_crates = ext_obs.values().filter(|&&n| n > 0).count();
    let kani_total: usize = kani_obs.values().sum();
    let summary = format!(
        "{ext_total} external_body in {ext_crates} crates, {kani_total} kani::assume in {} files",
        kani_obs.len()
    );
    for n in &notes {
        println!("  {n}");
    }
    if errs.is_empty() {
        println!("trust ledgers: OK ({summary}, all at ledger)");
        ExitCode::SUCCESS
    } else {
        eprintln!("trust ledgers: FAIL ({summary})");
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

    /// Test sources write `KA(`; this turns it into the real needle.
    fn ka(src: &str) -> String {
        src.replace("KA(", ASSUME)
    }

    fn cmp(o: &[(&str, usize)], l: &[(&str, usize)]) -> Vec<String> {
        compare("external_body", "the crate has no src/", &m(o), &m(l))
    }

    #[test]
    fn counts_attribute_lines_only() {
        let src = "#[verifier::external_body]\nfn a() {}\n    #[verifier::external_body]\n// #[verifier::external_body] in a comment\n/// mentions #[verifier::external_body]\n";
        assert_eq!(count_in(src), 2);
    }

    #[test]
    fn counts_assumes_outside_comments_including_two_on_a_line() {
        let src = "    KA(x < 4);\n// KA(y) commented out\n/// doc: KA(z)\nKA(a); KA(b); // trailing KA(c)\nlet k = kani::any();\n";
        assert_eq!(count_assume(&ka(src)), 4);
    }

    #[test]
    fn scan_keys_by_relative_path_and_skips_build_outputs() {
        let root = std::env::temp_dir().join(format!("cvt-scan-{}", std::process::id()));
        let _ = fs::remove_dir_all(&root);
        fs::create_dir_all(root.join("crates/a/src")).unwrap();
        fs::create_dir_all(root.join("target/debug")).unwrap();
        fs::create_dir_all(root.join("bazel-out")).unwrap();
        fs::write(root.join("crates/a/src/k.rs"), ka("KA(x);\nKA(y);\n")).unwrap();
        fs::write(root.join("crates/a/src/none.rs"), "fn f() {}\n").unwrap();
        fs::write(root.join("target/debug/gen.rs"), ka("KA(x);\n")).unwrap();
        fs::write(root.join("bazel-out/gen.rs"), ka("KA(x);\n")).unwrap();
        let mut out = BTreeMap::new();
        scan_assumes(&root, &root, &mut out);
        let _ = fs::remove_dir_all(&root);
        assert_eq!(out, m(&[("crates/a/src/k.rs", 2)]));
    }

    #[test]
    fn equal_counts_pass() {
        assert!(cmp(&[("a", 3), ("b", 0)], &[("a", 3)]).is_empty());
    }

    #[test]
    fn growth_fails() {
        let e = cmp(&[("a", 4)], &[("a", 3)]);
        assert!(e.len() == 1 && e[0].contains("grew"), "{e:?}");
    }

    #[test]
    fn shrink_fails_until_the_ledger_is_lowered() {
        let e = cmp(&[("a", 2)], &[("a", 3)]);
        assert!(e.len() == 1 && e[0].contains("lower the ledger to 2"), "{e:?}");
    }

    #[test]
    fn unlisted_key_with_trust_fails() {
        let e = cmp(&[("new", 1)], &[]);
        assert!(e.len() == 1 && e[0].contains("no ledger entry"), "{e:?}");
    }

    #[test]
    fn stale_ledger_entry_fails() {
        let e = cmp(&[], &[("gone", 2)]);
        assert!(e.len() == 1 && e[0].contains("remove the entry"), "{e:?}");
    }

    #[test]
    fn ledger_parse_rejects_garbage_and_duplicates() {
        assert!(parse_ledger("t", "a 1\n# c\n\nb 2 # note\n").is_ok());
        assert!(parse_ledger("t", "a x\n").is_err());
        assert!(parse_ledger("t", "a 1 2\n").is_err());
        assert!(parse_ledger("t", "a 1\na 2\n").is_err());
    }

    #[test]
    fn a_new_ledger_entry_that_raises_the_total_fails_shrink_only() {
        let base = m(&[("x.rs", 3)]);
        let e = shrink_only("L", &m(&[("x.rs", 3), ("y.rs", 1)]), Some(&base)).unwrap_err();
        assert!(e.contains("total 4 > base total 3"), "{e}");
    }

    #[test]
    fn a_rename_or_a_removal_passes_shrink_only() {
        let base = m(&[("old.rs", 3)]);
        assert!(shrink_only("L", &m(&[("new.rs", 3)]), Some(&base)).is_ok());
        assert!(shrink_only("L", &m(&[("old.rs", 2)]), Some(&base)).is_ok());
    }

    #[test]
    fn a_ledger_the_base_lacks_is_introduced() {
        let n = shrink_only("L", &m(&[("x.rs", 5)]), None).unwrap();
        assert!(n.contains("introduced"), "{n}");
    }

    #[test]
    fn the_base_comparison_is_never_skipped_silently() {
        let s = |v: &[&str]| v.iter().map(|x| x.to_string()).collect::<Vec<_>>();
        assert!(parse_args(&s(&[])).is_err());
        assert!(parse_args(&s(&["--no-base", "  "])).is_err());
        assert!(parse_args(&s(&["--base"])).is_err());
        assert!(parse_args(&s(&["--frobnicate", "x"])).is_err());
        assert!(matches!(parse_args(&s(&["--base", "d"])), Ok(Base::Dir(_))));
        assert!(matches!(parse_args(&s(&["--no-base", "push"])), Ok(Base::None(_))));
    }

    #[test]
    fn a_base_ledger_that_does_not_parse_is_an_error_not_an_introduction() {
        let d = std::env::temp_dir().join(format!("cvt-base-{}", std::process::id()));
        let _ = fs::remove_dir_all(&d);
        fs::create_dir_all(d.join("scripts")).unwrap();
        fs::write(d.join(KANI_LEDGER), "x.rs notanumber\n").unwrap();
        let r = read_base(&d, KANI_LEDGER);
        let missing = read_base(&d, EXT_LEDGER);
        let _ = fs::remove_dir_all(&d);
        assert!(r.is_err());
        assert!(matches!(missing, Ok(None)));
    }
}
