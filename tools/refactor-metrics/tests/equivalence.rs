//! EQUIVALENCE: the frozen instrument against the baseline run's own outputs.
//!
//! Baseline = the Python method (dataset + report pass) run on relay
//! origin/main @55af1e9, 500 first-parent commits. Its outputs are committed
//! under `baseline/`:
//!   analyze-out-55af1e9.txt          the report (stdout), 24 raw pairs
//!   pairs-55af1e9.json               the report pass's pairs + depth dump
//!   edges-55af1e9.json               every raw blame edge (fix, origin, file, lines)
//!   commits-55af1e9.json             per-commit [sha, date, strict, broad, window, subj2, pr, files]
//!   pairs-55af1e9-no-ignore-rev.json an EARLIER run without --ignore-rev (31 pairs);
//!                                    kept as a cross-check, NOT the baseline.
//!   falcon-tags-55af1e9.txt          the falcon-v* tag set present at the baseline run
//!
//! Needs the relay history (this tool's own repo), so it is `#[ignore]`:
//!   cargo test --release -- --ignored --nocapture
//! Set REFIX_REPO to measure another clone of relay.

use serde_json::Value;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::Command;

const BASE_REF: &str = "55af1e9679015018c564df064bc77457a07f8658";

fn manifest() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

fn repo() -> PathBuf {
    std::env::var_os("REFIX_REPO")
        .map(PathBuf::from)
        .unwrap_or_else(|| manifest().join("../.."))
}

fn tool(args: &[&str], repo: &Path) -> String {
    let out = Command::new(env!("CARGO_BIN_EXE_refactor-metrics"))
        .arg("--repo")
        .arg(repo)
        .args(args)
        .output()
        .expect("run tool");
    assert!(
        out.status.success(),
        "tool failed: {}",
        String::from_utf8_lossy(&out.stderr)
    );
    String::from_utf8(out.stdout).unwrap()
}

fn load(name: &str) -> Value {
    let p = manifest().join("baseline").join(name);
    serde_json::from_str(&std::fs::read_to_string(&p).unwrap()).unwrap()
}

/// Line diff (both directions) of two multisets of canonical strings.
fn multiset_diff(label: &str, want: Vec<String>, got: Vec<String>) -> Vec<String> {
    let mut m: BTreeMap<String, i64> = BTreeMap::new();
    for w in want {
        *m.entry(w).or_default() += 1;
    }
    for g in got {
        *m.entry(g).or_default() -= 1;
    }
    m.into_iter()
        .filter(|(_, v)| *v != 0)
        .map(|(k, v)| {
            if v > 0 {
                format!("{label} - baseline only: {k}")
            } else {
                format!("{label} + rust only:     {k}")
            }
        })
        .collect()
}

fn pairs_canon(pairs: &Value) -> Vec<String> {
    // baseline shape: [[fix, origin, {path: n}], ...]
    pairs
        .as_array()
        .unwrap()
        .iter()
        .flat_map(|p| {
            let (f, o) = (p[0].as_str().unwrap(), p[1].as_str().unwrap());
            p[2].as_object()
                .unwrap()
                .iter()
                .map(move |(path, n)| format!("{f} {o} {path} {n}"))
                .collect::<Vec<_>>()
        })
        .collect()
}

fn rust_pairs_canon(mode: &Value) -> Vec<String> {
    mode["pairs"]
        .as_array()
        .unwrap()
        .iter()
        .flat_map(|p| {
            let (f, o) = (
                p["fix"].as_str().unwrap().to_string(),
                p["origin"].as_str().unwrap().to_string(),
            );
            p["files"]
                .as_object()
                .unwrap()
                .iter()
                .map(move |(path, n)| format!("{f} {o} {path} {n}"))
                .collect::<Vec<_>>()
        })
        .collect()
}

fn depth_canon(d: &Value) -> Vec<String> {
    d.as_object()
        .unwrap()
        .iter()
        .map(|(k, v)| format!("{k} {v}"))
        .collect()
}

/// Windows are read from the repo's tag set at run time (a reproduced baseline
/// quirk), so the per-commit `window` field and the bucket rows are only
/// comparable when the tag set equals the baseline run's. A later falcon tag
/// reassigns the baseline's `unreleased` commits BY DESIGN; that makes the
/// equivalence not evaluable here, it is not a method defect to "fix".
fn require_baseline_tag_set(repo: &Path) {
    let out = Command::new("git")
        .arg("-C")
        .arg(repo)
        .args(["tag", "-l", "falcon-v*", "--sort=v:refname"])
        .output()
        .expect("git tag");
    let got = String::from_utf8_lossy(&out.stdout).into_owned();
    let want =
        std::fs::read_to_string(manifest().join("baseline/falcon-tags-55af1e9.txt")).unwrap();
    if got != want {
        panic!(
            "tag set differs from baseline (baseline/falcon-tags-55af1e9.txt); windows differ by \
             design (README: tag set read at run time), equivalence not evaluable in this clone"
        );
    }
}

#[test]
#[ignore = "needs relay history at 55af1e9; run with --ignored"]
fn reproduces_the_55af1e9_baseline_exactly() {
    let repo = repo();
    require_baseline_tag_set(&repo);
    let mut diffs = Vec::new();

    // (1) the report, byte for byte
    let text = tool(
        &[
            "--ref",
            BASE_REF,
            "--merges",
            "500",
            "--format",
            "baseline-text",
        ],
        &repo,
    );
    let want =
        std::fs::read_to_string(manifest().join("baseline/analyze-out-55af1e9.txt")).unwrap();
    if text != want {
        diffs.extend(multiset_diff(
            "report",
            want.lines().map(str::to_string).collect(),
            text.lines().map(str::to_string).collect(),
        ));
        if diffs.is_empty() {
            diffs.push("report: same lines, different order/whitespace".into());
        }
    }

    // (2) machine output
    let js: Value = serde_json::from_str(&tool(
        &[
            "--ref",
            BASE_REF,
            "--merges",
            "500",
            "--format",
            "json",
            "--dataset",
        ],
        &repo,
    ))
    .unwrap();
    assert_eq!(js["universe_commits"], 500);
    assert_eq!(js["scope"]["merges"], 500);
    let cur = &js["modes"]["curated"];

    // (2a) per-commit classification / windows / numstat
    let base_commits = load("commits-55af1e9.json");
    let want: Vec<String> = base_commits
        .as_array()
        .unwrap()
        .iter()
        .map(|r| r.to_string())
        .collect();
    let got: Vec<String> = js["dataset"]["commits"]
        .as_array()
        .unwrap()
        .iter()
        .map(|c| {
            serde_json::json!([
                c["sha"],
                c["date"],
                c["strict"],
                c["broad"],
                c["window"],
                c["subj2"],
                c["pr"],
                c["files"]
            ])
            .to_string()
        })
        .collect();
    diffs.extend(multiset_diff("commit", want, got));

    // (2b) every raw blame edge, as a multiset
    let canon = |v: &Value| -> Vec<String> {
        v.as_array()
            .unwrap()
            .iter()
            .map(|e| e.to_string())
            .collect()
    };
    diffs.extend(multiset_diff(
        "edge",
        canon(&load("edges-55af1e9.json")),
        canon(&js["dataset"]["edges"]),
    ));

    // (2c) pairs (fix, origin, file, lines) and depth
    let base = load("pairs-55af1e9.json");
    diffs.extend(multiset_diff(
        "pair",
        pairs_canon(&base["pairs"]),
        rust_pairs_canon(cur),
    ));
    diffs.extend(multiset_diff(
        "depth",
        depth_canon(&base["depth"]),
        depth_canon(&cur["depth_universe"]),
    ));

    // (2d) headline counts of the baseline report
    assert_eq!(cur["fixes"], 78);
    assert_eq!(cur["raw_pairs"], 24);
    assert_eq!(cur["refixing_commits"], 19);
    assert_eq!(
        js["git_failures"].as_array().unwrap().len(),
        0,
        "git failures: {}",
        js["git_failures"]
    );

    if !diffs.is_empty() {
        panic!(
            "EQUIVALENCE DIFF ({} lines):\n{}",
            diffs.len(),
            diffs.join("\n")
        );
    }
    eprintln!("equivalence: report, 500 commits, edges, 24 pairs, depth: diff empty");
}

#[test]
#[ignore = "needs relay history at 55af1e9; run with --ignored"]
fn without_ignore_rev_reproduces_the_earlier_31_pair_run() {
    // Cross-check: the earlier run built WITHOUT --ignore-rev. Its report pass
    // still hard-coded the 4c5feda fixer exclusion, which an empty ignore list
    // drops -- so the comparison is restricted to pairs whose fixing side is
    // not 4c5feda.
    let repo = repo();
    let dir = std::env::temp_dir().join(format!("refix-noign-{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    let empty = dir.join("empty.txt");
    std::fs::write(&empty, "# none\n").unwrap();
    let js: Value = serde_json::from_str(&tool(
        &[
            "--ref",
            BASE_REF,
            "--format",
            "json",
            "--ignore-revs",
            empty.to_str().unwrap(),
        ],
        &repo,
    ))
    .unwrap();
    let _ = std::fs::remove_dir_all(&dir);
    let base = load("pairs-55af1e9-no-ignore-rev.json");
    let not_fmt = |s: &String| !s.starts_with("4c5feda");
    let want: Vec<String> = pairs_canon(&base["pairs"])
        .into_iter()
        .filter(not_fmt)
        .collect();
    let got: Vec<String> = rust_pairs_canon(&js["modes"]["curated"])
        .into_iter()
        .filter(not_fmt)
        .collect();
    let d = multiset_diff("pair(no-ignore-rev)", want, got);
    assert!(d.is_empty(), "{}", d.join("\n"));
}
