#!/usr/bin/env rust-script
//! Does every `verified` artifact rest on evidence that exists and runs?
//!
//! WHY THIS EXISTS (#415, SWREQ-RELAY-EVIDENCE-P01). A census of the
//! verification artifacts found `verified` statuses whose cited evidence cannot
//! execute: a "Gazebo" run that builds the stub backend (no `--features
//! gazebo`), a `cd` that cannot reach the next step (each step is its own
//! subprocess), MC/DC claims whose steps are `$WITNESS` and `/path/to/...`
//! placeholders. A `verified` status on evidence that cannot run is the
//! traceability form of a gate that cannot fail.
//!
//! ONE SOURCE OF TRUTH FOR "DOES THIS STEP RUN". Steps are not re-classified
//! here: this tool reads the verification gate's own dry-run labels
//! (scripts/run-falcon-verification.py --dry-run over ALL artifacts). A step
//! counts as executed evidence if the gate runs it (`dry-run`), the required
//! Kani gate does (`enforced-by-kani-gate`), or it is a `bazel test` of a
//! //proofs/lean, //proofs/rocq or //proofs/gappa target — the gate labels those
//! bench-only, but lean.yml and rocq.yml run `//proofs/lean:all` and
//! `//proofs/rocq:all //proofs/gappa:all` on every main push, and both were
//! green on 9eacd4a and 916fbca (measured 2026-09-17). `enforced-by-verus-gate`
//! is NOT counted: verus.yml failed on those same commits and has verified
//! nothing since 2026-09-11 (#405), so today that label is a claim, not
//! evidence. Artifacts resting on it alone are reported apart.
//!
//! Findings on `verified`/`accepted` artifacts fail (exit 1); findings on other
//! statuses are reported but do not fail. Exit 2 = could not evaluate.
//!
//! Usage:
//!   scripts/check-evidence.rs                      # runs the gate dry-run itself
//!   scripts/check-evidence.rs --dry-run-file out.txt
//!   rust-script --test scripts/check-evidence.rs
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_yaml = "0.9"
//! toml = "0.8"
//! regex = "1"
//! walkdir = "2"
//! ```

use anyhow::{bail, Context, Result};
use regex::Regex;
use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

#[derive(Debug, Clone)]
struct Step {
    label: String,
    cmd: String,
}

/// What a step citation is checked against.
struct Tree {
    root: PathBuf,
    /// package name -> crate directory
    packages: BTreeMap<String, PathBuf>,
}

impl Tree {
    fn exists(&self, rel: &str) -> bool {
        self.root.join(rel).exists()
    }

    /// Does any `fn` under the crate's directory contain `needle`?
    fn has_fn_containing(&self, pkg: &str, needle: &str) -> bool {
        let Some(dir) = self.packages.get(pkg) else { return false };
        // cargo's filter matches the test PATH, so a module name matches too.
        let pat = Regex::new(&format!(r"\b(?:fn|mod)\s+\w*{}\w*\s*[<({{]", regex::escape(needle))).expect("escaped regex");
        walkdir::WalkDir::new(dir)
            .into_iter()
            .filter_map(|e| e.ok())
            .filter(|e| e.path().extension().is_some_and(|x| x == "rs"))
            .filter(|e| !e.path().components().any(|c| c.as_os_str() == "target"))
            .any(|e| std::fs::read_to_string(e.path()).map(|t| pat.is_match(&t)).unwrap_or(false))
    }
}

const KNOWN_ENV: &[&str] = &["HOME", "PATH", "PWD", "REPO", "RUNNER_TEMP", "CARGO_TARGET_DIR", "TMPDIR"];

/// `cmd` with single-quoted segments removed. The shell expands nothing inside
/// '...', so `$VAR` and `<name>` there are literal text (a grep pattern, say),
/// not an unset variable or an unfilled placeholder.
fn unquoted(cmd: &str) -> String {
    let mut out = String::with_capacity(cmd.len());
    let mut in_single = false;
    for ch in cmd.chars() {
        if ch == '\'' {
            in_single = !in_single;
            continue;
        }
        if !in_single {
            out.push(ch);
        }
    }
    out
}

/// Every reason one step's citation does not resolve. Pure given `tree`.
fn check_step(cmd: &str, tree: &Tree) -> Vec<String> {
    let mut out = Vec::new();
    let expanded = unquoted(cmd);

    // A step that is only `cd <dir>`: each step is its own subprocess, so it
    // changes nothing for the next one.
    if Regex::new(r"^\s*cd\s+\S+\s*$").unwrap().is_match(cmd) {
        out.push("a lone `cd` — steps run as separate processes, the next step starts at the repo root".into());
    }

    // Placeholders that were never filled in.
    if cmd.contains("/path/to/") {
        out.push("placeholder path `/path/to/…`".into());
    }
    for c in Regex::new(r"\$\{?([A-Z][A-Z0-9_]*)").unwrap().captures_iter(&expanded) {
        let v = &c[1];
        if !KNOWN_ENV.contains(&v) && !v.starts_with("GITHUB_") && std::env::var(v).is_err() {
            out.push(format!("unset variable `${v}`"));
        }
    }
    // `<crate>`-style placeholders, but not generics such as `Option<f32>`.
    if Regex::new(r"(?:^|[\s/=])<[a-z][a-z0-9_-]*>").unwrap().is_match(&expanded) && !expanded.contains("<<") {
        out.push("angle-bracket placeholder `<…>`".into());
    }

    // The gz stub trap: without the feature, falcon-sitl-gz builds a stub
    // backend that reports plausible numbers and never touches Gazebo.
    // An artifact that verifies the stub itself says so explicitly in the step.
    if cmd.contains("--backend=gazebo") && cmd.contains("cargo run") && !cmd.contains("--features gazebo") && !cmd.contains("intentionally the STUB") {
        out.push("`--backend=gazebo` without `--features gazebo` runs the STUB backend".into());
    }

    // A bare `.wasm` filename with no path is not reproducible from the root.
    for c in Regex::new(r"(?:^|\s)([A-Za-z0-9_.-]+\.wasm)\b").unwrap().captures_iter(cmd) {
        out.push(format!("bare `{}` — no path, only exists inside a build tree", &c[1]));
    }

    // Repo paths must exist.
    let path_re = Regex::new(
        r"(?:^|[\s='(])(?:\./)?((?:scripts|crates|examples|tools|wit|proofs|docs|tests|host|wasm|artifacts|spar|\.github)/[^\s'\x22`|;&)>]+)",
    )
    .unwrap();
    for c in path_re.captures_iter(cmd) {
        let p = c[1].trim_end_matches([',', '.', ':']);
        if p.contains('*') || p.contains('$') || p.contains('{') || p.contains("/target/") {
            continue;
        }
        if !tree.exists(p) {
            out.push(format!("path `{p}` does not exist"));
        }
    }

    // cargo -p <pkg> must be a package; a cargo test name filter must match a fn.
    let cargo = Regex::new(r"\bcargo\s+(test|run|build|bench|check|clippy|kani)\b(.*)").unwrap();
    if let Some(c) = cargo.captures(cmd) {
        let sub = c[1].to_string();
        let rest = c[2].split(" -- ").next().unwrap_or("").to_string();
        let toks: Vec<&str> = rest.split_whitespace().collect();
        let mut pkg: Option<&str> = None;
        let mut positional: Option<&str> = None;
        let mut i = 0;
        while i < toks.len() {
            let t = toks[i];
            let takes_value = matches!(
                t,
                "-p" | "--package" | "--features" | "-F" | "--test" | "--bin" | "--example" | "--manifest-path"
                    | "--target" | "--harness" | "--profile" | "-j" | "--jobs" | "--bench"
            );
            if t == "-p" || t == "--package" {
                pkg = toks.get(i + 1).copied();
            }
            if takes_value {
                i += 2;
                continue;
            }
            if !t.starts_with('-') && positional.is_none() && sub == "test" {
                positional = Some(t);
            }
            i += 1;
        }
        if let Some(p) = pkg {
            if !tree.packages.contains_key(p) {
                out.push(format!("cargo package `{p}` does not exist"));
            } else if let Some(f) = positional {
                let needle = f.rsplit("::").find(|s| !s.is_empty()).unwrap_or(f);
                if !tree.has_fn_containing(p, needle) {
                    out.push(format!("`cargo test -p {p} {f}` matches no fn in {p} — a filter that matches nothing passes vacuously"));
                }
            }
        }
    }
    out
}

fn parse_dry_run(text: &str) -> BTreeMap<String, Vec<Step>> {
    let re = Regex::new(r"^\s*\[\s*([A-Za-z-]+)\]\s+(\S+?)(?: step \d+)?:\s(.*)$").unwrap();
    let mut by: BTreeMap<String, Vec<Step>> = BTreeMap::new();
    for line in text.lines() {
        if let Some(c) = re.captures(line) {
            by.entry(c[2].to_string()).or_default().push(Step { label: c[1].to_string(), cmd: c[3].to_string() });
        }
    }
    by
}

fn statuses(root: &Path) -> Result<BTreeMap<String, String>> {
    let mut out = BTreeMap::new();
    for e in walkdir::WalkDir::new(root.join("artifacts")) {
        let e = e.context("walking artifacts/")?;
        if !e.path().extension().is_some_and(|x| x == "yaml") {
            continue;
        }
        let doc: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(e.path())?)
            .with_context(|| format!("parsing {}", e.path().display()))?;
        for a in doc["artifacts"].as_sequence().into_iter().flatten() {
            if let (Some(id), Some(st)) = (a["id"].as_str(), a["status"].as_str()) {
                out.insert(id.to_owned(), st.to_owned());
            }
        }
    }
    Ok(out)
}

fn packages(root: &Path) -> Result<BTreeMap<String, PathBuf>> {
    let mut out = BTreeMap::new();
    for e in walkdir::WalkDir::new(root).into_iter().filter_entry(|e| {
        let n = e.file_name().to_string_lossy();
        n != "target" && n != ".git" && n != "node_modules"
    }) {
        let e = e?;
        if e.file_name() != "Cargo.toml" {
            continue;
        }
        let Ok(t) = toml::from_str::<toml::Value>(&std::fs::read_to_string(e.path())?) else { continue };
        if let Some(name) = t.get("package").and_then(|p| p.get("name")).and_then(|n| n.as_str()) {
            out.insert(name.to_owned(), e.path().parent().unwrap().to_path_buf());
        }
    }
    Ok(out)
}

#[derive(Default)]
struct Report {
    failing: Vec<(String, String, Vec<String>)>,
    other: Vec<(String, String, Vec<String>)>,
    verus_only: Vec<String>,
}

fn evaluate(dry: &BTreeMap<String, Vec<Step>>, st: &BTreeMap<String, String>, tree: &Tree) -> Report {
    let mut r = Report::default();
    for (id, steps) in dry {
        let status = st.get(id).cloned().unwrap_or_else(|| "?".into());
        let terminal = status == "verified" || status == "accepted";
        let mut findings = Vec::new();
        let proof_wf = Regex::new(r"\bbazel\s+test\b.*//proofs/(?:lean|rocq|gappa):").unwrap();
        let executed = steps
            .iter()
            .filter(|s| s.label == "dry-run" || s.label == "enforced-by-kani-gate" || proof_wf.is_match(&s.cmd))
            .count();
        let verus = steps.iter().filter(|s| s.label == "enforced-by-verus-gate").count();
        if terminal && executed == 0 {
            if verus > 0 {
                r.verus_only.push(id.clone());
            } else {
                findings.push("verified, but NO step is executed by the verification gate, the Kani gate, or lean.yml/rocq.yml".to_string());
            }
        }
        for s in steps.iter().filter(|s| s.label != "skip-no-steps") {
            for f in check_step(&s.cmd, tree) {
                findings.push(format!("[{}] {} — {}", s.label, s.cmd.chars().take(90).collect::<String>(), f));
            }
        }
        if !findings.is_empty() {
            if terminal { r.failing.push((id.clone(), status, findings)) } else { r.other.push((id.clone(), status, findings)) }
        }
    }
    r
}

fn run() -> Result<(Report, usize)> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let root = PathBuf::from(".");
    let dry = match args.iter().position(|a| a == "--dry-run-file").and_then(|i| args.get(i + 1)) {
        Some(f) => std::fs::read_to_string(f)?,
        None => {
            let o = std::process::Command::new("python3")
                .args(["scripts/run-falcon-verification.py", "--filter", "(not (= id \"\"))", "--dry-run"])
                .output()
                .context("running the verification gate in dry-run")?;
            if !o.status.success() {
                bail!("gate dry-run failed: {}", String::from_utf8_lossy(&o.stderr));
            }
            String::from_utf8(o.stdout)?
        }
    };
    let steps = parse_dry_run(&dry);
    if steps.is_empty() {
        bail!("the gate dry-run listed no artifacts — refusing to report a clean census");
    }
    let tree = Tree { packages: packages(&root)?, root };
    Ok((evaluate(&steps, &statuses(Path::new("."))?, &tree), steps.len()))
}

fn main() -> ExitCode {
    let (r, n) = match run() {
        Ok(v) => v,
        Err(e) => {
            eprintln!("check-evidence: could not evaluate: {e:#}");
            return ExitCode::from(2);
        }
    };
    println!(
        "checked {n} verification artifacts: {} verified/accepted with unresolved evidence, {} others, {} verified resting only on the (dark) Verus gate",
        r.failing.len(),
        r.other.len(),
        r.verus_only.len()
    );
    for (title, list) in [("FAIL (verified/accepted)", &r.failing), ("report only (not verified)", &r.other)] {
        if list.is_empty() {
            continue;
        }
        println!("\n== {title}");
        for (id, st, fs) in list {
            println!("{id} [{st}]");
            for f in fs {
                println!("    {f}");
            }
        }
    }
    if !r.verus_only.is_empty() {
        println!("\n== verified on the Verus gate alone (dark since 2026-09-11, #405): {}", r.verus_only.join(", "));
    }
    if r.failing.is_empty() { ExitCode::SUCCESS } else { ExitCode::from(1) }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A fixture tree unique to the calling test: tests run in parallel, and a
    /// shared directory let one test's cleanup delete another's files.
    fn tree_named(name: &str) -> Tree {
        let root = std::env::temp_dir().join(format!("check-evidence-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&root);
        std::fs::create_dir_all(root.join("scripts")).unwrap();
        std::fs::create_dir_all(root.join("crates/demo/src")).unwrap();
        std::fs::write(root.join("scripts/real.rs"), "").unwrap();
        std::fs::write(root.join("crates/demo/src/lib.rs"), "#[test]\nfn absent_battery_blocks_arming() {}\n").unwrap();
        Tree { packages: BTreeMap::from([("demo".to_string(), root.join("crates/demo"))]), root }
    }

    #[test]
    fn the_415_gazebo_stub_trap_is_caught() {
        let f = check_step("cargo run -p falcon-sitl-gz -- --backend=gazebo --world=falcon --model=quad", &tree_named("the_415_gazebo_stub_trap_is_caught"));
        assert!(f.iter().any(|m| m.contains("STUB")), "{f:?}");
        assert!(check_step("cargo run -p demo --features gazebo -- --backend=gazebo", &tree_named("the_415_gazebo_stub_trap_is_caught")).iter().all(|m| !m.contains("STUB")));
        assert!(check_step("cargo run -p demo -- --backend=gazebo  # intentionally the STUB backend: verifies the scaffold", &tree_named("the_415_gazebo_stub_trap_is_caught")).is_empty());
    }

    #[test]
    fn placeholders_and_bare_wasm_are_caught() {
        let t = tree_named("placeholders_and_bare_wasm_are_caught");
        assert!(check_step("witness run /path/to/instrumented.wasm", &t).iter().any(|m| m.contains("/path/to/")));
        assert!(check_step("$WITNESS_UNSET_FOR_TEST report", &t).iter().any(|m| m.contains("unset variable")));
        assert!(check_step("wasm-tools print falcon_mixer_cm.wasm", &t).iter().any(|m| m.contains("bare")));
        assert!(check_step("cd wasm/cm/rate", &t).iter().any(|m| m.contains("lone `cd`")));
    }

    #[test]
    fn paths_packages_and_test_filters_resolve_or_are_reported() {
        let t = tree_named("paths_packages_and_test_filters_resolve_or_are_reported");
        assert!(check_step("rust-script scripts/real.rs", &t).is_empty());
        assert!(check_step("rust-script scripts/missing.rs", &t).iter().any(|m| m.contains("does not exist")));
        assert!(check_step("cargo test -p demo --release absent_battery_blocks_arming", &t).is_empty());
        assert!(check_step("cargo test -p demo tests::absent_battery", &t).is_empty(), "module-qualified substring");
        assert!(check_step("cargo test -p demo no_such_test", &t).iter().any(|m| m.contains("matches no fn")));
        assert!(check_step("cargo test -p nope", &t).iter().any(|m| m.contains("does not exist")));
    }

    #[test]
    fn generics_are_not_placeholders_and_module_filters_resolve() {
        let t = tree_named("generics_are_not_placeholders_and_module_filters_resolve");
        assert!(check_step("grep -A1 'fn read(&mut self) -> Option<f32>' scripts/real.rs", &t).is_empty());
        assert!(check_step("tool --crate <crate>", &t).iter().any(|m| m.contains("placeholder")));
        std::fs::write(t.root.join("crates/demo/src/extra.rs"), "mod flow_tests {\n}\n").unwrap();
        assert!(check_step("cargo test -p demo flow_tests", &t).is_empty(), "a module name is a valid filter");
    }

    #[test]
    fn single_quoted_text_is_literal_not_a_variable_or_placeholder() {
        let t = tree_named("single_quoted_text_is_literal_not_a_variable_or_placeholder");
        std::fs::write(t.root.join("scripts/verification-tracks.rs"), "").unwrap();
        // FV-RELAY-VGATE-005's real step: `$OUT` is grep pattern text, not a variable.
        let step = "grep -q './scripts/verification-tracks.rs --sha \"$(git rev-parse HEAD)\" >> \"$OUT\" || rc=$?' scripts/real.rs";
        assert!(check_step(step, &t).is_empty(), "{:?}", check_step(step, &t));
        assert!(check_step("grep -q '<crate>' scripts/real.rs", &t).is_empty());
        // Outside quotes the same text is still caught.
        assert!(check_step("echo $OUT_UNSET_FOR_TEST", &t).iter().any(|m| m.contains("unset variable")));
    }

    #[test]
    fn lean_and_rocq_workflow_targets_count_as_executed() {
        let t = tree_named("lean_and_rocq_workflow_targets_count_as_executed");
        let dry = parse_dry_run("  [   skip-bench-only] FV-L: bazel test //proofs/lean:strict_lyapunov_test\n");
        let st = BTreeMap::from([("FV-L".to_string(), "verified".to_string())]);
        assert!(evaluate(&dry, &st, &t).failing.is_empty());
    }

    #[test]
    fn verified_with_nothing_executed_fails_but_verus_only_is_reported_apart() {
        let t = tree_named("verified_with_nothing_executed_fails_but_verus_only_is_reported_apart");
        let dry = parse_dry_run(
            "  [   skip-bench-only] FV-A: some bench thing\n  [enforced-by-verus-gate] FV-B: bazel test //:x_verus_test\n  [dry-run] FV-C step 1: rust-script scripts/real.rs\n",
        );
        let st = BTreeMap::from([
            ("FV-A".to_string(), "verified".to_string()),
            ("FV-B".to_string(), "verified".to_string()),
            ("FV-C".to_string(), "verified".to_string()),
        ]);
        let r = evaluate(&dry, &st, &t);
        assert_eq!(r.failing.len(), 1);
        assert_eq!(r.failing[0].0, "FV-A");
        assert_eq!(r.verus_only, vec!["FV-B".to_string()]);
    }
}
