#!/usr/bin/env rust-script
//! Fail if a wasm component's published description claims verification its
//! wrapped crates do not have.
//!
//! WHY THIS EXISTS (#412, SWREQ-FALCON-CLAIMS-P01). `release.yml` publishes each
//! `wasm/cm/*/Cargo.toml` `description` verbatim as the OCI image description.
//! On 2026-09-17 five of the six published components said "Formally-verified",
//! and three of those wrapped crates with NO code-level verification in CI —
//! including `attitude`, advertised as a "geometric SO(3)" controller while
//! wrapping `relay-att`, a quaternion-error proportional controller. A user
//! picks a component on its description; the description has to be true.
//!
//! THE RULE, deliberately narrow so it can be checked mechanically:
//!   * Generic formal adjectives — "formally", "verified", "proven", "proof",
//!     "certified" — are refused outright: they do not say what was checked.
//!   * "Lean" and "Rocq"/"Coq" are refused: those proofs are about control-law
//!     MODELS, not about the component's code.
//!   * "Kani" is allowed only if every crate the description names in
//!     parentheses, e.g. `(relay-mix-quad)`, is in kani.yml's engine matrix
//!     (or, if it names none, every wrapped relay-*/falcon-core crate is).
//!   * "Verus" is allowed only on the same terms against the `verus_test`
//!     targets in BUILD.bazel.
//!
//! Exit codes: 0 all descriptions honest · 1 a claim is unsupported ·
//! 2 could not evaluate (a file failed to parse). 2 is never folded into 0.
//!
//! Usage:
//!   scripts/check-component-claims.rs
//!   rust-script --test scripts/check-component-claims.rs
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_yaml = "0.9"
//! toml = "0.8"
//! regex = "1"
//! ```

use anyhow::{bail, Context, Result};
use regex::Regex;
use std::collections::BTreeSet;
use std::process::ExitCode;

const GENERIC_FORMAL: &[&str] = &["formally", "verified", "proven", "proof", "certified"];
const MODEL_ONLY: &[&str] = &["lean", "rocq", "coq"];

/// Every unsupported claim in one description. Pure, so it is unit-testable.
fn check(
    component: &str,
    description: &str,
    wrapped: &BTreeSet<String>,
    kani: &BTreeSet<String>,
    verus: &BTreeSet<String>,
) -> Vec<String> {
    let lower = description.to_lowercase();
    let word = |w: &str| Regex::new(&format!(r"\b{}\b", regex::escape(w))).expect("static regex").is_match(&lower);
    let mut out = Vec::new();

    for w in GENERIC_FORMAL {
        if word(w) {
            out.push(format!(
                "{component}: says \"{w}\" — generic formal claims are refused; name the method and what it covers"
            ));
        }
    }
    for w in MODEL_ONLY {
        if word(w) {
            out.push(format!(
                "{component}: mentions \"{w}\" — those proofs cover control-law models, not this component's code"
            ));
        }
    }

    let named: BTreeSet<String> = Regex::new(r"\(((?:relay|falcon)-[a-z0-9-]+)\)")
        .expect("static regex")
        .captures_iter(description)
        .map(|c| c[1].to_string())
        .collect();
    let subjects: Vec<&String> = if named.is_empty() { wrapped.iter().collect() } else { named.iter().collect() };

    for (method, set) in [("kani", kani), ("verus", verus)] {
        if !word(method) {
            continue;
        }
        if subjects.is_empty() {
            out.push(format!("{component}: claims {method} but wraps no relay-*/falcon-core crate"));
        }
        for s in &subjects {
            if !set.contains(*s) {
                out.push(format!("{component}: claims {method} for {s}, which is not covered by that track"));
            }
        }
    }
    out
}

fn kani_matrix(path: &str) -> Result<BTreeSet<String>> {
    let y: serde_yaml::Value = serde_yaml::from_str(&std::fs::read_to_string(path)?)?;
    let engines = y["jobs"]["kani"]["strategy"]["matrix"]["engine"]
        .as_sequence()
        .context("kani.yml: jobs.kani.strategy.matrix.engine is not a list")?;
    let set: BTreeSet<String> = engines.iter().filter_map(|e| e.as_str().map(str::to_owned)).collect();
    if set.is_empty() {
        bail!("kani.yml: the engine matrix is empty — refusing to judge claims against nothing");
    }
    Ok(set)
}

fn verus_targets(path: &str) -> Result<BTreeSet<String>> {
    let text = std::fs::read_to_string(path)?;
    let block = Regex::new(r"(?s)verus_test\((.*?)\n\)")?;
    let src = Regex::new(r"crates/([a-z0-9-]+)/src/")?;
    Ok(block
        .captures_iter(&text)
        .flat_map(|b| src.captures_iter(&b[1]).map(|c| c[1].to_string()).collect::<Vec<_>>())
        .collect())
}

fn components(root: &str) -> Result<Vec<(String, String, BTreeSet<String>)>> {
    let mut out = Vec::new();
    let mut dirs: Vec<_> = std::fs::read_dir(root)?.filter_map(|e| e.ok()).map(|e| e.path()).collect();
    dirs.sort();
    for dir in dirs {
        let manifest = dir.join("Cargo.toml");
        if !manifest.exists() {
            continue;
        }
        let t: toml::Value = toml::from_str(&std::fs::read_to_string(&manifest)?)
            .with_context(|| format!("parsing {}", manifest.display()))?;
        let name = dir.file_name().unwrap().to_string_lossy().into_owned();
        let desc = t["package"].get("description").and_then(|d| d.as_str()).unwrap_or("").to_owned();
        let wrapped = t
            .get("dependencies")
            .and_then(|d| d.as_table())
            .map(|d| {
                d.keys()
                    .filter(|k| k.starts_with("relay-") || k.as_str() == "falcon-core")
                    .cloned()
                    .collect()
            })
            .unwrap_or_default();
        out.push((name, desc, wrapped));
    }
    Ok(out)
}

fn run() -> Result<Vec<String>> {
    let kani = kani_matrix(".github/workflows/kani.yml")?;
    let verus = verus_targets("BUILD.bazel")?;
    let comps = components("wasm/cm")?;
    if comps.is_empty() {
        bail!("no components found under wasm/cm — refusing to report a clean check");
    }
    println!(
        "checked {} component descriptions against {} Kani engines and {} Verus targets",
        comps.len(),
        kani.len(),
        verus.len()
    );
    Ok(comps.iter().flat_map(|(n, d, w)| check(n, d, w, &kani, &verus)).collect())
}

fn main() -> ExitCode {
    match run() {
        Ok(v) if v.is_empty() => {
            println!("PASS: every verification claim in a component description is backed by CI");
            ExitCode::SUCCESS
        }
        Ok(v) => {
            for line in &v {
                println!("UNSUPPORTED  {line}");
            }
            ExitCode::from(1)
        }
        Err(e) => {
            eprintln!("check-component-claims: could not evaluate: {e:#}");
            ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn set(xs: &[&str]) -> BTreeSet<String> {
        xs.iter().map(|s| s.to_string()).collect()
    }

    #[test]
    fn the_412_attitude_description_is_refused() {
        // The description published through falcon-v1.138.0.
        let v = check(
            "attitude",
            "Formally-verified geometric SO(3) attitude controller as a WebAssembly Component — attitude setpoint + vehicle state → body-rate setpoint.",
            &set(&["relay-att"]),
            &set(&["relay-mix-quad", "relay-iekf"]),
            &set(&["relay-lc"]),
        );
        assert!(v.iter().any(|m| m.contains("\"formally\"")), "{v:?}");
        assert!(v.iter().any(|m| m.contains("\"verified\"")), "{v:?}");
    }

    #[test]
    fn a_kani_claim_needs_the_named_crate_in_the_matrix() {
        let kani = set(&["relay-mix-quad"]);
        let ok = check("mixer", "Allocator (relay-mix-quad). Kani harnesses run in CI.", &set(&["relay-mix-quad"]), &kani, &set(&[]));
        assert!(ok.is_empty(), "{ok:?}");
        let bad = check("rate", "PID (relay-rate). Kani harnesses run in CI.", &set(&["relay-rate"]), &kani, &set(&[]));
        assert_eq!(bad.len(), 1, "{bad:?}");
    }

    #[test]
    fn with_no_crate_named_every_wrapped_crate_must_be_covered() {
        // iekf wraps relay-iekf AND relay-math; a bare "Kani" claim covers both.
        let v = check("iekf", "Estimator. Kani checked.", &set(&["relay-iekf", "relay-math"]), &set(&["relay-iekf"]), &set(&[]));
        assert_eq!(v.len(), 1);
        assert!(v[0].contains("relay-math"));
    }

    #[test]
    fn model_proofs_are_not_code_claims() {
        let v = check("flight", "Runs the cascade that is Lean-verified.", &set(&["falcon-core"]), &set(&[]), &set(&[]));
        assert!(v.iter().any(|m| m.contains("\"lean\"")), "{v:?}");
        assert!(v.iter().any(|m| m.contains("\"verified\"")), "{v:?}");
    }

    #[test]
    fn an_honest_description_passes() {
        let v = check(
            "attitude",
            "LEGACY quaternion-error proportional attitude controller (relay-att) as a WebAssembly Component.",
            &set(&["relay-att"]),
            &set(&["relay-mix-quad"]),
            &set(&[]),
        );
        assert!(v.is_empty(), "{v:?}");
    }

    #[test]
    fn words_match_whole_words_only() {
        // "unverified" and "improve" must not trip "verified"/"proven".
        let v = check("x", "An unverified draft we plan to improve.", &set(&[]), &set(&[]), &set(&[]));
        assert!(v.is_empty(), "{v:?}");
    }
}
