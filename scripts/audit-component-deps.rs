#!/usr/bin/env rust-script
//! Fail if a published wasm component wraps a crate the flight core does not use.
//!
//! WHY THIS EXISTS. falcon-core moved to relay-adrc on 2026-06-03 and dropped
//! relay-pos / relay-att / relay-rate. The wasm components kept wrapping them.
//! For three months and ~40 releases we published, cosign-signed, pushed to
//! ghcr and handed to jess a control stack the flight core does not fly — and
//! nothing noticed, because nothing compared the two dependency sets.
//!
//! This is that comparison. It is deliberately five minutes of work, because
//! its absence cost three months.
//!
//! Usage:  scripts/audit-component-deps.rs [--json]
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! toml = "0.8"
//! ```

use anyhow::{bail, Context, Result};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

/// Deliberate exceptions. A component may legitimately wrap something the
/// flight core does not — but it must be WRITTEN DOWN with a reason, so the
/// decision is visible rather than inferred from silence.
///
/// The four below are the #388 defect itself, waived so this audit can be WIRED
/// IN NOW rather than after the fix. That is a deliberate trade and worth being
/// explicit about: a waiver is not silence. Every run prints these by name with
/// their issue, so the debt is in front of anyone reading CI — whereas what let
/// the defect last three months was that nothing compared the two sets at all.
///
/// The value of landing it waived is that it catches the NEXT one immediately.
/// Remove each entry as #388 brings that component up to the flown stack; the
/// audit then fails if it regresses.
const WAIVERS: &[(&str, &str)] = &[
    ("relay-att", "#388 — wasm/cm/attitude wraps the legacy PID attitude controller; falcon-core flies geometric SE(3). Also built into the shipped P3 stream pipeline via wasm/cm/cascade/src/orch.rs (#411)"),
    ("relay-pos", "#388 — wasm/cm/position wraps the legacy PID position controller. Also in the stream pipeline via orch.rs (#411)"),
    ("relay-rate", "#388 — wasm/cm/rate wraps the legacy PID rate controller; falcon-core flies ADRC. Also in the stream pipeline via orch.rs (#411)"),
    ("relay-ekf", "#388 — wasm/cm/ekf is the Mahony filter; falcon-core flies the IEKF"),
];

fn deps_of(manifest: &Path) -> Result<BTreeSet<String>> {
    let text = std::fs::read_to_string(manifest)
        .with_context(|| format!("reading {}", manifest.display()))?;
    let doc: toml::Value = text.parse().context("parsing TOML")?;
    let mut out = BTreeSet::new();
    for table in ["dependencies", "target"] {
        if let Some(t) = doc.get(table).and_then(|v| v.as_table()) {
            collect(t, &mut out);
        }
    }
    Ok(out)
}

/// Recurse so `[target.'cfg(...)'.dependencies]` is not a blind spot — a crate
/// moved behind a cfg would otherwise vanish from the audit.
fn collect(t: &toml::map::Map<String, toml::Value>, out: &mut BTreeSet<String>) {
    for (k, v) in t {
        if k.starts_with("relay-") {
            out.insert(k.clone());
        }
        if let Some(inner) = v.as_table() {
            collect(inner, out);
        }
    }
}

/// `relay_foo_bar` identifiers in a Rust source, as crate names `relay-foo-bar`.
fn relay_imports(text: &str) -> BTreeSet<String> {
    let mut out = BTreeSet::new();
    let bytes = text.as_bytes();
    let mut i = 0;
    while let Some(off) = text[i..].find("relay_") {
        let start = i + off;
        let boundary = start == 0 || !(bytes[start - 1].is_ascii_alphanumeric() || bytes[start - 1] == b'_');
        let mut end = start + "relay_".len();
        while end < bytes.len() && (bytes[end].is_ascii_lowercase() || bytes[end].is_ascii_digit() || bytes[end] == b'_') {
            end += 1;
        }
        if boundary && end > start + "relay_".len() {
            out.insert(text[start..end].replace('_', "-"));
        }
        i = end.max(start + 1);
    }
    out
}

/// Rust sources BUILD.bazel compiles from wasm/cm that are NOT a cargo crate
/// root. #411: `wasm/cm/cascade/src/orch.rs` imported relay_att/pos/rate — the
/// legacy cascade, shipped in the signed bundle — while cascade/Cargo.toml
/// listed only falcon-core, so a manifest-only audit reported the directory
/// clean. The divergence lived in undeclared sources; audit those too.
fn bazel_only_sources() -> Result<Vec<(String, String)>> {
    let build = std::fs::read_to_string("BUILD.bazel").context("reading BUILD.bazel")?;
    let mut seen = BTreeSet::new();
    let mut out = Vec::new();
    for piece in build.split('"') {
        if let Some(rest) = piece.strip_prefix("wasm/cm/") {
            if piece.ends_with(".rs") && !piece.ends_with("/lib.rs") && seen.insert(piece.to_string()) {
                let comp = rest.split('/').next().unwrap_or("").to_string();
                out.push((comp, piece.to_string()));
            }
        }
    }
    Ok(out)
}

fn main() -> Result<()> {
    let json = std::env::args().any(|a| a == "--json");

    let flown = deps_of(Path::new("crates/falcon-core/Cargo.toml"))
        .context("falcon-core is the flight core — its dep list is the reference")?;

    let mut components: BTreeMap<String, BTreeSet<String>> = BTreeMap::new();
    for entry in std::fs::read_dir("wasm/cm").context("reading wasm/cm")? {
        let dir = entry?.path();
        let manifest = dir.join("Cargo.toml");
        if !manifest.is_file() {
            continue;
        }
        let name = dir.file_name().unwrap().to_string_lossy().into_owned();
        components.insert(name, deps_of(&manifest)?);
    }
    if components.is_empty() {
        // Empty scope must not equal pass: a wasm/cm that vanished or moved
        // would otherwise make this audit report "all clear".
        bail!("found no components under wasm/cm — refusing to report a clean audit");
    }
    let bazel_sources = bazel_only_sources()?;
    let mut bazel_seen: Vec<(String, BTreeSet<String>)> = Vec::new();
    for (comp, path) in &bazel_sources {
        let text = std::fs::read_to_string(path).with_context(|| format!("reading {path}"))?;
        let deps = relay_imports(&text);
        let key = format!("{comp} [bazel-only {}]", path.trim_start_matches(&format!("wasm/cm/{comp}/")));
        bazel_seen.push((path.clone(), deps.clone()));
        components.insert(key, deps);
    }

    let waived: BTreeSet<&str> = WAIVERS.iter().map(|(c, _)| *c).collect();
    let mut offenders: Vec<(String, String)> = Vec::new();
    for (comp, deps) in &components {
        for d in deps {
            if !flown.contains(d) && !waived.contains(d.as_str()) {
                offenders.push((comp.clone(), d.clone()));
            }
        }
    }

    // Informational: what the flight core flies that no component wraps. Not a
    // failure — most of these are subsystems, not cascade stages — but it is
    // how you see that the FLOWN control law has no component at all.
    let wrapped: BTreeSet<&String> = components.values().flatten().collect();
    let unshipped: Vec<&String> = flown.iter().filter(|d| !wrapped.contains(d)).collect();

    if json {
        println!("{{\"offenders\":{:?},\"unshipped\":{:?}}}", offenders, unshipped);
    } else {
        println!("flight core (crates/falcon-core) depends on {} relay-* crates", flown.len());
        println!("components under wasm/cm: {}", components.len());
        println!();
        println!("BAZEL-ONLY SOURCES AUDITED (compiled by BUILD.bazel, in no Cargo.toml — #411): {}", bazel_seen.len());
        for (path, deps) in &bazel_seen {
            let list: Vec<&str> = deps.iter().map(String::as_str).collect();
            println!("    {path:<40} {}", if list.is_empty() { "(no relay-* imports)".to_string() } else { list.join(" ") });
        }
        println!();
        if !unshipped.is_empty() {
            println!("FLOWN BUT NOT WRAPPED BY ANY COMPONENT (informational):");
            for d in &unshipped {
                println!("    {d}");
            }
            println!();
        }
        if !WAIVERS.is_empty() {
            println!("WAIVED — tracked debt, printed every run so it cannot go quiet:");
            for (c, why) in WAIVERS {
                println!("    {c:<12} {why}");
            }
            println!();
        }
        if offenders.is_empty() {
            println!("PASS: every crate wrapped by a component is one the flight core uses.");
        } else {
            println!("SHIPPED BUT NOT FLOWN — {} violation(s):", offenders.len());
            for (comp, dep) in &offenders {
                println!("    wasm/cm/{comp:<14} wraps {dep}, which crates/falcon-core does NOT depend on");
            }
        }
    }

    if !offenders.is_empty() {
        bail!(
            "{} component dependency/dependencies are not in the flight core. Either bring the \
             component up to what falcon-core flies, or add a WAIVER with a reason and an issue. \
             Publishing a component built on a crate the vehicle does not use means the \
             verification evidence does not describe the artifact. See #388.",
            offenders.len()
        );
    }
    Ok(())
}
