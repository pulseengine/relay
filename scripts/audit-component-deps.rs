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
    ("relay-att", "#388 — wasm/cm/attitude wraps the legacy PID attitude controller; falcon-core flies geometric SE(3)"),
    ("relay-pos", "#388 — wasm/cm/position wraps the legacy PID position controller"),
    ("relay-rate", "#388 — wasm/cm/rate wraps the legacy PID rate controller; falcon-core flies ADRC"),
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
