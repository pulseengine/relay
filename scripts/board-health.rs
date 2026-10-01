#!/usr/bin/env rust-script
//! Measure the decay of the two places this project records what is left to do:
//! the ISSUE BOARD and the REQUIREMENT TRACE.
//!
//! WHY THIS EXISTS (SWREQ-RELAY-BOARD-P01). A sweep is the easy half; without a
//! number someone sees, the board and the trace rot back within weeks and nobody
//! notices until a planning question gets a wrong answer. Two measured examples:
//!
//!   - 57 open issues with 42% untouched for 30+ days, and nothing reporting it.
//!   - 79 sw-reqs whose status says "not started" while a verification artifact
//!     already points at them, NONE of which carry a `release:` field. So they
//!     are invisible to the plan twice. SWREQ-FALCON-MIX-P05 read `approved`
//!     while `mix_thrust_floor` was implemented, FLOWN in the production loop,
//!     and verified by FV-FALCON-MIX-002 — two lifecycle stages behind reality.
//!
//! The trace half is the worse one. A stale issue is visible on a list someone
//! reads; a requirement whose status contradicts its own trace silently
//! corrupts every readiness number, and readiness is what gates a tag.
//!
//! EMPTY SCOPE MUST NOT EQUAL PASS. The issue half needs `gh`, which is NOT
//! installed on the self-hosted runners. When it is missing this reports
//! SKIPPED and says so loudly rather than printing a clean board — a gate that
//! silently measures nothing is the defect class this repo keeps finding.
//!
//! Usage:
//!   scripts/board-health.rs              report only (exit 0)
//!   scripts/board-health.rs --gate       exit 1 if a threshold is breached
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! ```

use anyhow::{Context, Result};
use std::collections::BTreeMap;
use std::process::Command;

/// A requirement whose status claims it is un-started.
const UNSTARTED: &[&str] = &["draft", "proposed", "approved"];

struct Req {
    id: String,
    status: String,
    release: Option<String>,
}

/// One artifact that points a `verifies` link at a requirement.
struct Verifier {
    verified: bool,
    has_steps: bool,
}

/// How far an un-started requirement's evidence chain actually reaches.
///
/// WHY THIS IS GRADED RATHER THAN BOOLEAN. The first version of this script
/// asked only "does something point a `verifies` link at it", counted 79, and
/// called every one a status contradiction. Too weak, and it misled: of 90
/// un-started requirements, 61 are verified-by an `SV-RELAY-*` artifact that is
/// ITSELF `approved` and — in the case checked, SV-RELAY-015 — carries no
/// `steps:` at all, only `method: automated-test`. Those are NOT status-lagged;
/// their chain is incomplete two levels up, and promoting them would be wrong.
/// Only a requirement whose verifier is itself `verified` AND runnable is a
/// promotion candidate. That distinction took the count from 79 to 18.
#[derive(PartialEq, Eq)]
enum Chain {
    Promotable,
    VerifiedButStepless,
    VerifierUnpromoted,
    NoVerifier,
}

impl Chain {
    fn label(&self) -> &'static str {
        match self {
            Chain::Promotable => "verifier VERIFIED + has steps (promotion candidate)",
            Chain::VerifiedButStepless => "verifier verified but runs NOTHING",
            Chain::VerifierUnpromoted => "verifier itself NOT verified (chain incomplete)",
            Chain::NoVerifier => "no verifier at all (genuinely un-started)",
        }
    }
}

fn field(block: &str, name: &str) -> Option<String> {
    for line in block.lines() {
        let t = line.trim_start();
        if let Some(rest) = t.strip_prefix(&format!("{name}: ")) {
            if line.len() - t.len() == 4 {
                return Some(rest.trim().trim_matches('"').to_string());
            }
        }
    }
    None
}

/// Parse `artifacts/**.yaml` directly. `rivet list --format json` drops
/// `fields`, and a census built on it reports every artifact stepless — so the
/// YAML is the source here deliberately.
fn scan() -> Result<(Vec<Req>, BTreeMap<String, Vec<Verifier>>)> {
    let mut reqs = Vec::new();
    let mut verified_targets: BTreeMap<String, Vec<Verifier>> = BTreeMap::new();
    let mut stack = vec![std::path::PathBuf::from("artifacts")];
    while let Some(dir) = stack.pop() {
        for e in std::fs::read_dir(&dir).with_context(|| format!("reading {}", dir.display()))? {
            let p = e?.path();
            if p.is_dir() {
                stack.push(p);
                continue;
            }
            if p.extension().and_then(|s| s.to_str()) != Some("yaml") {
                continue;
            }
            let text = std::fs::read_to_string(&p)?;
            for block in text.split("\n  - id: ").skip(1) {
                let id = block.lines().next().unwrap_or("").trim().to_string();
                if field(block, "type").as_deref() == Some("sw-req") {
                    reqs.push(Req {
                        id,
                        status: field(block, "status").unwrap_or_default(),
                        release: field(block, "release"),
                    });
                }
                // Any artifact that `verifies` something counts as evidence
                // pointing at that target.
                let mut lines = block.lines().peekable();
                while let Some(l) = lines.next() {
                    if l.trim() == "- type: verifies" {
                        if let Some(n) = lines.peek() {
                            if let Some(tg) = n.trim().strip_prefix("target: ") {
                                verified_targets
                                    .entry(tg.trim().to_string())
                                    .or_default()
                                    .push(Verifier {
                                        verified: field(block, "status").as_deref()
                                            == Some("verified"),
                                        has_steps: block.contains("steps:"),
                                    });
                            }
                        }
                    }
                }
            }
        }
    }
    Ok((reqs, verified_targets))
}

fn main() -> Result<()> {
    let gate = std::env::args().any(|a| a == "--gate");
    let mut breaches: Vec<String> = Vec::new();

    // ── THE REQUIREMENT TRACE (offline; always runs) ──────────────────────
    let (reqs, has_verifier) = scan()?;
    let mut by_status: BTreeMap<&str, usize> = BTreeMap::new();
    let mut unstarted = Vec::new();
    for r in &reqs {
        *by_status.entry(r.status.as_str()).or_default() += 1;
        if UNSTARTED.contains(&r.status.as_str()) {
            unstarted.push(r);
        }
    }
    let mut graded: BTreeMap<&str, usize> = BTreeMap::new();
    let mut promotable = 0usize;
    for r in &unstarted {
        let vs = has_verifier.get(&r.id).map(|v| v.as_slice()).unwrap_or(&[]);
        let c = if vs.is_empty() {
            Chain::NoVerifier
        } else if vs.iter().any(|v| v.verified && v.has_steps) {
            Chain::Promotable
        } else if vs.iter().any(|v| v.verified) {
            Chain::VerifiedButStepless
        } else {
            Chain::VerifierUnpromoted
        };
        if c == Chain::Promotable {
            promotable += 1;
        }
        *graded.entry(c.label()).or_default() += 1;
    }
    let no_release = unstarted.iter().filter(|r| r.release.is_none()).count();

    println!("REQUIREMENT TRACE  ({} sw-req)", reqs.len());
    for (s, n) in &by_status {
        println!("  {:<12} {n}", if s.is_empty() { "(none)" } else { s });
    }
    println!("  status says NOT STARTED            {}", unstarted.len());
    println!("  ...carrying no `release:` field          {no_release}");
    println!("  evidence chain of those {}:", unstarted.len());
    for (label, n) in &graded {
        println!("    {n:3}  {label}");
    }
    if promotable > 0 {
        breaches.push(format!(
            "{promotable} requirement(s) say un-started while a VERIFIED, runnable verifier \
             points at them (status lag — re-verify individually, never bulk-promote)"
        ));
    }

    // ── THE ISSUE BOARD (needs gh; SKIPPED is not PASSED) ────────────────
    println!();
    let gh = Command::new("gh")
        .args([
            "issue", "list", "--repo", "pulseengine/relay", "--state", "open",
            "--limit", "200", "--json", "number,updatedAt",
        ])
        .output();
    match gh {
        Ok(o) if o.status.success() => {
            let body = String::from_utf8_lossy(&o.stdout);
            let open = body.matches("\"number\":").count();
            // A ROLLING 30-DAY CUT, not calendar-month matching. The first
            // version compared updatedAt's year-month against the current one
            // and reported "0 of 55 touched this month" on the 1st of October,
            // because everything had been touched in September — a false
            // breach on exactly one day in thirty. Caught on first run.
            // ISO-8601 dates compare correctly as strings, so a lexicographic
            // >= against a computed cutoff is both right and dependency-free.
            let cut = Command::new("date")
                .args(["-u", "-v-30d", "+%Y-%m-%d"])
                .output()
                .ok()
                .filter(|o| o.status.success())
                .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
                // GNU date (the Linux runners) has no -v; fall back to -d.
                .or_else(|| {
                    Command::new("date")
                        .args(["-u", "-d", "30 days ago", "+%Y-%m-%d"])
                        .output()
                        .ok()
                        .filter(|o| o.status.success())
                        .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
                })
                .context("neither BSD `date -v` nor GNU `date -d` worked")?;
            let fresh = body
                .split("\"updatedAt\":\"")
                .skip(1)
                .filter(|s| s.get(..10).map(|d| d >= cut.as_str()).unwrap_or(false))
                .count();
            let stale = open.saturating_sub(fresh);
            println!("ISSUE BOARD  (staleness cut {cut}, rolling 30 days)");
            println!("  open                               {open}");
            println!("  touched in the last 30 days        {fresh}");
            println!("  STALE (untouched 30+ days)         {stale}");
            if open > 0 && stale * 2 > open {
                breaches.push(format!(
                    "more than half the board ({stale} of {open}) is untouched for 30+ days"
                ));
            }
        }
        _ => {
            println!("ISSUE BOARD: **SKIPPED — `gh` unavailable**");
            println!("  This is NOT a pass. `gh` is not installed on the self-hosted");
            println!("  runners (see .github/actions/setup-gh). The board was not measured.");
            if gate {
                breaches.push("the issue board could not be measured (gh unavailable)".into());
            }
        }
    }

    println!();
    if breaches.is_empty() {
        println!("PASS: nothing measured here is decaying.");
        return Ok(());
    }
    println!("BREACHES ({}):", breaches.len());
    for b in &breaches {
        println!("  - {b}");
    }
    if gate {
        anyhow::bail!("board-health --gate: {} breach(es)", breaches.len());
    }
    println!("\n(report mode — rerun with --gate to fail on these)");
    Ok(())
}
