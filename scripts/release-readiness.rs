#!/usr/bin/env rust-script
//! What is the next release actually waiting on?
//!
//! WHY THIS EXISTS. Release readiness has been carried in an agent's head and in
//! a scratch state file, and both drifted. The loop's `pending_gates` still said
//! "TAG falcon-v1.136.0" after v1.138.0 had shipped; its watermark sat six days
//! stale while twelve new issues were filed. A number that is recomputed from
//! the tree every night cannot drift like that.
//!
//! It REPORTS. It does not tag, merge, or close anything. Cutting a release is a
//! decision with a signature and a partner on the other end, and the release
//! process names it an explicit stop-and-ask. This tool exists to make that
//! decision well-informed, not to make it automatically.
//!
//! Readiness is the rivet query the release plan already defines: a release is
//! cuttable when every artifact scoped to it is `verified` (or `accepted`) and
//! nothing in its scope is still open.
//!
//! Usage:
//!   scripts/release-readiness.rs                 # the next unreleased version
//!   scripts/release-readiness.rs --release falcon-v1.139.0
//!   scripts/release-readiness.rs --markdown      # GitHub job-summary form
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_yaml = "0.9"
//! serde = { version = "1", features = ["derive"] }
//! walkdir = "2"
//! ```

use anyhow::{Context, Result};
use std::collections::BTreeMap;

#[derive(Debug, serde::Deserialize)]
struct Doc {
    #[serde(default)]
    artifacts: Vec<Artifact>,
}

#[derive(Debug, serde::Deserialize)]
struct Artifact {
    id: String,
    #[serde(default)]
    #[allow(dead_code)]
    r#type: String,
    #[serde(default)]
    title: String,
    #[serde(default)]
    status: String,
    #[serde(default)]
    release: String,
}

/// A status that means the artifact is DONE for release purposes. Everything
/// else blocks. `implemented` deliberately counts as blocking: the two-commit
/// rule makes promotion to `verified` a separate, evidence-bearing step, so an
/// `implemented` artifact is precisely one whose evidence has not been shown.
fn is_done(status: &str) -> bool {
    matches!(status, "verified" | "accepted")
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let markdown = args.iter().any(|a| a == "--markdown");
    let want = args
        .iter()
        .position(|a| a == "--release")
        .and_then(|i| args.get(i + 1))
        .cloned();

    let mut by_release: BTreeMap<String, Vec<Artifact>> = BTreeMap::new();
    for e in walkdir::WalkDir::new("artifacts")
        .into_iter()
        .filter_map(|e| e.ok())
        .filter(|e| e.path().extension().is_some_and(|x| x == "yaml"))
    {
        let text = std::fs::read_to_string(e.path())
            .with_context(|| format!("reading {}", e.path().display()))?;
        // A malformed artifact file must not silently shrink the scope — that is
        // the empty-scope-passes shape. Report it and keep going.
        let doc: Doc = match serde_yaml::from_str(&text) {
            Ok(d) => d,
            Err(err) => {
                eprintln!("::warning::unparseable artifact {}: {err}", e.path().display());
                continue;
            }
        };
        for a in doc.artifacts {
            if !a.release.is_empty() {
                by_release.entry(a.release.clone()).or_default().push(a);
            }
        }
    }

    // Default target: the lowest release that is NOT YET TAGGED.
    //
    // "Lowest release with something incomplete" is the obvious rule and it is
    // wrong: it selects falcon-v0.1.0, which shipped long ago and still carries
    // artifacts left at `implemented`. A shipped release's stale statuses are a
    // traceability debt, not a thing the NEXT release is waiting on. The next
    // release is the one whose tag does not exist yet.
    let tags: std::collections::HashSet<String> = std::process::Command::new("git")
        .args(["tag", "--list", "falcon-v*"])
        .output()
        .map(|o| {
            String::from_utf8_lossy(&o.stdout)
                .lines()
                .map(|l| l.trim().to_string())
                .collect()
        })
        .unwrap_or_default();

    // ...and it must be AHEAD of the latest tag. "Lowest untagged" is still
    // wrong on its own: artifacts exist scoped to falcon-v1.113.0, which was
    // never tagged under that number (v1.136 and v1.137 were likewise folded
    // into the v1.138.0 cut). Those are historical scopes, not pending work.
    let ver = |r: &str| -> (u32, u32, u32) {
        let n = r.trim_start_matches("falcon-v");
        let mut it = n.split('.').map(|x| x.parse::<u32>().unwrap_or(0));
        (
            it.next().unwrap_or(0),
            it.next().unwrap_or(0),
            it.next().unwrap_or(0),
        )
    };
    let latest = tags.iter().map(|t| ver(t)).max().unwrap_or((0, 0, 0));

    // Scoped to a version at or below the latest tag, but never tagged under
    // that number. Reported because it is exactly how a release scope goes
    // quietly missing — not because it blocks the next release.
    let stranded: Vec<&String> = by_release
        .keys()
        .filter(|r| !tags.contains(*r) && ver(r) <= latest)
        .collect();

    let target = match want {
        Some(r) => r,
        None => by_release
            .keys()
            .find(|r| !tags.contains(*r) && ver(r) > latest)
            .cloned()
            .unwrap_or_else(|| "(nothing scoped beyond the latest tag)".into()),
    };

    let empty = Vec::new();
    let arts = by_release.get(&target).unwrap_or(&empty);
    let mut blocking: Vec<&Artifact> = arts.iter().filter(|a| !is_done(&a.status)).collect();
    blocking.sort_by(|a, b| a.status.cmp(&b.status).then(a.id.cmp(&b.id)));

    let mut counts: BTreeMap<&str, usize> = BTreeMap::new();
    for a in arts {
        *counts.entry(a.status.as_str()).or_default() += 1;
    }

    let done = arts.iter().filter(|a| is_done(&a.status)).count();
    let pct = if arts.is_empty() { 0 } else { done * 100 / arts.len() };

    if markdown {
        println!("## Release readiness — `{target}`\n");
        println!("**{done} of {} artifacts done ({pct}%).**\n", arts.len());
        if arts.is_empty() {
            println!("No artifacts are scoped to this release yet.\n");
        }
        println!("| status | count |");
        println!("|---|---|");
        for (s, n) in &counts {
            println!("| `{s}` | {n} |");
        }
        if blocking.is_empty() && !arts.is_empty() {
            println!("\n**Every artifact in scope is done.** The V-model gate and a green CI run on the tag commit are what remain — see the release process; tagging stays a human decision.\n");
        } else if !blocking.is_empty() {
            println!("\n### Blocking\n");
            println!("| status | artifact | title |");
            println!("|---|---|---|");
            for a in &blocking {
                let t = a.title.chars().take(90).collect::<String>();
                println!("| `{}` | `{}` | {} |", a.status, a.id, t);
            }
        }
        if !stranded.is_empty() {
            println!("\n### Scoped to a version that was never tagged\n");
            println!("Not blocking this release — but this is how a scope goes quietly missing.\n");
            for r in &stranded {
                println!("- `{r}`");
            }
        }
        println!("\n_Reported, not enforced. This tool never tags, merges or closes._");
    } else {
        println!("release-readiness: {target}");
        println!("  {done}/{} artifacts done ({pct}%)", arts.len());
        for (s, n) in &counts {
            println!("    {s:<12} {n}");
        }
        if !stranded.is_empty() {
            println!("  scoped but never tagged (historical): {}",
                     stranded.iter().map(|s| s.as_str()).collect::<Vec<_>>().join(", "));
        }
        if !blocking.is_empty() {
            println!("  blocking:");
            for a in &blocking {
                println!("    [{:<11}] {}", a.status, a.id);
            }
        }
    }

    // Exit code is information, not a gate: 0 = ready, 1 = still blocked. A
    // caller that wants to fail on "not ready" can; the nightly job does not.
    std::process::exit(if blocking.is_empty() && !arts.is_empty() { 0 } else { 1 });
}
