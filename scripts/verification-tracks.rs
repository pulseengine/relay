#!/usr/bin/env rust-script
//! What did each verification track actually do on the commit being released?
//!
//! WHY THIS EXISTS (SWREQ-RELAY-VGATE-P04). falcon-v1.138.0 shipped while two
//! verification tracks were dark — Verus failed in 0.1 s on every target for
//! days (#405), and a Bazel target had not built since #393 — and the release
//! said nothing, because the release notes were written from memory. This
//! generates the "verification tracks" section of the release notes from the
//! OBSERVED conclusions of each workflow on the released commit, so a dark or
//! unfinished track is recorded instead of remembered or forgotten.
//!
//! It RECORDS. It does not block the release: VGATE-P04's v1.139 scope is that
//! a track which cannot run is stated in the release as a known gap. Making the
//! tracks run is SWREQ-RELAY-VGATE-P05.
//!
//! Exit: 0 section generated · 2 could not read CI state. The caller must not
//! silently omit the section on 2; release.yml writes an explicit UNKNOWN.
//!
//! Usage:
//!   scripts/verification-tracks.rs [--sha <commit>] [--repo owner/name]
//!   rust-script --test scripts/verification-tracks.rs
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde = { version = "1", features = ["derive"] }
//! serde_json = "1"
//! ```

use anyhow::{bail, Context, Result};
use std::process::ExitCode;

/// Each verification track: its workflow, what it checks, and how it is
/// triggered (a track that never runs per main commit is reported as such, not
/// as "did not run").
struct Track {
    workflow: &'static str,
    name: &'static str,
    trigger: Trigger,
}

#[derive(Clone, Copy, PartialEq)]
enum Trigger {
    EveryMainCommit,
    PullRequestOnly,
    Scheduled,
}

const TRACKS: &[Track] = &[
    Track { workflow: "ci.yml", name: "Tests ×3, Clippy, Format", trigger: Trigger::EveryMainCommit },
    Track { workflow: "kani.yml", name: "Kani bounded model checking", trigger: Trigger::EveryMainCommit },
    Track { workflow: "verus.yml", name: "Verus SMT proofs", trigger: Trigger::EveryMainCommit },
    Track { workflow: "lean.yml", name: "Lean proofs", trigger: Trigger::EveryMainCommit },
    Track { workflow: "rocq.yml", name: "Rocq / Gappa proofs", trigger: Trigger::EveryMainCommit },
    Track { workflow: "gazebo.yml", name: "Gazebo SITL (native + wasm)", trigger: Trigger::EveryMainCommit },
    Track { workflow: "bazel.yml", name: "Bazel build", trigger: Trigger::EveryMainCommit },
    Track { workflow: "coverage.yml", name: "Coverage", trigger: Trigger::EveryMainCommit },
    Track { workflow: "spar.yml", name: "spar AADL analysis + WIT drift", trigger: Trigger::EveryMainCommit },
    Track { workflow: "verification-gate.yml", name: "rivet verification gate", trigger: Trigger::PullRequestOnly },
    Track { workflow: "soak.yml", name: "Nightly endurance soak", trigger: Trigger::Scheduled },
];

/// Known STRUCTURAL gaps: not visible in one commit's run conclusions (a track
/// that is green but checks less than it seems, or never runs where it
/// should). Each is an open issue; when the issue closes, this tool says so
/// instead of repeating a gap that may be fixed.
const STRUCTURAL: &[(u32, &str)] = &[
    (405, "Verus cannot find core/std — every verus_test fails before verifying anything"),
    (418, "verus.yml's PR trigger is path-filtered on Lean paths, so Verus does not run on the PRs that change Verus code"),
    (410, "the required verification gate has no main backstop; its scope filter can exclude what a PR changed"),
    (407, "//:falcon-cascade-coverage fails to fuse; cascade MC/DC coverage has never been produced in CI"),
    (417, "the required gates can pass vacuously for a PR that touches only the shipped wasm components"),
    (436, "scheduled monitors run every few hours, not on their cron cadence"),
];

#[derive(Debug, Clone, serde::Deserialize)]
struct Run {
    status: String,
    #[serde(default)]
    conclusion: Option<String>,
    created_at: String,
    html_url: String,
}

#[derive(Debug, PartialEq)]
enum State {
    Green,
    Dark(String),
    Unfinished(String),
    NotRun,
}

/// The latest run of a workflow for the commit decides its state.
fn classify(runs: &[Run]) -> (State, Option<String>) {
    let Some(r) = runs.iter().max_by(|a, b| a.created_at.cmp(&b.created_at)) else {
        return (State::NotRun, None);
    };
    let st = match (r.status.as_str(), r.conclusion.as_deref()) {
        ("completed", Some("success")) => State::Green,
        ("completed", Some(c)) => State::Dark(c.to_string()),
        (s, _) => State::Unfinished(s.to_string()),
    };
    (st, Some(r.html_url.clone()))
}

fn cell(track: &Track, state: &State, url: &Option<String>) -> String {
    let link = |t: &str| match url {
        Some(u) => format!("[{t}]({u})"),
        None => t.to_string(),
    };
    match (state, track.trigger) {
        (State::Green, _) => link("✅ passed"),
        (State::Dark(c), _) => link(&format!("🔴 **DARK** — {c}")),
        (State::Unfinished(s), _) => link(&format!("⏳ not finished at release time ({s})")),
        (State::NotRun, Trigger::PullRequestOnly) => "⚪ runs on pull requests only — no run on this commit".into(),
        (State::NotRun, Trigger::Scheduled) => "⚪ scheduled, not per commit — see its latest run".into(),
        (State::NotRun, Trigger::EveryMainCommit) => "⚪ **did not run** for this commit".into(),
    }
}

fn render(sha: &str, rows: &[(&Track, State, Option<String>)], gaps: &[(u32, &str, bool)]) -> String {
    let dark = rows.iter().filter(|(_, s, _)| matches!(s, State::Dark(_))).count();
    let missing = rows
        .iter()
        .filter(|(t, s, _)| t.trigger == Trigger::EveryMainCommit && matches!(s, State::NotRun | State::Unfinished(_)))
        .count();
    let mut out = format!("## Verification tracks at `{}`\n\n", &sha[..sha.len().min(12)]);
    out += "*Generated from the observed CI conclusions on the released commit by `scripts/verification-tracks.rs` \
(SWREQ-RELAY-VGATE-P04) — not written by hand.*\n\n";
    if dark == 0 && missing == 0 {
        out += "Every per-commit track ran and passed on this commit.\n\n";
    } else {
        out += &format!(
            "**{dark} track(s) DARK and {missing} per-commit track(s) not run or unfinished on this commit.** \
A dark track verified nothing for this release; treat its claims accordingly.\n\n"
        );
    }
    out += "| track | workflow | on this commit |\n|---|---|---|\n";
    for (t, s, u) in rows {
        out += &format!("| {} | `{}` | {} |\n", t.name, t.workflow, cell(t, s, u));
    }
    out += "\n### Known structural gaps\n\nA green track can still check less than it appears to. Open at release time:\n\n";
    for (n, text, open) in gaps {
        if *open {
            out += &format!("- #{n} — {text}\n");
        } else {
            out += &format!("- #{n} — {text} *(issue now closed — verify the gap is gone and remove this entry)*\n");
        }
    }
    out
}

fn gh_json(path: &str) -> Result<serde_json::Value> {
    let o = std::process::Command::new("gh").args(["api", path]).output().context("running gh")?;
    if !o.status.success() {
        bail!("gh api {path}: {}", String::from_utf8_lossy(&o.stderr).trim());
    }
    Ok(serde_json::from_slice(&o.stdout)?)
}

fn run() -> Result<String> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let arg = |k: &str| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).cloned();
    let repo = arg("--repo").or_else(|| std::env::var("GITHUB_REPOSITORY").ok()).unwrap_or_else(|| "pulseengine/relay".into());
    let sha = match arg("--sha") {
        Some(s) => s,
        None => String::from_utf8(std::process::Command::new("git").args(["rev-parse", "HEAD"]).output()?.stdout)?
            .trim()
            .to_string(),
    };
    if sha.len() < 7 {
        bail!("no commit sha");
    }
    let mut rows = Vec::new();
    for t in TRACKS {
        let v = gh_json(&format!("repos/{repo}/actions/workflows/{}/runs?head_sha={sha}&per_page=30", t.workflow))?;
        let runs: Vec<Run> = serde_json::from_value(v["workflow_runs"].clone()).context("parsing workflow_runs")?;
        let (s, u) = classify(&runs);
        rows.push((t, s, u));
    }
    let mut gaps = Vec::new();
    for (n, text) in STRUCTURAL {
        let v = gh_json(&format!("repos/{repo}/issues/{n}"))?;
        gaps.push((*n, *text, v["state"].as_str() == Some("open")));
    }
    Ok(render(&sha, &rows, &gaps))
}

fn main() -> ExitCode {
    match run() {
        Ok(md) => {
            print!("{md}");
            ExitCode::SUCCESS
        }
        Err(e) => {
            eprintln!("verification-tracks: could not evaluate: {e:#}");
            ExitCode::from(2)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn run_of(status: &str, conclusion: Option<&str>, at: &str) -> Run {
        Run { status: status.into(), conclusion: conclusion.map(Into::into), created_at: at.into(), html_url: format!("https://x/{at}") }
    }

    #[test]
    fn the_latest_run_decides_and_failure_is_dark() {
        let runs = [run_of("completed", Some("success"), "2026-09-17T01:00:00Z"), run_of("completed", Some("failure"), "2026-09-17T02:00:00Z")];
        assert_eq!(classify(&runs).0, State::Dark("failure".into()));
        assert_eq!(classify(&[]).0, State::NotRun);
        assert_eq!(classify(&[run_of("in_progress", None, "t")]).0, State::Unfinished("in_progress".into()));
    }

    #[test]
    fn a_dark_verus_track_is_recorded_not_hidden() {
        // The v1.138.0 situation: Verus red, everything else green.
        let rows: Vec<(&Track, State, Option<String>)> = TRACKS
            .iter()
            .map(|t| {
                let s = match t.workflow {
                    "verus.yml" => State::Dark("failure".into()),
                    "verification-gate.yml" | "soak.yml" => State::NotRun,
                    _ => State::Green,
                };
                (t, s, None)
            })
            .collect();
        let md = render("9c53bff0000000", &rows, &[(405, "Verus cannot find core/std", true)]);
        assert!(md.contains("**1 track(s) DARK"), "{md}");
        assert!(md.contains("| Verus SMT proofs | `verus.yml` | 🔴 **DARK** — failure |"), "{md}");
        assert!(md.contains("runs on pull requests only"), "PR-only is not reported as a miss: {md}");
        assert!(md.contains("- #405 — Verus cannot find core/std"), "{md}");
        assert!(!md.contains("Every per-commit track ran and passed"), "{md}");
    }

    #[test]
    fn a_per_commit_track_that_did_not_run_is_counted() {
        let t = &TRACKS[0];
        let md = render("abcdef1234567", &[(t, State::NotRun, None)], &[]);
        assert!(md.contains("1 per-commit track(s) not run"), "{md}");
    }

    #[test]
    fn a_closed_structural_issue_is_flagged_for_removal() {
        let md = render("abcdef1234567", &[], &[(407, "coverage never produced", false)]);
        assert!(md.contains("issue now closed — verify the gap is gone"), "{md}");
    }
}
