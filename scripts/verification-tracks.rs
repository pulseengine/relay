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
    probe: Probe,
}

/// Which part of a run actually VERIFIES something. A run's own conclusion
/// cannot say: kani.yml's "Detect engine changes" scope job can skip every
/// matrix leg and its "Kani gate" aggregator still succeeds, so the RUN is
/// `success` having checked nothing (run 42010229156, merge_group on
/// 725f146). The verification gate is one job whose sweep step is skipped on
/// a FAST-PASS (run 42010229350). So each track names its substantive part,
/// read from the jobs API (job and step conclusions; no log scraping).
#[derive(Clone, Copy, Debug)]
enum Probe {
    /// Every job is substantive (single-job and plain multi-job workflows).
    AnyJob,
    /// Only jobs whose name starts with this. A skipped matrix job is
    /// reported with its UNEXPANDED name (`Kani (${{ matrix.engine }})`), a
    /// run leg as `Kani (relay-ccsds)`; both match the prefix.
    JobPrefix(&'static str),
    /// One named step inside one named job.
    Step { job: &'static str, step: &'static str },
}

#[derive(Clone, Copy, PartialEq)]
enum Trigger {
    EveryMainCommit,
    PullRequestOnly,
    /// Per-PR AND a nightly cron over main. Absence from a given commit is
    /// expected rather than a miss, same as PullRequestOnly, but the rendered
    /// text must not claim PR-only coverage.
    PullRequestAndNightly,
    Scheduled,
}

const TRACKS: &[Track] = &[
    Track { workflow: "ci.yml", name: "Tests ×3, Clippy, Format", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "kani.yml", name: "Kani bounded model checking", trigger: Trigger::EveryMainCommit, probe: Probe::JobPrefix("Kani (") },
    Track { workflow: "verus.yml", name: "Verus SMT proofs", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "lean.yml", name: "Lean proofs", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "rocq.yml", name: "Rocq / Gappa proofs", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "gazebo.yml", name: "Gazebo SITL (native + wasm)", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "bazel.yml", name: "Bazel build", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "coverage.yml", name: "Coverage", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    Track { workflow: "spar.yml", name: "spar AADL analysis + WIT drift", trigger: Trigger::EveryMainCommit, probe: Probe::AnyJob },
    // NOT PullRequestOnly since #410 gave this workflow `schedule: 43 1 * * *`,
    // a nightly backstop on main whose first scheduled run swept 207 artifacts.
    // The stale label shipped in falcon-v1.140.0's notes as "runs on pull
    // requests only", understating the coverage that release actually had.
    Track { workflow: "verification-gate.yml", name: "rivet verification gate", trigger: Trigger::PullRequestAndNightly,
        // The sweep step, skipped on a FAST-PASS. verification-gate.yml carries
        // a comment at that step saying this tool reads its name.
        probe: Probe::Step { job: "Verification gate (rivet-driven)", step: "Run falcon verification gate" } },
    Track { workflow: "soak.yml", name: "Nightly endurance soak", trigger: Trigger::Scheduled, probe: Probe::AnyJob },
];

/// Known STRUCTURAL gaps: not visible in one commit's run conclusions (a track
/// that is green but checks less than it seems, or never runs where it
/// should). Each is an open issue; when the issue closes, this tool says so
/// instead of repeating a gap that may be fixed.
// FOUR ENTRIES RETIRED AT falcon-v1.140.0, each verified fixed ON MAIN rather
// than assumed from a merge:
//   405 — verus.yml now asserts a floor and reports 281 functions / 0 errors /
//         19 targets (run 37881361372). Issue closed.
//   418 — verus.yml's PR `paths` is ['crates/**', 'BUILD.bazel', 'MODULE.bazel',
//         'MODULE.bazel.lock', '.github/workflows/verus.yml'].
//   410 — verification-gate.yml carries `schedule: 43 1 * * *` AND its first
//         schedule-event run (37902164712) swept 207 artifacts on main. A cron
//         in a file is not evidence; an observed non-empty sweep is.
//   407 — bazel.yml queries (`kind("alias rule", //:*)` + a MUST guard) instead
//         of enumerating.
// Their issues may still be open here — closure cites a TAG, not a merge — so
// they are removed by VERIFIED STATE, not by issue state. Do not re-add one
// without re-measuring it.
//
// TWO MORE RETIRED 2026-10-09, same rule, each re-measured on main:
//   452 — the mechanism AND the claim are now both verified. Zeroing the
//         achieved-acceleration term (so specific_force_ned reverts to the
//         pre-fix [0,0,-GRAVITY]) makes 10 of falcon-core's 69 tests FAIL,
//         including two that FV-FALCON-HOLD-001 cites by name. Baseline 69/0,
//         mutated 59/10, file restored byte-identical. Issue closed on
//         falcon-v1.140.0.
//   546 — RE-ADDED 2026-11-04 (#609 finding 12). The horizontal half is
//         closed on main: the gz verdict bars peak_horiz < 1.0 and
//         rms_steady < 0.5, equal to HOLD-P01's figures. The VERTICAL half is
//         not: altitude is bounded only by the final sample plus an RMS over
//         the last 5 s (the wasm leg by the final sample alone), and the
//         required gz gate flies 25 s, so it cannot evidence HOLD-P01's
//         ">= 60 s" at all. Retiring the whole entry overstated the evidence.
//         Maintainer decision: disclose now; a defined hold window plus a
//         settled-window vertical peak bar, judged on >= 60 s soak rungs, is
//         a falcon-v1.142.0 item.
const STRUCTURAL: &[(u32, &str)] = &[
    (417, "the required gates can pass vacuously for a PR that touches only the shipped wasm components"),
    (436, "scheduled monitors run every few hours, not on their cron cadence"),
    (546, "the gz HOLD verdict bounds altitude only by the final sample and a 5 s RMS, and the required gate flies 25 s, not HOLD-P01's 60 s"),
];

#[derive(Debug, Clone, serde::Deserialize)]
struct Run {
    #[serde(default)]
    id: u64,
    status: String,
    #[serde(default)]
    conclusion: Option<String>,
    created_at: String,
    html_url: String,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct Job {
    name: String,
    #[serde(default)]
    conclusion: Option<String>,
    #[serde(default)]
    steps: Vec<JobStep>,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct JobStep {
    name: String,
    #[serde(default)]
    conclusion: Option<String>,
}

#[derive(Debug, PartialEq)]
enum State {
    Green,
    Dark(String),
    Unfinished(String),
    NotRun,
    /// The run succeeded but every substantive job/step was skipped by the
    /// workflow's own scoping: it verified nothing on this commit.
    SkippedByScope,
    /// The run succeeded but the probe matched no job/step at all (renamed?).
    /// Not counted as passed: an unrecognised run is not evidence.
    Unrecognised,
}

fn latest(runs: &[Run]) -> Option<&Run> {
    runs.iter().max_by(|a, b| a.created_at.cmp(&b.created_at))
}

/// The latest run of a workflow for the commit decides its state; `jobs` are
/// THAT run's jobs, and `probe` says which of them verify anything.
fn classify(runs: &[Run], jobs: &[Job], probe: Probe) -> (State, Option<String>) {
    let Some(r) = latest(runs) else {
        return (State::NotRun, None);
    };
    let st = match (r.status.as_str(), r.conclusion.as_deref()) {
        ("completed", Some("success")) => substantive(jobs, probe),
        ("completed", Some(c)) => State::Dark(c.to_string()),
        (s, _) => State::Unfinished(s.to_string()),
    };
    (st, Some(r.html_url.clone()))
}

/// For a SUCCESSFUL run: did its substantive part run? Any non-success,
/// non-skipped conclusion is dark; none matched is unrecognised; all skipped
/// is skipped-by-scope; otherwise at least one ran and none failed.
fn substantive(jobs: &[Job], probe: Probe) -> State {
    let units: Vec<(&str, Option<&str>)> = match probe {
        Probe::AnyJob => jobs.iter().map(|j| (j.name.as_str(), j.conclusion.as_deref())).collect(),
        Probe::JobPrefix(p) => jobs
            .iter()
            .filter(|j| j.name.starts_with(p))
            .map(|j| (j.name.as_str(), j.conclusion.as_deref()))
            .collect(),
        Probe::Step { job, step } => jobs
            .iter()
            .filter(|j| j.name == job)
            .flat_map(|j| j.steps.iter())
            .filter(|s| s.name == step)
            .map(|s| (s.name.as_str(), s.conclusion.as_deref()))
            .collect(),
    };
    if let Some((name, c)) = units.iter().find(|(_, c)| !matches!(c, Some("success") | Some("skipped"))) {
        return State::Dark(format!("{} in {name}", c.unwrap_or("no conclusion")));
    }
    if units.is_empty() {
        State::Unrecognised
    } else if units.iter().all(|(_, c)| *c == Some("skipped")) {
        State::SkippedByScope
    } else {
        State::Green
    }
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
        (State::SkippedByScope, _) => link("⚪ skipped by scope on this commit — the run passed but verified nothing here"),
        (State::Unrecognised, _) => link("⚪ ran, but no recognised verification job/step — not counted as passed"),
        (State::NotRun, Trigger::PullRequestOnly) => "⚪ runs on pull requests only — no run on this commit".into(),
        (State::NotRun, Trigger::PullRequestAndNightly) => "⚪ runs per-PR + nightly on main — no run on this exact commit".into(),
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
    // A run that verified nothing counts against the headline whatever its
    // trigger: it is not a pass, so "ran and passed" must not be printed.
    let hollow = rows.iter().filter(|(_, s, _)| matches!(s, State::SkippedByScope | State::Unrecognised)).count();
    let mut out = format!("## Verification tracks at `{}`\n\n", &sha[..sha.len().min(12)]);
    out += "*Generated from the observed CI conclusions on the released commit by `scripts/verification-tracks.rs` \
(SWREQ-RELAY-VGATE-P04) — not written by hand.*\n\n";
    if dark == 0 && missing == 0 && hollow == 0 {
        out += "Every per-commit track ran and passed on this commit.\n\n";
    } else {
        out += &format!(
            "**{dark} track(s) DARK and {missing} per-commit track(s) not run or unfinished on this commit.** \
A dark track verified nothing for this release; treat its claims accordingly.\n\n"
        );
        if hollow > 0 {
            out += &format!(
                "**{hollow} track(s) ran but verified nothing on this commit** (every verifying job or step \
skipped by the workflow's own scoping, or none recognised). A green run is not a verification.\n\n"
            );
        }
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

/// Every job of a run, all pages. `--paginate` without `--jq` emits one JSON
/// document per page back to back, so flatten with `--jq` to one job per line.
fn run_jobs(repo: &str, run_id: u64) -> Result<Vec<Job>> {
    if run_id == 0 {
        bail!("workflow run without an id");
    }
    let path = format!("repos/{repo}/actions/runs/{run_id}/jobs?per_page=100");
    let o = std::process::Command::new("gh")
        .args(["api", "--paginate", &path, "--jq", ".jobs[]"])
        .output()
        .context("running gh")?;
    if !o.status.success() {
        bail!("gh api {path}: {}", String::from_utf8_lossy(&o.stderr).trim());
    }
    serde_json::Deserializer::from_slice(&o.stdout)
        .into_iter::<Job>()
        .collect::<Result<Vec<_>, _>>()
        .with_context(|| format!("parsing jobs of run {run_id}"))
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
        // Only the run classify() picks matters, and only once it succeeded.
        let jobs = match latest(&runs) {
            Some(r) if r.status == "completed" && r.conclusion.as_deref() == Some("success") => run_jobs(&repo, r.id)?,
            _ => Vec::new(),
        };
        let (s, u) = classify(&runs, &jobs, t.probe);
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
        Run { id: 1, status: status.into(), conclusion: conclusion.map(Into::into), created_at: at.into(), html_url: format!("https://x/{at}") }
    }

    #[test]
    fn the_latest_run_decides_and_failure_is_dark() {
        let runs = [run_of("completed", Some("success"), "2026-09-17T01:00:00Z"), run_of("completed", Some("failure"), "2026-09-17T02:00:00Z")];
        assert_eq!(classify(&runs, &[], Probe::AnyJob).0, State::Dark("failure".into()));
        assert_eq!(classify(&[], &[], Probe::AnyJob).0, State::NotRun);
        assert_eq!(classify(&[run_of("in_progress", None, "t")], &[], Probe::AnyJob).0, State::Unfinished("in_progress".into()));
    }

    // ── REVIEW-140 finding H: a run's own `success` is not a verification ──

    fn job(name: &str, conclusion: &str) -> Job {
        Job { name: name.into(), conclusion: Some(conclusion.into()), steps: Vec::new() }
    }
    fn ok_run() -> [Run; 1] {
        [run_of("completed", Some("success"), "2026-11-05T10:22:54Z")]
    }
    const KANI: Probe = Probe::JobPrefix("Kani (");
    fn gate_job(sweep: &str) -> Job {
        Job {
            name: "Verification gate (rivet-driven)".into(),
            conclusion: Some("success".into()),
            steps: [("Scope check (does this PR need the full sweep?)", "success"), ("Run falcon verification gate", sweep)]
                .iter()
                .map(|(n, c)| JobStep { name: (*n).into(), conclusion: Some((*c).into()) })
                .collect(),
        }
    }
    const GATE: Probe = Probe::Step { job: "Verification gate (rivet-driven)", step: "Run falcon verification gate" };

    #[test]
    fn kani_with_every_matrix_leg_skipped_by_scope_is_skipped_not_passed() {
        // Run 42010229156 (merge_group, 725f146), verbatim job list.
        let jobs = [
            job("Detect engine changes", "success"),
            job("Kani (${{ matrix.engine }})", "skipped"),
            job("Kani gate", "success"),
        ];
        assert_eq!(classify(&ok_run(), &jobs, KANI).0, State::SkippedByScope);
    }

    #[test]
    fn kani_with_one_leg_run_and_none_failed_is_passed() {
        let jobs = [
            job("Detect engine changes", "success"),
            job("Kani (relay-ccsds)", "success"),
            job("Kani (relay-sec)", "skipped"),
            job("Kani gate", "success"),
        ];
        assert_eq!(classify(&ok_run(), &jobs, KANI).0, State::Green);
    }

    #[test]
    fn any_failed_substantive_job_is_dark_even_if_the_run_says_success() {
        let jobs = [job("Detect engine changes", "success"), job("Kani (relay-ccsds)", "success"), job("Kani (relay-sec)", "failure"), job("Kani gate", "success")];
        assert!(matches!(classify(&ok_run(), &jobs, KANI).0, State::Dark(_)), "{:?}", classify(&ok_run(), &jobs, KANI).0);
        assert_eq!(classify(&ok_run(), &[job("Verus proofs", "cancelled")], Probe::AnyJob).0, State::Dark("cancelled in Verus proofs".into()));
    }

    #[test]
    fn a_fast_passed_verification_gate_is_skipped_and_a_sweep_is_passed() {
        // Run 42010229350: one job, success, sweep step `skipped`.
        assert_eq!(classify(&ok_run(), &[gate_job("skipped")], GATE).0, State::SkippedByScope);
        assert_eq!(classify(&ok_run(), &[gate_job("success")], GATE).0, State::Green);
    }

    #[test]
    fn a_probe_that_matches_nothing_is_not_counted_as_passed() {
        // A renamed job/step must not silently turn back into "passed".
        assert_eq!(classify(&ok_run(), &[job("Kani gate", "success")], KANI).0, State::Unrecognised);
        assert_eq!(classify(&ok_run(), &[], Probe::AnyJob).0, State::Unrecognised);
        let mut renamed = gate_job("success");
        renamed.steps[1].name = "Run the sweep".into();
        assert_eq!(classify(&ok_run(), &[renamed], GATE).0, State::Unrecognised);
    }

    #[test]
    fn plain_single_job_tracks_still_pass() {
        assert_eq!(classify(&ok_run(), &[job("Verus proofs", "success")], Probe::AnyJob).0, State::Green);
        assert_eq!(classify(&ok_run(), &[job("Verus proofs", "skipped")], Probe::AnyJob).0, State::SkippedByScope);
    }

    #[test]
    fn a_skipped_track_suppresses_the_all_passed_headline() {
        let rows: Vec<(&Track, State, Option<String>)> = TRACKS
            .iter()
            .map(|t| {
                let s = match t.workflow {
                    "kani.yml" => State::SkippedByScope,
                    "soak.yml" => State::NotRun,
                    _ => State::Green,
                };
                (t, s, None)
            })
            .collect();
        let md = render("725f1467d198", &rows, &[]);
        assert!(!md.contains("Every per-commit track ran and passed"), "{md}");
        assert!(md.contains("| Kani bounded model checking | `kani.yml` | ⚪ skipped by scope on this commit"), "{md}");
        assert!(md.contains("1 track(s) ran but verified nothing"), "{md}");
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
        // #554 reclassified verification-gate.yml PR-only -> PR + nightly
        // (#410 retired) and left this assertion naming the old wording, so the
        // suite failed on main from 2026-10-09 with nothing running it (VGATE-005
        // is not tagged `falcon`). The property is unchanged: a track that is
        // not per-commit is reported as such, never as a miss.
        assert!(md.contains("runs per-PR + nightly on main"), "PR+nightly is not reported as a miss: {md}");
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
