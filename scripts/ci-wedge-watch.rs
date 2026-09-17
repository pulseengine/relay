#!/usr/bin/env rust-script
//! Is any CI job stuck `in_progress` far past its normal duration?
//!
//! WHY THIS EXISTS (#429). On 2026-09-16 `Kani (relay-mix-quad)` wedged three
//! times — 52, 96 and 105 minutes against a baseline under 4 — each holding a
//! runner and blocking a PR until someone looked at the job's DURATION rather
//! than its status. The same leg completed normally in about three minutes on
//! four other runs that day, two of them while a wedged instance was still
//! running, so this is a per-instance hang, not a slow workload.
//!
//! The fleet monitor could not see it. It looks for runs QUEUED too long
//! (starvation: work with nowhere to run). A wedged job is the opposite shape:
//! it HAS a runner and reports itself healthily `in_progress`.
//!
//! The comparison is against each job's OWN history: a job is wedged when it
//! has been running longer than `multiplier` × the median duration of its last
//! successful completions, and longer than an absolute floor. A job with too
//! little history is reported as having no baseline — never guessed at.
//!
//! RUN STATUS IS NOT JOB STATUS. A matrix run whose other legs are still queued
//! reports the whole run as `queued` even while one of its jobs runs (observed
//! on #430's CI run the same day: run `queued`, `Format` already `completed`).
//! Querying only `in_progress` runs would miss exactly the wedged-Kani-leg
//! case, so jobs are enumerated from both queued and in-progress runs.
//!
//! HISTORY COMES FROM SUCCESSFUL RUNS, NOT COMPLETED ONES. `cancel-in-progress`
//! makes most completed runs cancelled ones. Measured on relay, 2026-09-16: the
//! last 5 *completed* Kani runs held ONE successful `relay-mix-quad` leg — below
//! the sample minimum, so the very wedge this exists for would have been
//! reported as "no baseline" and never alarmed. The last 5 *successful* runs
//! held 3 (Tier-B path filtering skips unchanged crates' legs, so not 5); the
//! default window is therefore 8 successful runs, not 5.
//!
//! Exit codes — "could not evaluate" is NOT a pass:
//!   0  evaluated, nothing wedged
//!   1  evaluated, at least one job wedged
//!   2  could not evaluate (a GitHub API call failed)
//!
//! It REPORTS. It cancels nothing: whether a wedged job is re-run, or its
//! runner restarted, is the operator's call.
//!
//! Usage:
//!   scripts/ci-wedge-watch.rs --repo pulseengine/relay
//!   scripts/ci-wedge-watch.rs --format json
//!   scripts/ci-wedge-watch.rs --summary-md s.md --alarm-tsv rows.tsv   # the workflow
//!   scripts/ci-wedge-watch.rs --multiplier 5 --floor-min 10 --min-samples 3
//!   rust-script --test scripts/ci-wedge-watch.rs     # replays #429's data
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde = { version = "1", features = ["derive"] }
//! serde_json = "1"
//! ```

use anyhow::{bail, Context, Result};
use std::collections::BTreeMap;
use std::process::ExitCode;

#[derive(Debug, Clone, serde::Deserialize)]
struct Run {
    id: u64,
    workflow_id: u64,
    name: String,
}

#[derive(Debug, Clone, serde::Deserialize)]
struct Job {
    name: String,
    status: String,
    #[serde(default)]
    conclusion: Option<String>,
    #[serde(default)]
    started_at: Option<String>,
    #[serde(default)]
    completed_at: Option<String>,
    #[serde(default)]
    html_url: Option<String>,
    #[serde(default)]
    runner_name: Option<String>,
}

/// A job that is running now.
#[derive(Debug, Clone)]
struct Running {
    workflow: String,
    job: String,
    started: i64,
    runner: String,
    url: String,
}

#[derive(Debug, Clone, Copy)]
struct Policy {
    multiplier: f64,
    floor_s: i64,
    min_samples: usize,
}

#[derive(Debug, serde::Serialize)]
struct Wedge {
    workflow: String,
    job: String,
    runner: String,
    job_url: String,
    elapsed_s: i64,
    median_s: i64,
    threshold_s: i64,
    samples: usize,
}

#[derive(Debug, serde::Serialize)]
struct NoBaseline {
    workflow: String,
    job: String,
    runner: String,
    job_url: String,
    elapsed_s: i64,
    samples: usize,
}

#[derive(Debug, serde::Serialize)]
struct Report {
    running: usize,
    wedged: Vec<Wedge>,
    no_baseline: Vec<NoBaseline>,
}

/// Seconds since the epoch for GitHub's `YYYY-MM-DDTHH:MM:SSZ` timestamps.
fn parse_utc(s: &str) -> Option<i64> {
    let b = s.as_bytes();
    if b.len() != 20 || b[4] != b'-' || b[7] != b'-' || b[10] != b'T' || b[19] != b'Z' {
        return None;
    }
    let num = |r: std::ops::Range<usize>| s.get(r)?.parse::<i64>().ok();
    let (y, m, d) = (num(0..4)?, num(5..7)?, num(8..10)?);
    let (hh, mm, ss) = (num(11..13)?, num(14..16)?, num(17..19)?);
    // Days from civil (Howard Hinnant's algorithm), proleptic Gregorian.
    let y = if m <= 2 { y - 1 } else { y };
    let era = y.div_euclid(400);
    let yoe = y - era * 400;
    let doy = (153 * (if m > 2 { m - 3 } else { m + 9 }) + 2) / 5 + d - 1;
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    let days = era * 146_097 + doe - 719_468;
    Some(days * 86_400 + hh * 3_600 + mm * 60 + ss)
}

fn median(xs: &[i64]) -> Option<i64> {
    if xs.is_empty() {
        return None;
    }
    let mut v = xs.to_vec();
    v.sort_unstable();
    let n = v.len();
    Some(if n % 2 == 1 { v[n / 2] } else { (v[n / 2 - 1] + v[n / 2]) / 2 })
}

/// The pure decision: which running jobs are wedged, and which cannot be
/// judged. `history` is keyed by (workflow name, job name) and holds the
/// durations, in seconds, of recent successful completions.
fn find_wedges(
    now: i64,
    running: &[Running],
    history: &BTreeMap<(String, String), Vec<i64>>,
    p: Policy,
) -> Report {
    let mut wedged = Vec::new();
    let mut no_baseline = Vec::new();
    for r in running {
        let elapsed_s = now - r.started;
        let samples = history
            .get(&(r.workflow.clone(), r.job.clone()))
            .map(Vec::as_slice)
            .unwrap_or(&[]);
        if samples.len() < p.min_samples {
            no_baseline.push(NoBaseline {
                workflow: r.workflow.clone(),
                job: r.job.clone(),
                runner: r.runner.clone(),
                job_url: r.url.clone(),
                elapsed_s,
                samples: samples.len(),
            });
            continue;
        }
        let median_s = median(samples).expect("non-empty by the check above");
        let threshold_s = ((median_s as f64 * p.multiplier).ceil() as i64).max(p.floor_s);
        if elapsed_s > threshold_s {
            wedged.push(Wedge {
                workflow: r.workflow.clone(),
                job: r.job.clone(),
                runner: r.runner.clone(),
                job_url: r.url.clone(),
                elapsed_s,
                median_s,
                threshold_s,
                samples: samples.len(),
            });
        }
    }
    Report { running: running.len(), wedged, no_baseline }
}

/// `gh api --paginate <path> --jq '<sel> | tojson'`, one value per line.
/// Any failure is an error: an empty answer from a failed call must never be
/// read as "nothing is running".
fn gh_values<T: serde::de::DeserializeOwned>(path: &str, sel: &str, paginate: bool) -> Result<Vec<T>> {
    let mut cmd = std::process::Command::new("gh");
    cmd.arg("api");
    if paginate {
        cmd.arg("--paginate");
    }
    cmd.arg(path).arg("--jq").arg(format!("{sel} | tojson"));
    let out = cmd.output().with_context(|| format!("running gh api {path}"))?;
    if !out.status.success() {
        bail!("gh api {path} failed: {}", String::from_utf8_lossy(&out.stderr).trim());
    }
    String::from_utf8(out.stdout)?
        .lines()
        .filter(|l| !l.trim().is_empty())
        .map(|l| serde_json::from_str(l).with_context(|| format!("parsing {path}")))
        .collect()
}

fn collect(repo: &str, history_runs: usize) -> Result<(Vec<Running>, BTreeMap<(String, String), Vec<i64>>)> {
    let mut active: Vec<Run> = Vec::new();
    for status in ["in_progress", "queued"] {
        active.extend(gh_values::<Run>(
            &format!("repos/{repo}/actions/runs?status={status}&per_page=100"),
            ".workflow_runs[]",
            true,
        )?);
    }

    let mut running = Vec::new();
    let mut workflows: BTreeMap<u64, String> = BTreeMap::new();
    for run in &active {
        let jobs: Vec<Job> =
            gh_values(&format!("repos/{repo}/actions/runs/{}/jobs?per_page=100", run.id), ".jobs[]", true)?;
        for j in jobs.into_iter().filter(|j| j.status == "in_progress") {
            let Some(started) = j.started_at.as_deref().and_then(parse_utc) else { continue };
            workflows.insert(run.workflow_id, run.name.clone());
            running.push(Running {
                workflow: run.name.clone(),
                job: j.name,
                started,
                runner: j.runner_name.unwrap_or_else(|| "?".into()),
                url: j.html_url.unwrap_or_default(),
            });
        }
    }

    // History only for workflows that have something running — the API budget
    // is per repository and shared with every other workflow.
    let mut history: BTreeMap<(String, String), Vec<i64>> = BTreeMap::new();
    for (wid, wname) in &workflows {
        let runs: Vec<Run> = gh_values(
            &format!("repos/{repo}/actions/workflows/{wid}/runs?status=success&per_page={history_runs}"),
            ".workflow_runs[]",
            false,
        )?;
        for run in runs {
            let jobs: Vec<Job> =
                gh_values(&format!("repos/{repo}/actions/runs/{}/jobs?per_page=100", run.id), ".jobs[]", true)?;
            for j in jobs {
                if j.conclusion.as_deref() != Some("success") {
                    continue;
                }
                let (Some(s), Some(e)) = (
                    j.started_at.as_deref().and_then(parse_utc),
                    j.completed_at.as_deref().and_then(parse_utc),
                ) else {
                    continue;
                };
                if e >= s {
                    history.entry((wname.clone(), j.name)).or_default().push(e - s);
                }
            }
        }
    }
    Ok((running, history))
}

fn minutes(s: i64) -> String {
    format!("{:.1}", s as f64 / 60.0)
}

/// The job-summary section. Rendered here rather than by `jq` in the workflow:
/// the job runs on self-hosted `light`, where `jq` is not known to be installed.
fn summary_markdown(r: &Report, p: Policy) -> String {
    let mut s = String::from("## Wedged jobs\n\n");
    if r.wedged.is_empty() {
        s += &format!("### 🟢 No job running past {}x its median duration\n\n", p.multiplier);
        s += &format!("{} job(s) running.\n", r.running);
    } else {
        s += &format!(
            "### 🔴 {} job(s) wedged — running past {}x their median\n\n",
            r.wedged.len(),
            p.multiplier
        );
        for w in &r.wedged {
            s += &format!(
                "- `{} / {}` on `{}` — running **{} min**, median {} min over {} runs ([job]({}))\n",
                w.workflow,
                w.job,
                w.runner,
                w.elapsed_s / 60,
                minutes(w.median_s),
                w.samples,
                w.job_url
            );
        }
    }
    if !r.no_baseline.is_empty() {
        s += "\nRunning without enough successful history to judge (reported, never guessed):\n\n";
        for n in &r.no_baseline {
            s += &format!(
                "- `{} / {}` on `{}` — {} min, {} sample(s)\n",
                n.workflow,
                n.job,
                n.runner,
                n.elapsed_s / 60,
                n.samples
            );
        }
    }
    s
}

/// One line per wedged job: `<job url>\t<markdown row>`. The URL comes first so
/// the alarm step can skip jobs an open issue already names.
fn alarm_rows(r: &Report) -> String {
    let clean = |x: &str| x.replace(['\t', '\n'], " ");
    r.wedged
        .iter()
        .map(|w| {
            format!(
                "{}\t- `{} / {}` on `{}` — running {} min against a median of {} min over {} successful runs ([job]({}))\n",
                clean(&w.job_url),
                clean(&w.workflow),
                clean(&w.job),
                clean(&w.runner),
                w.elapsed_s / 60,
                minutes(w.median_s),
                w.samples,
                clean(&w.job_url)
            )
        })
        .collect()
}

fn arg<'a>(args: &'a [String], key: &str) -> Option<&'a str> {
    args.iter().position(|a| a == key).and_then(|i| args.get(i + 1)).map(String::as_str)
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let parse = |key: &str, default: f64| -> f64 {
        arg(&args, key).map(|v| v.parse().unwrap_or(default)).unwrap_or(default)
    };
    let repo = arg(&args, "--repo")
        .map(str::to_owned)
        .or_else(|| std::env::var("GITHUB_REPOSITORY").ok())
        .unwrap_or_else(|| "pulseengine/relay".into());
    let policy = Policy {
        multiplier: parse("--multiplier", 5.0),
        floor_s: (parse("--floor-min", 10.0) * 60.0) as i64,
        min_samples: parse("--min-samples", 3.0) as usize,
    };
    let history_runs = parse("--history-runs", 8.0) as usize;
    let json = arg(&args, "--format") == Some("json");

    let (running, history) = match collect(&repo, history_runs) {
        Ok(v) => v,
        Err(e) => {
            eprintln!("ci-wedge-watch: could not evaluate {repo}: {e:#}");
            return ExitCode::from(2);
        }
    };
    let now = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(0);
    let report = find_wedges(now, &running, &history, policy);

    // Files for the workflow. Failing to write one is failing to report.
    for (key, body) in [
        ("--summary-md", summary_markdown(&report, policy)),
        ("--alarm-tsv", alarm_rows(&report)),
    ] {
        if let Some(path) = arg(&args, key) {
            if let Err(e) = std::fs::write(path, body) {
                eprintln!("ci-wedge-watch: could not write {key} {path}: {e}");
                return ExitCode::from(2);
            }
        }
    }

    if json {
        println!("{}", serde_json::to_string_pretty(&report).expect("report serializes"));
    } else {
        println!(
            "{repo}: {} job(s) running, {} wedged, {} without a baseline ({}x median, floor {} min, >= {} samples)",
            report.running,
            report.wedged.len(),
            report.no_baseline.len(),
            policy.multiplier,
            minutes(policy.floor_s),
            policy.min_samples
        );
        for w in &report.wedged {
            println!(
                "  WEDGED  {} / {} on {} — {} min, median {} min over {} runs (threshold {} min)  {}",
                w.workflow,
                w.job,
                w.runner,
                minutes(w.elapsed_s),
                minutes(w.median_s),
                w.samples,
                minutes(w.threshold_s),
                w.job_url
            );
        }
        for n in &report.no_baseline {
            println!(
                "  no baseline  {} / {} on {} — {} min, {} successful sample(s)",
                n.workflow,
                n.job,
                n.runner,
                minutes(n.elapsed_s),
                n.samples
            );
        }
    }
    if report.wedged.is_empty() {
        ExitCode::SUCCESS
    } else {
        ExitCode::from(1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const POLICY: Policy = Policy { multiplier: 5.0, floor_s: 600, min_samples: 3 };

    fn job(name: &str, started: i64) -> Running {
        Running {
            workflow: "Kani".into(),
            job: name.into(),
            started,
            runner: "pulseengine-ci-01-7".into(),
            url: format!("https://example.invalid/{name}"),
        }
    }

    fn hist(name: &str, d: &[i64]) -> BTreeMap<(String, String), Vec<i64>> {
        BTreeMap::from([(("Kani".to_string(), name.to_string()), d.to_vec())])
    }

    // The four normal completions of `Kani (relay-mix-quad)` measured on
    // 2026-09-16 (#429): 2m48s, 2m50s, 2m53s, 3m21s.
    const MIX_QUAD_OK: [i64; 4] = [168, 170, 173, 201];

    #[test]
    fn parses_github_timestamps() {
        assert_eq!(parse_utc("1970-01-01T00:00:00Z"), Some(0));
        // Cross-checked with `date -u -j -f %Y-%m-%dT%H:%M:%SZ 2026-09-16T15:17:58Z +%s`.
        assert_eq!(parse_utc("2026-09-16T15:17:58Z"), Some(1_789_571_878));
        assert_eq!(parse_utc("2024-02-29T23:59:59Z"), Some(1_709_251_199));
        assert_eq!(parse_utc("2026-09-16 15:17:58"), None);
        assert_eq!(parse_utc(""), None);
    }

    #[test]
    fn median_of_odd_and_even_counts() {
        assert_eq!(median(&[]), None);
        assert_eq!(median(&[5, 1, 3]), Some(3));
        assert_eq!(median(&MIX_QUAD_OK), Some(171));
    }

    #[test]
    fn the_429_wedge_is_caught_within_fifteen_minutes() {
        let start = parse_utc("2026-09-16T15:17:58Z").unwrap();
        let h = hist("Kani (relay-mix-quad)", &MIX_QUAD_OK);
        let running = [job("Kani (relay-mix-quad)", start)];

        // Threshold is 5 x 171 s = 855 s. At 14 minutes it is not yet flagged...
        let at14 = find_wedges(start + 14 * 60, &running, &h, POLICY);
        assert!(at14.wedged.is_empty());
        // ...at 15 it is, 90 minutes before a human cancelled the real one.
        let at15 = find_wedges(start + 15 * 60, &running, &h, POLICY);
        assert_eq!(at15.wedged.len(), 1);
        assert_eq!(at15.wedged[0].median_s, 171);
        assert_eq!(at15.wedged[0].threshold_s, 855);
        // And the 105-minute instance is, of course, flagged.
        assert_eq!(find_wedges(start + 105 * 60, &running, &h, POLICY).wedged.len(), 1);
    }

    #[test]
    fn a_slow_leg_running_normally_is_not_flagged() {
        // `Kani (relay-notch)`'s real successful durations on 2026-09-16: 888,
        // 816, 656 s (median 13.6 min). Watching it sit at 18.8 min, a human
        // guessed "wedged" from a remembered two-minute sighting; the detector,
        // reading the history, did not. Per-job baselines exist for this case.
        let h = hist("Kani (relay-notch)", &[888, 816, 656]);
        let running = [job("Kani (relay-notch)", 0)];
        let r = find_wedges(1_128, &running, &h, POLICY);
        assert!(r.wedged.is_empty());
        assert!(r.no_baseline.is_empty());
        // Its threshold is 5 x 816 s = 68 min, not a global number.
        assert_eq!(find_wedges(4_081, &running, &h, POLICY).wedged[0].threshold_s, 4_080);
    }

    #[test]
    fn the_floor_keeps_short_jobs_from_alarming_on_jitter() {
        // A 20-second job taking 5 minutes is 15x its median but not a wedge.
        let h = hist("Format", &[20, 21, 19]);
        let running = [job("Format", 0)];
        assert!(find_wedges(5 * 60, &running, &h, POLICY).wedged.is_empty());
        assert_eq!(find_wedges(11 * 60, &running, &h, POLICY).wedged.len(), 1);
    }

    #[test]
    fn thin_history_is_reported_not_guessed() {
        let h = hist("Kani (relay-new)", &[170, 175]);
        let r = find_wedges(10_000, &[job("Kani (relay-new)", 0)], &h, POLICY);
        assert!(r.wedged.is_empty());
        assert_eq!(r.no_baseline.len(), 1);
        assert_eq!(r.no_baseline[0].samples, 2);
        // No history at all is the same case, not a crash.
        let r = find_wedges(10_000, &[job("Kani (relay-other)", 0)], &BTreeMap::new(), POLICY);
        assert_eq!(r.no_baseline.len(), 1);
    }

    #[test]
    fn the_workflow_files_render_without_jq() {
        let start = parse_utc("2026-09-16T15:17:58Z").unwrap();
        let h = hist("Kani (relay-mix-quad)", &MIX_QUAD_OK);
        let running = [job("Kani (relay-mix-quad)", start), job("Kani (relay-new)", start)];
        let r = find_wedges(start + 105 * 60, &running, &h, POLICY);

        let md = summary_markdown(&r, POLICY);
        assert!(md.contains("### 🔴 1 job(s) wedged"), "{md}");
        assert!(md.contains("`Kani / Kani (relay-mix-quad)`"), "{md}");
        assert!(md.contains("running **105 min**, median 2.9 min over 4 runs"), "{md}");
        assert!(md.contains("`Kani / Kani (relay-new)`"), "no-baseline job listed: {md}");

        let rows = alarm_rows(&r);
        assert_eq!(rows.lines().count(), 1, "only the wedged job alarms: {rows}");
        let (url, row) = rows.lines().next().unwrap().split_once('\t').unwrap();
        assert_eq!(url, "https://example.invalid/Kani (relay-mix-quad)");
        assert!(row.starts_with("- `Kani / Kani (relay-mix-quad)`"), "{row}");

        let quiet = find_wedges(start + 60, &running[..1], &h, POLICY);
        assert!(summary_markdown(&quiet, POLICY).contains("### 🟢"));
        assert!(alarm_rows(&quiet).is_empty());
    }

    #[test]
    fn history_is_per_workflow_and_job_not_per_job_name() {
        // Two workflows can both have a job named "build"; one's history must
        // not judge the other.
        let h = BTreeMap::from([(("Bazel".to_string(), "build".to_string()), vec![3_000, 3_100, 3_200])]);
        let mut r = job("build", 0);
        r.workflow = "Kani".into();
        let rep = find_wedges(20 * 60, &[r], &h, POLICY);
        assert!(rep.wedged.is_empty());
        assert_eq!(rep.no_baseline.len(), 1);
    }
}
