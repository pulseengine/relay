#!/usr/bin/env rust-script
//! Is a red check a real failure, or a job that never ran?
//!
//! WHY THIS EXISTS (#508). Measured on relay over 2026-10-01/02: of the failed
//! jobs inspected across #501, #504, #507 and #513, **nineteen were not real
//! failures and none were**. They occupied a runner, reported
//! `conclusion=failure`, and produced nothing — no steps, no log blob. Every one
//! cleared on `gh run rerun --failed` with a healthy fleet throughout.
//!
//! THE TWO FAILURE MODES LOOK IDENTICAL IN EVERY SUMMARY VIEW. `gh pr checks`
//! prints `fail`; the jobs API reports no failing step; the log 404s. So the cost
//! lands twice, and the second way is the dangerous one:
//!
//!   - chasing a phantom wastes a session — one of these was initially suspected
//!     to be a Monte-Carlo regression landing on main (it was not);
//!   - and once the rate is known to be high, the temptation is to assume `fail`
//!     means phantom and re-run without looking, which is how a genuine
//!     regression goes green by luck.
//!
//! The discriminator therefore has to be MECHANICAL, not a habit. That is this
//! tool's whole reason to exist, and it is why it exits non-zero on a REAL
//! finding: so it can gate rather than merely inform.
//!
//! FOUR VERDICTS, because two is not enough — learned the hard way:
//!
//!   REAL        has steps and a readable log. Read it; something is wrong.
//!   LOST        `steps == []` AND no log blob. It never ran. Re-run it.
//!   DERIVED     a ROLL-UP that failed only because its upstreams were LOST.
//!               It HAS steps and a log, so a two-valued rule calls it REAL and
//!               sends you into the wrong subsystem. Measured: one lost
//!               `Kani (relay-mavlink)` turned `Kani gate` red, and the gate's
//!               log says only `KANI_RESULT: failure`.
//!   SUPERSEDED  `conclusion == cancelled`, which `gh pr checks` ALSO renders as
//!               `fail`. Usually a newer run for the same ref. Of 19 classified,
//!               most were these — so raw FAIL counts in this repo systematically
//!               overstate how much is wrong.
//!
//! Complementary to `ci-wedge-watch.rs`, which is the opposite shape: a job that
//! HAS a runner and reports itself healthily `in_progress` while hung. This one
//! is about jobs that reported failure without running at all.
//!
//! Usage:
//!   scripts/ci-triage.rs 501 513          classify those PRs' failing checks
//!   scripts/ci-triage.rs --format json 501
//!   scripts/ci-triage.rs --gate 501       exit 1 only if something is REAL
//!
//! Exit codes:
//!   0  nothing REAL (or nothing failing at all)
//!   1  at least one REAL finding — or, without --gate, any finding at all
//!   2  usage error, or `gh` missing
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_json = "1"
//! ```

use anyhow::{bail, Context, Result};
use std::collections::BTreeMap;
use std::process::Command;

const REPO: &str = "pulseengine/relay";

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Verdict {
    Real,
    Lost,
    Derived,
    Superseded,
}

impl Verdict {
    fn label(self) -> &'static str {
        match self {
            Verdict::Real => "REAL",
            Verdict::Lost => "LOST",
            Verdict::Derived => "DERIVED",
            Verdict::Superseded => "SUPERSEDED",
        }
    }
    fn advice(self) -> &'static str {
        match self {
            Verdict::Real => "read the log; something is wrong",
            Verdict::Lost => "never ran (no steps, no log) — re-run it",
            Verdict::Derived => "roll-up of benign (lost/cancelled) upstreams — fix those, not this",
            Verdict::Superseded => "cancelled, usually by a newer run for the same ref",
        }
    }
}

struct Finding {
    check: String,
    job: u64,
    run: u64,
    steps: usize,
    conclusion: String,
    log_missing: bool,
    verdict: Verdict,
}

fn gh(args: &[&str]) -> Result<String> {
    let out = Command::new("gh").args(args).output().with_context(|| {
        "could not execute `gh` — it must be on PATH. A missing tool must fail \
         loudly: the fleet monitor ran `gh api ... || true` on boxes without it \
         for months and every answer was an empty result (#436)."
    })?;
    // `gh api` on a missing log blob exits non-zero with an XML error body; that
    // is a SIGNAL here, not an error, so stdout is returned either way.
    Ok(String::from_utf8_lossy(&out.stdout).into_owned())
}

/// Does this job's log exist at all?
///
/// A job that died before its first step never opens one, and the API answers
/// with an Azure `BlobNotFound` document rather than a 404.
fn log_missing(job: u64) -> Result<(bool, String)> {
    let body = gh(&[
        "api",
        "--allow-escape-sequences",
        &format!("/repos/{REPO}/actions/jobs/{job}/logs"),
    ])?;
    // MISSINGNESS is decided from the HEAD (the error document is the whole
    // body), but the text handed back is the TAIL as well — the first version
    // returned only the first 4000 chars, which on a real job is runner
    // preamble ("Current runner version", "Prepare all required actions"). A
    // roll-up's verdict line sits at the END of its log, so `looks_like_rollup`
    // never saw it and every roll-up classified as REAL. Found by running this
    // tool on #515/#501 and disbelieving its own answer.
    let head: String = body.chars().take(4000).collect();
    let missing = head.contains("BlobNotFound") || head.trim().is_empty();
    Ok((missing, body))
}

/// A log that contains ONLY an aggregate verdict is a roll-up: it reports that
/// something else failed, not that anything failed here.
fn looks_like_rollup(log: &str) -> bool {
    // MATCH THE SHAPE, NOT THE VALUE. The first version looked for
    // "matrix result: failure" and friends, and missed the case that actually
    // occurred: #501's Kani gate logged
    //     KANI_RESULT: cancelled
    //     Kani matrix result: cancelled
    // — a gate that rolled up a CANCELLED matrix, not a failed one. Requiring
    // the word "failure" meant the marker never matched and the gate was
    // reported REAL, which would have sent a reader into 46 Kani harnesses
    // where every single one had passed.
    //
    // So: recognise that the log's verdict IS an aggregate, then let the
    // upstream resolution decide what it aggregates.
    let agg = ["matrix result:", "_RESULT:"];
    agg.iter().any(|m| log.contains(m))
}

fn classify_pr(pr: &str) -> Result<Vec<Finding>> {
    let checks = gh(&["pr", "checks", pr])?;
    let mut findings = Vec::new();
    // Cache per-run job inventories so a roll-up can be resolved against its
    // upstreams without re-fetching the whole run for each one.
    let mut run_jobs: BTreeMap<u64, Vec<(String, usize, bool)>> = BTreeMap::new();

    for line in checks.lines() {
        let cols: Vec<&str> = line.split('\t').collect();
        if cols.len() < 4 || !cols[1].eq_ignore_ascii_case("fail") {
            continue;
        }
        let (check, url) = (cols[0].to_string(), cols[3]);
        let Some(job_s) = url.rsplit("/job/").next() else { continue };
        let Ok(job) = job_s.trim().parse::<u64>() else { continue };

        let j: serde_json::Value =
            serde_json::from_str(&gh(&["api", &format!("/repos/{REPO}/actions/jobs/{job}")])?)
                .with_context(|| format!("job {job} did not return JSON"))?;
        let steps = j["steps"].as_array().map(|a| a.len()).unwrap_or(0);
        let conclusion = j["conclusion"].as_str().unwrap_or("?").to_string();
        let run = j["run_id"].as_u64().unwrap_or(0);
        let (missing, log) = log_missing(job)?;

        let verdict = if conclusion == "cancelled" {
            Verdict::Superseded
        } else if steps == 0 && missing {
            Verdict::Lost
        } else if looks_like_rollup(&log) {
            // Resolve the roll-up against the run it aggregates. DERIVED only if
            // every other failed job in that run never ran; if even one is real,
            // this gate is reporting something true.
            let inv = run_jobs.entry(run).or_insert_with(|| {
                let mut v = Vec::new();
                if let Ok(txt) = gh(&[
                    "api",
                    "--paginate",
                    &format!("/repos/{REPO}/actions/runs/{run}/jobs"),
                ]) {
                    if let Ok(val) = serde_json::from_str::<serde_json::Value>(&txt) {
                        for job_v in val["jobs"].as_array().unwrap_or(&vec![]) {
                            // EVERY NON-SUCCESS MEMBER, not just `failure`.
                            // The first version collected only failures, so a
                            // matrix of 45 successes plus ONE CANCELLED member
                            // left this inventory EMPTY — and an empty inventory
                            // meant "cannot be derived", so the gate that failed
                            // purely because of that cancellation was reported
                            // REAL. Measured on #515: 45 success, 1 cancelled,
                            // 1 failure (the gate itself).
                            let concl = job_v["conclusion"].as_str().unwrap_or("");
                            if concl != "success" && concl != "skipped" {
                                let id = job_v["id"].as_u64().unwrap_or(0);
                                if id == job {
                                    continue; // the roll-up itself
                                }
                                let st = job_v["steps"].as_array().map(|a| a.len()).unwrap_or(0);
                                let miss = log_missing(id).map(|(m, _)| m).unwrap_or(false);
                                // A member is "not a real failure" if it never
                                // ran (LOST) or was cancelled (SUPERSEDED).
                                let benign = concl == "cancelled" || (st == 0 && miss);
                                v.push((
                                    job_v["name"].as_str().unwrap_or("?").to_string(),
                                    st,
                                    benign,
                                ));
                            }
                        }
                    }
                }
                v
            });
            // The third tuple field is now "benign" (lost OR cancelled), so a
            // roll-up is DERIVED when every non-success member it aggregates is
            // benign. `_st` is kept for the report only.
            let upstreams_all_benign =
                !inv.is_empty() && inv.iter().all(|(_, _st, benign)| *benign);
            if upstreams_all_benign {
                Verdict::Derived
            } else {
                Verdict::Real
            }
        } else {
            Verdict::Real
        };

        findings.push(Finding {
            check,
            job,
            run,
            steps,
            conclusion,
            log_missing: missing,
            verdict,
        });
    }
    Ok(findings)
}

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let json = args.iter().any(|a| a == "--format-json" || a == "--format=json");
    let gate = args.iter().any(|a| a == "--gate");
    let prs: Vec<&String> = args.iter().filter(|a| !a.starts_with("--") && a != &"json").collect();
    if prs.is_empty() {
        eprintln!("usage: ci-triage.rs [--gate] [--format=json] <pr-number>...");
        std::process::exit(2);
    }
    if Command::new("gh").arg("--version").output().is_err() {
        eprintln!("ci-triage: `gh` is not on PATH. Refusing to report a clean triage it could not perform.");
        std::process::exit(2);
    }

    let mut all: Vec<(String, Vec<Finding>)> = Vec::new();
    for pr in &prs {
        all.push(((*pr).clone(), classify_pr(pr)?));
    }

    let mut real = 0usize;
    if json {
        print!("{{\"prs\":[");
        for (i, (pr, fs)) in all.iter().enumerate() {
            if i > 0 {
                print!(",");
            }
            print!("{{\"pr\":{pr},\"findings\":[");
            for (k, f) in fs.iter().enumerate() {
                if k > 0 {
                    print!(",");
                }
                print!(
                    "{{\"check\":{:?},\"job\":{},\"run\":{},\"steps\":{},\"conclusion\":{:?},\"log_missing\":{},\"verdict\":{:?}}}",
                    f.check, f.job, f.run, f.steps, f.conclusion, f.log_missing, f.verdict.label()
                );
                if f.verdict == Verdict::Real {
                    real += 1;
                }
            }
            print!("]}}");
        }
        println!("]}}");
    } else {
        for (pr, fs) in &all {
            println!("=== #{pr} ===");
            if fs.is_empty() {
                println!("  no failing checks");
                continue;
            }
            for f in fs {
                println!(
                    "  {:<40} steps={:<3} concl={:<10} log={:<8} -> {} ({})",
                    f.check,
                    f.steps,
                    f.conclusion,
                    if f.log_missing { "missing" } else { "present" },
                    f.verdict.label(),
                    f.verdict.advice()
                );
                if f.verdict == Verdict::Real {
                    real += 1;
                }
            }
            let mut tally: BTreeMap<&str, usize> = BTreeMap::new();
            for f in fs {
                *tally.entry(f.verdict.label()).or_default() += 1;
            }
            let line: Vec<String> = tally.iter().map(|(k, v)| format!("{k}={v}")).collect();
            println!("  SUMMARY: {}", line.join(" "));
        }
        println!();
        if real == 0 {
            println!("NOTHING REAL: every failing check never ran, was superseded, or is a roll-up of those.");
            println!("Re-run the LOST ones; do NOT go looking in the subsystems they name.");
        } else {
            println!("{real} REAL finding(s) — read their logs. Do not re-run and hope.");
        }
    }

    if gate {
        if real > 0 {
            bail!("ci-triage --gate: {real} real failure(s)");
        }
        return Ok(());
    }
    Ok(())
}
