#!/usr/bin/env rust-script
//! Has the nightly soak actually RUN? Not "is soak.yml correct" — did it run.
//!
//! WHY THIS EXISTS (SWREQ-FALCON-ENDURANCE-P01, VGATE-P05). That requirement
//! says the position hold shall be "watched nightly". Two independent things
//! must hold for that clause, and BOTH have failed separately:
//!
//!   1. THE WORKFLOW MUST BE VALID. From #498 (2026-10-01 08:56) to #507
//!      (10-02 00:49) `soak.yml` was INVALID to Actions — an empty `${{ }}`
//!      inside a `run:` block — so every event produced a run with ZERO JOBS
//!      and `conclusion=failure`. Sixteen hours dark, and it read as noise.
//!
//!   2. THE SCHEDULE MUST FIRE. On 2026-10-02 the 03:17 UTC slot did NOT fire;
//!      checked 67 minutes late, there was no `schedule` event at all. This repo
//!      has measured GitHub cron throttling of 2.4-5.5 h (#436).
//!
//! AND `FV-FALCON-ENDURANCE-001` COULD NOT SEE EITHER. All three of its soak
//! assertions are CONTENT GREPS against soak.yml, so they passed happily
//! through the whole outage and would pass through any amount of throttling.
//! They check what the watch SAYS, never that the watch RAN.
//!
//! So this measures the one thing those greps cannot: the AGE of the newest
//! `schedule`-event soak run. A workflow that is syntactically perfect and never
//! fires is indistinguishable, from inside the repo, from one that is working.
//!
//! IT DELIBERATELY IGNORES `workflow_dispatch` RUNS. A human (or this loop)
//! dispatching the soak when it looks stale proves the file is valid; it does
//! NOT prove a nightly exists, and counting it would let the check be satisfied
//! by the very act of checking.
//!
//! Usage:
//!   scripts/soak-freshness.rs              report (exit 0)
//!   scripts/soak-freshness.rs --gate       exit 1 when stale
//!   scripts/soak-freshness.rs --max-age-h 36
//!
//! ```cargo
//! [dependencies]
//! anyhow = "1"
//! serde_json = "1"
//! ```

use anyhow::{bail, Context, Result};
use std::process::Command;

const REPO: &str = "pulseengine/relay";
/// A "nightly" that has not fired in a day and a half is not nightly. 36 h
/// rather than 24 h on purpose: this repo's measured cron throttling reaches
/// 5.5 h, and a gate that fires on normal lateness gets ignored, which is the
/// failure mode the soak itself already had.
const DEFAULT_MAX_AGE_H: i64 = 36;

fn main() -> Result<()> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let gate = args.iter().any(|a| a == "--gate");
    let max_age_h = args
        .windows(2)
        .find(|w| w[0] == "--max-age-h")
        .and_then(|w| w[1].parse::<i64>().ok())
        .unwrap_or(DEFAULT_MAX_AGE_H);

    if Command::new("gh").arg("--version").output().is_err() {
        // A missing tool must fail loudly: the fleet monitor ran
        // `gh api ... || true` on boxes without gh for months and every answer
        // was an empty result (#436).
        bail!("soak-freshness: `gh` is not on PATH — refusing to report a fresh soak it could not check");
    }

    let out = Command::new("gh")
        .args([
            "run", "list", "--repo", REPO, "--workflow", "soak.yml",
            "--limit", "40", "--json", "event,status,conclusion,createdAt,databaseId",
        ])
        .output()
        .context("gh run list failed")?;
    let runs: serde_json::Value =
        serde_json::from_slice(&out.stdout).context("gh run list did not return JSON")?;
    let runs = runs.as_array().cloned().unwrap_or_default();

    // Count what we are NOT allowed to count, so the report says why.
    let dispatched = runs.iter().filter(|r| r["event"] == "workflow_dispatch").count();
    let pushed = runs.iter().filter(|r| r["event"] == "push").count();
    let scheduled: Vec<&serde_json::Value> =
        runs.iter().filter(|r| r["event"] == "schedule").collect();

    println!("soak.yml, last {} runs: {} schedule, {} workflow_dispatch, {} push",
             runs.len(), scheduled.len(), dispatched, pushed);
    if pushed > 0 {
        // soak.yml declares only `schedule` and `workflow_dispatch`. A `push`
        // run means GitHub created one anyway, which it does when the file is
        // INVALID — the #498/#507 outage shape.
        println!("::warning::{pushed} soak run(s) on a `push` event. soak.yml declares only \
                  schedule + workflow_dispatch, so a push run means GitHub could not parse \
                  the file and created a 0-job failure (the #498 outage shape). Check \
                  `actionlint .github/workflows/soak.yml`.");
    }

    let Some(newest) = scheduled.first() else {
        let msg = format!(
            "NO `schedule`-event soak run in the last {} runs. The nightly has not fired at all. \
             ENDURANCE-P01's \"watched nightly\" clause is UNMET, whatever soak.yml says.",
            runs.len()
        );
        println!("::error::{msg}");
        if gate {
            bail!("{msg}");
        }
        return Ok(());
    };

    let created = newest["createdAt"].as_str().unwrap_or("");
    // Age via `date`, so there is no chrono dependency and no hand-rolled
    // calendar arithmetic. BSD `date -j -f` first, GNU `date -d` second — the
    // same fallback board-health.rs needed after a month-boundary bug.
    let epoch = |s: &str| -> Option<i64> {
        for a in [
            vec!["-j", "-f", "%Y-%m-%dT%H:%M:%SZ", s, "+%s"],
            vec!["-d", s, "+%s"],
        ] {
            if let Ok(o) = Command::new("date").args(&a).output() {
                if o.status.success() {
                    if let Ok(v) = String::from_utf8_lossy(&o.stdout).trim().parse::<i64>() {
                        return Some(v);
                    }
                }
            }
        }
        None
    };
    let now = epoch(&String::from_utf8_lossy(
        &Command::new("date").args(["-u", "+%Y-%m-%dT%H:%M:%SZ"]).output()?.stdout,
    ).trim().to_string())
        .context("could not read the current time")?;
    let then = epoch(created).with_context(|| format!("could not parse createdAt {created:?}"))?;
    let age_h = (now - then) / 3600;
    let concl = newest["conclusion"].as_str().unwrap_or("running");

    println!("newest scheduled soak: {created} ({age_h} h ago), conclusion={concl}");
    if age_h > max_age_h {
        let msg = format!(
            "the newest `schedule`-event soak run is {age_h} h old, over the {max_age_h} h bar. \
             A nightly that has not fired in {age_h} h is not nightly — ENDURANCE-P01's \
             \"watched nightly\" clause is UNMET. soak.yml being correct does not satisfy it; \
             see #436 for this repo's measured cron throttling."
        );
        println!("::error::{msg}");
        if gate {
            bail!("{msg}");
        }
        return Ok(());
    }
    println!("::notice::soak freshness OK: newest scheduled run {age_h} h old (bar {max_age_h} h), conclusion={concl}");
    Ok(())
}
