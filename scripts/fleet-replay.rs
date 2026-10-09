#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! serde_yaml = "0.9"
//! serde_json = "1"
//! ```
//!
//! REPLAY HARNESS FOR THE FLEET STARVATION DETECTOR (FV-RELAY-FLEET-002).
//!
//! WHY THIS EXISTS, measured rather than asserted. FV-RELAY-FLEET-002 was
//! DEMOTED because its four steps are content greps that cannot detect the
//! starvation comparison being wrong. An independent clean-room review proved
//! it: it inverted
//!
//!     [ "$age" -lt "$limit" ] && continue        (fleet-status.yml:142)
//!
//! to `-ge`, which makes the loop `continue` on exactly the jobs that are TOO
//! OLD so the alarm can NEVER fire — and all four steps still exited 0. A gate
//! that cannot notice its own subject being inverted is not evidence.
//!
//! WHAT IT DOES. It extracts the REAL step body from fleet-status.yml, stubs
//! only the two `gh api` calls, and runs it against fixtures with known job
//! ages. It does NOT re-implement the comparison — re-implementing it would
//! test this file's copy rather than the workflow's, which is the mistake that
//! makes a harness agree with itself forever.
//!
//! WHY BOTH DIRECTIONS ARE ASSERTED. Checking only "old job => alarm" would
//! still pass if the comparison were replaced by `true`; checking only "young
//! job => no alarm" would pass if it were `false`. Asserting both pins the
//! comparison from both sides, so any inversion fails at least one case.
//!
//! Run:  ./scripts/fleet-replay.rs
//! Exits non-zero, naming the case, if any fixture disagrees.

use std::io::Write;
use std::process::Command;

const WF: &str = ".github/workflows/fleet-status.yml";
const STEP: &str = "Detect starvation";

/// One fixture: a job row as the workflow's own `--jq` would emit it, plus the
/// `stuck` count the detector must report for it.
struct Case {
    name: &'static str,
    /// minutes ago this job was CREATED (queued)
    age_min: i64,
    /// minutes ago something in the run STARTED; None = nothing started
    last_start_min: Option<i64>,
    labels: &'static str,
    want_stuck: u32,
    why: &'static str,
}

fn iso(mins_ago: i64) -> String {
    // `date -u -v-Nм` is BSD-only and -d is GNU-only; the workflow's own
    // age_min() already handles both, so generate the timestamp here instead
    // of shelling out, and keep the harness portable.
    let out = Command::new("date")
        .args(["-u", "+%s"])
        .output()
        .expect("date");
    let now: i64 = String::from_utf8_lossy(&out.stdout).trim().parse().unwrap();
    let t = now - mins_ago * 60;
    let out = Command::new("date")
        .args(["-u", "-r", &t.to_string(), "+%Y-%m-%dT%H:%M:%SZ"])
        .output()
        .or_else(|_| {
            Command::new("date")
                .args(["-u", "-d", &format!("@{t}"), "+%Y-%m-%dT%H:%M:%SZ"])
                .output()
        })
        .expect("date format");
    String::from_utf8_lossy(&out.stdout).trim().to_string()
}

/// The step's `run:` body AND its `env:`, both taken from the workflow.
///
/// The thresholds are READ rather than hardcoded, then ASSERTED against what
/// the fixtures assume. Hardcoding them would let a threshold change silently
/// re-interpret every case; reading them without asserting would let the same
/// change quietly make the fixtures meaningless (a 60-minute job is only "too
/// old" relative to a 30-minute limit). Read-and-assert fails loudly instead,
/// which is the only outcome that keeps the fixtures honest.
fn step_body_and_env() -> (String, Vec<(String, String)>) {
    let y: serde_yaml::Value =
        serde_yaml::from_str(&std::fs::read_to_string(WF).expect("read workflow")).expect("parse");
    for (_, job) in y["jobs"].as_mapping().expect("jobs").iter() {
        for s in job["steps"].as_sequence().into_iter().flatten() {
            let n = s["name"].as_str().unwrap_or("");
            if !n.starts_with(STEP) {
                continue;
            }
            let body = s["run"].as_str().expect("run").to_string();
            let mut env = Vec::new();
            if let Some(m) = s["env"].as_mapping() {
                for (k, v) in m {
                    let k = k.as_str().unwrap_or("").to_string();
                    let v = match v {
                        serde_yaml::Value::String(x) => x.clone(),
                        serde_yaml::Value::Number(x) => x.to_string(),
                        _ => continue,
                    };
                    // `${{ }}` cannot be resolved outside Actions; the stub
                    // supplies REPO itself, and a secret has no business in a
                    // replay.
                    if !v.contains("${{") {
                        env.push((k, v));
                    }
                }
            }
            for (want_k, want_v) in [("THRESHOLD_MIN", "30"), ("HOSTED_THRESHOLD_MIN", "120")] {
                match env.iter().find(|(k, _)| k == want_k) {
                    Some((_, v)) if v == want_v => {}
                    Some((_, v)) => panic!(
                        "{want_k} is {v} in {WF}, but these fixtures are written for {want_v}. \
                         Update the cases and say why in the same commit — a fixture that no \
                         longer straddles the threshold tests nothing."
                    ),
                    None => panic!("{want_k} is not in the {STEP:?} step's env: in {WF}"),
                }
            }
            return (body, env);
        }
    }
    panic!("step starting {STEP:?} not found in {WF} — did it get renamed?");
}

fn run_case(body: &str, env: &[(String, String)], c: &Case) -> u32 {
    let rows = format!(
        "queued\t{}\t{}\t{}\t{}\thttps://example/job/1",
        c.last_start_min.map(iso).unwrap_or_else(|| "-".into()),
        iso(c.age_min),
        "Kani (relay-lc)",
        c.labels
    );
    let out_file = std::env::temp_dir().join(format!("fleet-out-{}", std::process::id()));
    let _ = std::fs::write(&out_file, "");
    // Stub ONLY the two `gh api` calls. Everything else — age_min, the label
    // classification, the last-start comparison, the threshold test — is the
    // workflow's own code, unmodified.
    let script = format!(
        r#"set -uo pipefail
gh() {{
  case "$*" in
    *"runs?status=queued"*) echo 1 ;;
    *"/jobs?per_page"*)     printf '%s\n' "{rows}" ;;
    *) return 1 ;;
  esac
}}
REPO=pulseengine/relay
GITHUB_OUTPUT={out}
{env}
{body}
"#,
        rows = rows,
        out = out_file.display(),
        env = env
            .iter()
            .map(|(k, v)| format!("{k}='{v}'"))
            .collect::<Vec<_>>()
            .join("\n"),
        body = body
    );
    let f = std::env::temp_dir().join(format!("fleet-replay-{}.sh", std::process::id()));
    std::fs::write(&f, &script).expect("write script");
    let out = Command::new("sh").arg(&f).output().expect("sh");
    let outputs = std::fs::read_to_string(&out_file).unwrap_or_default();
    let _ = std::fs::remove_file(&f);
    let _ = std::fs::remove_file(&out_file);
    let stuck = outputs
        .lines()
        .find_map(|l| l.strip_prefix("stuck="))
        .and_then(|v| v.trim().parse::<u32>().ok());
    match stuck {
        Some(n) => n,
        None => {
            eprintln!(
                "  the step produced no `stuck=` output for case {:?}\n  stdout: {}\n  stderr: {}",
                c.name,
                String::from_utf8_lossy(&out.stdout).trim(),
                String::from_utf8_lossy(&out.stderr).trim()
            );
            u32::MAX
        }
    }
}

fn main() {
    let (body, env) = step_body_and_env();
    // THRESHOLD_MIN is 30 and HOSTED_THRESHOLD_MIN is 120 in the workflow's
    // `env:`. These fixtures straddle both, so a change to either threshold
    // that is not reflected here shows up as a failing case rather than as a
    // silently weaker check.
    let cases = [
        Case {
            name: "self-hosted, 60 min, nothing started",
            age_min: 60,
            last_start_min: None,
            labels: "self-hosted,linux,x64,rust-cpu",
            want_stuck: 1,
            why: "60 > THRESHOLD_MIN(30) and nothing in the run has started: this is the alarm's whole purpose",
        },
        Case {
            name: "self-hosted, 5 min, nothing started",
            age_min: 5,
            last_start_min: None,
            labels: "self-hosted,linux,x64,rust-cpu",
            want_stuck: 0,
            why: "5 < THRESHOLD_MIN(30): a normal queue must NOT alarm. Pins the comparison from the other side, so replacing it with `true` fails here",
        },
        Case {
            name: "hosted, 60 min, nothing started",
            age_min: 60,
            last_start_min: None,
            labels: "ubuntu-latest",
            want_stuck: 0,
            why: "hosted contention is held to HOSTED_THRESHOLD_MIN(120), and 60 is under it — hosted slowness is not fleet death (#514)",
        },
        Case {
            name: "hosted, 180 min, nothing started",
            age_min: 180,
            last_start_min: None,
            labels: "ubuntu-latest",
            want_stuck: 1,
            why: "180 > HOSTED_THRESHOLD_MIN(120): even hosted has a limit",
        },
        Case {
            name: "self-hosted, 60 min, but something started 2 min ago",
            age_min: 60,
            last_start_min: Some(2),
            labels: "self-hosted,linux,x64,rust-cpu",
            want_stuck: 0,
            why: "the fleet is serving the run, slowly — the #449 fix that stopped three false alarms (#447/#455/#462)",
        },
    ];

    let mut bad = 0;
    println!("fleet starvation replay: {} cases against {WF}", cases.len());
    for c in &cases {
        let got = run_case(&body, &env, c);
        let ok = got == c.want_stuck;
        if !ok {
            bad += 1;
        }
        println!(
            "  [{}] {:<46} stuck={} want={}",
            if ok { "pass" } else { "FAIL" },
            c.name,
            if got == u32::MAX { "none".into() } else { got.to_string() },
            c.want_stuck
        );
        if !ok {
            println!("         why this matters: {}", c.why);
        }
    }
    if bad > 0 {
        let _ = std::io::stdout().flush();
        eprintln!(
            "\n{bad} case(s) disagree with the detector in {WF}.\n\
             This harness exists because FV-RELAY-FLEET-002's grep steps could not\n\
             notice the starvation comparison being INVERTED. If you changed that\n\
             comparison or a threshold deliberately, update these fixtures in the\n\
             same commit and say why. Do NOT relax a case to make this green."
        );
        std::process::exit(1);
    }
    println!("\nall cases agree: the detector alarms when it should and stays quiet when it should.");
}
