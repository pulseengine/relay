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
//! Exit codes — a verdict, or the honest absence of one:
//!   0  every artifact scoped to the release is done
//!   1  the release still has blocking artifacts (the normal state)
//!   2  could not evaluate: an unparseable artifact file, no release tags
//!      visible, or any other error. NEVER folded into 0 or 1 — a report that
//!      silently dropped a file, or measured the wrong release, is not a
//!      "not ready", it is no report at all.
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

fn main() -> std::process::ExitCode {
    match run() {
        Ok(code) => std::process::ExitCode::from(code),
        Err(e) => {
            eprintln!("release-readiness: could not evaluate: {e:#}");
            std::process::ExitCode::from(2)
        }
    }
}

fn run() -> Result<u8> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let markdown = args.iter().any(|a| a == "--markdown");
    let want = args
        .iter()
        .position(|a| a == "--release")
        .and_then(|i| args.get(i + 1))
        .cloned();
    run_at(std::path::Path::new("."), want, markdown)
}

/// Everything below reads `root` only, so the tests can point it at a fixture.
fn run_at(root: &std::path::Path, want: Option<String>, markdown: bool) -> Result<u8> {
    let mut by_release: BTreeMap<String, Vec<Artifact>> = BTreeMap::new();
    let mut unparseable: Vec<String> = Vec::new();
    // An unreadable directory entry is the same hole as an unparseable file —
    // it used to be skipped by `filter_map(|e| e.ok())`.
    for e in walkdir::WalkDir::new(root.join("artifacts")) {
        let e = e.context("walking artifacts/")?;
        if !e.path().extension().is_some_and(|x| x == "yaml") {
            continue;
        }
        let text = std::fs::read_to_string(e.path())
            .with_context(|| format!("reading {}", e.path().display()))?;
        // A malformed artifact file must not silently shrink the scope — that is
        // the empty-scope-passes shape. The first version of this loop warned
        // and `continue`d, which is exactly that: the file's artifacts left
        // every release's scope and the verdict could still read "ready".
        // Collect them all, then refuse to give a verdict.
        let doc: Doc = match serde_yaml::from_str(&text) {
            Ok(d) => d,
            Err(err) => {
                unparseable.push(format!("{}: {err}", e.path().display()));
                continue;
            }
        };
        for a in doc.artifacts {
            if !a.release.is_empty() {
                by_release.entry(a.release.clone()).or_default().push(a);
            }
        }
    }

    if !unparseable.is_empty() {
        anyhow::bail!(
            "{} artifact file(s) could not be parsed, so every release's scope is incomplete:\n  {}",
            unparseable.len(),
            unparseable.join("\n  ")
        );
    }

    // Default target: the lowest release that is NOT YET TAGGED.
    //
    // "Lowest release with something incomplete" is the obvious rule and it is
    // wrong: it selects falcon-v0.1.0, which shipped long ago and still carries
    // artifacts left at `implemented`. A shipped release's stale statuses are a
    // traceability debt, not a thing the NEXT release is waiting on. The next
    // release is the one whose tag does not exist yet.
    //
    // No tags is an error, not an empty set. `unwrap_or_default()` here used to
    // turn a failed `git` (or a clone without tags) into "nothing has shipped",
    // which silently re-targets the report at the oldest scope in the tree.
    let out = std::process::Command::new("git")
        .arg("-C")
        .arg(root)
        .args(["tag", "--list", "falcon-v*"])
        .output()
        .context("running `git tag`")?;
    if !out.status.success() {
        anyhow::bail!("`git tag` failed: {}", String::from_utf8_lossy(&out.stderr).trim());
    }
    let tags: std::collections::HashSet<String> = String::from_utf8_lossy(&out.stdout)
        .lines()
        .map(|l| l.trim().to_string())
        .filter(|l| !l.is_empty())
        .collect();
    if tags.is_empty() {
        anyhow::bail!("no falcon-v* tags visible — a shallow clone? (the workflow needs fetch-depth: 0)");
    }

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
        // By VERSION, not by map order: the keys sort as strings, and
        // "falcon-v1.100.0" < "falcon-v1.99.1" as strings.
        None => by_release
            .keys()
            .filter(|r| !tags.contains(*r) && ver(r) > latest)
            .min_by_key(|r| ver(r))
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
    Ok(if blocking.is_empty() && !arts.is_empty() { 0 } else { 1 })
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;
    use std::process::Command;

    /// A throwaway git repo holding `tags` and one artifact file per entry of
    /// `files`. Tags point at a blob, so no commit (and no signing) is needed.
    fn fixture(name: &str, tags: &[&str], files: &[&str]) -> PathBuf {
        let d = std::env::temp_dir().join(format!("release-readiness-{}-{name}", std::process::id()));
        let _ = std::fs::remove_dir_all(&d);
        std::fs::create_dir_all(d.join("artifacts")).unwrap();
        let git = |args: &[&str]| {
            let o = Command::new("git").arg("-C").arg(&d).args(args).output().unwrap();
            assert!(o.status.success(), "git {args:?}: {}", String::from_utf8_lossy(&o.stderr));
            String::from_utf8(o.stdout).unwrap().trim().to_string()
        };
        git(&["init", "-q"]);
        let blob = {
            let mut c = Command::new("git")
                .arg("-C")
                .arg(&d)
                .args(["hash-object", "-w", "--stdin"])
                .stdin(std::process::Stdio::piped())
                .stdout(std::process::Stdio::piped())
                .spawn()
                .unwrap();
            use std::io::Write;
            c.stdin.take().unwrap().write_all(b"fixture").unwrap();
            String::from_utf8(c.wait_with_output().unwrap().stdout).unwrap().trim().to_string()
        };
        for t in tags {
            git(&["tag", t, &blob]);
        }
        for (i, f) in files.iter().enumerate() {
            std::fs::write(d.join("artifacts").join(format!("{i}.yaml")), f).unwrap();
        }
        d
    }

    fn art(id: &str, status: &str, release: &str) -> String {
        format!("artifacts:\n  - {{id: {id}, type: sw-req, title: t, status: {status}, release: {release}}}\n")
    }

    #[test]
    fn verdicts_for_ready_and_not_ready() {
        let r = fixture("ready", &["falcon-v1.0.0"], &[&art("A", "verified", "falcon-v1.1.0")]);
        assert_eq!(run_at(&r, None, false).unwrap(), 0);
        let n = fixture("notready", &["falcon-v1.0.0"], &[&art("A", "proposed", "falcon-v1.1.0")]);
        assert_eq!(run_at(&n, None, false).unwrap(), 1);
    }

    #[test]
    fn an_unparseable_file_is_no_verdict_not_a_ready_one() {
        // The release's only blocker lives in a broken file. The first version
        // of this tool dropped the file with a warning and reported
        // "1/1 artifacts done (100%)", exit 0.
        let d = fixture(
            "hidden-blocker",
            &["falcon-v1.0.0"],
            &[
                &art("DONE", "verified", "falcon-v1.1.0"),
                "artifacts:\n  - {id: BLOCKER, type: sw-req, title: t, status: proposed, release: falcon-v1.1.0}\n    oops: [\n",
            ],
        );
        let err = run_at(&d, None, false).unwrap_err().to_string();
        assert!(err.contains("could not be parsed"), "{err}");
    }

    #[test]
    fn no_visible_tags_is_no_verdict() {
        // `unwrap_or_default()` used to read this as "nothing has shipped".
        let d = fixture("notags", &[], &[&art("A", "verified", "falcon-v1.1.0")]);
        assert!(run_at(&d, None, false).is_err());
    }

    #[test]
    fn the_next_release_is_chosen_by_version_not_string_order() {
        // As strings, "falcon-v1.100.0" sorts before "falcon-v1.99.1".
        let d = fixture(
            "ordering",
            &["falcon-v1.99.0"],
            &[&art("NEXT", "verified", "falcon-v1.99.1"), &art("LATER", "proposed", "falcon-v1.100.0")],
        );
        assert_eq!(run_at(&d, None, false).unwrap(), 0, "must target v1.99.1, which is ready");
    }
}
