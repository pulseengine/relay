#!/usr/bin/env rust-script
//! ```cargo
//! [dependencies]
//! serde_yaml = "0.9"
//! ```
//!
//! REPLAY HARNESS FOR KANI'S SCOPE CHECK (#548).
//!
//! WHY THIS EXISTS, measured rather than asserted. `kani.yml` already had the
//! change detection — its roll-up even prints "Kani matrix: skipped (no
//! Kani-relevant files changed) — OK". It was INERT on every push to main,
//! because the step read
//!
//!     BASE_SHA: ${{ github.event.pull_request.base.sha }}
//!
//! which does not exist on a `push`, so the empty-base branch ran all 47 legs
//! unconditionally. Four code-free merges for falcon-v1.140.0 each spawned a
//! full matrix; this workflow's concurrency is serial on main, so one of them
//! held that release's candidate for 2 h 20 m re-proving a proof that was
//! already green on a commit with identical crate Rust.
//!
//! WHAT IT DOES. It extracts the REAL step body from `kani.yml` and runs it,
//! unmodified, against fixtures in a throwaway git repository — so `git
//! cat-file` and `git diff` operate on actual commits rather than on stubs. It
//! does NOT re-implement the base selection or the grep. Re-implementing them
//! would test this file's copy instead of the workflow's, which is the mistake
//! that makes a harness agree with itself forever.
//!
//! WHY BOTH DIRECTIONS ARE ASSERTED, and why the backstops are cases too.
//! Checking only "docs-only ⇒ skip" would still pass if the grep were replaced
//! by `false` — the matrix would then never run and no proof would ever be
//! checked. Checking only "crates/ ⇒ run" would pass if it were replaced by
//! `true`, which is the bug being fixed. Asserting both pins the predicate from
//! both sides. The three unusable-base cases are asserted because each one must
//! SWEEP: a scope check that skips when it cannot determine scope is strictly
//! worse than no scope check, and collapsing them into one `-z` test is exactly
//! what made the push case silently unconditional.
//!
//! Run:  ./scripts/kani-scope-replay.rs
//! Exits non-zero, naming the case, if any fixture disagrees.

use std::process::Command;

const WF: &str = ".github/workflows/kani.yml";
const STEP: &str = "Scope check";

/// One fixture: what the step is handed, and the decision it must reach.
struct Case {
    name: &'static str,
    /// Paths changed in the second commit. Empty = base is HEAD (no diff).
    changed: &'static [&'static str],
    /// How BASE_SHA is supplied. `Base` = the real first-commit sha.
    base: BaseKind,
    event: &'static str,
    want_run: bool,
    why: &'static str,
}

enum BaseKind {
    /// The genuine parent commit — the normal push case.
    Parent,
    /// `github.event.before` on a branch creation, or an unknown predecessor.
    AllZeros,
    /// A well-formed sha that is not in this repository (force-push).
    NotInHistory,
    /// Neither field present — `workflow_dispatch`.
    Empty,
}

fn sh(cwd: &std::path::Path, script: &str) -> (bool, String) {
    let o = Command::new("bash")
        .arg("-c")
        .arg(script)
        .current_dir(cwd)
        .output()
        .expect("spawn bash");
    (
        o.status.success(),
        format!(
            "{}{}",
            String::from_utf8_lossy(&o.stdout),
            String::from_utf8_lossy(&o.stderr)
        ),
    )
}

/// Pull the step's `run:` body AND its `BASE_SHA` expression out of the
/// workflow. The `env` value cannot be *evaluated* here — `${{ }}` is resolved
/// by Actions, not by bash — so BASE_SHA is supplied per fixture below and the
/// expression itself is asserted instead. That split is the harness's main
/// coverage boundary and is stated in `main`.
fn step_body_and_base_expr() -> (String, String) {
    let y: serde_yaml::Value =
        serde_yaml::from_str(&std::fs::read_to_string(WF).expect("read kani.yml")).expect("parse");
    for (_, job) in y["jobs"].as_mapping().expect("jobs").iter() {
        for s in job["steps"].as_sequence().into_iter().flatten() {
            if s["name"].as_str().unwrap_or("").starts_with(STEP) {
                let body = s["run"].as_str().expect("run block").to_string();
                let expr = s["env"]["BASE_SHA"]
                    .as_str()
                    .expect("the step must set BASE_SHA in env")
                    .to_string();
                return (body, expr);
            }
        }
    }
    panic!("no step whose name starts with {STEP:?} in {WF} — the harness is pointed at the wrong step, which would make every case vacuous");
}

fn main() {
    let (body, base_expr) = step_body_and_base_expr();

    // THE ONE THING THE FIXTURES CANNOT TEST, asserted here instead. The defect
    // in #548 was not in the body — it was the `env` expression reading only
    // `pull_request.base.sha`, which is empty on a push. Actions evaluates
    // `${{ }}`, bash does not, so no fixture below can exercise it: every case
    // hands BASE_SHA in directly. Without this assertion the whole harness
    // would stay green after a revert of the actual fix, which is the shape of
    // a gate that cannot detect its own subject changing.
    for needle in ["github.event.pull_request.base.sha", "github.event.before"] {
        assert!(
            base_expr.contains(needle),
            "BASE_SHA is `{base_expr}`, which does not reference {needle:?}. A push has no \
             `pull_request.base.sha`, so dropping the `|| github.event.before` fallback makes \
             every push take the backstop branch and run all 47 legs again (#548)."
        );
    }

    // Sanity-check that the extracted body is the thing we think it is. Without
    // this, a rename or refactor turns every assertion below into a test of an
    // empty string that happens to pass.
    for needle in ["GITHUB_OUTPUT", "run=true", "run=false", "git diff --name-only"] {
        assert!(
            body.contains(needle),
            "extracted step body lacks {needle:?} — refusing to report a verdict on a body this harness does not recognise"
        );
    }
    // The grep is the predicate under test; assert its alternatives are the ones
    // the cases below exercise, so a silently-narrowed grep cannot pass.
    for needle in ["crates/", "Cargo\\.toml", "Cargo\\.lock", "kani\\.yml"] {
        assert!(
            body.contains(needle),
            "the scope grep no longer mentions {needle:?}; update the fixtures deliberately rather than letting coverage shrink"
        );
    }

    let cases = [
        Case {
            name: "push, docs + artifact only",
            changed: &["docs/RELEASE-PLAN.md", "artifacts/verification/FV-X.yaml"],
            base: BaseKind::Parent,
            event: "push",
            want_run: false,
            why: "THE WHOLE POINT: the four falcon-v1.140.0 merges looked exactly like this and each ran 47 legs",
        },
        Case {
            name: "push, a crate source file",
            changed: &["crates/relay-mix-quad/plain/src/lib.rs"],
            base: BaseKind::Parent,
            event: "push",
            want_run: true,
            why: "pins the predicate from the other side: replacing the grep with `false` must fail here",
        },
        Case {
            name: "push, workspace manifest",
            changed: &["Cargo.toml"],
            base: BaseKind::Parent,
            event: "push",
            want_run: true,
            why: "a dependency or feature change alters what the harnesses prove, without touching crates/",
        },
        Case {
            name: "push, this workflow itself",
            changed: &[".github/workflows/kani.yml"],
            base: BaseKind::Parent,
            event: "push",
            want_run: true,
            why: "changing the matrix or the kani-version must re-run it, or the gate can be weakened invisibly",
        },
        Case {
            name: "push, all-zeros base (branch created)",
            changed: &["docs/RELEASE-PLAN.md"],
            base: BaseKind::AllZeros,
            event: "push",
            want_run: true,
            why: "no predecessor to diff against — must SWEEP; note the changed set is docs-only, so a skip here would look 'correct' and be wrong",
        },
        Case {
            name: "push, base not in history (force-push)",
            changed: &["docs/RELEASE-PLAN.md"],
            base: BaseKind::NotInHistory,
            event: "push",
            want_run: true,
            why: "`git diff` against an unreachable commit cannot be trusted — must SWEEP, again from a docs-only diff",
        },
        Case {
            name: "workflow_dispatch, no base at all",
            changed: &["docs/RELEASE-PLAN.md"],
            base: BaseKind::Empty,
            event: "workflow_dispatch",
            want_run: true,
            why: "a manual run has neither field; it is the deliberate full-sweep escape hatch",
        },
    ];

    let tmp = std::env::temp_dir().join(format!("kani-scope-replay-{}", std::process::id()));
    let mut fails = 0usize;

    for c in &cases {
        let _ = std::fs::remove_dir_all(&tmp);
        std::fs::create_dir_all(&tmp).expect("mkdir fixture repo");

        // A real two-commit repository, so the step's `git cat-file -e` and
        // `git diff --name-only BASE...HEAD` do real work.
        let setup = r#"
set -e
git init -q .
git config user.email r@example.com
git config user.name 'Replay Harness'
git config commit.gpgsign false
mkdir -p seed && echo seed > seed/f.txt
git add -A && git commit -q -m base
"#;
        let (ok, out) = sh(&tmp, setup);
        assert!(ok, "fixture repo setup failed: {out}");

        let mut mk = String::from("set -e\n");
        for f in c.changed {
            mk.push_str(&format!(
                "mkdir -p \"$(dirname '{f}')\" && echo change > '{f}'\n"
            ));
        }
        mk.push_str("git add -A && git commit -q -m change\n");
        let (ok, out) = sh(&tmp, &mk);
        assert!(ok, "fixture commit failed: {out}");

        let (_, parent) = sh(&tmp, "git rev-parse HEAD~1");
        let base = match c.base {
            BaseKind::Parent => parent.trim().to_string(),
            BaseKind::AllZeros => "0".repeat(40),
            // A valid-shaped sha that this repo has never seen.
            BaseKind::NotInHistory => "dead".repeat(10),
            BaseKind::Empty => String::new(),
        };

        // READ $GITHUB_OUTPUT AS A FILE, FROM OUTSIDE THE SHELL — the way
        // Actions does. Appending `cat "$GITHUB_OUTPUT"` to the body instead
        // looks equivalent and is not: three of this step's branches end in
        // `echo run=true >>"$GITHUB_OUTPUT"; exit 0`, and `exit 0` kills the
        // process before any appended command runs. The harness's first draft
        // did exactly that, and the three backstop cases "failed" having
        // printed the correct message — a verdict about the harness, not the
        // workflow. A step that exits early is the normal case, not the odd one.
        let script = format!(
            "set -u\nexport GITHUB_OUTPUT=\"$PWD/.gh_out\"\n: > \"$GITHUB_OUTPUT\"\nexport GITHUB_EVENT_NAME='{}'\nexport BASE_SHA='{}'\n{}\n",
            c.event, base, body
        );
        let (exit_ok, out) = sh(&tmp, &script);
        let written = std::fs::read_to_string(tmp.join(".gh_out")).unwrap_or_default();

        // More than one `run=` means the step decided twice; that is a defect
        // worth failing on, so count rather than just taking the last.
        let decisions: Vec<&str> = written
            .lines()
            .filter(|l| l.trim() == "run=true" || l.trim() == "run=false")
            .map(|l| l.trim())
            .collect();
        let got = decisions.last().copied();
        let want = if c.want_run { "run=true" } else { "run=false" };

        let bad = !exit_ok || got != Some(want) || decisions.len() != 1;
        if bad {
            fails += 1;
            println!("FAIL  {}", c.name);
            println!("      want {want}, got {:?} (decisions written: {})", got, decisions.len());
            println!("      why this case exists: {}", c.why);
            println!("      base supplied: {}", if base.is_empty() { "<empty>" } else { &base });
            println!("      changed: {:?}", c.changed);
            for l in out.lines().take(14) {
                println!("      | {l}");
            }
        } else {
            println!("ok    {:<42} {want}", c.name);
        }
    }

    let _ = std::fs::remove_dir_all(&tmp);
    println!("\n{} case(s), {} FAIL", cases.len(), fails);
    if fails > 0 {
        std::process::exit(1);
    }
}
