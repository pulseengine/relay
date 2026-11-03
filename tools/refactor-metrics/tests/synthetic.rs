//! Behaviour on a tiny synthetic repository built per test in a temp dir:
//! a fix re-fixing a fix, a non-fix origin, an ignored (reformat) revision,
//! a curated fix, a --no-ff merge classified by its body, scope selection and
//! tag windows. Git is isolated from the user's/system config.

use serde_json::Value;
use std::path::{Path, PathBuf};
use std::process::{Command, Output};
use std::sync::atomic::{AtomicU32, Ordering};

static SEQ: AtomicU32 = AtomicU32::new(0);

struct Repo {
    dir: PathBuf,
    day: u32,
}

fn isolate(cmd: &mut Command) -> &mut Command {
    cmd.env("GIT_CONFIG_GLOBAL", "/dev/null")
        .env("GIT_CONFIG_NOSYSTEM", "1")
        .env("GIT_AUTHOR_NAME", "t")
        .env("GIT_AUTHOR_EMAIL", "t@example.invalid")
        .env("GIT_COMMITTER_NAME", "t")
        .env("GIT_COMMITTER_EMAIL", "t@example.invalid")
}

impl Repo {
    fn new() -> Self {
        let dir = std::env::temp_dir().join(format!(
            "refactor-metrics-test-{}-{}",
            std::process::id(),
            SEQ.fetch_add(1, Ordering::SeqCst)
        ));
        let _ = std::fs::remove_dir_all(&dir);
        std::fs::create_dir_all(&dir).unwrap();
        let r = Repo { dir, day: 0 };
        r.git(&["-c", "init.defaultBranch=main", "init", "-q"]);
        r
    }

    fn git(&self, args: &[&str]) -> String {
        let date = format!("2026-01-{:02}T12:00:00+0000", self.day.max(1));
        let out = isolate(&mut Command::new("git"))
            .arg("-C")
            .arg(&self.dir)
            .args(args)
            .env("GIT_AUTHOR_DATE", &date)
            .env("GIT_COMMITTER_DATE", &date)
            .output()
            .unwrap();
        assert!(
            out.status.success(),
            "git {args:?}: {}",
            String::from_utf8_lossy(&out.stderr)
        );
        String::from_utf8(out.stdout).unwrap().trim().to_string()
    }

    fn write(&self, path: &str, content: &str) {
        std::fs::write(self.dir.join(path), content).unwrap();
    }

    /// Replace line `n` (1-based) of `path`.
    fn set_line(&self, path: &str, n: usize, text: &str) {
        let s = std::fs::read_to_string(self.dir.join(path)).unwrap();
        let mut lines: Vec<String> = s.lines().map(str::to_string).collect();
        lines[n - 1] = text.to_string();
        self.write(path, &(lines.join("\n") + "\n"));
    }

    fn commit(&mut self, msg: &str) -> String {
        self.day += 1;
        self.git(&["add", "-A"]);
        self.git(&["commit", "-q", "-m", msg]);
        self.git(&["rev-parse", "HEAD"])
    }

    fn file(&self, name: &str, content: &str) -> PathBuf {
        let p = self.dir.join(".git").join(name);
        std::fs::write(&p, content).unwrap();
        p
    }
}

impl Drop for Repo {
    fn drop(&mut self) {
        let _ = std::fs::remove_dir_all(&self.dir);
    }
}

struct Fixture {
    repo: Repo,
    c: std::collections::HashMap<&'static str, String>,
}

/// The scenario. Names -> full shas in `c`.
fn fixture() -> Fixture {
    let mut r = Repo::new();
    let mut c = std::collections::HashMap::new();
    let ten: String = (1..=10).map(|i| format!("line{i}\n")).collect();
    r.write("a.txt", &ten);
    r.write("b.txt", "b1\nb2\nb3\n");
    c.insert("init", r.commit("init"));
    r.write("c.txt", "c1\nc2\nc3\n");
    c.insert("feat", r.commit("feat: add c"));
    r.set_line("a.txt", 3, "line3 fixed");
    c.insert("fix3", r.commit("fix: correct line 3"));
    r.set_line("a.txt", 3, "line3 fixed again");
    c.insert("refix3", r.commit("fix(a): re-fix line 3"));
    r.set_line("b.txt", 1, "b1 tweaked");
    c.insert("docs_b", r.commit("docs: tweak b"));
    r.set_line("b.txt", 1, "b1 corrected");
    c.insert("fix_b", r.commit("fix: b line 1"));
    r.git(&["tag", "falcon-v1.0.0"]);
    r.set_line("a.txt", 5, "line5 fixed");
    c.insert("fix5", r.commit("fix: correct line 5"));
    // A reformat that is itself a fix-prefixed commit (like 4c5feda).
    r.set_line("a.txt", 5, "LINE5 FIXED");
    c.insert("fmt", r.commit("fix(fmt): reformat everything"));
    r.set_line("a.txt", 5, "line5 final");
    c.insert("refix5", r.commit("fix: line 5 again"));
    r.set_line("c.txt", 2, "c2 doc");
    c.insert("curated", r.commit("docs: fix the setup gap"));
    r.set_line("c.txt", 2, "c2 fixed");
    c.insert("fix_c", r.commit("fix: c line 2"));
    r.git(&["tag", "falcon-v1.141.0"]);
    // A --no-ff merge: first-parent sees only the merge; it is classified on
    // the body's first line.
    r.git(&["checkout", "-q", "-b", "side"]);
    r.set_line("a.txt", 8, "line8 side");
    c.insert("side", r.commit("wip"));
    r.git(&["checkout", "-q", "main"]);
    r.day += 1;
    r.git(&[
        "merge",
        "-q",
        "--no-ff",
        "side",
        "-m",
        "Merge pull request #7 from u/side",
        "-m",
        "fix: line 8 side fix",
    ]);
    c.insert("merge", r.git(&["rev-parse", "HEAD"]));
    r.set_line("a.txt", 8, "line8 final");
    c.insert("fix8", r.commit("fix: line 8 again"));
    Fixture { repo: r, c }
}

fn run_tool(repo: &Path, args: &[&str]) -> Output {
    isolate(&mut Command::new(env!("CARGO_BIN_EXE_refactor-metrics")))
        .arg("--repo")
        .arg(repo)
        .args(args)
        .output()
        .unwrap()
}

fn json(repo: &Path, args: &[&str]) -> Value {
    let mut a = vec!["--ref", "main", "--format", "json"];
    a.extend_from_slice(args);
    let out = run_tool(repo, &a);
    assert!(
        out.status.success(),
        "{}",
        String::from_utf8_lossy(&out.stderr)
    );
    serde_json::from_slice(&out.stdout).unwrap()
}

fn pairs(f: &Fixture, mode: &Value) -> Vec<(String, String)> {
    let name = |sha: &str| {
        f.c.iter()
            .find(|(_, v)| v.as_str() == sha)
            .map(|(k, _)| k.to_string())
            .unwrap_or_else(|| sha.to_string())
    };
    mode["pairs"]
        .as_array()
        .unwrap()
        .iter()
        .map(|p| {
            (
                name(p["fix"].as_str().unwrap()),
                name(p["origin"].as_str().unwrap()),
            )
        })
        .collect()
}

fn p(a: &str, b: &str) -> (String, String) {
    (a.to_string(), b.to_string())
}

fn lists(f: &Fixture, curated: &[&str], ignore: &[&str]) -> (String, String) {
    let cur: String = curated
        .iter()
        .map(|k| format!("{}\n", &f.c[k][..7]))
        .collect();
    let ign: String = ignore
        .iter()
        .map(|k| format!("{}\n", &f.c[k][..7]))
        .collect();
    let cp = f.repo.file("curated.txt", &cur);
    let ip = f.repo.file("ignore.txt", &ign);
    (cp.display().to_string(), ip.display().to_string())
}

#[test]
fn ignored_rev_passes_blame_through_and_is_never_a_fixer() {
    let f = fixture();
    let d = f.repo.dir.clone();

    // Negative control first: with NO ignore-revs the reformat commit is both
    // blamed (refix5 -> fmt) and a fixer (fmt -> fix5). This is the failure
    // mode the ignore list exists for.
    let (cp, ip) = lists(&f, &["curated"], &[]);
    let js = json(&d, &["--curated-fixes", &cp, "--ignore-revs", &ip]);
    let got = pairs(&f, &js["modes"]["curated"]);
    assert!(got.contains(&p("refix5", "fmt")), "{got:?}");
    assert!(got.contains(&p("fmt", "fix5")), "{got:?}");
    assert!(!got.contains(&p("refix5", "fix5")), "{got:?}");

    // With the reformat ignored: blame passes through to the real origin and
    // the reformat never appears on the fixing side.
    let (cp, ip) = lists(&f, &["curated"], &["fmt"]);
    let js = json(&d, &["--curated-fixes", &cp, "--ignore-revs", &ip]);
    let cur = &js["modes"]["curated"];
    let got = pairs(&f, cur);
    assert_eq!(
        got,
        vec![
            p("refix3", "fix3"),
            p("refix5", "fix5"),
            p("fix_c", "curated"),
            p("fix8", "merge"),
        ]
    );
    assert_eq!(cur["raw_pairs"], 4);
    assert_eq!(cur["refixing_commits"], 4);
    assert_eq!(cur["fixes"], 10);
    assert_eq!(cur["merges"], 13);
    assert_eq!(js["git_failures"].as_array().unwrap().len(), 0);
}

#[test]
fn fix_of_fix_vs_non_fix_origin_and_depth() {
    let f = fixture();
    let (cp, ip) = lists(&f, &["curated"], &["fmt"]);
    let js = json(&f.repo.dir, &["--curated-fixes", &cp, "--ignore-revs", &ip]);
    let cur = &js["modes"]["curated"];
    let got = pairs(&f, cur);
    // fix_b re-touches a line from a NON-fix (docs_b): no pair.
    assert!(!got.iter().any(|(a, _)| a == "fix_b"), "{got:?}");
    // fix3 re-touches init's line: no pair (init is not a fix).
    assert!(!got.iter().any(|(a, _)| a == "fix3"), "{got:?}");
    let depth = &cur["depth_universe"];
    assert_eq!(depth[&f.c["fix3"]], 1);
    assert_eq!(depth[&f.c["refix3"]], 2);
    assert_eq!(depth[&f.c["fix_b"]], 1);
    assert_eq!(depth[&f.c["fmt"]], 1);
    assert_eq!(depth[&f.c["fix8"]], 2);
    assert!(depth.get(&f.c["docs_b"]).is_none());
    assert_eq!(
        cur["chain_depth_histogram"],
        serde_json::json!({"1": 6, "2": 4})
    );

    // Without the ignore list the chain deepens through the reformat.
    let (cp, ip) = lists(&f, &["curated"], &[]);
    let js = json(&f.repo.dir, &["--curated-fixes", &cp, "--ignore-revs", &ip]);
    let depth = &js["modes"]["curated"]["depth_universe"];
    assert_eq!(depth[&f.c["fmt"]], 2);
    assert_eq!(depth[&f.c["refix5"]], 3);
}

#[test]
fn automated_and_curated_figures_are_reported_separately() {
    let f = fixture();
    let (cp, ip) = lists(&f, &["curated"], &["fmt"]);
    let js = json(&f.repo.dir, &["--curated-fixes", &cp, "--ignore-revs", &ip]);
    let auto = &js["modes"]["automated"];
    let cur = &js["modes"]["curated"];
    assert_eq!(auto["fixes"], 9);
    assert_eq!(cur["fixes"], 10);
    assert!(!pairs(&f, auto).contains(&p("fix_c", "curated")));
    assert!(pairs(&f, cur).contains(&p("fix_c", "curated")));
    assert_eq!(auto["raw_pairs"], 3);
    assert_eq!(cur["raw_pairs"], 4);
    let per100 = cur["raw_pairs_per_100_merges"].as_f64().unwrap();
    assert!((per100 - 400.0 / 13.0).abs() < 1e-9);
}

#[test]
fn scope_counts_fixers_in_scope_but_origins_anywhere() {
    let f = fixture();
    let (cp, ip) = lists(&f, &["curated"], &["fmt"]);
    let js = json(
        &f.repo.dir,
        &[
            "--curated-fixes",
            &cp,
            "--ignore-revs",
            &ip,
            "--merges",
            "1",
        ],
    );
    let cur = &js["modes"]["curated"];
    assert_eq!(cur["merges"], 1);
    assert_eq!(pairs(&f, cur), vec![p("fix8", "merge")]);
    assert_eq!(cur["chain_depth_histogram"], serde_json::json!({"2": 1}));
    assert_eq!(js["universe_commits"], 13);

    // Committer-date window: day 12 = side (off the first-parent line),
    // day 13 = merge, day 14 = fix8. Day 12 is included to show that an
    // off-first-parent commit is never counted.
    let js = json(
        &f.repo.dir,
        &[
            "--curated-fixes",
            &cp,
            "--ignore-revs",
            &ip,
            "--since",
            "2026-01-12",
            "--until",
            "2026-01-14",
        ],
    );
    let cur = &js["modes"]["curated"];
    assert_eq!(cur["merges"], 2);
    assert_eq!(cur["fixes"], 2);
    assert_eq!(pairs(&f, cur), vec![p("fix8", "merge")]);
}

#[test]
fn merge_commit_is_classified_by_its_body() {
    let f = fixture();
    let (cp, ip) = lists(&f, &[], &["fmt"]);
    let js = json(
        &f.repo.dir,
        &["--curated-fixes", &cp, "--ignore-revs", &ip, "--dataset"],
    );
    let m = js["dataset"]["commits"]
        .as_array()
        .unwrap()
        .iter()
        .find(|c| c["sha"] == f.c["merge"].as_str())
        .unwrap()
        .clone();
    assert_eq!(m["strict"], true);
    assert_eq!(m["subj2"], "fix: line 8 side fix");
    assert_eq!(m["pr"], 7);
    // the side-branch commit is not on the first-parent line
    assert!(!js["dataset"]["commits"]
        .as_array()
        .unwrap()
        .iter()
        .any(|c| c["sha"] == f.c["side"].as_str()));
}

#[test]
fn tag_windows_and_baseline_bucket_quirk() {
    let f = fixture();
    let (cp, ip) = lists(&f, &["curated"], &["fmt"]);
    let js = json(&f.repo.dir, &["--curated-fixes", &cp, "--ignore-revs", &ip]);
    let cur = &js["modes"]["curated"];
    let tags: Vec<(String, u64)> = cur["windows_by_tag"]
        .as_array()
        .unwrap()
        .iter()
        .map(|w| {
            (
                w["window"].as_str().unwrap().to_string(),
                w["merges"].as_u64().unwrap(),
            )
        })
        .collect();
    assert_eq!(
        tags,
        vec![
            ("falcon-v1.0.0".into(), 6),
            ("falcon-v1.141.0".into(), 5),
            ("unreleased".into(), 2)
        ]
    );
    // BASELINE QUIRK reproduced: v1.141+ has no bucket (null).
    let b: Vec<Value> = cur["windows_baseline_buckets"]
        .as_array()
        .unwrap()
        .iter()
        .map(|w| w["window"].clone())
        .collect();
    assert_eq!(
        b,
        vec![
            Value::from("v1.0-49"),
            Value::Null,
            Value::from("post-v1.140 (unreleased)")
        ]
    );
}

#[test]
fn baseline_text_header() {
    let f = fixture();
    let (cp, ip) = lists(&f, &["curated"], &["fmt"]);
    let out = run_tool(
        &f.repo.dir,
        &[
            "--ref",
            "main",
            "--format",
            "baseline-text",
            "--curated-fixes",
            &cp,
            "--ignore-revs",
            &ip,
        ],
    );
    assert!(out.status.success());
    let s = String::from_utf8(out.stdout).unwrap();
    assert!(
        s.starts_with(
            "# FIX commits (first-parent, origin/main): 10 of 13\n\
         # blame fix->fix pairs (>= 1 overlapping lines): 4; distinct re-fixing commits: 4\n\
         # chain depth distribution (1 = fix of non-fix code): {1: 6, 2: 4}\n"
        ),
        "{s}"
    );
}

#[test]
fn cli_contract() {
    let f = fixture();
    let d = &f.repo.dir;
    let v = run_tool(d, &["--version"]);
    assert_eq!(v.status.code(), Some(0));
    assert_eq!(
        String::from_utf8_lossy(&v.stdout),
        format!("refactor-metrics {}\n", env!("CARGO_PKG_VERSION"))
    );
    assert_eq!(run_tool(d, &["-V"]).status.code(), Some(0));
    let h = run_tool(d, &["--help"]);
    assert_eq!(h.status.code(), Some(0));
    assert!(String::from_utf8_lossy(&h.stdout).contains("--since"));
    assert_eq!(run_tool(d, &["--bogus"]).status.code(), Some(2));
    assert_eq!(run_tool(d, &["--merges", "0"]).status.code(), Some(2));
    assert_eq!(run_tool(d, &["--format", "xml"]).status.code(), Some(2));
    assert_eq!(run_tool(d, &["--since", "2026-1-1"]).status.code(), Some(2));
    assert_eq!(
        run_tool(d, &["--merges", "5", "--since", "2026-01-01"])
            .status
            .code(),
        Some(2)
    );
    // empty scope is an error, not a zero measurement
    let (cp, ip) = lists(&f, &[], &["fmt"]);
    let e = run_tool(
        d,
        &[
            "--ref",
            "main",
            "--since",
            "2030-01-01",
            "--curated-fixes",
            &cp,
            "--ignore-revs",
            &ip,
        ],
    );
    assert_eq!(e.status.code(), Some(1));
    // an ignore-rev that does not resolve is an error (the baseline silently blamed nothing)
    let bad = f.repo.file("bad.txt", "deadbee\n");
    let e = run_tool(
        d,
        &["--ref", "main", "--ignore-revs", bad.to_str().unwrap()],
    );
    assert_eq!(e.status.code(), Some(1));
    // a curated entry that is not exactly 7 chars is an error
    let bad = f.repo.file("badc.txt", "abcdef12\n");
    let e = run_tool(
        d,
        &[
            "--ref",
            "main",
            "--curated-fixes",
            bad.to_str().unwrap(),
            "--ignore-revs",
            &ip,
        ],
    );
    assert_eq!(e.status.code(), Some(1));
}
