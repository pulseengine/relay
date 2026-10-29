//! refactor-metrics: the frozen fix-of-a-fix detector.
//!
//! This is a line-for-line port of the baseline method (a `build.py` dataset
//! pass plus an `analyze.py` report pass, run on relay origin/main @55af1e9).
//! Where the baseline had a quirk, the quirk is reproduced and marked
//! `BASELINE QUIRK` so the frozen instrument equals the method that produced
//! the baseline. See README.md for the method and its known error modes.

pub mod pycompat;

use pycompat::{decode_stdout, prefix_chars, split_ws, splitlines, strip};
use regex::Regex;
use std::collections::{BTreeMap, HashMap, HashSet};
use std::path::{Path, PathBuf};
use std::process::Command;
use std::sync::OnceLock;

pub const VERSION: &str = env!("CARGO_PKG_VERSION");

/// Built-in data files (the frozen lists). Overridable from the CLI.
pub const CURATED_FIXES_TXT: &str = include_str!("../curated-fixes.txt");
pub const IGNORE_REVS_TXT: &str = include_str!("../ignore-revs.txt");

/// git `-c` settings pinned to git's defaults, so a user/system config cannot
/// change hunk boundaries, path quoting or blame attribution. The baseline
/// ran with none of these set (i.e. the defaults).
const PINNED_CONFIG: &[&str] = &[
    "core.quotepath=true",
    "color.ui=never",
    "diff.algorithm=myers",
    "diff.indentHeuristic=true",
    "diff.renames=true",
    "diff.renameLimit=1000",
    "diff.noprefix=false",
    "diff.mnemonicPrefix=false",
    "diff.relative=false",
    "diff.interHunkContext=0",
    "blame.ignoreRevsFile=",
    "blame.markIgnoredLines=false",
    "blame.markUnblamableLines=false",
    "log.showSignature=false",
];

// ---------------------------------------------------------------- git runner

pub struct Git {
    repo: PathBuf,
    /// Commands that exited non-zero. The baseline ignored every git exit
    /// status (stdout only); the port keeps that behaviour but records them.
    pub failures: Vec<String>,
}

impl Git {
    pub fn new(repo: &Path) -> Self {
        Git {
            repo: repo.to_path_buf(),
            failures: Vec::new(),
        }
    }

    pub fn run(&mut self, args: &[String]) -> String {
        let mut cmd = Command::new("git");
        cmd.arg("-C").arg(&self.repo);
        for c in PINNED_CONFIG {
            cmd.arg("-c").arg(c);
        }
        cmd.args(args);
        match cmd.output() {
            Ok(out) => {
                if !out.status.success() {
                    self.failures.push(args.join(" "));
                }
                decode_stdout(&out.stdout)
            }
            Err(e) => {
                self.failures
                    .push(format!("{} (spawn: {e})", args.join(" ")));
                String::new()
            }
        }
    }

    fn run_s(&mut self, args: &[&str]) -> String {
        let v: Vec<String> = args.iter().map(|s| s.to_string()).collect();
        self.run(&v)
    }

    /// Strict resolution used only for input validation (not part of the
    /// measured method): `Some(full sha)` or `None`.
    pub fn resolve_commit(&self, rev: &str) -> Option<String> {
        let out = Command::new("git")
            .arg("-C")
            .arg(&self.repo)
            .args(["rev-parse", "--verify", "--quiet"])
            .arg(format!("{rev}^{{commit}}"))
            .output()
            .ok()?;
        if !out.status.success() {
            return None;
        }
        let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
        (!s.is_empty()).then_some(s)
    }

    pub fn version(&self) -> String {
        Command::new("git")
            .arg("--version")
            .output()
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
            .unwrap_or_default()
    }
}

// ------------------------------------------------------------------ patterns

struct Patterns {
    pr_suffix: Regex,
    pr_merge: Regex,
    strict: Regex,
    kw: Regex,
    broad_excl: Regex,
    rename: Regex,
    hunk: Regex,
    porcelain: Regex,
    bucket: Regex,
}

fn pats() -> &'static Patterns {
    static P: OnceLock<Patterns> = OnceLock::new();
    P.get_or_init(|| Patterns {
        // re.search(r'\(#(\d+)\)\s*$', s) or re.search(r'Merge pull request #(\d+)', s)
        pr_suffix: Regex::new(r"\(#(\d+)\)\s*$").unwrap(),
        pr_merge: Regex::new(r"Merge pull request #(\d+)").unwrap(),
        // STRICT = re.compile(r'^(fix|hotfix|revert)(\(|!|:)|^Revert ', re.I)
        strict: Regex::new(r"(?i)^(fix|hotfix|revert)(\(|!|:)|^Revert ").unwrap(),
        // KW = re.compile(r'\b(fix|...|wrong)\b', re.I)
        kw: Regex::new(
            r"(?i)\b(fix|fixes|fixed|fixing|regression|revert|repair|broke|broken|flake|flaky|stall(s|ed)?|bug|hotfix|re-?fix|wrong)\b",
        )
        .unwrap(),
        // re.match(r'^(Merge|plan|release|req|verify)', s)  -- case-SENSITIVE
        broad_excl: Regex::new(r"^(Merge|plan|release|req|verify)").unwrap(),
        // re.sub(r'\{[^}]*=> ([^}]*)\}', r'\1', f)
        rename: Regex::new(r"\{[^}]*=> ([^}]*)\}").unwrap(),
        // HUNK = re.compile(r'^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@')
        hunk: Regex::new(r"^@@ -(\d+)(?:,(\d+))? \+(\d+)(?:,(\d+))? @@").unwrap(),
        // re.match(r'^([0-9a-f]{40}) \d+ \d+', line)
        porcelain: Regex::new(r"^([0-9a-f]{40}) \d+ \d+").unwrap(),
        // re.match(r'falcon-v(\d+)\.(\d+)', w)
        bucket: Regex::new(r"^falcon-v(\d+)\.(\d+)").unwrap(),
    })
}

// ------------------------------------------------------------------- dataset

#[derive(Clone, Debug)]
pub struct FileStat {
    pub path: String,
    pub added: u64,
    pub deleted: u64,
}

#[derive(Clone, Debug)]
pub struct Commit {
    pub sha: String,
    /// `%ad --date=short` (author date) -- the baseline's `date` field.
    pub date: String,
    /// `%cd --date=short` (committer date: when it landed on the ref).
    pub cdate: String,
    pub subj: String,
    pub body: String,
    /// Body first line, set only for `Merge pull request` subjects.
    pub subj2: Option<String>,
    pub pr: Option<u64>,
    pub strict: bool,
    pub broad: bool,
    pub window: String,
    pub files: Vec<FileStat>,
}

impl Commit {
    /// `c.get('subj2') or c['subj']`
    pub fn display_subject(&self) -> &str {
        match &self.subj2 {
            Some(s) if !s.is_empty() => s,
            _ => &self.subj,
        }
    }
    /// `f"{c['sha'][:7]} {subj(c)[:90]}"`
    pub fn short(&self) -> String {
        format!(
            "{} {}",
            prefix_chars(&self.sha, 7),
            prefix_chars(self.display_subject(), 90)
        )
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Edge {
    pub fix: String,
    pub origin: String,
    pub path: String,
    pub lines: u64,
}

pub struct Dataset {
    /// Full first-parent history of the ref, oldest first (the universe).
    pub commits: Vec<Commit>,
    pub edges: Vec<Edge>,
}

pub fn classify(subj: &str, body: &str) -> (bool, bool, Option<String>) {
    let p = pats();
    let mut strict = p.strict.is_match(subj);
    let mut broad = strict || (p.kw.is_match(subj) && !p.broad_excl.is_match(subj));
    let mut subj2 = None;
    // BASELINE QUIRK: a `Merge pull request` subject is classified on the
    // body's first line instead, and the broad exclusion list is NOT applied.
    if subj.starts_with("Merge pull request") {
        let b = strip(body);
        let bl = if b.is_empty() {
            String::new()
        } else {
            splitlines(b).first().copied().unwrap_or("").to_string()
        };
        strict = p.strict.is_match(&bl);
        broad = strict || p.kw.is_match(&bl);
        subj2 = Some(bl);
    }
    (strict, broad, subj2)
}

fn parse_pr(subj: &str) -> Option<u64> {
    let p = pats();
    p.pr_suffix
        .captures(subj)
        .or_else(|| p.pr_merge.captures(subj))
        .and_then(|c| c[1].parse().ok())
}

/// Parse `git diff --numstat -M` output exactly as the baseline did.
pub fn parse_numstat(out: &str) -> Result<Vec<FileStat>, String> {
    let p = pats();
    let mut files = Vec::new();
    for line in splitlines(out) {
        let parts: Vec<&str> = line.splitn(3, '\t').collect();
        if parts.len() != 3 {
            return Err(format!("unparseable numstat line: {line:?}"));
        }
        let (a, d, mut f) = (parts[0], parts[1], parts[2].to_string());
        // BASELINE QUIRK: brace renames `{a => b}` are rewritten to the new
        // side, then a bare `old => new` keeps the last component.
        if f.contains("=>") {
            f = p.rename.replace_all(&f, "${1}").into_owned();
            f = f.rsplit(" => ").next().unwrap_or("").to_string();
        }
        let num = |s: &str| -> Result<u64, String> {
            if s == "-" {
                Ok(0)
            } else {
                strip(s)
                    .parse()
                    .map_err(|_| format!("bad numstat count {s:?}"))
            }
        };
        files.push(FileStat {
            path: f,
            added: num(a)?,
            deleted: num(d)?,
        });
    }
    Ok(files)
}

/// Old-side line ranges of a `-U0` diff: `(start, start+n-1)` for n > 0.
pub fn old_side_ranges(diff: &str) -> Vec<(u64, u64)> {
    let p = pats();
    let mut ranges = Vec::new();
    for line in splitlines(diff) {
        if let Some(m) = p.hunk.captures(line) {
            let st: u64 = m[1].parse().unwrap_or(0);
            let n: u64 = m
                .get(2)
                .map(|g| g.as_str().parse().unwrap_or(0))
                .unwrap_or(1);
            if n > 0 {
                ranges.push((st, st + n - 1));
            }
        }
    }
    ranges
}

/// Lines per blamed commit in `--line-porcelain` output, first-seen order.
pub fn count_porcelain(out: &str) -> Vec<(String, u64)> {
    let p = pats();
    let mut c = OrderedCounter::default();
    for line in splitlines(out) {
        if let Some(m) = p.porcelain.captures(line) {
            c.add(&m[1], 1);
        }
    }
    c.items()
}

pub fn build_dataset(
    git: &mut Git,
    ref_sha: &str,
    ignore_revs: &[String],
) -> Result<Dataset, String> {
    let log = git.run_s(&[
        "log",
        "--first-parent",
        "--format=%H%x1f%ad%x1f%cd%x1f%s%x1f%b%x1e",
        "--date=short",
        ref_sha,
    ]);
    let mut commits = Vec::new();
    for rec in log.split('\x1e') {
        let rec = rec.trim_matches('\n');
        if rec.is_empty() {
            continue;
        }
        let f: Vec<&str> = rec.split('\x1f').collect();
        if f.len() != 5 {
            return Err(format!("unparseable log record ({} fields)", f.len()));
        }
        let (subj, body) = (f[3].to_string(), f[4].to_string());
        let (strict, broad, subj2) = classify(&subj, &body);
        commits.push(Commit {
            sha: f[0].to_string(),
            date: f[1].to_string(),
            cdate: f[2].to_string(),
            pr: parse_pr(&subj),
            subj,
            body,
            subj2,
            strict,
            broad,
            window: String::new(),
            files: Vec::new(),
        });
    }
    if commits.is_empty() {
        return Err(format!("no first-parent history at {ref_sha}"));
    }
    commits.reverse(); // oldest first

    // Windows: every falcon-v* tag in the repo, version-sorted; a commit
    // belongs to the first tag whose first-parent range contains it.
    // BASELINE QUIRK: the tag set is the repo's at run time, not the ref's.
    let tags_out = git.run_s(&["tag", "-l", "falcon-v*", "--sort=v:refname"]);
    let tags: Vec<String> = split_ws(&tags_out).iter().map(|s| s.to_string()).collect();
    let mut win: HashMap<String, String> = HashMap::new();
    let mut prev: Option<&str> = None;
    for t in &tags {
        let rng = match prev {
            Some(p) => format!("{p}..{t}"),
            None => t.clone(),
        };
        let out = git.run(&["rev-list".into(), "--first-parent".into(), rng]);
        for h in split_ws(&out) {
            win.entry(h.to_string()).or_insert_with(|| t.clone());
        }
        prev = Some(t);
    }
    for c in commits.iter_mut() {
        c.window = win
            .get(&c.sha)
            .cloned()
            .unwrap_or_else(|| "unreleased".into());
    }

    // numstat vs first parent; the oldest commit of the universe is skipped.
    for c in commits.iter_mut().skip(1) {
        let sha = c.sha.clone();
        let ns = git.run(&[
            "diff".into(),
            "--numstat".into(),
            "-M".into(),
            format!("{sha}^1"),
            sha,
        ]);
        c.files = parse_numstat(&ns)?;
    }

    // Blame the old side of every changed line of every BROAD commit.
    // BASELINE QUIRK: gated on `broad`, not on `fix`; a curated fix whose
    // subject has no fix keyword is never blamed as a fixer.
    let mut edges = Vec::new();
    for c in commits.iter().skip(1) {
        if !c.broad {
            continue;
        }
        let sha = c.sha.clone();
        let par = format!("{sha}^1");
        for fs in &c.files {
            if fs.deleted == 0 {
                continue;
            }
            // BASELINE QUIRK: plain `-U0 -w` diff on the NEW path only (no
            // -M), so renamed files contribute no old-side lines; a git-quoted
            // path is passed through quoted and matches nothing.
            let diff = git.run(&[
                "diff".into(),
                "-U0".into(),
                "-w".into(),
                par.clone(),
                sha.clone(),
                "--".into(),
                fs.path.clone(),
            ]);
            let ranges = old_side_ranges(&diff);
            if ranges.is_empty() {
                continue;
            }
            let mut args: Vec<String> = vec![
                "blame".into(),
                "-w".into(),
                "--first-parent".into(),
                "--line-porcelain".into(),
            ];
            for r in ignore_revs {
                args.push("--ignore-rev".into());
                args.push(r.clone());
            }
            for (s, e) in &ranges {
                args.push("-L".into());
                args.push(format!("{s},{e}"));
            }
            args.push(par.clone());
            args.push("--".into());
            args.push(fs.path.clone());
            let out = git.run(&args);
            for (o, n) in count_porcelain(&out) {
                edges.push(Edge {
                    fix: sha.clone(),
                    origin: o,
                    path: fs.path.clone(),
                    lines: n,
                });
            }
        }
    }
    Ok(Dataset { commits, edges })
}

// ------------------------------------------------------------------ analysis

/// Python `collections.Counter` restricted to what the baseline used:
/// insertion-ordered keys and a stable `most_common`.
#[derive(Default, Clone, Debug)]
pub struct OrderedCounter {
    keys: Vec<String>,
    idx: HashMap<String, usize>,
    counts: Vec<u64>,
}

impl OrderedCounter {
    pub fn add(&mut self, k: &str, n: u64) {
        match self.idx.get(k) {
            Some(&i) => self.counts[i] += n,
            None => {
                self.idx.insert(k.to_string(), self.keys.len());
                self.keys.push(k.to_string());
                self.counts.push(n);
            }
        }
    }
    pub fn get(&self, k: &str) -> u64 {
        self.idx.get(k).map(|&i| self.counts[i]).unwrap_or(0)
    }
    pub fn total(&self) -> u64 {
        self.counts.iter().sum()
    }
    pub fn len(&self) -> usize {
        self.keys.len()
    }
    pub fn is_empty(&self) -> bool {
        self.keys.is_empty()
    }
    /// Insertion order.
    pub fn items(&self) -> Vec<(String, u64)> {
        self.keys
            .iter()
            .cloned()
            .zip(self.counts.iter().copied())
            .collect()
    }
    /// `Counter.most_common()`: count descending, ties in insertion order.
    pub fn most_common(&self) -> Vec<(String, u64)> {
        let mut v = self.items();
        v.sort_by_key(|p| std::cmp::Reverse(p.1)); // stable
        v
    }
}

#[derive(Clone, Debug)]
pub struct Pair {
    pub fix: usize,
    pub origin: usize,
    pub files: OrderedCounter,
}

#[derive(Clone, Debug, Default)]
pub struct WindowRow {
    pub name: Option<String>,
    pub n: u64,
    pub fix: u64,
    pub refix: u64,
    pub churn: u64,
    pub start: String,
    pub end: String,
}

pub struct Analysis {
    pub mode: &'static str,
    pub universe: usize,
    pub scope: Vec<usize>,
    pub fix: Vec<bool>,
    /// Pairs whose fixing side is in scope, in baseline insertion order.
    pub pairs: Vec<Pair>,
    /// Depth for every FIX in the universe (baseline `depth` dict).
    pub depth: Vec<Option<u32>>,
    pub parents: Vec<HashSet<usize>>,
    pub buckets: Vec<WindowRow>,
    pub tag_windows: Vec<WindowRow>,
    pub fc: OrderedCounter,
    pub fch: OrderedCounter,
    pub tc: OrderedCounter,
    pub rf: OrderedCounter,
}

/// The baseline's window grouping. BASELINE QUIRK: hard-coded through
/// falcon-v1.140; the major version is ignored unless it is 0; a tag of
/// minor >= 141 maps to `None` (the baseline script would crash printing it).
pub fn bucket(w: &str) -> Option<String> {
    if w == "unreleased" {
        return Some("post-v1.140 (unreleased)".into());
    }
    let m = pats().bucket.captures(w)?;
    let a: u64 = m[1].parse().ok()?;
    let b: u64 = m[2].parse().ok()?;
    if a == 0 {
        return Some("v0.x".into());
    }
    let table: &[(u64, u64, &str)] = &[
        (0, 49, "v1.0-49"),
        (50, 99, "v1.50-99"),
        (100, 119, "v1.100-119"),
        (120, 129, "v1.120-129"),
        (130, 135, "v1.130-135"),
        (136, 138, "v1.136-138"),
        (139, 139, "v1.139"),
        (140, 140, "v1.140"),
    ];
    table
        .iter()
        .find(|(lo, hi, _)| *lo <= b && b <= *hi)
        .map(|(_, _, n)| n.to_string())
}

fn window_rows<F: Fn(&Commit) -> Option<String>>(
    ds: &Dataset,
    scope: &[usize],
    fix: &[bool],
    parents: &[HashSet<usize>],
    key: F,
) -> Vec<WindowRow> {
    let mut rows: Vec<WindowRow> = Vec::new();
    for &i in scope {
        let c = &ds.commits[i];
        let k = key(c);
        let pos = match rows.iter().position(|r| r.name == k) {
            Some(p) => p,
            None => {
                rows.push(WindowRow {
                    name: k,
                    start: c.date.clone(),
                    end: c.date.clone(),
                    ..Default::default()
                });
                rows.len() - 1
            }
        };
        let w = &mut rows[pos];
        w.n += 1;
        w.end = c.date.clone();
        if fix[i] {
            w.fix += 1;
            w.churn += c.files.iter().map(|f| f.added + f.deleted).sum::<u64>();
            if !parents[i].is_empty() {
                w.refix += 1;
            }
        }
    }
    rows
}

pub fn analyze(
    ds: &Dataset,
    scope: &[usize],
    curated: &[String],
    ignore_revs: &[String],
    min_lines: u64,
    mode: &'static str,
) -> Analysis {
    let n = ds.commits.len();
    let by: HashMap<&str, usize> = ds
        .commits
        .iter()
        .enumerate()
        .map(|(i, c)| (c.sha.as_str(), i))
        .collect();
    // fix = strict or sha[:7] in EXTRA
    let fix: Vec<bool> = ds
        .commits
        .iter()
        .map(|c| c.strict || curated.iter().any(|e| e == prefix_chars(&c.sha, 7)))
        .collect();

    // Pairs over the WHOLE universe (needed for depth), insertion-ordered.
    let mut pair_idx: HashMap<(usize, usize), usize> = HashMap::new();
    let mut all: Vec<Pair> = Vec::new();
    for e in &ds.edges {
        if e.fix == e.origin {
            continue;
        }
        let Some(&oi) = by.get(e.origin.as_str()) else {
            continue;
        };
        // BASELINE QUIRK (generalised from the hard-coded `startswith('4c5feda')`):
        // an ignored revision is never the fixing side.
        if ignore_revs.iter().any(|r| e.fix.starts_with(r.as_str())) {
            continue;
        }
        let Some(&fi) = by.get(e.fix.as_str()) else {
            continue;
        };
        if fix[fi] && fix[oi] {
            let k = *pair_idx.entry((fi, oi)).or_insert_with(|| {
                all.push(Pair {
                    fix: fi,
                    origin: oi,
                    files: OrderedCounter::default(),
                });
                all.len() - 1
            });
            all[k].files.add(&e.path, e.lines);
        }
    }
    all.retain(|p| p.files.total() >= min_lines);

    let mut parents: Vec<HashSet<usize>> = vec![HashSet::new(); n];
    for p in &all {
        parents[p.fix].insert(p.origin);
    }
    let mut depth: Vec<Option<u32>> = vec![None; n];
    for i in 0..n {
        if !fix[i] {
            continue;
        }
        let m = parents[i]
            .iter()
            .filter_map(|&o| depth[o])
            .max()
            .unwrap_or(0);
        depth[i] = Some(1 + m);
    }

    let in_scope: HashSet<usize> = scope.iter().copied().collect();
    let pairs: Vec<Pair> = all
        .into_iter()
        .filter(|p| in_scope.contains(&p.fix))
        .collect();

    let buckets = window_rows(ds, scope, &fix, &parents, |c| bucket(&c.window));
    let tag_windows = window_rows(ds, scope, &fix, &parents, |c| Some(c.window.clone()));

    let mut fc = OrderedCounter::default();
    let mut fch = OrderedCounter::default();
    let mut tc = OrderedCounter::default();
    let mut rf = OrderedCounter::default();
    for &i in scope {
        for f in &ds.commits[i].files {
            tc.add(&f.path, 1);
            if fix[i] {
                fc.add(&f.path, 1);
                fch.add(&f.path, f.added + f.deleted);
            }
        }
    }
    for p in &pairs {
        for (path, _) in p.files.items() {
            rf.add(&path, 1);
        }
    }

    Analysis {
        mode,
        universe: n,
        scope: scope.to_vec(),
        fix,
        pairs,
        depth,
        parents,
        buckets,
        tag_windows,
        fc,
        fch,
        tc,
        rf,
    }
}

impl Analysis {
    pub fn fixes(&self) -> usize {
        self.scope.iter().filter(|&&i| self.fix[i]).count()
    }
    pub fn refixing_commits(&self) -> usize {
        self.pairs
            .iter()
            .map(|p| p.fix)
            .collect::<HashSet<_>>()
            .len()
    }
    pub fn depth_histogram(&self) -> BTreeMap<u32, u64> {
        let mut h = BTreeMap::new();
        for &i in &self.scope {
            if let Some(d) = self.depth[i] {
                *h.entry(d).or_insert(0) += 1;
            }
        }
        h
    }
    /// Pairs sorted by the fixing commit's position (stable).
    pub fn pairs_sorted(&self) -> Vec<&Pair> {
        let mut v: Vec<&Pair> = self.pairs.iter().collect();
        v.sort_by_key(|p| p.fix);
        v
    }
}

/// Byte-for-byte the baseline `analyze.py` stdout (for the same scope/mode).
pub fn baseline_text(ds: &Dataset, a: &Analysis, min_lines: u64) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let c = &ds.commits;
    let _ = writeln!(
        s,
        "# FIX commits (first-parent, origin/main): {} of {}",
        a.fixes(),
        a.scope.len()
    );
    let _ = writeln!(
        s,
        "# blame fix->fix pairs (>= {min_lines} overlapping lines): {}; distinct re-fixing commits: {}",
        a.pairs.len(),
        a.refixing_commits()
    );
    let hist: Vec<String> = a
        .depth_histogram()
        .iter()
        .map(|(k, v)| format!("{k}: {v}"))
        .collect();
    let _ = writeln!(
        s,
        "# chain depth distribution (1 = fix of non-fix code): {{{}}}",
        hist.join(", ")
    );
    for p in a.pairs_sorted() {
        let f = &c[p.fix];
        let o = &c[p.origin];
        let top: Vec<String> = p
            .files
            .most_common()
            .into_iter()
            .take(3)
            .map(|(k, v)| format!("{k}:{v}"))
            .collect();
        let _ = writeln!(
            s,
            "  {} {}\n      <- {}  [{} lines: {}]",
            f.date,
            prefix_chars(&f.short(), 70),
            prefix_chars(&o.short(), 70),
            p.files.total(),
            top.join(", ")
        );
    }
    let _ = writeln!(
        s,
        "\n# per window: commits, fixes, fix%, re-fixes (blame), fix lines churn"
    );
    for w in &a.buckets {
        // BASELINE QUIRK: the baseline crashes on a None bucket; we print None.
        let name = w.name.clone().unwrap_or_else(|| "None".into());
        let pct = pycompat::fmt_0f(100.0 * w.fix as f64 / w.n as f64);
        let _ = writeln!(
            s,
            "  {name:<26} {}..{} commits={:>3} fix={:>3} ({pct}%) refix={:>2} fixchurn={}",
            w.start, w.end, w.n, w.fix, w.refix, w.churn
        );
    }
    let _ = writeln!(
        s,
        "\n# hotspots: files by #fix commits, #refix commits (blame-overlap on that file), fix churn, total commits"
    );
    for (p, n) in a.fc.most_common().into_iter().take(30) {
        let _ = writeln!(
            s,
            "  {p:<70} fix={n:>2} refix_pairs={:>2} fixchurn={:>6} allcommits={}",
            a.rf.get(&p),
            a.fch.get(&p),
            a.tc.get(&p)
        );
    }
    let _ = writeln!(s, "\n# files by refix pairs");
    for (p, n) in a.rf.most_common().into_iter().take(20) {
        let _ = writeln!(s, "  {p:<70} {n}");
    }
    s
}

// ------------------------------------------------------------------ data files

/// Parse a data file: `#` comments and blank lines ignored; first token per
/// line is the entry.
pub fn parse_list(text: &str) -> Vec<String> {
    text.lines()
        .map(|l| l.trim())
        .filter(|l| !l.is_empty() && !l.starts_with('#'))
        .filter_map(|l| l.split_whitespace().next().map(str::to_string))
        .collect()
}

pub fn validate_curated(list: &[String]) -> Result<(), String> {
    for e in list {
        if e.len() != 7
            || !e
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        {
            return Err(format!(
                "curated fix {e:?}: must be exactly 7 lowercase hex chars (matched against sha[:7])"
            ));
        }
    }
    Ok(())
}

pub fn validate_ignore_revs(list: &[String]) -> Result<(), String> {
    for e in list {
        if !(4..=40).contains(&e.len())
            || !e
                .bytes()
                .all(|b| b.is_ascii_hexdigit() && !b.is_ascii_uppercase())
        {
            return Err(format!(
                "ignore-rev {e:?}: must be 4..40 lowercase hex chars"
            ));
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn strict_and_broad_classification() {
        assert_eq!(classify("fix(ci): x", ""), (true, true, None));
        assert_eq!(classify("Fix: x", ""), (true, true, None));
        assert_eq!(classify("hotfix!: x", ""), (true, true, None));
        assert_eq!(classify("Revert \"feat: x\"", ""), (true, true, None));
        assert!(!classify("fixup: x", "").0);
        // keyword without prefix: broad only
        assert_eq!(classify("docs: fix the gap", ""), (false, true, None));
        assert_eq!(classify("ci: retry the flaky job", ""), (false, true, None));
        // excluded prefixes are case-sensitive
        assert_eq!(classify("release: fix notes", ""), (false, false, None));
        assert_eq!(classify("Release: fix notes", ""), (false, true, None));
        assert_eq!(classify("feat: prefix handling", ""), (false, false, None));
        // merge subject classified on the body first line, no exclusion
        let (s, b, s2) = classify(
            "Merge pull request #7 from u/b",
            "\nrelease: fix it\n\nmore",
        );
        assert_eq!(
            (s, b, s2.as_deref()),
            (false, true, Some("release: fix it"))
        );
        let (s, _, s2) = classify("Merge pull request #7 from u/b", "");
        assert_eq!((s, s2.as_deref()), (false, Some("")));
    }

    #[test]
    fn numstat_rename_forms() {
        let out = "3\t1\tsrc/{old => new}/a.rs\n-\t-\tbin.png\n2\t2\told.txt => new.txt\n1\t0\tdir/{a => }/f\n";
        let f = parse_numstat(out).unwrap();
        let paths: Vec<&str> = f.iter().map(|x| x.path.as_str()).collect();
        assert_eq!(paths, vec!["src/new/a.rs", "bin.png", "new.txt", "dir//f"]);
        assert_eq!((f[1].added, f[1].deleted), (0, 0));
    }

    #[test]
    fn hunk_ranges_skip_pure_additions() {
        let d = "diff --git a/x b/x\n@@ -3 +3 @@\n-a\n+b\n@@ -10,0 +11,2 @@\n+c\n@@ -20,4 +22 @@\n";
        assert_eq!(old_side_ranges(d), vec![(3, 3), (20, 23)]);
    }

    #[test]
    fn porcelain_counts_header_lines_only() {
        let a = "a".repeat(40);
        let b = "b".repeat(40);
        let out = format!("{a} 1 1 2\nauthor x\n\tcode {b} 1 1\n{a} 2 2\n\tx\n{b} 9 3 1\n\ty\n");
        assert_eq!(count_porcelain(&out), vec![(a, 2), (b, 1)]);
    }

    #[test]
    fn buckets_match_baseline_table() {
        assert_eq!(bucket("falcon-v0.35.0").as_deref(), Some("v0.x"));
        assert_eq!(bucket("falcon-v1.49.0").as_deref(), Some("v1.0-49"));
        assert_eq!(bucket("falcon-v1.140.0").as_deref(), Some("v1.140"));
        assert_eq!(
            bucket("unreleased").as_deref(),
            Some("post-v1.140 (unreleased)")
        );
        // BASELINE QUIRKS, pinned:
        assert_eq!(bucket("falcon-v1.141.0"), None);
        assert_eq!(bucket("falcon-v2.5.0").as_deref(), Some("v1.0-49"));
    }

    #[test]
    fn most_common_is_stable() {
        let mut c = OrderedCounter::default();
        for (k, n) in [("x", 1), ("y", 2), ("z", 1), ("w", 2)] {
            c.add(k, n);
        }
        let keys: Vec<String> = c.most_common().into_iter().map(|p| p.0).collect();
        assert_eq!(keys, vec!["y", "w", "x", "z"]);
    }

    #[test]
    fn data_files_parse_and_validate() {
        let cur = parse_list(CURATED_FIXES_TXT);
        assert_eq!(cur, vec!["98d9b95", "26b7199", "efc0dbf"]);
        validate_curated(&cur).unwrap();
        let ign = parse_list(IGNORE_REVS_TXT);
        assert_eq!(ign, vec!["4c5feda"]);
        validate_ignore_revs(&ign).unwrap();
        assert!(validate_curated(&["98d9b95a".into()]).is_err());
        assert!(validate_ignore_revs(&["HEAD~1".into()]).is_err());
    }
}
