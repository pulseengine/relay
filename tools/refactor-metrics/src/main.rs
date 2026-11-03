//! refactor-metrics CLI. See README.md; `--help` for usage.

use refactor_metrics::*;
use serde_json::{json, Map, Value};
use std::path::PathBuf;
use std::process::ExitCode;

const USAGE: &str = "\
refactor-metrics - frozen fix-of-a-fix (re-fix) detector over first-parent history

USAGE:
    refactor-metrics [OPTIONS]

OPTIONS:
    --repo <PATH>            git repository to measure [default: .]
    --ref <REF>              ref whose first-parent history is measured [default: origin/main]
    --merges <N>             scope = the last N first-parent commits [default: 500]
    --since <YYYY-MM-DD>     scope = commits whose COMMITTER date is >= this (inclusive)
    --until <YYYY-MM-DD>     scope = commits whose COMMITTER date is <= this (inclusive)
                             (--since/--until replace --merges; they cannot be combined)
    --curated-fixes <PATH>   curated FIX list [default: built-in copy of curated-fixes.txt]
    --ignore-revs <PATH>     blame ignore-revs list [default: built-in copy of ignore-revs.txt]
    --min-lines <N>          minimum overlapping lines for a pair [default: 1]
    --format <FMT>           text | json | baseline-text [default: text]
                             baseline-text = byte-identical to the baseline analyze.py report
    --mode <MODE>            for baseline-text: curated | automated [default: curated]
    --dataset                json only: also emit per-commit data and raw blame edges
    -h, --help               print this help and exit 0
    -V, --version            print the version and exit 0

The whole first-parent history of --ref is always analysed (classification,
blame, chain depth, tag windows); the scope only selects which commits are
counted as merges and as re-fixing commits. Origins may lie before the scope.

EXIT CODES: 0 ok, 1 runtime error, 2 usage error.
";

struct Opts {
    repo: PathBuf,
    git_ref: String,
    merges: Option<usize>,
    since: Option<String>,
    until: Option<String>,
    curated: Option<PathBuf>,
    ignore: Option<PathBuf>,
    min_lines: u64,
    format: String,
    mode: String,
    dataset: bool,
}

enum Parsed {
    Run(Box<Opts>),
    Exit(ExitCode),
}

fn usage_err(msg: &str) -> Parsed {
    eprintln!("refactor-metrics: {msg}\n(see --help)");
    Parsed::Exit(ExitCode::from(2))
}

fn is_date(s: &str) -> bool {
    let b = s.as_bytes();
    b.len() == 10
        && b[4] == b'-'
        && b[7] == b'-'
        && b.iter()
            .enumerate()
            .all(|(i, c)| i == 4 || i == 7 || c.is_ascii_digit())
}

fn parse_args(args: Vec<String>) -> Parsed {
    let mut o = Opts {
        repo: PathBuf::from("."),
        git_ref: "origin/main".into(),
        merges: None,
        since: None,
        until: None,
        curated: None,
        ignore: None,
        min_lines: 1,
        format: "text".into(),
        mode: "curated".into(),
        dataset: false,
    };
    let mut it = args.into_iter();
    while let Some(arg) = it.next() {
        let (flag, inline) = match arg.split_once('=') {
            Some((f, v)) if f.starts_with("--") => (f.to_string(), Some(v.to_string())),
            _ => (arg.clone(), None),
        };
        let takes_value = matches!(
            flag.as_str(),
            "--repo"
                | "--ref"
                | "--merges"
                | "--since"
                | "--until"
                | "--curated-fixes"
                | "--ignore-revs"
                | "--min-lines"
                | "--format"
                | "--mode"
        );
        let val = if takes_value {
            match inline.or_else(|| it.next()) {
                Some(v) => v,
                None => return usage_err(&format!("{flag} needs a value")),
            }
        } else {
            if inline.is_some() {
                return usage_err(&format!("{flag} takes no value"));
            }
            String::new()
        };
        match flag.as_str() {
            "-h" | "--help" => {
                print!("{USAGE}");
                return Parsed::Exit(ExitCode::SUCCESS);
            }
            "-V" | "--version" => {
                println!("refactor-metrics {VERSION}");
                return Parsed::Exit(ExitCode::SUCCESS);
            }
            "--dataset" => o.dataset = true,
            "--repo" => o.repo = PathBuf::from(val),
            "--ref" => o.git_ref = val,
            "--merges" => match val.parse::<usize>() {
                Ok(n) if n > 0 => o.merges = Some(n),
                _ => {
                    return usage_err(&format!("--merges: positive integer expected, got {val:?}"))
                }
            },
            "--since" | "--until" => {
                if !is_date(&val) {
                    return usage_err(&format!("{flag}: YYYY-MM-DD expected, got {val:?}"));
                }
                if flag == "--since" {
                    o.since = Some(val)
                } else {
                    o.until = Some(val)
                }
            }
            "--curated-fixes" => o.curated = Some(PathBuf::from(val)),
            "--ignore-revs" => o.ignore = Some(PathBuf::from(val)),
            "--min-lines" => match val.parse::<u64>() {
                Ok(n) => o.min_lines = n,
                Err(_) => return usage_err(&format!("--min-lines: integer expected, got {val:?}")),
            },
            "--format" => {
                if !matches!(val.as_str(), "text" | "json" | "baseline-text") {
                    return usage_err(&format!("--format: text|json|baseline-text, got {val:?}"));
                }
                o.format = val
            }
            "--mode" => {
                if !matches!(val.as_str(), "curated" | "automated") {
                    return usage_err(&format!("--mode: curated|automated, got {val:?}"));
                }
                o.mode = val
            }
            _ => return usage_err(&format!("unknown argument {arg:?}")),
        }
    }
    if o.merges.is_some() && (o.since.is_some() || o.until.is_some()) {
        return usage_err("--merges cannot be combined with --since/--until");
    }
    Parsed::Run(Box::new(o))
}

fn load_list(path: &Option<PathBuf>, builtin: &str) -> Result<(Vec<String>, String), String> {
    match path {
        None => Ok((parse_list(builtin), "built-in".into())),
        Some(p) => std::fs::read_to_string(p)
            .map(|t| (parse_list(&t), p.display().to_string()))
            .map_err(|e| format!("{}: {e}", p.display())),
    }
}

fn per100(x: usize, n: usize) -> f64 {
    100.0 * x as f64 / n as f64
}

fn window_json(rows: &[WindowRow]) -> Value {
    Value::Array(
        rows.iter()
            .map(|w| {
                json!({
                    "window": w.name, "start": w.start, "end": w.end,
                    "merges": w.n, "fixes": w.fix, "refixing_commits": w.refix,
                    "fix_churn": w.churn,
                })
            })
            .collect(),
    )
}

fn mode_json(ds: &Dataset, a: &Analysis) -> Value {
    let c = &ds.commits;
    let m = a.scope.len();
    let pairs: Vec<Value> = a
        .pairs_sorted()
        .iter()
        .map(|p| {
            let files: Map<String, Value> = p.files.items().into_iter().map(|(k, v)| (k, json!(v))).collect();
            json!({"fix": c[p.fix].sha, "origin": c[p.origin].sha, "lines": p.files.total(), "files": files})
        })
        .collect();
    let hist: Map<String, Value> = a
        .depth_histogram()
        .into_iter()
        .map(|(k, v)| (k.to_string(), json!(v)))
        .collect();
    let depth: Map<String, Value> = a
        .depth
        .iter()
        .enumerate()
        .filter_map(|(i, d)| d.map(|d| (c[i].sha.clone(), json!(d))))
        .collect();
    let mut seen = std::collections::HashSet::new();
    let mut hot = Vec::new();
    for (p, n) in a.fc.most_common() {
        seen.insert(p.clone());
        hot.push(
            json!({"path": p, "fix_commits": n, "refix_pairs": a.rf.get(&p),
                        "fix_churn": a.fch.get(&p), "all_commits": a.tc.get(&p)}),
        );
    }
    for (p, n) in a.rf.most_common() {
        if !seen.contains(&p) {
            hot.push(
                json!({"path": p, "fix_commits": a.fc.get(&p), "refix_pairs": n,
                            "fix_churn": a.fch.get(&p), "all_commits": a.tc.get(&p)}),
            );
        }
    }
    let fixes = a.fixes();
    let refix = a.refixing_commits();
    json!({
        "merges": m,
        "fixes": fixes,
        "fix_share": fixes as f64 / m as f64,
        "raw_pairs": a.pairs.len(),
        "refixing_commits": refix,
        "raw_pairs_per_100_merges": per100(a.pairs.len(), m),
        "refixing_commits_per_100_merges": per100(refix, m),
        "fixes_per_100_merges": per100(fixes, m),
        "chain_depth_histogram": hist,
        "pairs": pairs,
        "depth_universe": depth,
        "windows_by_tag": window_json(&a.tag_windows),
        "windows_baseline_buckets": window_json(&a.buckets),
        "hotspots": hot,
    })
}

fn summary_text(
    o: &Opts,
    ref_sha: &str,
    scope_desc: &str,
    auto: &Analysis,
    cur: &Analysis,
    failures: usize,
) -> String {
    use std::fmt::Write;
    let mut s = String::new();
    let _ = writeln!(
        s,
        "refactor-metrics {VERSION}  ref {} @{}  scope: {scope_desc}",
        o.git_ref,
        &ref_sha[..12]
    );
    let m = cur.scope.len();
    let _ = writeln!(
        s,
        "merges in scope: {m}  (universe: {} first-parent commits)",
        cur.universe
    );
    for a in [auto, cur] {
        let f = a.fixes();
        let r = a.refixing_commits();
        let _ = writeln!(
            s,
            "  {:<10} fixes={f} ({:.1}%)  raw_pairs={} ({:.2}/100 merges)  refixing_commits={r} ({:.2}/100 merges)  depth={:?}",
            a.mode,
            per100(f, m),
            a.pairs.len(),
            per100(a.pairs.len(), m),
            per100(r, m),
            a.depth_histogram()
        );
    }
    if failures > 0 {
        let _ = writeln!(
            s,
            "  WARNING: {failures} git command(s) exited non-zero (see --format json)"
        );
    }
    s
}

fn run(o: Opts) -> Result<(), String> {
    let (curated, curated_src) = load_list(&o.curated, CURATED_FIXES_TXT)?;
    let (ignore, ignore_src) = load_list(&o.ignore, IGNORE_REVS_TXT)?;
    validate_curated(&curated)?;
    validate_ignore_revs(&ignore)?;
    let mut git = Git::new(&o.repo);
    let ref_sha = git.resolve_commit(&o.git_ref).ok_or_else(|| {
        format!(
            "cannot resolve --ref {:?} in {}",
            o.git_ref,
            o.repo.display()
        )
    })?;
    for r in &ignore {
        if git.resolve_commit(r).is_none() {
            return Err(format!(
                "ignore-rev {r:?} does not resolve to a commit in {}",
                o.repo.display()
            ));
        }
    }
    let ds = build_dataset(&mut git, &ref_sha, &ignore)?;
    let n = ds.commits.len();
    let (scope, scope_desc, scope_json): (Vec<usize>, String, Value) =
        if o.since.is_some() || o.until.is_some() {
            let lo = o.since.clone().unwrap_or_default();
            let hi = o.until.clone().unwrap_or_else(|| "9999-99-99".into());
            let v: Vec<usize> = (0..n)
                .filter(|&i| ds.commits[i].cdate >= lo && ds.commits[i].cdate <= hi)
                .collect();
            (
                v,
                format!(
                    "committer date {}..{}",
                    o.since.as_deref().unwrap_or("-"),
                    o.until.as_deref().unwrap_or("-")
                ),
                json!({"kind": "committer-date", "since": o.since, "until": o.until}),
            )
        } else {
            let k = o.merges.unwrap_or(500);
            let v: Vec<usize> = (n.saturating_sub(k)..n).collect();
            (
                v,
                format!("last {k} first-parent commits"),
                json!({"kind": "last-n", "requested": k}),
            )
        };
    if scope.is_empty() {
        return Err("scope is empty: no first-parent commits selected (an empty scope is not a zero measurement)".into());
    }
    let auto = analyze(&ds, &scope, &[], &ignore, o.min_lines, "automated");
    let cur = analyze(&ds, &scope, &curated, &ignore, o.min_lines, "curated");

    match o.format.as_str() {
        "baseline-text" => {
            let a = if o.mode == "automated" { &auto } else { &cur };
            print!("{}", baseline_text(&ds, a, o.min_lines));
        }
        "text" => print!(
            "{}",
            summary_text(&o, &ref_sha, &scope_desc, &auto, &cur, git.failures.len())
        ),
        _ => {
            let first = &ds.commits[scope[0]];
            let last = &ds.commits[*scope.last().unwrap()];
            let mut scope_json = scope_json;
            scope_json["merges"] = json!(scope.len());
            scope_json["first"] =
                json!({"sha": first.sha, "date": first.date, "cdate": first.cdate});
            scope_json["last"] = json!({"sha": last.sha, "date": last.date, "cdate": last.cdate});
            let mut out = json!({
                "tool": "refactor-metrics",
                "version": VERSION,
                "git_version": git.version(),
                "repo": o.repo.display().to_string(),
                "ref": o.git_ref,
                "ref_sha": ref_sha,
                "universe_commits": n,
                "scope": scope_json,
                "min_lines": o.min_lines,
                "curated_fixes": {"source": curated_src, "entries": curated},
                "ignore_revs": {"source": ignore_src, "entries": ignore},
                "git_failures": git.failures,
                "modes": {"automated": mode_json(&ds, &auto), "curated": mode_json(&ds, &cur)},
            });
            if o.dataset {
                let commits: Vec<Value> = ds
                    .commits
                    .iter()
                    .map(|c| {
                        let files: Vec<Value> = c
                            .files
                            .iter()
                            .map(|f| json!([f.path, f.added, f.deleted]))
                            .collect();
                        json!({"sha": c.sha, "date": c.date, "cdate": c.cdate, "subj": c.subj,
                               "subj2": c.subj2, "pr": c.pr, "strict": c.strict, "broad": c.broad,
                               "window": c.window, "files": files})
                    })
                    .collect();
                let edges: Vec<Value> = ds
                    .edges
                    .iter()
                    .map(|e| json!([e.fix, e.origin, e.path, e.lines]))
                    .collect();
                out["dataset"] = json!({"commits": commits, "edges": edges});
            }
            println!(
                "{}",
                serde_json::to_string_pretty(&out).map_err(|e| e.to_string())?
            );
        }
    }
    if !git.failures.is_empty() {
        eprintln!(
            "refactor-metrics: warning: {} git command(s) exited non-zero; the baseline method ignores git exit status (output kept as-is)",
            git.failures.len()
        );
    }
    Ok(())
}

fn main() -> ExitCode {
    match parse_args(std::env::args().skip(1).collect()) {
        Parsed::Exit(c) => c,
        Parsed::Run(o) => match run(*o) {
            Ok(()) => ExitCode::SUCCESS,
            Err(e) => {
                eprintln!("refactor-metrics: error: {e}");
                ExitCode::from(1)
            }
        },
    }
}
