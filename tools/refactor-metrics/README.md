# refactor-metrics: fix-of-a-fix detector (frozen instrument)

This tool counts **fix-of-a-fix chains per 100 merges** on a repository's
first-parent history. It is the measurement instrument for relay's
pre-registered exploratory arm of the experiment in
pulseengine/pulseengine.eu#294. The experiment window is 2026-11-01 to
2027-01-31.

## Status: FROZEN at 1.0.0

Version **1.0.0 is the frozen method**. It is a port of the Python scripts
that produced the baseline (a dataset pass and a report pass), run on relay
`origin/main` @`55af1e9`. On that input the port is equivalent to the
baseline:

- the report is byte-identical (`baseline/analyze-out-55af1e9.txt`);
- every per-commit classification, window and numstat matches (500 commits);
- every raw blame edge matches (471);
- every `(fix, origin, file, lines)` pair matches (24);
- every chain depth matches.

Run `cargo test --release -- --ignored`. The baseline outputs are committed
under `baseline/`. The Python scripts are not.

Any change to the code, to `curated-fixes.txt` or to `ignore-revs.txt`
changes the measurement. That includes a "bug fix". Such a change **must**:

1. bump the version in `Cargo.toml`;
2. be reported as an instrument change alongside any figure it produces;
3. be re-run on the baseline input, with the old and new figures stated side
   by side.

Do not absorb a change into the method. A figure from an unfrozen or modified
instrument is not comparable with the baseline. The content hash of this
directory, recorded at freeze time, identifies the frozen instrument.

## Method (what the code does)

The universe is the **whole first-parent history** of `--ref`, oldest first
(`git log --first-parent`).

**1. Classification.** Each commit is classified by its subject.

- **strict**: the subject matches `(?i)^(fix|hotfix|revert)(\(|!|:)|^Revert `.
- **broad**: the commit is strict, or the subject matches the keyword set
  `(?i)\b(fix|fixes|fixed|fixing|regression|revert|repair|broke|broken|flake|flaky|stall(s|ed)?|bug|hotfix|re-?fix|wrong)\b`
  and does not start with the case-sensitive `^(Merge|plan|release|req|verify)`.
- A `Merge pull request` subject is classified on the **first line of the
  body** instead, and the exclusion list is not applied.
- A commit is a **FIX** if it is strict, or if its `sha[:7]` is in
  `curated-fixes.txt`. The automated figure uses no curated list.

**2. Numstat.** For every commit except the oldest in the universe, the tool
runs `git diff --numstat -M C^1 C`. A brace rename `{a => b}` is rewritten to
the new path; for a bare `old => new` the last component is kept. A binary
`-` counts as 0.

**3. Blame.** This step runs for every **broad** commit `F`; it is gated on
broad, not on FIX. For each file with deleted lines greater than 0:

- `git diff -U0 -w F^1 F -- <path>` gives the old-side ranges `(start,
  start+n-1)` of hunks with `n > 0`;
- `git blame -w --first-parent --line-porcelain --ignore-rev <each
  ignore-revs entry, in file order> -L <ranges...> F^1 -- <path>` attributes
  each of those old lines to a first-parent commit `O`.

Each `(F, O, path, lines)` is one raw edge.

**4. Pairs.** A raw edge becomes a pair contribution when all of these hold:

- `F != O`;
- `O` is in the universe;
- `F` does not start with any ignore-revs entry;
- `F` and `O` are both FIX.

Contributions are summed per `(F, O)`. A pair needs at least `--min-lines`
lines in total; the default is 1.

**5. Depth.** For every FIX, oldest first, depth is
`1 + max(depth of its pair origins)`, or 1 if it has none.

**6. Windows.** Each commit is assigned to the first `falcon-v*` tag, in
version order, whose `git rev-list --first-parent prev..tag` contains it.
Otherwise it is `unreleased`. Two groupings are emitted:

- per raw tag;
- the baseline's hard-coded buckets, in the byte-identical
  `--format baseline-text` report.

**7. Scope.** `--merges N` (default 500), or `--since/--until`, selects the
commits that count:

- as merges (the denominator);
- as FIX commits;
- as the fixing side of a pair;
- in the depth histogram, the windows and the hotspots.

Classification, blame, depth and windows are always computed over the whole
history, so **an origin may lie before the scope**. At 55af1e9 the history is
exactly 500 commits, so the scope equals the universe, and this is the only
reading under which the baseline is reproduced exactly. For a windowed run
this is a definition, not something the baseline exercised. It is stated here
so that it is pre-registered.

**8. Outputs.** For both `automated` and `curated`, `--format json` reports:

- merges, fixes, fix share;
- raw pairs, re-fixing commits (distinct `F`), and both per 100 merges;
- the chain-depth histogram;
- the per-tag windows and the baseline-bucket windows;
- per-file hotspots (FIX commits, re-fix pairs, FIX churn, all commits);
- every git command that exited non-zero (`git_failures`).

`--dataset` adds the per-commit data and the raw edges.

**Git pinning.** git config that could move hunk boundaries, quoting or blame
attribution is pinned to git's defaults with `-c`. The baseline ran with
these unset, which is the same thing. The JSON records `git --version`. The
baseline used `git version 2.54.0 (Apple Git-157)`.

## Restated baseline (relay origin/main @55af1e9, 500 merges)

| figure                         | automated (no curation) | with curated list (3) |
|--------------------------------|------------------------:|----------------------:|
| FIX commits                    | 75 (15.0%)              | 78 (15.6%)            |
| raw fix-of-fix pairs           | 19 (3.8 / 100 merges)   | 24 (4.8 / 100 merges) |
| re-fixing commits              | 15 (3.0 / 100 merges)   | 19 (3.8 / 100 merges) |
| chain depth histogram          | {1: 60, 2: 10, 3: 4, 4: 1} | {1: 59, 2: 12, 3: 6, 4: 1} |

These are **raw instrument output**. Two earlier figures are not instrument
output and cannot be reproduced by it:

- "20 true pairs / 16 re-fixing commits" in the analysis notes is a
  precision-corrected estimate from manual line-level review;
- the per-window "re-fix (blame, true)" table is the same kind of estimate.

## Known error modes

**Precision.** About 75%. A manual line-level review of 16 sampled raw pairs
found 12 true and 4 false positives: a rivet pin bump, a comment edit, and two
gap-list data edits. A raw pair means "a fix rewrote lines last written by a
fix". It does not mean the second fix repaired the first one's defect.

**Recall.** Poor. Known misses:

- **Guard-only fixes.** A fix that only adds lines (a new check or guard) has
  no old-side lines, so nothing is blamed.
- **Feat-introduced defects.** The origin must be a FIX. A defect introduced
  by a `feat:` commit and fixed twice counts only from the second fix.
- **Unprefixed fixes.** A fix without a `fix`/`hotfix`/`revert` prefix is
  missed unless it is curated.
- **Renames.** The `-U0` diff runs on the new path only, so a fix inside a
  renamed file contributes nothing.
- **Quoted paths.** git C-quotes a path with special characters, and the
  quoted string matches no file.

**Reformat shadowing.** A bulk reformat re-attributes every line it touches.
`ignore-revs.txt` holds `4c5feda`, the repo-wide `cargo fmt`, which is itself
a `fix(ci):` commit. Without it the 55af1e9 run gives 31 raw pairs instead of
24 (`baseline/pairs-55af1e9-no-ignore-rev.json`). A future bulk reformat
inside the window would need the same treatment, and that is an instrument
change.

## Baseline quirks reproduced deliberately (do not "fix")

- Blame is gated on **broad**, not FIX. A curated commit whose subject has no
  fix keyword would never be blamed as a fixer. All three current entries are
  broad.
- A curated entry matches only `sha[:7]` exactly, so entries must be 7
  characters. The tool rejects other lengths instead of silently never
  matching them.
- The baseline hard-coded the fixer exclusion as `startswith('4c5feda')`.
  Here it is driven by `ignore-revs.txt`, which is identical for the frozen
  list.
- The broad exclusion is case-sensitive and is skipped for merge-subject
  commits.
- **Windows use the repo's tag set at run time**, not the tag set at the ref.
  New tags can reassign commits that were `unreleased`.
  The equivalence test therefore requires exactly the baseline tag set
  (`baseline/falcon-tags-55af1e9.txt`). It refuses to compare in any other
  clone, and that refusal is not a method defect.
- The baseline buckets are hard-coded through falcon-v1.140. The major version
  is ignored unless it is 0, and **v1.141+ has no bucket** (`null`; the
  baseline script would have crashed). The raw per-tag windows are the
  supported grouping for the experiment window.
- git exit status is ignored and only stdout is used, decoded with UTF-8
  replacement and universal newlines, then split with Python `splitlines`
  semantics. Failures are recorded in `git_failures` and on stderr; there
  were 0 on the baseline.
- The baseline-text header always says `origin/main`.

Input validation added beyond the baseline does not change any figure it
could produce. The tool exits 1, where the baseline would have silently
measured nothing, when:

- an ignore-rev does not resolve;
- a list entry is malformed;
- the scope is empty.

## Running

```bash
cd tools/refactor-metrics
cargo build --release
B=target/release/refactor-metrics

# baseline (restated)
$B --repo ../.. --ref 55af1e9 --merges 500 --format json

# experiment window (committer date = when the commit landed on main; inclusive)
$B --repo ../.. --ref origin/main --since 2026-11-01 --until 2027-01-31 --format json
```

For the experiment window:

- **Scope is selected by committer date.** For 15 of the 500 baseline
  commits, `%cd` and `%ad` differ. Both dates are in the JSON.
- `git fetch origin --tags` first. Windows depend on the tags present.
- Report both `automated` and `curated`. The curated list is frozen at the
  baseline's three entries, and window commits are never curated
  retroactively.

CLI: `--version`/`-V` prints `refactor-metrics <semver>` and exits 0. `--help`
exits 0. An unknown or invalid flag exits 2. A runtime error exits 1.

## Layout

| path | role |
|------|------|
| `src/lib.rs` | the method (dataset, pairs, depth, windows, hotspots, baseline report) |
| `src/pycompat.rs` | Python string semantics the baseline depended on |
| `src/main.rs` | CLI |
| `curated-fixes.txt`, `ignore-revs.txt` | versioned data files (embedded at build; `--curated-fixes`/`--ignore-revs` override) |
| `baseline/` | the baseline run's outputs, used by the equivalence test |
| `tests/synthetic.rs` | behaviour on a synthetic repo built per test |
| `tests/equivalence.rs` | `#[ignore]`d: equivalence on relay @55af1e9 |

This tool has its own `[workspace]`. It is not a root-workspace member, and
no CI workflow builds it.
