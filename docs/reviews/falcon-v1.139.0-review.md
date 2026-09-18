# falcon-v1.139.0 — independent review record

Evidence for `FV-RELAY-REVIEW-139` / `SWREQ-RELAY-REVIEW-P01`. Written into the
repository on purpose: the findings were delivered as GitHub PR comments, and a
release's audit trail must not depend on comments that can be edited or a PR
that can be deleted.

- **Candidate reviewed:** `falcon-v1.138.0..9f1df80` — 30 commits, 186 files,
  +15 861 / −5 559.
- **Reviewer:** `/code-review ultra`, maintainer-triggered, 2026-09-18.
- **How it was run:** the full candidate exceeds what the tool accepts as one
  diff (~9.5 k of those lines are Bazel lockfile churn), so it was split
  against a base branch pinned at the previous tag — PR #458 (gate tooling and
  CI workflows, 19 files) and PR #459 (the 28 source files with 50+ changed
  lines). Neither was ever merged; both are closed.

## Findings and dispositions (9)

### Fixed before the tag — PR #460, merged as `f9ea446`

1. **Disposition: FIXED.** `soak.yml` interpolated a `workflow_dispatch` input
   directly into a `run:` block. Actions expands `${{ }}` before bash sees the
   text, so an actor with dispatch rights could end the assignment and run
   commands with the job's environment. Inputs are now bound through `env:`.
2. **Disposition: FIXED.** The analytic soak tier lost its rungs to
   `shell: bash`'s `-e`: the bench exits 1 on a FAIL verdict, so the step died
   before the `FAIL=1; continue` arm could walk the remaining durations — a job
   whose purpose is publishing a number produced a red step and no number.
3. **Disposition: FIXED.** `check-evidence.rs` decided whether a `$VAR` in an
   artifact step was an unfilled placeholder by reading the tool's **own**
   environment, so the census disagreed with itself across machines.
4. **Disposition: FIXED.** The two `relay_` scanners disagreed on an identifier
   boundary, so `my_relay_foo` synthesised a phantom `relay-foo` root in one.
5. **Disposition: FIXED.** `setup-gh` cached under `$RUNNER_TEMP`, which Actions
   empties at both ends of every job, so `gh` was re-downloaded twice per
   fleet-status tick.
6. **Disposition: FIXED.** `release.yml`'s `--generate-notes` fallback could not
   fire. It is now a loud failure — deleting the dead branch outright would have
   left a silent no-op if the notes body were ever empty.

### Filed into a named release — v1.140, maintainer decision 2026-09-18

7. **Disposition: FILED (v1.140).** Losing the battery sense **in flight** raised
   no failsafe: on `None` the supervisor recorded absence but the failsafe reads
   latches that only advance when the estimator is updated, and the presence flag
   is consulted only by the pre-arm gate. A vehicle that armed healthy and lost
   its ADC would fly until the pack was flat. This is the in-flight half of #413.
   Reproduced (still `Loiter` after 4 s of silence), fixed on the v1.140 branch,
   tracked as `SWREQ-FALCON-BATTERY-P03` + `FV-FALCON-BATTERY-003`.

### Dismissed, each with the measurement

8. **Disposition: DISMISSED.** "`read_battery_v` → `Option<f32>` breaks two impls
   and a call site." On main both impls already return `Option<f32>`; their
   diffs are under 50 lines, so the slice excluded them and the reviewer saw a
   signature change with no updated callers.
9. **Disposition: DISMISSED.** "Three gate scripts are never invoked", and
   "`check-component-claims.rs` exits 1 for six of eight components;
   `capability-reachability.rs` exits 2 on a missing doc." The scripts are cited
   in rivet artifact steps (`FV-FALCON-CLAIMS-001/003/004`,
   `FV-RELAY-EVIDENCE-001`, `FV-FALCON-WASMEQ-002`) — the slice excluded
   `artifacts/`. Measured on main at `9f1df80`: all three exit 0. The slice
   lacked the `wasm/cm/*/Cargo.toml` rewrites and `docs/CAPABILITY-REACHABILITY.md`
   that make them pass.

**Lesson recorded:** a partial slice invites confident findings about files it
cannot see. Three of nine findings were slicing artefacts. A future split must
either be closed under the changes it references, or say plainly in its own body
that it is partial.

## What this review does NOT cover

The reviewed diff excluded rivet artifact YAML, the `Cargo.toml` component
descriptions, `wasm/`, `tests/`, `examples/`, docs and lockfiles. The claims
**text** a partner reads was therefore checked mechanically
(`check-component-claims.rs`, `capability-reachability.rs`) and not by a reader.
That is a weaker guarantee, and it is recorded here rather than implied away.
