#!/usr/bin/env python3
"""
Extract and run all falcon verification steps from rivet artifacts.

This is the reference implementation that spar's
tools/post_verification_comment.py-equivalent should follow for relay:

  1. List sw-verification artifacts matching a rivet filter.
  2. For each, rivet-get the full artifact JSON.
  3. Extract fields.steps[].run as shell commands.
  4. Skip commands marked with `# bench-only` or `# manual` (those need
     hardware / a running sim / etc.) — CI runs only the rest.
  5. Run each remaining command, capture exit code + duration.
  6. Aggregate pass/fail per artifact.
  7. Emit a Markdown summary suitable for posting as a PR comment.

The bench-only convention: any `run:` whose shell command contains the
substring `# bench-only` or `# manual` (case-insensitive) is reported
as "Skipped (bench-only)" rather than executed. Use this for steps
that require a real bench (gz sim, PX4-SITL, HackRF, etc.).

Usage:
    python3 scripts/run-falcon-verification.py
    python3 scripts/run-falcon-verification.py --filter '(has-tag "v0.1")'
    python3 scripts/run-falcon-verification.py --type sw-verification
    python3 scripts/run-falcon-verification.py --dry-run
    python3 scripts/run-falcon-verification.py --markdown
"""

import argparse
import json
import re
import subprocess
import sys
import time
from typing import Any

# Each pattern matches a command shape that needs infra the standard
# `Verification gate (rivet-driven)` ubuntu-latest runner does not
# provide today. Heuristic — rivet strips shell `# bench-only`
# comments at the YAML→JSON boundary, so we identify bench-only steps
# by command shape rather than a comment marker. Add to the list as
# new infra-needs appear; remove when CI gains the tool.
BENCH_PATTERNS = [
    # NOTE (v1.116): `cargo kani -p <crate>` is no longer blindly bench-only —
    # it is CROSS-CHECKED against the kani.yml CI matrix (kani_matrix_crates):
    # in-matrix ⇒ "enforced-by-kani-gate" (the required `Kani gate` check runs
    # it on every PR), out-of-matrix ⇒ FAIL (an orphaned proof CI never runs —
    # the traceability-audit finding). Only a kani step WITHOUT `-p` (run from
    # inside a crate dir) still falls through to bench-only.
    re.compile(r"\bcargo\s+kani\b(?!.*\s-p\s)"),      # kani without -p: bench-only
    re.compile(r"\bcargo\s+\+nightly\s+miri\b"),      # miri nightly component
    re.compile(r"^\s*MIRIFLAGS="),                    # same family
    re.compile(r"\brustup\s+component\s+add\s+miri"), # same family
    re.compile(r"--backend=(hackrf|mavlink|gazebo)\b"),  # needs PX4 / HackRF / gz sim
    re.compile(r"--preset=px4-sitl\b"),               # needs PX4-Autopilot or live PX4
    re.compile(r"\$WITNESS\b"),                       # template env-var placeholder
    re.compile(r"\bgz\s+sim\b"),                      # Gazebo Sim install
    re.compile(r"falcon-sitl-gz/plugins/"),           # gz custom-plugin bench scripts (build + run gz, not in CI)
    re.compile(r"\bmake\s+px4_sitl\b"),               # PX4-Autopilot install
    re.compile(r"\bbazel\s+(test|build|run)\b"),      # bazel not provisioned in the gate job
    # `cargo llvm-cov --workspace` has its OWN dedicated CI job (the `llvm-cov`
    # check); running it again inside the gate is redundant and flaky (heavy
    # full-workspace instrumentation intermittently rc=101). Skip here — the
    # dedicated job is the coverage gate.
    re.compile(r"\bcargo\s+llvm-cov\b"),
    # `meld` (the PulseEngine fusion tool) is a bench tool, not provisioned in
    # the CI gate runner; its fuse/inspect demos run locally (like gz).
    re.compile(r"\bmeld\s+(fuse|inspect)\b"),
    re.compile(r"\bmeld-fuse-cascade\b"),
    # the embedded ARM build + emulator are bench tools: the thumbv7em target
    # toolchain/linker and Renode/QEMU are not provisioned in the CI gate.
    re.compile(r"thumbv7em"),
    re.compile(r"\bbuild-cortex-m\b"),
    re.compile(r"\brenode\b"),
    re.compile(r"\bspar\s+\w"),                       # spar not on the gate runner
    re.compile(r"^\s*cd\s+~"),                        # tilde-expanded path = not portable
    re.compile(r"/Users/[^/]+/"),                     # developer-machine absolute path
    re.compile(r"/tmp/falcon-spar-wit"),              # temp dir created only by a bench-only spar step
    re.compile(r"\bgh\s+attestation\s+verify\b"),     # needs gh sigstore TUF root init
    # v0.18.x — gz-transport-rs's --features gazebo build pulls in
    # libzmq via zeromq-src (C compile); the gate runner doesn't
    # have CMake + the toolchain set up. The default-feature
    # `cargo test -p falcon-sitl-gz` runs fine.
    re.compile(r"--features\s+gazebo\b"),
    # Steps that read bazel-build output (witness-run.json etc.) —
    # the bazel build step itself is skipped, so the read fails.
    re.compile(r"\bcat\s+bazel-bin/"),
    # `cp target/wasm32-unknown-unknown/release/...` chains depend on
    # the prior wasm32 build step which needs `rustup target add` —
    # and the build step itself (`--target wasm32-…`) needs the same.
    re.compile(r"target/wasm32-"),
    re.compile(r"--target\s+wasm32-"),
    # v1.26 — the WASM Component Model build + wasmtime run are bench tools: the
    # gate runner lacks cargo-component (wasm32-wasip2 component toolchain) and
    # wasmtime. The native falcon-core tests cover the same logic; this runs
    # locally (like meld / gz / thumbv7em).
    re.compile(r"\bcargo\s+component\b"),
    # v1.56 — the component-bundle build scripts wrap `cargo component` (absent
    # on the gate runner); same bench-tool rationale as the cargo-component line.
    re.compile(r"\bbuild-(flight-)?components?\.sh\b"),
    re.compile(r"\bwasm-tools\b"),                   # wasm-tools not on the gate runner
    re.compile(r"\bcargo\s+component\b"),             # cargo-component (wasm32-wasip2) not provisioned
    re.compile(r"\brate-loop-proof\b"),               # standalone crate; needs a pre-built .wasm argument
    re.compile(r"\bwasmtime\b"),
    re.compile(r"\bwasmtime-flight-test\b"),
]


def is_bench_only(cmd: str) -> bool:
    return any(p.search(cmd) for p in BENCH_PATTERNS)


# `cargo kani ... -p <crate>` — the form the matrix cross-check can attribute.
KANI_P_STEP = re.compile(r"\bcargo\s+kani\b.*\s-p\s+([A-Za-z0-9_-]+)")

# Explicit, TRACKED waivers for crates whose cited proofs CI cannot run yet.
# A waiver is a debt: it must carry the issue that retires it, and it is
# REPORTED in every gate run (never silently skipped). New orphans still FAIL.
KANI_MATRIX_WAIVERS = {
    # verify_peak_jerk_sound hangs CBMC (>1200s, f32 intractability) — needs
    # unwind/concretisation bounding before the crate can be gated. #260.
    "relay-traj": "https://github.com/pulseengine/relay/issues/260",
}

_KANI_MATRIX_CACHE: set[str] | None = None


def kani_matrix_crates(workflow_path: str = ".github/workflows/kani.yml") -> set[str]:
    """Crate names in the kani.yml CI matrix (the required `Kani gate` check).

    Parsed as the bare `- crate-name` items of the matrix list (entries with a
    colon are YAML mappings — steps/uses — not crates). An unreadable workflow
    returns the empty set, which FAILS every kani step loudly rather than
    silently passing (fail-closed).
    """
    global _KANI_MATRIX_CACHE
    if _KANI_MATRIX_CACHE is not None:
        return _KANI_MATRIX_CACHE
    crates: set[str] = set()
    try:
        with open(workflow_path, encoding="utf-8") as f:
            for line in f:
                m = re.match(r"^\s+-\s+([a-z0-9][a-z0-9_-]*)\s*(#.*)?$", line)
                if m and ":" not in m.group(1):
                    crates.add(m.group(1))
    except OSError:
        pass
    _KANI_MATRIX_CACHE = crates
    return crates


def rivet_list(filter_expr: str, artifact_type: str) -> list[str]:
    out = subprocess.check_output([
        "rivet", "list",
        "--type", artifact_type,
        "--filter", filter_expr,
        "--format", "json",
    ])
    data = json.loads(out)
    return [a["id"] for a in data["artifacts"]]


def rivet_get(artifact_id: str) -> dict[str, Any]:
    out = subprocess.check_output([
        "rivet", "get", artifact_id, "--format", "json",
    ])
    return json.loads(out)


def cargo_test_names_a_filter(cmd: str) -> bool:
    """True if `cmd` is a `cargo test ...` that names a SPECIFIC test filter (a
    bare token that isn't a flag, a `-p <pkg>` package, or a `--` harness arg).
    Used to enforce test-level traceability: a step naming a test must run it."""
    import shlex
    try:
        toks = shlex.split(cmd)
    except ValueError:
        return False
    if toks[:2] != ["cargo", "test"]:
        return False
    i = 2
    while i < len(toks):
        t = toks[i]
        if t == "--":
            return False  # everything after -- is harness flags, not a filter
        if t in ("-p", "--package", "--manifest-path", "--features"):
            i += 2
            continue
        if t.startswith("-"):
            i += 1
            continue
        return True  # a bare non-flag token = a test-name filter
    return False


def cargo_tests_passed(output: str) -> int:
    """Sum of `test result: ok. N passed` across all test binaries in `output`."""
    return sum(int(m) for m in re.findall(r"test result: ok\. (\d+) passed", output))


# Whole-crate steps seen this run: (artifact-id, cmd). Reported, not failed —
# see cargo_test_is_whole_crate.
WHOLE_CRATE_STEPS: list[tuple[str, str]] = []


def cargo_test_is_whole_crate(cmd: str) -> bool:
    """True if `cmd` is a `cargo test` step that runs an ENTIRE crate/workspace
    instead of naming the test(s) that verify the requirement.

    Whole-crate steps are weak evidence: they assert "something in this crate
    passes", not "THIS test verifies THIS requirement". They are also the one
    step shape that CANNOT detect evidence drift — the named-test guard in
    run_steps (`cargo_tests_passed(...) == 0`) has no name to check, so a test
    that is renamed, deleted, or `#[ignore]`d leaves the step green.

    Counted and reported (#262), not failed: the backlog was 70 steps when this
    landed, so failing immediately would block unrelated work. The point is to
    stop the count drifting back up — it had already grown ~54 -> 70 since the
    2026-07 audit. Convert to a hard failure once the backlog is worked down."""
    import shlex
    try:
        toks = shlex.split(cmd)
    except ValueError:
        return False
    if toks[:2] != ["cargo", "test"]:
        return False
    return not cargo_test_names_a_filter(cmd)


def run_steps(artifact: dict[str, Any], dry_run: bool) -> tuple[bool, list[dict]]:
    aid = artifact["id"]
    steps = artifact.get("fields", {}).get("steps") or []
    results = []
    artifact_pass = True
    for i, step in enumerate(steps):
        cmd = step["run"]
        # Kani matrix CROSS-CHECK (v1.116): a `cargo kani -p <crate>` step is
        # not executed here (CBMC is heavy), but it must be ENFORCED somewhere —
        # the required `Kani gate` CI check runs the kani.yml matrix on every
        # PR. In-matrix ⇒ counted as enforced; out-of-matrix ⇒ FAIL: the cited
        # proof is orphaned (CI never runs it), which is exactly how MIX-P05..08
        # went unenforced for ~40 releases before v1.102.
        km = KANI_P_STEP.search(cmd)
        if km:
            crate = km.group(1)
            if crate in kani_matrix_crates():
                print(f"  [enforced-by-kani-gate] {aid}: {cmd}")
                results.append({"cmd": cmd, "pass": True, "skipped": True, "rc": 0, "duration": 0.0})
            elif crate in KANI_MATRIX_WAIVERS:
                # Tracked debt, kept LOUD: reported every run with its issue.
                print(
                    f"  [ WAIVED-kani-gate] {aid}: {cmd}"
                    f" — deferred, see {KANI_MATRIX_WAIVERS[crate]}"
                )
                results.append({"cmd": cmd, "pass": True, "skipped": True, "rc": 0, "duration": 0.0})
            else:
                artifact_pass = False
                print(
                    f"  [          FAIL] (  0.00s) {aid}: {cmd}"
                    f" — ORPHANED PROOF: crate '{crate}' is not in the kani.yml matrix"
                )
                results.append({"cmd": cmd, "pass": False, "skipped": False, "rc": 1, "duration": 0.0})
            continue
        # bench-only convention: see module docstring. Skip without
        # counting as a failure; still record in the report so an
        # assessor sees what would run on a real bench.
        if is_bench_only(cmd):
            print(f"  [   skip-bench-only] {aid}: {cmd}")
            results.append({"cmd": cmd, "pass": True, "skipped": True, "rc": 0, "duration": 0.0})
            continue
        if dry_run:
            print(f"  [dry-run] {aid} step {i+1}: {cmd}")
            results.append({"cmd": cmd, "pass": True, "skipped": False, "rc": 0, "duration": 0.0})
            continue
        start = time.monotonic()
        proc = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        rc = proc.returncode
        duration = time.monotonic() - start
        passed = rc == 0
        note = ""
        # Test-level traceability: a step that NAMES a specific test must actually
        # run it. `cargo test -p X typo` exits 0 with "0 passed" (filter matched
        # nothing) — that would silently mark the requirement verified. Require
        # >=1 test to have run (pulseengine.eu#89).
        if passed and cargo_test_names_a_filter(cmd) and cargo_tests_passed(proc.stdout) == 0:
            passed = False
            note = " — named test ran 0 (renamed/removed? evidence drift)"
        # Weak-evidence census (#262). Does NOT affect pass/fail — see
        # cargo_test_is_whole_crate for why this warns rather than fails.
        if cargo_test_is_whole_crate(cmd):
            WHOLE_CRATE_STEPS.append((aid, cmd))
            note += " — WHOLE-CRATE step: names no test (#262)"
        artifact_pass = artifact_pass and passed
        status = "PASS" if passed else (f"FAIL (rc={rc})" if rc != 0 else "FAIL (0 tests)")
        print(f"  [{status:>14}] ({duration:6.2f}s) {aid}: {cmd}{note}")
        if not passed and (proc.stdout or proc.stderr):
            tail = (proc.stdout + proc.stderr).strip().splitlines()[-15:]
            for line in tail:
                print(f"        | {line}")
        results.append({"cmd": cmd, "pass": passed, "skipped": False, "rc": rc, "duration": duration})
    if not steps:
        print(f"  [   skip-no-steps] {aid}: (no steps defined)")
    return artifact_pass, results


def emit_markdown(report: list[dict]) -> str:
    total = len(report)
    skipped_no_steps = sum(1 for r in report if not r["steps"])
    # An artifact "passed" iff every executed (non-skipped) step passed
    # AND at least one step was executed (otherwise it's bench-only).
    passed = 0
    bench_only_artifacts = 0
    for r in report:
        executed = [s for s in r["steps"] if not s.get("skipped")]
        if r["steps"] and not executed:
            bench_only_artifacts += 1
        elif executed and r["pass"]:
            # only artifacts that actually RAN a step count as "passed" — a
            # no-steps artifact is tallied under skipped_no_steps, not here
            # (counting it in both made `failed` go negative ⇒ a false ❌).
            passed += 1
    failed = total - passed - skipped_no_steps - bench_only_artifacts
    icon = "✅" if failed == 0 else "❌"
    lines = [
        f"## {icon} Rivet verification gate — falcon",
        "",
        f"**{passed}/{total - skipped_no_steps - bench_only_artifacts} passed**",
        "",
        "| count |   |",
        "|-----|---|",
        f"| Passed | {passed} |",
        f"| Failed | {failed} |",
        f"| Skipped (bench-only — needs hardware / sim) | {bench_only_artifacts} |",
        f"| Skipped (no steps) | {skipped_no_steps} |",
        "",
    ]
    if failed:
        lines.append("### Failed artifacts")
        for r in report:
            executed_fails = [s for s in r["steps"] if not s.get("skipped") and not s["pass"]]
            if executed_fails:
                lines.append(f"- `{r['id']}` — {r['title']}")
                for s in executed_fails:
                    lines.append(f"  - `{s['cmd']}` (rc={s['rc']})")
        lines.append("")
    if bench_only_artifacts:
        lines.append("### Bench-only artifacts (not run by CI)")
        for r in report:
            executed = [s for s in r["steps"] if not s.get("skipped")]
            if r["steps"] and not executed:
                lines.append(f"- `{r['id']}` — {r['title']}")
        lines.append("")
    lines.append("Source of truth: `artifacts/verification/FV-FALCON-*.yaml`.")
    return "\n".join(lines)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--filter", default='(has-tag "falcon")',
                   help='rivet S-expression filter (default: falcon-tagged)')
    p.add_argument("--type", default="sw-verification",
                   help='rivet artifact type (default: sw-verification — '
                        'matches every FV-FALCON-*.yaml)')
    p.add_argument("--dry-run", action="store_true",
                   help="print commands without executing")
    p.add_argument("--markdown", action="store_true",
                   help="emit Markdown summary at end (for PR comment)")
    args = p.parse_args()

    print(f"# falcon verification gate (type: {args.type}, filter: {args.filter})")
    ids = rivet_list(args.filter, args.type)
    print(f"# {len(ids)} artifact(s) matched: {', '.join(ids)}")
    print()

    report = []
    overall_pass = True
    for aid in ids:
        a = rivet_get(aid)
        ok, step_results = run_steps(a, args.dry_run)
        overall_pass = overall_pass and ok
        report.append({
            "id": aid,
            "title": a.get("title", ""),
            "pass": ok,
            "steps": step_results,
        })

    # Weak-evidence census (#262). Printed even on a green run — a gate that
    # only speaks when it fails cannot show a backlog shrinking.
    if WHOLE_CRATE_STEPS:
        print()
        print(f"# {len(WHOLE_CRATE_STEPS)} whole-crate step(s) — weak evidence, "
              f"name the verifying test(s) (#262):")
        for aid, cmd in WHOLE_CRATE_STEPS:
            print(f"#   {aid}: {cmd}")

    if args.markdown:
        print()
        print(emit_markdown(report))

    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
