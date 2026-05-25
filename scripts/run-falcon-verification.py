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
    re.compile(r"\bcargo\s+kani\b"),                  # kani-verifier + CBMC
    re.compile(r"\bcargo\s+\+nightly\s+miri\b"),      # miri nightly component
    re.compile(r"^\s*MIRIFLAGS="),                    # same family
    re.compile(r"\brustup\s+component\s+add\s+miri"), # same family
    re.compile(r"--backend=(hackrf|mavlink|gazebo)\b"),  # needs PX4 / HackRF / gz sim
    re.compile(r"--preset=px4-sitl\b"),               # needs PX4-Autopilot or live PX4
    re.compile(r"\$WITNESS\b"),                       # template env-var placeholder
    re.compile(r"\bgz\s+sim\b"),                      # Gazebo Sim install
    re.compile(r"\bmake\s+px4_sitl\b"),               # PX4-Autopilot install
    re.compile(r"\bbazel\s+(test|build|run)\b"),      # bazel not provisioned in the gate job
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
]


def is_bench_only(cmd: str) -> bool:
    return any(p.search(cmd) for p in BENCH_PATTERNS)


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


def run_steps(artifact: dict[str, Any], dry_run: bool) -> tuple[bool, list[dict]]:
    aid = artifact["id"]
    steps = artifact.get("fields", {}).get("steps") or []
    results = []
    artifact_pass = True
    for i, step in enumerate(steps):
        cmd = step["run"]
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
        rc = subprocess.call(cmd, shell=True)
        duration = time.monotonic() - start
        passed = rc == 0
        artifact_pass = artifact_pass and passed
        status = "PASS" if passed else f"FAIL (rc={rc})"
        print(f"  [{status:>14}] ({duration:6.2f}s) {aid}: {cmd}")
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
        elif r["pass"]:
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

    if args.markdown:
        print()
        print(emit_markdown(report))

    return 0 if overall_pass else 1


if __name__ == "__main__":
    sys.exit(main())
