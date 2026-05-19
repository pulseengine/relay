#!/usr/bin/env python3
"""
Extract and run all falcon verification steps from rivet artifacts.

This is the reference implementation that spar's
tools/post_verification_comment.py-equivalent should follow for relay:

  1. List unit-verification artifacts matching a rivet filter.
  2. For each, rivet-get the full artifact JSON.
  3. Extract fields.steps[].run as shell commands.
  4. Run each command, capture exit code + duration.
  5. Aggregate pass/fail per artifact.
  6. Emit a Markdown summary suitable for posting as a PR comment.

Usage:
    python3 scripts/run-falcon-verification.py
    python3 scripts/run-falcon-verification.py --filter '(has-tag "v0.1")'
    python3 scripts/run-falcon-verification.py --dry-run
    python3 scripts/run-falcon-verification.py --markdown
"""

import argparse
import json
import subprocess
import sys
import time
from typing import Any


def rivet_list(filter_expr: str) -> list[str]:
    out = subprocess.check_output([
        "rivet", "list",
        "--type", "unit-verification",
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
        if dry_run:
            print(f"  [dry-run] {aid} step {i+1}: {cmd}")
            results.append({"cmd": cmd, "pass": True, "duration": 0.0})
            continue
        start = time.monotonic()
        rc = subprocess.call(cmd, shell=True)
        duration = time.monotonic() - start
        passed = rc == 0
        artifact_pass = artifact_pass and passed
        status = "PASS" if passed else f"FAIL (rc={rc})"
        print(f"  [{status:>14}] ({duration:6.2f}s) {aid}: {cmd}")
        results.append({"cmd": cmd, "pass": passed, "rc": rc, "duration": duration})
    if not steps:
        print(f"  [   skip-no-steps] {aid}: (no steps defined)")
    return artifact_pass, results


def emit_markdown(report: list[dict]) -> str:
    total = len(report)
    passed = sum(1 for r in report if r["pass"])
    skipped = sum(1 for r in report if not r["steps"])
    failed = total - passed - skipped
    icon = "✅" if failed == 0 else "❌"
    lines = [
        f"## {icon} Rivet verification gate — falcon",
        "",
        f"**{passed}/{total} passed**",
        "",
        "| count |   |",
        "|-----|---|",
        f"| Passed | {passed} |",
        f"| Failed | {failed} |",
        f"| Skipped (no steps) | {skipped} |",
        "",
    ]
    if failed:
        lines.append("### Failed artifacts")
        for r in report:
            if not r["pass"] and r["steps"]:
                lines.append(f"- `{r['id']}` — {r['title']}")
                for s in r["steps"]:
                    if not s["pass"]:
                        lines.append(f"  - `{s['cmd']}` (rc={s['rc']})")
        lines.append("")
    lines.append("Source of truth: `artifacts/verification/FV-FALCON-*.yaml`.")
    return "\n".join(lines)


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--filter", default='(has-tag "falcon")',
                   help='rivet S-expression filter (default: falcon-tagged)')
    p.add_argument("--dry-run", action="store_true",
                   help="print commands without executing")
    p.add_argument("--markdown", action="store_true",
                   help="emit Markdown summary at end (for PR comment)")
    args = p.parse_args()

    print(f"# falcon verification gate (filter: {args.filter})")
    ids = rivet_list(args.filter)
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
