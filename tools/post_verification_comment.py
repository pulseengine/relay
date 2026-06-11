#!/usr/bin/env python3
"""
Post a sticky verification-gate comment on a pull request.

Mirrors the witness / spar pattern: idempotent updates via a hidden
HTML marker that lets us find and edit the same comment on each PR
update instead of accumulating noise.

Usage:
    post_verification_comment.py <PR_NUMBER> --body-file <PATH>

The body file should contain the Markdown produced by
`scripts/run-falcon-verification.py --markdown`. This script just
finds-or-creates the sticky comment and rewrites it.

Requires `gh` (GitHub CLI) on PATH with GH_TOKEN set in the
environment. In GitHub Actions, GH_TOKEN=secrets.GITHUB_TOKEN
gives the workflow comment-write permission (see the
`pull-requests: write` permission in the workflow).
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path

STICKY_MARKER = "<!-- falcon-verification-gate -->"


def list_pr_comments(pr_number: str) -> list[dict]:
    out = subprocess.check_output([
        "gh", "api", f"repos/{repo_slug()}/issues/{pr_number}/comments",
        "--paginate",
    ])
    return json.loads(out)


def repo_slug() -> str:
    out = subprocess.check_output([
        "gh", "repo", "view", "--json", "nameWithOwner", "-q", ".nameWithOwner",
    ])
    return out.decode().strip()


def find_sticky(comments: list[dict]) -> dict | None:
    for c in comments:
        if STICKY_MARKER in (c.get("body") or ""):
            return c
    return None


def upsert(pr_number: str, body: str) -> None:
    body_with_marker = f"{STICKY_MARKER}\n{body}"
    existing = find_sticky(list_pr_comments(pr_number))
    if existing:
        comment_id = existing["id"]
        print(f"updating sticky comment id={comment_id}", file=sys.stderr)
        # Pass the body via STDIN (`-F body=@-`), not a CLI arg — a large
        # report exceeds the OS argument-length limit (Errno 7).
        subprocess.run(
            [
                "gh", "api", "--method", "PATCH",
                f"repos/{repo_slug()}/issues/comments/{comment_id}",
                "-F", "body=@-",
            ],
            input=body_with_marker,
            text=True,
            check=True,
        )
    else:
        print("creating new sticky comment", file=sys.stderr)
        # `--body-file -` reads the body from STDIN (size-safe).
        subprocess.run(
            ["gh", "pr", "comment", pr_number, "--body-file", "-"],
            input=body_with_marker,
            text=True,
            check=True,
        )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("pr_number")
    p.add_argument("--body-file", required=True, type=Path)
    args = p.parse_args()
    body = args.body_file.read_text()
    # The sticky comment is a convenience, not the gate verdict. If the GitHub
    # CLI is unavailable (e.g. a self-hosted runner without `gh`), warn and skip
    # cleanly rather than fail with a traceback — the workflow step is also
    # continue-on-error, but a soft exit keeps the log readable.
    try:
        upsert(args.pr_number, body)
    except FileNotFoundError as e:
        print(f"::warning::skipping PR comment — {e.filename!r} not on PATH "
              f"(self-hosted runner without gh?); verification verdict is "
              f"unaffected", flush=True)
        return 0
    return 0


if __name__ == "__main__":
    sys.exit(main())
