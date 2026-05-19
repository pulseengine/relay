#!/usr/bin/env bash
#
# Post-merge release driver for falcon-v0.1.0.
#
# After PR #9 lands on `main`, run this from the repo root to:
#   1. Sync local main from origin.
#   2. Verify HEAD is the merge commit (so the release reflects PR content).
#   3. Create an annotated tag falcon-v0.1.0.
#   4. Push the tag, which triggers .github/workflows/release.yml.
#   5. Wait for the release workflow to finish and surface the resulting
#      GitHub Release URL.
#
# Override the version once future releases use this script:
#     TAG=falcon-v0.2.0 bash scripts/tag-and-release.sh

set -euo pipefail

TAG="${TAG:-falcon-v0.1.0}"
TITLE="${TITLE:-${TAG} — boots and waves}"
REPO="${REPO:-pulseengine/relay}"

REPO_ROOT=$(cd "$(dirname "$0")/.." && pwd)
cd "$REPO_ROOT"

echo "[tag-and-release] sync local main from origin..."
git checkout main
git pull --ff-only origin main

echo "[tag-and-release] HEAD now at $(git log -1 --oneline)"

# Confirm tag doesn't already exist remotely.
if git ls-remote --tags origin "${TAG}" | grep -q "${TAG}"; then
    echo "[tag-and-release] ERROR: tag ${TAG} already exists on origin" >&2
    exit 1
fi

# Confirm the falcon-hello demo runs cleanly on this revision before
# tagging, so we're not tagging a broken main.
echo "[tag-and-release] sanity check — falcon-hello-demo on current main..."
bash scripts/falcon-hello-demo.sh

echo "[tag-and-release] creating annotated tag ${TAG}..."
git tag -a "${TAG}" -m "${TITLE}"

echo "[tag-and-release] pushing tag (triggers release.yml)..."
git push origin "${TAG}"

echo "[tag-and-release] release workflow dispatched. Watch:"
echo "    gh run watch --repo ${REPO}"
echo "    gh release view ${TAG} --repo ${REPO}"

# Optional: poll for the release to land.
echo "[tag-and-release] waiting up to 30 minutes for release to land..."
deadline=$(( $(date +%s) + 1800 ))
while [ "$(date +%s)" -lt "$deadline" ]; do
    if gh release view "${TAG}" --repo "${REPO}" >/dev/null 2>&1; then
        URL=$(gh release view "${TAG}" --repo "${REPO}" --json url --jq .url)
        echo "[tag-and-release] PUBLISHED: ${URL}"
        exit 0
    fi
    sleep 30
done

echo "[tag-and-release] release did not land within 30 minutes; check the workflow:"
echo "    gh run list --repo ${REPO} --workflow=release.yml --limit 3"
exit 1
