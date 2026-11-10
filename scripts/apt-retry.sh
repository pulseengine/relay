#!/usr/bin/env bash
# apt-retry.sh <apt-get arguments...>
#
# BOUNDED, RETRIED apt-get for gazebo.yml's "Install Gazebo Harmonic" step.
# Each attempt is capped (a stalled mirror trickles one package every ~2 s, so
# no per-request apt timeout ever fires); downloaded .debs stay in apt's cache,
# so a retry resumes rather than restarts. Exit 0 on the first attempt that
# succeeds, 1 after the third failure.
#
# THREE THINGS THE INLINE VERSION GOT WRONG (#609 finding 10):
#   1. Its "switching to archive.ubuntu.com" changed nothing. On the hosted
#      ubuntu-24.04 image the sources do not name azure.archive.ubuntu.com:
#      runner-images' configure-apt-sources.sh rewrites ubuntu.sources to
#      `mirror+file:/etc/apt/apt-mirrors.txt` and lists azure there at
#      priority 1. apt's mirror method only fails over on an ERROR, never on a
#      slow mirror, so the sed on ubuntu.sources matched nothing (hidden by
#      `2>/dev/null || true`). Run 41738372410 shows it: attempt 1 timed out
#      at 600 s mid-download, and the retry fetched from azure again. Here the
#      azure line is REMOVED from apt-mirrors.txt, so archive.ubuntu.com
#      (already priority 2 in the image) becomes first, and a switch that
#      finds nothing to switch says so.
#   2. A timeout can land while dpkg is unpacking, leaving "dpkg was
#      interrupted", and every later attempt then fails on that. `dpkg
#      --configure -a` runs before each retry.
#   3. It ran a fallback `apt-get update` even after the LAST failure, which
#      only spent another 300 s before failing anyway.
#
# BUDGET (maintainer decision 2026-11-04, #609 finding 10): each attempt is
# capped at 300 s, not 600 s. Measured over 56 gz runs: a clean install takes
# median 71 s, max 273 s (300 s is ~1.1x that, so a slow-but-healthy mirror can
# be cut and retried on the other mirror). Worst case for one stalled call:
# 3x300 s of attempts + 2x300 s of update + 2x300 s of dpkg repair = 2100 s,
# plus at most 463 s for the rest of a successful job = 2563 s, inside the
# job's 2700 s (45 min). The update and repair calls take seconds when nothing
# stalls, so the realistic worst case is ~900 s + 463 s.
#
# Every timeout is `-k 10`: a call that ignores SIGTERM is SIGKILLed 10 s later
# instead of running on unbounded (#638 finding 15). That adds at most
# 7 x 10 s = 70 s, so the worst case is 2633 s, still inside 2700 s.
#
# Test seams (all default to the real system): APT_RETRY_MIRRORS_FILE,
# APT_RETRY_SOURCES (space-separated), APT_RETRY_ATTEMPT_S, APT_RETRY_UPDATE_S.
set -euo pipefail

MIRRORS="${APT_RETRY_MIRRORS_FILE:-/etc/apt/apt-mirrors.txt}"
SOURCES="${APT_RETRY_SOURCES:-/etc/apt/sources.list.d/ubuntu.sources /etc/apt/sources.list}"
ATTEMPT_S="${APT_RETRY_ATTEMPT_S:-300}"
UPDATE_S="${APT_RETRY_UPDATE_S:-300}"
AZURE='http://azure\.archive\.ubuntu\.com/ubuntu'

# Rewrite $1 through the filter command in "$2..." without `sed -i`, whose
# syntax differs between GNU and BSD.
rewrite() {
  local f="$1" tmp rc=0
  shift
  tmp=$(mktemp)
  "$@" < "$f" > "$tmp" || rc=$?
  # grep exits 1 for "no line selected", which is a valid (empty) result;
  # anything above 1 is an error and must not clobber the file.
  if [ "$rc" -gt 1 ]; then
    rm -f "$tmp"
    echo "::warning::could not rewrite $f (exit $rc); left unchanged"
    return 1
  fi
  sudo cp "$tmp" "$f"
  rm -f "$tmp"
}

off_azure() {
  local changed=0 f
  if [ -f "$MIRRORS" ] && grep -q "^$AZURE" "$MIRRORS"; then
    # Never leave apt with an empty mirror list: that would turn a slow retry
    # into a certain failure.
    if [ -n "$(grep -v "^$AZURE" "$MIRRORS" | tr -d '[:space:]')" ]; then
      rewrite "$MIRRORS" grep -v "^$AZURE" && changed=1
    else
      echo "::warning::$MIRRORS lists only the azure mirror; keeping it, since an empty mirror list is worse than a slow mirror"
    fi
  fi
  # Older images (and self-built ones) name the mirror in the sources directly.
  for f in $SOURCES; do
    if [ -f "$f" ] && grep -q "$AZURE" "$f"; then
      rewrite "$f" sed "s|$AZURE|http://archive.ubuntu.com/ubuntu|g" && changed=1
    fi
  done
  if [ "$changed" = 1 ]; then
    echo "apt-retry: moved off azure.archive.ubuntu.com"
  else
    echo "::warning::no azure mirror entry left to remove in $MIRRORS or $SOURCES (already moved, or never configured); the retry uses the current mirror list"
  fi
}

for n in 1 2 3; do
  if sudo timeout -k 10 "$ATTEMPT_S" apt-get -o Acquire::Retries=3 "$@"; then
    exit 0
  fi
  if [ "$n" = 3 ]; then
    break
  fi
  echo "::warning::apt-get $* failed or exceeded ${ATTEMPT_S} s (attempt $n/3) — retrying off the azure mirror"
  # A timeout can kill apt-get mid-unpack.
  sudo timeout -k 10 "$UPDATE_S" dpkg --configure -a \
    || echo "::warning::dpkg --configure -a failed; the next attempt will show why"
  off_azure
  sudo timeout -k 10 "$UPDATE_S" apt-get update || true
done
echo "::error::apt-get $* failed 3 times"
exit 1
