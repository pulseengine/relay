#!/usr/bin/env bash
# Make a per-release falcon video: composite a release's feature info-card
# (gen-release-card.py) onto flight footage, so each release ships a video that
# names + frames the new feature. The bench's ffmpeg has no drawtext, so the
# card is a PIL-rendered PNG overlaid via ffmpeg's overlay filter.
#
# Usage:
#   make-release-video.sh <frames-dir> <out.mp4> <version-title> <subtitle> \
#       <fact1> [fact2] [fact3]
#   <frames-dir> : a dir of gz frames (record_headless leaves them in
#                  /tmp/run_<k>); or set BASE_MP4=<clip> to overlay an existing
#                  clip instead.
#
# Honest note: gz can't fly most realism features cleanly (wind diverges) and
# proofs/sensor-models aren't visual — so the per-release distinction is the
# overlay CARD over the verified mission footage, not a feature-specific flight.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
# Pick a python with Pillow. Prefer the stable venv from setup-video-env.sh
# ($REPO/target/video-venv); fall back to the legacy /tmp/plotenv if that's all
# that exists; honor an explicit $PYTHON override. Run setup-video-env.sh first.
if [ -n "${PYTHON:-}" ]; then PY="$PYTHON"
elif [ -x "$REPO/target/video-venv/bin/python" ]; then PY="$REPO/target/video-venv/bin/python"
else PY="/tmp/plotenv/bin/python"; fi
if ! "$PY" -c 'import PIL' >/dev/null 2>&1; then
  echo "!! no Pillow-capable python ($PY). Run: $HERE/setup-video-env.sh" >&2; exit 1
fi
FRAMES="$1"; OUT="$2"; TITLE="$3"; SUB="$4"; shift 4
CARD="/tmp/falcon-card.png"
"$PY" "$HERE/gen-release-card.py" "$CARD" "$TITLE" "$SUB" "$@" >/dev/null

if [ -n "${BASE_MP4:-}" ]; then
  ffmpeg -y -i "$BASE_MP4" -i "$CARD" \
    -filter_complex "[0:v]format=yuv420p[v];[v][1:v]overlay=0:0:format=auto[o]" \
    -map "[o]" -c:v libx264 -crf 20 -movflags +faststart "$OUT" >/tmp/ff-rel.log 2>&1
else
  # encode frames (numeric order, ~30 fps), trim 2 s, overlay the card
  rm -rf /tmp/falcon-seq; mkdir -p /tmp/falcon-seq; i=0
  for f in $(ls "$FRAMES"/*.png | sed -E 's|.*_([0-9]+)\.png$|\1 &|' | sort -n -k1 | awk '{print $2}'); do
    printf -v p "/tmp/falcon-seq/f%06d.png" $i; ln -sf "$f" "$p"; i=$((i+1))
  done
  # vary the start of each release's clip a LITTLE (1.0–4.0 s, deterministic
  # from the title) so the videos are not byte-identical reuse of the footage.
  START="${START:-$(python3 -c "import sys;print(round(1.0+(sum(map(ord,sys.argv[1]))%31)/10.0,1))" "$TITLE")}"
  ffmpeg -y -ss "$START" -framerate 30 -i /tmp/falcon-seq/f%06d.png -i "$CARD" -t 72 \
    -filter_complex "[0:v]fps=30,format=yuv420p[v];[v][1:v]overlay=0:0:format=auto[o]" \
    -map "[o]" -c:v libx264 -crf 20 -movflags +faststart "$OUT" >/tmp/ff-rel.log 2>&1
fi
echo "-> $OUT ($(wc -c < "$OUT" 2>/dev/null || echo 0) bytes)"
