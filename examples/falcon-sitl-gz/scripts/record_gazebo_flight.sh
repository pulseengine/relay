#!/usr/bin/env bash
#
# record_gazebo_flight.sh — capture real Gazebo 3D footage of a
# falcon-sitl-gz flight using gz-sim's VideoRecorder GUI plugin.
#
# This opens a Gazebo GUI window on your display (the VideoRecorder
# needs a rendering context — it can't run fully headless). The 3D
# scene is recorded to mp4 via the /gui/record_video service while the
# verified-cascade bridge flies the quad.
#
# Usage:
#   examples/falcon-sitl-gz/scripts/record_gazebo_flight.sh [scenario] [duration_s]
#
#   scenario : alt-only (default, stable hover) | hover (full cascade,
#              still tuning as of v0.19.5) | open-loop-climb
#   duration : flight seconds (default 10)
#
# Output: bench-evidence/gz-sim/recordings/<ts>-gazebo-<scenario>.mp4
#
# Requires: gz-harmonic (gz sim 8), a display, the `gazebo` cargo
# feature built. See sim docs + https://gazebosim.org/api/sim/8/videorecorder.html
set -euo pipefail

SCENARIO="${1:-alt-only}"
DURATION="${2:-10}"
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
WORLD="$REPO/examples/falcon-sitl-gz/worlds/falcon-quad.sdf"
GUICFG="$REPO/examples/falcon-sitl-gz/scripts/gui-video.config"
OUTDIR="$REPO/bench-evidence/gz-sim/recordings"
TS="$(date +%s)"
OUT="$OUTDIR/${TS}-gazebo-${SCENARIO}.mp4"
mkdir -p "$OUTDIR"

echo "== launching gz sim (server + GUI with VideoRecorder) =="
gz sim -v3 --gui-config "$GUICFG" "$WORLD" >/tmp/gz-record.log 2>&1 &
GZ_PID=$!
trap 'kill $GZ_PID 2>/dev/null || true' EXIT
sleep 10  # GUI + render context warm-up

echo "== starting video recording =="
gz service -s /gui/record_video \
  --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 3000 \
  --req "start: true, format: 'mp4', save_filename: '$OUT'"

echo "== flying ($SCENARIO, ${DURATION}s) =="
( cd "$REPO" && cargo run -q -p falcon-sitl-gz --features gazebo -- \
    --backend=gazebo --world=falcon --model=quad \
    --home=47.3977,8.5456,488 \
    --scenario="$SCENARIO" --duration="$DURATION" \
    --evidence-dir="$OUTDIR" )

echo "== stopping video recording =="
gz service -s /gui/record_video \
  --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 3000 \
  --req "stop: true"
sleep 3  # let the encoder flush

echo "== done -> $OUT =="
ls -la "$OUT" 2>/dev/null || echo "  (if missing: the GUI may need the window focused; record manually via the VideoRecorder toolbar button)"
