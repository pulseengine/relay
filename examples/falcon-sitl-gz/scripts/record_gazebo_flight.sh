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
# macOS (gazebosim/gz-sim#44) needs server + GUI as SEPARATE processes.
pkill -f 'gz sim' 2>/dev/null || true; sleep 2
gz sim -s -r -v1 "$WORLD" >/tmp/gz-record-srv.log 2>&1 &
SRV_PID=$!
sleep 4
gz sim -g --gui-config "$GUICFG" >/tmp/gz-record-gui.log 2>&1 &
GUI_PID=$!
trap 'kill $SRV_PID $GUI_PID 2>/dev/null || true' EXIT

# POLL for the VideoRecorder's service instead of a fixed sleep — a cold gz
# GUI on macOS needs the render context up (and the window focused) before
# /gui/record_video registers. (The old fixed `sleep 10` fired the start call
# before the service existed → "Service call timed out" → a silent no-record.)
echo "== waiting for the VideoRecorder service =="
echo "   >>> CLICK the Gazebo window to focus it so the recorder can arm <<<"
ARMED=0
for i in $(seq 1 30); do   # up to ~60 s
  if gz service --list 2>/dev/null | grep -q '/gui/record_video'; then
    ARMED=1; break
  fi
  sleep 2
done
if [ "$ARMED" -ne 1 ]; then
  echo "!! /gui/record_video never appeared (the GUI recorder didn't arm on this"
  echo "   display). Fall back to a native screen recording of the GUI window:"
  echo "     ./examples/falcon-sitl-gz/tools/watch_mission.sh $SCENARIO $DURATION"
  echo "   then macOS Cmd+Shift+5 → record the Gazebo window."
  exit 3
fi

echo "== starting video recording =="
gz service -s /gui/record_video \
  --reqtype gz.msgs.VideoRecord --reptype gz.msgs.Boolean --timeout 5000 \
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
