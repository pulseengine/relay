#!/usr/bin/env bash
#
# record_headless.sh — capture real Gazebo 3D footage of a falcon-sitl-gz
# flight WITHOUT the GUI and WITHOUT screen-recording permission.
#
# Why this exists: on macOS the gz-sim GUI VideoRecorder plugin never arms its
# /gui/record_video service (no render context on this display), and macOS
# blocks `screencapture` from a non-GUI process. This path sidesteps both: it
# adds a *camera sensor* to the world, renders it headless via the existing
# Sensors(ogre2) system (which DOES get an offscreen context here), forces the
# camera to render by attaching a topic subscriber, saves frames to /tmp, and
# encodes them with ffmpeg. No GUI, no screen capture, no permissions.
#
# Two gotchas this script gets right (both bit the first attempts):
#   • lazy rendering — a gz camera only renders when something subscribes to its
#     image topic, so we run `gz topic -e` to force it (else 0 frames save);
#   • frame ORDER — gz saves frames as <...>_<n>.png with no zero-padding, so a
#     lexical sort interleaves 1,10,100,2,… and the video flickers. We sort by
#     the true trailing numeric index.
#
# Usage:
#   examples/falcon-sitl-gz/scripts/record_headless.sh [chase|static] [scenario] [duration_s]
#     view     : chase (camera follows the drone, default) | static (fixed vantage)
#     scenario : mission (default) | alt-only | hover | geo-hover
#     duration : flight seconds (default 55)
#
# Output: bench-evidence/gz-sim/recordings/<ts>-falcon-<view>.mp4  (git-ignored)
# Requires: gz-harmonic (gz sim 8), ffmpeg, the `gazebo` cargo feature.
set -uo pipefail

VIEW="${1:-chase}"
SCEN="${2:-mission}"
DUR="${3:-55}"
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
SRCW="$REPO/examples/falcon-sitl-gz/worlds/falcon-quad.sdf"
CAMW="/tmp/falcon-quad-cam-${VIEW}.sdf"
FRAMES="/tmp/falcon-cam-frames"; SEQ="/tmp/falcon-seq"
OUTDIR="$REPO/bench-evidence/gz-sim/recordings"
mkdir -p "$OUTDIR" "$FRAMES" "$SEQ"

# ── 1. inject the recording camera (chase = on base_link; static = a fixed model)
TOPIC="/falcon_record_cam"
python3 - "$SRCW" "$CAMW" "$VIEW" "$FRAMES" "$TOPIC" <<'PY'
import sys
src, dst, view, frames, topic = sys.argv[1:6]
s = open(src).read()
if view == "chase":
    cam = f'''        <sensor name="record_camera" type="camera">
          <pose>-2.2 0 1.0 0 0.427 0</pose>
          <camera><horizontal_fov>1.15</horizontal_fov>
            <image><width>1280</width><height>720</height></image>
            <clip><near>0.05</near><far>400</far></clip>
            <save enabled="true"><path>{frames}</path></save></camera>
          <update_rate>30</update_rate><topic>{topic}</topic><always_on>1</always_on>
        </sensor>
'''
    i = s.index("</link>", s.index('name="mag_sensor"'))  # base_link's </link>
    out = s[:i] + cam + "      " + s[i:]
else:
    cam = f'''
    <model name="record_cam"><static>true</static><pose>4 -4 3 0 0.289 2.232</pose>
      <link name="link"><sensor name="record_camera" type="camera">
        <camera><horizontal_fov>1.05</horizontal_fov>
          <image><width>1280</width><height>720</height></image>
          <clip><near>0.1</near><far>300</far></clip>
          <save enabled="true"><path>{frames}</path></save></camera>
        <update_rate>30</update_rate><topic>{topic}</topic><always_on>1</always_on>
      </sensor></link></model>
'''
    i = s.rfind("</world>")
    out = s[:i] + cam + "\n  " + s[i:]
open(dst, "w").write(out)
print(f"injected {view} camera -> {dst}")
PY

rm -f "$FRAMES"/*.png "$SEQ"/*.png 2>/dev/null
pkill -f 'gz sim' 2>/dev/null; sleep 2

# ── 2. headless server (server + GUI are separate on macOS; we run server only)
echo "== headless server (camera world) =="
gz sim -s -r -v1 "$CAMW" >/tmp/gz-headless-srv.log 2>&1 & SRV=$!
trap 'kill $SRV $SUB 2>/dev/null; pkill -f "gz sim" 2>/dev/null' EXIT
echo "   waiting for ogre2 render init…"; sleep 14

# ── 3. force the camera to render, then fly
echo "== forcing camera render + flying ($SCEN, ${DUR}s) =="
gz topic -e -t "$TOPIC" >/dev/null 2>&1 & SUB=$!
T0=$(python3 -c 'import time;print(time.time())')
( cd "$REPO" && cargo run -q -p falcon-sitl-gz --features gazebo -- \
    --backend=gazebo --world=falcon --model=quad --home=47.3977,8.5456,488 \
    --scenario="$SCEN" --duration="$DUR" --evidence-dir=/tmp/falcon-flight ) \
    >/tmp/gz-headless-fly.log 2>&1
grep -E 'verdict:' /tmp/gz-headless-fly.log | tail -1
T1=$(python3 -c 'import time;print(time.time())')
kill $SUB 2>/dev/null; sleep 2

# ── 4. encode with CORRECT numeric frame order (the flicker fix) at real-time fps
N=$(ls "$FRAMES"/*.png 2>/dev/null | wc -l | tr -d ' ')
[ "$N" -eq 0 ] && { echo "!! no frames rendered — render context failed"; exit 3; }
FPS=$(python3 -c "print(round($N/max(1.0,$T1-$T0),2))")
echo "== $N frames @ ${FPS} fps real-time -> encoding =="
i=0
for f in $(ls "$FRAMES"/*.png | sed -E 's|.*_([0-9]+)\.png$|\1 &|' | sort -n -k1 | awk '{print $2}'); do
  printf -v n "$SEQ/f%06d.png" $i; ln -sf "$f" "$n"; i=$((i+1))
done
OUT="$OUTDIR/$(date +%s)-falcon-${VIEW}.mp4"
ffmpeg -y -framerate "$FPS" -i "$SEQ/f%06d.png" -vf "fps=30,format=yuv420p" \
  -c:v libx264 -crf 20 -movflags +faststart "$OUT" >/tmp/ffmpeg-headless.log 2>&1
echo "== done -> $OUT =="
ls -la "$OUT" 2>/dev/null
