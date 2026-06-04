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
# camera to render by attaching a topic subscriber, saves frames, and encodes
# them with ffmpeg. No GUI, no screen capture, no permissions.
#
# Three gotchas this script gets right (all bit the first attempts):
#   • lazy rendering — a gz camera renders only when something subscribes to its
#     image topic, so we run `gz topic -e` to force it (else 0 frames save);
#   • frame ORDER — gz saves frames as <...>_<n>.png with no zero-padding, so a
#     lexical sort interleaves 1,10,100,2,… and the video flickers. We sort by
#     the true trailing numeric index;
#   • framerate — encode at the MEASURED capture rate (frames/elapsed), else a
#     fixed 30 fps yields ~2.5x slow-motion.
#
# Views:
#   chase   — camera welded to base_link, follows the drone (always centred, but
#             once the ground/horizon leave frame there's no motion reference).
#   static  — a fixed vantage; the drone moves within the frame (motion visible)
#             but is small.
#   markers — RECOMMENDED. A fixed 3/4 camera + four colour-coded posts at the
#             mission-square corners (ground/horizon/shadow as motion reference)
#             AND a best-of-N loop: the falcon mission has run-to-run variance
#             (some runs diverge >100 m off-screen), so we fly RUNS times and
#             keep the run that stays framed and returns home cleanest.
#
# Usage:
#   record_headless.sh [chase|static|markers] [scenario] [duration_s]
#     scenario : mission (default) | alt-only | hover | geo-hover
#     duration : flight seconds (default 55)
#   env: RUNS (markers best-of-N, default 4)  FRAME_CAP (peak<this stays framed,
#        default 8.0 m)  START_TRIM (drop leading idle seconds, default 2)
#
# Output: bench-evidence/gz-sim/recordings/<ts>-falcon-<view>.mp4  (git-ignored)
# Requires: gz-harmonic (gz sim 8), ffmpeg, python3, the `gazebo` cargo feature.
set -uo pipefail

VIEW="${1:-markers}"
SCEN="${2:-mission}"
DUR="${3:-55}"
RUNS="${RUNS:-4}"
FRAME_CAP="${FRAME_CAP:-8.0}"
START_TRIM="${START_TRIM:-2}"
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
SRCW="$REPO/examples/falcon-sitl-gz/worlds/falcon-quad.sdf"
CAMW="/tmp/falcon-quad-cam-${VIEW}.sdf"
FRAMES="/tmp/falcon-cam-frames"
OUTDIR="$REPO/bench-evidence/gz-sim/recordings"
TOPIC="/falcon_record_cam"
mkdir -p "$OUTDIR"

# ── 1. build the camera world (camera + optional marker posts) ───────────
python3 - "$SRCW" "$CAMW" "$VIEW" "$FRAMES" "$TOPIC" <<'PY'
import sys
src, dst, view, frames, topic = sys.argv[1:6]
s = open(src).read()
def cam_model(pose, fov):
    return (f'<model name="record_cam"><static>true</static><pose>{pose}</pose>'
            f'<link name="link"><sensor name="record_camera" type="camera">'
            f'<camera><horizontal_fov>{fov}</horizontal_fov>'
            f'<image><width>1280</width><height>720</height></image>'
            f'<clip><near>0.1</near><far>500</far></clip>'
            f'<save enabled="true"><path>{frames}</path></save></camera>'
            f'<update_rate>30</update_rate><topic>{topic}</topic>'
            f'<always_on>1</always_on></sensor></link></model>')
if view == "chase":
    cam = (f'        <sensor name="record_camera" type="camera">'
           f'<pose>-2.2 0 1.0 0 0.427 0</pose>'
           f'<camera><horizontal_fov>1.15</horizontal_fov>'
           f'<image><width>1280</width><height>720</height></image>'
           f'<clip><near>0.05</near><far>400</far></clip>'
           f'<save enabled="true"><path>{frames}</path></save></camera>'
           f'<update_rate>30</update_rate><topic>{topic}</topic>'
           f'<always_on>1</always_on></sensor>\n')
    i = s.index("</link>", s.index('name="mag_sensor"'))   # base_link's </link>
    out = s[:i] + cam + "      " + s[i:]
else:
    xml = ""
    if view == "markers":
        # mission square (NED) -> gz ENU corners; colour-coded posts as reference
        for name, xy, rgb in [("A","0 0","1 0.1 0.1"), ("B","0 2","0.1 1 0.1"),
                              ("C","2 2","0.2 0.4 1"), ("D","2 0","1 0.85 0.1")]:
            xml += (f'\n    <model name="post_{name}"><static>true</static>'
                    f'<pose>{xy} 0.75 0 0 0</pose><link name="l"><visual name="v">'
                    f'<geometry><cylinder><radius>0.05</radius><length>1.5</length>'
                    f'</cylinder></geometry><material><ambient>{rgb} 1</ambient>'
                    f'<diffuse>{rgb} 1</diffuse></material></visual></link></model>')
        xml += "\n    " + cam_model("-3.5 -3.5 2.8 0 0.245 0.785", "1.05")
    else:  # static
        xml += "\n    " + cam_model("4 -4 3 0 0.289 2.232", "1.05")
    i = s.rfind("</world>")
    out = s[:i] + xml + "\n  " + s[i:]
open(dst, "w").write(out)
print(f"built {view} camera world -> {dst}")
PY

# ── fly once: server + forced render + flight; echoes "<peak> <final> <rms>"
fly_once() {
  rm -rf "$FRAMES"; mkdir -p "$FRAMES" /tmp/falcon-flight
  pkill -f 'gz sim' 2>/dev/null; sleep 2
  gz sim -s -r -v1 "$CAMW" >/tmp/gz-headless-srv.log 2>&1 & local SRV=$!
  sleep 14
  gz topic -e -t "$TOPIC" >/dev/null 2>&1 & local SUB=$!
  local T0; T0=$(python3 -c 'import time;print(time.time())')
  ( cd "$REPO" && cargo run -q -p falcon-sitl-gz --features gazebo -- \
      --backend=gazebo --world=falcon --model=quad --home=47.3977,8.5456,488 \
      --scenario="$SCEN" --duration="$DUR" --evidence-dir=/tmp/falcon-flight ) \
      >/tmp/gz-headless-fly.log 2>&1
  local T1; T1=$(python3 -c 'import time;print(time.time())')
  kill $SUB $SRV 2>/dev/null; pkill -f 'gz sim' 2>/dev/null; sleep 1
  local V; V=$(grep -E 'verdict:' /tmp/gz-headless-fly.log | tail -1)
  local elapsed; elapsed=$(python3 -c "print(max(1.0,$T1-$T0))")
  local peak final rms
  peak=$(echo "$V" | sed -nE 's/.*peak_dist=([0-9.]+)m.*/\1/p'); peak=${peak:-9999}
  final=$(echo "$V" | sed -nE 's/.*final_dist=([0-9.]+)m.*/\1/p'); final=${final:-9999}
  rms=$(echo "$V" | sed -nE 's/.*rms_steady=([0-9.]+)m.*/\1/p'); rms=${rms:-9999}
  # echo ELAPSED as the 4th field: fly_once runs in a `< <(…)` subshell, so a
  # global assignment here would be lost — the caller captures it via `read`.
  echo "$peak $final $rms $elapsed"
}

# encode $FRAMES (in correct numeric order, real-time) -> $1 ; trims START_TRIM
encode() {
  local out="$1"
  local n; n=$(ls "$FRAMES"/*.png 2>/dev/null | wc -l | tr -d ' ')
  [ "$n" -eq 0 ] && { echo "!! no frames rendered — render context failed"; return 3; }
  local fps; fps=$(python3 -c "print(round($n/$ELAPSED,2))")
  rm -rf /tmp/falcon-seq; mkdir -p /tmp/falcon-seq
  local i=0 f
  for f in $(ls "$FRAMES"/*.png | sed -E 's|.*_([0-9]+)\.png$|\1 &|' | sort -n -k1 | awk '{print $2}'); do
    printf -v p "/tmp/falcon-seq/f%06d.png" $i; ln -sf "$f" "$p"; i=$((i+1))
  done
  ffmpeg -y -ss "$START_TRIM" -framerate "$fps" -i /tmp/falcon-seq/f%06d.png \
    -vf "fps=30,format=yuv420p" -c:v libx264 -crf 20 -movflags +faststart \
    "$out" >/tmp/ffmpeg-headless.log 2>&1
  echo "   $n frames @ ${fps} fps -> $out"
}

OUT="$OUTDIR/$(date +%s)-falcon-${VIEW}.mp4"
trap 'pkill -f "gz sim" 2>/dev/null' EXIT

if [ "$VIEW" = "markers" ]; then
  # best-of-N: keep the framed run (peak<FRAME_CAP) with the lowest final+rms;
  # fall back to the lowest-peak run if none stay framed.
  best_k=0; best_score=999999; best_framed=0
  for k in $(seq 1 "$RUNS"); do
    echo "== run $k/$RUNS =="
    read peak final rms ELAPSED < <(fly_once)
    echo "   peak=${peak}m final=${final}m rms=${rms}m"
    rm -rf "/tmp/run_$k"; cp -r "$FRAMES" "/tmp/run_$k"; echo "$ELAPSED" > "/tmp/run_$k.elap"
    framed=$(python3 -c "print(1 if float('$peak')<$FRAME_CAP else 0)")
    score=$(python3 -c "print(float('$final')+float('$rms'))")
    pick=$(python3 -c "
fr,sc=$framed,$score
bf,bs=$best_framed,$best_score
print(1 if (fr>bf) or (fr==bf and sc<bs) else 0)")
    if [ "$pick" = "1" ]; then best_k=$k; best_score=$score; best_framed=$framed; fi
    clean=$(python3 -c "print(1 if $framed and float('$final')<1.0 else 0)")
    [ "$clean" = "1" ] && { echo ">> run $k clean (framed, final<1m) — stopping"; break; }
  done
  echo "== best run: $best_k (framed=$best_framed score=$best_score) =="
  rm -rf "$FRAMES"; cp -r "/tmp/run_$best_k" "$FRAMES"; ELAPSED=$(cat "/tmp/run_$best_k.elap")
  encode "$OUT"
else
  echo "== flying ($VIEW, $SCEN, ${DUR}s) =="
  read _ _ _ ELAPSED < <(fly_once)
  encode "$OUT"
fi
echo "== done -> $OUT =="
ls -la "$OUT" 2>/dev/null
