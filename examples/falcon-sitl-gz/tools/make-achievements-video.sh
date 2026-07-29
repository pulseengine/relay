#!/usr/bin/env bash
# make-achievements-video.sh — build the ONE narrated "achievements" video that
# showcases what the falcon / relay verified flight stack achieves end to end.
#
# This is NOT a per-release clip. It tells the arc — verified cascade (IEKF +
# geometric SE(3) + Lean Lyapunov) → SITL realism → MAVLink/missions → verified
# authenticated comms (relay-sec) → drivers/failsafe/avoidance → "parity, but
# provable" — driven by REAL output (numbers captured from this repo) over REAL
# Gazebo flight footage (reused from the per-release relvids).
#
# Pipeline, per segment:
#   1. gen-release-card.py renders a 1280x720 overlay card (title/subtitle/facts);
#   2. a slice of that segment's real flight clip is cut, its narration line is
#      synthesized via TTS to size the slice to the voiceover (so audio + visuals
#      stay aligned), and the card is overlaid;
#   3. all segments are concatenated; the joined narration is muxed in.
# If the TTS server is unreachable, it builds a SILENT video and writes the
# narration script next to it (narration pending).
#
# Prereqs: run setup-video-env.sh first (Pillow venv + ffmpeg + TTS check).
#
# Usage:   make-achievements-video.sh [OUT.mp4]
#   env:   CONTENT (achievements-content.json)  CLIPDIR (relvids source footage)
#          SPEACHES_URL VOICE TTS_MODEL  SEG_SECS (fallback slice len if no TTS)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"

# python with Pillow (same resolution order as make-release-video.sh)
if [ -n "${PYTHON:-}" ]; then PY="$PYTHON"
elif [ -x "$REPO/target/video-venv/bin/python" ]; then PY="$REPO/target/video-venv/bin/python"
else PY="/tmp/plotenv/bin/python"; fi
if ! "$PY" -c 'import PIL' >/dev/null 2>&1; then
  echo "!! no Pillow-capable python ($PY). Run: $HERE/setup-video-env.sh" >&2; exit 1
fi

CONTENT="${CONTENT:-$HERE/achievements-content.json}"
CLIPDIR="${CLIPDIR:-$REPO/bench-evidence/gz-sim/recordings/relvids}"
OUTDIR="$REPO/bench-evidence/gz-sim/recordings/relvids"
OUT="${1:-$OUTDIR/falcon-achievements.mp4}"
URL="${SPEACHES_URL:-http://192.168.178.28:8000}"
VOICE="${VOICE:-af_nova}"
MODEL="${TTS_MODEL:-speaches-ai/Kokoro-82M-v1.0-ONNX}"
SEG_SECS="${SEG_SECS:-14}"          # fallback per-segment length if TTS is off
WORK="/tmp/falcon-ach"; rm -rf "$WORK"; mkdir -p "$WORK"
mkdir -p "$OUTDIR"

# is the TTS server up? (decides narrated vs silent)
TTS=0
if curl -s -m 8 "$URL/v1/models" 2>/dev/null | grep -q "Kokoro"; then TTS=1; fi
echo "== achievements video =="
echo "   content: $CONTENT"
echo "   clips:   $CLIPDIR"
echo "   tts:     $([ $TTS -eq 1 ] && echo "ON ($URL)" || echo OFF)"
echo "   out:     $OUT"

NSEG=$("$PY" -c "import json;print(len(json.load(open('$CONTENT'))['segments']))")
CONCAT="$WORK/concat.txt"; : > "$CONCAT"
NARR_ALL="$WORK/narration.txt"; : > "$NARR_ALL"
AUDIO_CONCAT="$WORK/audio.txt"; : > "$AUDIO_CONCAT"

tts_line() { # $1 text  $2 out.mp3 -> echoes duration secs (0 on failure)
  "$PY" - "$URL" "$MODEL" "$VOICE" "$1" "$2" <<'PY' 2>/dev/null || { echo 0; return; }
import sys, json, urllib.request
url, model, voice, text, out = sys.argv[1:6]
req = urllib.request.Request(url.rstrip('/')+"/v1/audio/speech",
    data=json.dumps({"model":model,"voice":voice,"input":text,"response_format":"mp3"}).encode(),
    headers={"Content-Type":"application/json"})
open(out,"wb").write(urllib.request.urlopen(req, timeout=90).read())
PY
  ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$2" 2>/dev/null || echo 0
}

for k in $(seq 0 $((NSEG-1))); do
  read -r CLIP   < <("$PY" -c "import json;print(json.load(open('$CONTENT'))['segments'][$k]['clip'])")
  read -r TITLE  < <("$PY" -c "import json;print(json.load(open('$CONTENT'))['segments'][$k]['title'])")
  read -r SUB    < <("$PY" -c "import json;print(json.load(open('$CONTENT'))['segments'][$k]['subtitle'])")
  NARR=$("$PY" -c "import json;print(json.load(open('$CONTENT'))['segments'][$k]['narration'])")
  CLIPPATH="$CLIPDIR/$CLIP"
  if [ ! -f "$CLIPPATH" ]; then echo "!! missing clip $CLIPPATH — skipping segment $k" >&2; continue; fi

  # facts -> args (bash 3.2 on macOS has no mapfile; read NUL-delimited)
  FACTS=()
  while IFS= read -r -d '' f; do FACTS+=("$f"); done < <(
    "$PY" -c "import json,sys
for x in json.load(open('$CONTENT'))['segments'][$k]['facts']: sys.stdout.write(x+'\0')")
  CARD="$WORK/card_$k.png"
  # The source relvids carry burned-in banners (top ~88px title+subtitle) and a
  # telemetry strip (bottom ~184px, y=536..720). gen-achievements-card.py renders
  # semi-transparent bands (alpha 215) that land EXACTLY on those regions, so the
  # new text replaces the old text region-for-region with NO zoom and NO crop.
  # The middle of the frame (y=90..536) stays 100 % unobscured.
  "$PY" "$HERE/gen-achievements-card.py" "$CARD" "$TITLE" "$SUB" "${FACTS[@]}" >/dev/null

  # determine segment length: narration duration (+1s pad) when TTS is on, else SEG_SECS
  SEG_LEN="$SEG_SECS"
  if [ "$TTS" -eq 1 ]; then
    DUR=$(tts_line "$NARR" "$WORK/nar_$k.mp3")
    SEG_LEN=$("$PY" -c "print(round(max(4.0,float('$DUR'))+1.0,2))")
    echo "file '$WORK/nar_$k.mp3'" >> "$AUDIO_CONCAT"
  fi
  echo "-- seg $k: $CLIP  ${SEG_LEN}s  ($TITLE)"
  printf '%s\n\n' "$NARR" >> "$NARR_ALL"

  # source clip is ~38-48s; loop it if the narration outruns it so we never
  # freeze on a black tail. Overlay the card at native 1280x720 — no zoom, no
  # crop. The card's transparent middle lets the full Gazebo scene show through.
  SEG="$WORK/seg_$k.mp4"
  ffmpeg -y -stream_loop 3 -i "$CLIPPATH" -i "$CARD" -t "$SEG_LEN" \
    -filter_complex "[0:v]scale=1280:720,fps=30,format=yuv420p[v];[v][1:v]overlay=0:0:format=auto[o]" \
    -map "[o]" -an -c:v libx264 -crf 20 -pix_fmt yuv420p -movflags +faststart \
    "$SEG" >"$WORK/ff_seg_$k.log" 2>&1
  echo "file '$SEG'" >> "$CONCAT"
done

# ── concat all video segments ──────────────────────────────────────────────
SILENT="$WORK/silent.mp4"
ffmpeg -y -f concat -safe 0 -i "$CONCAT" -c:v libx264 -crf 20 -pix_fmt yuv420p \
  -movflags +faststart "$SILENT" >"$WORK/ff_concat.log" 2>&1

if [ "$TTS" -eq 1 ] && [ -s "$AUDIO_CONCAT" ]; then
  # join per-segment narration mp3s in order, then mux (1s lead-in so the
  # opening word doesn't clip the very first frame).
  ffmpeg -y -f concat -safe 0 -i "$AUDIO_CONCAT" -c copy "$WORK/narration.mp3" \
    >"$WORK/ff_audiocat.log" 2>&1
  ffmpeg -y -i "$SILENT" -i "$WORK/narration.mp3" \
    -filter_complex "[1:a]adelay=800|800[a]" \
    -map 0:v -map "[a]" -c:v copy -c:a aac -b:a 160k -shortest \
    "$OUT" >"$WORK/ff_mux.log" 2>&1
  echo "== narrated -> $OUT"
else
  cp "$SILENT" "$OUT"
  cp "$NARR_ALL" "${OUT%.mp4}-narration.txt"
  echo "== SILENT (TTS off) -> $OUT"
  echo "   narration script written: ${OUT%.mp4}-narration.txt (narration pending)"
fi
echo "   $(ffprobe -v error -show_entries format=duration -of default=nw=1:nk=1 "$OUT" 2>/dev/null)s, $(wc -c < "$OUT") bytes"
