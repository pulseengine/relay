#!/usr/bin/env bash
# Add an af_nicole / Kokoro voiceover (a short success description) to each
# per-release falcon video via a self-hosted Speaches server (OpenAI-compatible
# TTS), and emit the channel metadata (title + description per release) for
# upload. Reads release-narration.json.
#
# Usage:   narrate-release-videos.sh
#   env:   SPEACHES_URL (default http://192.168.178.28:8000)
#          VOICE (af_nicole)   TTS_MODEL (speaches-ai/Kokoro-82M-v1.0-ONNX)
#          VIDDIR (bench-evidence/gz-sim/recordings/relvids)
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../../.." && pwd)"
URL="${SPEACHES_URL:-http://192.168.178.28:8000}"
VOICE="${VOICE:-af_nicole}"
MODEL="${TTS_MODEL:-speaches-ai/Kokoro-82M-v1.0-ONNX}"
VIDDIR="${VIDDIR:-$REPO/bench-evidence/gz-sim/recordings/relvids}"
# python is only used here for stdlib (json + urllib) — the stable venv works,
# any python3 works too. Prefer the setup-video-env.sh venv; fall back to legacy
# /tmp/plotenv, then plain python3. Run setup-video-env.sh first.
if [ -n "${PYTHON:-}" ]; then PY="$PYTHON"
elif [ -x "$REPO/target/video-venv/bin/python" ]; then PY="$REPO/target/video-venv/bin/python"
elif [ -x "/tmp/plotenv/bin/python" ]; then PY="/tmp/plotenv/bin/python"
else PY="$(command -v python3)"; fi
DATA="$HERE/release-narration.json"
META="$VIDDIR/channel-metadata.md"
mkdir -p "$VIDDIR"
echo "# Falcon release videos — channel metadata" > "$META"
echo "" >> "$META"

n=$($PY -c "import json;print(len(json.load(open('$DATA'))['releases']))")
for k in $(seq 0 $((n-1))); do
  read -r VER < <($PY -c "import json;print(json.load(open('$DATA'))['releases'][$k]['version'])")
  NAR=$($PY -c "import json;print(json.load(open('$DATA'))['releases'][$k]['narration'])")
  TIT=$($PY -c "import json;print(json.load(open('$DATA'))['releases'][$k]['title'])")
  VID="$VIDDIR/falcon-${VER}.mp4"
  [ -f "$VID" ] || { echo "skip $VER (no video $VID)"; continue; }
  echo "== $VER : TTS + mux =="
  # 1) TTS the narration (OpenAI-compatible /v1/audio/speech)
  $PY - "$URL" "$MODEL" "$VOICE" "$NAR" /tmp/nar.mp3 <<'PY'
import sys, json, urllib.request
url, model, voice, text, out = sys.argv[1:6]
req = urllib.request.Request(url.rstrip('/')+"/v1/audio/speech",
    data=json.dumps({"model":model,"voice":voice,"input":text,"response_format":"mp3"}).encode(),
    headers={"Content-Type":"application/json"})
open(out,"wb").write(urllib.request.urlopen(req, timeout=60).read())
print("tts ok", len(open(out,'rb').read()), "bytes")
PY
  # 2) mux: narration starts at 1 s, over the (silent) video; video length kept
  ffmpeg -y -i "$VID" -i /tmp/nar.mp3 \
    -filter_complex "[1:a]adelay=1000|1000[a]" \
    -map 0:v -map "[a]" -c:v copy -c:a aac -b:a 160k \
    "$VIDDIR/falcon-${VER}-narrated.mp4" >/tmp/ff-narrate.log 2>&1
  echo "   -> falcon-${VER}-narrated.mp4"
  # 3) channel metadata
  DESC=$($PY -c "import json;print(json.load(open('$DATA'))['releases'][$k]['description'])")
  { echo "## $VER"; echo "**Title:** $TIT"; echo ""; echo "$DESC"; echo ""; echo "---"; echo ""; } >> "$META"
done
echo "== done. narrated videos + $META =="
