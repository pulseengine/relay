#!/usr/bin/env bash
# annotate-rotorout-video.sh — layered explainer captions for the supervised
# rotor-out landing video (v1.117, FAULT-P04).
#
# Two-register captions per flight phase: a PLAIN-LANGUAGE line (anyone can
# follow the story) over a TECH line (PX4-author depth), per the release-video
# rule that the footage must visually show the feature and the captions must
# explain it to every audience at once.
#
# Usage:
#   annotate-rotorout-video.sh <raw.mov|mp4> <out.mp4> <kill_t> <touch_t> [end_t]
#     kill_t / touch_t : WALL seconds of the rotor-kill / touchdown in the raw
#                        footage (from the bench's "t=… KILLED / TOUCHDOWN"
#                        lines mapped through the run's wall/sim factor, or by
#                        frame inspection).
set -euo pipefail

RAW="${1:?raw video}"
OUT="${2:?output mp4}"
KILL="${3:?kill wall-time (s)}"
TOUCH="${4:?touchdown wall-time (s)}"
END="${5:-$(python3 - "$RAW" <<'PY'
import subprocess,sys
d=subprocess.run(["ffprobe","-v","quiet","-show_entries","format=duration","-of","csv=p=0",sys.argv[1]],capture_output=True,text=True).stdout.strip()
print(float(d))
PY
)}"

FONT="/System/Library/Fonts/Helvetica.ttc"
# Caption helper geometry: plain line large, tech line smaller beneath.
plain() { echo "drawtext=fontfile=$FONT:fontsize=34:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=12:x=(w-text_w)/2:y=h-150:text='$1':enable='between(t,$2,$3)'"; }
tech()  { echo "drawtext=fontfile=$FONT:fontsize=20:fontcolor=0xB8E0FF:box=1:boxcolor=black@0.55:boxborderw=10:x=(w-text_w)/2:y=h-96:text='$1':enable='between(t,$2,$3)'"; }
alert() { echo "drawtext=fontfile=$FONT:fontsize=44:fontcolor=0xFF5544:box=1:boxcolor=black@0.65:boxborderw=14:x=(w-text_w)/2:y=90:text='$1':enable='between(t,$2,$3)'"; }

T0=0; T1=5                       # title
H0=5; H1=$KILL                   # hover/climb
K0=$KILL; K1=$(echo "$KILL+2.5" | bc)      # kill flash
D0=$(echo "$KILL+1.0" | bc); D1=$TOUCH     # descent
L0=$TOUCH; L1=$(echo "$TOUCH+6" | bc)      # landed
C0=$(echo "$TOUCH+6" | bc); C1=$END        # close card

FILTERS=$(cat <<EOF
$(plain "falcon — a formally-verified drone flight stack" $T0 $T1),
$(tech "What happens when a motor dies mid-flight? (Gazebo simulation, production flight code)" $T0 $T1),
$(plain "Takeoff and hover on 4 rotors" $H0 $H1),
$(tech "Verified core: IEKF estimator → geometric SE(3) attitude → ADRC → mixer — every layer proof-carrying" $H0 $H1),
$(alert "MOTOR 1 OF 4 KILLED" $K0 $K1),
$(tech "Injected rotor failure: zero thrust, zero warning" $K0 $K1),
$(plain "Failure detected in milliseconds — landing commanded" $D0 $D1),
$(tech "CUSUM FDI on commanded-vs-achieved rotor RPM → LAND · yaw relinquished, rank-3 allocation (spin is by design)" $D0 $D1),
$(plain "Lands upright on 3 rotors. Motors provably off." $L0 $L1),
$(tech "Velocity touchdown with lift-deficit feed-forward → FSM Disarmed → estimate-only, zero actuation" $L0 $L1),
$(plain "150 randomized failure trials in simulation: 0 crashes" $C0 $C1),
$(tech "Falsifiable: wrong if a rotor-out from settled hover exceeds 45° tilt or fails an upright landing — github.com/pulseengine/relay" $C0 $C1)
EOF
)
FILTERS=$(echo "$FILTERS" | tr '\n' ' ')

ffmpeg -y -i "$RAW" -vf "$FILTERS" -c:v libx264 -pix_fmt yuv420p -crf 20 -an "$OUT"
echo "annotated -> $OUT"
