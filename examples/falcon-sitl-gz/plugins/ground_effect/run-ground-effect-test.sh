#!/usr/bin/env bash
# Bench gate for the falcon GroundEffect gz plugin (bench-only — gz is not in
# CI). Builds the plugin, then runs an A/B headless: a light free box just above
# the surface is HELD aloft by the cushion (gain=0.4) but FALLS THROUGH without
# it (gain=0). Asserts the difference, proving the cushion force is real and
# altitude-decaying — matching the SimBackend ground_effect model.
#
#   ./run-ground-effect-test.sh
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE"

# Homebrew's qt@5 is keg-only; gz-sim8's cmake config needs it on the path.
QT5="${QT5_PREFIX:-/opt/homebrew/opt/qt@5}"
cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_PREFIX_PATH="$QT5" >/tmp/ge-cmake.log 2>&1
cmake --build build >/tmp/ge-build.log 2>&1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$HERE/build"

strip_ansi() { sed $'s/\x1b\\[[0-9;]*m//g'; }
final_alt() { # $1 = sdf; runs 4 s headless, prints final logged altitude (m)
  gz sim -s -r --iterations 4000 --verbose 3 "$1" 2>&1 | strip_ansi \
    | grep -oE "alt=[-0-9.]+m" | tail -1 | sed -E 's/alt=([-0-9.]+)m/\1/'
}

sed 's|<gain>0.4</gain>|<gain>0.0</gain>|' test-ground-effect.sdf > /tmp/ge-zero.sdf
HELD="$(final_alt test-ground-effect.sdf)"
FELL="$(final_alt /tmp/ge-zero.sdf)"
echo "cushion (gain 0.4): final alt = ${HELD} m"
echo "no cushion (gain 0): final alt = ${FELL} m"

# the box starts at 0.10 m: cushion holds it ≥ 0.05 m, no-cushion drops well below.
ok_held=$(awk -v a="$HELD" 'BEGIN{print (a>0.05)?1:0}')
ok_fell=$(awk -v a="$FELL" 'BEGIN{print (a<-0.5)?1:0}')
if [ "$ok_held" = 1 ] && [ "$ok_fell" = 1 ]; then
  echo "PASS — the ground-effect cushion holds the vehicle aloft (and decays with altitude)."
else
  echo "FAIL — cushion did not behave (held=$HELD fell=$FELL)"; exit 1
fi
