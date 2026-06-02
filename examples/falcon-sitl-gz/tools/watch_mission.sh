#!/usr/bin/env bash
# Watch (and screen-record) the falcon fly in Gazebo Harmonic with the GUI.
#
#   ./watch_mission.sh [scenario] [duration_s]
#       scenario : mission (default) | geo-hover
#
# Opens the gz GUI (a 3-D window with the quadrotor + world), then flies the
# scenario through the full verified cascade so you can watch + record it.
# Also writes the flight log to /tmp/falcon-flight for the data-view plot.
set -e
cd "$(dirname "$0")/../../.."                     # repo root
WORLD="$PWD/examples/falcon-sitl-gz/worlds/falcon-quad.sdf"
HOME_LL="47.3977,8.5456,488"
SCEN="${1:-mission}"
DUR="${2:-50}"
mkdir -p /tmp/falcon-flight

pkill -f 'gz sim' 2>/dev/null || true; sleep 2
# macOS needs the server (-s) and GUI (-g) as SEPARATE processes
# (gazebosim/gz-sim#44); on Linux you can use a single `gz sim -r`.
echo "Starting headless server…"
gz sim -s -r -v1 "$WORLD" >/tmp/gz-srv.log 2>&1 &
sleep 4
echo "Opening the Gazebo GUI window on your screen…"
gz sim -g >/tmp/gz-gui.log 2>&1 &
sleep 10
echo "Flying scenario=$SCEN for ${DUR}s — watch the drone in the GUI window."
cargo run -q -p falcon-sitl-gz --features gazebo -- \
  --backend=gazebo --world=falcon --model=quad --home="$HOME_LL" \
  --scenario="$SCEN" --duration="$DUR" --evidence-dir=/tmp/falcon-flight
echo
echo "Flight done. Make the data-view plot with:"
echo "  /tmp/plotenv/bin/python examples/falcon-sitl-gz/tools/plot_flight.py \\"
echo "      /tmp/falcon-flight/*-${SCEN}-ticks.csv /tmp/falcon-flight/${SCEN}.png"
pkill -f 'gz sim' 2>/dev/null || true
