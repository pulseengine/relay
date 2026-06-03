#!/usr/bin/env bash
# build-cortex-m.sh — build the VERIFIED falcon flight core into a bare-metal
# ARM Cortex-M (STM32H743) ELF that runs the FlightCore + SimBackend loop on
# the MCU (the sim as the backend, on an emulated target). Stages the ELF at
# the path renode/falcon-cortex-m.resc loads, ready for Renode/QEMU.
#
# The embedded toolchain + emulator are bench tools (not in the CI gate); this
# runs locally. Needs: `rustup target add thumbv7em-none-eabihf`.
set -euo pipefail
cd "$(dirname "$0")/.."

echo "── building falcon-cortex-m (thumbv7em-none-eabihf, release) ──"
( cd embedded/falcon-cortex-m && cargo build --release )

SRC=embedded/falcon-cortex-m/target/thumbv7em-none-eabihf/release/falcon-cortex-m
DST=target/falcon-pipeline/falcon-fused.elf
mkdir -p "$(dirname "$DST")"
cp "$SRC" "$DST"

echo "── ELF ──"
file "$DST"
echo "staged for Renode at $DST ($(wc -c < "$DST") bytes)"
echo
echo "run on the emulated STM32H743 with:"
echo "  renode renode/falcon-cortex-m.resc   # then 'start' (Renode is a bench tool)"
