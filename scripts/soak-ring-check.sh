#!/usr/bin/env bash
# soak-ring-check.sh <harness-output-file>
#
# The #270 ring verdict for soak.yml's "The ring" step, judged on the
# `  ring: prekill_ticks=N prekill_rp_rate_rms=Xrad/s prekill_saturated_frac=Y`
# summary line that `falcon-sitl-gz --scenario=supervised-rotorout` prints on
# every run. Exit 0 = "ring quiet"; anything else is a failure.
#
# It lives in a file, not inline in the workflow, so the logic CI runs is the
# logic that was exercised locally against captured good and bad lines (#609
# finding 14). The inline version failed OPEN three ways, all reproduced:
#   - a non-finite RMS (Rust `{:.4}` prints NaN / inf / -inf) did not match the
#     sed capture `([0-9.]+)`, so RMS became the whole line, awk compared it as
#     a STRING (' ' < '0') and reported "ring quiet";
#   - a non-numeric tick count made `[ "$TICKS" -lt 100 ]` error with status 2
#     inside an `if`, which reads as false, so the empty-scope guard was
#     skipped;
#   - a dead IMU stream: the gz adapter zero-fills a silent IMU, so a hover with
#     no gyro samples at all measures RMS 0.0000 and read as the quietest ring
#     ever.
# So every number is validated as a plain decimal BEFORE it is compared, and a
# value that is not one fails closed.
#
# THE VERDICT LINES ARE AN API: soak's readers grep for "ring quiet" and
# "THE RING IS PRESENT". Keep them byte-identical.
set -euo pipefail

OUT="${1:?usage: soak-ring-check.sh <harness-output-file>}"

LINE=$(grep -m1 '^  ring: ' "$OUT" || true)
if [ -z "$LINE" ]; then
  echo "::error::no 'ring:' summary line — the harness did not report the pre-kill window, so this check verified NOTHING"
  exit 1
fi
echo "::notice::$LINE"

# field <name> <suffix>: the value of `<name>=<value><suffix>` as written,
# up to the next space. Empty if the field is absent.
field() {
  printf '%s\n' "$LINE" | tr ' ' '\n' | sed -n "s/^$1=\(.*\)$2\$/\1/p" | head -n1
}
RMS=$(field prekill_rp_rate_rms 'rad\/s')
SAT=$(field prekill_saturated_frac '')
TICKS=$(field prekill_ticks '')

# A NUMBER OR NOTHING. NaN, inf, -inf, an empty capture or any other text is
# not a measurement, and must not reach a comparison that would read it as one.
is_uint() { [[ "$1" =~ ^[0-9]+$ ]]; }
is_decimal() { [[ "$1" =~ ^[0-9]+(\.[0-9]+)?$ ]]; }
if ! is_uint "$TICKS"; then
  echo "::error::pre-kill tick count '${TICKS}' is not a count — the window was not measured, so this check verified NOTHING"
  exit 1
fi
# EMPTY SCOPE MUST NOT EQUAL PASS: zero pre-kill ticks means the window
# never existed, which would make any bar below vacuously true. (Checked
# before the RMS, because an empty window prints its RMS as NaN.)
if [ "$TICKS" -lt 100 ]; then
  echo "::error::only ${TICKS} pre-kill ticks — too few to judge the hover; the bar below would pass vacuously"
  exit 1
fi
if ! is_decimal "$RMS"; then
  echo "::error::pre-kill roll/pitch RMS '${RMS}' is not a finite number — a non-finite gyro sample in the pre-kill window is a FAILURE, not a quiet hover (#270)"
  exit 1
fi
if ! is_decimal "$SAT"; then
  echo "::error::pre-kill saturated fraction '${SAT}' is not a finite number — the window was not measured (#270)"
  exit 1
fi

# A DEAD IMU IS NOT A QUIET ONE. The gz adapter zero-fills a silent IMU, so an
# advertised-but-silent stream measures exactly 0.0000. This is a liveness
# check, not a new bar: a live gz IMU has never measured below 0.0015 rad/s
# here (20 soak runs, 2026-10-13..11-03, 19 scheduled + 1 dispatched, all
# 0.0015-0.0016), and
# `{:.4}` prints 0.0000 only below 0.00005, thirty times under that.
if awk -v r="$RMS" 'BEGIN { exit !(r + 0 == 0) }'; then
  echo "::error::pre-kill roll/pitch RMS is exactly ${RMS} over ${TICKS} ticks — that is a SILENT IMU (zero-filled), not a quiet hover; the ring was NOT measured (#270)"
  exit 1
fi

echo "pre-kill: ticks=$TICKS  rp_rate_rms=$RMS rad/s  saturated_frac=$SAT"
if awk -v r="$RMS" 'BEGIN { exit !(r + 0 >= 0.5) }'; then
  echo "::error::THE RING IS PRESENT: pre-kill roll/pitch RMS ${RMS} rad/s >= 0.5 (quiet is ~0.0014, ringing ~2). Saturated fraction ${SAT}. #270"
  exit 1
fi
echo "::notice::ring quiet — pre-kill roll/pitch RMS ${RMS} rad/s < 0.5, saturated fraction ${SAT} (#270)"
