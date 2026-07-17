#!/usr/bin/env bash
# Float discipline gate (v1.123, SWDD-FALCON-FLOAT-001).
#
# The flight core is f32 BY REQUIREMENT: the estimator partition runs on the
# i.MX RT1176's Cortex-M4, whose FPU (FPv4-SP) has no double-precision
# hardware — every f64 op there is soft-float, an order of magnitude slower
# (a WCET blow-up on the hottest loop). f64 belongs at exactly one place:
# the geodetic/API boundary, where absolute latitude/longitude in f32 would
# quantize at ~0.6 m (24-bit mantissa at mid-latitudes).
#
# This gate denies the `f64` token in the flight-path crates' non-test code.
# A file whose header carries the marker
#     //! float-discipline: allow-f64
# is an ANNOTATED boundary module (say why on the same line) and is exempt.
# Test / kani regions (`#[cfg(test)]` / `#[cfg(kani)]` to end-of-file, the
# repo's layout convention) are exempt — test oracles SHOULD be f64.
set -u

# The M4/M7 flight-partition crates (estimator, cascade, failsafe, drivers).
FLIGHT_CRATES=(
  relay-math relay-iekf relay-geo relay-adrc relay-mix-quad
  relay-fsm relay-preflight relay-calib relay-fsafe relay-rc
  relay-param relay-batt relay-notch relay-flowrange relay-log
  relay-mavlink falcon-mavlink falcon-core falcon-gnss-ubx falcon-baromag
  falcon-imu-icm42688 falcon-esc-dshot
)

fail=0
for crate in "${FLIGHT_CRATES[@]}"; do
  for src in "crates/$crate/plain/src" "crates/$crate/src"; do
    [ -d "$src" ] || continue
    # skip the verus twin when a plain tree exists (plain is what ships)
    if [ "$src" = "crates/$crate/src" ] && [ -d "crates/$crate/plain/src" ]; then
      continue
    fi
    while IFS= read -r -d '' f; do
      case "$(basename "$f")" in kani_proofs.rs|tests.rs) continue ;; esac
      if head -5 "$f" | grep -q "float-discipline: allow-f64"; then
        continue
      fi
      # portable word-boundary (BSD awk has no \y — a \y pattern silently
      # matches NOTHING there, which made the gate pass vacuously on macOS;
      # caught when CI's gawk found a hit the local run missed)
      # Match f64 in CODE only: exempt the cfg(test)/cfg(kani) region and
      # skip comment-only lines (a doc/line comment discussing the f64
      # policy — like relay-math's kernel notes — is not an f64 dependency).
      hits=$(awk '/#\[cfg\((test|kani)\)\]/{exempt=1}
                  { line=$0; sub(/^[[:space:]]+/,"",line) }
                  !exempt && line !~ /^\/\// && /(^|[^A-Za-z0-9_])f64([^A-Za-z0-9_]|$)/{print FILENAME":"FNR": "$0}' "$f")
      if [ -n "$hits" ]; then
        echo "FLOAT-DISCIPLINE FAIL — f64 in flight-path code (annotate the"
        echo "file '//! float-discipline: allow-f64 (<why>)' ONLY if it is a"
        echo "genuine geodetic/API boundary; the M4 has no f64 hardware):"
        echo "$hits"
        fail=1
      fi
    done < <(find "$src" -name '*.rs' -print0)
  done
done

if [ "$fail" = "0" ]; then
  echo "float discipline: OK (f32 flight core; f64 only at annotated boundaries)"
fi
exit $fail
