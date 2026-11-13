# Wire-decoder conformance vectors

Each `gen_<target>.py` builds reference vectors with an independent reference
library (pyubx2, dronecan, pymavlink) and prints them as Rust source. The
integration test `crates/<crate>/tests/conformance_<target>.rs` includes the
committed output from `crates/<crate>/tests/vectors/<target>.rs`. Every
generator is seeded, so its output is deterministic.

| target | reference library | crate |
|---|---|---|
| `ubx_nav_pvt` | pyubx2 1.4.1 | falcon-gnss-ubx |
| `dronecan_msg` (NodeStatus, esc.RawCommand) | dronecan 1.0.27 | relay-dronecan |
| `dronecan_sensors` (esc.Status, mag, baro, battery) | dronecan 1.0.27 | relay-dronecan |
| `mavlink_frame` | pymavlink 2.4.50 | relay-mavlink |
| `mavlink_heartbeat` | pymavlink 2.4.50 | relay-mavlink |
| `mavlink_command_long` | pymavlink 2.4.50 | relay-mavlink |
| `mavlink_global_position_int` | pymavlink 2.4.50 | relay-mavlink |

## Regenerate

```sh
python3 -m venv .venv && .venv/bin/pip install -r scripts/conformance/requirements.txt
for g in scripts/conformance/gen_*.py; do
  t=$(basename "$g" .py); t=${t#gen_}
  f=$(ls crates/*/tests/vectors/"$t".rs)
  .venv/bin/python -I "$g" > "$f"
done
git diff --stat crates/*/tests/vectors/   # empty with the pinned versions
```

Each generator prints a coverage summary as JSON on stderr: per-field counts
of zero, minimum, maximum and negative values, plus how many candidate
vectors it skipped because the library disagreed with the intended encoding.

`cargo fmt` does not format `include!`d files, so the raw generator output is
the committed form.

## MAVLink2 truncated payloads

MAVLink2 senders drop trailing zero bytes from a payload (#675). The MAVLink
tests check that the decoders reject an empty or over-long payload and decode a
shortened one as if zero-extended.
