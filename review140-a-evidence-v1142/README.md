# Known limit A (#638): v1.142 evidence, three designs rejected

This directory is **not built**. It holds the probes and drivers that rejected
three v1.142 designs for known limit A. The maintainer deferred the fix to
falcon-v1.143.0 on 2026-11-08. A v1.143 design has to pass all of them
against main. The earlier set is `review140-a-evidence/` on branch
`fix/review140-a-filter-rate`.

The code commit on this branch, 2f9314b, is the last rejected design
(the 16/15 guard). It is kept for reference and is not meant to merge.

| design | rejected by | measurement |
|---|---|---|
| freeze the filter design at arm | `rv_probe9.rs` (rv_grid9) | late10 + vib 0.4 at 250 Hz: 24/24 healthy-rotor isolations, main 0/24 |
| alternation guard, enter 12 of 16, exit at 4 | cold review, `rv_adv.rs` | late40 (1.5x) + vib 0.4 at 250 Hz: 25/48, main 1/48; late5 differs in every seed |
| alternation guard, enter 16 of 16, exit at 15 (2f9314b) | second cold review, `rv_adv2.rs` | alternation around 4.8-6.0 ms + vib 0.3: 48/48 isolations, main 0/48; `flap16` at 250 Hz: 48/48 fail, main 6/48 |

What all three show: holding the filter design loses to main's per-frame
redesign whenever there is rotor vibration. A notch with a held design cannot
null vibration sampled at irregular intervals. Main's defect is the
**zeroed state** on each redesign, not the redesign itself. The v1.143
candidate is a per-frame retune that keeps the filter state. Its known risk
from #644 is ringing and aliasing near Nyquist (120-133 Hz), so it needs the
near-Nyquist grid too.

Method that the grids rely on:
- The plant integrates each frame's **own** dt.
- Flights run 60 s, with Monte-Carlo seeds per cell.
- A noise floor comes from `RV_PERTURB=1` on both trees.
- Outcomes are scored on FDI false isolation, final mode, tilt and hold.
- Each tree gets its own `CARGO_TARGET_DIR`.

Drivers in `adv/` and `rv_run_tree.sh` use absolute scratch paths from the
session that wrote them; edit `S=` before use. `rv_run_tree.sh` appends
`mod X { include!(...) }` to falcon-core's lib.rs, so remove those lines
afterwards (python; BSD sed has no `\|` alternation).

| file | what it runs |
|---|---|
| `rv_probe9.rs`, `rv_probe8.rs` | late-frame grids: 360 and 1296 cells |
| `rv_altgrid.rs` | alternation and random jitter: 672 cells |
| `rv_adv.rs` | first review: 16 patterns, rotor-out latency, near-Nyquist via env |
| `rv_adv2.rs` | second review: about 30 patterns; scenarios swfix, swalt, fixsw, cycles, step |
| `rv_trace.rs`, `rv_tr2_*.rs` | per-tick traces of guard episodes |
| `adv/guardsim.py` | Python replica of the guard's crossing logic |
| `adv/sum.py`, `adv/sum2.py`, `rv_iso3.py` | summarisers |

Refs #638, #644.
