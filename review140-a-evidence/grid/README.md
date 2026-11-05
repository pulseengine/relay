# Filter-rate oracle grid (review of v1.140 defect A, PR #644)

One harness over every plant input the reviews of PR #644 touched, to
compare a branch against main cell by cell.

## What a cell is

`zg_grid.rs` is a falcon-core integration test (common public API only, so
it compiles on 725f146 and on the branch). Each cell: SimBackend with
`gyro_white 0.02, vibration 0.5`, ground contact, a 5 s pre-arm hold, Arm +
RequestTakeoff, 20 s supervised flight; `rotor_vib` is switched on at the
first Loiter. Recorded: start mode, whether Loiter was reached, final mode,
peak true tilt (non-finite = `inf`).

Axes (8604 cells; 120.000 appears twice, 8568 unique):

| axis | values |
|---|---|
| loop rate (Hz) | 10..100 step 10; 100.5..200 step 0.5; 120.000..120.020 step 0.001; 250, 300, 400, 500, 1000, 2000, 5000, 10000 |
| rotor vib (after Loiter) | 0, 0.1, 0.25, 0.3 |
| pre-arm | `plain`; `bump` = 3 rad/s gyro bump over the hold's last 20 ms; `rot` = 0.5 rad/s rotation through the whole hold |
| dt pattern | `fixed`; `alt` = strict alternation dt x 0.875 / 1.125; `rand` = the same two levels in seeded random order |

The plant integrates at the pattern's mean period; only the REPORTED frame
period jitters. `GRID_PERTURB=1` holds one tick longer (a physically
meaningless change) to measure the noise floor.

## Run

```bash
# main in its own tree and target dir (cargo's freshness check is mtime-based;
# a shared target dir silently reuses the other tree's build)
git -C <relay> archive 725f146 | tar -x -C /tmp/main-tree
TD=/tmp/target-main ./grid.sh /tmp/main-tree parent
TD=/tmp/target-main GRID_PERTURB=1 ./grid.sh /tmp/main-tree parentP   # noise floor
TD=/tmp/target-branch ./grid.sh <branch-worktree> fix4
```

About 15 s per tree (9 threads) after the build.

## Analysis

```bash
python3 cmpgrid.py parent fix4 [N]       # per-band same / better / WORSE, N worst listed
python3 pervib.py parentP fix4 aca46b8   # per rotor-vib: same / mode-better / tilt-better / mode-worse / tilt-worse
python3 failrate.py parent parentP fix4 aca46b8   # failures (Land/Disarmed/Terminated) per band x vib x pattern
python3 breakdown.py parent fix4 [lo] [hi]        # mode changes per vib x pattern in a rate range
python3 vsfixed.py parent fix4 [N]       # the maintainer's gate: >=250 Hz vs main same cell, <250 Hz vs main FIXED dt
```

Classification: final-mode rank first (Loiter < Rtl/Mission/Takeoff < Land
< Disarmed < Terminated); same mode -> worse if peak tilt exceeds the
reference by more than max(0.01 rad, 20 %); non-finite is worse than finite.

## Files

| file | content |
|---|---|
| `grid_parent.txt` | main 725f146 |
| `grid_parentP.txt` | main 725f146, `GRID_PERTURB=1` (noise floor) |
| `grid_aca46b8.txt` | PR #644 at aca46b8 (near-Nyquist pass-through; the regression of review 2) |
| `grid_fix4.txt` | PR #644 at c54fc2e (the accepted fix) |
| `cmp_parent_*.txt` | `cmpgrid.py` output, every worse/better cell listed |
| `pervib.txt`, `failrate.txt` | the two summary tables |
| `vsfixed_parent_*.txt` | the gate (`vsfixed.py`) for c54fc2e, main itself, and the noise floor |

## Results (c54fc2e vs main)

- >= 250 Hz: 0 worse (288 cells).
- All rates, vs main's same cell: 668 worse / 1268 better / 6668 same; the
  noise floor (main vs main + 1 tick) is 601 worse / 595 better. At rotor
  vib 0: 0 worse.
- Below 250 Hz vs main's FIXED dt: 881 worse (vib 0 / 0.1 / 0.25 / 0.3: 12 /
  81 / 372 / 416); main's own jittered cells fail the same check 1412 times
  (127 / 311 / 509 / 465), so this measures dt jitter itself, not the
  filter. The vib-0 cells are all <= 60 Hz, where main fixed dt already
  ends Disarmed / Rtl.
- 140-200 Hz with rotor vib >= 0.25: jittered dt now fails like main's fixed
  dt (alternation 38 / 99, random 84 / 107 of 363 at vib 0.25 / 0.3; main
  fixed 38 / 97; main jittered 0-2).
