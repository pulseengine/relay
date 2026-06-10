# bench-evidence — the cited evidence store (append-only)

This directory is **not** scratch: it is the reproducible evidence that the
verification artifacts cite. **17 rivet artifacts** (`FV-FALCON-SIM-*`,
`FV-FALCON-GZ*`, feature entries in the rollout) point at exact paths in here —
moving or pruning a cited file breaks the V-model evidence chain, which is why
this store is append-only and stays at this path.

What lives here, and why:

| Path | What it is |
|---|---|
| `gz-sim/2026-MM-DD-vX.Y-*.md` | Dated bench **findings** — the falsification write-ups per release (what was predicted, what was measured, what failed). These are the primary citations. |
| `gz-sim/*-harness.log`, `*-ticks.csv` | The **raw run evidence** behind those findings (small text/CSV only). |
| `px4-sitl/*.log` | PX4-SITL cross-check run logs (the v0.14 MavlinkBench loop). |
| `gz-sim/recordings/` | Flight video staging. **mp4 files are gitignored** — text/CSV is the in-repo evidence; videos publish to YouTube (see `recordings/.gitignore`). |

Conventions:
- **Append, don't rewrite.** A finding documents what was true at its date; later
  releases add new files rather than editing old ones.
- **Text only in git.** Logs and CSVs are kept small; anything heavy (video)
  stays out of the repo.
- A new bench finding should be cited from the FV artifact it supports —
  uncited evidence is the only thing here that may be pruned.
