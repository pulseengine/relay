# REVIEW-140 defect A: evidence for the v1.142 redesign

This directory is **not built**. These files are the probes that blocked each attempt to fix defect A in
falcon-v1.141 (#642, then #644 three times). A v1.142 design has to pass all of them against main.

| file | failure class it found |
|---|---|
| `probe_r140_jitter.rs` | the original defect: alternating dt rebuilds the filters on every tick and the vehicle tumbles |
| `probe_review_orig.rs`, `probe_review2.rs` | #642: one averager under both the filters and the FDI, where early outlier frames shrink the stall window; periodic patterns |
| `inprobe.rs` | genuine rate steps with a disturbance |
| `probe_lib.rs`, `probe_pr.rs` | #644 aca46b8: rotor vibration at 66.7 Hz aliases near Nyquist at 128–133 Hz once the low-pass is pass-through |
| `rv_probe8.rs`, `rv_probe9.rs`, `rv_run_tree.sh`, `rv_iso3.py`, `patch_v23.py` | #644 86a44a9: a minority of late frames plus vibration makes the FDI isolate a healthy rotor (250/400 Hz, up to 24/24 seeds, main 0/24); the plant integrates each frame's own dt |
| `grid/` | the 8604-cell grid (rate × rotor_vib × pre-arm × dt pattern). Blind to the late-frame class: mean-period plant, balanced jitter, 20 s, no FDI metric. |

Two lessons from it:
- Score **FDI false isolation** and position hold, not only the final mode and peak tilt.
- Use a plant that integrates each frame's **own** dt, flights of 60 s or more, minority late frames, and Monte-Carlo seeds per cell with a noise-floor control (main against main, +1 tick).

Give each tree its own `CARGO_TARGET_DIR`: cargo's mtime-based freshness check once silently reused the branch build for "main".
Refs #638, #644.
| `zz_probe_land_fdi_vib.rs` | pre-existing KNOWN LIMIT (d): FDI falsely isolates on entering Land under rotor vibration with an instant rotor (measured on main 95ab695) |
