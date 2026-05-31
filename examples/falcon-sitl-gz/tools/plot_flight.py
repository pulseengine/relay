#!/usr/bin/env python3
"""Plot a falcon-sitl-gz flight from its evidence ticks.csv.

Usage:  python plot_flight.py <ticks.csv> [out.png]

Produces a top-down (East–North) flight path coloured by time with the
mission waypoints, plus an altitude-vs-time panel — the "data view" of a
real-Gazebo flight (complements watching the drone in the gz GUI).
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np

# Default square mission waypoints (NED, metres) — see run_mission().
WAYPOINTS = [(0, 0, -2), (2, 0, -2), (2, 2, -2), (0, 2, -2), (0, 0, -2)]


def load(path):
    t, n, e, d = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t_s"]))
            n.append(float(row["n_m"]))
            e.append(float(row["e_m"]))
            d.append(float(row["d_m"]))
    return map(np.array, (t, n, e, d))


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else "flight.png"
    t, n, e, d = load(src)

    fig, (ax, axalt) = plt.subplots(
        1, 2, figsize=(13, 5.5), gridspec_kw={"width_ratios": [1.15, 1]}
    )
    fig.patch.set_facecolor("#0d1117")
    for a in (ax, axalt):
        a.set_facecolor("#0d1117")
        a.tick_params(colors="#9da7b3")
        for s in a.spines.values():
            s.set_color("#30363d")
        a.title.set_color("#e6edf3")
        a.xaxis.label.set_color("#9da7b3")
        a.yaxis.label.set_color("#9da7b3")

    # ── Top-down East–North path, coloured by time ──
    pts = np.array([e, n]).T.reshape(-1, 1, 2)
    segs = np.concatenate([pts[:-1], pts[1:]], axis=1)
    lc = LineCollection(segs, cmap="turbo", linewidth=2.0)
    lc.set_array(t)
    ax.add_collection(lc)
    cb = fig.colorbar(lc, ax=ax, fraction=0.046, pad=0.04)
    cb.set_label("time (s)", color="#9da7b3")
    cb.ax.yaxis.set_tick_params(color="#9da7b3")
    plt.setp(plt.getp(cb.ax.axes, "yticklabels"), color="#9da7b3")

    wpe = [w[1] for w in WAYPOINTS]
    wpn = [w[0] for w in WAYPOINTS]
    ax.plot(wpe, wpn, "--", color="#f778ba", lw=1.2, alpha=0.7, label="mission")
    ax.scatter(wpe, wpn, s=80, facecolors="none", edgecolors="#f778ba", lw=1.6, zorder=5)
    ax.scatter([e[0]], [n[0]], s=90, c="#3fb950", marker="^", zorder=6, label="start/home")
    ax.scatter([e[-1]], [n[-1]], s=90, c="#58a6ff", marker="o", zorder=6, label="end")
    ax.set_xlabel("East (m)")
    ax.set_ylabel("North (m)")
    ax.set_title("Real-Gazebo waypoint mission — top-down path")
    ax.set_aspect("equal", "box")
    ax.grid(True, color="#21262d")
    leg = ax.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="#e6edf3", loc="upper left")

    final = float(np.hypot(n[-1], e[-1]))  # distance to home (0,0)
    ax.text(0.97, 0.03, f"home error: {final:.2f} m", transform=ax.transAxes,
            ha="right", va="bottom", color="#e6edf3", fontsize=10,
            bbox=dict(boxstyle="round", fc="#161b22", ec="#30363d"))

    # ── Altitude vs time ──
    axalt.plot(t, -d, color="#58a6ff", lw=1.8, label="altitude")
    axalt.axhline(2.0, color="#f778ba", ls="--", lw=1.0, alpha=0.7, label="setpoint 2 m")
    axalt.set_xlabel("time (s)")
    axalt.set_ylabel("altitude (m, up)")
    axalt.set_title("Altitude hold through the mission")
    axalt.grid(True, color="#21262d")
    axalt.legend(facecolor="#161b22", edgecolor="#30363d", labelcolor="#e6edf3", loc="lower right")

    fig.suptitle(
        "falcon — formally-verified autonomy flying in Gazebo Harmonic",
        color="#e6edf3", fontsize=14, y=0.99,
    )
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    print(f"wrote {out}  (home error {final:.2f} m, {len(t)} ticks)")


if __name__ == "__main__":
    main()
