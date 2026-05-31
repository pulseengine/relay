#!/usr/bin/env python3
"""Render an animated flight VIDEO (mp4) from a falcon-sitl-gz ticks.csv.

Usage:  python animate_flight.py <ticks.csv> [out.mp4] [video_seconds]

A top-down East–North view with the path drawing over time + a quadrotor
marker + the mission waypoints, alongside a live altitude trace. The
data-driven 'flight video' (complements screen-recording the gz GUI).
"""
import csv
import sys

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FFMpegWriter
import numpy as np

WAYPOINTS = [(0, 0), (2, 0), (2, 2), (0, 2), (0, 0)]  # (North, East)
BG, FG, MUT, ACC, PINK, GRN = "#0d1117", "#e6edf3", "#9da7b3", "#58a6ff", "#f778ba", "#3fb950"


def load(path):
    t, n, e, d = [], [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r["t_s"])); n.append(float(r["n_m"]))
            e.append(float(r["e_m"])); d.append(float(r["d_m"]))
    return map(np.array, (t, n, e, d))


def style(a):
    a.set_facecolor(BG)
    a.tick_params(colors=MUT)
    for s in a.spines.values():
        s.set_color("#30363d")
    a.title.set_color(FG); a.xaxis.label.set_color(MUT); a.yaxis.label.set_color(MUT)


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else "flight.mp4"
    vid_s = float(sys.argv[3]) if len(sys.argv) > 3 else 24.0
    fps = 30
    t, n, e, d = load(src)

    nframes = int(vid_s * fps)
    idx = np.linspace(0, len(t) - 1, nframes).astype(int)

    fig = plt.figure(figsize=(12, 6.2), facecolor=BG)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.25, 1], wspace=0.25)
    ax = fig.add_subplot(gs[0]); axa = fig.add_subplot(gs[1])
    style(ax); style(axa)

    # static: waypoints + square
    wpe = [w[1] for w in WAYPOINTS]; wpn = [w[0] for w in WAYPOINTS]
    ax.plot(wpe, wpn, "--", color=PINK, lw=1.1, alpha=0.6)
    ax.scatter(wpe, wpn, s=70, facecolors="none", edgecolors=PINK, lw=1.5, zorder=3)
    ax.scatter([0], [0], s=80, c=GRN, marker="^", zorder=4)
    m = 0.6
    ax.set_xlim(min(e.min(), -0.5) - 0.2, max(e.max(), 2.0) + 0.2)
    ax.set_ylim(min(n.min(), -0.5) - 0.2, max(n.max(), 2.0) + 0.2)
    ax.set_aspect("equal", "box"); ax.grid(True, color="#21262d")
    ax.set_xlabel("East (m)"); ax.set_ylabel("North (m)")
    ax.set_title("Real-Gazebo waypoint mission — top-down")

    axa.set_xlim(0, t[-1]); axa.set_ylim(0, max(2.4, (-d).max() + 0.2))
    axa.axhline(2.0, color=PINK, ls="--", lw=1.0, alpha=0.6)
    axa.grid(True, color="#21262d"); axa.set_xlabel("time (s)")
    axa.set_ylabel("altitude (m, up)"); axa.set_title("Altitude hold")

    fig.suptitle("falcon — formally-verified autonomy flying in Gazebo Harmonic",
                 color=FG, fontsize=14, y=0.98)

    (trail,) = ax.plot([], [], color=ACC, lw=2.0, alpha=0.9)
    rotor = ax.scatter([], [], s=[], c=[], zorder=6)
    (altline,) = axa.plot([], [], color=ACC, lw=1.8)
    altdot = axa.scatter([], [], s=60, c=PINK, zorder=5)
    txt = ax.text(0.03, 0.97, "", transform=ax.transAxes, va="top", color=FG,
                  fontsize=10, bbox=dict(boxstyle="round", fc="#161b22", ec="#30363d"))

    def quad(cx, cy):
        # a little quadrotor: 4 rotor dots in an X + body
        r = 0.16
        xs = [cx - r, cx + r, cx - r, cx + r, cx]
        ys = [cy - r, cy + r, cy + r, cy - r, cy]
        return xs, ys

    def frame(k):
        i = idx[k]
        trail.set_data(e[: i + 1], n[: i + 1])
        xs, ys = quad(e[i], n[i])
        rotor.set_offsets(np.c_[xs, ys])
        rotor.set_sizes([55, 55, 55, 55, 130])
        rotor.set_color([ACC, ACC, ACC, ACC, "#ffffff"])
        altline.set_data(t[: i + 1], -d[: i + 1])
        altdot.set_offsets([[t[i], -d[i]]])
        home = float(np.hypot(n[i], e[i]))
        txt.set_text(f"t = {t[i]:4.1f} s\nalt = {-d[i]:.2f} m\nhome dist = {home:.2f} m")
        return trail, rotor, altline, altdot, txt

    writer = FFMpegWriter(fps=fps, bitrate=4000,
                          metadata={"title": "falcon autonomy — Gazebo"})
    from matplotlib.animation import FuncAnimation
    anim = FuncAnimation(fig, frame, frames=nframes, blit=False)
    anim.save(out, writer=writer, dpi=120,
              savefig_kwargs={"facecolor": BG})
    print(f"wrote {out}  ({nframes} frames @ {fps}fps ≈ {vid_s:.0f}s)")


if __name__ == "__main__":
    main()
