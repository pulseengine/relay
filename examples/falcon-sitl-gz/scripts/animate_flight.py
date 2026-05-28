#!/usr/bin/env python3
"""Render a falcon-sitl-gz bench ticks.csv as a flight animation (mp4).

Headless (Agg backend), needs matplotlib + ffmpeg on PATH. Reads the
v0.19.0 ticks.csv schema:

    step,t_s,n_m,e_m,d_m,ax_body,ay_body,az_body,gx_body,gy_body,gz_body,
    m0,m1,m2,m3,imu_recv,navsat_recv,motor_send

Renders two synced panels:
  - left:  3D NED trajectory (altitude = -d), quad marker + trail.
  - right: altitude vs time with the setpoint line.

Usage:
    animate_flight.py <ticks.csv> <out.mp4> [--setpoint-alt 2.0] [--fps 25]

This is faithful to the *logged* flight — it's a visualisation of the
same NED positions the verified geofence + the bench verdict consume,
not a re-render of Gazebo's scene. For Gazebo's own 3D footage use
`record_gazebo_flight.sh` (gz VideoRecorder).
"""
import csv
import sys
import argparse

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter


def load(path):
    t, n, e, alt, m = [], [], [], [], []
    with open(path) as f:
        r = csv.DictReader(f)
        for row in r:
            t.append(float(row["t_s"]))
            n.append(float(row["n_m"]))
            e.append(float(row["e_m"]))
            alt.append(-float(row["d_m"]))  # NED down → altitude
            m.append([float(row[k]) for k in ("m0", "m1", "m2", "m3")])
    return t, n, e, alt, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("out")
    ap.add_argument("--setpoint-alt", type=float, default=2.0)
    ap.add_argument("--fps", type=int, default=25)
    ap.add_argument("--decimate", type=int, default=4,
                    help="plot every Nth tick (100 Hz log → 25 fps at 4)")
    a = ap.parse_args()

    t, n, e, alt, _m = load(a.csv)
    idx = list(range(0, len(t), a.decimate))

    fig = plt.figure(figsize=(11, 5))
    ax3d = fig.add_subplot(1, 2, 1, projection="3d")
    axalt = fig.add_subplot(1, 2, 2)

    nmin, nmax = min(n) - 0.5, max(n) + 0.5
    emin, emax = min(e) - 0.5, max(e) + 0.5
    amax = max(max(alt) + 0.5, a.setpoint_alt + 0.5)

    def setup_3d():
        ax3d.clear()
        ax3d.set_xlabel("North (m)")
        ax3d.set_ylabel("East (m)")
        ax3d.set_zlabel("Altitude (m)")
        ax3d.set_xlim(nmin, nmax)
        ax3d.set_ylim(emin, emax)
        ax3d.set_zlim(0, amax)
        ax3d.set_title("falcon-cascade — gz-sim flight (NED)")

    def setup_alt():
        axalt.clear()
        axalt.set_xlabel("t (s)")
        axalt.set_ylabel("Altitude (m)")
        axalt.set_xlim(0, t[-1])
        axalt.set_ylim(0, amax)
        axalt.axhline(a.setpoint_alt, ls="--", c="tab:green", label="setpoint")
        axalt.legend(loc="lower right")
        axalt.set_title("Altitude hold")

    def frame(fi):
        k = idx[fi]
        setup_3d()
        setup_alt()
        # trail + marker
        ax3d.plot(n[: k + 1], e[: k + 1], alt[: k + 1], c="tab:blue", lw=1)
        ax3d.scatter([n[k]], [e[k]], [alt[k]], c="tab:red", s=60, marker="o")
        ax3d.scatter([0], [0], [a.setpoint_alt], c="tab:green", s=40, marker="x")
        axalt.plot(t[: k + 1], alt[: k + 1], c="tab:blue", lw=1.5)
        axalt.scatter([t[k]], [alt[k]], c="tab:red", s=40)
        fig.suptitle(f"t = {t[k]:5.2f} s   alt = {alt[k]:5.2f} m", fontsize=12)
        return []

    anim = FuncAnimation(fig, frame, frames=len(idx), blit=False)
    writer = FFMpegWriter(fps=a.fps, bitrate=2400)
    anim.save(a.out, writer=writer)
    print(f"wrote {a.out} ({len(idx)} frames @ {a.fps} fps)")


if __name__ == "__main__":
    main()
