#!/usr/bin/env python3
"""Animate altitude traces from tests/cascade-sitl-wasm into release footage.

Written for v1.137, where the thing being demonstrated is numeric rather than
visual: a Gazebo screen capture would look identical before and after the fix,
because the defect is a wrong integration step inside a wasm component. So the
footage IS the measurement — every pixel is driven by a CSV the harness wrote.

Usage:
  plot-trace-video.py <out-dir> <label>=<before.csv>,<after.csv> [...]
"""
import sys, os
from PIL import Image, ImageDraw, ImageFont

W, H, FPS = 1280, 720, 30
# Wall-clock seconds of video per second of simulated time. The trace should
# reveal in step with the narration rather than finishing in the first breath
# and leaving a still frame, so this is set from the voiceover length.
STRETCH = float(os.environ.get("STRETCH", "1"))
BG      = (14, 16, 20)
GRID    = (38, 42, 50)
TARGET  = (120, 130, 145)
BEFORE  = (226, 86, 74)
AFTER   = (86, 200, 130)
TEXT    = (224, 228, 235)
DIM     = (140, 148, 160)


def font(sz, bold=False):
    for p in ("/System/Library/Fonts/SFNSMono.ttf",
              "/System/Library/Fonts/Supplemental/Menlo.ttc",
              "/System/Library/Fonts/Helvetica.ttc"):
        if os.path.exists(p):
            try:
                return ImageFont.truetype(p, sz)
            except Exception:
                pass
    return ImageFont.load_default()


def read(path):
    rows = []
    with open(path) as fh:
        next(fh)
        for line in fh:
            t, alt, cmd = line.strip().split(",")
            rows.append((float(t), float(alt), float(cmd)))
    return rows


def main():
    out_dir, specs = sys.argv[1], sys.argv[2:]
    os.makedirs(out_dir, exist_ok=True)
    panels = []
    for spec in specs:
        label, files = spec.split("=", 1)
        b, a = files.split(",")
        panels.append((label, read(b), read(a)))

    dur = max(max(r[0] for r in p[1]) for p in panels)
    cmd = panels[0][1][0][2]
    # PER-PANEL scale. A shared axis sized for the 29 m runaway squashes the
    # held traces into the frame edge, which hides the thing being shown: that
    # v0.8 sits ON the target at every rate. Each panel gets the range its own
    # data needs, and the target line is drawn in every one, so the comparison
    # stays honest while both curves stay legible.
    scales = []
    for (_l, before, after) in panels:
        top = max(max(r[1] for r in before), max(r[1] for r in after), cmd * 1.6)
        scales.append((-0.35 * cmd, top * 1.10))

    f_h1, f_h2, f_lbl, f_sm = font(30, True), font(19), font(21, True), font(16)
    n = len(panels)
    pad_top, pad_bot, gap = 108, 56, 22
    ph = (H - pad_top - pad_bot - gap * (n - 1)) // n
    px0, px1 = 96, W - 250

    def y_of(py0, v, lo, hi):
        return py0 + ph - int((v - lo) / (hi - lo) * ph)

    frames = int(dur * FPS * STRETCH) + 1
    for fi in range(frames):
        t_now = fi / (FPS * STRETCH)
        im = Image.new("RGB", (W, H), BG)
        d = ImageDraw.Draw(im)
        d.text((96, 28), "wasm cascade: altitude hold vs host tick rate", font=f_h1, fill=TEXT)
        d.text((96, 72), "every pixel from a CSV the harness wrote  —  tests/cascade-sitl-wasm",
               font=f_sm, fill=DIM)

        for pi, (label, before, after) in enumerate(panels):
            lo, hi = scales[pi]
            py0 = pad_top + pi * (ph + gap)
            d.rectangle([px0, py0, px1, py0 + ph], outline=GRID)
            ty = y_of(py0, cmd, lo, hi)
            for x in range(px0, px1, 12):
                d.line([x, ty, x + 6, ty], fill=TARGET)
            d.text((px1 + 14, ty - 10), f"target {cmd:.1f} m", font=f_sm, fill=TARGET)
            d.text((px0 + 12, py0 + 8), label, font=f_lbl, fill=TEXT)
            if abs(before[-1][1] - after[-1][1]) < 0.05:
                d.text((px0 + 12, py0 + 34),
                       "identical — the one rate v0.7 assumed, so nothing ever looked wrong",
                       font=f_sm, fill=DIM)

            # v0.7 first and thicker: where the two agree exactly (the 1 kHz
            # panel) the green is drawn ON the red and both stay visible —
            # that panel is the whole reason this went unnoticed for so long.
            for rows, col, name, wdt in ((before, BEFORE, "v0.7", 6), (after, AFTER, "v0.8", 3)):
                pts, last = [], None
                for (t, alt, _c) in rows:
                    if t > t_now:
                        break
                    x = px0 + int((t / dur) * (px1 - px0))
                    y = max(py0 - 40, min(py0 + ph + 40, y_of(py0, alt, lo, hi)))
                    pts.append((x, y))
                    last = alt
                if len(pts) > 1:
                    d.line(pts, fill=col, width=wdt)
                if last is not None:
                    d.ellipse([pts[-1][0] - 4, pts[-1][1] - 4, pts[-1][0] + 4, pts[-1][1] + 4], fill=col)
                    ly = py0 + 8 + (0 if col is BEFORE else 24)
                    d.text((px1 + 14, ly), f"{name}  {last:7.2f} m", font=f_sm, fill=col)

        d.text((96, H - 40), f"t = {t_now:5.2f} s   /   {dur:.0f} s", font=f_h2, fill=DIM)
        im.save(os.path.join(out_dir, f"f{fi:05d}.png"))
    print(f"{frames} frames -> {out_dir}")


if __name__ == "__main__":
    main()
