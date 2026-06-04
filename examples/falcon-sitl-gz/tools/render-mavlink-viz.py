#!/usr/bin/env python3
"""Render the falcon v1.28 MAVLink video frames from the bridge-driven flight
timeline (falcon-mavlink-viz JSONL on stdin or argv[1]).

Each frame is a 2.5-D scene — ground grid, coloured corner posts, a HOME pad,
and the quad drawn at its real NED position (with a ground shadow + altitude
line) — under a "GCS <-> FALCON" HUD: a command ticker that fills in as each
real MAVLink command is decoded, a colour-coded MODE badge that flips with the
FSM, and a live telemetry panel showing the decoded HEARTBEAT + GLOBAL_POSITION_INT.

The aircraft motion and every number on screen come from the harness (the real
bridge + verified supervisor) — nothing here is hand-animated.

  cargo run -p falcon-mavlink-viz --release > /tmp/mvz.jsonl
  render-mavlink-viz.py /tmp/mvz.jsonl /tmp/mvframes
  ffmpeg -framerate 30 -i /tmp/mvframes/f%06d.png ... falcon-v1.28.mp4
"""
import json
import os
import sys

from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FONTS = ["/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc"]
FONTS_B = ["/System/Library/Fonts/Supplemental/Arial Bold.ttf"] + FONTS
MONO = ["/System/Library/Fonts/Menlo.ttc", "/System/Library/Fonts/Courier.ttc"] + FONTS


def font(sz, bold=False, mono=False):
    for p in (MONO if mono else (FONTS_B if bold else FONTS)):
        try:
            return ImageFont.truetype(p, sz)
        except OSError:
            pass
    return ImageFont.load_default()


# ---- 2.5-D dimetric projection: NED (north, east) + up=-down -> screen ----
CX, CY = 560, 400
SX, DX = 70.0, 26.0   # east -> right ; north -> right (depth)
SY, DY = 70.0, 34.0   # up -> up      ; north -> up (depth)


def project(n, e, up):
    sx = CX + e * SX + n * DX
    sy = CY - up * SY - n * DY
    return sx, sy


# Corner posts (north, east) and colours — like the gz scene's coloured posts.
POSTS = [
    (4, 4, (90, 220, 110)),    # NE green
    (4, -4, (110, 160, 255)),  # NW blue
    (-4, 4, (235, 110, 110)),  # SE red
    (-4, -4, (230, 205, 120)), # SW yellow
]

MODE_COLOR = {
    "DISARMED": (150, 150, 158),
    "ARMED": (240, 190, 90),
    "TAKEOFF": (90, 210, 235),
    "LOITER": (120, 230, 140),
    "MISSION": (180, 160, 240),
    "RTL": (110, 160, 255),
    "LAND": (245, 165, 95),
}


def draw_scene(d, r):
    # ground grid (north/east lines from -5..5)
    grid = (44, 52, 66)
    for n in range(-5, 6):
        d.line([project(n, -5, 0), project(n, 5, 0)], fill=grid, width=1)
    for e in range(-5, 6):
        d.line([project(-5, e, 0), project(5, e, 0)], fill=grid, width=1)

    # corner posts (a vertical bar from ground up ~2.4 m)
    for n, e, col in POSTS:
        b = project(n, e, 0)
        t = project(n, e, 2.4)
        d.line([b, t], fill=col, width=7)
        d.ellipse([t[0] - 6, t[1] - 6, t[0] + 6, t[1] + 6], fill=col)

    # HOME pad at origin
    hb = project(0, 0, 0)
    d.ellipse([hb[0] - 26, hb[1] - 13, hb[0] + 26, hb[1] + 13], outline=(90, 220, 220), width=2)
    d.text((hb[0] - 18, hb[1] + 16), "HOME", font=font(14, True), fill=(90, 220, 220, 255))

    # the quad: shadow on the ground, altitude line, body
    n, e, up = r["px"], r["py"], -r["pz"]
    g = project(n, e, 0)
    p = project(n, e, up)
    d.ellipse([g[0] - 16, g[1] - 8, g[0] + 16, g[1] + 8], fill=(20, 24, 32))  # shadow
    d.line([g, p], fill=(120, 130, 145), width=1)  # altitude line
    col = MODE_COLOR.get(r["mode"], (230, 230, 230))
    armc = 16
    for dx, dy in [(-armc, -7), (armc, 7), (-armc, 7), (armc, -7)]:
        d.line([p, (p[0] + dx, p[1] + dy)], fill=(210, 215, 225), width=3)
        d.ellipse([p[0] + dx - 7, p[1] + dy - 4, p[0] + dx + 7, p[1] + dy + 4], outline=col, width=3)
    d.ellipse([p[0] - 5, p[1] - 5, p[0] + 5, p[1] + 5], fill=col)


def draw_hud(d, r, cmd_history, t):
    # top banner
    d.rectangle([0, 0, W, 86], fill=(8, 12, 20, 200))
    d.text((28, 14), "FALCON  v1.28 — MAVLink Bridge", font=font(32, True), fill=(255, 255, 255, 255))
    d.text((30, 56), "live MAVLink  <->  verified flight supervisor   ·   QGroundControl / MAVSDK",
           font=font(19), fill=(150, 200, 255, 255))

    # command ticker (left) — GCS -> FALCON
    px, py, pw = 28, 110, 300
    fired = [c for c in cmd_history if c[0] <= t + 1e-3]
    ph = 52 + 30 * max(len(fired), 1)
    d.rectangle([px, py, px + pw, py + ph], fill=(8, 12, 20, 170))
    d.text((px + 14, py + 12), "GCS  →  FALCON", font=font(18, True), fill=(150, 200, 255, 255))
    d.text((px + 14, py + 32), "COMMAND_LONG", font=font(13), fill=(120, 140, 170, 255))
    for i, (ct, label) in enumerate(fired):
        recent = (t - ct) < 1.2
        col = (255, 240, 150, 255) if recent else (210, 215, 225, 255)
        d.text((px + 16, py + 56 + 30 * i), f">  {label}", font=font(20, recent), fill=col)

    # MODE badge (right)
    mode = r["mode"]
    col = MODE_COLOR.get(mode, (230, 230, 230))
    bw, bh = 250, 70
    bx, by = W - bw - 28, 110
    d.rectangle([bx, by, bx + bw, by + bh], fill=(8, 12, 20, 200), outline=col, width=3)
    d.text((bx + 16, by + 8), "FSM MODE", font=font(14, True), fill=(150, 160, 175, 255))
    d.text((bx + 16, by + 26), mode, font=font(34, True), fill=col + (255,))

    # telemetry panel (bottom) — FALCON -> GCS
    bh2 = 132
    d.rectangle([0, H - bh2, W, H], fill=(8, 12, 20, 205))
    d.text((28, H - bh2 + 12), "FALCON  →  GCS", font=font(18, True), fill=(150, 200, 255, 255))
    d.text((232, H - bh2 + 14), "HEARTBEAT (id 0)  +  GLOBAL_POSITION_INT (id 33)",
           font=font(15), fill=(120, 140, 170, 255))
    armed = "ARMED" if (r["base_mode"] & 0x80) else "STANDBY"
    status = "ACTIVE" if r["sys_status"] == 4 else "STANDBY"
    lat = r["lat_e7"] / 1e7
    lon = r["lon_e7"] / 1e7
    alt = r["rel_alt_mm"] / 1000.0
    vz = r["vz_cms"] / 100.0
    hdg = r["hdg_cdeg"] / 100.0
    m = font(24, mono=True)
    mb = font(24, bold=True, mono=True)
    y1 = H - bh2 + 48
    y2 = H - bh2 + 84
    d.text((30, y1), f"HEARTBEAT  custom_mode={r['hb_custom']}  {armed}  {status}", font=m, fill=(210, 230, 210, 255))
    d.text((30, y2), f"POS  {lat:9.5f}°N  {lon:8.5f}°E", font=mb, fill=(235, 235, 240, 255))
    d.text((600, y2), f"ALT {alt:5.2f} m   VZ {vz:+5.2f} m/s   HDG {hdg:03.0f}°", font=m, fill=(255, 210, 140, 255))


def main():
    src = sys.argv[1] if len(sys.argv) > 1 else "/dev/stdin"
    outdir = sys.argv[2] if len(sys.argv) > 2 else "/tmp/mvframes"
    os.makedirs(outdir, exist_ok=True)
    for f in os.listdir(outdir):
        if f.endswith(".png"):
            os.remove(os.path.join(outdir, f))
    rows = [json.loads(l) for l in open(src) if l.strip()]
    cmd_history = [(r["t"], r["rx"]) for r in rows if r["rx"]]

    for i, r in enumerate(rows):
        img = Image.new("RGBA", (W, H), (16, 20, 28, 255))
        d = ImageDraw.Draw(img)
        draw_scene(d, r)
        draw_hud(d, r, cmd_history, r["t"])
        img.convert("RGB").save(os.path.join(outdir, f"f{i:06d}.png"))
    print(f"{len(rows)} frames -> {outdir}")


if __name__ == "__main__":
    main()
