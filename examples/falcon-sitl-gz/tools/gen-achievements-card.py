#!/usr/bin/env python3
"""Render an 'achievements' overlay card (1280x720, transparent middle) for the
single falcon achievements video.

Unlike gen-release-card.py, this is sized to fully MASK the burned-in banners
that the source relvids already carry (their own top banner ~86 px, a GCS box
top-left, a HUD box top-right, and a bottom telemetry strip). So the bands here
are OPAQUE and tall enough to cover those, leaving only the central flight in
view. The middle stays transparent so the real Gazebo flight shows through.

Usage:
  gen-achievements-card.py OUT.png "Title" "subtitle" "fact1" ["fact2"] ["fact3"]

Needs Pillow (use the setup-video-env.sh venv). The bench ffmpeg has no
drawtext, so we render text to a PNG and overlay it.
"""
import sys
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FONTS = ["/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc"]
FONTS_B = ["/System/Library/Fonts/Supplemental/Arial Bold.ttf"] + FONTS
BG = (8, 12, 20, 145)          # semi-transparent — the burned-in HUD is cropped
                               # out of the sources, so the flight shows through


def font(sz, bold=False):
    for p in (FONTS_B if bold else FONTS):
        try:
            return ImageFont.truetype(p, sz)
        except OSError:
            pass
    return ImageFont.load_default()


def main():
    out, title = sys.argv[1], sys.argv[2]
    subtitle = sys.argv[3] if len(sys.argv) > 3 else ""
    facts = sys.argv[4:7]
    img = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(img)

    # ── top band: opaque, 132 px — covers the relvids' own banner + the top of
    #    their GCS / HUD boxes, so we don't double up. A thin accent rule below.
    TOP = 132
    d.rectangle([0, 0, W, TOP], fill=BG)
    d.rectangle([0, TOP, W, TOP + 3], fill=(90, 150, 230, 230))
    d.text((30, 22), f"FALCON   {title}", font=font(34, True), fill=(255, 255, 255, 255))
    if subtitle:
        d.text((32, 74), subtitle, font=font(22), fill=(150, 200, 255, 255))

    # ── bottom band: opaque, sized to the fact lines — covers the relvids'
    #    bottom telemetry strip + their own lower-third.
    fh = 38
    y0 = H - (52 + fh * len(facts))
    d.rectangle([0, y0 - 3, W, y0], fill=(90, 150, 230, 230))
    d.rectangle([0, y0, W, H], fill=BG)
    colors = [(170, 255, 170, 255), (235, 235, 235, 255), (255, 210, 140, 255)]
    for i, fact in enumerate(facts):
        d.text((30, y0 + 18 + fh * i), fact,
               font=font(23, i == 0), fill=colors[min(i, 2)])

    img.save(out)
    print(out)


if __name__ == "__main__":
    main()
