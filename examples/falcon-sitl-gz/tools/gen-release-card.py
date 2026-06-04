#!/usr/bin/env python3
"""Render a per-release info-overlay card (1280x720, transparent middle) for a
falcon release video — a top banner (FALCON + version-title + cascade subtitle)
and a lower-third with the release's feature facts. Composited onto the flight
footage by make-release-video.sh.

Usage:
  gen-release-card.py OUT.png "v1.16 — Wind rejection" \
      "IEKF → geometric SE(3) → ADRC → mixer   ·   no_std / no_alloc" \
      "fact line 1" ["fact line 2"] ["fact line 3"]

Needs Pillow (e.g. /tmp/plotenv/bin/python). The bench's ffmpeg lacks drawtext,
so we render text to a PNG and overlay it.
"""
import sys
from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FONTS = ["/System/Library/Fonts/Supplemental/Arial.ttf", "/System/Library/Fonts/Helvetica.ttc"]
FONTS_B = ["/System/Library/Fonts/Supplemental/Arial Bold.ttf"] + FONTS


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
    # top banner
    d.rectangle([0, 0, W, 86], fill=(8, 12, 20, 190))
    d.text((28, 15), f"FALCON  {title}", font=font(32, True), fill=(255, 255, 255, 255))
    if subtitle:
        d.text((30, 56), subtitle, font=font(20), fill=(150, 200, 255, 255))
    # lower-third — up to 3 fact lines, first bold-green (the headline result)
    y0 = H - (40 + 34 * len(facts))
    d.rectangle([0, y0, W, H], fill=(8, 12, 20, 190))
    colors = [(170, 255, 170, 255), (230, 230, 230, 255), (255, 210, 140, 255)]
    for i, fact in enumerate(facts):
        d.text((28, y0 + 14 + 34 * i), fact, font=font(21, i == 0), fill=colors[min(i, 2)])
    img.save(out)
    print(out)


if __name__ == "__main__":
    main()
