#!/usr/bin/env python3
"""Render an 'achievements' overlay card (1280x720, transparent middle) for the
single falcon achievements video.

The source relvids carry burned-in UI at fixed pixel offsets:
  - Top banner (title + subtitle):  y = 0 .. ~88 px
  - Bottom telemetry strip:         y = ~536 .. 720 px  (~184 px tall)
  Some clips also carry floating GCS/FSM boxes at y ~ 58-155 px (left/right sides)
  but those are outside the main flight area and are accepted as ambient context.

This card overlays two semi-transparent bands (alpha 215) that land EXACTLY on
those burned-in regions, suppressing the old text while our new text replaces it
region-for-region.  The middle of the frame (y = TOP_H .. BOT_Y, full width) is
left at alpha = 0 so the Gazebo flight fills it 100 % unobscured.  No zoom, no
crop — the source plays at native 1280 x 720.

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

# Fully opaque bands — suppresses burned-in text completely; the transparent middle
# (y = TOP_H .. BOT_Y) keeps the Gazebo flight 100 % visible
BG = (8, 12, 20, 255)
ACCENT = (90, 150, 230, 255)

# Band heights chosen to exactly cover the burned-in regions in all relvid clips
TOP_H = 90    # covers the title+subtitle header row (0..88 px)
BOT_Y = 536   # bottom strip starts here; 536..720 = 184 px of telemetry


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

    # ── top band: covers burned-in title+subtitle header (y = 0..TOP_H)
    d.rectangle([0, 0, W, TOP_H], fill=BG)
    d.rectangle([0, TOP_H, W, TOP_H + 3], fill=ACCENT)
    d.text((30, 14), f"FALCON   {title}", font=font(34, True), fill=(255, 255, 255, 255))
    if subtitle:
        d.text((32, 56), subtitle, font=font(20), fill=(150, 200, 255, 255))

    # ── bottom band: covers burned-in telemetry strip (y = BOT_Y..H)
    d.rectangle([0, BOT_Y - 3, W, BOT_Y], fill=ACCENT)
    d.rectangle([0, BOT_Y, W, H], fill=BG)
    fh = 37
    colors = [(170, 255, 170, 255), (235, 235, 235, 255), (255, 210, 140, 255)]
    for i, fact in enumerate(facts):
        d.text((30, BOT_Y + 12 + fh * i), fact,
               font=font(22, i == 0), fill=colors[min(i, 2)])

    img.save(out)
    print(out)


if __name__ == "__main__":
    main()
