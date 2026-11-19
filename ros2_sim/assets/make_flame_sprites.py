#!/usr/bin/env python3
"""Build the flame billboard textures from real, freely licensed fire photos (see CREDITS.md).

    python3 make_flame_sprites.py            # downloads the sources (Wikimedia Commons), writes flames/*.png

The photos show flames on a dark background, so alpha is keyed from brightness (dark = transparent), the borders and
the bottom are feathered, and each sprite is cropped to a tall flame. Also writes glow.png (radial halo).
Needs numpy + Pillow + network (only to rebuild; the generated PNGs are committed).
"""
import io
import os
import urllib.parse
import urllib.request

import numpy as np
from PIL import Image, ImageFilter

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "flames")
UA = {"User-Agent": "ai-fire-sim-assets/0.1 (https://github.com/mrsssok2/ai_fire)"}

# (output name, Commons file, crop box as fractions of the 1200 px wide image: x0, y0, x1, y1, alpha key low/high)
SOURCES = [
    ("flame_1", "Campfires_flames.jpg", (0.38, 0.00, 0.80, 0.72), (0.12, 0.60)),
    ("flame_2", "Fire_fire_flames.jpg", (0.02, 0.00, 0.50, 0.40), (0.40, 0.85)),
    ("flame_3", "Campfire_Flames.jpg", (0.15, 0.00, 0.75, 0.80), (0.12, 0.60)),
    ("flame_4", "Fire_fire_flames.jpg", (0.55, 0.00, 1.00, 0.40), (0.40, 0.85)),
]


def fetch(name):
    url = "https://commons.wikimedia.org/wiki/Special:FilePath/" + urllib.parse.quote(name) + "?width=1200"
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=90) as r:
        return Image.open(io.BytesIO(r.read())).convert("RGB")


def smooth(x, a, b):
    t = np.clip((x - a) / (b - a), 0, 1)
    return t * t * (3 - 2 * t)


def make_sprite(img, box, key, size=(256, 384)):
    w, h = img.size
    img = img.crop((int(box[0] * w), int(box[1] * h), int(box[2] * w), int(box[3] * h)))
    img = img.resize(size, Image.LANCZOS)
    rgb = np.asarray(img).astype(np.float32) / 255.0
    value = rgb.max(axis=2)
    alpha = smooth(value, *key)
    yy, xx = np.mgrid[0:size[1], 0:size[0]]
    feather = smooth(xx / size[0], 0.0, 0.12) * smooth(1 - xx / size[0], 0.0, 0.12) * smooth(yy / size[1], 0.0, 0.05)
    feather *= smooth(1 - yy / size[1], 0.0, 0.10)          # fade out at the bottom so the base blends into the logs
    alpha = alpha * feather
    rgb = np.clip(rgb * 1.35, 0, 1)                          # compensate the dark fringes of straight-alpha blending
    out = np.dstack([rgb, alpha])
    return Image.fromarray((out * 255).astype(np.uint8), "RGBA").filter(ImageFilter.GaussianBlur(0.6))


def make_glow(size=256):
    yy, xx = np.mgrid[0:size, 0:size]
    r = np.hypot(xx - size / 2, yy - size / 2) / (size / 2)
    a = np.clip(1 - r, 0, 1) ** 2.2 * 0.85
    rgb = np.dstack([np.ones_like(a), 0.45 + 0.25 * (1 - r).clip(0, 1), 0.10 * np.ones_like(a)])
    return Image.fromarray((np.dstack([rgb, a]) * 255).astype(np.uint8), "RGBA")


if __name__ == "__main__":
    os.makedirs(OUT, exist_ok=True)
    for name, src, box, key in SOURCES:
        make_sprite(fetch(src), box, key).save(os.path.join(OUT, name + ".png"))
        print("wrote", name, "from", src)
    make_glow().save(os.path.join(OUT, "glow.png"))
