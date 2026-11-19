"""Small image helpers (numpy + OpenCV): fire-coloured region proposals (HSV) and the CNN input preparation."""
from __future__ import annotations

import cv2
import numpy as np


def fire_regions(rgb: np.ndarray, min_area: int = 40, max_regions: int = 3, pad: float = 0.15):
    """Bright, saturated red-orange-yellow blobs -> boxes (x, y, w, h). A *proposal* step (HSV colour rule, like the
    classic hsv fire detectors); a classifier decides whether a proposal really is fire."""
    hsv = cv2.cvtColor(rgb, cv2.COLOR_RGB2HSV)
    h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
    mask = ((h <= 32) | (h >= 172)) & (s >= 90) & (v >= 170)
    mask = cv2.morphologyEx(mask.astype(np.uint8) * 255, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask)
    boxes = [tuple(int(x) for x in stats[i, :4]) + (int(stats[i, 4]),) for i in range(1, n) if stats[i, 4] >= min_area]
    boxes.sort(key=lambda b: -b[4])
    out = []
    H, W = rgb.shape[:2]
    for x, y, w, hh, _ in boxes[:max_regions]:
        px, py = int(w * pad), int(hh * pad)
        x0, y0, x1, y1 = max(0, x - px), max(0, y - py), min(W, x + w + px), min(H, y + hh + py)
        out.append((x0, y0, x1 - x0, y1 - y0))
    return out


def cnn_input(rgb: np.ndarray, mode: str = "squash", size: int = 224) -> np.ndarray:
    """Prepare a frame for the user's Keras CNN: RGB, resized to 224x224 ('squash' = what Keras' flow_from_directory did in
    training, 'fit' = centre crop like Teachable Machine's ImageOps.fit), scaled to 0..1. Returns float32 [1,224,224,3]."""
    h, w = rgb.shape[:2]
    if mode == "fit":
        s = min(h, w)
        y0, x0 = (h - s) // 2, (w - s) // 2
        rgb = rgb[y0:y0 + s, x0:x0 + s]
    img = cv2.resize(rgb, (size, size), interpolation=cv2.INTER_LINEAR)
    return (img.astype(np.float32) / 255.0)[None]
