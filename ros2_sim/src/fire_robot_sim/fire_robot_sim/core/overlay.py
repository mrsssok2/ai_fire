"""Drawing code for the live "Fire Detection" window (numpy + OpenCV, no ROS): camera frame with the status banner and
boxes on the left, a status panel on the right."""
from __future__ import annotations

import math

import cv2
import numpy as np

RED, GREEN, GREY, WHITE, BLUE, DARK, AMBER = (40, 40, 220), (60, 190, 60), (110, 110, 110), (240, 240, 240), (230, 140, 40), (35, 35, 38), (30, 170, 230)
FONT = cv2.FONT_HERSHEY_SIMPLEX


def banner(frame, conf, threshold=0.5):
    """Big status bar across the top of `frame` (BGR, modified in place)."""
    fire = conf >= threshold
    h = 54
    cv2.rectangle(frame, (0, 0), (frame.shape[1], h), RED if fire else GREEN, -1)
    text = f"FIRE DETECTED {conf * 100:.0f}%" if fire else f"NO FIRE {conf * 100:.0f}%"
    cv2.putText(frame, text, (14, 38), FONT, 1.15, WHITE, 3, cv2.LINE_AA)
    return fire


def boxes(frame, dets, scale):
    for d in dets:
        x, y, w, h = (int(v * scale) for v in (d["x"], d["y"], d["w"], d["h"]))
        cv2.rectangle(frame, (x, y), (x + w, y + h), RED, 2)
        cv2.putText(frame, f"fire {d['conf'] * 100:.0f}%", (x, max(70, y - 6)), FONT, 0.55, RED, 2, cv2.LINE_AA)


def panel(width, height, s):
    """s: dict with flame{left,front,right}, analog{...}, gas, ppm, water, ml, pump, servo, alert (text) -> BGR image."""
    p = np.full((height, width, 3), DARK, np.uint8)
    y = 34
    cv2.putText(p, "SENSORS", (16, y), FONT, 0.7, AMBER, 2, cv2.LINE_AA)
    y += 52
    for i, (key, label) in enumerate((("left", "L"), ("front", "F"), ("right", "R"))):
        cx = 52 + i * 90
        on = s["flame"].get(key, False)
        cv2.circle(p, (cx, y), 24, RED if on else (60, 60, 66), -1)
        cv2.circle(p, (cx, y), 24, WHITE, 2)
        cv2.putText(p, label, (cx - 9, y + 9), FONT, 0.8, WHITE, 2, cv2.LINE_AA)
        cv2.putText(p, f"{s['analog'].get(key, 0.0):.1f}", (cx - 18, y + 46), FONT, 0.5, GREY if not on else WHITE, 1, cv2.LINE_AA)
    cv2.putText(p, "flame sensors (signal, 1.0 = trips)", (16, y + 68), FONT, 0.45, GREY, 1, cv2.LINE_AA)
    y += 102
    cv2.putText(p, f"GAS  {s['ppm']:.0f} ppm", (16, y), FONT, 0.65, WHITE, 2, cv2.LINE_AA)
    cv2.rectangle(p, (16, y + 10), (width - 16, y + 28), (60, 60, 66), -1)
    cv2.rectangle(p, (16, y + 10), (16 + int((width - 32) * min(1.0, s["ppm"] / 1000.0)), y + 28), RED if s["gas"] else GREEN, -1)
    cv2.putText(p, "GAS DETECTED" if s["gas"] else "no gas", (16, y + 50), FONT, 0.55, RED if s["gas"] else GREY, 1, cv2.LINE_AA)
    y += 84
    cv2.putText(p, f"WATER TANK  {s['water']:.0f}%  ({s['ml']:.0f} ml)", (16, y), FONT, 0.6, WHITE, 2, cv2.LINE_AA)
    cv2.rectangle(p, (16, y + 10), (width - 16, y + 34), (60, 60, 66), -1)
    cv2.rectangle(p, (16, y + 10), (16 + int((width - 32) * s["water"] / 100.0), y + 34), RED if s["water"] < 15 else BLUE, -1)
    cv2.rectangle(p, (16, y + 10), (width - 16, y + 34), WHITE, 1)
    y += 70
    cv2.putText(p, "PUMP", (16, y), FONT, 0.65, WHITE, 2, cv2.LINE_AA)
    cv2.rectangle(p, (100, y - 24), (190, y + 6), BLUE if s["pump"] else (60, 60, 66), -1)
    cv2.putText(p, "ON" if s["pump"] else "OFF", (123, y), FONT, 0.7, WHITE, 2, cv2.LINE_AA)
    y += 52
    cv2.putText(p, f"SERVO  {s['servo']:.0f} deg", (16, y), FONT, 0.65, WHITE, 2, cv2.LINE_AA)
    cx, cy = width - 70, y + 6                                  # little gauge: 0..180 deg, 90 = straight ahead (up)
    cv2.ellipse(p, (cx, cy), (36, 36), 0, 180, 360, GREY, 2)
    a = math.radians(s["servo"] - 90)
    cv2.line(p, (cx, cy), (int(cx - 34 * math.sin(a)), int(cy - 34 * math.cos(a))), AMBER, 3)
    y += 56
    cv2.putText(p, "LAST ALERT", (16, y), FONT, 0.6, AMBER, 2, cv2.LINE_AA)
    words, line, yy = s["alert"].split(), "", y + 26
    for w in words:
        if len(line) + len(w) + 1 > 30:
            cv2.putText(p, line, (16, yy), FONT, 0.55, WHITE, 1, cv2.LINE_AA)
            line, yy = w, yy + 22
        else:
            line = (line + " " + w).strip()
    cv2.putText(p, line, (16, yy), FONT, 0.55, WHITE, 1, cv2.LINE_AA)
    return p


def compose(frame_rgb, conf, dets, status, threshold=0.5, scale=2):
    """Full window image (BGR): scaled camera frame + banner + boxes, status panel on the right."""
    cam = cv2.resize(cv2.cvtColor(frame_rgb, cv2.COLOR_RGB2BGR), None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
    boxes(cam, dets, scale)
    banner(cam, conf, threshold)
    return np.hstack([cam, panel(320, cam.shape[0], status)])
