#!/usr/bin/env python3
"""
Does scaling the frame to the detail a code needs still read the code.

The decoder is handed frames shrunk to about four pixels a QR module rather
than whatever the camera happened to have, because on the 0.50 m aisle it had
twelve and decoding all of them took 118 ms of a 157 ms frame interval. The
saving is only worth having if nothing is lost, and "nothing is lost" is a
claim about real pixels, so this renders the real label textures at the size
and bearing each camera sees them at and puts both the full frame and the
scaled one through the scanner's own decode_qr.

No simulator and no flight. Run it after changing the label size, the camera
resolution, the target detail, or the lane split.
"""
import glob
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scanner as s
import standoff_sweep as sweep

# The faces, the camera that reads each, and how far that camera sits from it
# on the lanes the route actually flies.
FACES = [
    ("A", "hires", 1024, 768, s.CAMERA_HFOV_DEG, 1.240),
    ("B", "rear", 1280, 800, s.TRACKING_HFOV_DEG, 1.045),
    ("C", "hires", 1024, 768, s.CAMERA_HFOV_DEG, 0.899),
    ("D", "rear", 1280, 800, s.TRACKING_HFOV_DEG, 0.756),
    ("E", "hires", 1024, 768, s.CAMERA_HFOV_DEG, 0.552),
    ("F", "rear", 1280, 800, s.TRACKING_HFOV_DEG, 0.463),
    ("G", "hires", 1024, 768, s.CAMERA_HFOV_DEG, 0.211),
    ("H", "rear", 1280, 800, s.TRACKING_HFOV_DEG, 0.174),
]
# A code sweeps across the frame as the vehicle passes it, so it has to be
# legible off axis as well as on it.
BEARINGS = (0, 15, 25)
LABELS = 8

failures = []


def check(name, ok, detail=""):
    print("  %-58s %s %s" % (name, "ok" if ok else "FAILED", detail))
    if not ok:
        failures.append(name)


class Fake:
    """Just enough of a CameraDecoder for decode_qr."""

    def __init__(self, hfov_deg, depth):
        self.hfov_deg = hfov_deg
        self.depth = depth
        self.name = "test"
        self.detector = cv2.wechat_qrcode_WeChatQRCode()
        self.fallback = cv2.QRCodeDetector()


print("the rule only ever gives detail away, never asks for more")
for name, cam, width, height, hfov, depth in FACES:
    scale = s.decode_scale(depth, hfov, width)
    check("face %s scale is at or below 1.0" % name, scale <= 1.0,
          "(%.3f)" % scale)

print()
print("and it never goes below the detail the detector needs")
modules = s.CODE_SIZE_M / s.CODE_MODULE_SIZE_M
MEASURED_FLOOR = 1.66
for name, cam, width, height, hfov, depth in FACES:
    view = 2 * depth * math.tan(math.radians(hfov) / 2)
    ppm = s.CODE_SIZE_M / view * width / modules
    scale = s.decode_scale(depth, hfov, width)
    after = ppm * scale
    check("face %s keeps %.2f px per module" % (name, after),
          after >= min(ppm, MEASURED_FLOOR),
          "(had %.2f, floor %.2f)" % (ppm, MEASURED_FLOOR))

print()
print("real labels, real decoder: what reads at full size must still read")
# The same textures standoff_sweep measures with, so the two agree on what a
# label looks like.
labels = sorted(glob.glob(os.path.join(sweep.texture_dir(), "box_*.png")))
if not labels:
    raise SystemExit("no QR label textures found in %s" % sweep.texture_dir())
labels = labels[:LABELS]
label_m = (0.100, 0.095)

print("  %-6s %8s %7s %9s %9s %10s %10s"
      % ("face", "depth", "scale", "full", "scaled", "full ms", "scaled ms"))
for name, cam, width, height, hfov, depth in FACES:
    scale = s.decode_scale(depth, hfov, width)
    fake = Fake(hfov, depth)
    full_hits = scaled_hits = 0
    full_ms = scaled_ms = 0.0
    trials = 0
    for path in labels:
        label = cv2.imread(path, cv2.IMREAD_COLOR)
        if label is None:
            continue
        for bearing in BEARINGS:
            frame = sweep.render(label, width, height, hfov, depth, bearing,
                                 label_m)
            trials += 1

            t = time.perf_counter()
            got = s.decode_qr(frame, fake)
            full_ms += (time.perf_counter() - t) * 1e3
            full_read = any(v for v, *_ in got)
            full_hits += bool(full_read)

            small = frame if scale >= 1.0 else cv2.resize(
                frame, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
            t = time.perf_counter()
            got = s.decode_qr(small, fake)
            scaled_ms += (time.perf_counter() - t) * 1e3
            scaled_read = any(v for v, *_ in got)
            scaled_hits += bool(scaled_read)

    print("  %-6s %8.3f %7.3f %5d/%-3d %5d/%-3d %10.1f %10.1f"
          % (name, depth, scale, full_hits, trials, scaled_hits, trials,
             full_ms / max(trials, 1), scaled_ms / max(trials, 1)))
    check("face %s loses nothing to scaling" % name, scaled_hits >= full_hits,
          "(full %d, scaled %d, of %d)" % (full_hits, scaled_hits, trials))

print()
if failures:
    print("%d FAILED: %s" % (len(failures), ", ".join(failures)))
    sys.exit(1)
print("all checks passed")
