"""
Check that reading in strips finds more codes and puts them in the same place.

Two things can go wrong with this, and they fail differently. The detector may
still miss codes, which shows up as a low count and costs coverage. Or the
strip coordinates may not be mapped back to the frame, which shows up as
nothing at all here and as every box on the wrong part of the shelf in flight,
because the bearing to a box comes from where its code sits in the frame.

So this measures both: how many of the codes placed in a frame come back, and
how far each one is reported from where it was actually put.
"""
import glob
import math
import os
import random
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scanner as s

LABEL_M = (0.10, 0.15)
TRIALS = 12
failures = []


def texture_dir():
    for candidate in (
            os.path.join(os.path.dirname(s.LAYOUT_PATH),
                         "../warehouse/generated/out/textures"),
            "/home/furk/PX4-Autopilot/Tools/simulation/gz/models/"
            "warehouse_assets/materials/textures"):
        if os.path.isdir(candidate):
            return candidate
    raise SystemExit("no label textures")


labels = [cv2.imread(p) for p in
          sorted(glob.glob(os.path.join(texture_dir(), "box_*.png")))[:12]]
random.seed(3)


def build(width, height, hfov_deg, dist, n_codes):
    """n codes at known positions, returned so they can be checked."""
    scale = width / (2 * dist * math.tan(math.radians(hfov_deg) / 2))
    w = max(4, int(LABEL_M[0] * scale))
    h = max(4, int(LABEL_M[1] * scale))
    img = np.full((height, width, 3), 160, dtype=np.uint8)
    placed = []
    tries = 0
    while len(placed) < n_codes and tries < 300:
        tries += 1
        x0 = random.randint(0, width - w)
        y0 = random.randint(0, height - h)
        if any(abs(x0 - px) < w + 12 and abs(y0 - py) < h + 12
               for px, py, _ in placed):
            continue
        img[y0:y0 + h, x0:x0 + w] = cv2.resize(
            labels[len(placed) % len(labels)], (w, h),
            interpolation=cv2.INTER_AREA)
        placed.append((x0 + w / 2.0, y0 + h / 2.0, len(placed)))
    return img, placed


def check(label, ok, detail=""):
    print("  %-52s %s" % (label, "ok" if ok else "FAILED " + detail))
    if not ok:
        failures.append(label)


cam = s.CameraDecoder("strips", 60.0, 0.0)

for name, width, height, hfov, dist in [
        ("hires 1024x768 at 1.24 m", 1024, 768, 60.0, 1.24),
        ("rear 1280x800 at 1.045 m", 1280, 800, 90.0, 1.045)]:
    print("\n%s" % name)
    for n in (3, 4):
        found_total = 0
        worst_offset = 0.0
        for _ in range(TRIALS):
            img, placed = build(width, height, hfov, dist, n)
            hits = s.decode_qr(img, cam)
            found_total += len(hits)
            # Every reported position should sit on top of a label that is
            # really there. Anything else means the strip offset is wrong.
            for _value, cx, cy, _w, _h in hits:
                nearest = min(placed,
                              key=lambda p: (p[0] - cx) ** 2 + (p[1] - cy) ** 2)
                offset = math.hypot(nearest[0] - cx, nearest[1] - cy)
                worst_offset = max(worst_offset, offset)
        mean = found_total / TRIALS
        check("%d codes in frame, finds %.2f of them" % (n, mean),
              mean >= n * 0.7, "(wanted at least %.1f)" % (n * 0.7))
        check("  reported within 20 px of the real label",
              worst_offset < 20, "(worst %.0f px)" % worst_offset)

print("\nno duplicates survive the overlap between strips")
img, placed = build(1024, 768, 60.0, 1.24, 4)
hits = s.decode_qr(img, cam)
values = [h[0] for h in hits]
check("every code reported once", len(values) == len(set(values)),
      "(%d hits, %d distinct)" % (len(values), len(set(values))))

print("\n%s" % ("all checks passed" if not failures
                else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
sys.exit(1 if failures else 0)
