"""
Choose the flight lane by measuring where each camera stops reading.

The vehicle flies one lane per aisle and reads both faces at once, the forward
hires camera ahead and the rear tracking camera behind. Those two distances
have to add up to the aisle width, so giving one camera more takes it from the
other, and the split is worth measuring rather than guessing.

This renders real label textures at the size and obliquity a camera sees them
at a given distance and bearing, on a grey background like the warehouse
renders, and puts them through the scanner's own decode_qr. No simulator and
no flight.

Read the result by asking for reliability out to about 25 degrees off axis
rather than at the extreme edge of the frame: a code sweeps from one side of
the frame to the other as the vehicle passes it, so it only has to be legible
somewhere in the middle of that sweep.

Run this after changing warehouse, since the aisle width sets what is on
offer and the label size sets what each camera needs.
"""
import glob
import math
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scanner as s

BEARINGS = (0, 15, 25, 35)
LABELS = 12

CAMERAS = [
    # name, width, height, hfov, distances to try
    ("hires, forward", 1024, 768, s.CAMERA_HFOV_DEG,
     (1.10, 1.20, 1.30, 1.40, 1.50, 1.60)),
    ("tracking, rear", 1280, 800, s.TRACKING_HFOV_DEG,
     (0.80, 0.90, 1.00, 1.10, 1.20, 1.30)),
]


def texture_dir():
    """
    Where the generated label textures ended up.

    Three places, in order: a directory the layout names, the generator's own
    output tree before it is installed, and the PX4 tree that setup_px4.sh
    copies it into. The last one used to be a path under one developer's home
    directory, which meant these tests only ran on that machine.  PX4_DIR is
    the same override setup_px4.sh and launch_sim.sh take.
    """
    here = os.path.dirname(os.path.abspath(s.LAYOUT_PATH))
    px4 = os.environ.get("PX4_DIR") or os.path.expanduser("~/PX4-Autopilot")
    candidates = []
    if s.LAYOUT.get("texture_dir"):
        candidates.append(os.path.join(here, s.LAYOUT["texture_dir"]))
    candidates += [
        os.path.join(here, "..", "warehouse", "generated", "gz", "models",
                     "warehouse_assets", "materials", "textures"),
        os.path.join(px4, "Tools", "simulation", "gz", "models",
                     "warehouse_assets", "materials", "textures"),
    ]
    for candidate in candidates:
        if os.path.isdir(candidate):
            return os.path.normpath(candidate)
    raise SystemExit("no label textures; run setup_px4.sh first")


def render(label, width, height, hfov_deg, dist, bearing_deg, label_m,
           background=160):
    """
    One label as that camera sees it, at that distance and that bearing.

    Two things change with bearing and both are applied: the range grows as
    one over the cosine, and the face turns away so the label narrows by the
    same cosine.
    """
    bearing = math.radians(bearing_deg)
    slant = dist / math.cos(bearing)
    scale = width / (2 * slant * math.tan(math.radians(hfov_deg) / 2))
    w = max(2, int(round(label_m[0] * scale * math.cos(bearing))))
    h = max(2, int(round(label_m[1] * scale)))
    small = cv2.resize(label, (w, h), interpolation=cv2.INTER_AREA)

    frame = np.full((height, width, 3), background, dtype=np.uint8)
    # Put it where that bearing actually falls in the frame.
    centre = int(width / 2 + (math.tan(bearing) /
                              math.tan(math.radians(hfov_deg) / 2)) * width / 2)
    x0 = max(0, min(width - w, centre - w // 2))
    y0 = (height - h) // 2
    frame[y0:y0 + h, x0:x0 + w] = small
    return frame


def px_per_module(width, hfov_deg, dist, label_m, module_m):
    """Modules of the code per pixel of frame, on axis."""
    return width / (2 * dist * math.tan(math.radians(hfov_deg) / 2)) * module_m


def main():
    truth = os.path.join(os.path.dirname(s.LAYOUT_PATH),
                         "../warehouse/ground_truth.json")
    label_m, module_m = (0.10, 0.15), 0.0028
    if os.path.exists(truth):
        import json
        for code in json.load(open(truth, encoding="utf-8"))["codes"]:
            if code.get("type") == "box_qr":
                label_m = tuple(code["label_size_m"])
                module_m = code["module_size_m"]
                break

    paths = sorted(glob.glob(os.path.join(texture_dir(), "box_*.png")))[:LABELS]
    if not paths:
        raise SystemExit("no box label textures")
    images = [cv2.imread(p) for p in paths]

    print("%d labels, %.0f by %.0f mm, %.2f mm per module"
          % (len(images), label_m[0] * 1000, label_m[1] * 1000,
             module_m * 1000))
    print("decoded through the scanner's own decode_qr\n")

    probe = s.CameraDecoder("sweep", s.CAMERA_HFOV_DEG, 0.0)

    for name, width, height, hfov, distances in CAMERAS:
        print("%s   %dx%d, %.0f degrees" % (name, width, height, hfov))
        print("  %-7s %-11s %s" % ("dist", "px/module",
                                   "  ".join("%5d deg" % b for b in BEARINGS)))
        for dist in distances:
            cells = []
            for bearing in BEARINGS:
                read = sum(
                    1 for img in images
                    if s.decode_qr(render(img, width, height, hfov, dist,
                                          bearing, label_m), probe))
                cells.append("%4d/%-4d" % (read, len(images)))
            print("  %-7.2f %-11.2f %s"
                  % (dist, px_per_module(width, hfov, dist, label_m, module_m),
                     "  ".join(cells)))
        print()

    print("The two distances must add to the aisle width. Pick the largest")
    print("that still reads at 25 degrees for each, and check they fit.")


if __name__ == "__main__":
    main()
