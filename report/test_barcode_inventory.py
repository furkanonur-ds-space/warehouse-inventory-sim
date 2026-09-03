#!/usr/bin/env python3
"""
Check that a barcode reading lands on the box it was read from.

    .venv/bin/python report/test_barcode_inventory.py

No simulator and no flight. A box is taken from ground truth, its barcode is
projected into the frame the camera would have seen it in, and that synthetic
reading is put through the same place() a real one goes through. The answer
has to come back where the box is.

This is worth having because the geometry is the scanner's, rewritten against
a different pose convention: the scanner is given MAVSDK's yaw, measured from
north, and this is given Gazebo's, measured from +X. A sign error there puts
every code on the wrong side of the aisle and nothing else notices - the shelf
snap would hide it on the faces and only the along-aisle position would move,
which is exactly the error that is hardest to see in a finished run.

Both cameras are checked, because the rear one is read with its heading turned
through 180 degrees, and the widest and the narrowest aisle, because the
standoff and the frame both change with the width.
"""
from __future__ import annotations

import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import barcode_inventory as bi                                   # noqa: E402
from warehouse_model import GROUND_TRUTH, LAYOUT, cameras        # noqa: E402

TOLERANCE_M = 0.005


def load():
    layout = json.loads(open(LAYOUT).read())
    codes = json.load(open(GROUND_TRUTH, encoding="utf-8"))["codes"]
    faces = {f["name"]: f for f in layout["aisle_faces"] if "name" in f}
    geometry = {"cameras": cameras(), "faces": faces,
                "code_plane": layout.get("code_plane_offset_m", 0.0),
                "flight_z": layout["flight_z"]}
    return layout, codes, geometry


def synthesise(codes, geometry, layout, row, bay, level, which, lane_x, index=0):
    """The reading a camera on that lane would have written for that box."""
    spec = geometry["cameras"][which]
    width, height = spec["frame_px"]
    h_fov = math.radians(spec["hfov_deg"])
    v_fov = 2 * math.atan(math.tan(h_fov / 2) * height / width)
    face = geometry["faces"][row]

    def of(kind):
        return sorted([c for c in codes
                       if c.get("type") == kind and c["row"] == row
                       and int(c["bay"]) == bay and int(c["level"]) == level],
                      key=lambda c: c["label_pose_xyzrpy"][1])

    bar, qr = of("box_placard")[index], of("box_qr")[index]
    bx, by, bz = bar["label_pose_xyzrpy"][:3]

    # Looking across the aisle at that face, from the lane, a little behind
    # the box so the code is off axis rather than dead ahead.
    psi = math.pi if face["face_x"] < lane_x else 0.0
    right_x, right_y = math.sin(psi), -math.cos(psi)
    ux, uy, uz = lane_x, by - 0.9, layout["flight_z"][level - 1]

    depth = abs(face["face_x"] - ux) - spec["mount_x"] + geometry["code_plane"]
    lateral = (by - uy) * right_y + (bx - ux) * right_x
    bearing = math.atan2(lateral, depth)
    elevation = math.atan2(-(bz - uz), depth)

    reading = {
        "symbology": "CODE128",
        "payload": bar["payload"],
        "centre": [width / 2 + math.tan(bearing) / math.tan(h_fov / 2) * width / 2,
                   height / 2 + math.tan(elevation) / math.tan(v_fov / 2) * height / 2],
        "frame_px": [width, height],
        "uav": {"x": ux, "y": uy, "z": uz},
        # The rear camera is mounted backwards, so the vehicle's own heading is
        # the one this camera's view is turned 180 degrees from.
        "uav_yaw_deg": math.degrees(psi) - (180.0 if which == "rear" else 0.0),
        "qr_drop_m": qr["label_pose_xyzrpy"][2] - bz,
        "camera_link": ("camera_track_rear_link" if which == "rear"
                        else "camera_hires_link"),
    }
    return reading, qr


def main() -> int:
    layout, codes, geometry = load()
    cases = [("A", "hires", -8.40), ("B", "rear", -8.40),
             ("C", "hires", -4.14), ("D", "rear", -4.14),
             ("E", "hires", -0.52), ("F", "rear", -0.52),
             ("G", "hires", 2.47), ("H", "rear", 2.47)]

    failures = 0
    checked = 0
    worst = 0.0
    print("%-5s %-6s %-6s %8s %8s %8s  %s"
          % ("face", "camera", "level", "dx", "dy", "dz", "filed"))
    for row, which, lane_x in cases:
        for bay, level in ((2, 1), (4, 2), (6, 3)):
            for index in (0, 1, 2):
                try:
                    reading, qr = synthesise(codes, geometry, layout,
                                             row, bay, level, which, lane_x,
                                             index)
                except IndexError:
                    continue
                spot = bi.place(reading, geometry)
                checked += 1
                if spot is None:
                    print("%-5s %-6s %-6d  not placed at all" % (row, which, level))
                    failures += 1
                    continue
                tx, ty, tz = qr["label_pose_xyzrpy"][:3]
                dx, dy, dz = spot["x"] - tx, spot["y"] - ty, spot["z"] - tz
                worst = max(worst, abs(dx), abs(dy), abs(dz))
                bad = (max(abs(dx), abs(dy), abs(dz)) > TOLERANCE_M
                       or spot["shelf"] != row or spot["level"] != level)
                if bad:
                    failures += 1
                    print("%-5s %-6s %-6d %+8.3f %+8.3f %+8.3f  %s %d  !! FAILED"
                          % (row, which, level, dx, dy, dz,
                             spot["shelf"], spot["level"]))
                elif index == 0 and bay == 2:
                    print("%-5s %-6s %-6d %+8.3f %+8.3f %+8.3f  %s %d"
                          % (row, which, level, dx, dy, dz,
                             spot["shelf"], spot["level"]))

    print()
    print("%d readings placed, worst axis error %.4f m, tolerance %.3f m"
          % (checked, worst, TOLERANCE_M))
    if failures:
        print("%d FAILED" % failures)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
