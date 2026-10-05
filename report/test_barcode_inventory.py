#!/usr/bin/env python3
"""
Check that a barcode reading lands on the label it was read from.

    .venv/bin/python report/test_barcode_inventory.py

No simulator and no flight. A box is taken from ground truth, its barcode is
projected into the frame the camera would have seen it in, and that synthetic
reading is put through the same place() a real one goes through. The answer
has to come back where that barcode label is - not where the QR above it is,
since the two are separate labels on the same box and each is scored against
its own truth.

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

import contextlib
import io
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import barcode_inventory as bi                                   # noqa: E402
from warehouse_model import CONFIG, LAYOUT, REPO_ROOT, cameras   # noqa: E402

sys.path.insert(0, str(REPO_ROOT / "warehouse"))
import gen_labels as gl                                          # noqa: E402
import gen_world as gw                                           # noqa: E402

TOLERANCE_M = 0.005

# Where each lens sits above the pose the readers record, read off the model
# BY HAND rather than through cameras(): starling2_base's <pose>0 0 0.06</pose>
# puts base_link 0.06 m above the model origin, and build_c27_drone.py mounts
# the hires at z 0.0 and the rear at -0.015 from base_link. The synthetic
# readings below take lens_z from cameras() as place() does, so on their own
# they would agree with a place() that got it wrong - which is how a 0.06 m
# height error went unseen until the 2026-10-05 flight measured it.
EXPECTED_LENS_Z = {"hires": 0.060, "rear": 0.045}


def load():
    layout = json.loads(open(LAYOUT).read())
    # The CLEAN world, built here, not the installed ground truth: when the
    # stressed world (warehouse/stress.py) is installed its labels are pushed
    # in and turned on purpose, and this checks the geometry, not what a
    # disturbance does to it.
    cfg = gl._load_cfg(CONFIG)
    cfg.setdefault("stress", {})["enabled"] = False
    with contextlib.redirect_stdout(io.StringIO()):
        _, codes, _ = gw.build(cfg)
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
    # Seen from the lens, which rides lens_z above the pose that is recorded.
    # Synthesised from the pose itself, as this test once was, it agreed with
    # a place() that made the same mistake and hid a 0.06 m height error.
    elevation = math.atan2(-(bz - (uz + spec["lens_z"])), depth)

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
        # Recorded by the reader and carried through, but not used to place
        # the barcode: it is what a tool comparing the two labels needs.
        "qr_drop_m": qr["label_pose_xyzrpy"][2] - bz,
        "camera_link": ("camera_track_rear_link" if which == "rear"
                        else "camera_hires_link"),
    }
    return reading, bar


def check_lens_height(geometry) -> int:
    """Whether cameras() found the lenses where the model puts them."""
    from warehouse_model import GZ_MODELS
    model = json.loads(open(LAYOUT).read()).get("model", "")
    if not (GZ_MODELS / model / "model.sdf").exists():
        print("vehicle model not installed; lens height checked on the fallback only")
    bad = 0
    for which, want in EXPECTED_LENS_Z.items():
        got = geometry["cameras"][which]["lens_z"]
        ok = abs(got - want) < 1e-6
        bad += not ok
        print(f"lens height {which:<5} {got:.3f} m (model says {want:.3f})"
              + ("" if ok else "  !! FAILED"))
    return bad


def main() -> int:
    layout, codes, geometry = load()
    failures = check_lens_height(geometry)
    cases = [("A", "hires", -8.40), ("B", "rear", -8.40),
             ("C", "hires", -4.14), ("D", "rear", -4.14),
             ("E", "hires", -0.52), ("F", "rear", -0.52),
             ("G", "hires", 2.47), ("H", "rear", 2.47)]

    checked = 0
    worst = 0.0
    print("%-5s %-6s %-6s %8s %8s %8s  %s"
          % ("face", "camera", "level", "dx", "dy", "dz", "filed"))
    for row, which, lane_x in cases:
        for bay, level in ((2, 1), (4, 2), (6, 3)):
            for index in (0, 1, 2):
                try:
                    reading, bar = synthesise(codes, geometry, layout,
                                              row, bay, level, which,
                                              lane_x, index)
                except IndexError:
                    continue
                spot = bi.place(reading, geometry)
                checked += 1
                if spot is None:
                    print("%-5s %-6s %-6d  not placed at all" % (row, which, level))
                    failures += 1
                    continue
                tx, ty, tz = bar["label_pose_xyzrpy"][:3]
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
