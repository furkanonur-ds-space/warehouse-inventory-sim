#!/usr/bin/env python3
"""
Turn a run's barcode readings into an inventory, in the same shape the QR scan
writes.

    .venv/bin/python report/barcode_inventory.py
    # -> out/inventory_barcode.json

Why this exists. Every box carries two labels that name it, a QR and a
Code128 barcode, read by different code from different cameras. The QR scan
files a box the moment it decodes one; the barcode reader only ever wrote a
list of sightings, so the two could not be compared as inventories - only as
counts. This produces the missing half, and because it writes the same schema,
every scoring tool already takes it:

    report/validate_inventory.py --inventory out/inventory_barcode.json \
                                --code-type box_placard
    report/coverage_report.py   --inventory out/inventory_barcode.json \
                                --code-type box_placard

NOTHING IS LOOKED UP. A box carries two codes that say different things, so
neither can be derived from the other: what this files is the barcode that was
read, at the position it was read from, and it is scored against the barcode
labels in ground truth rather than against the QR ones. An earlier version
resolved each payload to the box's QR through a catalogue, which quietly
reported a QR reading that never happened - the whole point of carrying two
labels is that they are two measurements, and a run has to be able to say that
one was read and the other was not.

THE GEOMETRY is the scanner's own, applied to the barcode reader's numbers:
a bearing from where the bars sat in the frame, a known perpendicular distance
to the face the camera was reading, and the along-aisle offset that follows.
Two things differ from the QR case and both are corrected here:

  * the two cameras are not the same camera and do not fly the same distance
    from their faces, which the vehicle model and the layout between them
    already know;
  * the bars are what was read and the barcode label is what is scored, so the
    height is the bars' own. The drop to the QR above them is recorded with
    every reading and left alone here; it is what a tool comparing the two
    labels needs, not this one.

A box read many times is filed once, from the reading with the smallest
bearing: a code straight ahead is the one whose distance the perpendicular
standoff actually describes, and the ones near the frame edge are where the
approximation is worst.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from warehouse_model import (GROUND_TRUTH, LAYOUT, REPO_ROOT,   # noqa: E402
                             cameras)
from barcode_vs_qr import (camera_of, default_readings,         # noqa: E402
                           load_readings)

OUT = REPO_ROOT / "out"


def faces_by_name(path: Path = LAYOUT) -> dict:
    layout = json.loads(Path(path).read_text())
    return {f["name"]: f for f in layout["aisle_faces"] if "name" in f}


def code_plane_x(face: dict, offset: float) -> float:
    """The plane the codes sit in, which is not the shelf surface."""
    return face["face_x"] + (offset if face["yaw_deg"] > 0 else -offset)


def place(row, geometry) -> dict | None:
    """
    One reading as a position, or None if it cannot be placed.

    Everything here is the scanner's geometry with this reader's numbers in
    it. The vehicle pose is Gazebo's, measured from +X counter-clockwise,
    where the scanner's is MAVSDK's, measured from north; the forward and
    right vectors below are written for the one this file is given.
    """
    uav = row.get("uav")
    frame = row.get("frame_px")
    centre = row.get("centre")
    if not uav or not frame or not centre:
        return None

    link = row.get("camera_link", "")
    which = "rear" if "rear" in link else "hires"
    spec = geometry["cameras"][which]
    frame_w, frame_h = frame
    h_fov = math.radians(spec["hfov_deg"])
    v_fov = 2 * math.atan(math.tan(h_fov / 2) * frame_h / frame_w)

    dx_norm = (centre[0] - frame_w / 2) / (frame_w / 2)
    dy_norm = (centre[1] - frame_h / 2) / (frame_h / 2)
    bearing = math.atan(dx_norm * math.tan(h_fov / 2))
    elevation = math.atan(dy_norm * math.tan(v_fov / 2))

    # Where this camera looks. The rear one is mounted backwards, so its
    # frames are read with the heading turned through 180 degrees, which is
    # what puts its codes on the far side of the aisle.
    yaw = math.radians(row.get("uav_yaw_deg", 0.0)
                       + (180.0 if which == "rear" else 0.0))
    forward_x, forward_y = math.cos(yaw), math.sin(yaw)
    right_x, right_y = math.sin(yaw), -math.cos(yaw)

    # The face this camera is looking at: the nearest one ahead of it whose
    # own surface faces back. Chosen from the building rather than from the
    # reading, for the reason the scanner chooses it that way - there are
    # eight shelf planes and a code is on one of them.
    best, best_d = None, None
    for face in geometry["faces"].values():
        along = (face["face_x"] - uav["x"]) * forward_x
        if along <= 0.01:
            continue
        if best_d is None or along < best_d:
            best, best_d = face, along
    if best is None:
        return None

    depth = best_d - spec["mount_x"] + geometry["code_plane"]
    lateral = depth * math.tan(bearing)
    vertical = -depth * math.tan(elevation)

    box_x = code_plane_x(best, geometry["code_plane"])
    box_y = uav["y"] + forward_y * depth + right_y * lateral
    # The bars' own height. This files the barcode label, which is what the
    # barcode truth records, so nothing is added to reach the QR above it.
    box_z = uav["z"] + vertical

    flight_z = geometry["flight_z"]
    level = min(range(len(flight_z)), key=lambda i: abs(flight_z[i] - box_z))
    return {
        "shelf": best["name"],
        "level": level + 1,
        "x": box_x,
        "y": box_y,
        "z": box_z,
        "bearing": math.degrees(bearing),
        "elevation": math.degrees(elevation),
        "uav": uav,
        "yaw": row.get("uav_yaw_deg", 0.0),
        "camera": link or which,
    }


def build(readings, truth_path=GROUND_TRUTH, layout_path=LAYOUT) -> dict:
    layout = json.loads(Path(layout_path).read_text())
    geometry = {
        "cameras": cameras(),
        "faces": faces_by_name(layout_path),
        "code_plane": layout.get("code_plane_offset_m", 0.0),
        "flight_z": layout["flight_z"],
    }
    best = {}
    placed = skipped = 0
    for row in readings:
        if row.get("symbology") != "CODE128":
            continue
        code = row.get("payload")
        if not code:
            continue
        spot = place(row, geometry)
        if spot is None:
            skipped += 1
            continue
        placed += 1
        # Straightest on first: the perpendicular standoff describes the range
        # to a code dead ahead, and least well to one at the edge.
        if code not in best or abs(spot["bearing"]) < abs(best[code]["bearing"]):
            best[code] = spot

    items = []
    for code in sorted(best):
        s = best[code]
        items.append({
            "id": code,
            "estimated_x": round(s["x"], 3),
            "estimated_y": round(s["y"], 2),
            "estimated_z": round(s["z"], 3),
            "shelf": s["shelf"],
            "level": s["level"],
            "camera": s["camera"],
            "uav_position": {"x": round(s["uav"]["x"], 2),
                             "y": round(s["uav"]["y"], 2),
                             "z": round(s["uav"]["z"], 2)},
            "uav_heading_deg": round(s["yaw"], 1),
            "bearing_deg": round(s["bearing"], 1),
            "elevation_deg": round(s["elevation"], 1),
            "timestamp": datetime.now().isoformat(timespec="seconds"),
        })
    return {
        "scan_date": datetime.now().isoformat(timespec="seconds"),
        "sensor_configuration": "C27, Code128 box barcodes, both cameras",
        "localization": "vehicle pose from the simulator, read passively",
        "total_detected": len(items),
        "readings_placed": placed,
        "readings_without_a_pose": skipped,
        "items": items,
    }


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("readings", nargs="*",
                    help="readings files under out/; defaults to every one "
                         "this run left there")
    ap.add_argument("--out", type=Path, default=OUT / "inventory_barcode.json")
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    args = ap.parse_args()

    names = args.readings or default_readings()
    if not names:
        raise SystemExit("no barcode readings in %s; fly with "
                         "scripts/scan_with_barcode.sh" % OUT)
    rows = []
    per_camera = defaultdict(int)
    for name in names:
        got = load_readings(os.path.join(OUT, name))
        rows.extend(got)
        per_camera[camera_of(name)] += len(got)
    if not rows:
        raise SystemExit("no barcode readings in %s" % ", ".join(names))

    inv = build(rows, args.ground_truth)
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(inv, indent=2), encoding="utf-8")

    print("barcode inventory from %s" % ", ".join(names))
    for camera in sorted(per_camera):
        print("  %-8s %d readings" % (camera, per_camera[camera]))
    print("  placed                     %d" % inv["readings_placed"])
    if inv["readings_without_a_pose"]:
        print("  no pose, so not placed     %d" % inv["readings_without_a_pose"])
    print("  barcodes filed             %d" % inv["total_detected"])
    print("\nwritten to %s" % args.out)
    print("score it with:")
    print("  report/validate_inventory.py --inventory %s --code-type box_placard"
          % args.out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
