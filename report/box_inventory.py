#!/usr/bin/env python3
"""
What the box detector found, as cartons in the warehouse rather than sightings.

    python3 report/box_inventory.py
    python3 report/box_inventory.py --html out/boxes_3d.html

A pure consumer. It reads the box log a flight with SAVE_BOXES=1 left in
out/yolo_boxes_<camera>.jsonl, the layout, and ground truth. It imports
nothing from scanner/ and runs no model: the detecting was done in flight.

WHY THIS EXISTS. The run can say it read 432 of 432 because ground truth says
there are 432. A warehouse that is not this one has no such file, so the only
available denominator is a detector that counts cartons. Whether that is a
denominator worth having is a question about the DETECTOR, and it cannot be
answered by counting sightings: the same carton is seen in dozens of frames
and the per-frame numbers say nothing about how many cartons were ever seen
at all. This turns 32167 sightings into a list of cartons, and scores that
list against the 432 that are really there.

HOW A SIGHTING BECOMES A PLACE. Exactly as a barcode reading does - the
bearing from where the box sat in the frame, the perpendicular distance to
the shelf plane the camera faces, and the vehicle pose - so `place()` is
imported from report/barcode_inventory.py rather than rewritten. One thing is
weaker here and worth knowing: a code carries its own size, so a reading can
be ranged. A carton cannot, and its distance comes entirely from the assumed
shelf plane. A carton standing on the face is placed well; one further back
is placed at the face anyway. Expect the scatter to be worse than the
barcode's, and read the position error as a sanity check rather than a
measurement of the detector.

THE AREA THRESHOLD is what makes the list meaningful. Measured on the
2026-09-27 flight, over 32167 sightings: no code was EVER decoded inside a
carton smaller than about 20000 px, while sightings below that make up four
fifths of the ones with no code in them. So the cut costs nothing real and
removes most of the noise - a carton half an aisle away that nobody could
have read. Confidence does not separate the two cases nearly as well (0.88
against 0.54) and is deliberately not used for this.

IDS ARE ASSIGNED, NOT READ. The detector cannot tell one carton from another.
A cluster is given the id of the nearest true carton so that the existing
viewer and validator can draw it, and the distance to that same carton is
then reported as its error. That is ordinary nearest-neighbour scoring, but
it means this file must never be fed to anything that treats an id as
something the flight decoded. It is written to its own name for that reason.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from warehouse_model import GROUND_TRUTH, LAYOUT, REPO_ROOT, cameras
from barcode_inventory import LANE_MARGIN_M, faces_by_name, place

OUT = REPO_ROOT / "out"

# Below this a sighting is dropped. See the threshold note in the docstring.
MIN_AREA_PX = 20000.0

# Two sightings closer than this are the same carton. The boxes sit about
# 0.40 m apart along a shelf, so this cannot reach the neighbour; it is
# generous vertically only because a carton's centre wanders in the frame as
# the vehicle passes it.
CLUSTER_M = 0.30

# How near a cluster has to land to be called that carton. Larger than the
# cluster radius on purpose: the carton is placed at the shelf plane and its
# true centre is behind it, so a systematic offset of a few centimetres is
# expected and is not a failure to find it.
MATCH_M = 0.60


def box_logs(out: Path) -> list[Path]:
    return sorted(out.glob("yolo_boxes_*.jsonl"))


def sightings(paths, min_area: float) -> tuple[list[dict], int]:
    """
    Every carton sighting big enough to be worth placing, and how many were
    not. Returned rather than yielded: the counts are only true once the last
    line has been read, and a generator that hands them back as it goes hands
    back the wrong ones at the end.
    """
    kept: list[dict] = []
    dropped = 0
    for path in paths:
        for line in path.read_text().splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            for b in row.get("boxes", []):
                x0, y0, x1, y1 = b["box"]
                area = (x1 - x0) * (y1 - y0)
                if area < min_area:
                    dropped += 1
                    continue
                kept.append({
                    # place() reads exactly these names; a sighting is shaped
                    # like a reading so nothing has to be special-cased there.
                    "uav": row.get("uav"),
                    "uav_yaw_deg": row.get("uav_yaw_deg", 0.0),
                    "frame_px": row.get("frame_px"),
                    "camera_link": row.get("camera_link", ""),
                    "centre": [(x0 + x1) / 2.0, (y0 + y1) / 2.0],
                    "area": area,
                    "conf": b.get("conf", 0.0),
                    "covered": bool(b.get("covered")),
                    "frame": row.get("frame"),
                })
    return kept, dropped


def geometry_of(layout_path: Path = LAYOUT) -> dict:
    layout = json.loads(Path(layout_path).read_text())
    return {
        "cameras": cameras(),
        "faces": faces_by_name(layout_path),
        "code_plane": layout.get("code_plane_offset_m", 0.0),
        "flight_z": layout["flight_z"],
    }


def cluster(spots: list[dict], radius: float) -> list[dict]:
    """
    Sightings of one carton, gathered into one carton.

    Greedy and single pass, on purpose. The alternative is a proper clustering
    and this does not need one: the sightings arrive in flight order, so the
    ones belonging to a carton arrive together, and a carton the vehicle
    passes twice makes two clusters that both match it and the second is
    reported as a duplicate rather than silently merged.
    """
    out: list[dict] = []
    r2 = radius * radius
    for s in spots:
        hit = None
        for c in reversed(out[-40:]):          # recent ones only; see above
            dx = c["x"] - s["x"]
            dy = c["y"] - s["y"]
            dz = c["z"] - s["z"]
            if dx * dx + dy * dy + dz * dz <= r2 and c["shelf"] == s["shelf"]:
                hit = c
                break
        if hit is None:
            out.append({"x": s["x"], "y": s["y"], "z": s["z"], "n": 1,
                        "shelf": s["shelf"], "level": s["level"],
                        "camera": s["camera"], "covered": int(s["covered"]),
                        "best_area": s["area"], "best_conf": s["conf"]})
            continue
        n = hit["n"] + 1
        hit["x"] += (s["x"] - hit["x"]) / n
        hit["y"] += (s["y"] - hit["y"]) / n
        hit["z"] += (s["z"] - hit["z"]) / n
        hit["n"] = n
        hit["covered"] += int(s["covered"])
        hit["best_area"] = max(hit["best_area"], s["area"])
        hit["best_conf"] = max(hit["best_conf"], s["conf"])
    return out


def true_cartons(path: Path = GROUND_TRUTH) -> list[dict]:
    """Every carton that is really there, from its own QR label's pose."""
    truth = json.loads(Path(path).read_text())
    out = []
    for c in truth.get("codes", []):
        if c.get("type") != "box_qr":
            continue
        x, y, z = c["label_pose_xyzrpy"][:3]
        out.append({"payload": c["payload"], "row": c.get("row"),
                    "bay": c.get("bay"), "level": c.get("level"),
                    "x": x, "y": y, "z": z})
    return out


def match(clusters: list[dict], cartons: list[dict], limit: float):
    """
    Each cluster to its nearest carton, each carton to at most one cluster.

    Nearest first across the whole set rather than cluster by cluster: taking
    them in flight order lets an early, badly placed cluster claim a carton
    that a later and much closer one wanted, and the carton then reads as
    found at 0.5 m when it was found at 0.05.
    """
    pairs = []
    for ci, c in enumerate(clusters):
        for ti, t in enumerate(cartons):
            d = math.dist((c["x"], c["y"], c["z"]), (t["x"], t["y"], t["z"]))
            if d <= limit:
                pairs.append((d, ci, ti))
    pairs.sort()
    taken_c, taken_t, out = set(), set(), {}
    for d, ci, ti in pairs:
        if ci in taken_c or ti in taken_t:
            continue
        taken_c.add(ci); taken_t.add(ti)
        out[ci] = (ti, d)
    return out


def build(out_dir: Path, min_area: float, radius: float, limit: float) -> dict:
    paths = box_logs(out_dir)
    if not paths:
        return {"used": False}

    geometry = geometry_of()
    ground = min(geometry["flight_z"]) - LANE_MARGIN_M
    raw, dropped = sightings(paths, min_area)
    spots, off_lane, unplaced = [], 0, 0
    for s in raw:
        uav = s.get("uav")
        if uav and uav.get("z", 0.0) < ground:
            off_lane += 1
            continue
        spot = place(s, geometry)
        if spot is None:
            unplaced += 1
            continue
        spot.update(area=s["area"], conf=s["conf"], covered=s["covered"])
        spots.append(spot)

    clusters = cluster(spots, radius)
    cartons = true_cartons()
    pairing = match(clusters, cartons, limit)

    errors = sorted(d for _, d in pairing.values())
    found = {ti for ti, _ in pairing.values()}

    # An unmatched cluster is one of two very different things and reporting
    # them as one number was misleading. The vehicle passes a face three
    # times, once per flight level, and both cameras see it, so the SAME
    # carton makes several clusters and only one of them can hold the match.
    # Those are duplicates and say nothing bad about the detector. A cluster
    # nowhere near any carton is a spurious one and does.
    duplicates = spurious = 0
    for ci, c in enumerate(clusters):
        if ci in pairing:
            continue
        near = min((math.dist((c["x"], c["y"], c["z"]), (t["x"], t["y"], t["z"]))
                    for t in cartons), default=None)
        if near is not None and near <= limit:
            duplicates += 1
        else:
            spurious += 1
    items = []
    for ci, c in enumerate(clusters):
        hit = pairing.get(ci)
        items.append({
            "id": cartons[hit[0]]["payload"] if hit else f"unmatched_{ci}",
            "estimated_x": round(c["x"], 3),
            "estimated_y": round(c["y"], 3),
            "estimated_z": round(c["z"], 3),
            "shelf": c["shelf"], "level": c["level"], "camera": c["camera"],
            "sightings": c["n"],
            "sightings_with_a_code": c["covered"],
            "best_area_px": round(c["best_area"]),
            "best_conf": round(c["best_conf"], 3),
            "error_m": round(hit[1], 3) if hit else None,
        })

    return {
        "used": True,
        "generated": datetime.now().isoformat(timespec="seconds"),
        "logs": [p.name for p in paths],
        "settings": {"min_area_px": min_area, "cluster_m": radius,
                     "match_m": limit},
        "sightings": {"kept": len(raw), "below_area": dropped,
                      "off_lane": off_lane, "unplaceable": unplaced,
                      "placed": len(spots)},
        "cartons": {
            "clusters": len(clusters),
            "true_total": len(cartons),
            "found": len(found),
            "never_detected": len(cartons) - len(found),
            "duplicate_clusters": duplicates,
            "spurious_clusters": spurious,
        },
        "position_error": {
            "median_m": round(errors[len(errors) // 2], 3) if errors else None,
            "p95_m": round(errors[int(0.95 * (len(errors) - 1))], 3)
            if errors else None,
            "max_m": round(errors[-1], 3) if errors else None,
        },
        "missed": sorted(c["payload"] for i, c in enumerate(cartons)
                         if i not in found),
        "items": items,
    }


def render(data: dict) -> str:
    if not data.get("used"):
        return ("No box log in out/: fly one with\n"
                "  SAVE_BOXES=1 YOLO=1 bash scripts/scan_with_barcode.sh")
    s, c, sg = data["settings"], data["cartons"], data["sightings"]
    pe = data["position_error"]
    pct = 100.0 * c["found"] / max(1, c["true_total"])
    lines = [
        "Cartons the box detector found",
        f"  sightings kept {sg['kept']} (dropped {sg['below_area']} under "
        f"{s['min_area_px']:.0f} px), placed {sg['placed']}",
        f"  clustered into {c['clusters']} cartons at {s['cluster_m']} m",
        "",
        f"  FOUND {c['found']} of {c['true_total']} cartons  ({pct:.1f} %)",
        f"  never detected            {c['never_detected']}",
        "",
        f"  of the {c['clusters']} clusters: {c['found']} are a carton, "
        f"{c['duplicate_clusters']} are another look at one already counted,",
        f"  and {c['spurious_clusters']} are near no carton at all "
        f"({c['spurious_clusters'] / max(1, c['clusters']):.0%} spurious).",
        "  Duplicates are expected: each face is flown three times, by two "
        "cameras.",
    ]
    if pe["median_m"] is not None:
        lines.append(f"  position error median {pe['median_m']} m, "
                     f"p95 {pe['p95_m']} m, max {pe['max_m']} m")
        lines.append("  (placed at the shelf plane, so read this as a sanity "
                     "check, not a measurement)")
    if data["missed"]:
        show = data["missed"][:15]
        lines += ["", "cartons the detector never saw:"]
        lines.append("  " + ", ".join(show)
                     + (" ..." if len(data["missed"]) > len(show) else ""))
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", type=Path, default=OUT)
    ap.add_argument("--min-area", type=float, default=MIN_AREA_PX,
                    help=f"drop sightings smaller than this (default "
                         f"{MIN_AREA_PX:.0f} px)")
    ap.add_argument("--cluster", type=float, default=CLUSTER_M,
                    help=f"sightings nearer than this are one carton "
                         f"(default {CLUSTER_M} m)")
    ap.add_argument("--match", type=float, default=MATCH_M,
                    help=f"how near a cluster must land to be called that "
                         f"carton (default {MATCH_M} m)")
    ap.add_argument("--json", type=Path, default=OUT / "box_report.json")
    ap.add_argument("--inventory", type=Path,
                    default=OUT / "inventory_boxes.json",
                    help="the cartons in the inventory shape, so "
                         "view_inventory.py can draw them")
    args = ap.parse_args()

    data = build(args.out_dir, args.min_area, args.cluster, args.match)
    print(render(data))
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    print(f"\nwritten: {args.json}")
    if data.get("used"):
        args.inventory.write_text(json.dumps({
            "scan_date": data["generated"],
            "sensor_configuration": "yolo box detector",
            "total_detected": len(data["items"]),
            "items": data["items"],
        }, indent=2, ensure_ascii=False))
        print(f"written: {args.inventory}")
        print(f"  draw it: .venv/bin/python report/view_inventory.py "
              f"--inventory {args.inventory} --out out/boxes_3d.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
