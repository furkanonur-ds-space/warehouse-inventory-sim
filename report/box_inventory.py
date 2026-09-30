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
import re
import sys
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from warehouse_model import (GROUND_TRUTH, LAYOUT, REPO_ROOT, WORLD_SDF,
                             cameras, load_config)
from barcode_inventory import LANE_MARGIN_M, faces_by_name, place
import carton_edges

OUT = REPO_ROOT / "out"

# Below this a sighting is dropped. See the threshold note in the docstring.
MIN_AREA_PX = 20000.0

# A box nearer the frame's side edge than this is cut off by the frame, and
# its centre is not the carton's: it sits between the carton's one visible
# edge and the frame edge, which drags it towards the middle of the frame.
# Measured on the 2026-09-29 flight, a box cut off on the left landed a median
# 0.16-0.25 m to one side of its carton and one cut off on the right the same
# distance to the other, on every face. 60 per cent of the front camera's boxes
# and 26 per cent of the rear's are cut off like this, so dropping them costs
# 64 cartons; they are kept for FINDING a carton and left out of WHERE it is.
EDGE_PX = 3.0

# How much further away than the face a box may look before it is not on the
# face. The detector also boxes cartons deeper in the rack, seen through the
# gap between two on the face, and the placement pins every box to the face's
# plane - so a carton two metres back lands on the face beside a real one, as
# a spurious cluster or as a wrong position for the real one. Its apparent
# height gives it away: the shortest carton the warehouse stocks, standing on
# the face, can look no shorter than it does at the face's own distance.
#
# Measured on the 2026-09-29 flight, as cartons more than 0.25 m out:
#     no gate  101     1.15 to 1.5   74-76     2.0  86     3.0  101
# Flat between 1.15 and 1.5, so the looser end is taken. It lifted cartons
# found from 426 to 430 and the unlabelled ones from 15 to 16 of 16, because
# the far boxes were also pulling real clusters off their cartons.
# A box cut off at the top or bottom of the frame has no true height and is
# not judged by it.
RANGE_GATE = 1.5

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

# How near a DECODED code has to sit for a carton to count as read.
#
# Not the per-frame test it replaces. `covered` asks whether a code polygon
# landed inside the carton's own pixels in that frame, and that fails whenever
# the model does not box the carton at the moment its code is readable, which
# is exactly when the carton fills the frame. On the 2026-09-27 flight, which
# read 432 of 432, the per-frame test left 129 cartons looking unread. Matching
# the two lists in three dimensions instead leaves 21.
#
# Measured on that flight, as the share of real cartons wrongly called unread:
#     0.40 m   15.1 %
#     0.50 m    2.6 %      <- this
#     0.60 m    0.4 %
#
# It does not keep going down usefully. Cartons stand about 0.40 m apart, so a
# radius past 0.5 m reaches the neighbour and a genuinely unread carton beside
# a read one is quietly called read. The honest reading of this warning is AT
# BAY LEVEL: something in this bay was not read. It cannot name which of two
# neighbours.
#
# The table above was measured when a cluster was good to 0.157 m. Since the
# range gate and the cut boxes it is good to 0.06, and on the 2026-09-29
# flight this radius then warns about 13 of the 16 cartons with no code and 3
# that were read, against 14 and 1 before. Re-measured, nothing is simply
# better: 0.35 m warns about all 16 and 23 read ones, 0.60 m about 10 and none,
# and letting each code clear only the cluster nearest to it gives 16 and 7.
# The distance is to where the codes were read, and a label is not at its
# carton's middle, so a better cluster does not by itself make a better test.
CODE_RADIUS_M = 0.50


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
            frame_w, frame_h = row.get("frame_px") or (math.inf, math.inf)
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
                    "height_px": y1 - y0,
                    "x_edges": [x0, x1],
                    "cut_side": x0 < EDGE_PX or x1 > frame_w - EDGE_PX,
                    "cut_top": y0 < EDGE_PX or y1 > frame_h - EDGE_PX,
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
        "shortest_carton_m": shortest_carton_m(),
    }


def shortest_carton_m() -> float:
    """The lowest front face of any carton size, from warehouse.yaml."""
    return min(s["dims"][2] for s in load_config()["boxes"]["sizes"])


def too_far(s: dict, spot: dict, geometry: dict, gate: float) -> bool:
    """
    Whether a box looks too small to be standing on the face it was placed on.
    See RANGE_GATE.
    """
    if s["cut_top"] or s["height_px"] <= 0:
        return False
    which = "rear" if "rear" in s.get("camera_link", "") else "hires"
    spec = geometry["cameras"][which]
    focal = (s["frame_px"][0] / 2) / math.tan(math.radians(spec["hfov_deg"]) / 2)
    face = geometry["faces"][spot["shelf"]]
    depth = abs(face["face_x"] - s["uav"]["x"]) - spec["mount_x"]
    looks = geometry["shortest_carton_m"] * focal / s["height_px"]
    return looks > gate * depth


def cluster(spots: list[dict], radius: float) -> list[dict]:
    """
    Sightings of one carton, gathered into one carton.

    Greedy and single pass, on purpose. The alternative is a proper clustering
    and this does not need one: the sightings arrive in flight order, so the
    ones belonging to a carton arrive together, and a carton the vehicle
    passes twice makes two clusters that both match it and the second is
    reported as a duplicate rather than silently merged.

    A cluster sits where its whole boxes put it, and only where none of its
    boxes is whole does it fall back on the cut ones. See EDGE_PX. Such a
    cluster is moved again afterwards, by report/carton_edges.py, which is
    why each keeps the sightings it was made of.
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
            hit = {"n": 0, "all": [0.0, 0.0, 0.0], "whole": [0.0, 0.0, 0.0],
                   "n_whole": 0, "members": [], "placed_by": "cut boxes",
                   "shelf": s["shelf"], "level": s["level"],
                   "camera": s["camera"], "covered": 0,
                   "best_area": s["area"], "best_conf": s["conf"]}
            out.append(hit)
        hit["n"] += 1
        hit["members"].append(s.get("sid"))
        for i, k in enumerate("xyz"):
            hit["all"][i] += s[k]
        if not s.get("cut_side"):
            hit["n_whole"] += 1
            for i, k in enumerate("xyz"):
                hit["whole"][i] += s[k]
        total, n = ((hit["whole"], hit["n_whole"]) if hit["n_whole"]
                    else (hit["all"], hit["n"]))
        if hit["n_whole"]:
            hit["placed_by"] = "whole boxes"
        hit["x"], hit["y"], hit["z"] = (v / n for v in total)
        hit["covered"] += int(s["covered"])
        hit["best_area"] = max(hit["best_area"], s["area"])
        hit["best_conf"] = max(hit["best_conf"], s["conf"])
    return out


def decoded_places(out: Path) -> list[tuple]:
    """
    Where the run actually read a code, from the inventories it wrote.

    Both inventories, because the two labels fail independently: a carton
    whose QR read and whose barcode did not has been read, and warning about
    it would be false. Missing files are not an error - a flight with no
    reader leaves none and then every carton is unread, which is true.
    """
    places = []
    for name in ("inventory_barcode.json", "inventory_scanned.json"):
        path = out / name
        if not path.exists():
            continue
        try:
            body = json.loads(path.read_text())
        except json.JSONDecodeError:
            continue
        for item in body.get("items", []):
            try:
                places.append((float(item["estimated_x"]),
                               float(item["estimated_y"]),
                               float(item["estimated_z"])))
            except (KeyError, TypeError, ValueError):
                continue
    return places


def true_cartons(path: Path = GROUND_TRUTH) -> list[dict]:
    """
    Every carton that is really there, from where its QR label sits.

    `box_unlabelled` counts too, and this is the whole point of it. Those are
    the cartons the world was generated WITHOUT codes on, for the experiment
    that asks whether the detector finds a carton nothing can read. Leaving
    them out would make the experiment score itself: the twelve would vanish
    from the denominator and a detector that missed every one of them would
    still read 100 per cent.

    They carry no payload, so they are named by position instead. Nothing
    downstream reads that name as something decoded - see the note about
    assigned ids at the top of this file.

    THE HEIGHT IS THE CARTON'S, NOT THE LABEL'S. The detector boxes the
    carton, so a cluster sits at the carton's middle, and the QR label is
    stuck 0.06 m above it on a small carton and 0.12 m on a large one. Scored
    against the label that difference came out as a constant error on every
    carton on every face - about a third of the reported error and none of it
    the detector's. The height is read from the generated world, where the
    carton is built, and the label's is kept only if the world is missing.
    """
    truth = json.loads(Path(path).read_text())
    heights = carton_heights()
    out = []
    for c in truth.get("codes", []):
        kind = c.get("type")
        if kind not in ("box_qr", "box_unlabelled"):
            continue
        x, y, z = c["label_pose_xyzrpy"][:3]
        payload = c.get("payload") or (
            f"UNLABELLED|{c.get('row')}|{c.get('bay'):02d}|{c.get('level')}")
        z = heights.get(c.get("entity", "").split("::")[-1], z)
        out.append({"payload": payload, "row": c.get("row"),
                    "bay": c.get("bay"), "level": c.get("level"),
                    "unlabelled": kind == "box_unlabelled",
                    "x": x, "y": y, "z": z})
    return out


def carton_heights(world: Path = WORLD_SDF) -> dict:
    """The middle height of every carton body in the generated world."""
    if not world.exists():
        return {}
    found = re.finditer(
        r'<link name="(box_[^"]+)">\s*<visual name="body">\s*<pose>([^<]+)</pose>',
        world.read_text())
    return {m.group(1): float(m.group(2).split()[2]) for m in found}


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


def build(out_dir: Path, min_area: float, radius: float, limit: float,
          code_radius: float = CODE_RADIUS_M,
          gate: float = RANGE_GATE) -> dict:
    paths = box_logs(out_dir)
    if not paths:
        return {"used": False}

    geometry = geometry_of()
    ground = min(geometry["flight_z"]) - LANE_MARGIN_M
    raw, dropped = sightings(paths, min_area)
    spots, placed_from, off_lane, unplaced, behind = [], [], 0, 0, 0
    for s in raw:
        uav = s.get("uav")
        if uav and uav.get("z", 0.0) < ground:
            off_lane += 1
            continue
        spot = place(s, geometry)
        if spot is None:
            unplaced += 1
            continue
        if too_far(s, spot, geometry, gate):
            behind += 1
            continue
        spot.update(area=s["area"], conf=s["conf"], covered=s["covered"],
                    cut_side=s["cut_side"], sid=len(spots))
        spots.append(spot)
        placed_from.append(s)

    clusters = cluster(spots, radius)
    corners, of_sid = carton_edges.corners(placed_from, spots, geometry,
                                           EDGE_PX)
    from_corners, of_corner = carton_edges.cartons_from(corners)
    moved = carton_edges.replace(clusters, from_corners, of_corner, of_sid)
    cartons = true_cartons()
    pairing = match(clusters, cartons, limit)

    # THE WARNING. A carton the detector placed, with no code decoded anywhere
    # near it, is one the run saw and did not read. In a warehouse with no
    # ground truth this is the only thing that can raise a hand.
    codes = decoded_places(out_dir)
    for c in clusters:
        near = min((math.dist((c["x"], c["y"], c["z"]), k) for k in codes),
                   default=None)
        c["code_m"] = round(near, 3) if near is not None else None
        c["read"] = near is not None and near <= code_radius

    errors = sorted(d for _, d in pairing.values())
    found = {ti for ti, _ in pairing.values()}

    # THE EXPERIMENT. Cartons the world was generated with no codes on. They
    # cannot be read by anything, so the detector is the only thing that can
    # report them at all, and the warning has to name every one it found. A
    # world with none of them leaves this empty and it is not printed.
    bare = [(i, c) for i, c in enumerate(cartons) if c.get("unlabelled")]
    bare_found = [c["payload"] for i, c in bare if i in found]
    bare_missed = [c["payload"] for i, c in bare if i not in found]

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
            "placed_by": c["placed_by"],
            "error_m": round(hit[1], 3) if hit else None,
            "nearest_code_m": c["code_m"],
            "read": c["read"],
        })

    return {
        "used": True,
        "generated": datetime.now().isoformat(timespec="seconds"),
        "logs": [p.name for p in paths],
        "settings": {"min_area_px": min_area, "cluster_m": radius,
                     "match_m": limit, "range_gate": gate,
                     "edge_px": EDGE_PX},
        "sightings": {"kept": len(raw), "below_area": dropped,
                      "off_lane": off_lane, "unplaceable": unplaced,
                      "behind_the_face": behind,
                      "cut_by_frame_side": sum(1 for s in spots
                                               if s["cut_side"]),
                      "placed": len(spots)},
        "corners": {
            "followed": len(corners),
            "cartons_from_pairs": sum(1 for c in from_corners
                                      if c["from"] == "pair"),
            "cartons_from_one_corner": sum(1 for c in from_corners
                                           if c["from"] != "pair"),
            "clusters_never_seen_whole": sum(1 for c in clusters
                                             if not c["n_whole"]),
            "clusters_moved": moved,
        },
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
        "seen_but_not_read": sorted(
            {cartons[pairing[ci][0]]["payload"]
             for ci, c in enumerate(clusters)
             if ci in pairing and not c["read"]}),
        "code_radius_m": code_radius,
        "unlabelled": {
            "total": len(bare),
            "detected": len(bare_found),
            "not_detected": sorted(bare_missed),
        },
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
        f"  dropped {sg.get('behind_the_face', 0)} too small to be on the face "
        f"(deeper in the rack); {sg.get('cut_by_frame_side', 0)} cut by the "
        f"frame side, used to find but not to place",
        f"  clustered into {c['clusters']} cartons at {s['cluster_m']} m",
        f"  {data['corners']['clusters_never_seen_whole']} never seen whole; "
        f"{data['corners']['clusters_moved']} of them placed from "
        f"{data['corners']['followed']} carton corners followed across "
        f"frames",
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
        lines.append("  against the carton's middle; its depth is assumed at "
                     "the shelf plane, not measured")
    warn = data.get("seen_but_not_read") or []
    lines += ["",
              f"SEEN AND NOT READ: {len(warn)} cartons placed with no decoded "
              f"code within {data['code_radius_m']} m"]
    if warn:
        show = warn[:15]
        lines.append("  " + ", ".join(show)
                     + (" ..." if len(warn) > len(show) else ""))
    lines.append("  This is the warning a warehouse with no ground truth "
                 "would act on. Read it at BAY")
    lines.append("  level: cartons stand 0.40 m apart and a label is not at "
                 "its carton's middle, so it cannot")
    lines.append("  tell which of two neighbours went unread.")

    bare = data.get("unlabelled") or {}
    if bare.get("total"):
        n, d = bare["total"], bare["detected"]
        lines += ["",
                  f"CARTONS WITH NO CODE PRINTED ON THEM (the experiment): "
                  f"{d} of {n} detected",
                  "  Nothing can read these, so the detector is the only "
                  "thing that can report them."]
        if bare["not_detected"]:
            lines.append("  invisible to it: "
                         + ", ".join(bare["not_detected"]))

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
    ap.add_argument("--code-radius", type=float, default=CODE_RADIUS_M,
                    help=f"a carton with no decoded code this near was seen "
                         f"and not read (default {CODE_RADIUS_M} m)")
    ap.add_argument("--match", type=float, default=MATCH_M,
                    help=f"how near a cluster must land to be called that "
                         f"carton (default {MATCH_M} m)")
    ap.add_argument("--range-gate", type=float, default=RANGE_GATE,
                    help=f"drop a box that looks this many times further away "
                         f"than the face (default {RANGE_GATE})")
    ap.add_argument("--json", type=Path, default=OUT / "box_report.json")
    ap.add_argument("--experiment-inventory", type=Path,
                    default=OUT / "inventory_boxes_unlabelled.json",
                    help="just the cartons built with no code on them, so the "
                         "viewer can draw the experiment on its own instead "
                         "of hiding twelve dots among nine hundred")
    ap.add_argument("--inventory", type=Path,
                    default=OUT / "inventory_boxes.json",
                    help="the cartons in the inventory shape, so "
                         "view_inventory.py can draw them")
    args = ap.parse_args()

    data = build(args.out_dir, args.min_area, args.cluster, args.match,
                 args.code_radius, args.range_gate)
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
        bare = data.get("unlabelled") or {}
        if bare.get("total"):
            # Only the experiment's cartons, scored against only the
            # experiment's truth. Drawn on its own the page answers one
            # question - which of the twelve unreadable cartons the detector
            # found, and where the one it missed is standing - instead of
            # putting twelve dots among nine hundred.
            names = {i["id"] for i in data["items"]}
            items = [i for i in data["items"] if i["id"].startswith("UNLABELLED|")]
            args.experiment_inventory.write_text(json.dumps({
                "scan_date": data["generated"],
                "sensor_configuration": "yolo box detector, unlabelled cartons",
                "total_detected": len(items),
                "items": items,
            }, indent=2, ensure_ascii=False))
            print(f"written: {args.experiment_inventory}")
            print(f"  draw it: .venv/bin/python report/view_inventory.py "
                  f"--inventory {args.experiment_inventory} "
                  f"--code-type box_unlabelled --out out/experiment_3d.html")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
