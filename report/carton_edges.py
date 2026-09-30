#!/usr/bin/env python3
"""
Where a carton is, from how its edges move while the vehicle flies past it.

Used by report/box_inventory.py; nothing to run on its own.

WHY THE BOX CENTRE IS NOT ENOUGH. A sighting is placed from the centre of the
box the detector drew, at the shelf plane. That is right for a carton seen
whole and square on, and wrong in two ways that add up for any other:

  * a carton cut off by the side of the frame has a box centre between its one
    visible edge and the frame edge, dragged towards the middle of the frame;
  * a carton seen at an angle is boxed WITH its side face, so the inner edge
    of the box is the BACK corner of the carton, a carton's depth behind the
    plane the placement assumes.

On the 2026-09-29 flight 140 of 430 cartons were never seen whole, and those
140 held 70 of the 76 cartons placed more than 0.25 m out. The cartons built
with no code were hit hardest: the detector does not recognise a blank carton
square on, only at an angle, so 12 of the 16 were never seen whole. Measured
in frame 1718: a 0.8 m blank carton filling the middle of the frame, unboxed.

WHAT THIS DOES INSTEAD. The edge of a box that is not cut by the frame is a
real corner of the carton, front or back. As the vehicle moves along the
shelf that corner slides across the frame, and HOW FAST it slides says how
far away it is - a near corner crosses quickly, a far one slowly. So one
corner, followed over a few frames, gives both its position along the shelf
and its depth, with no carton size assumed:

    y_corner = y_vehicle + side * depth * tan(bearing)

one line per frame, two unknowns, fitted by least squares. The depth comes
out a carton's depth behind the face exactly when the corner is a back one,
which is the check that it worked: 1.88 m against the 1.76 the world has for
the 0.8 m blank carton UNLABELLED|A|03|3, whose centre moved from 0.327 m out
to 0.014.

A corner bounding a carton on its low side and one bounding it on its high
side, the right distance apart, make a carton. Where only one side was ever
followed, the carton is centred half a carton's width from it - the width
MEASURED from the pairs on that face, and only on a face whose pairs agree,
because a face of mixed stock has no one width to use.

Then every cluster that was never seen whole moves to the carton its own
sightings' corners belong to, by vote. Not to the nearest carton: a cluster
dragged a whole carton-pitch sideways is nearest to the neighbour, which is
how the first attempt at this moved clusters onto the wrong carton.

WHAT IT CANNOT DO. A corner seen in one frame has no motion to measure. The
rear camera renders at 8 Hz and on the 0.50 m aisle a corner is in its frame
for one or two frames, so face H gets few corners and keeps most of its old
positions. That is a camera rate question, not a placement one.
"""

from __future__ import annotations

import math
from collections import Counter, defaultdict

from warehouse_model import load_config

# A corner is followed from frame to frame if it reappears within this many
# frames. One dropped detection is common; more than that and the vehicle has
# moved on far enough that a new corner is likelier than the old one.
MAX_GAP_FRAMES = 2

# The widest a pair of corners may be apart, and the narrowest, as fractions
# of the narrowest and widest carton the warehouse stocks. A back corner sits
# in front of the next carton's, so pairs measure a few centimetres short of
# the carton; measured XS pairs came out at 0.26-0.28 for a 0.32 m carton.
PAIR_SLACK = (0.8, 1.15)

# A face whose paired widths spread more than this between the quartiles is
# of mixed stock, and one width cannot turn a lone corner into a centre there.
ONE_WIDTH_IQR_M = 0.12

# Two corners closer than this along the shelf are the same corner.
SAME_CORNER_M = 0.05


def carton_sizes() -> tuple[float, float, float]:
    """Narrowest and widest front, and the deepest carton, from warehouse.yaml."""
    sizes = [s["dims"] for s in load_config()["boxes"]["sizes"]]
    return (min(d[0] for d in sizes), max(d[0] for d in sizes),
            max(d[1] for d in sizes))


def _corners_of(s: dict, spot: dict, geometry: dict, edge_px: float):
    """The uncut edges of one sighting, each as one line of the fit."""
    which = "rear" if "rear" in s.get("camera_link", "") else "hires"
    spec = geometry["cameras"][which]
    frame_w = s["frame_px"][0]
    tan_half = math.tan(math.radians(spec["hfov_deg"]) / 2)
    yaw = math.radians(s.get("uav_yaw_deg", 0.0)
                       + (180.0 if which == "rear" else 0.0))
    # The world-y direction of the frame's right edge; see place().
    right_y = -math.cos(yaw)
    face = geometry["faces"][spot["shelf"]]
    depth = (abs(face["face_x"] - s["uav"]["x"]) - spec["mount_x"]
             + geometry["code_plane"])
    x0, x1 = s["x_edges"]
    for px, uncut, is_right in ((x0, x0 >= edge_px, False),
                                (x1, x1 <= frame_w - edge_px, True)):
        if not uncut:
            continue
        yield {
            "t": right_y * (px - frame_w / 2) / (frame_w / 2) * tan_half,
            # Which side of the carton this corner bounds, in world y.
            "side": "hi" if right_y * (1 if is_right else -1) > 0 else "lo",
            "uy": s["uav"]["y"],
            "depth": depth,
            "frame": s.get("frame"),
        }


def corners(sightings: list[dict], spots: list[dict], geometry: dict,
            edge_px: float) -> tuple[list[dict], dict]:
    """
    Every carton corner that could be followed over two frames or more, and
    which corners each sighting contributed to.

    `sightings` and `spots` are parallel: the placed spot of each kept
    sighting, in flight order, each carrying its `sid`.
    """
    _, _, deepest = carton_sizes()
    open_tracks: dict[tuple, list] = defaultdict(list)
    done: list[dict] = []
    for s, spot in zip(sightings, spots):
        cam = s.get("camera_link", "")
        for key in [k for k in open_tracks if k[0] == cam]:
            still = []
            for tr in open_tracks[key]:
                gap = (s.get("frame") or 0) - (tr["obs"][-1]["frame"] or 0)
                (still if gap <= MAX_GAP_FRAMES else done).append(tr)
            open_tracks[key] = still
        for c in _corners_of(s, spot, geometry, edge_px):
            c["sid"] = spot["sid"]
            key = (cam, c["side"], spot["shelf"], spot["level"])
            lo, hi = 0.6 * c["depth"], c["depth"] + deepest + 0.4
            best = None
            for tr in open_tracks[key]:
                last = tr["obs"][-1]
                if last["frame"] == c["frame"]:
                    continue
                duy, dt = c["uy"] - last["uy"], c["t"] - last["t"]
                if abs(duy) < 1e-3 or abs(dt) < 1e-4:
                    continue
                rho = -duy / dt
                if not lo <= rho <= hi:
                    continue
                miss = abs(rho - tr.get("rho", c["depth"] + deepest / 2))
                if best is None or miss < best[0]:
                    best = (miss, tr, rho)
            if best:
                best[1]["obs"].append(c)
                best[1]["rho"] = best[2]
            else:
                open_tracks[key].append({"key": key, "obs": [c]})
    for trs in open_tracks.values():
        done += trs

    found, of_sid = [], defaultdict(list)
    for tr in done:
        obs = tr["obs"]
        if len(obs) < 2:
            continue
        xs = [o["t"] for o in obs]
        ys = [o["uy"] for o in obs]
        n = len(obs)
        mx, my = sum(xs) / n, sum(ys) / n
        sxx = sum((x - mx) ** 2 for x in xs)
        if sxx < 1e-6:
            continue
        k = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / sxx
        rho = -k
        if not 0.6 * obs[0]["depth"] <= rho <= obs[0]["depth"] + deepest + 0.4:
            continue
        cid = len(found)
        found.append({"side": tr["key"][1], "shelf": tr["key"][2],
                      "level": tr["key"][3], "y": my - k * mx, "depth": rho,
                      "frames": n})
        for o in obs:
            of_sid[o["sid"]].append(cid)
    return found, of_sid


def cartons_from(found: list[dict]) -> tuple[list[dict], dict]:
    """
    Corners paired into cartons, and which carton each corner belongs to.
    """
    narrow, wide, _ = carton_sizes()
    min_w, max_w = narrow * PAIR_SLACK[0], wide * PAIR_SLACK[1]
    by = defaultdict(lambda: {"lo": [], "hi": []})
    for i, c in enumerate(found):
        by[(c["shelf"], c["level"])][c["side"]].append(i)

    out, of_corner = [], {}
    for key, sides in by.items():
        lows = sorted(sides["lo"], key=lambda i: found[i]["y"])
        highs = sorted(sides["hi"], key=lambda i: found[i]["y"])
        for i in lows:
            low = found[i]["y"]
            fits = [j for j in highs if min_w <= found[j]["y"] - low <= max_w]
            if not fits:
                continue
            j = min(fits, key=lambda j: found[j]["y"])
            # Another carton starts between them: not one carton.
            if any(low + SAME_CORNER_M < found[m]["y"]
                   < found[j]["y"] - SAME_CORNER_M for m in lows):
                continue
            of_corner.setdefault(i, len(out))
            of_corner.setdefault(j, len(out))
            out.append({"y": (low + found[j]["y"]) / 2,
                        "width": found[j]["y"] - low,
                        "shelf": key[0], "level": key[1], "from": "pair"})
        # The same corner followed again on another pass joins its carton.
        for side in ("lo", "hi"):
            for i in sides[side]:
                if i in of_corner:
                    continue
                twin = [j for j in sides[side] if j in of_corner
                        and abs(found[j]["y"] - found[i]["y"]) < SAME_CORNER_M]
                if twin:
                    of_corner[i] = of_corner[twin[0]]

    # A lone corner, on a face of one carton width, measured from its pairs.
    for face in {c["shelf"] for c in found}:
        widths = sorted(c["width"] for c in out if c["shelf"] == face)
        if len(widths) < 3:
            continue
        iqr = widths[3 * len(widths) // 4] - widths[len(widths) // 4]
        if iqr > ONE_WIDTH_IQR_M:
            continue
        w = widths[len(widths) // 2]
        for i, c in enumerate(found):
            if c["shelf"] != face or i in of_corner:
                continue
            of_corner[i] = len(out)
            out.append({"y": c["y"] + (w / 2 if c["side"] == "lo" else -w / 2),
                        "width": w, "shelf": face, "level": c["level"],
                        "from": "one corner"})
    return out, of_corner


def replace(clusters: list[dict], found_cartons: list[dict], of_corner: dict,
            of_sid: dict) -> int:
    """
    Move every cluster never seen whole to the carton its corners vote for.
    Returns how many moved.
    """
    moved = 0
    for c in clusters:
        if c["n_whole"]:
            continue
        votes = Counter(of_corner[k] for sid in c["members"]
                        for k in of_sid.get(sid, ()) if k in of_corner)
        if not votes:
            continue
        c["y"] = found_cartons[votes.most_common(1)[0][0]]["y"]
        c["placed_by"] = "corners"
        moved += 1
    return moved
