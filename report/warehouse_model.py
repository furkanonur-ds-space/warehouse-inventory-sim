#!/usr/bin/env python3
"""
What the reporting tools need to know about the warehouse and about a run.

Everything here is read-only. Nothing in `report/` writes to `warehouse/` or
`scanner/`, and nothing in it is imported by the scanner: these tools run after
a flight, on the JSON that flight produced. A broken report cannot break a
scan.

Three coordinate facts, because getting any of them wrong is silent:

  * `warehouse.yaml` is written in the generator's own frame. `gen_world.py`
    rotates the whole warehouse by `world_yaw` on the way out, so a rack
    position read straight out of the yaml is not where that rack is in the
    world. The same rotation is applied here, from the same field.
  * `ground_truth.json` is already in world coordinates - it comes out the far
    side of that rotation - and so are the scanner's estimated positions. Those
    two can be compared directly.
  * Ground truth is read for MEASUREMENT only. It never feeds an estimate.
"""

from __future__ import annotations

import json
import math
import re
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent

CONFIG = REPO_ROOT / "warehouse" / "warehouse.yaml"
# The generated world, which carries each box's actual dimensions. Written by
# setup_px4.sh and gitignored, so it may not be there; everything that uses it
# degrades to "unknown" rather than failing.
WORLD_SDF = REPO_ROOT / "warehouse" / "generated" / "gz" / "worlds" / "warehouse.sdf"
LAYOUT = REPO_ROOT / "scanner" / "layout.json"
GROUND_TRUTH = REPO_ROOT / "warehouse" / "ground_truth.json"
INVENTORY = REPO_ROOT / "out" / "inventory_scanned.json"
NAV_REPORT = REPO_ROOT / "out" / "navigation_report.json"


# --------------------------------------------------------------- geometry

def load_config(path: Path = CONFIG) -> dict:
    return yaml.safe_load(Path(path).read_text())


def world_yaw_rad(cfg: dict) -> float:
    return math.radians(cfg.get("world_yaw", 0.0) or 0.0)


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    """The same rotation `gen_world.rotate_xy` applies, kept identical."""
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def unrotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    """World back into the config frame. Needed to ask which bay a point is in."""
    return rotate_xy(x, y, -yaw)


def level_span(cfg: dict, index: int) -> tuple[float, float]:
    """
    The floor and ceiling of one shelf level, in metres.

    The top level has no level above it to bound it, so the uprights do.
    """
    heights = cfg["racking"]["level_heights"]
    bottom = heights[index]
    top = (heights[index + 1] if index + 1 < len(heights)
           else cfg["racking"]["upright_height"])
    return bottom, top


def rack_cells(cfg: dict) -> list[dict]:
    """
    One wireframe box per (face, bay, level), in world coordinates.

    This is the skeleton the 3D view draws. A product that lands outside its
    cell is the thing the view exists to make obvious, so the cells have to be
    where the racks actually are, not where the yaml frame puts them.
    """
    r = cfg["racking"]
    yaw = world_yaw_rad(cfg)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    cells = []
    for row in r["rows"]:
        # The scannable surface is the face the aisle sees; the block sits
        # `depth` behind it, on the side `facing` points away from.
        face = row["y0"] + r["depth"] if row["facing"] > 0 else row["y0"]
        y_c = face - row["facing"] * r["depth"] / 2.0
        for bay in range(1, r["bay_count"] + 1):
            x_c = r["x_origin"] + (bay - 0.5) * r["bay_width"]
            wx, wy = rotate_xy(x_c, y_c, yaw)
            hx_cfg, hy_cfg = r["bay_width"] / 2.0, r["depth"] / 2.0
            for li in range(len(r["level_heights"])):
                bottom, top = level_span(cfg, li)
                cells.append({
                    "row": row["id"], "bay": bay, "level": li + 1,
                    "c": [round(wx, 3), round(wy, 3), round((bottom + top) / 2, 3)],
                    # Half-extents rotate too. For the 90 degree yaw this
                    # warehouse uses that is a swap; the general form keeps it
                    # correct if the yaw is ever changed.
                    "h": [round(c * hx_cfg + s * hy_cfg, 3),
                          round(s * hx_cfg + c * hy_cfg, 3),
                          round((top - bottom) / 2, 3)],
                })
    return cells


def bay_of(cfg: dict, x_world: float, y_world: float) -> int:
    """
    Which bay a world point falls in, 1-based, clamped to the racking.

    The scanner names a shelf face and a level but not a bay, so the bay it
    implicitly claimed has to be read back out of the position it wrote. That
    is a fair reading: the bay is where the estimate says the box is.
    """
    r = cfg["racking"]
    x_cfg, _ = unrotate_xy(x_world, y_world, world_yaw_rad(cfg))
    idx = int((x_cfg - r["x_origin"]) // r["bay_width"]) + 1
    return max(1, min(r["bay_count"], idx))


def bounds(cfg: dict, margin: float = 2.0) -> list[float]:
    """World-frame [x_min, x_max, y_min, y_max] around the racking."""
    cells = rack_cells(cfg)
    xs = [c["c"][0] for c in cells]
    ys = [c["c"][1] for c in cells]
    return [round(min(xs) - margin, 2), round(max(xs) + margin, 2),
            round(min(ys) - margin, 2), round(max(ys) + margin, 2)]


# ------------------------------------------------------------------- data

def load_ground_truth(path: Path = GROUND_TRUTH) -> dict[str, dict]:
    """The box QR labels, keyed by payload. Placards and markers are dropped."""
    codes = json.loads(Path(path).read_text())["codes"]
    return {c["payload"]: c for c in codes if c["type"] == "box_qr"}


def load_markers(path: Path = GROUND_TRUTH) -> list[dict]:
    """The floor markers, for drawing where the drift fixes came from."""
    codes = json.loads(Path(path).read_text())["codes"]
    return [c for c in codes if c["type"] == "aisle_marker"]


def load_run(path: Path = INVENTORY) -> dict:
    """
    A scan as `scanner.py` writes it.

    Raises FileNotFoundError with the command that produces the file, because
    the usual reason it is missing is that the scan has not been run yet.
    """
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(
            f"no scan output at {p}\n"
            f"run a scan first: cd scanner && python3 scanner.py")
    return json.loads(p.read_text())


def records(run: dict, cfg: dict) -> list[dict]:
    """
    Normalise the scanner's items into what the reports compare.

    The scanner writes `id` (the decoded QR payload), an estimated position, a
    shelf face and a level. Bay and product code are derived here rather than
    asked of the scanner, so this stays a pure consumer of its output and the
    scanner needs no change to be reportable.
    """
    out = []
    for it in run.get("items", []):
        x, y, z = it["estimated_x"], it["estimated_y"], it["estimated_z"]
        payload = it["id"]
        out.append({
            "qr": payload,
            # Payload is WH1|<row>|<bay>|<level>|<sku>; the product code is the
            # last field. Anything that does not split that way is kept whole,
            # which is what an unknown or misdecoded payload should look like.
            "product_id": payload.split("|")[-1] if "|" in payload else payload,
            "x": x, "y": y, "z": z,
            "shelf": it.get("shelf"),
            "level": it.get("level"),
            "bay": bay_of(cfg, x, y),
            "uav": it.get("uav_position"),
            "bearing_deg": it.get("bearing_deg"),
            "timestamp": it.get("timestamp"),
        })
    return out


def error_to_truth(rec: dict, truth: dict) -> float | None:
    """Distance from an estimate to the true label position, metres."""
    t = truth.get(rec["qr"])
    if not t:
        return None
    tx, ty, tz = t["label_pose_xyzrpy"][:3]
    return math.dist((rec["x"], rec["y"], rec["z"]), (tx, ty, tz))


def percentile(sorted_values: list[float], q: float) -> float:
    """Nearest-rank percentile of an already sorted list."""
    if not sorted_values:
        raise ValueError("empty")
    return sorted_values[min(len(sorted_values) - 1,
                             int(q * (len(sorted_values) - 1)))]


# ------------------------------------------------------- box geometry

# One box link in the generated world: its name, then the body visual's pose
# and size. Anchored on `name="body"` so the label and placard visuals in the
# same link cannot match instead.
_BOX_LINK = re.compile(
    r'<link name="(box_[^"]+)">\s*<visual name="body">\s*'
    r'<pose>([-\d.eE ]+)</pose>\s*'
    r'<geometry><box><size>([-\d.eE ]+)</size></box></geometry>',
    re.S)


def load_box_geometry(cfg: dict, path: Path = WORLD_SDF) -> dict[str, dict]:
    """
    Every box's real dimensions, keyed by the link name ground truth records.

    Read from the generated world rather than recomputed from the yaml. The
    yaml says which sizes exist and how they may be stacked; which size landed
    in which slot was a seeded random draw at generation time. Re-rolling that
    draw here would be a second implementation of the same decision, and the
    two would drift apart silently.

    The world file is written in the generator's frame, so a -90 degree
    world_yaw swaps width and depth. Sizes are returned in world orientation
    to match every other number these tools handle.

    Returns {} if the world has not been generated. Callers show "unknown"
    rather than guessing.
    """
    p = Path(path)
    if not p.exists():
        alt = Path.home() / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "worlds" / "warehouse.sdf"
        if not alt.exists():
            return {}
        p = alt

    yaw = world_yaw_rad(cfg)
    c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
    out = {}
    for name, pose, size in _BOX_LINK.findall(p.read_text()):
        px, py, pz = (float(v) for v in pose.split()[:3])
        w, d, h = (float(v) for v in size.split()[:3])
        wx, wy = rotate_xy(px, py, yaw)
        out[name] = {
            "centre": [round(wx, 3), round(wy, 3), round(pz, 3)],
            # Width across the shelf face, depth into it, height.
            "size": [round(c * w + s * d, 3), round(s * w + c * d, 3), round(h, 3)],
            "volume_m3": round(w * d * h, 4),
        }
    return out


def flight_altitudes(path: Path = LAYOUT) -> list[float]:
    """The altitude flown at each shelf level, from the scanner's layout."""
    return json.loads(Path(path).read_text())["flight_z"]


def standoffs(path: Path = LAYOUT) -> dict[str, float]:
    """
    Shelf face -> how far the camera flew from it, from the scanner's layout.

    One number for the whole building would be wrong here: the aisles taper
    from 2.40 m to 0.50 m, so the camera stands 0.80 m off the shelves on the
    wide aisles and only 0.25 m off them on the narrowest. Readability and
    what fits in the frame both scale with that distance, and a report that
    assumed 0.80 everywhere would call the narrow end far worse than it is on
    resolution and far better than it is on framing.
    """
    layout = json.loads(Path(path).read_text())
    fallback = layout.get("shelf_standoff", 0.80)
    return {f["name"]: f.get("standoff", fallback)
            for f in layout["aisle_faces"] if "name" in f}


# Half the camera's vertical field in metres, measured at the 0.80 m standoff
# of the wide aisles. Kept as the measured number rather than recomputed from
# the lens, because it is what the missed-box tally has always been scored
# against; the geometric value for 1280x720 at 60 degrees is 0.260 m, and the
# gap is real vignetting and decode margin at the frame edge.
HALF_FRAME_AT_M = (0.23, 0.80)


def half_frame_m(standoff: float) -> float:
    """
    Half the camera's vertical field, in metres, at this distance.

    A label further off the optical axis than this was never in shot. It is a
    distance, so it scales with the aisle: 0.23 m at the 0.80 m standoff of
    the wide aisles, 0.07 m at the 0.25 m of the narrowest, which is less than
    a third of the 0.25 m label stack. That is the narrow end's real problem.
    Resolution improves as the camera closes in; framing collapses.
    """
    measured, at = HALF_FRAME_AT_M
    return standoff * measured / at


def label_profile(code: dict, geometry: dict, flight_z: list[float],
                  standoff: float | None = None, frame_px: int = 1280,
                  hfov_deg: float = 60.0) -> dict:
    """
    What a box looked like to the camera: size, and how far off the axis it sat.

    The vertical offset is the one that matters. Commanded altitudes put the
    optical axis at `flight_z` for the level, and a label sitting below that is
    the failure this warehouse has already produced once: the camera's vertical
    half-frame is about 0.23 m at the 0.80 m standoff, and labels further down
    than that leave the frame entirely.

    Readability is the other half. A QR needs roughly 3 pixels per module to
    decode, and the module size travels with the label in ground truth.

    `standoff` defaults to the one the scanner actually flies for this code's
    shelf face. The aisles taper, so it is not the same for every face, and
    passing a single number for the building would misreport both halves.
    """
    if standoff is None:
        standoff = standoffs().get(code.get("row"), 0.80)
    x, y, z = code["label_pose_xyzrpy"][:3]
    level_z = flight_z[code["level"] - 1] if code["level"] - 1 < len(flight_z) else None
    px_per_module = (frame_px / 2) / math.tan(math.radians(hfov_deg) / 2) \
        * code["module_size_m"] / standoff
    g = geometry.get(code["entity"].split("::")[-1], {})
    return {
        "size": g.get("size"),
        "volume_m3": g.get("volume_m3"),
        "label_z": round(z, 3),
        "axis_z": level_z,
        # Negative means the label sat below the optical axis.
        "z_offset_m": round(z - level_z, 3) if level_z is not None else None,
        "px_per_module": round(px_per_module, 2),
        "standoff_m": round(standoff, 3),
        # Half the vertical field at that standoff; a label further off the
        # axis than this was out of shot.
        "half_frame_m": round(half_frame_m(standoff), 3),
        "label_size_m": code.get("label_size_m"),
        "position": [round(x, 3), round(y, 3), round(z, 3)],
    }
