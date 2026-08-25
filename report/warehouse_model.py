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
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent

CONFIG = REPO_ROOT / "warehouse" / "warehouse.yaml"
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
