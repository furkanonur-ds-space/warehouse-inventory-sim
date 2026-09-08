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
# The vehicle that flew, which carries the cameras' own resolution and field
# of view. Installed into the PX4 tree by setup_px4.sh, so it may not be on
# the machine reading a run; the C27 values below stand in when it is not.
GZ_MODELS = Path.home() / "PX4-Autopilot" / "Tools" / "simulation" / "gz" / "models"
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

def load_ground_truth(path: Path = GROUND_TRUTH,
                      code_type: str = "box_qr") -> dict[str, dict]:
    """
    One symbology's labels, keyed by payload. Markers are always dropped.

    A box carries two codes that say different things, and each is read by its
    own decoder and scored against its own truth. Neither can stand in for the
    other: the QR names the box, the barcode carries what the barcode carries,
    and deriving one from the other would report a reading that never happened.

    `box_qr` is the default because that is what scanner.py writes.
    `box_placard` scores the barcode inventory the same way.
    """
    codes = json.loads(Path(path).read_text())["codes"]
    return {c["payload"]: c for c in codes if c["type"] == code_type}


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


# The two cameras that read shelves, as the vehicle carries them. The route
# flies one lane per aisle and reads both its faces from it, the hires ahead
# and the rear tracking camera behind, so "how far was the camera" has two
# answers per aisle and they are not the same camera: 1024 px across 60
# degrees against 1280 across 90.
#
# Falls back to the C27 configuration when the model is not installed. These
# are the values build_c27_drone.py writes; they are stated once, here, and
# read from the model itself whenever it can be found.
CAMERA_FALLBACK = {
    "hires": {"link": "camera_hires_link", "frame_px": (1024, 768),
              "hfov_deg": 60.0, "mount_x": 0.06},
    "rear": {"link": "camera_track_rear_link", "frame_px": (1280, 800),
             "hfov_deg": 90.0, "mount_x": 0.055},
}

_CAM_LINK = re.compile(
    r'<link name="(camera_hires_link|camera_track_rear_link)">'
    r'.*?<pose>([^<]*)</pose>'
    r'.*?<horizontal_fov>([^<]*)</horizontal_fov>'
    r'.*?<width>([^<]*)</width>\s*<height>([^<]*)</height>',
    re.S)


def cameras(model: str | None = None, models_dir: Path = GZ_MODELS) -> dict:
    """
    What each reading camera is, from the model that flew.

    Read rather than copied, for the reason load_box_geometry is read: the
    resolution and the field of view are decided in build_c27_drone.py, and a
    second copy here would drift apart from it silently. The mount offset
    matters too - the lens sits ahead of or behind the vehicle centre, and the
    distance that decides what it resolves is the lens to the shelf, not the
    airframe to the shelf.
    """
    out = {name: dict(spec) for name, spec in CAMERA_FALLBACK.items()}
    if model is None:
        try:
            model = json.loads(Path(LAYOUT).read_text())["model"]
        except Exception:
            return out
    path = Path(models_dir) / model / "model.sdf"
    if not path.exists():
        return out
    by_link = {spec["link"]: name for name, spec in out.items()}
    for link, pose, fov, width, height in _CAM_LINK.findall(path.read_text()):
        name = by_link.get(link)
        if name is None:
            continue
        out[name] = {
            "link": link,
            "frame_px": (int(float(width)), int(float(height))),
            "hfov_deg": math.degrees(float(fov)),
            # Either sign means the same thing: displaced towards the face it
            # reads, so it stands that much closer than the airframe does.
            "mount_x": abs(float(pose.split()[0])),
        }
    return out


def _split_aisle(width: float, hires_max: float, rear_max: float):
    """
    How far the lane sits from each face of an aisle of this width.

    The rule is the layout's own, stated in its `_camera_reach_note`: both
    distances add up to the width and each stays inside its camera's reach, so
    the width is split in proportion to the two reaches. Returns None for an
    aisle wider than the two together, which is flown as one pass per face
    with the hires alone.
    """
    total = hires_max + rear_max
    if total <= 0:
        return None
    hires = width * hires_max / total
    rear = width - hires
    if rear > rear_max:
        rear, hires = rear_max, width - rear_max
    if hires > hires_max:
        return None
    return hires, rear


def face_cameras(path: Path = LAYOUT, model_cameras: dict | None = None) -> dict:
    """
    Shelf face -> which camera read it and how far its lens stood from the
    codes on it.

    One number for the whole building would be wrong twice over. The aisles
    taper from 2.40 m to 0.50 m, and since the rear camera started reading the
    face behind, the two faces of one aisle are read from different distances
    by different cameras. A report that assumed a single standoff and a single
    camera overstated the resolution in the widest aisle by about two times,
    which is where reading is hardest.

    Faces are paired the way the route pairs them: a face is across the aisle
    when it looks the other way and sits ahead of the vehicle, and the nearest
    such face is the one the rear camera sees. The face reached first is the
    one the hires reads, which is the order the route walks them in.
    """
    layout = json.loads(Path(path).read_text())
    faces = [f for f in layout["aisle_faces"] if "name" in f]
    cams = model_cameras if model_cameras is not None else cameras()
    hires_max = layout.get("hires_max_standoff", 1.30)
    rear_max = layout.get("rear_max_standoff", 1.10)
    alone = min(layout.get("shelf_standoff", hires_max), hires_max)
    # The lane is placed relative to face_x, which names the shelf surface.
    # The codes are not on it: a label is mounted on a box standing behind
    # that surface, so the plane they sit in is this much further from the
    # aisle, and that much further from the lens than the standoff says.
    code_plane = layout.get("code_plane_offset_m", 0.0)

    def facing(face):
        ahead = 1.0 if face["yaw_deg"] < 0 else -1.0
        candidates = [o for o in faces
                      if o is not face
                      and o["yaw_deg"] * face["yaw_deg"] < 0
                      and (o["face_x"] - face["face_x"]) * ahead > 0]
        if not candidates:
            return None
        return min(candidates, key=lambda f: abs(f["face_x"] - face["face_x"]))

    out = {}
    done = set()
    for face in faces:
        if face["name"] in done:
            continue
        opposite = facing(face)
        pairs = []
        if opposite is None:
            pairs = [(face, "hires", alone)]
        else:
            width = abs(opposite["face_x"] - face["face_x"])
            split = _split_aisle(width, hires_max, rear_max)
            if split is None:
                pairs = [(face, "hires", alone), (opposite, "hires", alone)]
            else:
                pairs = [(face, "hires", split[0]),
                         (opposite, "rear", split[1])]
        for one, which, depth in pairs:
            spec = cams[which]
            out[one["name"]] = {
                "camera": which,
                # Lens to the plane the codes are on, which is what decides
                # both numbers below. Neither the airframe nor the shelf
                # surface is the right end of that measurement.
                "distance_m": round(
                    max(depth - spec["mount_x"] + code_plane, 0.01), 3),
                "frame_px": spec["frame_px"],
                "hfov_deg": spec["hfov_deg"],
            }
            done.add(one["name"])
    return out


def standoffs(path: Path = LAYOUT) -> dict[str, float]:
    """Shelf face -> how far the camera's lens flew from the codes on it."""
    return {name: cam["distance_m"] for name, cam in face_cameras(path).items()}


# Half the camera's vertical field in metres, measured on the hires camera at
# the 0.80 m standoff the wide aisles were flown at before the rear camera
# read the far face. Kept as the measured number rather than the geometric
# one: the geometric half-frame there is 0.346 m, and the gap is real
# vignetting and decode margin at the edge of the frame.
HALF_FRAME_AT_M = (0.23, 0.80)


def _frame_ratio(camera: dict) -> float:
    """Half the vertical field per metre of distance, from the lens alone."""
    width, height = camera["frame_px"]
    return (height / width) * math.tan(math.radians(camera["hfov_deg"]) / 2)


def half_frame_m(standoff: float, camera: dict | None = None) -> float:
    """
    Half the camera's vertical field, in metres, at this distance.

    A label further off the optical axis than this was never in shot. It is a
    distance, so it scales with the aisle: at the narrow end the camera closes
    in and resolution improves while framing collapses, which is that end's
    real problem.

    The measured anchor is a hires number, so it is carried across to the rear
    camera as the fraction of the geometric field it represents rather than as
    a fraction of the distance. The two lenses do not see the same slice: 60
    degrees over a 4:3 frame against 90 over 16:10.
    """
    measured, at = HALF_FRAME_AT_M
    cams = CAMERA_FALLBACK
    hires_ratio = _frame_ratio(cams["hires"])
    usable = measured / (at * hires_ratio)
    ratio = _frame_ratio(camera) if camera is not None else hires_ratio
    return standoff * ratio * usable


def label_profile(code: dict, geometry: dict, flight_z: list[float],
                  standoff: float | None = None, frame_px: int | None = None,
                  hfov_deg: float | None = None) -> dict:
    """
    What a box looked like to the camera: size, and how far off the axis it sat.

    The vertical offset is the one that matters. Commanded altitudes put the
    optical axis at `flight_z` for the level, and a label sitting below that is
    the failure this warehouse has already produced once: the camera's vertical
    half-frame is about 0.23 m at the 0.80 m standoff, and labels further down
    than that leave the frame entirely.

    Readability is the other half. A QR needs roughly 3 pixels per module to
    decode, and the module size travels with the label in ground truth.

    The camera, the distance and the frame all default to the ones that read
    this code's shelf face. None of the three is a property of the building:
    the aisles taper, and the two faces of one aisle are read from different
    distances by different cameras. Assuming one standoff and one 1280 px
    frame for all of them, which this did, overstated the resolution in the
    widest aisle by about two times.
    """
    face = face_cameras().get(code.get("row"))
    if face is None:
        face = {"camera": "hires", "distance_m": 0.80,
                "frame_px": CAMERA_FALLBACK["hires"]["frame_px"],
                "hfov_deg": CAMERA_FALLBACK["hires"]["hfov_deg"]}
    if standoff is None:
        standoff = face["distance_m"]
    width_px = frame_px if frame_px is not None else face["frame_px"][0]
    fov = hfov_deg if hfov_deg is not None else face["hfov_deg"]
    x, y, z = code["label_pose_xyzrpy"][:3]
    level_z = flight_z[code["level"] - 1] if code["level"] - 1 < len(flight_z) else None
    px_per_module = (width_px / 2) / math.tan(math.radians(fov) / 2) \
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
        # Which of the two cameras read this face, since neither the distance
        # nor the frame means anything without it.
        "camera": face["camera"],
        # Half the vertical field at that distance; a label further off the
        # axis than this was out of shot.
        "half_frame_m": round(half_frame_m(standoff, face), 3),
        "label_size_m": code.get("label_size_m"),
        "position": [round(x, 3), round(y, 3), round(z, 3)],
    }
