#!/usr/bin/env python3
"""
What each disturbance cost: one flight over the stressed warehouse, split by
what was done to each carton.

    .venv/bin/python report/stress_report.py [--out-dir out]

The stressed world (warehouse/stress.py, `stress:` in warehouse.yaml) gives
every carton at most one disturbance - pushed in, pulled out, slid, turned,
tilted, or taken away - and writes which into ground truth. The cartons left
alone (`none`) fly in the same flight and are the control. So a row here is
an experiment of its own, and its numbers mean something only next to the
`none` row above it: a face that reads badly reads badly for every row.

Three readers, each scored on its own:
  * QR       out/inventory_scanned.json   matched by payload
  * barcode  out/inventory_barcode.json   matched by payload
  * carton   out/box_report.json          the YOLO carton a cluster was matched to

ERROR IS SPLIT, BECAUSE DEPTH IS NOT MEASURED. The scanner and the barcode
inventory file every code ON THE SHELF FACE: their x is the face plane, not a
range to the label. A carton pushed 0.25 m into the rack therefore comes out
0.25 m wrong in depth whatever the reader does, and folded into one distance
that would look like the reader failing. In-plane (along the run and in
height) is the error a reader can be blamed for; depth is reported beside it
so the push_in rows say how far the face-plane assumption is from the truth.

OLD LABELS (`decoy`) are codes that read perfectly and say the wrong thing:
a QR with another address, a barcode with a number the warehouse does not
have. Every one that turns up in an inventory is a false record, so for
those rows the numbers are how many were read and how many of those the
stale-label rules then caught (report/stale_labels.py for the QR,
barcode_inventory.stale_barcodes() for the barcode) - next to whether the
real labels above them still read.

EMPTY SLOTS have nothing to read. What they can show is a phantom: a carton
cluster the detector placed where no carton is. Counted from every cluster in
box_report.json, matched or not, on the same face and shelf level within
PHANTOM_M of the empty spot ALONG THE RUN. Height is left out on purpose: the
spot is the middle of a carton that is not there, and a cluster is placed at
the middle of whatever was boxed, which is not the same height. Judged in
the full plane the count moved with every fix to the height and said nothing
about phantoms.

Reads only: ground truth and the JSON a run leaves in out/. Nothing from
scanner/ is imported.
"""
from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from warehouse_model import (GROUND_TRUTH, REPO_ROOT, load_config,  # noqa: E402
                             rotate_xy, world_yaw_rad)

OUT = REPO_ROOT / "out"

#: How near a cluster must sit to an empty spot, along the run, to be its phantom.
#: Half the narrowest carton (XS, 0.32 m): nearer than that and the cluster is
#: on the empty spot, not on the neighbour beside it.
PHANTOM_M = 0.16

ARM_ORDER = ("none", "push_in", "pull_out", "slide", "yaw", "tilt", "empty",
             "fade", "smudge", "tear", "wrinkle", "skew", "decoy")


def cell_key(tag: dict) -> tuple:
    return (ARM_ORDER.index(tag["arm"]) if tag["arm"] in ARM_ORDER else 99,
            tag["level"])


def cell_name(tag: dict) -> str:
    if tag["arm"] in ("none", "empty"):
        return tag["arm"]
    return f"{tag['arm']} L{tag['level']}"


def load(path: Path) -> dict | None:
    return json.loads(path.read_text()) if path.exists() else None


def by_id(inventory: dict | None) -> dict[str, dict]:
    """First item per id. A duplicate is a scanner matter, scored elsewhere."""
    out: dict[str, dict] = {}
    for item in (inventory or {}).get("items", []):
        out.setdefault(str(item.get("id")), item)
    return out


def face_axes(cfg: dict) -> tuple[tuple, tuple]:
    """World unit vectors across the face (depth) and along the run.

    In the generator's frame faces are planes of constant y and the run is x;
    the world is that frame turned by world_yaw.
    """
    yaw = world_yaw_rad(cfg)
    return rotate_xy(0.0, 1.0, yaw), rotate_xy(1.0, 0.0, yaw)


def split_error(item: dict, truth_xyz, depth_ax, run_ax) -> tuple[float, float]:
    """(in-plane, depth) distance from a reported position to the truth."""
    try:
        d = (float(item["estimated_x"]) - truth_xyz[0],
             float(item["estimated_y"]) - truth_xyz[1],
             float(item["estimated_z"]) - truth_xyz[2])
    except (KeyError, TypeError, ValueError):
        return math.nan, math.nan
    depth = abs(d[0] * depth_ax[0] + d[1] * depth_ax[1])
    along = d[0] * run_ax[0] + d[1] * run_ax[1]
    return math.hypot(along, d[2]), depth


def median(xs) -> float | None:
    xs = [x for x in xs if not math.isnan(x)]
    return round(statistics.median(xs), 3) if xs else None


def build(truth_path: Path, out_dir: Path, cfg: dict) -> dict:
    codes = json.loads(truth_path.read_text())["codes"]
    stressed = [c for c in codes if "stress" in c]
    if not stressed:
        return {"stressed": False}

    qr = by_id(load(out_dir / "inventory_scanned.json"))
    bc_inv = load(out_dir / "inventory_barcode.json")
    bc = by_id(bc_inv)
    # Read but left out as an old label. The barcode inventory drops them
    # itself; the QR inventory is the scanner's and keeps them, the checked
    # copy beside it does not.
    stale_bc = {s["id"] for s in (bc_inv or {}).get("stale_labels", [])}
    stale_qr = {s["id"] for s in (load(out_dir / "stale_labels_qr.json") or {}).get("stale", [])}
    box_report = load(out_dir / "box_report.json")
    cartons = by_id(box_report)
    clusters = (box_report or {}).get("items", [])
    have = {"qr": bool(qr), "barcode": bool(bc), "carton": box_report is not None}
    depth_ax, run_ax = face_axes(cfg)

    # One record per carton, keyed by entity: its cell, face, and what each
    # reader made of it.
    boxes: dict[str, dict] = {}
    for c in stressed:
        b = boxes.setdefault(c["entity"], {
            "entity": c["entity"], "row": c["row"], "cell": c["stress"],
            "kind": None, "qr": None, "barcode": None, "carton": None})
        xyz = c["label_pose_xyzrpy"][:3]
        if c["type"] == "box_qr":
            b["kind"] = "labelled"
            hit = qr.get(c["payload"])
            b["qr"] = {"read": hit is not None,
                       "err": split_error(hit, xyz, depth_ax, run_ax) if hit else None}
            hit = cartons.get(c["payload"])
            b["carton"] = {"found": hit is not None}
        elif c["type"] == "box_placard":
            hit = bc.get(c["payload"])
            b["barcode"] = {"read": hit is not None,
                            "err": split_error(hit, xyz, depth_ax, run_ax) if hit else None}
        elif c["type"] == "box_unlabelled":
            b["kind"] = "unlabelled"
            b["carton"] = {"found": c["payload"] in cartons}
        elif c["type"] == "box_decoy":
            inv, caught = (qr, stale_qr) if c["symbology"] == "QR" else (bc, stale_bc)
            p = c["payload"]
            b.setdefault("decoys", []).append({
                "symbology": c["symbology"], "payload": p,
                "read": p in inv or p in caught, "caught": p in caught})
        elif c["type"] == "box_absent":
            b["kind"] = "absent"
            near = []
            for k in clusters:
                if k.get("shelf") != c["row"] or k.get("level") != c["level"]:
                    continue
                try:
                    along = ((float(k["estimated_x"]) - xyz[0]) * run_ax[0]
                             + (float(k["estimated_y"]) - xyz[1]) * run_ax[1])
                except (KeyError, TypeError, ValueError):
                    continue
                if abs(along) <= PHANTOM_M:
                    near.append(k.get("id"))
            b["phantoms"] = near

    rows: dict[tuple, dict] = {}
    for b in boxes.values():
        key = cell_key(b["cell"])
        r = rows.setdefault(key, {
            "cell": cell_name(b["cell"]), "arm": b["cell"]["arm"],
            "level": b["cell"]["level"], "unit": b["cell"]["unit"],
            "values": set(), "cartons": 0, "unlabelled": 0, "absent": 0,
            "faces": {}, "qr_read": 0, "qr_of": 0, "bc_read": 0, "bc_of": 0,
            "carton_found": 0, "carton_of": 0, "phantom_slots": 0,
            "decoy_codes": 0, "decoy_read": 0, "decoy_caught": 0,
            "_qr_in": [], "_qr_depth": [], "_bc_in": [], "_bc_depth": []})
        r["values"].add(abs(b["cell"]["value"]))
        f = r["faces"].setdefault(b["row"], {"n": 0, "qr": 0, "bc": 0, "carton": 0})
        f["n"] += 1
        if b["kind"] == "absent":
            r["absent"] += 1
            r["phantom_slots"] += bool(b.get("phantoms"))
            continue
        r["cartons"] += 1
        r["unlabelled"] += b["kind"] == "unlabelled"
        for d in b.get("decoys", []):
            r["decoy_codes"] += 1
            r["decoy_read"] += d["read"]
            r["decoy_caught"] += d["caught"]
        if b["qr"]:
            r["qr_of"] += 1
            if b["qr"]["read"]:
                r["qr_read"] += 1
                f["qr"] += 1
                r["_qr_in"].append(b["qr"]["err"][0])
                r["_qr_depth"].append(b["qr"]["err"][1])
        if b["barcode"]:
            r["bc_of"] += 1
            if b["barcode"]["read"]:
                r["bc_read"] += 1
                f["bc"] += 1
                r["_bc_in"].append(b["barcode"]["err"][0])
                r["_bc_depth"].append(b["barcode"]["err"][1])
        if b["carton"]:
            r["carton_of"] += 1
            if b["carton"]["found"]:
                r["carton_found"] += 1
                f["carton"] += 1

    table = []
    for key in sorted(rows):
        r = rows[key]
        r["values"] = sorted(r["values"])
        r["qr_inplane_median_m"] = median(r.pop("_qr_in"))
        r["qr_depth_median_m"] = median(r.pop("_qr_depth"))
        r["barcode_inplane_median_m"] = median(r.pop("_bc_in"))
        r["barcode_depth_median_m"] = median(r.pop("_bc_depth"))
        table.append(r)
    phantoms = [{"entity": b["entity"], "clusters": b["phantoms"]}
                for b in boxes.values() if b.get("phantoms")]
    decoys_read = [{"entity": b["entity"], **d} for b in boxes.values()
                   for d in b.get("decoys", []) if d["read"]]
    return {"stressed": True, "readers": have, "phantom_m": PHANTOM_M,
            "rows": table, "phantoms": phantoms, "decoys_read": decoys_read,
            "boxes": sorted(boxes.values(), key=lambda b: b["entity"])}


def pct(a: int, n: int) -> str:
    return f"{a:>3}/{n:<3} {100 * a / n:5.1f}%" if n else f"{'-':>14}"


def mm(v) -> str:
    return f"{v * 1000:5.0f}" if v is not None else "    -"


def render(data: dict) -> str:
    if not data.get("stressed"):
        return ("this ground truth carries no `stress` tags: the world was built "
                "with stress.enabled false, nothing to split")
    have = data["readers"]
    lines = ["what each disturbance cost  (in-plane / depth error, median mm)", ""]
    head = f"{'cell':<12}{'value':>10} {'n':>4}  {'QR':>14} {'in':>5} {'dep':>5}  " \
           f"{'barcode':>14} {'in':>5} {'dep':>5}  {'carton':>14}"
    lines += [head, "-" * len(head)]
    for r in data["rows"]:
        vals = ",".join(f"{v:g}" for v in r["values"]) if r["unit"] else ""
        value = f"{vals} {r['unit']}".strip()
        if r["arm"] == "empty":
            lines.append(f"{r['cell']:<12}{value:>10} {r['absent']:>4}  "
                         f"phantom cartons on {r['phantom_slots']} of {r['absent']} empty slots")
            continue
        lines.append(
            f"{r['cell']:<12}{value:>10} {r['cartons']:>4}  "
            f"{pct(r['qr_read'], r['qr_of']) if have['qr'] else 'no file':>14} "
            f"{mm(r['qr_inplane_median_m'])} {mm(r['qr_depth_median_m'])}  "
            f"{pct(r['bc_read'], r['bc_of']) if have['barcode'] else 'no file':>14} "
            f"{mm(r['barcode_inplane_median_m'])} {mm(r['barcode_depth_median_m'])}  "
            f"{pct(r['carton_found'], r['carton_of']) if have['carton'] else 'no file':>14}")
        if r["decoy_codes"]:
            lines.append(f"{'':<12}{'':>10} {'':>4}  old labels read: {r['decoy_read']} of "
                         f"{r['decoy_codes']}, caught as stale: {r['decoy_caught']}, "
                         f"left as false records: {r['decoy_read'] - r['decoy_caught']}")
    lines.append("")
    lines.append("compare every row with `none`: that is the same flight, the same "
                 "faces, nothing done to the carton.")
    lines.append("depth is the face-plane assumption, not the reader: codes are "
                 "filed on the shelf face.")
    misses = [b for b in data["boxes"] if b["kind"] == "labelled" and b["cell"]["arm"] != "none"
              and ((b["qr"] and not b["qr"]["read"]) or (b["barcode"] and not b["barcode"]["read"]))]
    if misses:
        lines.append("")
        lines.append(f"disturbed cartons with a code unread ({len(misses)}):")
        for b in misses[:30]:
            what = [k for k in ("qr", "barcode") if b[k] and not b[k]["read"]]
            lines.append(f"  {b['entity'].split('::')[-1]:<18} {cell_name(b['cell']):<12} "
                         f"{b['cell']['value']:>+7g} {b['cell']['unit']:<3} unread: {'+'.join(what)}")
        if len(misses) > 30:
            lines.append(f"  ... and {len(misses) - 30} more in the JSON")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--out-dir", type=Path, default=OUT)
    ap.add_argument("--json", type=Path, default=None,
                    help="default <out-dir>/stress_report.json")
    args = ap.parse_args()
    data = build(args.ground_truth, args.out_dir, load_config())
    print(render(data))
    if data.get("stressed"):
        path = args.json or args.out_dir / "stress_report.json"
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        print(f"\nwritten: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
