#!/usr/bin/env python3
"""
Check that stress_report.py splits a flight the way the world was built.

    .venv/bin/python report/test_stress_report.py

No simulator and no flight. The stressed world is generated in memory, and a
run is invented over it whose answer is known in advance: every code is
reported 30 mm along the run from where it is, except that the heavily
turned cartons lose their QR, the heavily tilted ones are never found as
cartons, and one cluster is planted on one empty slot. The report has to
give back exactly those numbers, and nothing for the rows that were left
alone.
"""
from __future__ import annotations

import contextlib
import copy
import io
import json
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "warehouse"))

import gen_labels as gl                                   # noqa: E402
import gen_world as gw                                    # noqa: E402
import stress as sx                                       # noqa: E402
from stress_report import build, face_axes, render        # noqa: E402
from warehouse_model import load_config                   # noqa: E402

SHIFT = 0.030
failures: list[str] = []


def check(ok: bool, what: str) -> None:
    if not ok:
        failures.append(what)


def item(c: dict, run_ax, ident=None, shelf=None) -> dict:
    x, y, z = c["label_pose_xyzrpy"][:3]
    return {"id": ident or c["payload"], "shelf": shelf or c["row"], "level": c["level"],
            "estimated_x": x + SHIFT * run_ax[0],
            "estimated_y": y + SHIFT * run_ax[1], "estimated_z": z}


def check_decoys(cfg: dict, run_ax) -> None:
    """The label group: every old QR is reported as read, no old barcode is.

    The report must count exactly the QR ones against their rows and none of
    the barcodes, and must not mistake an old label for the real one.
    """
    cfg = copy.deepcopy(cfg)
    cfg["stress"]["use"] = list(sx.LABEL_ARMS)
    cfg["stress"].pop("levels", None)   # every level, whatever the working copy flies
    with contextlib.redirect_stdout(io.StringIO()):
        _, manifest, _ = gw.build(cfg)
    real = [c for c in manifest if c["type"] == "box_qr"]
    old = [c for c in manifest if c["type"] == "box_decoy"]
    qr = [item(c, run_ax) for c in real] + \
         [item(c, run_ax) for c in old if c["symbology"] == "QR"]
    bc = [item(c, run_ax) for c in manifest if c["type"] == "box_placard"]
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        (out / "gt.json").write_text(json.dumps({"codes": manifest}))
        (out / "inventory_scanned.json").write_text(json.dumps({"items": qr}))
        (out / "inventory_barcode.json").write_text(json.dumps({"items": bc}))
        data = build(out / "gt.json", out, load_config())
    want = {}
    for c in old:
        r = want.setdefault(f"decoy L{c['stress']['level']}", [0, 0])
        r[0] += 1
        r[1] += c["symbology"] == "QR"
    rows = {r["cell"]: r for r in data["rows"]}
    check(bool(want), "label group built no old labels")
    for name, (n, n_qr) in want.items():
        r = rows.get(name, {})
        check(r.get("decoy_codes") == n, f"{name}: {r.get('decoy_codes')} old labels, want {n}")
        check(r.get("decoy_read") == n_qr, f"{name}: {r.get('decoy_read')} read, want {n_qr}")
        check(r.get("qr_read") == r.get("qr_of"), f"{name}: an old label cost the real QR")
    check(len(data["decoys_read"]) == sum(v[1] for v in want.values()), "decoys_read list")
    for name in ("none", "fade L1", "skew L3"):
        check(rows.get(name, {}).get("decoy_codes") == 0, f"{name}: counted old labels")
    render(data)


def main() -> int:
    cfg = gl._load_cfg(HERE.parent / "warehouse" / "warehouse.yaml")
    cfg = copy.deepcopy(cfg)
    cfg["stress"]["enabled"] = True
    cfg["stress"]["use"] = list(sx.GEOMETRY_ARMS)
    cfg["stress"].pop("levels", None)   # every level, whatever the working copy flies
    with contextlib.redirect_stdout(io.StringIO()):
        _, manifest, _ = gw.build(cfg)
    _, run_ax = face_axes(load_config())

    arm = lambda c: (c["stress"]["arm"], c["stress"]["level"])   # noqa: E731
    qr = [item(c, run_ax) for c in manifest
          if c["type"] == "box_qr" and arm(c) != ("yaw", 3)]
    bc = [item(c, run_ax) for c in manifest if c["type"] == "box_placard"]
    cartons = [item(c, run_ax) for c in manifest
               if c["type"] in ("box_qr", "box_unlabelled") and arm(c) != ("tilt", 3)]
    absent = [c for c in manifest if c["type"] == "box_absent"]
    cartons.append(item(absent[0], run_ax, ident="unmatched_0"))

    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        (out / "gt.json").write_text(json.dumps({"codes": manifest}))
        (out / "inventory_scanned.json").write_text(json.dumps({"items": qr}))
        (out / "inventory_barcode.json").write_text(json.dumps({"items": bc}))
        (out / "box_report.json").write_text(json.dumps({"items": cartons}))
        data = build(out / "gt.json", out, load_config())

    rows = {r["cell"]: r for r in data["rows"]}
    for name, r in rows.items():
        if name == "empty":
            check(r["absent"] == len(absent), "empty: wrong slot count")
            check(r["phantom_slots"] == 1, f"empty: {r['phantom_slots']} phantom slots, want 1")
            continue
        want_qr = 0 if name == "yaw L3" else r["qr_of"]
        check(r["qr_read"] == want_qr, f"{name}: QR {r['qr_read']}/{r['qr_of']}, want {want_qr}")
        check(r["bc_read"] == r["bc_of"], f"{name}: barcode {r['bc_read']}/{r['bc_of']}")
        want_c = 0 if name == "tilt L3" else r["carton_of"]
        check(r["carton_found"] == want_c, f"{name}: carton {r['carton_found']}, want {want_c}")
        if r["qr_inplane_median_m"] is not None:
            check(abs(r["qr_inplane_median_m"] - SHIFT) < 1e-3,
                  f"{name}: in-plane {r['qr_inplane_median_m']}, want {SHIFT}")
            check(r["qr_depth_median_m"] < 1e-3, f"{name}: depth {r['qr_depth_median_m']}, want 0")
    n_cartons = sum(r["cartons"] for r in data["rows"])
    check(n_cartons + len(absent) == 432, f"{n_cartons} + {len(absent)} cartons, want 432")
    check("none" in rows and rows["none"]["cartons"] > 100, "no control row")
    render(data)                       # must not raise
    check_decoys(cfg, run_ax)

    print(f"{len(data['rows'])} rows, {n_cartons} cartons, {len(absent)} empty slots")
    if failures:
        for f in failures:
            print("  FAIL:", f)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
