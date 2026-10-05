#!/usr/bin/env python3
"""
Check that light_report.py splits a flight by the light each label got.

    .venv/bin/python report/test_light_report.py

No simulator. The dimmed world is built in memory and a run is invented over
it in which a QR reads exactly when its label gets E >= 0.5: every band below
has to come back at 0 % and every band above at 100 %.

Then the shadowed world, with a run that reads only the labels in full light:
the shade split has to come back 100 / 0 / 0, every label in exactly one
class. And a reader's summary and readings invented with sim time running at
half the wall clock: the camera line has to say RTF 0.5 and the frames given.
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
from light_report import SHADE_CLASSES, build, shade_of  # noqa: E402


def main() -> int:
    cfg = copy.deepcopy(gl._load_cfg(HERE.parent / "warehouse" / "warehouse.yaml"))
    cfg["stress"]["enabled"] = False
    cfg["lights_stress"]["enabled"] = True
    with contextlib.redirect_stdout(io.StringIO()):
        _, manifest, _ = gw.build(cfg)
    read = [{"id": c["payload"]} for c in manifest
            if c["type"] == "box_qr" and c["light"]["E"] >= 0.5]
    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        (out / "gt.json").write_text(json.dumps({"codes": manifest}))
        (out / "inventory_scanned.json").write_text(json.dumps({"items": read}))
        data = build(out / "gt.json", out)
    for r in data["rows"]:
        lo = float(r["band"].split("-")[0])
        a, n = r["qr"]
        want = n if lo >= 0.5 else 0
        if a != want:
            failures.append(f"band {r['band']}: {a}/{n} read, want {want}")
    total = sum(r["qr"][1] for r in data["rows"])
    if total != sum(1 for c in manifest if c["type"] == "box_qr"):
        failures.append(f"{total} QRs in the bands")
    # Shadows.
    cfg = copy.deepcopy(gl._load_cfg(HERE.parent / "warehouse" / "warehouse.yaml"))
    cfg["stress"]["enabled"] = False
    cfg["lights_stress"]["enabled"] = False
    cfg["shadows_stress"] = {"enabled": True, "lamps": "all"}
    with contextlib.redirect_stdout(io.StringIO()):
        _, man_sh, _ = gw.build(cfg)
    lit = [{"id": c["payload"]} for c in man_sh
           if c["type"] == "box_qr" and shade_of(c["light"]) == "full light"]
    with tempfile.TemporaryDirectory() as tmp:
        out = Path(tmp)
        (out / "gt.json").write_text(json.dumps({"codes": man_sh}))
        (out / "inventory_scanned.json").write_text(json.dumps({"items": lit}))
        (out / "barcode_inventory_front.json").write_text(json.dumps(
            {"frames": {"arrived": 100, "decoded": 90, "shed_queue_full": 10,
                        "published_gap_ms": {"median": 50.0}}}))
        (out / "barcode_readings_front.jsonl").write_text(
            json.dumps({"t": "2026-10-05T10:00:00", "sim_t": 10.0}) + "\n"
            + json.dumps({"t": "2026-10-05T10:01:40", "sim_t": 60.0}) + "\n")
        sh = build(out / "gt.json", out)
    shade = sh.get("shade") or {}
    n_qr = sum(1 for c in man_sh if c["type"] == "box_qr")
    if sum(shade.get(k, {}).get("qr", [0, 0])[1] for k in SHADE_CLASSES) != n_qr:
        failures.append(f"shade classes do not hold every QR once: {shade}")
    for k in SHADE_CLASSES:
        a, n = shade.get(k, {}).get("qr", [0, 0])
        if n == 0:
            failures.append(f"no QR in '{k}'")
        if a != (n if k == "full light" else 0):
            failures.append(f"'{k}': {a}/{n} read")
    # Dimmed AND shadowed: a label with nothing in its way is in full light,
    # however far the dimming took it below the clean world.
    if shade_of({"E": 0.5, "E_open": 0.5, "rel": 0.2, "E_min": 0.5, "E_max": 0.5}) != "full light":
        failures.append("a dimmed, unshadowed label is not in full light")
    if shade_of({"E": 0.2, "E_open": 0.5, "rel": 0.08, "E_min": 0.2, "E_max": 0.2}) != "shade":
        failures.append("a shadowed label in a dimmed world is not in shade")
    cam = sh.get("camera") or [{}]
    if cam[0].get("rtf") != 0.5 or cam[0].get("arrived") != 100 or cam[0].get("shed") != 10:
        failures.append(f"camera line {cam}")
    print(f"{len(data['rows'])} bands, {total} QRs; shade "
          + " / ".join(f"{k} {shade[k]['qr'][1]}" for k in SHADE_CLASSES if k in shade)
          + f"; camera RTF {cam[0].get('rtf')}")
    for f in failures:
        print("  FAIL:", f)
    print("all checks passed" if not failures else f"{len(failures)} FAILED")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
