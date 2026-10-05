#!/usr/bin/env python3
"""
Check that light_report.py splits a flight by the light each label got.

    .venv/bin/python report/test_light_report.py

No simulator. The dimmed world is built in memory and a run is invented over
it in which a QR reads exactly when its label gets E >= 0.5: every band below
has to come back at 0 % and every band above at 100 %.
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
from light_report import build                            # noqa: E402


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
    print(f"{len(data['rows'])} bands, {total} QRs")
    for f in failures:
        print("  FAIL:", f)
    print("all checks passed" if not failures else f"{len(failures)} FAILED")
    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
