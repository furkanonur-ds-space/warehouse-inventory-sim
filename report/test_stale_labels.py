#!/usr/bin/env python3
"""
Check that both old-label rules take out the old labels and nothing else.

    .venv/bin/python report/test_stale_labels.py

No simulator and no flight. The label-defect world is built in memory and a
run is invented over it: every real QR filed where its carton stands, every
old QR filed where IT was read - on its carton, which is not the place it
names - and barcode readings linked the way the reader links them.

  * report/stale_labels.py has to remove exactly the old QRs.
  * barcode_inventory.stale_barcodes() has to remove exactly the old barcodes
    whose carton's own barcode also read, keep a real barcode that also
    linked, further off, to a neighbour's QR, and - its stated limit - keep an
    old barcode on a carton whose own barcode never read.
"""
from __future__ import annotations

import contextlib
import copy
import io
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "warehouse"))

import gen_labels as gl                                   # noqa: E402
import gen_world as gw                                    # noqa: E402
import stress as sx                                       # noqa: E402
from barcode_inventory import stale_barcodes              # noqa: E402
from stale_labels import check                            # noqa: E402
from warehouse_model import load_config                   # noqa: E402

failures: list[str] = []


def ok(cond: bool, what: str) -> None:
    if not cond:
        failures.append(what)


def item(payload: str, carton: dict) -> dict:
    """A QR filed where `carton`'s QR is, as the scanner files it."""
    x, y, z = carton["label_pose_xyzrpy"][:3]
    return {"id": payload, "shelf": carton["row"], "level": carton["level"],
            "estimated_x": x, "estimated_y": y, "estimated_z": z}


def main() -> int:
    cfg = copy.deepcopy(gl._load_cfg(HERE.parent / "warehouse" / "warehouse.yaml"))
    cfg["stress"]["enabled"] = True
    cfg["stress"]["use"] = list(sx.LABEL_ARMS)
    cfg["stress"].pop("levels", None)   # every level, whatever the working copy flies
    with contextlib.redirect_stdout(io.StringIO()):
        _, manifest, _ = gw.build(cfg)
    qr_of = {c["entity"]: c for c in manifest if c["type"] == "box_qr"}
    bar_of = {c["entity"]: c for c in manifest if c["type"] == "box_placard"}
    old = [c for c in manifest if c["type"] == "box_decoy"]
    old_qr = [c for c in old if c["symbology"] == "QR"]
    old_bar = [c for c in old if c["symbology"] == "CODE128"]
    ok(old_qr and old_bar, "the label world built no old labels")

    # ---- QR: the place a code names against the place it was read
    items = [item(c["payload"], c) for c in qr_of.values()]
    items += [item(c["payload"], qr_of[c["entity"]]) for c in old_qr]
    kept, stale = check({"items": items}, load_config())
    got = {s["id"] for s in stale}
    want = {c["payload"] for c in old_qr}
    ok(got == want, f"QR: took out {len(got)}, want the {len(want)} old ones; "
       f"wrong {sorted(got - want)[:3]}, missed {sorted(want - got)[:3]}")
    ok(len(kept) == len(qr_of), f"QR: kept {len(kept)} of {len(qr_of)} real")

    # ---- barcode: nearest under each QR
    readings = []
    for e, bar in bar_of.items():
        readings.append({"symbology": "CODE128", "payload": bar["payload"],
                         "linked_qr": qr_of[e]["payload"], "link_error_m": 0.004})
    # A real barcode that also linked, further off, to its neighbour's QR.
    entities = sorted(bar_of)
    a, b = entities[0], entities[1]
    readings.append({"symbology": "CODE128", "payload": bar_of[a]["payload"],
                     "linked_qr": qr_of[b]["payload"], "link_error_m": 0.10})
    # The limit: one carton's own barcode never read.
    unread = old_bar[0]["entity"]
    readings = [r for r in readings
                if not (r["payload"] == bar_of[unread]["payload"])]
    for c in old_bar:
        readings.append({"symbology": "CODE128", "payload": c["payload"],
                         "linked_qr": qr_of[c["entity"]]["payload"], "link_error_m": 0.037})
    got = set(stale_barcodes(readings))
    want = {c["payload"] for c in old_bar if c["entity"] != unread}
    ok(got == want, f"barcode: took out {len(got)}, want {len(want)}; "
       f"wrong {sorted(got - want)[:3]}, missed {sorted(want - got)[:3]}")
    ok(bar_of[a]["payload"] not in got, "barcode: a real code was taken out for "
       "also linking to a neighbour")

    print(f"{len(old_qr)} old QRs, {len(old_bar)} old barcodes, "
          f"{len(qr_of)} real cartons")
    if failures:
        for f in failures:
            print("  FAIL:", f)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
