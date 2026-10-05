#!/usr/bin/env python3
"""
QR codes that say they belong somewhere other than where they were read.

    .venv/bin/python report/stale_labels.py [--inventory out/inventory_scanned.json]

Every box QR names its own place: `WH1|<face>|<bay>|<level>|<SKU>`. The
scanner files each code at the face, bay and level it was SEEN at. On a
carton's own label the two agree - on every one of 369 real QRs of the
2026-10-05 label flight. An old label left from an earlier shipment names
the place that carton used to be, and the two disagree. Such a reading is a
perfect decode of a false record, and this takes it out.

Truth-free: it reads the inventory and the warehouse layout, never ground
truth, so it works on a warehouse nobody has a truth file for. On that flight
it caught all 26 old QR labels that were read and none of the real ones.

What it cannot catch: an old label naming the same face, bay and level - a
carton moved within its own slot keeps its address - and anything that does
not carry its address at all, which is why the barcode is handled apart, in
barcode_inventory.stale_barcodes().

Writes:
    out/inventory_qr_checked.json   the inventory without them, same shape,
                                    for validate_inventory.py to score
    out/stale_labels_qr.json        what was taken out and why

The scanner's own file is never changed: it is scanner.py's output, and the
point is to be able to compare the two.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from warehouse_model import (INVENTORY, REPO_ROOT, load_config,   # noqa: E402
                             rotate_xy, world_yaw_rad)

OUT = REPO_ROOT / "out"

# How near a bay boundary a code may be filed and still be allowed either
# bay. The narrowest gap between a carton and an upright is several
# centimetres and a QR is filed to about one, so this only absorbs the odd
# reading right at the edge.
BAY_EDGE_M = 0.05


def address(payload: str):
    """(face, bay, level) a box QR names, or None if it is not one."""
    parts = str(payload).split("|")
    if len(parts) != 5 or parts[0] != "WH1":
        return None
    try:
        return parts[1], int(parts[2]), int(parts[3])
    except ValueError:
        return None


def bays_at(item: dict, cfg: dict) -> set[int]:
    """The bay a reading was filed in, or both bays when it is on the edge."""
    rk = cfg["racking"]
    # Back into the generator's frame, where bays run along x.
    x, _ = rotate_xy(float(item["estimated_x"]), float(item["estimated_y"]),
                     -world_yaw_rad(cfg))
    along = (x - rk["x_origin"]) / rk["bay_width"]
    out = {math.floor(along) + 1}
    frac = along - math.floor(along)
    edge = BAY_EDGE_M / rk["bay_width"]
    if frac < edge:
        out.add(math.floor(along))
    if frac > 1 - edge:
        out.add(math.floor(along) + 2)
    return out


def check(inventory: dict, cfg: dict) -> tuple[list, list]:
    """(kept items, stale items with the reason)."""
    kept, stale = [], []
    for item in inventory.get("items", []):
        named = address(item.get("id"))
        if named is None:
            kept.append(item)
            continue
        face, bay, level = named
        try:
            seen_bays = bays_at(item, cfg)
        except (KeyError, TypeError, ValueError):
            kept.append(item)
            continue
        why = []
        if face != item.get("shelf"):
            why.append(f"names face {face}, read on {item.get('shelf')}")
        if level != item.get("level"):
            why.append(f"names level {level}, read on {item.get('level')}")
        if bay not in seen_bays:
            why.append(f"names bay {bay}, read in {sorted(seen_bays)}")
        if why:
            stale.append({"id": item["id"], "why": "; ".join(why),
                          "shelf": item.get("shelf"), "level": item.get("level"),
                          "estimated_x": item.get("estimated_x"),
                          "estimated_y": item.get("estimated_y"),
                          "estimated_z": item.get("estimated_z")})
        else:
            kept.append(item)
    return kept, stale


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path, default=INVENTORY)
    ap.add_argument("--out", type=Path, default=OUT / "inventory_qr_checked.json")
    ap.add_argument("--report", type=Path, default=OUT / "stale_labels_qr.json")
    args = ap.parse_args()

    if not args.inventory.exists():
        raise SystemExit(f"no inventory at {args.inventory}")
    inventory = json.loads(args.inventory.read_text())
    kept, stale = check(inventory, load_config())

    checked = dict(inventory, items=kept, total_detected=len(kept),
                   stale_labels_removed=len(stale))
    args.out.write_text(json.dumps(checked, indent=2, ensure_ascii=False))
    args.report.write_text(json.dumps({
        "inventory": str(args.inventory), "kept": len(kept),
        "stale": stale}, indent=2, ensure_ascii=False))

    print(f"QR codes read: {len(kept) + len(stale)}, naming another place: {len(stale)}")
    for s in stale[:15]:
        print(f"  {s['id']:<24} {s['why']}")
    if len(stale) > 15:
        print(f"  ... and {len(stale) - 15} more in {args.report}")
    print(f"\nwritten: {args.out}\nwritten: {args.report}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
