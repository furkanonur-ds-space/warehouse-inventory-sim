#!/usr/bin/env python3
"""
Check that a carton is counted from its QR, and only where it has one.

    .venv/bin/python report/test_box_labels.py

No simulator and no flight. It takes the box log already in out/, removes
every carton box from a copy and keeps only the labels the detector located,
then builds the inventory from that copy:

- with --use-labels off, nothing is left to find a carton with, so none may
  be found;
- with it on, the cartons of the 0.50 m aisle (faces G and H) are found from
  their QRs; elsewhere a QR is too small to place, by design;
- and none of the cartons built with no code on them may be among those
  found, because they have no QR. One that were would be found from a
  neighbour's label, which is exactly the false count this must not make.

WHY. In the 0.50 m aisle the hires frame is one carton from side to side and
the model boxes it at 0.1 or not at all while locating its QR at 0.92; on
7 November 2026 that left 5 to 13 cartons of faces G and H unfound. Counting
the QR is what found all 432. This keeps that path honest.

Skipped, not failed, when the log in out/ carries no labels: only a flight
with YOLO_LOG_CONF records them.
"""
from __future__ import annotations

import json
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from box_inventory import CODE_RADIUS_M, build, true_cartons   # noqa: E402
from warehouse_model import REPO_ROOT                          # noqa: E402

OUT = REPO_ROOT / "out"

# The faces of the 0.50 m aisle, the only ones near enough for a QR to be
# placed (see main()).
NARROW_FACES = ("G", "H")


def labels_only(src: Path, dst: Path) -> int:
    """Copy each box log keeping its labels and dropping every carton box."""
    labels = 0
    for log in sorted(src.glob("yolo_boxes_*.jsonl")):
        rows = []
        for line in log.read_text().splitlines():
            if not line.strip():
                continue
            row = json.loads(line)
            row["boxes"] = []
            row["uncovered"] = 0
            labels += len(row.get("labels", []))
            rows.append(json.dumps(row))
        (dst / log.name).write_text("\n".join(rows) + "\n")
    return labels


def main() -> int:
    logs = [p for p in sorted(OUT.glob("yolo_boxes_*.jsonl"))
            if p.stat().st_size > 0]
    has_labels = any('"labels"' in p.read_text() for p in logs)
    if not logs or not has_labels:
        print("no box log with labels in out/ - nothing to test")
        print("  fly one with  SAVE_BOXES=1 YOLO=1 YOLO_LOG_CONF=0.10 bash "
              "scripts/scan_with_barcode.sh")
        return 0

    bare = {c["payload"] for c in true_cartons() if c.get("unlabelled")}
    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        n = labels_only(OUT, work)
        for name in ("inventory_scanned.json", "inventory_barcode.json"):
            if (OUT / name).exists():
                shutil.copy2(OUT / name, work / name)
        print("box log reduced to its labels: %d label sightings" % n)

        off = build(work, 20000.0, 0.30, 0.60, CODE_RADIUS_M)
        found_off = off["cartons"]["found"] if off.get("used") else 0
        if found_off != 0:
            print("FAILED: %d cartons found with no carton box and labels "
                  "off" % found_off)
            failures += 1
        else:
            print("  labels off: 0 cartons found, as it must be")

        # Only the 0.50 m aisle's faces. A QR is 70 mm, so from the wider
        # aisles it is far smaller than the area threshold and is not placed,
        # by design: there the cartons are boxed whole and need no help. On
        # the 7 November 2026 flight labels alone found 52 of 52 on G and on
        # H and none elsewhere.
        on = build(work, 20000.0, 0.30, 0.60, CODE_RADIUS_M, use_labels=True)
        found = {i["id"] for i in on["items"] if i["error_m"] is not None}
        narrow = [c["payload"] for c in true_cartons()
                  if not c.get("unlabelled")
                  and c["payload"].split("|")[1] in NARROW_FACES]
        got = sum(1 for p in narrow if p in found)
        if got < 0.9 * len(narrow):
            print("FAILED: labels on found %d of %d labelled cartons on "
                  "faces %s" % (got, len(narrow), "".join(NARROW_FACES)))
            failures += 1
        else:
            print("  labels on: %d of %d labelled cartons on faces %s found"
                  % (got, len(narrow), "".join(NARROW_FACES)))

        wrong = bare & {i["id"] for i in on["items"]}
        if wrong:
            print("FAILED: %d carton(s) with no code found from a label: %s"
                  % (len(wrong), ", ".join(sorted(wrong))))
            failures += 1
        else:
            print("  none of the %d cartons with no code found from a label"
                  % len(bare))

    print()
    if failures:
        print("%d FAILED" % failures)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
