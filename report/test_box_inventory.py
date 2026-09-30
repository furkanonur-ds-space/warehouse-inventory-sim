#!/usr/bin/env python3
"""
Check that a carton nobody read is the one the warning names.

    .venv/bin/python report/test_box_inventory.py

No simulator and no flight. It takes the run already in out/ and removes ONE
carton's codes from a copy of the inventories, leaving everything else alone.
The detector still saw that carton, so a run built from the doctored copy has
to name it - and has to keep naming the same few it named before, and no more.

WHY IT IS DONE BY SUBTRACTION. The warning cannot be tested by flying a
warehouse where everything reads, because then it should say nothing and a
warning that says nothing is indistinguishable from one that is broken. The
honest test is to make a carton unread on purpose and see it appear. Doing
that in the world would need a regenerated warehouse and another flight;
doing it to the inventory is the same subtraction with none of the cost, and
it exercises the real placement, clustering and matching on real data.

WHAT IT CANNOT CHECK. Removing a code from the inventory is not the same as a
code that failed to decode: the frames are untouched, so the detector's own
view of that carton is unchanged. This proves the warning FIRES for a carton
with no code near it. It does not prove a real decode failure looks like this
one - only a flight can, and face G is where to look for it.

Skipped, not failed, when out/ holds no box log: a checkout that has never
flown with SAVE_BOXES=1 has nothing to test and should not fail its suite.
"""
from __future__ import annotations

import json
import math
import shutil
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from box_inventory import CODE_RADIUS_M, build, true_cartons   # noqa: E402
from warehouse_model import REPO_ROOT                          # noqa: E402

OUT = REPO_ROOT / "out"

# How far a code may sit from a carton and still be one of ITS codes. The two
# labels of a carton sit within a few centimetres of each other and the next
# carton is 0.40 m away, so this reaches both of a carton's labels and neither
# of its neighbour's.
SAME_CARTON_M = 0.20


def strip(src: Path, dst: Path, centre) -> int:
    """Copy the run, dropping every code read within SAME_CARTON_M of here."""
    dropped = 0
    for name in ("inventory_barcode.json", "inventory_scanned.json"):
        path = src / name
        if not path.exists():
            continue
        body = json.loads(path.read_text())
        keep = []
        for item in body.get("items", []):
            here = (item["estimated_x"], item["estimated_y"],
                    item["estimated_z"])
            if math.dist(here, centre) <= SAME_CARTON_M:
                dropped += 1
                continue
            keep.append(item)
        body["items"] = keep
        (dst / name).write_text(json.dumps(body))
    return dropped


def main() -> int:
    logs = [p for p in sorted(OUT.glob("yolo_boxes_*.jsonl"))
            if p.stat().st_size > 0]
    if not logs:
        print("no box log in out/ - nothing to test")
        print("  fly one with  SAVE_BOXES=1 YOLO=1 bash "
              "scripts/scan_with_barcode.sh")
        return 0

    base = build(OUT, 20000.0, 0.30, 0.60, CODE_RADIUS_M)
    if not base.get("used") or base["cartons"]["found"] == 0:
        # A log that exists and holds nothing is the same case as no log:
        # there is nothing to subtract from. It happens - on 2026-09-28 the
        # readers died at the first frame and left two empty files - and it is
        # not this test failing, it is this test having nothing to test.
        print("the box log in out/ holds no cartons - nothing to test")
        print("  (an empty log means the readers never ran; check "
              "out/barcode_*.log)")
        return 0
    before = set(base["seen_but_not_read"])
    print("the run as flown: %d of %d cartons found, %d seen and not read"
          % (base["cartons"]["found"], base["cartons"]["true_total"],
             len(before)))

    # A carton the detector definitely saw and the run definitely read, so
    # that removing its codes is the only thing that changes.
    placed = {i["id"]: i for i in base["items"] if i["error_m"] is not None}
    victim = next((i for i in placed.values()
                   if i["id"] not in before and i["read"]
                   and i["sightings"] >= 3), None)
    if victim is None and not any(i["read"] for i in placed.values()):
        # A world built with no codes at all (codes.unlabelled_all): nothing
        # was read, so there is nothing to take away. Skipped for the same
        # reason as a missing box log - a failure here would say nothing.
        print("nothing was read on this run - no code to remove, "
              "nothing to test")
        return 0
    if victim is None:
        print("FAILED: no carton was both seen and read; the run is not one "
              "this test can use")
        return 1

    truth = {c["payload"]: c for c in true_cartons()}[victim["id"]]
    centre = (truth["x"], truth["y"], truth["z"])
    print("removing every code read near %s (seen %d times)"
          % (victim["id"], victim["sightings"]))

    failures = 0
    with tempfile.TemporaryDirectory() as tmp:
        work = Path(tmp)
        for log in logs:
            shutil.copy2(log, work / log.name)
        dropped = strip(OUT, work, centre)
        print("  %d readings removed" % dropped)
        if dropped == 0:
            print("FAILED: nothing was removed, so nothing was tested")
            return 1

        after = build(work, 20000.0, 0.30, 0.60, CODE_RADIUS_M)
        named = set(after["seen_but_not_read"])

        if victim["id"] not in named:
            print("FAILED: %s was made unread and the warning did not name it"
                  % victim["id"])
            failures += 1
        else:
            print("  named: %s" % victim["id"])

        # Nothing else may change. A warning that fires for the doctored
        # carton but also for six of its neighbours has not found it, it has
        # blurred the whole bay.
        extra = named - before - {victim["id"]}
        if extra:
            print("FAILED: %d carton(s) newly named that were not touched: %s"
                  % (len(extra), ", ".join(sorted(extra))))
            failures += 1
        else:
            print("  no other carton newly named")

        if after["cartons"]["found"] != base["cartons"]["found"]:
            print("FAILED: the cartons found changed from %d to %d; removing "
                  "a code must not change what was SEEN"
                  % (base["cartons"]["found"], after["cartons"]["found"]))
            failures += 1
        else:
            print("  cartons found unchanged at %d" % after["cartons"]["found"])

    print()
    if failures:
        print("%d FAILED" % failures)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
