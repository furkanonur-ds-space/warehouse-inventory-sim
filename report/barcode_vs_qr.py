"""
What did the barcodes add that the QR codes did not?

The placard payload names a slot, not a box: three boxes in a bay at one level
all carry the same text, so a barcode reading can never say which box it was.
That rules out using it to fill a hole in the inventory.

What it can do is notice that a slot was occupied when the scan came away with
nothing from it. That is what a torn or missing QR label looks like from the
air, and reporting it is better than a silent gap.

So this asks two questions of a finished run:

  1. Which slots did the barcode see that the QR scan has no box in at all?
     Those are the ones worth flying back to.
  2. Where both read, did they agree about which slot the box is in? A
     disagreement means one of the two is wrong about the shelf, and the
     inventory is only as good as that agreement.

Reads out/inventory_scanned.json, the ground truth, and every barcode readings
file the run left in out/. It writes nothing.

A run now carries a reader on each shelf-reading camera, so there are two
files, one per camera, and the questions above are only answerable across
both: the hires reads one face of an aisle and the rear camera the other, so
either file on its own has nothing to say about half the warehouse. They are
merged here, and each answer also says which camera saw it.
"""
import glob
import json
import os
import sys
from collections import Counter, defaultdict

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

OUT = os.path.join(ROOT, "out")
TRUTH = os.path.join(ROOT, "warehouse", "ground_truth.json")


def slot_of_qr(payload):
    """WH1|A|06|1|SKU59435 -> A0601, the placard text for that box."""
    parts = payload.split("|")
    if len(parts) < 4:
        return None
    # Row, then bay and level each padded to two digits: A, 06, 1 is A0601.
    # The level is the one that catches you out, since it is written with one
    # digit in the QR payload and two on the placard.
    try:
        return "%s%02d%02d" % (parts[1], int(parts[2]), int(parts[3]))
    except ValueError:
        return None


def camera_of(path):
    """
    Which camera a readings file came from, from the name the run gave it.

    barcode_readings_front.jsonl -> front. A file with no tag is from a run
    that had a single reader and did not need one.
    """
    stem = os.path.basename(path)
    for prefix in ("barcode_readings_", "barcode_readings"):
        if stem.startswith(prefix):
            tag = stem[len(prefix):].rsplit(".", 1)[0].strip("_")
            return tag or "camera"
    return stem


def default_readings():
    """
    Every readings file in out/, newest run first.

    Globbed rather than named because how many there are is a property of the
    run: one reader or two, and the tags come from the cameras it was put on.
    """
    found = sorted(glob.glob(os.path.join(OUT, "barcode_readings*.jsonl")))
    return [os.path.basename(f) for f in found]


def load_readings(path):
    """One file's readings, each tagged with the camera that produced it."""
    rows = []
    if not os.path.exists(path):
        return rows
    camera = camera_of(path)
    with open(path, encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if line:
                row = json.loads(line)
                row.setdefault("camera", camera)
                rows.append(row)
    return rows


def main(readings_names=None):
    truth = json.load(open(TRUTH, encoding="utf-8"))["codes"]
    boxes = [c for c in truth if c.get("type") == "box_qr"]
    slots = defaultdict(list)
    for box in boxes:
        slots[slot_of_qr(box["payload"])].append(box["payload"])

    scan = json.load(open(os.path.join(OUT, "inventory_scanned.json"),
                          encoding="utf-8"))
    found = {item["id"] for item in scan["items"]}

    names = readings_names or default_readings()
    if not names:
        raise SystemExit("no barcode readings in %s; fly with "
                         "scripts/scan_with_barcode.sh" % OUT)
    readings = []
    per_camera = {}
    for name in names:
        rows = load_readings(os.path.join(OUT, name))
        if not rows:
            print("  (nothing in %s)" % name)
            continue
        readings.extend(rows)
        per_camera.setdefault(camera_of(name), []).extend(rows)
    if not readings:
        raise SystemExit("no barcode readings in %s"
                         % ", ".join(names))

    seen_slots = Counter(r["payload"] for r in readings
                         if r.get("symbology") == "CODE128")
    linked = [r for r in readings if r.get("linked_qr")]

    print("run: %s" % scan.get("scan_date", "unknown"))
    print("  boxes in the warehouse       %d" % len(boxes))
    print("  read by QR                   %d" % len(found))
    print("  barcode readings             %d" % len(readings))
    print("  of those, linked to a QR     %d" % len(linked))
    print("  distinct slots seen          %d of %d"
          % (len(seen_slots), len(slots)))

    # Per camera, because the two read different faces and a total hides a
    # reader that saw nothing at all.
    if len(per_camera) > 1:
        print("\n  by camera:")
        for camera in sorted(per_camera):
            rows = per_camera[camera]
            camera_slots = {r["payload"] for r in rows
                            if r.get("symbology") == "CODE128"}
            print("    %-8s %6d readings, %4d linked, %3d slots"
                  % (camera, len(rows),
                     sum(1 for r in rows if r.get("linked_qr")),
                     len(camera_slots)))

    # 1. Slots the barcode saw where the scan has nothing at all.
    print("\nslots the barcode saw with no box read from them:")
    empty = []
    for slot in sorted(seen_slots):
        if slot not in slots:
            continue
        if not any(box in found for box in slots[slot]):
            empty.append(slot)
    if empty:
        by_slot = defaultdict(set)
        for row in readings:
            by_slot[row.get("payload")].add(row.get("camera", "camera"))
        for slot in empty:
            print("    %-8s %d readings, %d boxes in that slot, none read  (%s)"
                  % (slot, seen_slots[slot], len(slots[slot]),
                     ", ".join(sorted(by_slot[slot]))))
    else:
        print("    none; every slot the barcode saw has at least one box read")

    # A slot holds three boxes, so a partly read slot is the more common gap.
    print("\nslots read only in part, which is where a missed box hides:")
    partial = []
    for slot in sorted(seen_slots):
        if slot not in slots:
            continue
        got = sum(1 for box in slots[slot] if box in found)
        if 0 < got < len(slots[slot]):
            partial.append((slot, got, len(slots[slot])))
    if partial:
        for slot, got, total in partial:
            missing = [b for b in slots[slot] if b not in found]
            print("    %-8s %d of %d read, missing %s"
                  % (slot, got, total, ", ".join(missing)))
    else:
        print("    none")

    # 2. Where both read, do they agree which slot the box is in?
    print("\nwhere a barcode was linked to a QR, did they agree?")
    agree = disagree = 0
    for row in linked:
        expected = slot_of_qr(row["linked_qr"])
        if expected == row["payload"]:
            agree += 1
        else:
            disagree += 1
            if disagree <= 5:
                print("    %s linked to %s, which belongs to slot %s"
                      % (row["payload"], row["linked_qr"], expected))
    total = agree + disagree
    if total:
        print("    %d of %d agreed, %.1f per cent"
              % (agree, total, 100.0 * agree / total))


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "readings", nargs="*",
        help="readings files under out/. Defaults to every "
             "barcode_readings*.jsonl the run left there, which is one per "
             "camera it put a reader on")
    main(parser.parse_args().readings)
