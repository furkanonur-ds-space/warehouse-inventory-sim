"""
What did the barcodes add that the QR codes did not?

Every box carries two labels: a QR naming the box, and a Code128 barcode
carrying that box's own SKU digits. The two say the same thing by different
routes - different symbology, different decoder, a different string - so
reading both is evidence that either can be read on its own.

That makes the barcode able to do what it could not when it named a shelf slot
shared by three boxes: identify a box the QR scan came away without. A torn or
unreadable QR is no longer a silent gap in the inventory but a box the other
label recovered.

So this asks three questions of a finished run:

  1. Which boxes did the barcode read that the QR scan missed? Those are
     recovered, not merely suspected.
  2. Where both read the same box, did they agree? A disagreement means one of
     the two labels was misread, and the inventory is only as good as that
     agreement.
  3. Which boxes did neither read? Those are the ones worth flying back for.

Reads out/inventory_scanned.json, the ground truth, and every barcode readings
file the run left in out/. It writes nothing.

A run carries a reader on each shelf-reading camera, so there are two files,
one per camera, and the questions above are only answerable across both: the
hires reads one face of an aisle and the rear camera the other, so either file
on its own has nothing to say about half the warehouse. They are merged here,
and each answer also says which camera saw it.
"""
import glob
import json
import os
import sys
from collections import Counter, defaultdict
from datetime import datetime, timedelta

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)

OUT = os.path.join(ROOT, "out")
TRUTH = os.path.join(ROOT, "warehouse", "ground_truth.json")


def barcode_of_qr(payload):
    """WH1|A|06|1|SKU59435 -> 59435, the barcode on that same box."""
    parts = payload.split("|")
    if len(parts) < 5:
        return None
    sku = parts[4]
    return sku[3:] if sku.startswith("SKU") else sku


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


def flight_window():
    """
    When the run being scored was in the air, as (start, end) ISO strings.

    The inventory is written on landing and the navigation report says how
    long the flight took, so the window follows from the two. Returns None if
    either is missing, and nothing is then filtered by time.
    """
    try:
        scan = json.load(open(os.path.join(OUT, "inventory_scanned.json"),
                              encoding="utf-8"))
        nav = json.load(open(os.path.join(OUT, "navigation_report.json"),
                             encoding="utf-8"))
        end = datetime.fromisoformat(scan["scan_date"])
        return (end - timedelta(seconds=float(nav["mission_duration_s"])), end)
    except Exception:
        return None


def newest_reading(path):
    """The timestamp of the last reading in a file, or None."""
    last = None
    try:
        with open(path, encoding="utf-8") as handle:
            for line in handle:
                line = line.strip()
                if line:
                    last = json.loads(line).get("t") or last
    except Exception:
        return None
    try:
        return datetime.fromisoformat(last) if last else None
    except ValueError:
        return None


def default_readings():
    """
    The readings files this run left in out/, one per camera.

    Globbed rather than named because how many there are is a property of the
    run: one reader or two, and the tags come from the cameras it was put on.

    Files left by an earlier run are dropped, not merged. out/ is rewritten by
    every run, but only under the names that run uses, so a file from an older
    naming scheme survives and would otherwise be added to this run's totals -
    which it was, silently doubling them. A file belongs to this run when it
    has a reading from after the flight started.
    """
    found = sorted(glob.glob(os.path.join(OUT, "barcode_readings*.jsonl")))
    window = flight_window()
    if window is None:
        return [os.path.basename(f) for f in found]
    start, _ = window
    keep, stale = [], []
    for path in found:
        newest = newest_reading(path)
        (keep if newest is not None and newest >= start else stale).append(path)
    for path in stale:
        print("  (ignoring %s: nothing in it from this flight)"
              % os.path.basename(path))
    return [os.path.basename(f) for f in (keep or found)]


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
    # The barcode on a box, back to the box it is on. One to one now: the
    # payload is that box's own SKU digits.
    box_of = {}
    for box in boxes:
        code = barcode_of_qr(box["payload"])
        if code is not None:
            box_of[code] = box["payload"]

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
        raise SystemExit("no barcode readings in %s" % ", ".join(names))

    bars = [r for r in readings if r.get("symbology") == "CODE128"]
    seen = Counter(r["payload"] for r in bars)
    # Which camera read each barcode, for the answers that name a box.
    cameras_of = defaultdict(set)
    for row in bars:
        cameras_of[row["payload"]].add(row.get("camera", "camera"))

    read_by_barcode = {box_of[code] for code in seen if code in box_of}
    unknown = [code for code in seen if code not in box_of]

    print("run: %s" % scan.get("scan_date", "unknown"))
    print("  boxes in the warehouse       %d" % len(boxes))
    print("  read by QR                   %d" % len(found))
    print("  read by barcode              %d" % len(read_by_barcode))
    print("  read by either               %d" % len(found | read_by_barcode))
    print("  barcode readings             %d" % len(bars))
    print("  of those, linked to a QR     %d"
          % sum(1 for r in readings if r.get("linked_qr")))
    if unknown:
        print("  payloads matching no box     %d  (%s)"
              % (len(unknown), ", ".join(sorted(unknown)[:5])))

    # Per camera, because the two read different faces and a total hides a
    # reader that saw nothing at all.
    if len(per_camera) > 1:
        print("\n  by camera:")
        for camera in sorted(per_camera):
            rows = [r for r in per_camera[camera]
                    if r.get("symbology") == "CODE128"]
            print("    %-8s %6d readings, %4d linked, %3d boxes"
                  % (camera, len(rows),
                     sum(1 for r in per_camera[camera] if r.get("linked_qr")),
                     len({r["payload"] for r in rows})))

    # 1. What the barcode recovered: a box the QR scan does not have.
    recovered = sorted(read_by_barcode - found)
    print("\nboxes the barcode read that the QR scan missed:")
    if recovered:
        for payload in recovered:
            code = barcode_of_qr(payload)
            print("    %-22s barcode %-8s %d readings  (%s)"
                  % (payload, code, seen[code],
                     ", ".join(sorted(cameras_of[code]))))
        print("    %d box%s recovered by the second label"
              % (len(recovered), "" if len(recovered) == 1 else "es"))
    else:
        print("    none; the QR scan already had every box the barcode read")

    # 2. Where both read the same box, did the two labels agree?
    print("\nwhere a barcode was linked to a QR, did they agree?")
    agree = disagree = 0
    for row in readings:
        if not row.get("linked_qr"):
            continue
        if barcode_of_qr(row["linked_qr"]) == row["payload"]:
            agree += 1
        else:
            disagree += 1
            if disagree <= 5:
                print("    %s read beside %s, whose barcode is %s"
                      % (row["payload"], row["linked_qr"],
                         barcode_of_qr(row["linked_qr"])))
    total = agree + disagree
    if total:
        print("    %d of %d agreed, %.1f per cent"
              % (agree, total, 100.0 * agree / total))
    else:
        print("    nothing was linked")

    # 3. What neither label reached.
    missed = sorted({b["payload"] for b in boxes} - found - read_by_barcode)
    print("\nboxes neither label read:")
    if missed:
        by_face = Counter(p.split("|")[1] for p in missed)
        for payload in missed[:10]:
            print("    %s" % payload)
        if len(missed) > 10:
            print("    ... and %d more" % (len(missed) - 10))
        print("    %d of %d, by face: %s"
              % (len(missed), len(boxes),
                 ", ".join("%s %d" % kv for kv in sorted(by_face.items()))))
    else:
        print("    none; every box was read by at least one label")


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
