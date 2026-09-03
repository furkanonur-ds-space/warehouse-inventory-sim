"""
Both labels on every box, side by side.

A box carries two codes and they say different things: a QR that names the
box, and a Code128 barcode carrying its own payload. Neither can be derived
from the other, which is exactly why both are read on the same pass and both
have to be reported. This is the report that holds them next to each other.

For every box in the warehouse it asks two independent questions - was its QR
read, and was its barcode read - and never answers one with the other. The
pairing comes from ground truth, which knows the two labels sit on the same
box; that is scoring both measurements, not deriving one from the other.

What it produces:

  1. How many boxes gave up both labels, one, or neither. A box read by only
     one is a box whose record is half there, and which half matters.
  2. Where a barcode was read in the same frame as a QR, whether the pair sits
     on the same box. A mismatch means a misread or a bad link.
  3. What each camera contributed, since the hires reads one face of an aisle
     and the rear camera the other.

Reads out/inventory_scanned.json, the ground truth, and every barcode readings
file the run left in out/. It writes nothing.
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


def label_pairs(truth):
    """
    Box QR payload -> the barcode payload on that same box, from ground truth.

    The world generator puts both labels on one box and records both, so the
    pairing is a fact about the warehouse rather than a rule about payloads.
    An earlier version derived the barcode from the QR's SKU, which only held
    while the two carried the same fact; the moment they carry different ones
    it reports a reading nobody made.
    """
    by_slot = {}
    for code in truth:
        if code.get("type") != "box_placard":
            continue
        key = (code["row"], int(code["bay"]), int(code["level"]),
               round(code["label_pose_xyzrpy"][1], 3))
        by_slot[key] = code["payload"]

    pairs = {}
    for code in truth:
        if code.get("type") != "box_qr":
            continue
        key = (code["row"], int(code["bay"]), int(code["level"]),
               round(code["label_pose_xyzrpy"][1], 3))
        if key in by_slot:
            pairs[code["payload"]] = by_slot[key]
    return pairs


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
    pairs = label_pairs(truth)            # QR payload -> barcode on that box
    box_of_barcode = {bar: qr for qr, bar in pairs.items()}

    scan = json.load(open(os.path.join(OUT, "inventory_scanned.json"),
                          encoding="utf-8"))
    qr_read = {item["id"] for item in scan["items"]}

    names = readings_names or default_readings()
    if not names:
        raise SystemExit("no barcode readings in %s; fly with "
                         "scripts/scan_with_barcode.sh" % OUT)
    readings, per_camera = [], {}
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
    seen_bar = Counter(r["payload"] for r in bars)
    unknown = sorted(code for code in seen_bar if code not in box_of_barcode)
    bar_read = {box_of_barcode[c] for c in seen_bar if c in box_of_barcode}

    both = qr_read & bar_read
    only_qr = qr_read - bar_read
    only_bar = bar_read - qr_read
    neither = {b["payload"] for b in boxes} - qr_read - bar_read

    print("run: %s" % scan.get("scan_date", "unknown"))
    print("  boxes in the warehouse       %d" % len(boxes))
    print("  both labels read             %d" % len(both))
    print("  QR only                      %d" % len(only_qr))
    print("  barcode only                 %d" % len(only_bar))
    print("  neither                      %d" % len(neither))
    print("  barcode readings             %d" % len(bars))
    if unknown:
        print("  payloads on no box           %d  (%s)"
              % (len(unknown), ", ".join(unknown[:6])))

    if len(per_camera) > 1:
        print("\n  by camera:")
        for camera in sorted(per_camera):
            rows = [r for r in per_camera[camera]
                    if r.get("symbology") == "CODE128"]
            print("    %-8s %6d readings, %4d linked, %3d barcodes"
                  % (camera, len(rows),
                     sum(1 for r in per_camera[camera] if r.get("linked_qr")),
                     len({r["payload"] for r in rows})))

    # 1. A box whose record is half there, and which half.
    def by_face(payloads):
        return ", ".join("%s %d" % kv for kv in
                         sorted(Counter(p.split("|")[1]
                                        for p in payloads).items()))

    print("\nboxes that gave up only one of their two labels:")
    if only_qr:
        print("    QR but no barcode   %4d   by face: %s"
              % (len(only_qr), by_face(only_qr)))
    if only_bar:
        print("    barcode but no QR   %4d   by face: %s"
              % (len(only_bar), by_face(only_bar)))
    if not only_qr and not only_bar:
        print("    none; every box read gave up both")

    print("\nboxes neither label read:")
    if neither:
        for payload in sorted(neither)[:10]:
            print("    %s" % payload)
        if len(neither) > 10:
            print("    ... and %d more" % (len(neither) - 10))
        print("    %d of %d, by face: %s"
              % (len(neither), len(boxes), by_face(neither)))
    else:
        print("    none; every box was read by at least one label")

    # 2. A barcode read in the same frame as a QR: same box or not?
    print("\nwhere a barcode was read beside a QR, were they on the same box?")
    agree = disagree = 0
    for row in readings:
        linked = row.get("linked_qr")
        if not linked:
            continue
        if pairs.get(linked) == row["payload"]:
            agree += 1
        else:
            disagree += 1
            if disagree <= 5:
                print("    %s read beside %s, which carries %s"
                      % (row["payload"], linked, pairs.get(linked)))
    total = agree + disagree
    if total:
        print("    %d of %d on the same box, %.1f per cent"
              % (agree, total, 100.0 * agree / total))
    else:
        print("    nothing was linked")


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
