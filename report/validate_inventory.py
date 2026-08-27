#!/usr/bin/env python3
"""
Score a scan against ground truth: what was found, where it was put, how wrong.

    python3 report/validate_inventory.py
    python3 report/validate_inventory.py --list-worst 5 --list-missed

Reads `out/inventory_scanned.json` and `warehouse/ground_truth.json`, writes
`out/validation_report.json`. No simulator, no flight, no ROS - two files in,
one file and a printed summary out.

THE HEADLINE NUMBER is inventory accuracy, not position error. A record counts
as correct when (a) its payload exists in ground truth and (b) the shelf face,
level and bay it was filed under are the right ones. That is the question a
warehouse actually asks: is the right product recorded in the right location.
Position error is reported next to it in metres but stays out of the
definition - a 5 cm error that leaves the box in its own bay has not
misfiled anything.

The scanner snaps each estimate to the nearest shelf face and flight altitude,
so a large part of the residual position error is the offset between a flight
altitude and where the label sits inside that level. That is expected and does
not move a record out of its bay; it is the reason the two metrics are kept
apart.

DUPLICATES are checked two ways, because they fail differently:
  identity - the same payload filed twice (a de-duplication miss)
  spatial  - two different payloads landing within DUP_DIST_M of each other,
             which is one physical box counted twice under two readings

THE PER-RECORD SHIFT goes to `out/position_offsets.csv`: for every scored box,
where ground truth says its label is, where the scan put it, and the signed
difference on each axis. The summary numbers say how large the error is; that
file says which way it points, box by box, which is the only form in which a
systematic lean can be told apart from scatter. The 3D view draws the same
thing: a hollow marker at the true position, the estimate beside it, and the
shift between them.

Ground truth is read here for MEASUREMENT ONLY. The estimate is made in
`scanner/scanner.py`, which never opens this file.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics as st
from collections import Counter
from pathlib import Path

from warehouse_model import (CONFIG, GROUND_TRUTH, INVENTORY, REPO_ROOT,
                             error_to_truth, flight_altitudes, label_profile,
                             load_box_geometry, load_config, load_ground_truth,
                             load_run, percentile, records)

# Two records nearer than this are treated as one physical box seen twice.
# Boxes on a pallet here sit about 0.4 m apart, so 0.10 m cannot mistake a
# genuine neighbour for a duplicate.
DUP_DIST_M = 0.10

# The bar a run has to clear to be called a pass.
ACCURACY_TARGET_PCT = 95.0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path, default=INVENTORY)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--out", type=Path,
                    default=REPO_ROOT / "out" / "validation_report.json")
    ap.add_argument("--offsets", type=Path,
                    default=REPO_ROOT / "out" / "position_offsets.csv",
                    help="per-record shift between truth and estimate")
    ap.add_argument("--list-missed", action="store_true",
                    help="list every box that was never decoded")
    ap.add_argument("--list-worst", type=int, default=0,
                    help="list the N records with the largest position error")
    args = ap.parse_args()

    cfg = load_config(args.config)
    truth = load_ground_truth(args.ground_truth)
    run = load_run(args.inventory)
    items = records(run, cfg)

    matched, unknown_payload = [], []
    wrong_shelf, wrong_level, wrong_bay = [], [], []
    errors, per_record = [], []
    correct = 0

    for rec in items:
        t = truth.get(rec["qr"])
        if not t:
            # A payload that is not in ground truth is either a misdecode or a
            # texture rendered somewhere it should not be. Either way it cannot
            # be scored, and counting it as correct would flatter the run.
            unknown_payload.append(rec)
            continue
        matched.append(rec)
        ok = True
        if rec["shelf"] != t["row"]:
            wrong_shelf.append(rec); ok = False
        if rec["level"] != t["level"]:
            wrong_level.append(rec); ok = False
        if rec["bay"] != t["bay"]:
            wrong_bay.append(rec); ok = False
        correct += ok

        err = error_to_truth(rec, truth)
        errors.append(err)
        per_record.append((err, rec, t))

    scanned = {rec["qr"] for rec in matched}
    missed = [c for c in truth.values() if c["payload"] not in scanned]

    by_payload = Counter(rec["qr"] for rec in items)
    dup_id = [k for k, v in by_payload.items() if v > 1]

    # A few hundred records; the quadratic scan is well under a second and a
    # spatial index would be code to maintain for no gain.
    dup_spatial = []
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            a, b = items[i], items[j]
            if math.dist((a["x"], a["y"], a["z"]), (b["x"], b["y"], b["z"])) < DUP_DIST_M:
                dup_spatial.append([a["qr"], b["qr"]])

    e = sorted(errors)
    pos = {}
    if e:
        pos = {
            "median_m": round(st.median(e), 3),
            "mean_m": round(st.mean(e), 3),
            "p95_m": round(percentile(e, 0.95), 3),
            "max_m": round(e[-1], 3),
            "within_10cm_pct": round(100 * sum(x <= 0.10 for x in e) / len(e), 1),
            "within_25cm_pct": round(100 * sum(x <= 0.25 for x in e) / len(e), 1),
        }

    denom = len(items) or 1
    accuracy = 100.0 * correct / denom
    total = len(truth)

    report = {
        "inventory": str(args.inventory),
        "scan_date": run.get("scan_date"),
        "records": {"total_in_file": len(items), "scored": len(matched)},
        "detected": len(matched),
        "total_ground_truth": total,
        "detection_rate_pct": round(100 * len(matched) / total, 1) if total else 0,
        "missed": len(missed),
        "duplicate_id": len(dup_id),
        "duplicate_spatial": len(dup_spatial),
        "unknown_payload": len(unknown_payload),
        "wrong_shelf": len(wrong_shelf),
        "wrong_level": len(wrong_level),
        "wrong_bay": len(wrong_bay),
        "correct_records": correct,
        "inventory_accuracy_pct": round(accuracy, 1),
        "accuracy_target_pct": ACCURACY_TARGET_PCT,
        "position_error": pos,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))

    # Worst first: the rows anyone opens this file for are at the top, and the
    # tail is the bulk that landed where it should have.
    args.offsets.parent.mkdir(parents=True, exist_ok=True)
    with args.offsets.open("w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["qr", "product_id", "filed", "truth",
                    "est_x", "est_y", "est_z", "true_x", "true_y", "true_z",
                    "dx_m", "dy_m", "dz_m", "offset_m"])
        for err, rec, t in sorted(per_record, key=lambda p: -p[0]):
            tx, ty, tz = t["label_pose_xyzrpy"][:3]
            w.writerow([
                rec["qr"], rec["product_id"],
                f"{rec['shelf']}-{rec['bay']:02d}-L{rec['level']}",
                f"{t['row']}-{t['bay']:02d}-L{t['level']}",
                round(rec["x"], 3), round(rec["y"], 3), round(rec["z"], 3),
                round(tx, 3), round(ty, 3), round(tz, 3),
                round(rec["x"] - tx, 3), round(rec["y"] - ty, 3),
                round(rec["z"] - tz, 3), round(err, 3)])

    print("INVENTORY VALIDATION")
    print(f"source: {args.inventory}")
    if run.get("scan_date"):
        print(f"scan  : {run['scan_date']}")
    print()
    print(f"  detected / total   : {len(matched)} / {total}  "
          f"({report['detection_rate_pct']}%)")
    print(f"  missed             : {len(missed)}")
    print(f"  duplicate payload  : {len(dup_id)}")
    print(f"  duplicate position : {len(dup_spatial)}")
    print(f"  payload not in truth: {len(unknown_payload)}")
    print(f"  wrong shelf face   : {len(wrong_shelf)}")
    print(f"  wrong level        : {len(wrong_level)}")
    print(f"  wrong bay          : {len(wrong_bay)}")
    if pos:
        print(f"\n  position error     : median {pos['median_m']:.3f} m   "
              f"p95 {pos['p95_m']:.3f} m   max {pos['max_m']:.3f} m")
        print(f"                       {pos['within_10cm_pct']:.1f}% <=10 cm   "
              f"{pos['within_25cm_pct']:.1f}% <=25 cm")
    verdict = "PASS" if accuracy >= ACCURACY_TARGET_PCT else "FAIL"
    print(f"\n  INVENTORY ACCURACY : {accuracy:.1f}%  "
          f"({correct}/{len(items)} records)   target >={ACCURACY_TARGET_PCT:.0f}% -> {verdict}")
    print(f"\nreport: {args.out}")
    print(f"shifts: {args.offsets}")

    if args.list_missed and missed:
        geometry = load_box_geometry(cfg)
        flight_z = flight_altitudes()
        print(f"\nmissed boxes ({len(missed)}):")
        if not geometry:
            print("  (box dimensions unavailable - the world has not been")
            print("   generated; run ./setup_px4.sh to produce it)")
        print("  location     size w x d x h      label z   off axis   px/module")
        profiles = []
        for c in sorted(missed, key=lambda c: (c["row"], c["level"], c["bay"])):
            pr = label_profile(c, geometry, flight_z)
            profiles.append(pr)
            size = ("%.2f x %.2f x %.2f" % tuple(pr["size"])) if pr["size"] else "unknown"
            off = f"{pr['z_offset_m']:+.2f} m" if pr["z_offset_m"] is not None else "   ?  "
            print(f"  {c['row']}-{c['bay']:02d}-L{c['level']}   {size:>18}   "
                  f"{pr['label_z']:6.2f}   {off:>8}   {pr['px_per_module']:6.2f}")
            print(f"               {c['payload']}")

        # Say whether the misses share anything, rather than leaving a table to
        # be eyeballed. Two of the three failures this warehouse has produced
        # were a shared property that a per-box listing does not show.
        if geometry:
            all_profiles = [label_profile(c, geometry, flight_z)
                            for c in truth.values()]
            vols = [p["volume_m3"] for p in all_profiles if p["volume_m3"]]
            miss_vols = [p["volume_m3"] for p in profiles if p["volume_m3"]]
            offs = [abs(p["z_offset_m"]) for p in profiles
                    if p["z_offset_m"] is not None]
            print("\n  against the whole warehouse:")
            if vols and miss_vols:
                smallest = sorted(vols)[max(0, int(0.25 * (len(vols) - 1)))]
                small = sum(v <= smallest for v in miss_vols)
                print(f"    volume    : missed {min(miss_vols):.3f}-{max(miss_vols):.3f} m3, "
                      f"all {min(vols):.3f}-{max(vols):.3f} m3"
                      f"   ({small}/{len(miss_vols)} in the smallest quarter)")
            if offs:
                worst = max(offs)
                print(f"    off axis  : worst {worst:+.2f} m against a "
                      f"0.23 m half-frame"
                      + ("   - inside the frame, so geometry does not explain "
                         "these" if worst < 0.23 else
                         "   - outside the frame, the known failure"))
    if args.list_worst and per_record:
        worst = sorted(per_record, key=lambda p: -p[0])[:args.list_worst]
        print(f"\nlargest position errors ({len(worst)}):")
        print("   total    dx      dy      dz     filed        truth")
        for err, rec, t in worst:
            tx, ty, tz = t["label_pose_xyzrpy"][:3]
            print(f"  {err:5.2f} m {rec['x']-tx:+6.2f} {rec['y']-ty:+6.2f} "
                  f"{rec['z']-tz:+6.2f}   "
                  f"{rec['shelf']}-{rec['bay']:02d}-L{rec['level']}  "
                  f"{t['row']}-{t['bay']:02d}-L{t['level']}  {rec['product_id']}")
    if dup_id:
        print(f"\nDUPLICATE PAYLOADS: {', '.join(dup_id[:10])}")
    if unknown_payload:
        print(f"\nPAYLOADS NOT IN GROUND TRUTH ({len(unknown_payload)}):")
        for rec in unknown_payload[:10]:
            print(f"  {rec['qr']!r}")

    return 0 if accuracy >= ACCURACY_TARGET_PCT else 2


if __name__ == "__main__":
    raise SystemExit(main())
