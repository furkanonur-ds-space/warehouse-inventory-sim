#!/usr/bin/env python3
"""
One growing log of the boxes a run never decoded, so repeat offenders are visible.

    python3 report/missed_log.py                      # log this run, print the tally
    python3 report/missed_log.py --inventory docs/runs/2026-08-25/inventory_scanned.json
    python3 report/missed_log.py --summary            # tally only, append nothing

Appends to ONE file, `out/missed_boxes.jsonl`, and that file holds nothing but
misses. A run contributes one header line and one line per box it failed to
decode, so the same warehouse scanned five times leaves a record of which boxes
failed every time and which failed once.

WHY A PERSISTENT FILE. `validation_report.json` is overwritten by the next run,
and the misses this warehouse produces are largely the same boxes each flight.
The question worth answering is not "what did this run miss" but "what does
every run miss", and that cannot be read out of a file that only remembers the
last one.

Each miss line carries what is needed to reason about the cause without opening
the world: the QR payload, the shelf location, the label position in world
coordinates, the box's real dimensions, how far the label sat off the camera's
optical axis, and the pixels per QR module at the standoff flown for its aisle.

The header line is metadata about the run, not a box. It exists so the tally
has a denominator: a run that missed nothing still has to be counted, otherwise
"missed in 3 of 3 runs" is unprovable.

A run is logged once. Re-running this on the same scan is a no-op unless
--force is given, so it is safe to call after every flight.

Ground truth is read for MEASUREMENT ONLY, as everywhere else in `report/`.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

from warehouse_model import (CONFIG, GROUND_TRUTH, INVENTORY, REPO_ROOT,
                             flight_altitudes, label_profile, load_box_geometry,
                             load_config, load_ground_truth, load_run, records)

LOG = REPO_ROOT / "out" / "missed_boxes.jsonl"

# The camera's vertical half-frame is no longer one number. It scales with the
# standoff, and the standoff now varies by aisle: these aisles taper from
# 2.40 m to 0.50 m, so a face on the narrow end is flown at 0.25 m and sees a
# 0.07 m half-frame against a 0.25 m label stack. label_profile carries the
# per-face value out as "half_frame_m"; see warehouse_model.half_frame_m.


def location(row: str, bay: int, level: int) -> str:
    return f"{row}-{bay:02d}-L{level}"


def read_log(path: Path) -> tuple[list[dict], list[dict]]:
    """The log split into run headers and miss lines. Missing file reads empty."""
    if not path.exists():
        return [], []
    runs, misses = [], []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line:
            continue
        rec = json.loads(line)
        (runs if rec.get("kind") == "run" else misses).append(rec)
    return runs, misses


def run_id(run: dict, inventory: Path) -> str:
    """
    What makes a run the same run.

    The scan date is the scanner's own stamp and is the honest identity. A scan
    written without one falls back to the file it came from, which at least
    stops the default path being logged twice.
    """
    return run.get("scan_date") or f"file:{inventory.name}"


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path, default=INVENTORY)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--log", type=Path, default=LOG)
    ap.add_argument("--summary", action="store_true",
                    help="print the tally over the existing log, append nothing")
    ap.add_argument("--force", action="store_true",
                    help="append even if this scan is already in the log")
    ap.add_argument("--top", type=int, default=20,
                    help="how many boxes to list, worst first (0 for all)")
    args = ap.parse_args()

    runs, misses = read_log(args.log)

    if not args.summary:
        cfg = load_config(args.config)
        truth = load_ground_truth(args.ground_truth)
        run = load_run(args.inventory)
        geometry = load_box_geometry(cfg)
        flight_z = flight_altitudes()

        rid = run_id(run, args.inventory)
        if any(r["run_id"] == rid for r in runs) and not args.force:
            print(f"already logged: {rid}")
            print(f"  {args.log} is unchanged. Use --force to append it again.")
        else:
            scanned = {rec["qr"] for rec in records(run, cfg)}
            missed = [c for c in truth.values() if c["payload"] not in scanned]
            missed.sort(key=lambda c: (c["row"], c["level"], c["bay"]))
            stamp = datetime.now(timezone.utc).isoformat(timespec="seconds")

            lines = [json.dumps({
                "kind": "run",
                "run_id": rid,
                "scan_date": run.get("scan_date"),
                "source": str(args.inventory),
                "logged_at": stamp,
                "boxes_in_truth": len(truth),
                "decoded": len(scanned & set(truth)),
                "missed": len(missed),
            }, ensure_ascii=False)]

            for c in missed:
                pr = label_profile(c, geometry, flight_z)
                lines.append(json.dumps({
                    "kind": "miss",
                    "run_id": rid,
                    "qr": c["payload"],
                    "sku": c["payload"].split("|")[-1],
                    "location": location(c["row"], c["bay"], c["level"]),
                    "row": c["row"], "bay": c["bay"], "level": c["level"],
                    "entity": c["entity"],
                    # Where the label is, and where the box body is. They are
                    # not the same point: the label sits on the shelf face.
                    "label_pos": pr["position"],
                    "box_size_m": pr["size"],
                    "volume_m3": pr["volume_m3"],
                    "label_size_m": c.get("label_size_m"),
                    "module_size_m": c.get("module_size_m"),
                    # Negative means the label sat below the optical axis.
                    "label_z": pr["label_z"],
                    "flight_z": pr["axis_z"],
                    "z_offset_m": pr["z_offset_m"],
                    "outside_frame": (pr["z_offset_m"] is not None
                                      and abs(pr["z_offset_m"])
                                      > pr["half_frame_m"]),
                    "standoff_m": pr["standoff_m"],
                    "half_frame_m": pr["half_frame_m"],
                    "px_per_module": pr["px_per_module"],
                }, ensure_ascii=False))

            args.log.parent.mkdir(parents=True, exist_ok=True)
            with args.log.open("a") as fh:
                fh.write("\n".join(lines) + "\n")
            print(f"logged {len(missed)} missed boxes from {rid}")
            print(f"  -> {args.log}")
            runs, misses = read_log(args.log)

    if not runs:
        print(f"\nno runs in {args.log} yet.")
        return 1

    # The tally. Grouped by QR because that is the box's identity; the location
    # and geometry are carried along from the most recent sighting of it.
    by_qr: dict[str, list[dict]] = defaultdict(list)
    for m in misses:
        by_qr[m["qr"]].append(m)
    total_runs = len(runs)
    ranked = sorted(by_qr.values(),
                    key=lambda g: (-len(g), g[-1]["location"]))

    print(f"\nMISSED BOX LOG   {args.log}")
    print(f"runs logged: {total_runs}"
          + (f"   ({runs[0]['run_id']} .. {runs[-1]['run_id']})"
             if total_runs > 1 else f"   ({runs[0]['run_id']})"))
    print(f"boxes ever missed: {len(by_qr)} of {runs[-1]['boxes_in_truth']}")
    always = [g for g in ranked if len(g) == total_runs]
    print(f"missed in every run: {len(always)}")

    shown = ranked if args.top <= 0 else ranked[:args.top]
    print(f"\n  runs  location     size w x d x h        off axis   px/module   QR")
    for g in shown:
        m = g[-1]
        size = ("%.2f x %.2f x %.2f" % tuple(m["box_size_m"])) if m["box_size_m"] else "unknown"
        off = f"{m['z_offset_m']:+.2f} m" if m["z_offset_m"] is not None else "   ?  "
        flag = " !frame" if m["outside_frame"] else ""
        print(f"  {len(g)}/{total_runs}   {m['location']:<10}  {size:>18}   "
              f"{off:>8}{flag}   {m['px_per_module']:6.2f}   {m['qr']}")
    if args.top > 0 and len(ranked) > args.top:
        print(f"  ... {len(ranked) - args.top} more, --top 0 for all")

    # Say the one thing a per-box listing cannot: whether the misses are the
    # same boxes each time. That is the difference between a geometry problem
    # and a flaky detector, and it is the reason this log exists.
    if total_runs > 1:
        once = sum(1 for g in ranked if len(g) == 1)
        print(f"\n  repeatability: {len(always)} boxes missed every run, "
              f"{once} missed exactly once")
        print("  same boxes every run points at geometry or readability; "
              "a shifting set points at timing or detection.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
