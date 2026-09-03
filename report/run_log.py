#!/usr/bin/env python3
"""
One row per flight, in a spreadsheet, so a run can be compared with the ones before it.

    python3 report/run_log.py            # add this run to out/runs.csv
    python3 report/run_log.py --show     # print the table, add nothing

Every number this writes already exists somewhere in out/, and every one of
them is overwritten by the next flight. The console logs survive and the
numbers do not, which is the wrong way round: an afternoon of scans leaves
twelve logs of scrolling text and the figures of exactly one of them.

So this reads what a finished run left behind and appends a single line. Open
out/runs.csv in a spreadsheet and the history is a table: what changed, when,
and what it did to coverage, to accuracy, to the margin and to the clearances.

It deliberately keeps no raw data. The recordings are 150 MB a run, PX4's own
logs reached six gigabytes across seventy eight flights, and neither answers a
question anybody asks twice. What gets asked twice is "was that better than
last time".

A run is logged once, keyed on the scan date, so calling this after every
flight is safe and calling it twice does nothing.

One column needs reading with care. `codes_read_once` counts codes the run
read in exactly one frame, which is the margin: those are the ones that go the
other way on a different machine. But it can only count codes that were read
at all, so a run whose coverage collapsed reports fewer of them, not more. The
run that lost 94 codes to uncorrected drift has the lowest count in the table
and the worst flight in it. Compare that column between runs of similar
coverage, and read it beside `codes`.
"""
import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "out")

# The columns, in the order a person reads them: what the run was, then what it
# found, then how well it flew, then how close it came to not working.
COLUMNS = [
    "scan_date",
    "duration_s",
    "waypoints",
    # What it found. The whole point of the flight.
    "codes",
    "of",
    "coverage_pct",
    "wrong_shelf",
    "wrong_level",
    "wrong_bay",
    "duplicates",
    # Where it put them.
    "pos_median_m",
    "pos_p95_m",
    "pos_max_m",
    "within_10cm_pct",
    # Whether it flew into anything.
    "clearance_alarms",
    "min_obstacle_m",
    # How close the run came to missing something. A total at its ceiling
    # cannot say; the thinnest face can.
    "thinnest_face",
    "thinnest_sightings",
    "codes_read_once",
    # Whether the decoders kept up.
    "frames_decoded",
    "frames_dropped",
    "frames_too_old",
    "frame_age_median_s",
    # Localisation.
    "drift_correction",
    "marker_fixes",
    "loc_error_median_m",
    "correction_offset_m",
    # What was injected on purpose, if anything.
    "drift_injected_m",
    "drift_path_m",
    # The configuration worth being able to blame.
    "opencv_threads",
    "recording",
]


def read(path):
    if not os.path.exists(path):
        return None
    with open(path, encoding="utf-8") as handle:
        return json.load(handle)


def thinnest(sightings):
    """The face with the fewest readings on its worst code, and that count."""
    if not sightings:
        return "", "", ""
    worst = min(sightings.items(), key=lambda kv: (kv[1]["min"], -kv[1]["read_once"]))
    once = sum(row["read_once"] for row in sightings.values())
    return worst[0], worst[1]["min"], once


def injected(path):
    """What the drift relay actually put in, if it was running."""
    if not os.path.exists(path):
        return "", ""
    with open(path, encoding="utf-8") as handle:
        rows = list(csv.DictReader(handle))
    if not rows:
        return "", ""
    return round(float(rows[-1]["error_m"]), 4), round(
        float(rows[-1]["travelled_m"]), 1)


def gather(source=OUT):
    nav = read(os.path.join(source, "navigation_report.json"))
    val = read(os.path.join(source, "validation_report.json"))
    if nav is None:
        raise SystemExit("no out/navigation_report.json; has a scan finished?")
    if val is None:
        raise SystemExit("no out/validation_report.json; run "
                         "report/validate_inventory.py first")

    config = nav.get("configuration", {})
    pos = val.get("position_error", {})
    age = nav.get("frame_age_s", {})
    frames = nav.get("frames", {})
    face, fewest, once = thinnest(nav.get("sightings_per_code", {}))
    error, path = injected(os.path.join(source, "drift_injected.csv"))

    return {
        "scan_date": val.get("scan_date", nav.get("report_date", "")),
        "duration_s": round(nav.get("mission_duration_s", 0), 1),
        "waypoints": "%s/%s" % (nav["waypoints"]["reached"],
                                nav["waypoints"]["planned"]),
        "codes": val.get("detected", ""),
        "of": val.get("total_ground_truth", ""),
        "coverage_pct": val.get("detection_rate_pct", ""),
        "wrong_shelf": val.get("wrong_shelf", ""),
        "wrong_level": val.get("wrong_level", ""),
        "wrong_bay": val.get("wrong_bay", ""),
        "duplicates": (val.get("duplicate_id", 0)
                       + val.get("duplicate_spatial", 0)),
        "pos_median_m": pos.get("median_m", ""),
        "pos_p95_m": pos.get("p95_m", ""),
        "pos_max_m": pos.get("max_m", ""),
        "within_10cm_pct": pos.get("within_10cm_pct", ""),
        "clearance_alarms": nav.get("clearance_alarms", ""),
        "min_obstacle_m": nav.get("minimum_obstacle_distance_m", ""),
        "thinnest_face": face,
        "thinnest_sightings": fewest,
        "codes_read_once": once,
        "frames_decoded": sum(c.get("decoded", 0) for c in frames.values()),
        "frames_dropped": sum(c.get("dropped", 0) for c in frames.values()),
        "frames_too_old": sum(c.get("too_old", 0) for c in frames.values()),
        "frame_age_median_s": age.get("median", ""),
        "drift_correction": config.get("drift_correction", ""),
        "marker_fixes": nav.get("marker_correction_count", ""),
        "loc_error_median_m": (nav.get("localization_error_m") or {}).get(
            "median", ""),
        "correction_offset_m": nav.get("final_drift_offset_m", ""),
        "drift_injected_m": error,
        "drift_path_m": path,
        "opencv_threads": config.get("opencv_threads", ""),
        "recording": config.get("recording", ""),
    }


def existing(path):
    if not os.path.exists(path):
        return []
    with open(path, encoding="utf-8", newline="") as handle:
        return list(csv.DictReader(handle))


def show(rows):
    if not rows:
        print("no runs logged yet")
        return
    keep = ["scan_date", "codes", "of", "pos_median_m", "clearance_alarms",
            "thinnest_sightings", "codes_read_once", "drift_correction",
            "drift_injected_m", "duration_s"]
    widths = {k: max(len(k), max(len(str(r.get(k, ""))) for r in rows))
              for k in keep}
    print("  ".join(k.ljust(widths[k]) for k in keep))
    for row in rows:
        print("  ".join(str(row.get(k, "")).ljust(widths[k]) for k in keep))


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--log", default=os.path.join(OUT, "runs.csv"))
    parser.add_argument("--show", action="store_true",
                        help="print the table and append nothing")
    parser.add_argument("--force", action="store_true",
                        help="append even if this scan is already there")
    parser.add_argument("--from", dest="source", default=OUT,
                        help="read a run from somewhere other than out/, so "
                             "flights already kept by hand can be filled in")
    args = parser.parse_args()

    rows = existing(args.log)
    if args.show:
        show(rows)
        return 0

    row = gather(args.source)
    if any(r.get("scan_date") == row["scan_date"] for r in rows) \
            and not args.force:
        print("already logged: %s" % row["scan_date"])
        print("  %s is unchanged. Use --force to append it again."
              % os.path.relpath(args.log, ROOT))
        return 0

    new = not os.path.exists(args.log)
    os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
    with open(args.log, "a", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        if new:
            writer.writeheader()
        writer.writerow(row)

    print("added %s to %s" % (row["scan_date"],
                              os.path.relpath(args.log, ROOT)))
    print("  %s of %s codes, %s wrong shelf, position %s m at the median"
          % (row["codes"], row["of"], row["wrong_shelf"], row["pos_median_m"]))
    print("  thinnest face %s at %s readings, %s codes read once"
          % (row["thinnest_face"], row["thinnest_sightings"],
             row["codes_read_once"]))
    if row["drift_injected_m"] not in ("", 0, 0.0):
        print("  %s m of drift injected over %s m of path, correction %s"
              % (row["drift_injected_m"], row["drift_path_m"],
                 "on" if row["drift_correction"] else "OFF"))
    print()
    show(existing(args.log))
    return 0


if __name__ == "__main__":
    sys.exit(main())
