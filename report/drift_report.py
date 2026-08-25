#!/usr/bin/env python3
"""
Drift report: how far the position estimate wandered, and how well it was pulled back.

    python3 report/drift_report.py
    python3 report/drift_report.py --html out/drift.html

Reads the marker corrections `scanner.py` records during a scan, plus the
navigation report it writes at the end. Writes `out/drift_report.json` and,
with --html, a single-file visual report.

WHAT IS BEING MEASURED. Indoors there is no GPS, so nothing in flight knows
where the vehicle truly is. The one absolute reference is an ArUco marker on
the floor at a surveyed position: when the downward camera sees one, the
difference between that fix and the estimator's own position is the
accumulated drift at that moment. Every number here is built from those
sightings and nothing else.

Three things it separates, because they fail differently:

  drift at fix      how wrong the estimator was, per sighting. This is the
                    localization error, and it is what grows between markers.
  offset trace      what the correction was doing about it. The offset is
                    subtracted from every setpoint, so the vehicle arrives at
                    the right place while the estimator stays wrong.
  direction bias    the mean north/east error across all fixes. Noise averages
                    to zero here; a bias that does not is a systematic lean,
                    which is a different fault from a random walk.

Per-marker breakdown matters for a reason worth stating: the markers sit at the
ends of the aisles, so error measured at marker 7 is error accumulated over the
run up to aisle 4. One marker reading much worse than its neighbours points at
that part of the route, not at the marker.

This tool never flies. It reads what a finished scan left behind.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics as st
from collections import defaultdict
from pathlib import Path

from warehouse_model import (GROUND_TRUTH, INVENTORY, NAV_REPORT, REPO_ROOT,
                             load_markers, load_run, percentile)


def summarise(values: list[float]) -> dict:
    v = sorted(values)
    return {
        "samples": len(v),
        "median_m": round(st.median(v), 3),
        "mean_m": round(st.mean(v), 3),
        "p95_m": round(percentile(v, 0.95), 3),
        "max_m": round(v[-1], 3),
    }


def write_html(path: Path, data: dict) -> None:
    """
    One self-contained file: a plan view of the fixes and the error over the run.

    Inline SVG, no script and no external asset, for the same reason the 3D
    view embeds its renderer - a report that only opens on the machine that
    made it is not a report.
    """
    ev = data["events"]
    markers = data["markers"]
    xs = [m["x"] for m in markers] or [0]
    ys = [m["y"] for m in markers] or [0]
    pad = 2.0
    x0, x1 = min(xs) - pad, max(xs) + pad
    y0, y1 = min(ys) - pad, max(ys) + pad
    # Keep the plan view to the warehouse's own aspect ratio. Stretching it to
    # a square would tilt every error vector on the page.
    h = 460
    w = round(h * (x1 - x0) / (y1 - y0))

    def px(x, y):
        # World +x right, +y up, so the y axis is flipped for screen space.
        return (round((x - x0) / (x1 - x0) * w, 1),
                round((1 - (y - y0) / (y1 - y0)) * h, 1))

    worst = max((e["error_m"] for e in ev), default=1.0) or 1.0
    plan = []
    for m in markers:
        cx, cy = px(m["x"], m["y"])
        plan.append(f'<rect x="{cx-6}" y="{cy-6}" width="12" height="12" '
                    f'fill="none" stroke="#4fd6d6" stroke-width="2"/>'
                    f'<text x="{cx+10}" y="{cy+4}" fill="#8b98a9" font-size="12">'
                    f'{m["id"]}</text>')
    for e in ev:
        # The fix is drawn where the marker is; the line shows which way, and
        # how far, the estimator disagreed. Scaled up 10x or a 3 cm error would
        # be a pixel.
        mx, my = e["marker_world"]["x"], e["marker_world"]["y"]
        dn = (e["fix"]["n"] - e["estimate_before"]["n"]) * 10
        de = (e["fix"]["e"] - e["estimate_before"]["e"]) * 10
        a = px(mx, my)
        b = px(mx + de, my + dn)      # NED north is +y, east is +x in the world
        t = min(1.0, e["error_m"] / worst)
        col = f"rgb({int(60+170*t)},{int(200-130*t)},90)"
        plan.append(f'<line x1="{a[0]}" y1="{a[1]}" x2="{b[0]}" y2="{b[1]}" '
                    f'stroke="{col}" stroke-width="1" opacity="0.55"/>')

    cw, ch = 940, 200
    n = len(ev)
    trace = []
    if n:
        top = max(max(e["error_m"] for e in ev),
                  max(e["offset_after_m"] for e in ev), 0.01)
        def tp(i, v):
            return (round(i / max(1, n - 1) * cw, 1), round(ch - v / top * ch, 1))
        err_pts = " ".join(f"{x},{y}" for x, y in
                           (tp(i, e["error_m"]) for i, e in enumerate(ev)))
        off_pts = " ".join(f"{x},{y}" for x, y in
                           (tp(i, e["offset_after_m"]) for i, e in enumerate(ev)))
        trace.append(f'<polyline points="{err_pts}" fill="none" stroke="#ffb020" '
                     f'stroke-width="1.2"/>')
        trace.append(f'<polyline points="{off_pts}" fill="none" stroke="#5b8dfc" '
                     f'stroke-width="1.6"/>')
        for frac in (0.5, 1.0):
            y = round(ch - frac * ch, 1)
            trace.append(f'<line x1="0" y1="{y}" x2="{cw}" y2="{y}" stroke="#2a3648"/>')
            trace.append(f'<text x="4" y="{y-4}" fill="#8b98a9" font-size="11">'
                         f'{top*frac:.2f} m</text>')

    rows = "".join(
        f'<tr><td>{m["id"]}</td><td>{m["x"]:+.1f}, {m["y"]:+.1f}</td>'
        f'<td>{m["fixes"]}</td><td>{m["median_m"]:.3f}</td>'
        f'<td>{m["max_m"]:.3f}</td></tr>'
        for m in data["per_marker"])

    s = data["drift_at_fix"]
    bias = data["direction_bias_m"]
    path.write_text(f"""<!doctype html>
<meta charset="utf-8"><title>Drift Report - Warehouse Scan</title>
<style>
 body{{margin:0;padding:2rem;background:#11151c;color:#dfe6ef;
      font-family:system-ui,sans-serif}}
 h1{{font-size:1.5rem;margin:0 0 .2rem}} h2{{font-size:1.05rem;margin:1.8rem 0 .5rem}}
 .src{{color:#8b98a9;font-size:.85rem;margin-bottom:1.4rem}}
 .cards{{display:flex;flex-wrap:wrap;gap:.8rem}}
 .card{{background:#182130;border:1px solid #2a3648;border-radius:8px;
        padding:.7rem 1rem;min-width:150px}}
 .card b{{display:block;font-size:1.35rem;font-weight:600}}
 .card span{{color:#8b98a9;font-size:.8rem}}
 table{{border-collapse:collapse;margin-top:.4rem}}
 td,th{{border:1px solid #2a3648;padding:.3rem .7rem;font-size:.88rem;text-align:right}}
 th{{background:#182130;color:#8b98a9}} td:first-child,th:first-child{{text-align:left}}
 .k{{display:inline-block;width:22px;height:3px;vertical-align:3px;margin-right:6px}}
 .note{{color:#8b98a9;font-size:.85rem;max-width:60rem;line-height:1.5}}
</style>
<h1>Drift report</h1>
<div class="src">{data['source']} &middot; {data.get('scan_date') or 'unknown date'}</div>
<div class="cards">
  <div class="card"><b>{s['median_m']:.3f} m</b><span>drift at fix, median</span></div>
  <div class="card"><b>{s['p95_m']:.3f} m</b><span>p95</span></div>
  <div class="card"><b>{s['max_m']:.3f} m</b><span>worst single fix</span></div>
  <div class="card"><b>{data['final_offset_m']:.3f} m</b><span>final offset</span></div>
  <div class="card"><b>{s['samples']}</b><span>marker fixes</span></div>
  <div class="card"><b>{len(data['per_marker'])}/{data['markers_total']}</b>
       <span>markers seen</span></div>
</div>

<h2>Where the fixes came from</h2>
<p class="note">Each line starts at a marker and points the way the estimator
was wrong, at ten times scale. Colour runs green to red with the size of the
error.</p>
<svg viewBox="0 0 {w} {h}" width="{w}" height="{h}"
     style="max-width:100%;height:auto;background:#0d1119;border:1px solid #2a3648;
     border-radius:8px">{''.join(plan)}</svg>

<h2>Over the run</h2>
<p class="note"><span class="k" style="background:#ffb020"></span>drift measured
at each fix &nbsp; <span class="k" style="background:#5b8dfc"></span>correction
offset in force. The offset should track the measured error and settle, not
climb: an offset that grows without bound is the correction summing its
measurements instead of converging on them.</p>
<svg viewBox="0 0 {cw} {ch}" width="{cw}" height="{ch}"
     style="max-width:100%;height:auto;background:#0d1119;border:1px solid #2a3648;
     border-radius:8px">{''.join(trace)}</svg>

<h2>By marker</h2>
<p class="note">Markers sit at the aisle ends, so error read at one of them is
error accumulated on the way to it.</p>
<table><tr><th>marker</th><th>position</th><th>fixes</th><th>median</th>
<th>max</th></tr>{rows}</table>

<h2>Direction bias</h2>
<p class="note">Mean error across every fix: north {bias['n']:+.3f} m, east
{bias['e']:+.3f} m, magnitude {bias['magnitude_m']:.3f} m. Random walk averages
towards zero; what is left is a systematic lean.</p>
""")


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path, default=INVENTORY)
    ap.add_argument("--navigation", type=Path, default=NAV_REPORT)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--out", type=Path,
                    default=REPO_ROOT / "out" / "drift_report.json")
    ap.add_argument("--html", type=Path,
                    help="also write a visual report here, e.g. out/drift.html")
    args = ap.parse_args()

    run = load_run(args.inventory)
    events = run.get("marker_corrections", [])
    nav = json.loads(args.navigation.read_text()) if args.navigation.exists() else {}
    all_markers = load_markers(args.ground_truth)

    if not events:
        # Not an error. A scan with no marker sighting is a scan flown entirely
        # on dead reckoning, and saying so is more useful than a page of zeros.
        print("NO MARKER FIXES in this run.")
        print("Nothing measured the position estimate, so drift is unknown for it.")
        print("The usual causes: the marker map did not load, the downward")
        print("camera published nothing, or the ArUco dictionary in the layout")
        print("does not match the one the world was generated with.")
        return 1

    errors = [e["error_m"] for e in events]
    drift = summarise(errors)

    by_marker = defaultdict(list)
    for e in events:
        by_marker[e["marker_id"]].append(e)
    per_marker = []
    for mid in sorted(by_marker):
        group = by_marker[mid]
        errs = sorted(x["error_m"] for x in group)
        per_marker.append({
            "id": mid,
            "x": group[0]["marker_world"]["x"],
            "y": group[0]["marker_world"]["y"],
            "fixes": len(group),
            "median_m": round(st.median(errs), 3),
            "max_m": round(errs[-1], 3),
        })

    bias_n = st.mean(e["fix"]["n"] - e["estimate_before"]["n"] for e in events)
    bias_e = st.mean(e["fix"]["e"] - e["estimate_before"]["e"] for e in events)

    # Did the correction settle? Compare the last tenth of the offset trace
    # against the drift it was supposed to match. A trace that ends far above
    # the measured error is the accumulation bug, not a converged offset.
    tail = [e["offset_after_m"] for e in events[-max(1, len(events) // 10):]]
    settled = {
        "offset_tail_mean_m": round(st.mean(tail), 3),
        "drift_median_m": drift["median_m"],
        "converged": bool(st.mean(tail) <= max(0.05, 3 * drift["median_m"])),
    }

    seen_ids = {m["id"] for m in per_marker}
    unseen = sorted(m["marker_id"] for m in all_markers
                    if m["marker_id"] not in seen_ids)

    report = {
        "source": str(args.inventory),
        "scan_date": run.get("scan_date"),
        "drift_at_fix": drift,
        "final_offset_m": run.get("final_drift_offset_m",
                                  nav.get("final_drift_offset_m", 0.0)),
        "direction_bias_m": {"n": round(bias_n, 3), "e": round(bias_e, 3),
                             "magnitude_m": round(math.hypot(bias_n, bias_e), 3)},
        "convergence": settled,
        "per_marker": per_marker,
        "markers_total": len(all_markers),
        "markers_unseen": unseen,
        "mission_duration_s": nav.get("mission_duration_s"),
        "waypoints": nav.get("waypoints"),
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False))

    print("DRIFT REPORT")
    print(f"source: {args.inventory}")
    if nav:
        wp = nav.get("waypoints") or {}
        print(f"flight: {nav.get('mission_duration_s')} s, "
              f"{wp.get('reached')}/{wp.get('planned')} waypoints")
    print()
    print(f"  marker fixes       : {drift['samples']}")
    print(f"  drift at fix       : median {drift['median_m']:.3f} m   "
          f"p95 {drift['p95_m']:.3f} m   max {drift['max_m']:.3f} m")
    print(f"  final offset       : {report['final_offset_m']:.3f} m")
    print(f"  direction bias     : n {bias_n:+.3f} m  e {bias_e:+.3f} m  "
          f"|{report['direction_bias_m']['magnitude_m']:.3f}| m")
    print(f"  correction         : "
          + ("converged" if settled["converged"] else
             "NOT CONVERGED -- offset ends at "
             f"{settled['offset_tail_mean_m']:.3f} m against a measured "
             f"{drift['median_m']:.3f} m"))
    print(f"  markers seen       : {len(per_marker)}/{len(all_markers)}"
          + (f"   never seen: {unseen}" if unseen else ""))

    print("\nby marker:")
    print("  id   position        fixes   median     max")
    for m in per_marker:
        print(f"  {m['id']:<3d}  {m['x']:+6.1f},{m['y']:+6.1f}   "
              f"{m['fixes']:5d}   {m['median_m']:6.3f}  {m['max_m']:6.3f}")

    print(f"\nreport: {args.out}")

    if args.html:
        args.html.parent.mkdir(parents=True, exist_ok=True)
        write_html(args.html, {**report, "events": events,
                               "markers": per_marker,
                               "source": args.inventory.name})
        print(f"html  : {args.html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
