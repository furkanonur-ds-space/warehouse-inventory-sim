#!/usr/bin/env python3
"""
What the label locator did on the flight that is currently in out/.

    python3 report/yolo_report.py
    python3 report/yolo_report.py --html out/yolo.html

A pure consumer, like every other tool in here: it reads the summaries and
readings the flight left behind and imports nothing from scanner/ or from the
reader. Run it after a scan has landed; make_reports.sh runs it for you.

THE QUESTION IT ANSWERS. A run with YOLO=1 puts a locator in front of zbar:
the model finds the qr and barkod labels, each one is cropped out and upscaled
and read on its own, after the ordinary full-frame pass. Since the full-frame
pass is unchanged and runs first, every reading the locator is responsible for
is a reading the run would not otherwise have had. Those are tagged `via:
crop` in the readings, and the boxes in the RECOVERED table below are exactly
the boxes that depend on one.

THE NUMBER THAT MATTERS MORE THAN THE RECOVERY. `labels_seen` counts placards
the model could LOCATE, whether or not anything read them. Held against the
readings it separates two failures that look identical in every other report:

    saw many, read few   the placard is being framed and not decoded - a
                         symbol problem, and narrower bars are the lever
    saw almost none      the placard is not in the picture at all - framing,
                         and no decoder can help

On 2026-09-22, replayed over the 100 saved frames where a barcode had failed,
the locator found 112 QRs and only 16 barcode placards, and recovered one
code. That is the second case, and it is why the bar width is still the open
question rather than the decoder.

A flight that flew without the locator leaves no `yolo` block in its summary
and this prints that it has nothing to report, which is deliberately not the
same message as a flight that used it and recovered nothing.
"""

from __future__ import annotations

import argparse
import html
import json
from collections import Counter
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
OUT = REPO_ROOT / "out"


def summaries(out: Path) -> list[tuple[Path, dict]]:
    """Every per-camera summary the run left, in a stable order."""
    found = []
    for path in sorted(out.glob("barcode_inventory*.json")):
        try:
            found.append((path, json.loads(path.read_text())))
        except (OSError, json.JSONDecodeError):
            continue
    return found


def readings(out: Path) -> list[dict]:
    """Every barcode reading of the run, from every camera."""
    rows = []
    for path in sorted(out.glob("barcode_readings*.jsonl")):
        try:
            for line in path.read_text().splitlines():
                if line.strip():
                    rows.append(json.loads(line))
        except (OSError, json.JSONDecodeError):
            continue
    return rows


def collect(out: Path) -> dict:
    per_camera = []
    seen = Counter()
    by_pass = Counter()
    settings = None

    for path, body in summaries(out):
        y = body.get("yolo")
        if not y:
            continue
        settings = settings or {k: y.get(k) for k in
                                ("weights", "conf", "margin", "min_side_px")}
        cam_seen = Counter(y.get("labels_seen", {}))
        cam_pass = Counter(y.get("readings_by_pass", {}))
        seen += cam_seen
        by_pass += cam_pass
        per_camera.append({
            "summary": path.name,
            "source": body.get("source", ""),
            "frames": body.get("frames_processed", 0),
            "labels_seen": dict(cam_seen),
            "readings_by_pass": dict(cam_pass),
            "barcode_readings": body.get("barcode_readings", 0),
            "boxes_with_barcode": body.get("boxes_with_barcode", 0),
        })

    if not per_camera:
        return {"used": False}

    # The boxes that exist in this run only because of a crop. A box counts as
    # recovered when EVERY reading of it came from the crop pass: one
    # full-frame reading means the run already had the box and the crop only
    # added another sighting of it.
    per_box: dict[str, Counter] = {}
    for row in readings(out):
        qr = row.get("linked_qr")
        if not qr:
            continue
        per_box.setdefault(qr, Counter())[row.get("via", "frame")] += 1
    recovered = sorted(qr for qr, c in per_box.items()
                       if c.get("crop", 0) and not c.get("frame", 0))

    return {
        "used": True,
        "settings": settings,
        "labels_seen": dict(seen),
        "readings_by_pass": dict(by_pass),
        "boxes_linked": len(per_box),
        "boxes_recovered": recovered,
        "per_camera": per_camera,
    }


def verdict(data: dict) -> str:
    """The one sentence a reader of this report should leave with."""
    seen_bar = data["labels_seen"].get("barkod", 0)
    read = sum(data["readings_by_pass"].values())
    crop = data["readings_by_pass"].get("crop", 0)
    if seen_bar == 0:
        return ("The locator never found a barcode placard. Nothing here is "
                "about the decoder; the placards were not framed.")
    if read == 0:
        return (f"{seen_bar} barcode placards located and not one decoded: "
                "framed and unreadable, which is a symbol problem.")
    ratio = crop / read
    if crop == 0:
        return (f"{seen_bar} barcode placards located; the full frame read "
                "every code the crops did, so the locator added nothing to "
                "this run.")
    n = len(data["boxes_recovered"])
    return (f"{crop} of {read} readings ({ratio:.0%}) came only from a crop, "
            f"and {n} box{'' if n == 1 else 'es'} "
            f"exist{'s' if n == 1 else ''} in this run because of one.")


def render_text(data: dict) -> str:
    if not data.get("used"):
        return ("No locator on this flight: no summary in out/ carries a "
                "yolo block.\n"
                "  fly one with  YOLO=1 bash scripts/scan_with_barcode.sh")
    s = data["settings"]
    lines = [
        "YOLO label locator",
        f"  weights {Path(s['weights']).name}  conf {s['conf']}  "
        f"margin {s['margin']}  crops upscaled to {s['min_side_px']} px",
        "",
        "labels located by the model",
    ]
    for cls, n in sorted(data["labels_seen"].items()):
        lines.append(f"  {cls:8s} {n:6d}")
    lines += ["", "barcode readings by pass"]
    for name, n in sorted(data["readings_by_pass"].items()):
        what = "full frame" if name == "frame" else "YOLO crop only"
        lines.append(f"  {name:8s} {n:6d}   {what}")
    lines += ["",
              f"boxes with a linked barcode : {data['boxes_linked']}",
              f"boxes owed to a crop        : {len(data['boxes_recovered'])}"]
    if data["boxes_recovered"]:
        lines.append("  " + ", ".join(data["boxes_recovered"][:20])
                     + (" ..." if len(data["boxes_recovered"]) > 20 else ""))
    lines += ["", "per camera"]
    for cam in data["per_camera"]:
        lines.append(f"  {cam['summary']}  {cam['frames']} frames  "
                     f"seen {cam['labels_seen']}  by pass {cam['readings_by_pass']}")
    lines += ["", verdict(data)]
    return "\n".join(lines)


def render_html(data: dict) -> str:
    if not data.get("used"):
        body = "<p>No locator on this flight.</p>"
    else:
        def table(title, mapping):
            rows = "".join(
                f"<tr><td>{html.escape(str(k))}</td><td>{v}</td></tr>"
                for k, v in sorted(mapping.items()))
            return f"<h2>{title}</h2><table>{rows}</table>"

        s = data["settings"]
        body = (
            f"<p class=set>weights <b>{html.escape(Path(s['weights']).name)}</b>"
            f" &middot; conf {s['conf']} &middot; margin {s['margin']}"
            f" &middot; crops to {s['min_side_px']} px</p>"
            + table("Labels located", data["labels_seen"])
            + table("Barcode readings by pass", data["readings_by_pass"])
            + f"<h2>Boxes owed to a crop</h2><p>"
            + (html.escape(", ".join(data["boxes_recovered"]))
               if data["boxes_recovered"] else "none")
            + f"</p><p class=verdict>{html.escape(verdict(data))}</p>")
    return ("<!doctype html><meta charset=utf-8><title>YOLO locator</title>"
            "<style>body{font:14px system-ui;margin:2rem;max-width:50rem}"
            "table{border-collapse:collapse;margin:.5rem 0}"
            "td{border:1px solid #ccc;padding:.2rem .6rem}"
            ".set{color:#666}.verdict{background:#f4f4f4;padding:.8rem;"
            "border-left:3px solid #888}</style>"
            "<h1>YOLO label locator</h1>" + body)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", type=Path, default=OUT,
                    help="where the flight left its files (default out/)")
    ap.add_argument("--json", type=Path, default=OUT / "yolo_report.json")
    ap.add_argument("--html", type=Path,
                    help="also write a page, e.g. out/yolo.html")
    args = ap.parse_args()

    data = collect(args.out_dir)
    print(render_text(data))
    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps(data, indent=2, ensure_ascii=False))
    print(f"\nwritten: {args.json}")
    if args.html:
        args.html.write_text(render_html(data))
        print(f"written: {args.html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
