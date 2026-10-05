#!/usr/bin/env python3
"""
What darkness cost: one flight over the dimmed warehouse, split by how much
light each label got.

    .venv/bin/python report/light_report.py [--out-dir out]

The dimmed world (`lights_stress:` in warehouse.yaml) turns the ambient light
and the lamps down and switches half the lamps off, a checkerboard of dark
aisle halves. Every label then gets a different amount of light, and the
generator writes it into ground truth: `light.E` in this world, `light.E_clean`
in the clean one, in the light model's own unit (warehouse/stress.py,
illuminance). This report only READS those numbers; how the light was set up
is the generator's decision and computing it again here would let the two
drift apart.

The bands are absolute E, not the ratio to the clean world, because a reader
answers to the light a label has, not to how much it lost: the clean world
already has labels at E 0.55 that read. The 0.5-1 band exists in both worlds,
which is what lets this flight be held against a clean one band for band.
To score a clean flight the same way, build its truth with the lights on at
full (ambient_scale 1, diffuse_scale 1, dead []) - the world comes out
identical and the truth carries `light` - and pass it with --ground-truth.

The vehicle's own health is printed beside it: the downward camera reads the
ArUco markers and the floor's optical flow, and both see the same darkness.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from warehouse_model import GROUND_TRUTH, REPO_ROOT       # noqa: E402

OUT = REPO_ROOT / "out"
BANDS = (0.0, 0.25, 0.5, 1.0, 2.0, 4.0, float("inf"))


def load(path: Path):
    return json.loads(path.read_text()) if path.exists() else None


def ids(body) -> set:
    return {str(i.get("id")) for i in (body or {}).get("items", [])}


def band_of(e: float) -> int:
    for i in range(len(BANDS) - 1):
        if BANDS[i] <= e < BANDS[i + 1]:
            return i
    return len(BANDS) - 2


def build(truth_path: Path, out_dir: Path) -> dict:
    truth = json.loads(truth_path.read_text())
    codes = truth["codes"]
    if not any("light" in c for c in codes):
        return {"lit": False}
    qr_body = load(out_dir / "inventory_qr_checked.json") or load(out_dir / "inventory_scanned.json")
    qr, bc = ids(qr_body), ids(load(out_dir / "inventory_barcode.json"))
    cartons = ids(load(out_dir / "box_report.json"))
    have = {"qr": qr_body is not None, "barcode": bool(bc), "carton": bool(cartons)}

    rows = [{"band": f"{BANDS[i]:g}-{BANDS[i + 1]:g}", "qr": [0, 0], "barcode": [0, 0],
             "carton": [0, 0], "faces": {}} for i in range(len(BANDS) - 1)]
    for c in codes:
        light = c.get("light")
        if not light:
            continue
        r = rows[band_of(light["E"])]
        kind = c["type"]
        if kind == "box_qr":
            r["qr"][1] += 1
            r["qr"][0] += c["payload"] in qr
            r["carton"][1] += 1
            r["carton"][0] += c["payload"] in cartons
            f = r["faces"].setdefault(c["row"], [0, 0])
            f[1] += 1
            f[0] += c["payload"] in qr
        elif kind == "box_placard":
            r["barcode"][1] += 1
            r["barcode"][0] += c["payload"] in bc
        elif kind == "box_unlabelled":
            r["carton"][1] += 1
            r["carton"][0] += c["payload"] in cartons

    scan = load(out_dir / "inventory_scanned.json") or {}
    nav = {"waypoints_completed": scan.get("waypoints_completed"),
           "marker_corrections": len(scan.get("marker_corrections") or []),
           "final_drift_offset_m": scan.get("final_drift_offset_m")}
    return {"lit": True, "lighting": truth.get("lighting"), "readers": have,
            "rows": [r for r in rows if r["qr"][1] or r["carton"][1]], "navigation": nav}


def pct(hit_of) -> str:
    a, n = hit_of
    return f"{a:>3}/{n:<3} {100 * a / n:5.1f}%" if n else f"{'-':>14}"


def render(data: dict) -> str:
    if not data.get("lit"):
        return ("this ground truth carries no `light` tags: build it with "
                "lights_stress enabled (full scales and no dead lamps gives the "
                "clean world with the tags)")
    lt = data.get("lighting") or {}
    lines = ["what darkness cost, by the light each label got (model units; "
             "the clean world runs 0.55-8.3)"]
    if lt:
        lines.append(f"  ambient {lt['ambient'][:3]}, lamps {lt['diffuse'][:3]}, "
                     f"dark lamps {lt['dead_lamps']}")
    lines.append("")
    head = f"{'light E':<10} {'QR':>14}  {'barcode':>14}  {'carton':>14}   QR by face"
    lines += [head, "-" * len(head)]
    for r in data["rows"]:
        faces = " ".join(f"{f}:{a}/{n}" for f, (a, n) in sorted(r["faces"].items()))
        lines.append(f"{r['band']:<10} {pct(r['qr']):>14}  {pct(r['barcode']):>14}  "
                     f"{pct(r['carton']):>14}   {faces}")
    n = data["navigation"]
    lines += ["", f"vehicle: waypoints {n['waypoints_completed']}, marker fixes "
              f"{n['marker_corrections']}, final drift offset {n['final_drift_offset_m']}"]
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--out-dir", type=Path, default=OUT)
    ap.add_argument("--json", type=Path, default=None)
    args = ap.parse_args()
    data = build(args.ground_truth, args.out_dir)
    print(render(data))
    if data.get("lit"):
        path = args.json or args.out_dir / "light_report.json"
        path.write_text(json.dumps(data, indent=2, ensure_ascii=False))
        print(f"\nwritten: {path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
