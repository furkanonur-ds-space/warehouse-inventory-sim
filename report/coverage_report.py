#!/usr/bin/env python3
"""
Hierarchical coverage: which parts of the warehouse the scan actually covered.

    python3 report/coverage_report.py
    python3 report/coverage_report.py --missed
    python3 report/coverage_report.py --html out/coverage.html

    Warehouse -> aisle -> shelf face -> level -> bay -> box

Every node is scanned, partly scanned, or not scanned. A run that reads 98% of
the warehouse still fails its job if the 2% is one whole bay, and a single
percentage cannot say which it is.

A PASS is one (shelf face, level) pair - one flight down one face at one
altitude. The count comes from the layout, not from a constant: eight faces by
three levels is 24 passes in this warehouse.

Coverage is counted on the box QR payload, which is the only identifier unique
per box. The Code 128 placards repeat across boxes in the same location, so
counting those would over-count.

The aisle each face belongs to is derived from the layout as well. A hand
written "A, B -> 1" table would go quietly wrong the first time the racking
moved.

No flight and no simulator: reads `out/inventory_scanned.json` and
`warehouse/ground_truth.json`.
"""

from __future__ import annotations

import argparse
import json
from collections import defaultdict
from pathlib import Path

from warehouse_model import (CONFIG, GROUND_TRUTH, INVENTORY, REPO_ROOT,
                             load_config, load_ground_truth, load_run)

FULL, PARTIAL, EMPTY = "full", "partial", "none"
LABEL = {FULL: "scanned", PARTIAL: "partly scanned", EMPTY: "not scanned"}
MARK = {FULL: "[x]", PARTIAL: "[~]", EMPTY: "[ ]"}


def status_of(scanned: int, total: int) -> str:
    if total == 0 or scanned == 0:
        return EMPTY
    return FULL if scanned == total else PARTIAL


def row_to_aisle(cfg: dict) -> dict[str, int]:
    """
    Shelf face -> aisle, by nearest aisle centre to the face.

    Both are in the config frame here, so no world rotation is involved: the
    question is which aisle a face looks into, and that is unchanged by
    rotating the whole building.
    """
    r = cfg["racking"]
    out = {}
    for row in r["rows"]:
        face = row["y0"] + r["depth"] if row["facing"] > 0 else row["y0"]
        out[row["id"]] = min(r["aisles"],
                             key=lambda a: abs(a["y_center"] - face))["id"]
    return out


def build_tree(truth: list, scanned: set, aisle_of: dict) -> dict:
    tree: dict = {"total": 0, "scanned": 0, "aisles": {}}
    for c in truth:
        a = aisle_of.get(c["row"], 0)
        hit = c["payload"] in scanned
        node_a = tree["aisles"].setdefault(a, {"total": 0, "scanned": 0, "faces": {}})
        node_f = node_a["faces"].setdefault(
            c["row"], {"total": 0, "scanned": 0, "levels": {}})
        node_l = node_f["levels"].setdefault(
            c["level"], {"total": 0, "scanned": 0, "bays": {}})
        node_b = node_l["bays"].setdefault(c["bay"], {"total": 0, "scanned": 0})
        for n in (tree, node_a, node_f, node_l, node_b):
            n["total"] += 1
            n["scanned"] += hit
    return tree


def annotate(node: dict) -> dict:
    node["status"] = status_of(node["scanned"], node["total"])
    node["pct"] = round(100 * node["scanned"] / node["total"], 1) if node["total"] else 0.0
    for key in ("aisles", "faces", "levels", "bays"):
        for child in node.get(key, {}).values():
            annotate(child)
    return node


def write_html(tree: dict, path: Path, hit: int, total: int, source: str) -> None:
    """One self-contained file, no external assets - it opens from a mail attachment."""
    color = {FULL: "#2e7d32", PARTIAL: "#f9a825", EMPTY: "#c62828"}
    p = ['<meta charset="utf-8"><title>Coverage Report</title>',
         '<style>body{font-family:system-ui,sans-serif;margin:2rem;background:#fafafa;color:#222}'
         'h1{font-size:1.4rem}h2{font-size:1.05rem;margin-top:1.6rem}'
         'table{border-collapse:collapse;margin:.5rem 0}'
         'td,th{border:1px solid #ddd;padding:.35rem .6rem;font-size:.9rem;text-align:center}'
         'th{background:#f0f0f0}.b{color:#fff;border-radius:3px;padding:.15rem .4rem;font-size:.8rem}'
         '.sum{font-size:1.1rem;margin:.6rem 0}.src{color:#666;font-size:.85rem}</style>',
         '<h1>Warehouse coverage</h1>',
         f'<p class="sum"><b>{hit}/{total}</b> boxes scanned '
         f'({100*hit/total:.1f}%)</p>' if total else '',
         f'<p class="src">{source}</p>']
    for aid, a in sorted(tree["aisles"].items()):
        p.append(f'<h2>Aisle {aid} &mdash; {a["scanned"]}/{a["total"]} ({a["pct"]}%)</h2>')
        for fid, f in sorted(a["faces"].items()):
            bays = sorted({b for l in f["levels"].values() for b in l["bays"]})
            p.append(f'<b>Shelf face {fid}</b> &mdash; {f["scanned"]}/{f["total"]} '
                     f'<span class="b" style="background:{color[f["status"]]}">'
                     f'{LABEL[f["status"]]}</span>')
            p.append('<table><tr><th>level</th>'
                     + "".join(f"<th>bay {b}</th>" for b in bays)
                     + '<th>level total</th></tr>')
            for lid in sorted(f["levels"]):
                l = f["levels"][lid]
                cells = []
                for b in bays:
                    n = l["bays"].get(b)
                    cells.append("<td>-</td>" if n is None else
                                 f'<td style="background:{color[n["status"]]};color:#fff">'
                                 f'{n["scanned"]}/{n["total"]}</td>')
                cells.append(f'<td>{l["scanned"]}/{l["total"]} ({LABEL[l["status"]]})</td>')
                p.append(f'<tr><th>L{lid}</th>' + "".join(cells) + '</tr>')
            p.append('</table>')
    path.write_text("\n".join(p))


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--inventory", type=Path, default=INVENTORY)
    ap.add_argument("--ground-truth", type=Path, default=GROUND_TRUTH)
    ap.add_argument("--config", type=Path, default=CONFIG)
    ap.add_argument("--json", type=Path,
                    default=REPO_ROOT / "out" / "coverage_report.json")
    ap.add_argument("--html", type=Path,
                    help="also write a visual report here, e.g. out/coverage.html")
    ap.add_argument("--missed", action="store_true",
                    help="list every box that was not scanned")
    args = ap.parse_args()

    truth_by_payload = load_ground_truth(args.ground_truth)
    truth = list(truth_by_payload.values())
    if not truth:
        print("no box_qr entries in ground truth.")
        return 1

    cfg = load_config(args.config)
    aisle_of = row_to_aisle(cfg)
    run = load_run(args.inventory)

    decoded = {it["id"] for it in run.get("items", [])}
    scanned = decoded & set(truth_by_payload)
    unknown = decoded - set(truth_by_payload)

    rows = sorted({c["row"] for c in truth})
    levels = sorted({c["level"] for c in truth})
    tree = annotate(build_tree(truth, scanned, aisle_of))

    cell = defaultdict(lambda: [0, 0])
    for c in truth:
        cell[(c["row"], c["level"])][0] += 1
        if c["payload"] in scanned:
            cell[(c["row"], c["level"])][1] += 1

    total, hit = len(truth), len(scanned)

    print("COVERAGE REPORT")
    print(f"source: {args.inventory}  ({len(decoded)} decoded, {hit} matched to a box)")
    print("aisles: " + " ; ".join(f"{r} -> {aisle_of[r]}" for r in rows) + "\n")

    print("pass matrix  (scanned / total boxes):")
    print("        " + "   ".join(f"L{l}".center(7) for l in levels) + "    face")
    for r in rows:
        parts, rtot, rsc = [], 0, 0
        for l in levels:
            t, s = cell[(r, l)]
            rtot += t; rsc += s
            parts.append(f"{s:2d}/{t:<2d}".center(7))
        print(f"  {r}    " + "   ".join(parts) + f"    {rsc:2d}/{rtot}")
    print(f"\n  TOTAL: {hit}/{total} boxes  ({100*hit/total:.0f}%)")

    print("\nhierarchy:")
    print(f"  {MARK[tree['status']]} Warehouse            {tree['scanned']:3d}/{tree['total']:<3d} "
          f"{LABEL[tree['status']]}")
    for aid, a in sorted(tree["aisles"].items()):
        print(f"    {MARK[a['status']]} Aisle {aid}            {a['scanned']:3d}/{a['total']:<3d} "
              f"{LABEL[a['status']]}")
        for fid, f in sorted(a["faces"].items()):
            print(f"      {MARK[f['status']]} Shelf face {fid}    {f['scanned']:3d}/{f['total']:<3d} "
                  f"{LABEL[f['status']]}")
            for lid, l in sorted(f["levels"].items()):
                bad = [f"bay {b}" for b, n in sorted(l["bays"].items())
                       if n["status"] != FULL]
                note = ("  missing: " + ", ".join(bad)) if bad else ""
                print(f"        {MARK[l['status']]} Level {lid}      {l['scanned']:3d}/{l['total']:<3d} "
                      f"{LABEL[l['status']]}{note}")

    full, partial, empty = [], [], []
    for r in rows:
        for l in levels:
            t, s = cell[(r, l)]
            if t == 0:
                continue
            tag = f"{r}-L{l}"
            (full if s == t else empty if s == 0 else partial).append(
                tag if s in (0, t) else f"{tag} ({s}/{t})")
    print(f"\npasses ({len(full)+len(partial)+len(empty)}):")
    print(f"  complete : {', '.join(full) or '-'}")
    print(f"  partial  : {', '.join(partial) or '-'}")
    print(f"  none     : {', '.join(empty) or '-'}")

    if unknown:
        print(f"\nWARNING: {len(unknown)} decoded payloads are not in ground truth "
              f"(misdecode, or a texture rendered where it should not be):")
        for p in sorted(unknown)[:10]:
            print(f"  {p!r}")

    if args.missed:
        gone = [c for c in truth if c["payload"] not in scanned]
        print(f"\nnot scanned ({len(gone)}):")
        for c in sorted(gone, key=lambda c: (c["row"], c["level"], c["bay"])):
            print(f"  {c['row']}-{c['bay']:02d}-L{c['level']}  {c['payload']}")

    args.json.parent.mkdir(parents=True, exist_ok=True)
    args.json.write_text(json.dumps({
        "inventory": str(args.inventory),
        "scan_date": run.get("scan_date"),
        "total": total, "scanned": hit,
        "coverage_pct": round(100 * hit / total, 1) if total else 0.0,
        "unknown_payloads": sorted(unknown),
        "passes": {"complete": full, "partial": partial, "none": empty},
        "tree": tree,
    }, indent=2, ensure_ascii=False))
    print(f"\nreport: {args.json}")

    if args.html:
        args.html.parent.mkdir(parents=True, exist_ok=True)
        write_html(tree, args.html, hit, total, f"{args.inventory.name} · "
                   f"{run.get('scan_date', 'unknown date')}")
        print(f"html  : {args.html}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
