#!/usr/bin/env python3
"""
Every visual report of the run in out/, as one HTML page with buttons.

    python3 report/report_page.py                 # writes out/report.html
    python3 report/report_page.py --keep-parts    # and leaves the pages it read

A pure consumer, like the rest of report/. It does not draw anything itself:
the 3D viewers, the coverage tree, the drift and YOLO pages are written by
their own tools as before, and this embeds each one whole. A page is held as
text and only handed to its frame when its button is first pressed, so the
seven canvases do not all render at once and a page opens exactly as it
would on its own.

Which pages exist depends on the flight - no barcode reader, no barcode
pages; no box log, no carton pages - so the buttons are built from what is
in out/, not from a fixed list. A page that is missing is left out rather
than shown empty.

Once the page is written the parts it embedded are removed, so a run leaves
one HTML file instead of eight. They are rebuilt by make_reports.sh on every
run; --keep-parts leaves them in place.

The header carries the run's headline numbers, read from the JSON reports
the other tools wrote. A number whose report is missing is left out, never
shown as zero.
"""

from __future__ import annotations

import argparse
import html
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

# (file, button group, button label, what the page shows). The text is for
# whoever opens the page after a flight, so it says what to look at, not how
# the page was made.
PAGES = [
    ("inventory_3d.html", "QR", "QR envanteri (3B)",
     "Okunan her kutu QR'ı, uçuşun hesapladığı konumda bir nokta olarak "
     "çizilir. Rengi gerçek konumdan uzaklığını gösterir. Hiç okunamayan "
     "kutular gerçek yerlerinde tel kafes kutu olarak durur. 15 cm'den fazla "
     "sapan okumalar ok ile gösterilir. Bir noktanın üstüne gelince kimliğini, "
     "konumunu ve hatasını görürsün."),
    ("coverage.html", "QR", "QR kapsama",
     "Depo → koridor → raf yüzü → seviye → göz → kutu ağacı. Her düğüm "
     "taranmış, kısmen taranmış ya da taranmamış olarak işaretli. %98 gibi "
     "tek bir oran, eksik %2'nin tek bir gözün tamamı olup olmadığını "
     "söyleyemez; bu ağaç söyler."),
    ("inventory_3d_barcode.html", "Barkod", "Barkod envanteri (3B)",
     "QR envanteriyle aynı görünüm, bu kez kutuların Code128 barkodu için. "
     "Barkod QR'dan bağımsız okunur ve kendi gerçek verisiyle puanlanır; "
     "biri okunup diğeri okunamayan kutular burada ayrışır."),
    ("coverage_barcode.html", "Barkod", "Barkod kapsama",
     "Barkodlar için kapsama ağacı: hangi yüz, seviye ve gözde barkod "
     "okunamadı. Kaçan barkodlar genelde dar koridordaki G ve H "
     "yüzlerinde toplanır."),
    ("boxes_3d.html", "Koliler", "Koliler – YOLO (3B)",
     "YOLO'nun uçuş sırasında gördüğü kolilerin karelerden 3B'ye "
     "yerleştirilmiş hâli; her biri en yakın gerçek koliyle eşlenmiştir. "
     "Kod okumaya bakmaz, rafta ne olduğunu sayar. Bulunamayan koliler "
     "gerçek yerlerinde tel kafes kutu olarak görünür."),
    ("experiment_3d.html", "Koliler", "Kodsuz koliler deneyi (3B)",
     "Yalnızca üzerinde hiç kod basılmamış koliler. Okunacak bir şeyleri "
     "olmadığı için onları yalnız YOLO bulabilir; bu sayfa onları bulup "
     "bulamadığını gösterir."),
    ("drift.html", "Uçuş", "Kayma (drift)",
     "İç mekânda GPS olmadığı için konum kestirimi zamanla kayar. Bu sayfa "
     "zemindeki ArUco markörleri her görüldüğünde ölçülen konum hatasını, "
     "düzeltmenin zaman içindeki seyrini ve hatanın bir yöne eğilimi olup "
     "olmadığını gösterir."),
    ("yolo.html", "Uçuş", "YOLO etiket bulucu",
     "YOLO kaç QR ve barkod etiketi buldu, okumaların kaçı tam kareden, "
     "kaçı YOLO'nun kırptığı parçadan geldi ve YOLO olmasa hangi kutular "
     "kaçacaktı. Çok görüp az okumak kod sorununa, hiç görmemek kadraj "
     "sorununa işaret eder."),
]


def _load(path: Path):
    try:
        return json.loads(path.read_text())
    except (OSError, ValueError):
        return None


def headline(out: Path) -> list[tuple[str, str]]:
    """(label, value) pairs for the header, only for reports that exist."""
    cells = []
    qr = _load(out / "validation_report.json")
    if qr:
        cells.append(("QR", f"{qr['detected']}/{qr['total_ground_truth']}"
                            f" · doğruluk %{qr['inventory_accuracy_pct']:g}"
                            f" · medyan {qr['position_error']['median_m']:.3f} m"))
    bc = _load(out / "validation_report_barcode.json")
    if bc:
        cells.append(("Barkod", f"{bc['detected']}/{bc['total_ground_truth']}"
                                f" · doğruluk %{bc['inventory_accuracy_pct']:g}"
                                f" · medyan {bc['position_error']['median_m']:.3f} m"))
    box = _load(out / "box_report.json")
    if box and box.get("cartons"):
        c = box["cartons"]
        cells.append(("Koliler", f"{c['found']}/{c['true_total']}"
                                 f" · sahte küme {c['spurious_clusters']}"))
    nav = _load(out / "navigation_report.json")
    if nav:
        wp = nav.get("waypoints", {})
        cells.append(("Waypoint", f"{wp.get('reached')}/{wp.get('planned')}"
                                  f" · {nav.get('mission_duration_s', 0):.0f} s"))
    drift = _load(out / "drift_report.json")
    if drift and drift.get("drift_at_fix"):
        cells.append(("Kayma", f"medyan {drift['drift_at_fix']['median_m']:.3f} m"
                               f" · son ofset {drift.get('final_offset_m', 0):.3f} m"))
    return cells


def scan_date(out: Path) -> str:
    for name in ("validation_report.json", "navigation_report.json"):
        d = _load(out / name)
        if d:
            return str(d.get("scan_date") or d.get("report_date") or "")
    return ""


def build(out: Path) -> tuple[str, list[Path]]:
    found = [(out / f, g, t, d) for f, g, t, d in PAGES if (out / f).is_file()]
    if not found:
        raise SystemExit(f"no report pages in {out}; run make_reports.sh first")

    groups: dict[str, list] = {}
    for i, (_, g, t, _) in enumerate(found):
        groups.setdefault(g, []).append((i, t))

    buttons = []
    for g, items in groups.items():
        row = "".join(f'<button data-i="{i}">{html.escape(t)}</button>' for i, t in items)
        buttons.append(f'<div class="grp"><span>{html.escape(g)}</span>{row}</div>')

    cells = "".join(f'<div class="cell"><b>{html.escape(k)}</b>{html.escape(v)}</div>'
                    for k, v in headline(out))
    # Each page travels as a JSON string. "</" is escaped so a page's own
    # </script> cannot close the tag that carries it.
    pages = json.dumps([p.read_text() for p, *_ in found]).replace("</", "<\\/")
    meta = json.dumps([{"title": t, "desc": d} for _, _, t, d in found])
    date = html.escape(scan_date(out))

    page = f"""<!doctype html>
<html lang="tr"><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Uçuş raporu {date}</title>
<style>
  html,body{{margin:0;height:100%;background:#11151c;color:#dfe6ef;
            font-family:system-ui,sans-serif}}
  body{{display:flex;flex-direction:column}}
  header{{padding:10px 14px 8px;border-bottom:1px solid #263041;background:#161c26}}
  h1{{font-size:16px;margin:0 0 8px;font-weight:600}}
  h1 small{{color:#8b98a9;font-weight:400;margin-left:8px}}
  .cells{{display:flex;flex-wrap:wrap;gap:6px 18px;font-size:12.5px;color:#b9c4d2;
          margin-bottom:9px}}
  .cell b{{color:#dfe6ef;margin-right:6px}}
  nav{{display:flex;flex-wrap:wrap;gap:6px 16px}}
  .grp{{display:flex;align-items:center;gap:5px;flex-wrap:wrap}}
  .grp span{{font-size:11px;text-transform:uppercase;letter-spacing:.06em;
             color:#7d8a9b;margin-right:2px}}
  button{{background:#222b39;color:#dfe6ef;border:1px solid #33405a;border-radius:6px;
          padding:5px 10px;font:inherit;font-size:13px;cursor:pointer}}
  button:hover{{background:#2a3547}}
  button.on{{background:#3d6ee0;border-color:#3d6ee0;color:#fff}}
  #desc{{font-size:13px;line-height:1.5;color:#c3ccd8;margin-top:9px;max-width:980px}}
  main{{flex:1;position:relative;min-height:0}}
  iframe{{position:absolute;inset:0;width:100%;height:100%;border:0;display:none;
          background:#11151c}}
  iframe.on{{display:block}}
</style>
<header>
  <h1>Uçuş raporu<small>{date}</small></h1>
  <div class="cells">{cells}</div>
  <nav>{''.join(buttons)}</nav>
  <div id="desc"></div>
</header>
<main id="m"></main>
<script id="pages" type="application/json">{pages}</script>
<script>
const PAGES = JSON.parse(document.getElementById("pages").textContent);
const META = {meta};
const frames = {{}};
function show(i) {{
  if (!frames[i]) {{
    const f = document.createElement("iframe");
    f.title = META[i].title;
    f.srcdoc = PAGES[i];
    document.getElementById("m").appendChild(f);
    frames[i] = f;
  }}
  for (const k in frames) frames[k].classList.toggle("on", +k === i);
  document.querySelectorAll("nav button").forEach(
    b => b.classList.toggle("on", +b.dataset.i === i));
  document.getElementById("desc").textContent = META[i].desc;
  try {{ history.replaceState(null, "", "#" + i); }} catch (e) {{}}
  frames[i].focus();
}}
document.querySelectorAll("nav button").forEach(
  b => b.addEventListener("click", () => show(+b.dataset.i)));
const start = parseInt(location.hash.slice(1), 10);
show(start >= 0 && start < PAGES.length ? start : 0);
</script>
</html>
"""
    return page, [p for p, *_ in found]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", type=Path, default=ROOT / "out",
                    help="the run's output folder (default out/)")
    ap.add_argument("--out", type=Path, help="default <dir>/report.html")
    ap.add_argument("--keep-parts", action="store_true",
                    help="leave the separate pages in place after embedding them")
    args = ap.parse_args()

    target = args.out or args.dir / "report.html"
    page, parts = build(args.dir)
    target.write_text(page)
    if not args.keep_parts:
        for p in parts:
            if p.resolve() != target.resolve():
                p.unlink()
    print(f"report page: {target}  ({len(parts)} views, {len(page)/1e6:.1f} MB)")
    for p in parts:
        print(f"  {p.name}{'' if args.keep_parts else '  (embedded, removed)'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
