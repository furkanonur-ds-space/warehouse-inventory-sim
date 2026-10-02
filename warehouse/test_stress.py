#!/usr/bin/env python3
"""
Zorlu senaryonun ürettiği dünyayı, onu üreten koddan bağımsız denetler.

    .venv/bin/python warehouse/test_stress.py

Simülatör yok, uçuş yok. Dünya iki kez bellekte üretilir, bozulma kapalı ve
açık, ve açık olan SDF'ten geri okunarak denetlenir. Kontroller stress.py'nin
fits()'ini ÇAĞIRMAZ: aynı hatayı iki kez yapan iki kontrol birbirini onaylar.
Koliler SDF metninden okunur, köşeleri burada yeniden hesaplanır.

Sorulanlar:
  1. Kapalıyken hiçbir bozulma izi yok (yer gerçeğinde `stress` alanı yok).
  2. Ana üretecin çekilişleri kaymadı: her kolinin SKU'su ve rengi açık ve
     kapalı dünyada aynı. Kayarsa iki uçuş karşılaştırılamaz.
  3. Bozulmasız koliler (`none`) temiz dünyadakiyle harfi harfine aynı.
  4. Hiçbir koli komşusuna, dikmeye ya da rafın arkasına girmiyor.
  5. Her koli tablaya oturuyor: ne gömülü ne havada.
  6. Hiçbir koli aracın payını min_side_clearance'ın altına indirmiyor.
  7. Etiketler kolinin ön yüzünde, onunla birlikte dönmüş.
  8. Yer gerçeğindeki etiket pozları SDF'teki etiketlerin dünyadaki yeri.
  9. Her bozulma, yazdığı büyüklükte uygulanmış (kayma kadar kaymış,
     dönme kadar dönmüş).
"""
from __future__ import annotations

import copy
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gen_labels as gl  # noqa: E402
import gen_world as gw   # noqa: E402

CONFIG = HERE / "warehouse.yaml"
LAYOUT = HERE.parent / "scanner" / "layout.json"
TOL = 2e-4               # SDF 6 anlamlı haneyle yazılıyor

failures: list[str] = []


def check(ok: bool, what: str) -> None:
    if not ok:
        failures.append(what)


def rpy_mat(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr]])


def links(sdf: str) -> dict[str, dict]:
    """Kolilerin SDF'teki hali: gövde pozu/boyu, etiket pozları, renk."""
    out = {}
    for m in re.finditer(r'<link name="(box_[^"]+)">(.*?)</link>', sdf, re.S):
        body = m.group(2)
        vis = dict(re.findall(r'<visual name="(\w+)">\s*<pose>([^<]+)</pose>', body))
        size = re.search(r"<size>([^<]+)</size>", body).group(1)
        diffuse = re.search(r"<diffuse>([^<]+)</diffuse>", body).group(1)
        out[m.group(1)] = {
            "text": m.group(0),
            "body": [float(v) for v in vis["body"].split()],
            "size": [float(v) for v in size.split()],
            "label": [float(v) for v in vis["label"].split()] if "label" in vis else None,
            "placard": [float(v) for v in vis["placard"].split()] if "placard" in vis else None,
            "diffuse": diffuse,
        }
    return out


def corners(body, size) -> np.ndarray:
    c, R = np.array(body[:3]), rpy_mat(*body[3:])
    s = np.array([[a, b, d] for a in (-1, 1) for b in (-1, 1) for d in (-1, 1)])
    return c + (s * np.array(size) / 2) @ R.T


def hull(pts):
    pts = sorted(map(tuple, pts))
    def half(seq):
        h = []
        for p in seq:
            while len(h) >= 2 and ((h[-1][0] - h[-2][0]) * (p[1] - h[-2][1])
                                   - (h[-1][1] - h[-2][1]) * (p[0] - h[-2][0])) <= 0:
                h.pop()
            h.append(p)
        return h[:-1]
    return np.array(half(pts) + half(pts[::-1]))


def overlap(a, b) -> bool:
    """İki dışbükey taban izi kesişiyor mu (ayırıcı eksen)."""
    for poly in (a, b):
        for i in range(len(poly)):
            e = poly[(i + 1) % len(poly)] - poly[i]
            n = np.array([-e[1], e[0]])
            pa, pb = a @ n, b @ n
            if pa.max() <= pb.min() + 1e-9 or pb.max() <= pa.min() + 1e-9:
                return False
    return True


def main() -> int:
    base_cfg = gl._load_cfg(CONFIG)
    off_cfg = copy.deepcopy(base_cfg)
    off_cfg.setdefault("stress", {})["enabled"] = False
    on_cfg = copy.deepcopy(base_cfg)
    on_cfg["stress"]["enabled"] = True

    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()):
        sdf_off, man_off, _ = gw.build(off_cfg)
        sdf_on, man_on, _ = gw.build(on_cfg)

    # 1
    check(not any("stress" in c for c in man_off), "kapalı dünyada stress alanı var")

    L_off, L_on = links(sdf_off), links(sdf_on)
    by_entity = {}
    for c in man_on:
        if c.get("entity", "").startswith("inventory::"):
            by_entity.setdefault(c["entity"].split("::")[1], []).append(c)

    # 2
    sku_off = {c["entity"]: c["caption"] for c in man_off if c["type"] in ("box_qr", "box_unlabelled")}
    sku_on = {c["entity"]: c["caption"] for c in man_on
              if c["type"] in ("box_qr", "box_unlabelled", "box_absent")}
    check(sku_off == sku_on, "SKU'lar kaydı: ana üretecin çekilişleri değişmiş")
    for name, rec in L_on.items():
        check(rec["diffuse"] == L_off[name]["diffuse"], f"{name}: rengi değişmiş")
    pc_off = {c["entity"]: c["payload"] for c in man_off if c["type"] == "box_placard"}
    for c in man_on:
        if c["type"] == "box_placard":
            check(pc_off.get(c["entity"]) == c["payload"],
                  f"{c['entity']}: barkod numarası kaydı")

    rk = base_cfg["racking"]
    rows = {r["id"]: r for r in rk["rows"]}
    depth, ft, bw, x0 = rk["depth"], rk["frame_thickness"], rk["bay_width"], rk["x_origin"]
    layout = json.loads(LAYOUT.read_text())
    half = layout["vehicle_half_span"]
    need = on_cfg["stress"].get("min_side_clearance", 0.05)
    standoff = {f["name"]: f.get("standoff", layout["shelf_standoff"]) for f in layout["aisle_faces"]}
    yaw_w = gw.world_yaw_rad(base_cfg)

    arms_seen = {}
    slots: dict[tuple, list] = {}
    for name, rec in L_on.items():
        _, rid, bay, lev, si = name.split("_")
        bay, lev = int(bay), int(lev)
        recs = by_entity[name]
        cell = recs[0]["stress"]
        arms_seen[cell["arm"]] = arms_seen.get(cell["arm"], 0) + 1
        row = rows[rid]
        facing = row["facing"]
        y_face = row["y0"] + depth if facing > 0 else row["y0"]
        k = corners(rec["body"], rec["size"])
        slots.setdefault((rid, bay, lev), []).append((name, hull(k[:, :2])))

        # 3
        if cell["arm"] == "none":
            check(rec["text"] == L_off[name]["text"], f"{name}: kontrol kolisi temiz dünyadan farklı")

        # 4 dikme ve raf arkası
        lo = x0 + (bay - 1) * bw + ft / 2
        hi = x0 + bay * bw - ft / 2
        check(k[:, 0].min() >= lo - TOL and k[:, 0].max() <= hi + TOL, f"{name}: dikmeye giriyor")
        if facing > 0:
            check(k[:, 1].min() >= row["y0"] - TOL, f"{name}: rafın arkasından taşıyor")
        else:
            check(k[:, 1].max() <= row["y0"] + depth + TOL, f"{name}: rafın arkasından taşıyor")

        # 5
        z = rk["level_heights"][lev - 1]
        check(abs(k[:, 2].min() - z) < 1e-3, f"{name}: tablaya oturmuyor "
              f"(alt {k[:, 2].min():.4f}, tabla {z:.4f})")

        # 6
        prot = (k[:, 1].max() - y_face) if facing > 0 else (y_face - k[:, 1].min())
        check(standoff[rid] - half - prot >= need - TOL,
              f"{name}: araç payı {standoff[rid] - half - prot:.3f} m")

        # 7 etiket kolinin kendi çerçevesinde: ön yüzün önünde, normali yüz normali
        c, R = np.array(rec["body"][:3]), rpy_mat(*rec["body"][3:])
        for key in ("label", "placard"):
            if rec[key] is None:
                continue
            local = R.T @ (np.array(rec[key][:3]) - c)
            want = facing * (rec["size"][1] / 2 + gw.LABEL_STANDOFF)
            check(abs(local[0]) < TOL and abs(local[1] - want) < TOL,
                  f"{name}/{key}: etiket ön yüzde değil ({local})")
            n_label = rpy_mat(*rec[key][3:]) @ np.array([0, 0, 1.0])
            check(np.allclose(R.T @ n_label, [0, facing, 0], atol=1e-4),
                  f"{name}/{key}: etiket koliyle dönmemiş")

        # 8 yer gerçeği = SDF etiketi, dünyaya çevrilmiş
        for r in recs:
            key = {"box_qr": "label", "box_placard": "placard",
                   "box_unlabelled": None}.get(r["type"])
            src = rec[key] if key else None
            if src is None:
                continue
            wx, wy = gw.rotate_xy(src[0], src[1], yaw_w)
            got = r["label_pose_xyzrpy"]
            check(abs(got[0] - wx) < 1e-3 and abs(got[1] - wy) < 1e-3 and abs(got[2] - src[2]) < 1e-3,
                  f"{name}/{r['type']}: yer gerçeği SDF ile tutmuyor")

        # 9 büyüklük
        off = L_off[name]["body"]
        d = np.array(rec["body"][:3]) - np.array(off[:3])
        v = cell["value"]
        a = cell["arm"]
        if a == "push_in":
            check(abs(d[1] + facing * v) < TOL, f"{name}: push_in {v} uygulanmamış ({d})")
        elif a == "pull_out":
            check(abs(d[1] - facing * v) < TOL, f"{name}: pull_out {v} uygulanmamış ({d})")
        elif a == "slide":
            check(abs(d[0] - v) < TOL, f"{name}: slide {v} uygulanmamış ({d})")
        elif a == "yaw":
            check(abs(math.degrees(rec["body"][5]) - v) < 1e-3, f"{name}: yaw {v} uygulanmamış")
        elif a == "tilt":
            check(abs(math.degrees(rec["body"][3]) - v) < 1e-3, f"{name}: tilt {v} uygulanmamış")

    # 4 komşular
    for key, items in slots.items():
        for i in range(len(items)):
            for j in range(i + 1, len(items)):
                check(not overlap(items[i][1], items[j][1]),
                      f"{items[i][0]} ile {items[j][0]} iç içe")

    absent = [c for c in man_on if c["type"] == "box_absent"]
    check(all(c["entity"].split("::")[1] not in L_on for c in absent),
          "boş göz kolisi SDF'te duruyor")
    arms_seen["empty"] = len(absent)

    n_box = len(L_on) + len(absent)
    print(f"{n_box} koli, {len(absent)} boş göz; bozulmalar: "
          + ", ".join(f"{a} {n}" for a, n in sorted(arms_seen.items())))
    if failures:
        for f in failures[:40]:
            print("  HATA:", f)
        print(f"{len(failures)} kontrol başarısız")
        return 1
    print("bütün kontroller geçti")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
