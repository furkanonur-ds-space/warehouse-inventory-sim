#!/usr/bin/env python3
"""
Gölgeli deponun ürettiği dünyayı ve yer gerçeğindeki gölgeyi denetler.

    .venv/bin/python warehouse/test_shadows.py

Simülatör yok. Dünya bellekte üretilir: gölge kapalı, bütün lambalar gölgeli,
ve yalnız iki lamba gölgeli.

  1. Gölge kapalıyken dünya ve yer gerçeği, shadows_stress hiç yokmuş gibi
     harfi harfine aynı. Temiz dünya bozulmasın diye şart.
  2. Gölge düşüren lambalar SDF'te tam olarak seçilenler.
  3. Engellenen lambalar, SDF'in kendisinden DÜNYA koordinatında okunan
     çarpışma kutularına karşı, ışını 1 cm adımla yürüterek yeniden bulunur
     ve yer gerçeğindekiyle aynı çıkar. stress.blocked çağrılmaz: dilim testi
     ile adım adım yürüyüş iki ayrı yöntem, aynı hatayı yapmazlar.
  4. Hiçbir etiket temiz dünyadakinden fazla ışık almıyor; E_min <= E <= E_max;
     gölge düşürmeyen bir lamba hiçbir etiketten kesilmiyor; en üst katın
     etiketleri (üstlerinde tabla yok) alt katlarınkinden aydınlık; tabla
     kenarı bir kısım etiketi ikiye bölüyor.
"""
from __future__ import annotations

import contextlib
import copy
import io
import math
import random
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import gen_labels as gl  # noqa: E402
import gen_world as gw   # noqa: E402

failures: list[str] = []


def check(ok: bool, what: str) -> None:
    if not ok:
        failures.append(what)


def build(cfg):
    with contextlib.redirect_stdout(io.StringIO()):
        sdf, man, _ = gw.build(cfg)
    return sdf, man


def lamps_of(sdf: str) -> dict:
    out = {}
    for m in re.finditer(r'<light type="point" name="ceiling_(\d+)">(.*?)</light>', sdf, re.S):
        body = m.group(2)
        pos = [float(v) for v in re.search(r"<pose>([^<]+)</pose>", body).group(1).split()[:3]]
        shadow = re.search(r"<cast_shadows>(\w+)</cast_shadows>", body).group(1) == "true"
        rng = float(re.search(r"<range>([^<]+)</range>", body).group(1))
        out[int(m.group(1))] = {"pos": pos, "shadow": shadow, "range": rng}
    return out


def boxes_of(sdf: str) -> tuple[np.ndarray, np.ndarray]:
    """Bütün çarpışma kutuları, modelin pozuyla dünyaya çevrilmiş (yalnız yaw)."""
    lo, hi = [], []
    for m in re.finditer(r'<model name="[^"]+">(.*?)</model>', sdf, re.S):
        body = m.group(1)
        mp = re.match(r"\s*<static>true</static>\s*<pose>([^<]+)</pose>", body)
        yaw = float(mp.group(1).split()[5]) if mp else 0.0
        c, s = math.cos(yaw), math.sin(yaw)
        for col in re.finditer(r"<collision name=\"[^\"]+\">\s*<pose>([^<]+)</pose>\s*"
                               r"<geometry><box><size>([^<]+)</size>", body):
            x, y, z = (float(v) for v in col.group(1).split()[:3])
            sx_, sy_, sz_ = (float(v) for v in col.group(2).split())
            cx, cy = c * x - s * y, s * x + c * y
            hx = abs(c) * sx_ / 2 + abs(s) * sy_ / 2
            hy = abs(s) * sx_ / 2 + abs(c) * sy_ / 2
            lo.append((cx - hx, cy - hy, z - sz_ / 2))
            hi.append((cx + hx, cy + hy, z + sz_ / 2))
    return np.array(lo), np.array(hi)


def walk_blocked(p, q, lo, hi, step=0.01) -> bool:
    p, q = np.asarray(p), np.asarray(q)
    n = max(2, int(np.linalg.norm(q - p) / step))
    t = np.linspace(0.0, 1.0, n)[1:-1]
    pts = p + (q - p) * t[:, None]
    # Önce kabaca: parçanın kutusuna değen kutular.
    a, b = np.minimum(p, q), np.maximum(p, q)
    near = np.all((hi >= a) & (lo <= b), axis=1)
    L, H = lo[near], hi[near]
    if not len(L):
        return False
    inside = np.all((pts[:, None, :] > L[None]) & (pts[:, None, :] < H[None]), axis=2)
    return bool(inside.any())


def main() -> int:
    base = gl._load_cfg(HERE / "warehouse.yaml")
    base["stress"]["enabled"] = False
    base["lights_stress"]["enabled"] = False
    none = copy.deepcopy(base)
    none.pop("shadows_stress", None)
    off = copy.deepcopy(base)
    off["shadows_stress"] = {"enabled": False, "lamps": "all"}
    allc = copy.deepcopy(base)
    allc["shadows_stress"] = {"enabled": True, "lamps": "all"}
    two = copy.deepcopy(base)
    two["shadows_stress"] = {"enabled": True, "lamps": [1, 5]}

    sdf_none, man_none = build(none)
    sdf_off, man_off = build(off)
    sdf_all, man_all = build(allc)
    sdf_two, man_two = build(two)

    # 1
    check(sdf_off == sdf_none and man_off == man_none, "gölge kapalıyken dünya değişmiş")
    check(all("light" not in c for c in man_off), "gölge kapalıyken light alanı yazılmış")

    # 2
    L_all, L_two = lamps_of(sdf_all), lamps_of(sdf_two)
    check(len(L_all) == 16 and all(v["shadow"] for v in L_all.values()),
          "bütün lambalar gölgeli değil")
    check({i for i, v in L_two.items() if v["shadow"]} == {1, 5},
          f"gölgeli lambalar {sorted(i for i, v in L_two.items() if v['shadow'])}, beklenen [1, 5]")
    check([v["pos"] for v in L_all.values()] == [v["pos"] for v in lamps_of(sdf_none).values()],
          "lambalar yer değiştirmiş")

    # 3
    lo, hi = boxes_of(sdf_all)
    qrs = [c for c in man_all if c["type"] == "box_qr"]
    rng = random.Random(4)
    sample = rng.sample(qrs, 40)
    rays = disagree = 0
    for c in sample:
        p = np.array(c["label_pose_xyzrpy"][:3]) + 0.002 * np.array(c["normal"])
        want = set()
        for i, lamp in L_all.items():
            v = np.array(lamp["pos"]) - p
            d = float(np.linalg.norm(v))
            if d >= lamp["range"] or float(np.array(c["normal"]) @ v) <= 0:
                continue
            rays += 1
            if walk_blocked(p, lamp["pos"], lo, hi):
                want.add(i)
        got = set(c["light"]["lamps_blocked"])
        disagree += len(want ^ got)
    # 1 cm adım bir kutunun köşesini sıyıran ışını kaçırabilir; o kadar pay.
    check(disagree <= max(1, rays // 100),
          f"engellenen lambalar {disagree}/{rays} ışında SDF'ten bulunandan farklı")

    # 4
    for man, name in ((man_all, "hepsi"), (man_two, "iki lamba")):
        lab = [c for c in man if c["type"] in ("box_qr", "box_placard")]
        check(all(c["light"]["E"] <= c["light"]["E_clean"] + 1e-9 for c in lab),
              f"{name}: bir etiket temizden fazla ışık alıyor")
        check(all(c["light"]["E_min"] - 1e-9 <= c["light"]["E"] <= c["light"]["E_max"] + 1e-9
                  for c in lab), f"{name}: E, E_min..E_max dışında")
    # Gölge ve loşluk birlikte: E_open loşluğu taşır ama gölgeyi taşımaz.
    both = copy.deepcopy(allc)
    both["lights_stress"].update(enabled=True, ambient_scale=0.5, diffuse_scale=0.75, dead=[0, 6])
    _, man_both = build(both)
    lab_b = [c for c in man_both if c["type"] == "box_qr"]
    check(all(c["light"]["E"] <= c["light"]["E_open"] + 1e-9 < c["light"]["E_clean"] for c in lab_b),
          "gölge+loşluk: E <= E_open < E_clean tutmuyor")
    open_frac = sum(c["light"]["E"] >= 0.9 * c["light"]["E_open"] for c in lab_b) / len(lab_b)
    rel_frac = sum(c["light"]["rel"] >= 0.9 for c in lab_b) / len(lab_b)
    check(open_frac > 0.2 and rel_frac == 0.0,
          f"gölge+loşluk: tam ışıkta {open_frac:.2f} (E_open'a göre), {rel_frac:.2f} (rel'e göre)")

    cut_two = {i for c in man_two if "light" in c for i in c["light"].get("lamps_blocked", [])}
    check(cut_two <= {1, 5}, f"gölge düşürmeyen lamba kesilmiş: {sorted(cut_two - {1, 5})}")
    check(cut_two, "iki gölgeli lambanın hiçbiri hiçbir etiketten kesilmemiş")
    rel = {lv: np.median([c["light"]["rel"] for c in qrs if c["level"] == lv]) for lv in (1, 2, 3)}
    check(rel[3] > rel[1] and rel[3] > rel[2], f"üst kat alt katlardan karanlık: {rel}")

    edge = sum(1 for c in qrs if c["light"]["E_max"] > 1.5 * c["light"]["E_min"])
    # Köşeler gerçekten köşede: tabla kenarı etiketlerin bir kısmını ikiye
    # böler. Hepsi ortadan ölçülseydi E_min = E_max olurdu.
    check(edge >= 10, f"üstünden gölge kenarı geçen QR yalnız {edge}")
    print(f"{len(lo)} kutu; {rays} ışında {disagree} fark; QR ışık oranı kat 1/2/3: "
          f"{rel[1]:.2f}/{rel[2]:.2f}/{rel[3]:.2f}; üstünden gölge kenarı geçen QR {edge}")
    if failures:
        for f in failures:
            print("  HATA:", f)
        print(f"{len(failures)} kontrol başarısız")
        return 1
    print("bütün kontroller geçti")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
