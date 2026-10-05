#!/usr/bin/env python3
"""
Karartılmış deponun ürettiği dünyayı ve yer gerçeğindeki ışığı denetler.

    .venv/bin/python warehouse/test_lights.py

Simülatör yok. Dünya bellekte üç kez üretilir: ışık bozulması kapalı, tam
güçte açık (kısma yok, sönük lamba yok) ve warehouse.yaml'daki ayarla.

  1. Tam güçte açık dünya, kapalı dünyayla harfi harfine aynı SDF'i verir;
     yer gerçekleri de `light` alanı dışında aynıdır. Temiz bir uçuşu ışığa
     göre puanlamanın yolu bu olduğu için şart.
  2. Sönük lambalar sahnede yok, yananlar kısılmış, ortam ışığı kısılmış.
  3. Yer gerçeğindeki ışık, lambaların SDF'teki DÜNYA konumundan ve
     etiketin dünyadaki yerinden, buradaki ayrı bir formülle yeniden
     hesaplanınca aynı çıkar. stress.illuminance çağrılmaz: aynı hatayı iki
     kez yapan iki hesap birbirini onaylar.
  4. Hiçbir etiket temiz dünyadakinden fazla ışık almıyor; ışığının çoğunu
     veren lamba sönük olan etiketler, o lambası yanan etiketlerden karanlıkta.
     EN YAKIN lamba değil: gölge kapalı, ışık rafların içinden geçer ve bir
     etiket tepesindeki lambayı neredeyse yandan görür, ışığının çoğunu
     karşıdaki koridorun lambalarından alır.
"""
from __future__ import annotations

import contextlib
import copy
import io
import math
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


def lights_of(sdf: str) -> dict:
    out = {}
    for m in re.finditer(r'<light type="point" name="ceiling_(\d+)">(.*?)</light>', sdf, re.S):
        body = m.group(2)
        out[int(m.group(1))] = {
            "pos": [float(v) for v in re.search(r"<pose>([^<]+)</pose>", body).group(1).split()[:3]],
            "diffuse": [float(v) for v in re.search(r"<diffuse>([^<]+)</diffuse>", body).group(1).split()],
            "att": [float(re.search(f"<{k}>([^<]+)</{k}>", body).group(1))
                    for k in ("range", "constant", "linear", "quadratic")],
        }
    return out


def scene_ambient(sdf: str) -> list[float]:
    return [float(v) for v in re.search(r"<scene>.*?<ambient>([^<]+)</ambient>", sdf, re.S).group(1).split()]


def light_at(p, n, lamps, ambient) -> float:
    """Ayrı yazılmış model: ortam + her lamba için kosinüs / sönüm."""
    e = sum(ambient[:3]) / 3
    for lamp in lamps.values():
        rng, c, l, q = lamp["att"]
        v = np.array(lamp["pos"]) - np.array(p)
        d = math.sqrt(float(v @ v))
        if d >= rng:
            continue
        cos = max(0.0, float(np.array(n) @ v) / d)
        e += sum(lamp["diffuse"][:3]) / 3 * cos / (c + l * d + q * d * d)
    return e


def main() -> int:
    base = gl._load_cfg(HERE / "warehouse.yaml")
    base["stress"]["enabled"] = False
    off = copy.deepcopy(base)
    off.setdefault("lights_stress", {})["enabled"] = False
    full = copy.deepcopy(base)
    full["lights_stress"].update(enabled=True, ambient_scale=1.0, diffuse_scale=1.0, dead=[])
    full["lights_stress"]["dead_fraction"] = 0.0
    dim = copy.deepcopy(base)
    dim["lights_stress"]["enabled"] = True
    st = dim["lights_stress"]

    sdf_off, man_off = build(off)
    sdf_full, man_full = build(full)
    sdf_dim, man_dim = build(dim)

    # 1
    check(sdf_full == sdf_off, "tam güçteki ışık dünyası temiz dünyadan farklı")
    strip = lambda m: [{k: v for k, v in c.items() if k != "light"} for c in m]   # noqa: E731
    check(strip(man_full) == man_off, "yer gerçeği light alanı dışında da değişmiş")
    check(all("light" not in c for c in man_off), "kapalı dünyada light alanı var")

    # 2
    L_off, L_dim = lights_of(sdf_off), lights_of(sdf_dim)
    dead = set(st["dead"])
    check(set(L_dim) == set(L_off) - dead, f"yanan lambalar {sorted(L_dim)}, beklenen {sorted(set(L_off) - dead)}")
    for i, lamp in L_dim.items():
        want = [round(v * st["diffuse_scale"], 6) for v in L_off[i]["diffuse"][:3]]
        check(np.allclose(lamp["diffuse"][:3], want, atol=1e-5), f"lamba {i} kısılmamış: {lamp['diffuse']}")
        check(lamp["pos"] == L_off[i]["pos"], f"lamba {i} yer değiştirmiş")
    a_off, a_dim = scene_ambient(sdf_off), scene_ambient(sdf_dim)
    check(np.allclose(a_dim[:3], np.array(a_off[:3]) * st["ambient_scale"], atol=1e-5),
          f"ortam ışığı {a_dim}, beklenen {a_off[:3]} x {st['ambient_scale']}")

    # 3
    labels = [c for c in man_dim if c["type"] == "box_qr"]
    worst = 0.0
    for c in labels:
        p, n = c["label_pose_xyzrpy"][:3], c["normal"]
        e = light_at(p, n, L_dim, a_dim)
        e0 = light_at(p, n, L_off, a_off)
        worst = max(worst, abs(e - c["light"]["E"]), abs(e0 - c["light"]["E_clean"]))
    check(worst < 2e-3, f"yer gerçeğindeki ışık, SDF'ten hesaplanandan {worst:.4f} sapıyor")

    # 4
    check(all(c["light"]["E"] <= c["light"]["E_clean"] + 1e-9 for c in labels),
          "bir etiket temiz dünyadakinden fazla ışık alıyor")
    def main_lamp(c):
        p, n = c["label_pose_xyzrpy"][:3], c["normal"]
        return max(L_off, key=lambda i: light_at(p, n, {i: L_off[i]}, [0, 0, 0]))
    dark = [c["light"]["rel"] for c in labels if main_lamp(c) in dead]
    lit = [c["light"]["rel"] for c in labels if main_lamp(c) not in dead]
    check(dark and lit and np.median(dark) < np.median(lit),
          f"asıl lambası sönük olanlar daha aydınlık: {np.median(dark):.2f} / {np.median(lit):.2f}")

    es = sorted(c["light"]["E"] for c in labels)
    print(f"{len(L_dim)} lamba yanıyor, {len(dead)} sönük; QR ışığı {es[0]:.2f}..{es[-1]:.2f}, "
          f"model ile SDF arası en çok {worst:.5f}; sönük lamba altında oran "
          f"{np.median(dark):.2f}, yanık altında {np.median(lit):.2f}")
    if failures:
        for f in failures:
            print("  HATA:", f)
        print(f"{len(failures)} kontrol başarısız")
        return 1
    print("bütün kontroller geçti")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
