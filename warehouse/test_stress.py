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

Etiket kusurları ayrı bir grup olarak, aynı dünyanın ikinci bir üretiminde:
 10. Etiket kusuru yalnız etiketli koliye verilmiş.
 11. Dokusu bozulan kolinin iki dokusu da temiz dünyadakinden farklı;
     başka hiçbir dokuya dokunulmamış.
 12. Eğik etiket kolinin ön yüzünün içinde ve yazdığı açıda dönmüş.
 13. Eski etiketler yüzün içinde, asıl etiketlerin altında, onlara binmiyor
     ve depoda olmayan bir şey söylüyor.

Biçim ve renk, üçüncü bir üretimde:
 14. Kolinin modelinde her üçgen dışa bakıyor (ters sarılmış yüzü Gazebo
     çizmez, koli kameradan görünmez olurdu).
 15. Şişme ve çukur yazdığı derinlikte, yüzün kenarında sıfır; şişen yüz
     aracın payını min_side_clearance'ın altına indirmiyor.
 16. Kıvrık etiketin her köşesi yüzeyin LABEL_STANDOFF önünde: yüzü izliyor,
     ne içine giriyor ne havada duruyor.
 17. Beyaz ve koyu koli yazdığı renkte, baskılı koli dokulu; öbür her koli
     temiz dünyadakiyle harfi harfine aynı.
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
import stress as sx      # noqa: E402

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
        scale = {}
        for n, block in re.findall(r'<visual name="(\w+)">(.*?)</visual>', body, re.S):
            sc = re.search(r"<scale>([^<]+)</scale>", block)
            if sc:
                scale[n] = [float(v) for v in sc.group(1).split()[:2]]
        size = re.search(r"<size>([^<]+)</size>", body).group(1)
        diffuse = re.search(r"<diffuse>([^<]+)</diffuse>", body).group(1)
        out[m.group(1)] = {
            "text": m.group(0),
            "body": [float(v) for v in vis["body"].split()],
            "size": [float(v) for v in size.split()],
            "label": [float(v) for v in vis["label"].split()] if "label" in vis else None,
            "placard": [float(v) for v in vis["placard"].split()] if "placard" in vis else None,
            "diffuse": diffuse,
            "visuals": {n: [float(v) for v in pz.split()] for n, pz in vis.items()},
            "scale": scale,
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


def label_rects(rec):
    """Etiket quad'larının köşeleri, kolinin kendi yüz düzleminde (x, z)."""
    c, R = np.array(rec["body"][:3]), rpy_mat(*rec["body"][3:])
    out = {}
    for name, pose in rec["visuals"].items():
        if name == "body" or name not in rec["scale"]:
            continue
        w, h = rec["scale"][name]
        L = rpy_mat(*pose[3:])
        pts = []
        for sx_, sy_ in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
            p = np.array(pose[:3]) + L @ np.array([sx_ * w / 2, sy_ * h / 2, 0.0])
            q = R.T @ (p - c)
            pts.append((q[0], q[2]))
        out[name] = pts
    return out


def check_labels(cfg, sdf_off, tex_off, sdf_on, man_on, tex_on) -> dict:
    L_off, L_on = links(sdf_off), links(sdf_on)
    by_entity = {}
    for c in man_on:
        if c.get("entity", "").startswith("inventory::"):
            by_entity.setdefault(c["entity"].split("::")[1], []).append(c)
    real_payloads = {c["payload"] for c in man_on if c["type"] in ("box_qr", "box_placard")}
    seen = {}
    touched = set()
    for name, rec in L_on.items():
        recs = by_entity[name]
        cell = recs[0]["stress"]
        arm, v = cell["arm"], cell["value"]
        seen[arm] = seen.get(arm, 0) + 1
        _, rid, bay, lev, si = name.split("_")
        stem = f"{rid}{bay}{lev}{si}.png"
        bare = any(r["type"] == "box_unlabelled" for r in recs)
        # 10
        check(not (bare and arm != "none"), f"{name}: kodsuz koliye etiket kusuru ({arm})")
        # kolinin kendisi hiç oynamamış
        check(rec["body"] == L_off[name]["body"], f"{name}: etiket kusurunda koli oynamış")
        dx, _, dz = rec["size"]
        inside = lambda pts: all(-dx / 2 - TOL <= x <= dx / 2 + TOL and
                                 -dz / 2 - TOL <= z <= dz / 2 + TOL for x, z in pts)
        rects = label_rects(rec)
        # 11
        if arm in ("fade", "smudge", "tear", "wrinkle"):
            for t in (f"box_{stem}", f"placard_{stem}"):
                touched.add(t)
                check(tex_on[t].tobytes() != tex_off[t].tobytes(), f"{t}: {arm} dokuyu değiştirmemiş")
        # 12
        if arm == "skew":
            for key in ("label", "placard"):
                check(inside(rects[key]), f"{name}/{key}: eğik etiket yüzden taşıyor")
                a, b = rects[key][0], rects[key][1]
                got = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0]))
                # Koridora +y'de bakan yüzlerde dokunun u ekseni -x'e döşeli,
                # kenar 180 - açı çıkar: yataya göre katlanır.
                got = (got + 90.0) % 180.0 - 90.0
                check(abs(abs(got) - abs(v)) < 0.05, f"{name}/{key}: açı {got:.2f}, istenen {v}")
        elif arm != "decoy" and rec["label"] is not None:
            check(rec["label"] == L_off[name]["label"] and rec["placard"] == L_off[name]["placard"],
                  f"{name}: {arm} etiketi yerinden oynatmış")
        # 13
        decoys = [r for r in recs if r["type"] == "box_decoy"]
        want = {"none": 0, "decoy": {1: 1, 2: 1, 3: 2}.get(int(v), 0)}.get(arm, 0)
        check(len(decoys) == want, f"{name}: {len(decoys)} eski etiket, beklenen {want}")
        for d in decoys:
            check(d["payload"] not in real_payloads, f"{name}: eski etiket gerçek bir kodu taşıyor")
        if arm == "decoy":
            real_bottom = min(z for k in ("label", "placard") for _, z in rects[k])
            for key, pts in rects.items():
                if not key.startswith("decoy_"):
                    continue
                check(inside(pts), f"{name}/{key}: eski etiket yüzden taşıyor")
                check(max(z for _, z in pts) < real_bottom - 1e-4, f"{name}/{key}: asıl etiketlere biniyor")
                touched.add(f"decoy_{rid}{bay}{lev}{si}_{'qr' if key == 'decoy_qr' else 'pc'}.png")
            names = sorted(k for k in rects if k.startswith("decoy_"))
            if len(names) == 2:
                a, b = rects[names[0]], rects[names[1]]
                check(max(z for _, z in a) < min(z for _, z in b) or max(z for _, z in b) < min(z for _, z in a),
                      f"{name}: iki eski etiket üst üste")
    # 11, öbür yarısı: dokunulmayan her doku temiz dünyadakiyle aynı
    for t, img in tex_on.items():
        if t in touched:
            continue
        check(t in tex_off and img.tobytes() == tex_off[t].tobytes(), f"{t}: kusursuz kolinin dokusu değişmiş")
    return seen


def read_obj(text: str):
    v = np.array([[float(t) for t in l.split()[1:4]] for l in text.splitlines() if l.startswith("v ")])
    vt = np.array([[float(t) for t in l.split()[1:3]] for l in text.splitlines() if l.startswith("vt ")])
    f = [[int(t.split("/")[0]) - 1 for t in l.split()[1:4]] for l in text.splitlines() if l.startswith("f ")]
    return v, vt, f


def check_shapes(cfg, sdf_off, sdf_on, man_on, tex_on) -> dict:
    L_off, L_on = links(sdf_off), links(sdf_on)
    rows = {r["id"]: r for r in cfg["racking"]["rows"]}
    layout = json.loads(LAYOUT.read_text())
    half = layout["vehicle_half_span"]
    need = cfg["stress"].get("min_side_clearance", 0.05)
    standoff = {f["name"]: f.get("standoff", layout["shelf_standoff"]) for f in layout["aisle_faces"]}
    front_gap = cfg["boxes"]["front_gap"]
    tags = {c["entity"].split("::")[1]: c["stress"] for c in man_on if "stress" in c}
    seen = {}
    for name, rec in L_on.items():
        st = tags[name]
        arm = st["arm"]
        seen[arm] = seen.get(arm, 0) + 1
        _, rid, bay, lev, si = name.split("_")
        stem = f"{rid}{bay}{lev}{si}"
        facing = rows[rid]["facing"]
        dx, dy, dz = rec["size"]
        # 17
        if arm == "none":
            check(rec["text"] == L_off[name]["text"], f"{name}: kontrol kolisi değişmiş")
            continue
        check(rec["body"][:3] == L_off[name]["body"][:3], f"{name}: biçim bozulmasında koli yer değiştirmiş")
        if arm == "colour":
            kind = st.get("kind")
            if kind in ("white", "dark"):
                want = " ".join(gw.fmt(c) for c in sx.COLOUR_RGB[kind]) + " 1"
                check(rec["diffuse"] == want, f"{name}: {kind} koli rengi {rec['diffuse']}")
                continue
            check(f"print_{stem}.png" in tex_on, f"{name}: baskılı kolinin dokusu yok")
        mesh = tex_on.get(f"mesh:carton_{stem}.obj")
        check(mesh is not None, f"{name}: {arm} kolisinin modeli yok")
        if mesh is None:
            continue
        v, vt, faces = read_obj(mesh)
        # 14 - centre is the box centre in mesh coordinates
        bad = 0
        for a, b, c in faces:
            n = np.cross(v[b] - v[a], v[c] - v[a])
            if np.dot(n, (v[a] + v[b] + v[c]) / 3) <= 0:
                bad += 1
        check(bad == 0, f"{name}: {bad} üçgen içe bakıyor")
        # 15 - the front face is the first grid carton_obj writes; picked by
        # position instead it would also take the side faces' front corners.
        nu, nv = sx.FRONT_GRID
        n_front = (nu + 1) * (nv + 1)
        front = v[:n_front]
        check(np.all(facing * front[:, 1] > dy / 2 - 0.05), f"{name}: ön yüz ızgarası ön yüzde değil")
        d = facing * front[:, 1] - dy / 2
        edge = (np.abs(np.abs(front[:, 0]) - dx / 2) < 1e-6) | (np.abs(np.abs(front[:, 2]) - dz / 2) < 1e-6)
        check(np.abs(d[edge]).max() < 1e-6, f"{name}: ön yüzün kenarı yerinden oynamış")
        if arm == "bulge":
            check(abs(d.max() - st["value"]) < 1e-4 and d.min() > -1e-9,
                  f"{name}: şişme {d.max():.4f}, istenen {st['value']}")
            prot = d.max() - front_gap
            check(standoff[rid] - half - prot >= need - TOL, f"{name}: şişme aracın payını yiyor")
        elif arm == "dent":
            check(-st["value"] - 1e-6 <= d.min() <= -0.8 * st["value"] and d.max() < 1e-9,
                  f"{name}: çukur {d.min():.4f}, istenen -{st['value']}")
        elif arm == "colour":
            check(np.allclose(d, 0, atol=1e-9), f"{name}: baskılı kolinin yüzü düz değil")
            uv_front = vt[:n_front]
            check(uv_front[:, 0].max() <= sx.PRINT_U + 1e-6, f"{name}: baskı dokusu taşıyor")
        # 16 - curved labels lie LABEL_STANDOFF in front of the surface
        if arm in ("bulge", "dent") and rec["label"] is not None:
            grid = front[np.lexsort((front[:, 0], front[:, 2]))]
            check(len(grid) == n_front, f"{name}: ön yüz ızgarası {len(grid)} köşe")
            if len(grid) == n_front:
                xs = np.linspace(-dx / 2, dx / 2, nu + 1)
                zs = np.linspace(-dz / 2, dz / 2, nv + 1)
                H = (facing * grid[:, 1] - dy / 2).reshape(nv + 1, nu + 1)

                def surf(x, z):
                    i = np.clip(np.searchsorted(xs, x) - 1, 0, nu - 1)
                    j = np.clip(np.searchsorted(zs, z) - 1, 0, nv - 1)
                    tx, tz = (x - xs[i]) / (xs[i + 1] - xs[i]), (z - zs[j]) / (zs[j + 1] - zs[j])
                    return ((1 - tx) * (1 - tz) * H[j, i] + tx * (1 - tz) * H[j, i + 1]
                            + (1 - tx) * tz * H[j + 1, i] + tx * tz * H[j + 1, i + 1])
                c = np.array(rec["body"][:3])
                for key in ("label", "placard"):
                    lm = tex_on.get(f"mesh:lab_{stem}_{key}.obj")
                    check(lm is not None, f"{name}/{key}: kıvrık etiket modeli yok")
                    if lm is None:
                        continue
                    lv, _, _ = read_obj(lm)
                    pose = rec["visuals"][key]
                    world = np.array(pose[:3]) + lv @ rpy_mat(*pose[3:]).T - c
                    gapm = facing * world[:, 1] - dy / 2 - np.array([surf(x, z) for x, z in world[:, [0, 2]]])
                    check(np.abs(gapm - gw.LABEL_STANDOFF).max() < 1.5e-3,
                          f"{name}/{key}: etiket yüzeyden {gapm.min()*1000:.1f}..{gapm.max()*1000:.1f} mm")
    return seen


def main() -> int:
    base_cfg = gl._load_cfg(CONFIG)
    off_cfg = copy.deepcopy(base_cfg)
    off_cfg.setdefault("stress", {})["enabled"] = False
    on_cfg = copy.deepcopy(base_cfg)
    on_cfg["stress"]["enabled"] = True
    on_cfg["stress"]["use"] = list(sx.GEOMETRY_ARMS)
    lab_cfg = copy.deepcopy(base_cfg)
    lab_cfg["stress"]["enabled"] = True
    lab_cfg["stress"]["use"] = list(sx.LABEL_ARMS)
    shp_cfg = copy.deepcopy(base_cfg)
    shp_cfg["stress"]["enabled"] = True
    shp_cfg["stress"]["use"] = list(sx.SHAPE_ARMS)

    import contextlib, io
    with contextlib.redirect_stdout(io.StringIO()):
        sdf_off, man_off, tex_off = gw.build(off_cfg)
        sdf_on, man_on, _ = gw.build(on_cfg)
        sdf_lab, man_lab, tex_lab = gw.build(lab_cfg)
        sdf_shp, man_shp, tex_shp = gw.build(shp_cfg)

    # 1
    check(not any("stress" in c for c in man_off), "kapalı dünyada stress alanı var")
    seen_lab = check_labels(base_cfg, sdf_off, tex_off, sdf_lab, man_lab, tex_lab)
    seen_shp = check_shapes(shp_cfg, sdf_off, sdf_shp, man_shp, tex_shp)

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
    print("etiket kusurları: " + ", ".join(f"{a} {n}" for a, n in sorted(seen_lab.items())))
    print("biçim ve renk: " + ", ".join(f"{a} {n}" for a, n in sorted(seen_shp.items())))
    if failures:
        for f in failures[:40]:
            print("  HATA:", f)
        print(f"{len(failures)} kontrol başarısız")
        return 1
    print("bütün kontroller geçti")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
