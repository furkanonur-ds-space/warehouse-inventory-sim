#!/usr/bin/env python3
"""config/warehouse.yaml'dan Gazebo depo world'ünü ve etiket dokularını üretir.

World elle yazılmak yerine üretiliyor çünkü:
  * her kod benzersiz, onlarca doku ve yerleşim elle sürdürülemez;
  * sentetik veri aşamasında her kodun dünya koordinatındaki pozunu bilmek
    gerekiyor -- üretici bunu ground truth olarak yazıyor;
  * ileride domain randomization için tohumu değiştirip yeniden üretmek yeter.

    .venv/bin/python tools/gen_world.py

Çıktılar:
    gz/models/warehouse_assets/    dokular + etiket quad mesh'i
    gz/worlds/warehouse.sdf        world
    out/ground_truth.json          her kodun yükü, pozu ve boyutu
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import gen_labels as gl  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

PROJECT_ROOT = Path(__file__).resolve().parent.parent
ASSET_MODEL = "warehouse_assets"

# Zemin dokusu karo boyu (m). Optik akış sensörü zemindeki GÖRSEL ÖZELLİKLERİ
# izler; düz renk zeminde iz yok -> "2/20 eşleşme" -> L1'de bozuk hız -> runaway
# (2026-08-13 kök neden). Beton benekli doku tekrarlı UV ile döşenir; karo ~0.5 m
# olunca L1'de (0.4 m irtifa, ~0.7 m alt-kamera FOV) kadrajda bol özellik olur.
FLOOR_TILE_M = 0.5


def floor_texture(px: int = 256, seed: int = 7) -> "Image.Image":
    """Beton benekli, tekrarlanabilir zemin dokusu. Optik akış için asıl olan
    YÜKSEK FREKANSLI kontrast (özellik köşeleri); ince benek + orta ölçek leke."""
    rng = np.random.default_rng(seed)
    fine = rng.normal(0.0, 0.10, (px, px))                    # ince benek
    cs = rng.normal(0.0, 1.0, (px // 16, px // 16))           # orta ölçek
    cs = (cs - cs.min()) / (np.ptp(cs) + 1e-9)
    coarse = np.asarray(Image.fromarray((cs * 255).astype(np.uint8))
                        .resize((px, px), Image.BILINEAR), dtype=np.float64) / 255.0 - 0.5
    g = np.clip(0.5 + fine + 0.15 * coarse, 0.18, 0.82)
    a = (g * 255).astype(np.uint8)
    rgb = np.stack([a, a, np.clip(a.astype(int) + 4, 0, 255).astype(np.uint8)], -1)
    return Image.fromarray(rgb, "RGB")


def floor_mesh_obj(L: float, W: float, tile_m: float) -> str:
    """Zemin quad'ı; UV 0..(L/tile) x 0..(W/tile) -> doku tekrarlı döşenir
    (gz albedo_map varsayılan sarma REPEAT). Gerçek boyutta, SDF scale 1."""
    ru, rv = L / tile_m, W / tile_m
    return f"""# Zemin quad -- gen_world.py üretti; UV {ru:.1f}x{rv:.1f} tekrar (~{tile_m} m/karo)
v {-L/2:.4f} {-W/2:.4f} 0.0
v {L/2:.4f} {-W/2:.4f} 0.0
v {L/2:.4f} {W/2:.4f} 0.0
v {-L/2:.4f} {W/2:.4f} 0.0
vt 0 0
vt {ru:.4f} 0
vt {ru:.4f} {rv:.4f}
vt 0 {rv:.4f}
vn 0 0 1
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""

# Etiket quad'ı: XY düzleminde 1x1 m, normali +Z, UV [0,1].
# Kendi mesh'imizi üretiyoruz çünkü SDF <box>/<plane> primitiflerinde UV'nin
# hangi yüze nasıl oturduğu garanti değil; aynalanmış bir doku QR'ı ve barkodu
# okunamaz yapar. 4 köşeli bir OBJ'de belirsizlik kalmıyor.
LABEL_QUAD_OBJ = """# Etiket quad'ı -- gen_world.py tarafından üretildi
# XY düzlemi, normal +Z, u -> +X, v -> +Y (v=1 dokunun üst satırı)
v -0.5 -0.5 0.0
v  0.5 -0.5 0.0
v  0.5  0.5 0.0
v -0.5  0.5 0.0
vt 0.0 0.0
vt 1.0 0.0
vt 1.0 1.0
vt 0.0 1.0
vn 0.0 0.0 1.0
# Dört köşe de aynı normali paylaşır -- yüzlerde normal indeksi hep 1.
# (2/3 yazmak "vertex normal indices out of bounds" verip mesh'i düşürüyor.)
f 1/1/1 2/2/1 3/3/1
f 1/1/1 3/3/1 4/4/1
"""

MODEL_CONFIG = """<?xml version="1.0"?>
<model>
  <name>warehouse_assets</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>
    Depo world'ünün paylaşılan varlıkları: barkod/QR etiket dokuları ve
    etiket quad mesh'i. Bu model doğrudan spawn edilmez; world SDF'i
    içindeki dosyalara model:// ile başvurur.
  </description>
</model>
"""

# Bu model spawn edilmek için değil, sadece kaynak yolu çözümlemesi için var.
ASSET_STUB_SDF = """<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="warehouse_assets">
    <static>true</static>
    <link name="link"/>
  </model>
</sdf>
"""

# Etiketin gömüldüğü yüzeyden ne kadar önde durduğu. Sıfır olursa z-fighting
# olur, çok büyük olursa etiket havada durur.
LABEL_STANDOFF = 0.004

# QR ile barkod arasındaki dikey boşluk. İkisi tek dikey blok olarak kutunun
# ön yüzü içine ortalanıyor (bkz. inventory() içindeki `margin` hesabı) -- en
# küçük kutuda (S, dz=0.30) bile taşmasın diye.
# MODÜL DÜZEYİNDE: scan_boxes barkodu QR'a göre konumlandırırken bu değere
# ihtiyaç duyuyor; iki yerde ayrı tutmak sessiz bir ROI kayması üretirdi.
LABEL_GAP = 0.010


# --------------------------------------------------------------------------
# SDF yardımcıları
# --------------------------------------------------------------------------

def fmt(v: float) -> str:
    return f"{v:.6g}"


def pose(x, y, z, roll=0.0, pitch=0.0, yaw=0.0) -> str:
    return " ".join(fmt(v) for v in (x, y, z, roll, pitch, yaw))


def box_visual(name: str, size, xyz, rgb, ind="        ") -> str:
    r, g, b = rgb
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>
{ind}  <material>
{ind}    <ambient>{fmt(r*0.5)} {fmt(g*0.5)} {fmt(b*0.5)} 1</ambient>
{ind}    <diffuse>{fmt(r)} {fmt(g)} {fmt(b)} 1</diffuse>
{ind}    <specular>0.05 0.05 0.05 1</specular>
{ind}    <pbr><metal>
{ind}      <metalness>0.0</metalness>
{ind}      <roughness>0.9</roughness>
{ind}    </metal></pbr>
{ind}  </material>
{ind}</visual>
"""


def box_collision(name: str, size, xyz, ind="        ") -> str:
    return f"""{ind}<collision name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><box><size>{fmt(size[0])} {fmt(size[1])} {fmt(size[2])}</size></box></geometry>
{ind}</collision>
"""


def label_visual(name: str, texture: str, size_wh, xyz_rpy, ind="        ") -> str:
    """Etiket quad'ı. Metalness 0 / roughness 1: kod üzerinde parlama olursa
    okunmaz, o yüzden tamamen mat."""
    w, h = size_wh
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz_rpy)}</pose>
{ind}  <geometry>
{ind}    <mesh>
{ind}      <uri>model://{ASSET_MODEL}/meshes/label_quad.obj</uri>
{ind}      <scale>{fmt(w)} {fmt(h)} 1</scale>
{ind}    </mesh>
{ind}  </geometry>
{ind}  <material>
{ind}    <ambient>1 1 1 1</ambient>
{ind}    <diffuse>1 1 1 1</diffuse>
{ind}    <specular>0 0 0 1</specular>
{ind}    <pbr><metal>
{ind}      <albedo_map>model://{ASSET_MODEL}/materials/textures/{texture}</albedo_map>
{ind}      <metalness>0.0</metalness>
{ind}      <roughness>1.0</roughness>
{ind}    </metal></pbr>
{ind}  </material>
{ind}</visual>
"""


#: Etiketin dünya yönelimi. Quad'ın normali yerelde +Z; roll=+90 onu -Y'ye
#: çevirir ve dokunun üst kenarı +Z'ye bakar. Yaw=180 ile +Y'ye döner.
FACE_NEG_Y = (math.pi / 2, 0.0, 0.0)
FACE_POS_Y = (math.pi / 2, 0.0, math.pi)


def facing_rpy(facing: int):
    return FACE_POS_Y if facing > 0 else FACE_NEG_Y


# --------------------------------------------------------------------------
# config çerçevesi -> dünya çerçevesi
# --------------------------------------------------------------------------
#
# Yerleşim mantığının tamamı (raflar X boyunca, koridorlar Y'de ayrık) config
# çerçevesinde yazılı ve ÖYLE KALIYOR. Furkan'ın çerçevesine geçiş tek bir Z
# dönüşüyle yapılır: `world_yaw`.
#
# İki yerde uygulanır, başka hiçbir yerde:
#
#   1. SDF tarafı -- her üst düzey <model>'e <pose>0 0 0 0 0 yaw</pose>.
#      Model pozu, içindeki bütün visual/collision'ları dünya orijini
#      etrafında döndürür. Yüzlerce pozu tek tek çevirmeye gerek yok;
#      dönüşüm Gazebo'nun kendi kinematik zincirinde olur, yani yerleşim
#      koduna hiç dokunulmaz.
#
#   2. manifest (ground_truth.json) tarafı -- rotate_manifest().
#      Burası Python'da hesaplanıp dünya koordinatı olarak yazıldığı için
#      dönüşümü elle uygulamak şart.
#
# DÖNÜŞÜMDEN GEÇMEYENLER: koridor ArUco markörleri ve spawn pozu. İkisi de
# config'e zaten DÜNYA koordinatında yazıldı (Furkan'ın marker_map.json'ı ve
# warehouse_scanner.py'siyle birebir karşılaştırılabilsin diye). Bu yüzden
# aisle_markers() kendi modelini dönüşsüz emit eder.


def world_yaw_rad(cfg) -> float:
    return math.radians(cfg.get("world_yaw", 0.0) or 0.0)


def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return x * c - y * s, x * s + y * c


def model_pose_tag(yaw: float, ind: str = "    ") -> str:
    """Üst düzey modelin dünya pozu. yaw=0 ise hiç yazma -- üretilen SDF
    gereksiz satırla şişmesin."""
    return "" if abs(yaw) < 1e-12 else f"{ind}<pose>0 0 0 0 0 {fmt(yaw)}</pose>\n"


def rotate_manifest(manifest: list, yaw: float) -> None:
    """ground_truth kayıtlarını yerinde config çerçevesinden dünyaya çevirir.

    Konum: XY düzleminde döndürülür, Z değişmez.
    Yönelim: SDF'in rpy sırası R = Rz(yaw)Ry(pitch)Rx(roll). Sola bir Rz(t)
    çarpmak Rz(t)Rz(yaw)Ry(pitch)Rx(roll) = Rz(t+yaw)Ry(pitch)Rx(roll) verir,
    yani SADECE yaw'a eklenir -- roll/pitch aynen kalır. Etiketlerimizin
    roll'u +90 olduğu için bu tam olarak istenen sonuç.
    Normal: konumla aynı XY dönüşü.
    """
    if abs(yaw) < 1e-12:
        return
    for rec in manifest:
        x, y, z, r, pth, yw = rec["label_pose_xyzrpy"]
        rx, ry = rotate_xy(x, y, yaw)
        rec["label_pose_xyzrpy"] = [round(rx, 4), round(ry, 4), z,
                                    r, pth, round(yw + yaw, 6)]
        nx, ny, nz = rec["normal"]
        rnx, rny = rotate_xy(nx, ny, yaw)
        rec["normal"] = [round(rnx, 6), round(rny, 6), nz]


# --------------------------------------------------------------------------
# world parçaları
# --------------------------------------------------------------------------

def building(cfg, textures) -> str:
    b = cfg["building"]
    L, W, H, t = b["length"], b["width"], b["height"], b["wall_thickness"]
    yaw = world_yaw_rad(cfg)
    parts = ['  <model name="warehouse_building">\n    <static>true</static>\n'
             + model_pose_tag(yaw) + '    <link name="structure">\n']

    # zemin -- BETON DOKULU (optik akış için; düz renk zemin akışı öldürüyordu,
    # bkz. FLOOR_TILE_M). Görsel = tekrarlı-UV dokulu quad (z=0 yüzeyi, hafif
    # yukarıda ki z-fighting olmasın); collision hâlâ kutu.
    textures["floor.png"] = floor_texture()
    parts.append(f"""      <visual name="floor_v">
        <pose>0 0 0.002 0 0 0</pose>
        <geometry><mesh><uri>model://{ASSET_MODEL}/meshes/floor_tile.obj</uri></mesh></geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.04 0.04 0.04 1</specular>
          <pbr><metal>
            <albedo_map>model://{ASSET_MODEL}/materials/textures/floor.png</albedo_map>
            <metalness>0.0</metalness>
            <roughness>0.95</roughness>
          </metal></pbr>
        </material>
      </visual>
""")
    parts.append(box_collision("floor_c", (L, W, t), (0, 0, -t / 2), "      "))
    # tavan
    parts.append(box_visual("ceiling_v", (L, W, t), (0, 0, H + t / 2), (0.80, 0.80, 0.82), "      "))
    parts.append(box_collision("ceiling_c", (L, W, t), (0, 0, H + t / 2), "      "))

    walls = [
        ("wall_xp", (t, W + 2 * t, H), (L / 2 + t / 2, 0, H / 2)),
        ("wall_xn", (t, W + 2 * t, H), (-L / 2 - t / 2, 0, H / 2)),
        ("wall_yp", (L + 2 * t, t, H), (0, W / 2 + t / 2, H / 2)),
        ("wall_yn", (L + 2 * t, t, H), (0, -W / 2 - t / 2, H / 2)),
    ]
    for name, size, xyz in walls:
        parts.append(box_visual(f"{name}_v", size, xyz, (0.78, 0.78, 0.75), "      "))
        parts.append(box_collision(f"{name}_c", size, xyz, "      "))

    parts.append("    </link>\n  </model>\n")
    return "".join(parts)


def racking(cfg) -> str:
    rk = cfg["racking"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    ft, uh = rk["frame_thickness"], rk["upright_height"]
    levels, x0 = rk["level_heights"], rk["x_origin"]
    yaw = world_yaw_rad(cfg)

    upright_rgb = (0.72, 0.30, 0.08)   # turuncu dikme
    beam_rgb = (0.10, 0.26, 0.55)      # mavi kiriş
    deck_rgb = (0.55, 0.56, 0.58)      # galvaniz raf tablası

    out = []
    for row in rk["rows"]:
        rid, y0 = row["id"], row["y0"]
        out.append(f'  <model name="rack_{rid}">\n    <static>true</static>\n'
                   + model_pose_tag(yaw) + '    <link name="frame">\n')
        yc = y0 + depth / 2

        # dikmeler: her göz sınırında, derinliğin ön ve arkasında
        for i in range(nb + 1):
            x = x0 + i * bw
            for tag, y in (("f", y0 + ft / 2), ("b", y0 + depth - ft / 2)):
                out.append(box_visual(f"up_{i}_{tag}_v", (ft, ft, uh), (x, y, uh / 2),
                                      upright_rgb, "      "))
                out.append(box_collision(f"up_{i}_{tag}_c", (ft, ft, uh), (x, y, uh / 2), "      "))

        for li, z in enumerate(levels):
            for bi in range(nb):
                xc = x0 + (bi + 0.5) * bw
                # ön ve arka kirişler
                for tag, y in (("f", y0 + ft / 2), ("b", y0 + depth - ft / 2)):
                    out.append(box_visual(f"beam_{li}_{bi}_{tag}_v", (bw - ft, ft, ft),
                                          (xc, y, z - ft / 2), beam_rgb, "      "))
                    out.append(box_collision(f"beam_{li}_{bi}_{tag}_c", (bw - ft, ft, ft),
                                             (xc, y, z - ft / 2), "      "))
                # raf tablası: kutuların üstünde durduğu yüzey
                out.append(box_visual(f"deck_{li}_{bi}_v", (bw - ft, depth - 2 * ft, 0.02),
                                      (xc, yc, z - 0.01), deck_rgb, "      "))
                out.append(box_collision(f"deck_{li}_{bi}_c", (bw - ft, depth - 2 * ft, 0.02),
                                         (xc, yc, z - 0.01), "      "))
        out.append("    </link>\n  </model>\n")
    return "".join(out)


def inventory(cfg, rng, textures, manifest) -> str:
    """Raflardaki kutular, üzerlerindeki QR etiketleri ve konum barkodları."""
    rk, bx, codes = cfg["racking"], cfg["boxes"], cfg["codes"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    levels, x0 = rk["level_heights"], rk["x_origin"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    spec = codes["box_label"]
    lw, lh = spec["label"]
    pc_spec = codes["box_placard"]
    pw, ph = pc_spec["label"]
    label_gap = LABEL_GAP

    out = ['  <model name="inventory">\n    <static>true</static>\n'
           + model_pose_tag(world_yaw_rad(cfg))]
    n_box = 0

    for row in rk["rows"]:
        rid, y0, facing = row["id"], row["y0"], row["facing"]
        # ürün yüzü: koridora bakan kenar
        y_face = (y0 + depth) if facing > 0 else y0

        for bi in range(nb):
            for li, z in enumerate(levels):
                if rng.random() > bx["fill_probability"]:
                    continue
                count = rng.randint(*bx["per_slot"])
                sizes = [rng.choice(bx["sizes"]) for _ in range(count)]
                total_w = sum(s["dims"][0] for s in sizes)
                if total_w > bw - rk["frame_thickness"] - 0.1:
                    sizes = sizes[:1]
                    total_w = sizes[0]["dims"][0]

                # gözün içinde yatayda ortala, aralarına eşit boşluk koy
                gap = (bw - rk["frame_thickness"] - total_w) / (len(sizes) + 1)
                cursor = x0 + bi * bw + rk["frame_thickness"] / 2 + gap

                for si, size in enumerate(sizes):
                    dx, dy, dz = size["dims"]
                    cx = cursor + dx / 2
                    cursor += dx + gap
                    # kutunun ön yüzü raf ön kenarından `front_gap` içeride
                    if facing > 0:
                        cy = y_face - bx["front_gap"] - dy / 2
                        y_label = cy + dy / 2
                    else:
                        cy = y_face + bx["front_gap"] + dy / 2
                        y_label = cy - dy / 2
                    cz = z + dz / 2

                    sku = f"SKU{rng.randint(10000, 99999)}"
                    payload = gl.box_payload(sku, rid, bi + 1, li + 1)
                    tex = f"box_{rid}{bi+1:02d}{li+1}{si}.png"
                    pc_payload = gl.placard_payload(rid, bi + 1, li + 1)
                    pc_caption = gl.placard_caption(rid, bi + 1, li + 1)
                    pc_tex = f"placard_{rid}{bi+1:02d}{li+1}{si}.png"

                    img, module_m = gl.make_box_label(payload, sku, spec, ppm, maxpx)
                    textures[tex] = img
                    pc_img, pc_module_m = gl.make_bay_placard(pc_payload, pc_caption,
                                                              pc_spec, ppm, maxpx)
                    textures[pc_tex] = pc_img

                    link = f"box_{rid}_{bi+1:02d}_{li+1}_{si}"
                    shade = rng.uniform(0.88, 1.06)
                    cardboard = tuple(min(1.0, c * shade) for c in (0.68, 0.52, 0.34))

                    out.append(f'    <link name="{link}">\n')
                    out.append(box_visual("body", (dx, dy, dz), (cx, cy, cz), cardboard, "      "))
                    out.append(box_collision("body_c", (dx, dy, dz), (cx, cy, cz), "      "))

                    # QR + barkod tek dikey blok olarak kutunun ön yüzüne
                    # ortalanır. dz - stack_h negatif çıkarsa (kutu bu iki
                    # etiketi sığdıramayacak kadar alçaksa) margin sıfıra
                    # kenetlenir ve etiketler kutu sınırının biraz dışına taşar
                    # -- config'teki box boyutlarıyla box_placard/box_label
                    # ölçüleri arasında bir tutarsızlık olduğunun işareti.
                    stack_h = lh + label_gap + ph
                    margin = max(0.0, (dz - stack_h) / 2)
                    qr_z = cz + dz / 2 - margin - lh / 2
                    pc_z = qr_z - lh / 2 - label_gap - ph / 2

                    off = LABEL_STANDOFF * (1 if facing > 0 else -1)
                    rpy = facing_rpy(facing)
                    out.append(label_visual("label", tex, (lw, lh),
                                            (cx, y_label + off, qr_z, *rpy), "      "))
                    out.append(label_visual("placard", pc_tex, (pw, ph),
                                            (cx, y_label + off, pc_z, *rpy), "      "))
                    out.append("    </link>\n")

                    manifest.append({
                        "type": "box_qr",
                        "symbology": "QR",
                        "payload": payload,
                        "caption": sku,
                        "entity": f"inventory::{link}",
                        "row": rid, "bay": bi + 1, "level": li + 1,
                        "label_pose_xyzrpy": [round(cx, 4), round(y_label + off, 4),
                                              round(qr_z, 4), *[round(v, 6) for v in rpy]],
                        "label_size_m": [lw, lh],
                        "module_size_m": round(module_m, 6),
                        "normal": [0.0, float(facing), 0.0],
                    })
                    manifest.append({
                        "type": "box_placard",
                        "symbology": "CODE128",
                        "payload": pc_payload,
                        "caption": pc_caption,
                        "entity": f"inventory::{link}",
                        "row": rid, "bay": bi + 1, "level": li + 1,
                        "label_pose_xyzrpy": [round(cx, 4), round(y_label + off, 4),
                                              round(pc_z, 4), *[round(v, 6) for v in rpy]],
                        "label_size_m": [pw, ph],
                        "module_size_m": round(pc_module_m, 6),
                        "normal": [0.0, float(facing), 0.0],
                    })
                    n_box += 1

    out.append("  </model>\n")
    print(f"  kutu           : {n_box}")
    return "".join(out)


def aisle_markers(cfg, textures, manifest) -> str:
    """Koridor başlarındaki ArUco markörleri.

    Eski projede burada iki ayrı AprilTag hattı vardı: koridor zeminine 4 m
    arayla dizilmiş `floor_marker`'lar (alt kamera için) ve raf dikmelerine
    çakılmış `rack_marker`'lar (ön kamera için). İkisi de bizim AprilTag
    lokalizasyonumuza aitti; bu projede lokalizasyon bizde olmadığı için
    ikisi de kaldırıldı. Yerlerine Furkan'ın hattının beklediği tek şey
    kaldı: her koridorun iki ucunda birer ArUco, id 1..8.

    Konumlar config'te DÜNYA koordinatındadır ve `world_yaw` dönüşümünden
    GEÇMEZ -- bu yüzden model pozu verilmiyor. Böylece üretilen dünya,
    ref/furkan/marker_map.json ile satır satır karşılaştırılabilir kalıyor.

    UYARI: x500_scanner'da aşağı bakan kamera yok. Bu markörler şu hâliyle
    DEKORATİF -- hiçbir kamera onları göremiyor. Sapma düzeltmesi
    uygulanacaksa önce alt kamera eklenmeli. (Ölçülen sapma zaten hedefin
    çok altında olduğu için bu gerekmeyebilir; bkz. docs/00-durum.md.)
    """
    codes = cfg["codes"]
    spec = codes["aisle_marker"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    lw, lh = spec["label"]
    dict_name = spec["dictionary"]

    out = ['  <model name="aisle_markers">\n    <static>true</static>\n']
    n = 0
    for marker_id, x, y in spec["positions"]:
        caption = ""                      # markörün kendisi tam kareyi doldursun
        link = f"aruco_{marker_id}"
        tex = f"{link}.png"
        img, module_m = gl.make_aisle_marker(marker_id, caption, spec, ppm, maxpx)
        textures[tex] = img

        out.append(f'    <link name="{link}">\n')
        # zemine yatık: quad normali zaten +Z
        out.append(label_visual("label", tex, (lw, lh),
                                (x, y, LABEL_STANDOFF, 0, 0, 0), "      "))
        out.append("    </link>\n")

        manifest.append({
            "type": "aisle_marker",
            "symbology": "ARUCO",
            "dictionary": dict_name,
            "marker_id": marker_id,
            "payload": gl.aruco_payload(marker_id, dict_name),
            "caption": caption,
            "entity": f"aisle_markers::{link}",
            "world_frame": True,          # world_yaw uygulanmadı
            "label_pose_xyzrpy": [round(x, 4), round(y, 4), LABEL_STANDOFF, 0.0, 0.0, 0.0],
            "label_size_m": [lw, lh],
            "module_size_m": round(module_m, 6),
            "normal": [0.0, 0.0, 1.0],
        })
        n += 1

    out.append("  </model>\n")
    print(f"  koridor ArUco  : {n}  ({dict_name}, "
          f"{gl.aruco_modules(dict_name)}x{gl.aruco_modules(dict_name)} modül)")
    return "".join(out)


def lighting(cfg) -> str:
    lt = cfg["lighting"]["ceiling_lights"]
    dr, dg, db, da = lt["diffuse"]
    sr, sg, sb, sa = lt["specular"]
    yaw = world_yaw_rad(cfg)
    out = []
    i = 0
    for xc in lt["x_positions"]:
        for yc in lt["y_positions"]:
            # Işıklar <model> içinde değil, world düzeyinde <light> -- yani
            # model pozuyla dönmezler. Konumları burada elle çevriliyor.
            x, y = rotate_xy(xc, yc, yaw)
            # cast_shadows kapalı: nokta ışık gölgeleri OGRE2'de pahalı ve
            # iGPU'da kare hızını yarıya düşürüyor. Kodların okunması için
            # gölge değil, düzgün ve parlamasız aydınlatma gerekiyor.
            out.append(f"""  <light type="point" name="ceiling_{i}">
    <pose>{pose(x, y, lt['height'])}</pose>
    <cast_shadows>false</cast_shadows>
    <diffuse>{fmt(dr)} {fmt(dg)} {fmt(db)} {fmt(da)}</diffuse>
    <specular>{fmt(sr)} {fmt(sg)} {fmt(sb)} {fmt(sa)}</specular>
    <attenuation>
      <range>{fmt(lt['attenuation_range'])}</range>
      <constant>0.3</constant>
      <linear>0.05</linear>
      <quadratic>0.005</quadratic>
    </attenuation>
  </light>
""")
            i += 1
    print(f"  tavan lambası  : {i}")
    return "".join(out)


# --------------------------------------------------------------------------
# birleştirme
# --------------------------------------------------------------------------

def build(cfg) -> tuple[str, list]:
    rng = random.Random(cfg["seed"])
    textures: dict = {}
    manifest: list = []
    lg = cfg["lighting"]
    ar, ag, ab, aa = lg["ambient"]
    br, bg, bb, ba = lg["background"]

    yaw = world_yaw_rad(cfg)

    # SIRA ÖNEMLİ. inventory() manifest'i CONFIG çerçevesinde doldurur;
    # rotate_manifest onu dünyaya çevirir. aisle_markers() ise zaten dünya
    # koordinatı yazdığı için dönüşten SONRA çağrılmalı -- önce çağrılsaydı
    # ArUco konumları bir kez fazladan dönerdi.
    body = [
        building(cfg, textures),
        racking(cfg),
        inventory(cfg, rng, textures, manifest),
    ]
    rotate_manifest(manifest, yaw)
    body.append(aisle_markers(cfg, textures, manifest))
    body.append(lighting(cfg))
    if yaw:
        print(f"  dünya dönüşü   : {math.degrees(yaw):+.0f} deg "
              f"(config çerçevesi -> Furkan'ın çerçevesi)")

    sdf = f"""<?xml version="1.0" encoding="UTF-8"?>
<!-- tools/gen_world.py tarafından üretildi -- elle düzenlemeyin.
     Değişiklik için config/warehouse.yaml'ı düzenleyip yeniden çalıştırın. -->
<sdf version="1.9">
  <world name="warehouse">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>

    <!-- Sistem eklentileri world'de tanımlanmaz; PX4 bunları
         src/modules/simulation/gz_bridge/server.config ile yükler
         (GZ_SIM_SERVER_CONFIG_PATH). PX4'ün kendi world'leri de aynı
         şekilde çalışıyor. -->

    <scene>
      <grid>false</grid>
      <ambient>{fmt(ar)} {fmt(ag)} {fmt(ab)} {fmt(aa)}</ambient>
      <background>{fmt(br)} {fmt(bg)} {fmt(bb)} {fmt(ba)}</background>
      <shadows>true</shadows>
    </scene>

    <!-- Kapalı ortam: yönlü güneş ışığı yok, aydınlatma tavan lambalarından.
         Yine de PX4'ün NavSat'ı için bir referans konum gerekiyor. -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>41.015137</latitude_deg>
      <longitude_deg>28.979530</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

{''.join(body)}  </world>
</sdf>
"""
    return sdf, manifest, textures


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--out", type=Path, default=PROJECT_ROOT)
    ap.add_argument("--seed", type=int, help="config'deki tohumu geçersiz kıl")
    args = ap.parse_args()

    cfg = gl._load_cfg(args.config)
    if args.seed is not None:
        cfg["seed"] = args.seed

    print(f"world üretiliyor (tohum {cfg['seed']}):")
    sdf, manifest, textures = build(cfg)

    assets = args.out / "gz" / "models" / ASSET_MODEL
    tex_dir = assets / "materials" / "textures"
    mesh_dir = assets / "meshes"
    for d in (tex_dir, mesh_dir, args.out / "gz" / "worlds", args.out / "out"):
        d.mkdir(parents=True, exist_ok=True)

    # eski dokuları temizle: tohum veya yerleşim değişince artık dosyalar kalmasın
    for old in tex_dir.glob("*.png"):
        old.unlink()

    (assets / "model.config").write_text(MODEL_CONFIG)
    (assets / "model.sdf").write_text(ASSET_STUB_SDF)
    (mesh_dir / "label_quad.obj").write_text(LABEL_QUAD_OBJ)
    (mesh_dir / "floor_tile.obj").write_text(floor_mesh_obj(
        cfg["building"]["length"], cfg["building"]["width"], FLOOR_TILE_M))
    for name, img in textures.items():
        img.save(tex_dir / name, optimize=True)

    world_path = args.out / "gz" / "worlds" / "warehouse.sdf"
    world_path.write_text(sdf)

    # Furkan'ın hattı markör haritasını KENDİ biçiminde okuyor
    # ({"1": {"x":..., "y":...}}). Konumlar burada değiştiği için haritayı
    # da burada üretiyoruz -- elle senkronlanan iki dosya olsaydı sapma
    # düzeltmesi sessizce yanlış konuma çeker.
    mm = {str(c["marker_id"]): {"x": c["label_pose_xyzrpy"][0],
                                "y": c["label_pose_xyzrpy"][1]}
          for c in manifest if c["type"] == "aisle_marker"}
    mm_path = args.out / "out" / "marker_map.json"
    mm_path.write_text(json.dumps(mm, indent=2))

    gt_path = args.out / "out" / "ground_truth.json"
    gt_path.write_text(json.dumps({
        "world": "warehouse",
        "seed": cfg["seed"],
        "codes": manifest,
    }, indent=2, ensure_ascii=False))

    tex_bytes = sum((tex_dir / n).stat().st_size for n in textures)
    print(f"  toplam kod     : {len(manifest)}")
    print(f"  doku           : {len(textures)} dosya, {tex_bytes/1e6:.1f} MB")
    print(f"\n  {world_path.relative_to(PROJECT_ROOT)}")
    print(f"  {gt_path.relative_to(PROJECT_ROOT)}")
    print(f"  {mm_path.relative_to(PROJECT_ROOT)}  (Furkan'ın biçiminde)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
