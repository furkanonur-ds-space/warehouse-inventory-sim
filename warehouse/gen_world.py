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
import stress as sx  # noqa: E402
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
# küçük kutuda (XS, dz=0.28) bile taşmasın diye: 150 + 10 + 90 = 250 mm.
# MODÜL DÜZEYİNDE: scan_boxes barkodu QR'a göre konumlandırırken bu değere
# ihtiyaç duyuyor; iki yerde ayrı tutmak sessiz bir ROI kayması üretirdi.
LABEL_GAP = 0.002
# QR sembolünün kutu merkezinin kaç metre üstünde durduğu. Etiket ölçüleri
# değişirken bu değişmemeli: scanner/layout.json'daki code_z ve uçuş
# irtifaları buradan türetiliyor.
QR_SYMBOL_RISE_M = 0.0610


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


def mesh_visual(name: str, mesh: str, xyz, rgb, texture: str | None = None,
                ind="        ") -> str:
    """Kendi modeliyle çizilen koli: eğri ön yüz ya da baskılı doku."""
    r, g, b = (1.0, 1.0, 1.0) if texture else rgb
    albedo = (f"\n{ind}      <albedo_map>model://{ASSET_MODEL}/materials/textures/"
              f"{texture}</albedo_map>") if texture else ""
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><mesh><uri>model://{ASSET_MODEL}/meshes/{mesh}</uri></mesh></geometry>
{ind}  <material>
{ind}    <ambient>{fmt(r*0.5)} {fmt(g*0.5)} {fmt(b*0.5)} 1</ambient>
{ind}    <diffuse>{fmt(r)} {fmt(g)} {fmt(b)} 1</diffuse>
{ind}    <specular>0.05 0.05 0.05 1</specular>
{ind}    <pbr><metal>{albedo}
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


def label_visual(name: str, texture: str, size_wh, xyz_rpy, ind="        ",
                 mesh: str | None = None) -> str:
    """Etiket quad'ı. Metalness 0 / roughness 1: kod üzerinde parlama olursa
    okunmaz, o yüzden tamamen mat.

    `mesh` verilirse etiket o modelle çizilir, gerçek boyunda (ölçek 1): eğri
    bir yüze yapışmış, onunla kıvrılmış etiket (stress.label_obj)."""
    w, h = size_wh
    uri = mesh or "label_quad.obj"
    scale = "1 1 1" if mesh else f"{fmt(w)} {fmt(h)} 1"
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz_rpy)}</pose>
{ind}  <geometry>
{ind}    <mesh>
{ind}      <uri>model://{ASSET_MODEL}/meshes/{uri}</uri>
{ind}      <scale>{scale}</scale>
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


def check_aisles(cfg) -> None:
    """`aisles` ile `rows` birbirini tutuyor mu?

    Koridorlar artık eşit genişlikte değil (2.40'tan 0.50'ye daralıyor), yani
    koridor merkezi ve genişliği raf sırası kotlarından ARTIK türetilemiyor
    gibi görünse de tam tersi: ikisi de aynı geometriyi iki ayrı yerde
    yazıyor. Elle senkron tutulan iki kaynak sessizce ayrışır -- markörler ve
    lambalar bir yere, raflar başka yere gider. Onun yerine burada kontrol
    ediliyor ve tutmazsa üretim duruyor.

    Bir koridorun net açıklığı, ona bakan iki raf YÜZÜ arasındaki mesafedir
    (dış sıralarda tek yüz olsaydı bu tanım çökerdi -- bu yerleşimde her
    koridorun iki yüzü var: rows[].aisle her id'yi tam iki kez veriyor).
    """
    rk = cfg["racking"]
    depth = rk["depth"]
    faces: dict[int, list[float]] = {}
    for row in rk["rows"]:
        # sıranın koridora bakan yüzü
        y_face = (row["y0"] + depth) if row["facing"] > 0 else row["y0"]
        faces.setdefault(row["aisle"], []).append(y_face)

    for aisle in rk["aisles"]:
        aid = aisle["id"]
        ys = sorted(faces.get(aid, []))
        if len(ys) != 2:
            raise SystemExit(f"koridor {aid}: {len(ys)} raf yüzü var, 2 olmalı "
                             f"(rows[].aisle alanlarına bak)")
        width, centre = ys[1] - ys[0], (ys[0] + ys[1]) / 2
        if abs(width - aisle["width"]) > 1e-6:
            raise SystemExit(f"koridor {aid}: rows {width:.3f} m diyor, "
                             f"aisles[].width {aisle['width']:.3f} m diyor")
        if abs(centre - aisle["y_center"]) > 1e-6:
            raise SystemExit(f"koridor {aid}: rows merkezi {centre:.3f}, "
                             f"aisles[].y_center {aisle['y_center']:.3f}")


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


def parse_unlabelled(spec) -> set:
    """
    "G/01/2/0" gibi seçicileri (yüz, göz, seviye, kutu) kümeye çevirir.

    Deney için: kodu olmayan bir kutuyu kutu dedektörü buluyor mu? Kutunun
    kendisi yerinde durur, yalnız QR ve barkod görselleri basılmaz.

    Seçici yanlışsa SESSİZ GEÇİLMEZ. Yazım hatası olan bir seçici hiçbir
    kutuyla eşleşmez, deney de "YOLO hepsini buldu" diye biter - ki o sonuç
    deneyin kurulmadığının işaretidir, başarısının değil.
    """
    out = set()
    for item in spec or []:
        parts = str(item).split("/")
        if len(parts) != 4:
            raise SystemExit(
                f"unlabelled_boxes: '{item}' dört parça olmalı: "
                "yüz/göz/seviye/kutu, örnek G/01/2/0")
        row, bay, level, idx = parts
        try:
            out.add((row.strip().upper(), int(bay), int(level), int(idx)))
        except ValueError:
            raise SystemExit(
                f"unlabelled_boxes: '{item}' içinde sayı olmayan alan var")
    return out


def skewed_block(cx, qr_z, pc_z, lw, lh, pw, ph, angle):
    """QR ve barkod etiketinin köşeleri, blok ortası etrafında `angle` dönmüş."""
    bz = (qr_z + lh / 2 + pc_z - ph / 2) / 2
    return [sx.rect(cx, qr_z, lw, lh, (cx, bz), angle),
            sx.rect(cx, pc_z, pw, ph, (cx, bz), angle)]


def decoy_layout(level: int, cx, pc_z, lw, lh, pw, ph) -> list[dict]:
    """Eski etiketlerin yeri: asıl barkodun altında, yukarıdan aşağı.

    1 yalnız eski barkod, 2 yalnız eski QR, 3 ikisi birden. Altta, çünkü
    üstte kolinin kapağı var ve yanlarda en dar kolide (XS) 40 mm kalıyor.
    """
    top = pc_z - ph / 2 - sx.DECOY_GAP_M
    kinds = {1: ["placard"], 2: ["qr"], 3: ["placard", "qr"]}[level]
    out = []
    for kind in kinds:
        w, h = (pw, ph) if kind == "placard" else (lw, lh)
        out.append({"kind": kind, "x": cx, "z": top - h / 2, "w": w, "h": h})
        top -= h + LABEL_GAP
    return out


def inventory(cfg, rng, textures, manifest) -> str:
    """Raflardaki kutular, üzerlerindeki QR etiketleri ve konum barkodları."""
    rk, bx, codes = cfg["racking"], cfg["boxes"], cfg["codes"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    levels, x0 = rk["level_heights"], rk["x_origin"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    spec = codes["box_label"]
    # Kodu basılmayacak kutular. warehouse.yaml'da
    #   codes.unlabelled_boxes: ["G/01/2/0", "G/03/1/1", ...]
    # biçiminde, yüz/göz/seviye/kutu. Boşsa hiçbir şey değişmez ve dünya
    # eskisiyle bit bit aynı kalır.
    unlabelled = parse_unlabelled(codes.get("unlabelled_boxes"))
    # Hepsi birden: deponun HİÇBİR kutusunda kod yok. Liste yerinde kalır ki
    # anahtar kapanınca önceki deney aynen geri gelsin.
    every = bool(codes.get("unlabelled_all"))
    n_bare = 0
    lw, lh = spec["label"]
    # QR'ın kendi etiketinin merkezine göre yüksekliği; caption şeridi
    # kalkınca sıfır olur, ama hesap etiketten okunur, varsayılmaz.
    _, qr_rise = gl.box_label_geometry(spec, ppm, maxpx)
    pc_spec = codes["box_placard"]
    pw, ph = pc_spec["label"]
    label_gap = LABEL_GAP

    # Koridor genişliği -> o koridordan geçirilebilecek en büyük kutu. Bir
    # kutu rafa konmadan önce koridordan geçmek zorunda, ve en dar koridor
    # 0.50 m: sınır konmazsa dünya oraya 0.80 m'lik kutular dizer ve kimse
    # onların nasıl geldiğini soramaz.
    aisle_width = {a["id"]: a["width"] for a in rk["aisles"]}
    clearance = bx.get("aisle_clearance", 0.0)

    # ZORLU SENARYO (warehouse/stress.py). Kapalıyken hiçbir şey çekilmez ve
    # aşağıdaki her satır eskisiyle aynı metni üretir.
    st = cfg.get("stress") or {}
    assigner = sx.Assigner(cfg) if st.get("enabled") else None
    caps = (sx.face_caps(cfg, PROJECT_ROOT / st.get("layout", "scanner/layout.json"))
            if assigner else {})
    # Etiket kusurlarının dokudaki ölçeği: modülün piksel boyu.
    qr_scale = gl._canvas(spec["label"], ppm, maxpx)[1]
    pc_scale = gl._canvas(pc_spec["label"], ppm, maxpx)[1]

    out = ['  <model name="inventory">\n    <static>true</static>\n'
           + model_pose_tag(world_yaw_rad(cfg))]
    n_box = 0
    used_sizes: dict[str, set] = {}

    for row in rk["rows"]:
        rid, y0, facing = row["id"], row["y0"], row["facing"]
        # bu sıraya hizmet eden koridorun geçirebildiği kutular
        limit = aisle_width[row["aisle"]] - clearance
        allowed = [s for s in bx["sizes"]
                   if max(s["dims"][0], s["dims"][1]) <= limit]
        if not allowed:
            raise SystemExit(
                f"sıra {rid}: koridor {row['aisle']} "
                f"({aisle_width[row['aisle']]:.2f} m) hiçbir kutu boyutunu "
                f"geçirmiyor (sınır {limit:.2f} m). boxes.sizes'a daha küçük "
                f"bir kutu ekle ya da koridoru genişlet.")
        used_sizes[rid] = {s["name"] for s in allowed}
        # ürün yüzü: koridora bakan kenar
        y_face = (y0 + depth) if facing > 0 else y0

        for bi in range(nb):
            for li, z in enumerate(levels):
                if rng.random() > bx["fill_probability"]:
                    continue
                count = rng.randint(*bx["per_slot"])
                sizes = [rng.choice(allowed) for _ in range(count)]
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

                    # Bu koliye düşen bozulma. Komşuya ve dikmeye en fazla
                    # aradaki boşluğun yarısı kadar yaklaşabilir; komşu da
                    # öbür yarıyı kullanabilsin diye.
                    cell = sx.CONTROL
                    if assigner:
                        room = gap / 2 - sx.MARGIN_M
                        slot = sx.Slot(
                            x_lo=cx - dx / 2 - room, x_hi=cx + dx / 2 + room,
                            y_back=y0 if facing > 0 else y0 + depth,
                            y_face=y_face, facing=facing,
                            max_protrusion=caps[rid]["max_protrusion"])
                        listed = (rid, bi + 1, li + 1, si) in unlabelled
                        # Etiketlerin, kolinin ön yüzündeki yeri (aşağıda
                        # bir daha, aynı formülle hesaplanıyor).
                        qr_z0 = cz + QR_SYMBOL_RISE_M - qr_rise
                        pc_z0 = qr_z0 - lh / 2 - label_gap - ph / 2
                        face_box = (cx - dx / 2 + sx.MARGIN_M, cx + dx / 2 - sx.MARGIN_M,
                                    z + sx.MARGIN_M, z + dz - sx.MARGIN_M)

                        def ok(c, _d=(dx, dy, dz), _s=slot, _listed=listed,
                               _c=(cx, cy, cz), _z=z, _q=qr_z0, _p=pc_z0, _f=face_box):
                            if c.arm == "empty":
                                # Listedeki kodsuz koli deneyin parçası; onu
                                # rafdan almak o deneyi bozar.
                                return not _listed
                            if c.arm in sx.LABEL_ARMS:
                                # Etiketi olmayan koliye etiket kusuru verilmez.
                                if every or _listed:
                                    return False
                                if c.arm == "skew":
                                    return sx.rects_inside(
                                        skewed_block(_c[0], _q, _p, lw, lh, pw, ph,
                                                     math.radians(c.signed)), *_f)
                                if c.arm == "decoy":
                                    return sx.rects_inside(
                                        [sx.rect(d["x"], d["z"], d["w"], d["h"])
                                         for d in decoy_layout(int(c.value), _c[0], _p,
                                                               lw, lh, pw, ph)], *_f)
                                return True
                            if c.arm == "bulge":
                                # Şişen yüz koridora çıkar: aracın payını yer.
                                return c.value - bx["front_gap"] <= _s.max_protrusion
                            if c.arm in sx.SHAPE_ARMS:
                                return True
                            return sx.fits(sx.place(c, _c, _d, facing, _z), _d, _s)

                        cell = assigner.pick(rid, ok)

                    sku = f"SKU{rng.randint(10000, 99999)}"
                    payload = gl.box_payload(sku, rid, bi + 1, li + 1)
                    tex = f"box_{rid}{bi+1:02d}{li+1}{si}.png"
                    # Kutunun sıra numarası. Barkodun taşıdığı şey bu:
                    # QR'ın alanlarından hiçbirini tekrar etmiyor ve 4 haneye
                    # sığdığı için çubuklar en dar koridorun kadrajına giren
                    # bir ene inebiliyor.
                    box_index = n_box + 1
                    pc_payload = gl.placard_payload(box_index)
                    pc_caption = gl.placard_caption(box_index)
                    pc_tex = f"placard_{rid}{bi+1:02d}{li+1}{si}.png"

                    # ETİKETSİZ KUTU. Deney için: kod hiç basılmamış bir
                    # kutuyu YOLO buluyor mu? Kutu her şeyiyle aynı yerde
                    # duruyor, yalnız QR ve barkod görselleri basılmıyor.
                    #
                    # rng ÇEKİLİŞLERİ YUKARIDA, BU KONTROLÜN ÖNÜNDE KALIYOR.
                    # sku ve shade zaten çekildi; burada atlanan tek şey
                    # görsel üretimi. Çekilişi atlasaydık ondan sonraki her
                    # kutunun SKU'su ve rengi kayardı, dünya baştan aşağı
                    # değişirdi ve iki koşu karşılaştırılamazdı.
                    bare = every or (rid, bi + 1, li + 1, si) in unlabelled
                    gone = cell.arm == "empty"
                    img, module_m = gl.make_box_label(payload, sku, spec, ppm, maxpx)
                    pc_img, pc_module_m = gl.make_bay_placard(pc_payload, pc_caption,
                                                              pc_spec, ppm, maxpx)
                    if not bare and not gone:
                        textures[tex] = img
                        textures[pc_tex] = pc_img

                    link = f"box_{rid}_{bi+1:02d}_{li+1}_{si}"
                    shade = rng.uniform(0.88, 1.06)
                    cardboard = tuple(min(1.0, c * shade) for c in (0.68, 0.52, 0.34))
                    tag = {"stress": cell.record()} if assigner else {}
                    if cell.arm in sx.TEXTURE_ARMS and not bare:
                        # Aynı baskı: QR etiketi de barkod etiketi de aynı
                        # kusuru taşır, ikisi ayrı ayrı puanlanır.
                        textures[tex] = sx.damage(img, cell, assigner.tex_rng,
                                                  module_m * qr_scale, cardboard)
                        textures[pc_tex] = sx.damage(pc_img, cell, assigner.tex_rng,
                                                     pc_module_m * pc_scale, cardboard)

                    if gone:
                        # BOŞ GÖZ. Koli rafta yok; bütün çekilişler yukarıda
                        # yapıldı, sıra numarası da harcanıyor ki sonraki
                        # kolilerin barkodları temiz dünyadakiyle aynı kalsın.
                        # Yer gerçeğine yine yazılır: "burada bir şey gördüm"
                        # diyen bir rapor bu kayda bakıp hayalet koli
                        # olduğunu anlayabilsin.
                        rpy = facing_rpy(facing)
                        manifest.append({
                            "type": "box_absent",
                            "symbology": None,
                            "payload": f"ABSENT|{rid}|{bi+1:02d}|{li+1}|{si}",
                            "caption": sku,
                            "entity": f"inventory::{link}",
                            "row": rid, "bay": bi + 1, "level": li + 1,
                            "label_pose_xyzrpy": [round(cx, 4), round(cy + facing * dy / 2, 4),
                                                  round(cz, 4), *[round(v, 6) for v in rpy]],
                            "label_size_m": [dx, dz],
                            "normal": [0.0, float(facing), 0.0],
                            **tag,
                        })
                        n_box += 1
                        continue

                    if cell.arm not in sx.GEOMETRY_ARMS:
                        placed = None
                        body_pose = (cx, cy, cz)
                    else:
                        placed = sx.place(cell, (cx, cy, cz), (dx, dy, dz), facing, z)
                        # +0.0: -0.0 SDF'e "-0" diye yazılıyordu
                        body_pose = tuple(float(v) + 0.0 for v in
                                          (*placed.centre, *sx.mat_to_rpy(placed.rot)))

                    # BİÇİM VE RENK. Şişme ve ezik kolinin ön yüzünü eğer, koli
                    # kendi modelini alır; baskılı koli dokulu bir model alır;
                    # beyaz ve koyu yalnız rengini değiştirir. Çarpışma her
                    # durumda düz kutu: şişme payı fits()'te araca karşı
                    # sınandı.
                    stem = f"{rid}{bi+1:02d}{li+1}{si}"
                    shape_f = None
                    body_xml = None
                    if cell.arm in ("bulge", "dent"):
                        shape_f, extra = sx.surface(cell, (dx, dy, dz), assigner.tex_rng)
                        tag["stress"].update(extra)
                        mesh = f"carton_{stem}.obj"
                        textures[f"mesh:{mesh}"] = sx.carton_obj((dx, dy, dz), facing, shape_f)
                        body_xml = mesh_visual("body", mesh, body_pose, cardboard, ind="      ")
                    elif cell.arm == "colour":
                        kind = sx.COLOURS[int(cell.value)]
                        tag["stress"]["kind"] = kind
                        if kind == "printed":
                            pimg, extra = sx.print_texture(assigner.tex_rng, (dx, dy, dz))
                            tag["stress"].update(extra)
                            mesh = f"carton_{stem}.obj"
                            ptex = f"print_{stem}.png"
                            textures[ptex] = pimg
                            textures[f"mesh:{mesh}"] = sx.carton_obj(
                                (dx, dy, dz), facing, sx.surface(sx.CONTROL, (dx, dy, dz), None)[0],
                                printed=True)
                            body_xml = mesh_visual("body", mesh, body_pose, cardboard,
                                                   texture=ptex, ind="      ")
                        else:
                            cardboard = sx.COLOUR_RGB[kind]

                    out.append(f'    <link name="{link}">\n')
                    out.append(body_xml or box_visual("body", (dx, dy, dz), body_pose,
                                                      cardboard, "      "))
                    out.append(box_collision("body_c", (dx, dy, dz), body_pose, "      "))

                    # QR SEMBOLÜ kutu merkezinin sabit bir yüksekliğinde
                    # durur, barkod da onun altına asılır.
                    #
                    # Önceden iki etiket tek blok olarak kutu ön yüzüne
                    # ortalanıyordu, ki blok yüksekliği değiştiğinde QR da
                    # yer değiştiriyordu. Artık değişmemesi gerekiyor:
                    # scanner/layout.json'daki code_z ve uçuş irtifaları QR
                    # yüksekliklerinden türetiliyor, ve bu düzenlemenin amacı
                    # barkodu yukarı almak, QR'ı oynatmak değil. Sabit,
                    # önceki yerleşimin verdiği değerdir -- 432 kutunun
                    # hepsinde ölçüldü, hiçbirinde değişmiyordu.
                    stack_h = lh + label_gap + ph
                    qr_z = cz + QR_SYMBOL_RISE_M - qr_rise
                    pc_z = qr_z - lh / 2 - label_gap - ph / 2
                    if qr_z + lh / 2 > cz + dz / 2 or pc_z - ph / 2 < cz - dz / 2:
                        # Etiketler kutunun ön yüzüne sığmıyor. Sessizce
                        # taşırmak yerine söylenir: config'teki kutu boyları
                        # ile etiket ölçüleri arasında tutarsızlık var.
                        raise SystemExit(
                            f"etiket bloğu {stack_h*1000:.0f} mm, {rid} sırasında "
                            f"{dz*1000:.0f} mm yüksek kutuya sığmıyor")

                    off = LABEL_STANDOFF * (1 if facing > 0 else -1)
                    rpy = facing_rpy(facing)
                    qr_xyz = (cx, y_label + off, qr_z)
                    pc_xyz = (cx, y_label + off, pc_z)
                    normal = (0.0, float(facing), 0.0)
                    if placed is not None:
                        # Etiketler kolinin yüzüne yapışık: kolinin merkezine
                        # göre yerleri ve yönelimleri koliyle birlikte döner.
                        qr_xyz = tuple(placed.point(np.subtract(qr_xyz, (cx, cy, cz))))
                        pc_xyz = tuple(placed.point(np.subtract(pc_xyz, (cx, cy, cz))))
                        rpy = tuple(v + 0.0 for v in
                                    sx.mat_to_rpy(placed.rot @ sx.rpy_to_mat(*rpy)))
                        normal = tuple(float(v) for v in placed.rot @ np.array(normal))
                    decoys = []
                    if cell.arm == "skew":
                        # EĞRİ YAPIŞTIRILMIŞ: iki etiket tek blok olarak,
                        # bloğun ortası etrafında, yüzün kendi düzleminde
                        # döner. Ayrı ayrı döndürülseydi barkod QR'ın
                        # içine girerdi.
                        a = math.radians(cell.signed)
                        bz = (qr_z + lh / 2 + pc_z - ph / 2) / 2
                        turn = sx.rot_y(-a)
                        centre = np.array((cx, y_label + off, bz))
                        qr_xyz = tuple(centre + turn @ (np.array(qr_xyz) - centre))
                        pc_xyz = tuple(centre + turn @ (np.array(pc_xyz) - centre))
                        rpy = tuple(v + 0.0 for v in
                                    sx.mat_to_rpy(turn @ sx.rpy_to_mat(*rpy)))
                    elif cell.arm == "decoy":
                        # ESKİ ETİKET: kolinin önceki sevkiyatından kalmış,
                        # okunabilir ama YANLIŞ şey söyleyen bir QR ve/veya
                        # barkod, asıl etiketlerin altında. QR'ı başka bir
                        # adresi, barkodu depoda olmayan bir numarayı
                        # taşır; okunan her biri envantere yanlış bir kayıt
                        # demektir.
                        tr = assigner.tex_rng
                        for d in decoy_layout(int(cell.value), cx, pc_z, lw, lh, pw, ph):
                            if d["kind"] == "qr":
                                other = rk["rows"][int(tr.integers(len(rk["rows"])))]["id"]
                                dp = gl.box_payload(f"SKU{int(tr.integers(10000, 100000))}",
                                                    other, int(tr.integers(nb)) + 1,
                                                    int(tr.integers(len(levels))) + 1)
                                dimg, _ = gl.make_box_label(dp, "", spec, ppm, maxpx)
                                dtex = f"decoy_{rid}{bi+1:02d}{li+1}{si}_qr.png"
                                sym = "QR"
                            else:
                                dp = gl.placard_payload(int(tr.integers(5000, 10000)))
                                dimg, _ = gl.make_bay_placard(dp, dp, pc_spec, ppm, maxpx)
                                dtex = f"decoy_{rid}{bi+1:02d}{li+1}{si}_pc.png"
                                sym = "CODE128"
                            textures[dtex] = dimg
                            decoys.append({**d, "payload": dp, "tex": dtex, "symbology": sym,
                                           "xyz": (d["x"], y_label + off, d["z"])})
                    lab_mesh = {"label": None, "placard": None}
                    if shape_f is not None:
                        # Etiketler eğri yüze yapışık: merkezleri yüzeyin o
                        # noktadaki yerine çıkar ya da iner, kendileri de
                        # yüzeyle kıvrılır. Etiketin u ekseni, koridora +y'de
                        # bakan yüzde -x'e döşeli (FACE_POS_Y).
                        moved = []
                        for key, xyz, (w_, h_) in (("label", qr_xyz, (lw, lh)),
                                                   ("placard", pc_xyz, (pw, ph))):
                            zr = xyz[2] - cz
                            at = float(shape_f(0.0, zr))
                            mesh = f"lab_{stem}_{key}.obj"
                            textures[f"mesh:{mesh}"] = sx.label_obj(
                                w_, h_, lambda u, v, _z=zr, _a=at:
                                float(shape_f(-facing * u, _z + v)) - _a)
                            lab_mesh[key] = mesh
                            moved.append((xyz[0], xyz[1] + facing * at, xyz[2]))
                        qr_xyz, pc_xyz = moved
                    if not bare:
                        out.append(label_visual("label", tex, (lw, lh),
                                                (*qr_xyz, *rpy), "      ", mesh=lab_mesh["label"]))
                        out.append(label_visual("placard", pc_tex, (pw, ph),
                                                (*pc_xyz, *rpy), "      ", mesh=lab_mesh["placard"]))
                        for d in decoys:
                            out.append(label_visual(f"decoy_{d['kind']}", d["tex"],
                                                    (d["w"], d["h"]), (*d["xyz"], *rpy), "      "))
                    out.append("    </link>\n")

                    if bare:
                        # Kutu yerinde duruyor ama okunacak hiçbir şeyi yok.
                        # Yer gerçeğine yine de yazılır, ÇÜNKÜ VAR: envanter
                        # raporları kod tipine göre süzdüğü için bu kayıt
                        # onların sayısına girmez, ama kutuyu sayan rapor onu
                        # buradan bilir. Yazmasak "hiç olmayan kutu" ile
                        # "kodu olmayan kutu" ayırt edilemezdi.
                        manifest.append({
                            "type": "box_unlabelled",
                            "symbology": None,
                            # Okunacak bir kodu yok ama ADI olmak zorunda:
                            # raporlar ve 3B görüntüleyici kayıtları payload
                            # ile eşliyor. None bırakılınca deneyin kolileri
                            # çizilemiyordu. Kod gibi görünmemesi için
                            # bilerek QR yükünden farklı bir biçim.
                            #
                            # Sondaki kutu sırası şart: bir gözün bir
                            # seviyesinde üç kutu var. Onsuz 432 kodsuz kutu
                            # 144 ada düşüyordu ve raporlar üçte birini
                            # sayıyordu; 16'lık deneyde her seçici ayrı bir
                            # gözdeydi, o yüzden görünmemişti.
                            "payload": f"UNLABELLED|{rid}|{bi+1:02d}|{li+1}|{si}",
                            "caption": sku,
                            "entity": f"inventory::{link}",
                            "row": rid, "bay": bi + 1, "level": li + 1,
                            "label_pose_xyzrpy": [*[round(float(v), 4) for v in qr_xyz],
                                                  *[round(v, 6) for v in rpy]],
                            "label_size_m": [lw, lh],
                            "normal": [round(v, 6) for v in normal],
                            **tag,
                        })
                        n_box += 1
                        n_bare += 1
                        continue

                    manifest.append({
                        "type": "box_qr",
                        "symbology": "QR",
                        "payload": payload,
                        "caption": sku,
                        "entity": f"inventory::{link}",
                        "row": rid, "bay": bi + 1, "level": li + 1,
                        "label_pose_xyzrpy": [*[round(float(v), 4) for v in qr_xyz],
                                              *[round(v, 6) for v in rpy]],
                        "label_size_m": [lw, lh],
                        "module_size_m": round(module_m, 6),
                        "normal": [round(v, 6) for v in normal],
                        **tag,
                    })
                    manifest.append({
                        "type": "box_placard",
                        "symbology": "CODE128",
                        "payload": pc_payload,
                        "caption": pc_caption,
                        "entity": f"inventory::{link}",
                        "row": rid, "bay": bi + 1, "level": li + 1,
                        "label_pose_xyzrpy": [*[round(float(v), 4) for v in pc_xyz],
                                              *[round(v, 6) for v in rpy]],
                        "label_size_m": [pw, ph],
                        "module_size_m": round(pc_module_m, 6),
                        "normal": [round(v, 6) for v in normal],
                        **tag,
                    })
                    for d in decoys:
                        manifest.append({
                            "type": "box_decoy",
                            "symbology": d["symbology"],
                            "payload": d["payload"],
                            "caption": "",
                            "entity": f"inventory::{link}",
                            "row": rid, "bay": bi + 1, "level": li + 1,
                            "label_pose_xyzrpy": [*[round(float(v), 4) for v in d["xyz"]],
                                                  *[round(v, 6) for v in rpy]],
                            "label_size_m": [d["w"], d["h"]],
                            "normal": [round(v, 6) for v in normal],
                            **tag,
                        })
                    n_box += 1

    out.append("  </model>\n")
    print(f"  kutu           : {n_box}")
    if assigner:
        print(f"  ZORLU SENARYO  : tohum {st['seed']}, araç payı en az "
              f"{st.get('min_side_clearance', 0.05):.3f} m")
        for f, c in caps.items():
            print(f"    {f}: koridora en fazla {c['max_protrusion']*1000:+.0f} mm taşabilir "
                  f"(standoff {c['standoff']:.3f}, yarı açıklık {c['half_span']:.3f})")
        print("\n".join(assigner.summary()))
    if n_bare:
        print(f"  ETİKETSİZ      : {n_bare} kutu kodsuz basıldı (deney)")
        if not every and n_bare != len(unlabelled):
            raise SystemExit(
                f"unlabelled_boxes {len(unlabelled)} kutu istedi, {n_bare} "
                "tanesi eşleşti - seçicilerden biri hiçbir kutuya denk "
                "gelmiyor, deney kurulmadan uçulurdu")
    for aisle in rk["aisles"]:
        rows_here = [r["id"] for r in rk["rows"] if r["aisle"] == aisle["id"]]
        names = sorted(set().union(*(used_sizes[r] for r in rows_here)),
                       key=lambda n: [s["name"] for s in bx["sizes"]].index(n))
        print(f"  koridor {aisle['id']}      : {aisle['width']:.2f} m açıklık, "
              f"yüzler {'+'.join(rows_here)}, kutular {'/'.join(names)}")
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


def lighting(cfg, plan=None) -> str:
    lt = cfg["lighting"]["ceiling_lights"]
    # Işık bozulması (stress.light_plan): lambalar kısılmış, bazıları sönük.
    # Kapalıyken plan temiz dünyanınkidir ve buradan çıkan metin değişmez.
    plan = plan or sx.light_plan(cfg)
    dr, dg, db, da = plan["diffuse"]
    sr, sg, sb, sa = lt["specular"]
    yaw = world_yaw_rad(cfg)
    out = []
    i = 0
    for xc in lt["x_positions"]:
        for yc in lt["y_positions"]:
            # Işıklar <model> içinde değil, world düzeyinde <light> -- yani
            # model pozuyla dönmezler. Konumları burada elle çevriliyor.
            x, y = rotate_xy(xc, yc, yaw)
            if i in plan["dead"]:
                # SÖNÜK LAMBA. Sahneye hiç konmaz; sıra numarası yine harcanır
                # ki öbür lambaların adı temiz dünyadakiyle aynı kalsın.
                i += 1
                continue
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
    print(f"  tavan lambası  : {i - len(plan['dead'])}"
          + (f"  ({len(plan['dead'])} sönük: {sorted(plan['dead'])})" if plan["dead"] else ""))
    return "".join(out)


# --------------------------------------------------------------------------
# birleştirme
# --------------------------------------------------------------------------

def tag_light(cfg, plan, manifest) -> None:
    """Her etiketin aldığı ışık, yer gerçeğine: bu dünyada ve temiz dünyada.

    Config çerçevesinde, rotate_manifest'ten ÖNCE: lambaların konumu da o
    çerçevede yazılı. Raporlar bunu okur, hesaplamaz - ışığın nasıl kurulduğu
    üreticinin kararı ve iki yerde hesaplanırsa ayrışır.
    """
    clean = sx.light_plan(dict(cfg, lights_stress=None))
    n = 0
    for rec in manifest:
        if rec["type"] not in ("box_qr", "box_placard", "box_unlabelled", "box_decoy"):
            continue
        p, nrm = rec["label_pose_xyzrpy"][:3], rec["normal"]
        e = sx.illuminance(p, nrm, plan, cfg)
        e0 = sx.illuminance(p, nrm, clean, cfg)
        rec["light"] = {"E": round(e, 4), "E_clean": round(e0, 4),
                        "rel": round(e / e0, 4) if e0 > 0 else None}
        n += 1
    es = sorted(r["light"]["E"] for r in manifest if r.get("type") == "box_qr" and "light" in r)
    if es:
        print(f"  IŞIK           : {n} etiket; QR ışığı en az {es[0]:.2f}, ortanca "
              f"{es[len(es)//2]:.2f}, en çok {es[-1]:.2f} (model birimi)")


def build(cfg) -> tuple[str, list]:
    rng = random.Random(cfg["seed"])
    textures: dict = {}
    manifest: list = []
    lg = cfg["lighting"]
    plan = sx.light_plan(cfg)
    ar, ag, ab, aa = plan["ambient"]
    br, bg, bb, ba = lg["background"]

    yaw = world_yaw_rad(cfg)
    check_aisles(cfg)

    # SIRA ÖNEMLİ. inventory() manifest'i CONFIG çerçevesinde doldurur;
    # rotate_manifest onu dünyaya çevirir. aisle_markers() ise zaten dünya
    # koordinatı yazdığı için dönüşten SONRA çağrılmalı -- önce çağrılsaydı
    # ArUco konumları bir kez fazladan dönerdi.
    body = [
        building(cfg, textures),
        racking(cfg),
        inventory(cfg, rng, textures, manifest),
    ]
    if plan["on"]:
        tag_light(cfg, plan, manifest)
    rotate_manifest(manifest, yaw)
    body.append(aisle_markers(cfg, textures, manifest))
    body.append(lighting(cfg, plan))
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
    # kolilere özel modeller de (şişmiş/ezik/baskılı koli, kıvrık etiket)
    for old in list(mesh_dir.glob("carton_*.obj")) + list(mesh_dir.glob("lab_*.obj")):
        old.unlink()

    (assets / "model.config").write_text(MODEL_CONFIG)
    (assets / "model.sdf").write_text(ASSET_STUB_SDF)
    (mesh_dir / "label_quad.obj").write_text(LABEL_QUAD_OBJ)
    (mesh_dir / "floor_tile.obj").write_text(floor_mesh_obj(
        cfg["building"]["length"], cfg["building"]["width"], FLOOR_TILE_M))
    for name, img in textures.items():
        if name.startswith("mesh:"):
            (mesh_dir / name[5:]).write_text(img)
        else:
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
    truth = {"world": "warehouse", "seed": cfg["seed"], "codes": manifest}
    plan = sx.light_plan(cfg)
    if plan["on"]:
        # Kapalıyken yazılmaz: temiz dünyanın yer gerçeği bit bit aynı kalsın.
        truth["lighting"] = {"ambient": plan["ambient"], "diffuse": plan["diffuse"],
                             "dead_lamps": sorted(plan["dead"])}
    gt_path.write_text(json.dumps(truth, indent=2, ensure_ascii=False))

    images = [n for n in textures if not n.startswith("mesh:")]
    tex_bytes = sum((tex_dir / n).stat().st_size for n in images)
    print(f"  toplam kod     : {len(manifest)}")
    print(f"  doku           : {len(images)} dosya, {tex_bytes/1e6:.1f} MB")
    if len(images) < len(textures):
        print(f"  koli modeli    : {len(textures) - len(images)} dosya")
    def shown(path: Path) -> Path:
        # --out depo dışında ya da göreli verilince relative_to patlıyordu.
        path = path.resolve()
        return path.relative_to(PROJECT_ROOT) if path.is_relative_to(PROJECT_ROOT) else path
    print(f"\n  {shown(world_path)}")
    print(f"  {shown(gt_path)}")
    print(f"  {shown(mm_path)}  (Furkan'ın biçiminde)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
