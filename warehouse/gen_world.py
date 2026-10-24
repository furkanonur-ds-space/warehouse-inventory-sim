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
import zlib
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


def box_visual(name: str, size, xyz, rgb, ind="        ", rpy=(0.0, 0.0, 0.0)) -> str:
    r, g, b = rgb
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz, *rpy)}</pose>
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


# --------------------------------------------------------------------------
# gerçekçilik (warehouse.yaml -> realism:)
# --------------------------------------------------------------------------
#
# Gerçek depo fotoğraflarına (mavi delikli dikme, turuncu kiriş, çapraz
# bağlantılı çerçeve, paletli yük) yaklaşmak için. Her anahtar kapalıyken
# dünya bit bit eskisi gibi üretilir; açıkken de KUTULARIN VE KODLARIN
# YERİ DEĞİŞMEZ -- ground_truth.json aynı kalır, scanner/layout.json'daki
# code_z ve uçuş irtifaları geçerliliğini korur.

# Euro palet (EN 13698-1): 1200 x 800 x 144 mm. 800 kenarı koridora bakar.
PALLET_H = 0.144
PALLET_W = 0.80
# Kutudan taşma payı: palet yükten biraz geniş durur, ama en fazla Euro eni.
PALLET_OVERHANG = 0.05
# Dikme delik dokusunun bir karosu (m). Gerçek dikmelerde delik adımı 50 mm;
# bir karoda iki delik var.
UPRIGHT_TILE_M = 0.10


def realism(cfg) -> dict:
    r = cfg.get("realism") or {}
    return {k: bool(r.get(k)) for k in ("rack", "pallets", "cartons", "film", "building")}


def mesh_visual(name: str, uri: str, xyz, rgb, ind="        ", texture=None,
                roughness=0.9, specular=0.05) -> str:
    """Kendi ürettiğimiz OBJ mesh'i. `texture` verilirse renk dokudan gelir,
    `rgb` de verildiyse onu tonlar."""
    r, g, b = rgb if rgb else (1.0, 1.0, 1.0)
    tex = (f"\n{ind}      <albedo_map>model://{ASSET_MODEL}/materials/textures/{texture}</albedo_map>"
           if texture else "")
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><mesh><uri>model://{ASSET_MODEL}/meshes/{uri}</uri></mesh></geometry>
{ind}  <material>
{ind}    <ambient>{fmt(r*0.5)} {fmt(g*0.5)} {fmt(b*0.5)} 1</ambient>
{ind}    <diffuse>{fmt(r)} {fmt(g)} {fmt(b)} 1</diffuse>
{ind}    <specular>{fmt(specular)} {fmt(specular)} {fmt(specular)} 1</specular>
{ind}    <pbr><metal>{tex}
{ind}      <metalness>0.0</metalness>
{ind}      <roughness>{fmt(roughness)}</roughness>
{ind}    </metal></pbr>
{ind}  </material>
{ind}</visual>
"""


def quads_obj(quads, header: str) -> str:
    """Dörtgenlerden OBJ. Her dörtgen (köşeler, normal, uv'ler); köşeler
    dışarıdan bakınca saat yönünün tersine: sol-alt, sağ-alt, sağ-üst, sol-üst."""
    vs, vts, vns, fs = [], [], [], []
    for corners, normal, uvs in quads:
        b = len(vs)
        vs += corners
        vts += uvs
        vns.append(normal)
        n = len(vns)
        a, c, d, e = (f"{b+k+1}/{b+k+1}/{n}" for k in range(4))
        fs += [f"f {a} {c} {d}", f"f {a} {d} {e}"]
    lines = [f"# {header} -- gen_world.py üretti"]
    lines += [f"v {x:.4f} {y:.4f} {z:.4f}" for x, y, z in vs]
    lines += [f"vt {u:.4f} {v:.4f}" for u, v in vts]
    lines += [f"vn {x:.0f} {y:.0f} {z:.0f}" for x, y, z in vns]
    return "\n".join(lines + fs) + "\n"


def box_quads(cx, cy, cz, sx, sy, sz, ru=1.0, rv=1.0, caps=True):
    """Kutunun yan yüzleri; her yüzde u yatay 0..ru, v dikey 0..rv.
    SDF <box>'ın UV'si yüze nasıl oturuyor garanti değil (etiket quad'ının
    neden kendi mesh'i olduğuna bak); deliklerin dikey durması için burada
    elle veriliyor."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    lo, hi = cz - hz, cz + hz
    side = [((0, 0), (ru, 0), (ru, rv), (0, rv))]
    q = []
    for (x0, y0), (x1, y1), n in (
            ((cx + hx, cy - hy), (cx + hx, cy + hy), (1, 0, 0)),
            ((cx - hx, cy + hy), (cx - hx, cy - hy), (-1, 0, 0)),
            ((cx + hx, cy + hy), (cx - hx, cy + hy), (0, 1, 0)),
            ((cx - hx, cy - hy), (cx + hx, cy - hy), (0, -1, 0))):
        q.append(([(x0, y0, lo), (x1, y1, lo), (x1, y1, hi), (x0, y0, hi)], n, side[0]))
    if caps:
        flat = ((0, 0), (0, 0), (0, 0), (0, 0))
        q.append(([(cx - hx, cy - hy, hi), (cx + hx, cy - hy, hi),
                   (cx + hx, cy + hy, hi), (cx - hx, cy + hy, hi)], (0, 0, 1), flat))
        q.append(([(cx - hx, cy + hy, lo), (cx + hx, cy + hy, lo),
                   (cx + hx, cy - hy, lo), (cx - hx, cy - hy, lo)], (0, 0, -1), flat))
    return q


def upright_texture(px: int = 64) -> "Image.Image":
    """Mavi boyalı dikme, ortada 50 mm adımlı delik sırası. Bir karo
    UPRIGHT_TILE_M x UPRIGHT_TILE_M; mesh UV'si onu dikmece tekrar ettirir."""
    from PIL import ImageDraw
    img = Image.new("RGB", (px, px), (22, 88, 168))
    d = ImageDraw.Draw(img)
    # profil kenarları: bükümler biraz koyu görünür
    d.rectangle([0, 0, 2, px - 1], fill=(14, 62, 124))
    d.rectangle([px - 3, 0, px - 1, px - 1], fill=(14, 62, 124))
    sw, sh = px * 12 // 100, px * 28 // 100          # 12 x 28 mm delik
    for k in range(2):
        yc = px // 4 + k * px // 2
        x0 = (px - sw) // 2
        d.rectangle([x0, yc - sh // 2, x0 + sw, yc + sh // 2], fill=(18, 22, 30))
        # damla biçimi: üst ucu biraz geniş
        d.rectangle([x0 - 2, yc - sh // 2, x0 + sw + 2, yc - sh // 2 + sh // 4],
                    fill=(18, 22, 30))
    return img


def world_uv(quads, tu: float, tv: float = None, along_long: bool = False):
    """Dörtgenlere gerçek boyuta göre UV verir: u yüzün eni / tu, v boyu / tv.
    Doku böylece her parçada aynı ölçekte tekrarlar. `along_long` ise u her
    yüzde uzun kenar boyunca gider -- tahtada lif, tahtanın boyunca akar."""
    tv = tv or tu
    out = []
    for c, n, _ in quads:
        e1 = [b - a for a, b in zip(c[0], c[1])]
        e2 = [b - a for a, b in zip(c[0], c[3])]
        w, h = math.hypot(*e1), math.hypot(*e2)
        ab = ((0, 0), (w, 0), (w, h), (0, h))
        if along_long and h > w:
            ab = tuple((b, a) for a, b in ab)
        out.append((c, n, tuple((a / tu, b / tv) for a, b in ab)))
    return out


def _tileable_field(px: int, rng, terms: int, kmax: int) -> np.ndarray:
    """Tam periyotlu sinüslerin toplamı: karo kenarında dikiş olmaz."""
    yy, xx = np.mgrid[0:px, 0:px] / px
    f = np.zeros((px, px))
    for _ in range(terms):
        k1, k2 = rng.integers(-kmax, kmax + 1, 2)
        f += rng.uniform(0.3, 1.0) * np.sin(2 * np.pi * (k1 * xx + k2 * yy) + rng.uniform(0, 6.3))
    return f / terms


WOOD_TILE_M = 0.30


def wood_texture(paint=None, px: int = 256, seed: int = 3) -> "Image.Image":
    """Palet tahtası: u boyunca akan lif, budak, ince benek. `paint` verilirse
    üstü o renge boyalı (mavi kiralık palet); lif boyanın altından seçilir,
    yer yer boya aşınmış ve ham tahta görünür."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:px, 0:px] / px
    warp = _tileable_field(px, rng, 6, 2)
    lines = np.sin(2 * np.pi * (34 * yy + 1.6 * warp))
    grain = 0.5 + 0.5 * np.sign(lines) * np.abs(lines) ** 0.4
    fine = rng.normal(0, 1, (px, px))
    raw = np.array([212, 182, 136], float)
    img = raw[None, None] * (1 - 0.16 * grain[..., None]) + 5 * fine[..., None]
    for _ in range(2):           # budak: koyu çekirdek, etrafında halka
        cx, cy = rng.uniform(0.15, 0.85, 2)
        r = np.hypot((xx - cx) * 0.6, yy - cy)
        img *= (1 - 0.45 * np.exp(-(r / 0.018) ** 2))[..., None]
        img *= (1 - 0.10 * np.exp(-((r - 0.035) / 0.008) ** 2))[..., None]
    if paint is not None:
        p = np.array(paint, float)
        wear = _tileable_field(px, rng, 8, 3) + 0.15 * fine
        keep = np.clip((wear + 0.55) * 4, 0, 1)[..., None]   # 1 = boya duruyor
        painted = p[None, None] * (1 - 0.10 * grain[..., None]) + 3 * fine[..., None]
        img = keep * painted + (1 - keep) * img
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), "RGB")


BEAM_TILE_U, BEAM_TILE_V = 0.50, 0.10


def beam_texture(w: int = 320, h: int = 64, seed: int = 5) -> "Image.Image":
    """Boyalı çelik kiriş profili: üstte parlak büküm, altta gölge, üst
    kenara yakın basamak (step) çizgisi, yer yer çizik ve forklift izi."""
    rng = np.random.default_rng(seed)
    v = np.linspace(0, 1, h)[:, None]
    shade = (1.12 - 0.30 * v) * np.ones((1, w))
    shade[: h // 16] *= 1.15                          # üst büküm parlak
    shade[-h // 12:] *= 0.70                          # alt büküm gölgede
    shade[h // 5: h // 5 + 2] *= 0.55                 # basamak çizgisi
    base = np.array([214, 86, 18], float)
    img = base[None, None] * shade[..., None] + rng.normal(0, 3, (h, w, 1))
    for _ in range(14):                               # çizikler: ince, yatay
        x0 = rng.integers(0, w - 20)
        y0 = rng.integers(h // 4, h - 4)
        ln = rng.integers(6, 30)
        img[y0, x0:x0 + ln] = img[y0, x0:x0 + ln] * 0.6 + np.array([120, 110, 100]) * 0.4
    return Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), "RGB")


def beam_obj(length: float, ft: float) -> str:
    """Kiriş ve iki ucundaki bağlantı plakası tek mesh'te. Plaka kirişten
    3 cm taşar, dikmeye bu plakayla asılır; kirişin içinde kalır ki
    dikmeye girmesin."""
    q = box_quads(0, 0, 0, length, ft, ft)
    for sx in (-1, 1):
        q += box_quads(sx * (length / 2 - 0.006), 0, 0, 0.012, ft * 1.04, ft + 0.06)
    return quads_obj(world_uv(q, BEAM_TILE_U, BEAM_TILE_V),
                     f"Kiriş {length*1000:.0f} mm, uçta bağlantı plakası")


def pallet_obj(w: float, d: float) -> str:
    """Euro palet: üstte boyuna tahtalar (aralarında boşluk), üç kuşak, 3x3
    takoz, üç alt kızak. Önden bakınca gerçek paletteki iki forklift
    boşluğu görünür. Orijin paletin geometrik merkezi. UV gerçek boyutta,
    lif her parçanın boyunca."""
    t = 0.022                                  # tahta kalınlığı
    bh = PALLET_H - 3 * t                      # takoz yüksekliği (78 mm)
    bw = min(0.145, w * 0.22)                  # dar paletlerde takoz daralır
    bd = 0.10
    z0 = -PALLET_H / 2
    xs = (-w / 2 + bw / 2, 0.0, w / 2 - bw / 2)
    ys = (-d / 2 + bd / 2, 0.0, d / 2 - bd / 2)
    # üst tahtalar: Euro'da 145/100/145/100/145 mm, dar palette üç tahta
    pattern = (1.45, 1.0, 1.45, 1.0, 1.45) if w >= 0.6 else (1.45, 1.0, 1.45)
    cover = 0.78 * w
    widths = [cover * k / sum(pattern) for k in pattern]
    gap = (w - cover) / (len(widths) - 1)
    q = []
    x = -w / 2
    for bwid in widths:
        q += box_quads(x + bwid / 2, 0, z0 + PALLET_H - t / 2, bwid, d, t)  # üst tahta
        x += bwid + gap
    for y in ys:
        q += box_quads(0, y, z0 + t + bh + t / 2, w, bd, t)               # kuşak
    for x in xs:
        for y in ys:
            q += box_quads(x, y, z0 + t + bh / 2, bw, bd, bh)            # takoz
        q += box_quads(x, 0, z0 + t / 2, bw, d, t)                       # alt kızak
    return quads_obj(world_uv(q, WOOD_TILE_M, along_long=True),
                     f"Euro palet {w*1000:.0f}x{d*1000:.0f} mm")


def racking(cfg, textures, meshes) -> str:
    rk = cfg["racking"]
    bw, nb, depth = rk["bay_width"], rk["bay_count"], rk["depth"]
    ft, uh = rk["frame_thickness"], rk["upright_height"]
    levels, x0 = rk["level_heights"], rk["x_origin"]
    yaw = world_yaw_rad(cfg)
    real = realism(cfg)

    if real["rack"]:
        # Fotoğraflardaki gibi: mavi delikli dikme, turuncu kiriş.
        beam_rgb = (0.82, 0.28, 0.04)
        textures["beam.png"] = beam_texture()
        meshes["beam.obj"] = beam_obj(bw - ft, ft)
        textures["upright.png"] = upright_texture()
        meshes["upright.obj"] = quads_obj(
            box_quads(0, 0, 0, ft, ft, uh, ru=1.0, rv=uh / UPRIGHT_TILE_M),
            f"Dikme {ft*1000:.0f}x{ft*1000:.0f}x{uh*1000:.0f} mm, delik dokusu tekrarlı")
    else:
        upright_rgb = (0.72, 0.30, 0.08)   # turuncu dikme
        beam_rgb = (0.10, 0.26, 0.55)      # mavi kiriş
    deck_rgb = (0.55, 0.56, 0.58)      # galvaniz raf tablası
    brace_rgb = (0.05, 0.24, 0.52)
    # Paletliyse yük paletin üstüne oturur ve level_heights yükün kotudur;
    # kirişler palet boyu kadar aşağı iner, kutular yerinde kalır. Gerçek
    # paletli rafta tabla yoktur, palet ön ve arka kirişe basar.
    drop = PALLET_H if real["pallets"] else 0.0

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
                if real["rack"]:
                    out.append(mesh_visual(f"up_{i}_{tag}_v", "upright.obj", (x, y, uh / 2),
                                           None, "      ", texture="upright.png",
                                           roughness=0.6))
                else:
                    out.append(box_visual(f"up_{i}_{tag}_v", (ft, ft, uh), (x, y, uh / 2),
                                          upright_rgb, "      "))
                out.append(box_collision(f"up_{i}_{tag}_c", (ft, ft, uh), (x, y, uh / 2), "      "))
            if real["rack"]:
                out.append(frame_bracing(f"br_{i}", x, y0 + ft, y0 + depth - ft, uh,
                                         brace_rgb))

        for li, z in enumerate(levels):
            zb = z - drop
            for bi in range(nb):
                xc = x0 + (bi + 0.5) * bw
                # ön ve arka kirişler
                for tag, y in (("f", y0 + ft / 2), ("b", y0 + depth - ft / 2)):
                    if real["rack"]:
                        out.append(mesh_visual(f"beam_{li}_{bi}_{tag}_v", "beam.obj",
                                               (xc, y, zb - ft / 2), None, "      ",
                                               texture="beam.png", roughness=0.4,
                                               specular=0.3))
                    else:
                        out.append(box_visual(f"beam_{li}_{bi}_{tag}_v", (bw - ft, ft, ft),
                                              (xc, y, zb - ft / 2), beam_rgb, "      "))
                    out.append(box_collision(f"beam_{li}_{bi}_{tag}_c", (bw - ft, ft, ft),
                                             (xc, y, zb - ft / 2), "      "))
                if real["pallets"]:
                    continue
                # raf tablası: kutuların üstünde durduğu yüzey
                out.append(box_visual(f"deck_{li}_{bi}_v", (bw - ft, depth - 2 * ft, 0.02),
                                      (xc, yc, z - 0.01), deck_rgb, "      "))
                out.append(box_collision(f"deck_{li}_{bi}_c", (bw - ft, depth - 2 * ft, 0.02),
                                         (xc, yc, z - 0.01), "      "))
        out.append("    </link>\n  </model>\n")
    return "".join(out)


def frame_bracing(name: str, x: float, y_f: float, y_b: float, uh: float, rgb) -> str:
    """Yan çerçevenin çapraz bağlantısı: ön ve arka dikme arasında altta ve
    üstte yatay, aralarında zikzak. Çerçeve düzlemi göz sınırında, kameranın
    baktığı ön yüze dik -- kutuların önünü kapatmaz. Yalnız görsel; çarpışma
    kutusu yok, raf zarfının içinde kalıyor ve drone oraya girmiyor."""
    tx, tz = 0.03, 0.04
    span = y_b - y_f
    z_lo, z_hi = 0.15, uh - 0.15
    n = max(1, round((z_hi - z_lo) / 1.0))
    h = (z_hi - z_lo) / n
    ym = (y_f + y_b) / 2
    out = []
    for tag, z in (("lo", z_lo), ("hi", z_hi)):
        out.append(box_visual(f"{name}_{tag}_v", (tx, span, tz), (x, ym, z), rgb, "      "))
    length = math.hypot(span, h)
    for k in range(n):
        za = z_lo + k * h
        # zikzak: bir çubuk önden arkaya yükselir, sonraki arkadan öne
        roll = math.atan2(h, span if k % 2 == 0 else -span)
        out.append(box_visual(f"{name}_d{k}_v", (tx, length, tz), (x, ym, za + h / 2),
                              rgb, "      ", rpy=(roll, 0.0, 0.0)))
    return "".join(out)


# Koli baskısı. Markalar uydurma; gerçek bir markanın adı ya da logosu
# kullanılmıyor. Renkler fotoğraflardaki mürekkeplerden: pembe, mavi, yeşil,
# lacivert, kırmızı, turuncu.
CARTON_BRANDS = [
    ("NOVA", (214, 64, 140)), ("PERLA", (38, 108, 186)), ("VITA", (34, 132, 70)),
    ("ZEFIR", (32, 44, 92)), ("AKSU", (40, 140, 200)), ("SOLE", (206, 46, 44)),
    ("ORKA", (28, 28, 30)), ("LUMA", (228, 118, 22)), ("TERRA", (120, 70, 40)),
    ("MAVI", (24, 84, 160)),
]
CARTON_PPM = 500          # baskı dokusu çözünürlüğü (piksel / m)
CARTON_TAPE_M = 0.048     # koli bandı eni


def carton_regions(dx: float, dy: float, dz: float, ppm: int = CARTON_PPM):
    """Koli dokusunun yerleşimi (piksel, y aşağı): üst sırada ön yüz (dx x dz)
    ve yan yüz (dy x dz), altında kapak (dx x dy). Ön/arka ve iki yan aynı
    bölgeyi paylaşır -- gerçekte de karşılıklı yüzler aynı basılır."""
    fx, fz, sy = round(dx * ppm), round(dz * ppm), round(dy * ppm)
    W, H = fx + sy, fz + sy
    return W, H, {"front": (0, 0, fx, fz), "side": (fx, 0, W, fz), "top": (0, fz, fx, H)}


def carton_obj(dx: float, dy: float, dz: float) -> str:
    W, H, reg = carton_regions(dx, dy, dz)

    def uv(r):
        x0, y0, x1, y1 = r
        u0, u1, vb, vt = x0 / W, x1 / W, 1 - y1 / H, 1 - y0 / H
        return ((u0, vb), (u1, vb), (u1, vt), (u0, vt))

    q = box_quads(0, 0, 0, dx, dy, dz)
    # box_quads sırası: +x, -x, +y, -y, üst, alt. Ürün yüzü ±y (koridor).
    faces = ["side", "side", "front", "front", "top", "top"]
    q = [(c, n, uv(reg[f])) for (c, n, _), f in zip(q, faces)]
    return quads_obj(q, f"Koli {dx*1000:.0f}x{dy*1000:.0f}x{dz*1000:.0f} mm, baskı atlası")


def carton_brand(link: str) -> int:
    """Kolinin markası. rng ÇEKİLMEZ (bkz. pallet_look)."""
    return zlib.crc32(link.encode()) % len(CARTON_BRANDS)


def carton_texture(dx: float, dy: float, dz: float, brand: int) -> "Image.Image":
    """Baskılı oluklu mukavva: kraft zemin, oluk izi, kenar gölgesi, marka,
    ürün numarası, koli bandı, yan yüzde "bu taraf yukarı" okları.
    Sahte barkod BİLEREK yok: okuyucu onu kod sanıp envantere yazabilirdi.
    Yazı tipi Pillow'un gömülü fontu -- iki makinede de aynı doku çıksın."""
    from PIL import ImageDraw, ImageFont
    W, H, reg = carton_regions(dx, dy, dz)
    name, ink = CARTON_BRANDS[brand]
    seed = zlib.crc32(f"{name}{dx}{dy}{dz}".encode())
    rng = np.random.default_rng(seed)

    # kraft zemin: ince benek + iri leke + oluk çizgileri (~8 mm adım)
    yy, xx = np.mgrid[0:H, 0:W]
    fine = rng.normal(0, 5.0, (H, W))
    blot = rng.normal(0, 1.0, (max(2, H // 40), max(2, W // 40)))
    blot = np.asarray(Image.fromarray(((blot - blot.min()) / (np.ptp(blot) + 1e-9) * 255)
                                      .astype(np.uint8)).resize((W, H), Image.BILINEAR),
                      dtype=np.float64) / 255 - 0.5
    flute = 2.5 * np.sin(2 * np.pi * yy / (0.008 * CARTON_PPM))
    g = fine + 14 * blot + flute
    kraft = np.array([192, 154, 106], dtype=np.float64)
    img = np.clip(kraft[None, None, :] * (1 + g[..., None] / 255 * 2.2), 0, 255)
    # yüz kenarları biraz koyu: katlama ve kir
    for x0, y0, x1, y1 in reg.values():
        e = max(3, (x1 - x0 + y1 - y0) // 60)
        sub = img[y0:y1, x0:x1]
        dist = np.minimum.reduce([xx[y0:y1, x0:x1] - x0, x1 - 1 - xx[y0:y1, x0:x1],
                                  yy[y0:y1, x0:x1] - y0, y1 - 1 - yy[y0:y1, x0:x1]])
        sub *= (0.86 + 0.14 * np.clip(dist / e, 0, 1))[..., None]
    base = Image.fromarray(img.astype(np.uint8), "RGB")

    ink_layer = Image.new("RGBA", (W, H), (0, 0, 0, 0))
    d = ImageDraw.Draw(ink_layer)
    dark = (44, 40, 36)

    def font(px):
        return ImageFont.load_default(size=max(6, int(px)))

    def fit(text, px, max_w):
        f = font(px)
        while px > 6 and d.textlength(text, font=f) > max_w:
            px *= 0.9
            f = font(px)
        return f

    number = f"{68000000 + seed % 999999:08d}"
    pack = f"{4 + seed % 3 * 4}x{1 + seed % 3}"

    # ön yüz: marka sol üstte, logo halkası, numara sağ altta
    x0, y0, x1, y1 = reg["front"]
    fw, fh = x1 - x0, y1 - y0
    m = int(0.06 * min(fw, fh))
    f = fit(name, fh * 0.17, fw * 0.55)
    d.text((x0 + m, y0 + m), name, font=f, fill=ink + (225,),
           stroke_width=max(1, f.size // 12), stroke_fill=ink + (225,))
    r = f.size * 0.45
    cx_, cy_ = x0 + m + d.textlength(name, font=f) + r * 1.6, y0 + m + f.size * 0.55
    d.ellipse([cx_ - r, cy_ - r, cx_ + r, cy_ + r], outline=ink + (220,), width=max(2, int(r / 4)))
    fn = fit(number, fh * 0.07, fw * 0.4)
    d.text((x1 - m - d.textlength(number, font=fn), y1 - m - fn.size), number, font=fn,
           fill=dark + (210,))
    fp = font(fh * 0.065)
    d.rectangle([x0 + m, y1 - m - fp.size * 1.3, x0 + m + d.textlength(pack, font=fp) + fp.size * 0.6,
                 y1 - m], outline=dark + (200,), width=2)
    d.text((x0 + m + fp.size * 0.3, y1 - m - fp.size * 1.15), pack, font=fp, fill=dark + (210,))

    # ürün paneli sağ üstte (şişe silüeti) ve alt tarafta boydan boya şerit
    pw_, ph_ = fh * 0.30, fh * 0.30
    px0, py0 = x1 - m - pw_, y0 + m
    d.rounded_rectangle([px0, py0, px0 + pw_, py0 + ph_], radius=pw_ * 0.12,
                        outline=ink + (215,), width=max(2, int(fh / 90)))
    bx0, bx1 = px0 + pw_ * 0.36, px0 + pw_ * 0.64
    d.rectangle([bx0, py0 + ph_ * 0.38, bx1, py0 + ph_ * 0.86], fill=ink + (190,))
    d.rectangle([px0 + pw_ * 0.45, py0 + ph_ * 0.18, px0 + pw_ * 0.55, py0 + ph_ * 0.38],
                fill=ink + (190,))
    sy_ = y0 + fh * 0.74
    d.rectangle([x0, sy_, x1, sy_ + fh * 0.03], fill=ink + (170,))

    # yan yüz: marka, "bu taraf yukarı" okları, numara
    x0, y0, x1, y1 = reg["side"]
    sw, sh = x1 - x0, y1 - y0
    m = int(0.07 * min(sw, sh))
    f = fit(name, sh * 0.13, sw * 0.7)
    d.text((x0 + m, y0 + sh * 0.42), name, font=f, fill=ink + (225,),
           stroke_width=max(1, f.size // 12), stroke_fill=ink + (225,))
    a = sh * 0.09
    for k in range(2):
        ax = x1 - m - a * (2.4 - 1.2 * k)
        ay = y0 + m
        d.polygon([(ax, ay + a * 0.5), (ax + a * 0.5, ay), (ax + a, ay + a * 0.5)], fill=dark + (220,))
        d.rectangle([ax + a * 0.38, ay + a * 0.5, ax + a * 0.62, ay + a * 1.3], fill=dark + (220,))
    fn = fit(number, sh * 0.07, sw * 0.6)
    d.text((x0 + m, y1 - m - fn.size), number, font=fn, fill=dark + (210,))

    # kapak: ortadaki ek yeri ve boydan boya bant; bant ön/arka yüze de iner
    x0, y0, x1, y1 = reg["top"]
    tw = CARTON_TAPE_M * CARTON_PPM
    ym = (y0 + y1) / 2
    tape = (214, 186, 132, 150)
    d.rectangle([x0, ym - tw / 2, x1, ym + tw / 2], fill=tape)
    d.line([x0, ym, x1, ym], fill=(70, 52, 32, 200), width=2)
    fx0, _, fx1, _ = reg["front"]
    fc = (fx0 + fx1) / 2
    d.rectangle([fc - tw / 2, 0, fc + tw / 2, 0.06 * CARTON_PPM], fill=tape)

    return Image.alpha_composite(base.convert("RGBA"), ink_layer).convert("RGB")


# Streç film: kolinin dışında ince, parlak, yarı saydam bir kabuk.
FILM_GAP = 0.010          # kolinin yüzünden uzaklık; etiket 4 mm'de, film önünde
FILM_DROP = 0.05          # paletliyse film palet üst tahtasına iner
FILM_TILE_M = 0.30        # kırışık dokusunun bir karosu
FILM_TRANSPARENCY = 0.62  # 0 opak, 1 görünmez
FILM_SHARE = 0.75         # filmli kolilerin oranı; gerisi çıplak (fotoğraf 2)


def film_texture(px: int = 128, seed: int = 11) -> "Image.Image":
    """Tekrarlanabilir kırışık doku: açık gri zemin, eğik parlak şeritler ve
    ince kırışık çizgileri. Saydamlık malzemeden gelir; doku yalnız filmin
    nerede daha beyaz, nerede daha açık göründüğünü belirler."""
    rng = np.random.default_rng(seed)
    yy, xx = np.mgrid[0:px, 0:px] / px
    g = np.full((px, px), 185.0)
    for _ in range(6):          # eğik şeritler; karo kenarında sürekli kalsın diye tam periyot
        k = rng.integers(1, 4)
        ph = rng.uniform(0, 2 * np.pi)
        g += rng.uniform(12, 34) * np.sin(2 * np.pi * k * (xx + yy) + ph)
    for _ in range(10):         # kırışıklar
        k = rng.integers(2, 7)
        ph = rng.uniform(0, 2 * np.pi)
        g += 55 * np.maximum(0, np.sin(2 * np.pi * k * (xx - 2 * yy) + ph)) ** 24
    a = np.clip(g, 150, 255).astype(np.uint8)
    return Image.fromarray(np.stack([a, a, np.clip(a.astype(int) + 6, 0, 255).astype(np.uint8)],
                                    -1), "RGB")


def film_wanted(link: str) -> bool:
    """Bu koli filmli mi. rng ÇEKİLMEZ (bkz. pallet_look)."""
    return (zlib.crc32(("film" + link).encode()) % 100) < FILM_SHARE * 100


def film_obj(dx: float, dy: float, dz: float, drop: float) -> str:
    """Kolinin yan yüzleri ve kapağı, FILM_GAP kadar dışarıda; alt yüz yok.
    Orijin kolinin merkezi -- kabuk aşağıya `drop` kadar uzanır."""
    sx, sy, sz = dx + 2 * FILM_GAP, dy + 2 * FILM_GAP, dz + FILM_GAP + drop
    cz = (FILM_GAP - drop) / 2
    q = box_quads(0, 0, cz, sx, sy, sz, ru=max(sx, sy) / FILM_TILE_M, rv=sz / FILM_TILE_M)
    return quads_obj(q[:5], f"Streç film {sx*1000:.0f}x{sy*1000:.0f}x{sz*1000:.0f} mm")


def film_visual(name: str, mesh: str, xyz, ind="        ") -> str:
    """Yalnız görsel, çarpışma yok: film kolinin zarfında kalıyor. Parlak
    (düşük pürüzlülük, yüksek specular) -- lambaların yansıması kodun
    üstüne düşebilir, deneyin konusu da bu."""
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz)}</pose>
{ind}  <geometry><mesh><uri>model://{ASSET_MODEL}/meshes/{mesh}</uri></mesh></geometry>
{ind}  <transparency>{fmt(FILM_TRANSPARENCY)}</transparency>
{ind}  <cast_shadows>false</cast_shadows>
{ind}  <material>
{ind}    <ambient>0.9 0.9 0.92 1</ambient>
{ind}    <diffuse>0.95 0.95 0.97 1</diffuse>
{ind}    <specular>0.8 0.8 0.8 1</specular>
{ind}    <pbr><metal>
{ind}      <albedo_map>model://{ASSET_MODEL}/materials/textures/film.png</albedo_map>
{ind}      <metalness>0.0</metalness>
{ind}      <roughness>0.15</roughness>
{ind}    </metal></pbr>
{ind}  </material>
{ind}</visual>
"""


PALLET_BLUE = (34, 78, 150)


def pallet_look(name: str, textures: dict) -> tuple:
    """Paletin dokusu ve tonu: çoğu ham tahta (beyazla sarı arası), bir
    kısmı mavi boyalı kiralık palet. Karışık dağılır ama her üretimde aynı
    çıkar: seçim adın özetinden yapılır, rng ÇEKİLMEZ -- inventory'nin
    çekiliş sırası değişirse bütün SKU'lar ve kutu renkleri kayardı."""
    h = zlib.crc32(name.encode())
    shade = 0.86 + 0.14 * ((h >> 8) % 11) / 10        # palet palet ton farkı
    blue = h % 5 < 2                                    # %40 mavi
    tex = "wood_blue.png" if blue else "wood_raw.png"
    if tex not in textures:
        textures[tex] = wood_texture(PALLET_BLUE if blue else None)
    return tex, (min(1.0, shade),) * 3


def pallets(cfg, slots, meshes, textures) -> str:
    """Her kutunun altına bir Euro palet. Palet üstü = level_heights kotu,
    yani kutuların oturduğu yer; kutular ve kodlar yerinden oynamaz.
    Palet derinliği raf derinliği: gerçekte 1200 mm palet kirişten 50'şer mm
    taşar, ama en dar koridor 0.50 m ve o 10 cm drone'un payından yenirdi."""
    rk = cfg["racking"]
    bw, depth, ft, x0 = rk["bay_width"], rk["depth"], rk["frame_thickness"], rk["x_origin"]
    rows = {r["id"]: r for r in rk["rows"]}
    out = ['  <model name="pallets">\n    <static>true</static>\n'
           + model_pose_tag(world_yaw_rad(cfg)) + '    <link name="pallets">\n']

    by_slot: dict = {}
    for s in slots:
        by_slot.setdefault((s["row"], s["bay"], s["level"]), []).append(s)
    for (rid, bi, li), group in by_slot.items():
        group.sort(key=lambda s: s["cx"])
        # paletler birbirine ve dikmelere girmesin
        left = x0 + bi * bw + ft / 2
        right = x0 + (bi + 1) * bw - ft / 2
        edges = [left]
        for s in group:
            w = min(PALLET_W, s["dx"] + 2 * PALLET_OVERHANG)
            s["w"] = w
            edges += [s["cx"] - w / 2, s["cx"] + w / 2]
        edges.append(right)
        if any(b < a - 1e-9 for a, b in zip(edges, edges[1:])):
            raise SystemExit(f"{rid} göz {bi+1} seviye {li+1}: paletler sığmıyor "
                             f"({', '.join(f'{s['w']:.2f}' for s in group)} m)")
        y0 = rows[rid]["y0"]
        for s in group:
            w = s["w"]
            mesh = f"pallet_{w*1000:.0f}.obj"
            if mesh not in meshes:
                meshes[mesh] = pallet_obj(w, depth)
            name = f"p_{rid}_{bi+1:02d}_{li+1}_{s['si']}"
            tex, tint = pallet_look(name, textures)
            z = s["z"] - PALLET_H / 2
            out.append(mesh_visual(f"{name}_v", mesh, (s["cx"], y0 + depth / 2, z),
                                   tint, "      ", texture=tex, roughness=0.9))
            out.append(box_collision(f"{name}_c", (w, depth, PALLET_H),
                                     (s["cx"], y0 + depth / 2, z), "      "))
    out.append("    </link>\n  </model>\n")
    print(f"  palet          : {len(slots)}")
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


def inventory(cfg, rng, textures, manifest, pallet_slots=None, meshes=None,
              film_meshes=None) -> str:
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
                    if pallet_slots is not None:
                        pallet_slots.append({"row": rid, "bay": bi, "level": li, "si": si,
                                             "cx": cx, "dx": dx, "z": z})

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
                    img, module_m = gl.make_box_label(payload, sku, spec, ppm, maxpx)
                    pc_img, pc_module_m = gl.make_bay_placard(pc_payload, pc_caption,
                                                              pc_spec, ppm, maxpx)
                    if not bare:
                        textures[tex] = img
                        textures[pc_tex] = pc_img

                    link = f"box_{rid}_{bi+1:02d}_{li+1}_{si}"
                    shade = rng.uniform(0.88, 1.06)
                    cardboard = tuple(min(1.0, c * shade) for c in (0.68, 0.52, 0.34))

                    out.append(f'    <link name="{link}">\n')
                    if meshes is not None:
                        # Baskılı koli. Doku boy+marka başına bir tane; aynı
                        # ürün aynı baskıyla gelir. Kutudan kutuya ton farkı
                        # eskisi gibi `shade`'den.
                        brand = carton_brand(link)
                        ctex = f"carton_{size['name']}_{brand}.png"
                        cmesh = f"carton_{size['name']}.obj"
                        if ctex not in textures:
                            textures[ctex] = carton_texture(dx, dy, dz, brand)
                        if cmesh not in meshes:
                            meshes[cmesh] = carton_obj(dx, dy, dz)
                        tint = (min(1.0, shade),) * 3
                        out.append(mesh_visual("body", cmesh, (cx, cy, cz), tint, "      ",
                                               texture=ctex))
                    else:
                        out.append(box_visual("body", (dx, dy, dz), (cx, cy, cz), cardboard,
                                              "      "))
                    out.append(box_collision("body_c", (dx, dy, dz), (cx, cy, cz), "      "))
                    if film_meshes is not None and film_wanted(link):
                        drop = FILM_DROP if pallet_slots is not None else 0.0
                        fmesh = f"film_{size['name']}{'_p' if drop else ''}.obj"
                        if fmesh not in film_meshes:
                            film_meshes[fmesh] = film_obj(dx, dy, dz, drop)
                            textures.setdefault("film.png", film_texture())
                        out.append(film_visual("film", fmesh, (cx, cy, cz), "      "))

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
                    if not bare:
                        out.append(label_visual("label", tex, (lw, lh),
                                                (cx, y_label + off, qr_z, *rpy), "      "))
                        out.append(label_visual("placard", pc_tex, (pw, ph),
                                                (cx, y_label + off, pc_z, *rpy), "      "))
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
                            "label_pose_xyzrpy": [round(cx, 4), round(y_label + off, 4),
                                                  round(qr_z, 4), *[round(v, 6) for v in rpy]],
                            "label_size_m": [lw, lh],
                            "normal": [0.0, float(facing), 0.0],
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


# --------------------------------------------------------------------------
# bina ayrıntıları (realism.building)
# --------------------------------------------------------------------------

WALL_TILE_U = 4.0         # duvar dokusu yatayda 4 m'de bir tekrarlar


def wall_texture(H: float, px_w: int = 400, seed: int = 9) -> "Image.Image":
    """Bir duvar şeridi, 4 m x tam yükseklik: altta 1.2 m koyu gri boyalı
    süpürgelik ve sarı şerit, üstünde badanalı briket (400 x 200 mm), en
    üstte 3 m oluklu sac kaplama. Fotoğraf 2'deki duvar."""
    from PIL import ImageDraw
    rng = np.random.default_rng(seed)
    ppm = px_w / WALL_TILE_U
    ph = int(round(H * ppm))
    img = np.empty((ph, px_w, 3))
    img[:] = (214, 211, 203)
    img += rng.normal(0, 4, (ph, px_w, 1))
    img += 10 * _tileable_field(px_w, rng, 8, 3)[np.arange(ph) % px_w][..., None]
    im = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8), "RGB")
    d = ImageDraw.Draw(im)
    z2y = lambda z: ph - 1 - z * ppm          # noqa: E731  (z m -> piksel satırı)
    # briket derzleri
    clad = H - 3.0
    k = 0
    z = 1.2
    while z < clad:
        y = z2y(z)
        d.line([0, y, px_w, y], fill=(184, 181, 174), width=1)
        off = 0 if k % 2 == 0 else 0.2 * ppm
        x = off
        while x < px_w:
            d.line([x, z2y(min(z + 0.2, clad)), x, y], fill=(184, 181, 174), width=1)
            x += 0.4 * ppm
        z += 0.2
        k += 1
    # oluklu sac: dikey nervürler
    d.rectangle([0, 0, px_w, z2y(clad)], fill=(176, 182, 188))
    x = 0.0
    while x < px_w:
        d.rectangle([x, 0, x + 0.04 * ppm, z2y(clad)], fill=(150, 156, 162))
        x += 0.2 * ppm
    # süpürgelik ve sarı şerit
    d.rectangle([0, z2y(1.2), px_w, ph], fill=(92, 96, 100))
    d.rectangle([0, z2y(1.25), px_w, z2y(1.2)], fill=(226, 184, 30))
    return im


def walls_obj(L: float, W: float, H: float) -> str:
    """Dört duvarın İÇ yüzleri, 5 mm içeride. Normal içeri bakar; u duvar
    boyunca, v tabandan tavana 0..1 (doku tam yüksekliği bir kez taşır)."""
    q = []
    e = 0.005
    for (px, py), (nx, ny), ln in (((L / 2 - e, 0), (-1, 0), W), ((-L / 2 + e, 0), (1, 0), W),
                                   ((0, W / 2 - e), (0, -1), L), ((0, -W / 2 + e), (0, 1), L)):
        rx, ry = -ny, nx                    # sağ = z x n
        a = (px - rx * ln / 2, py - ry * ln / 2)
        b = (px + rx * ln / 2, py + ry * ln / 2)
        ru = ln / WALL_TILE_U
        q.append(([(a[0], a[1], 0), (b[0], b[1], 0), (b[0], b[1], H), (a[0], a[1], H)],
                  (nx, ny, 0), ((0, 0), (ru, 0), (ru, 1), (0, 1))))
    return quads_obj(q, "Duvar iç yüzleri")


def cyl_visual(name, r, ln, xyz, rpy, rgb, ind="      ", rough=0.6, spec=0.1) -> str:
    cr, cg, cb = rgb
    return f"""{ind}<visual name="{name}">
{ind}  <pose>{pose(*xyz, *rpy)}</pose>
{ind}  <geometry><cylinder><radius>{fmt(r)}</radius><length>{fmt(ln)}</length></cylinder></geometry>
{ind}  <material>
{ind}    <ambient>{fmt(cr*0.5)} {fmt(cg*0.5)} {fmt(cb*0.5)} 1</ambient>
{ind}    <diffuse>{fmt(cr)} {fmt(cg)} {fmt(cb)} 1</diffuse>
{ind}    <specular>{fmt(spec)} {fmt(spec)} {fmt(spec)} 1</specular>
{ind}    <pbr><metal><metalness>0.0</metalness><roughness>{fmt(rough)}</roughness></metal></pbr>
{ind}  </material>
{ind}</visual>
"""


def building_details(cfg, textures, meshes) -> str:
    """Fotoğraflardaki depo ayrıntıları: dokulu duvar, tavan makasları ve
    havalandırma kanalı, zeminde sarı yürüme yolu ve taralı yasak alanlar,
    duvar dibinde yangın dolabı, tüp, sarı-siyah koruma direkleri, sarmal
    kapı, boş palet yığını. Hepsi duvar dibinde ya da zeminde düz; raf
    bloğuna ve uçuş koridorlarına hiçbir şey girmiyor."""
    b, rk = cfg["building"], cfg["racking"]
    L, W, H = b["length"], b["width"], b["height"]
    x0, nb, bw, depth = rk["x_origin"], rk["bay_count"], rk["bay_width"], rk["depth"]
    out = ['  <model name="building_details">\n    <static>true</static>\n'
           + model_pose_tag(world_yaw_rad(cfg)) + '    <link name="details">\n']
    ind = "      "
    yellow, black, red = (0.90, 0.72, 0.08), (0.08, 0.08, 0.09), (0.72, 0.07, 0.06)

    # duvarlar
    textures["wall.png"] = wall_texture(H)
    meshes["walls.obj"] = walls_obj(L, W, H)
    out.append(mesh_visual("walls_v", "walls.obj", (0, 0, 0), None, ind, texture="wall.png",
                           roughness=0.95))

    # tavan: Y boyunca makaslar (6 m arayla), X boyunca aşıklar, iki kanal.
    # Lambalar 6 m'de ve gölgesiz; bunlar 8.5 m'nin üstünde.
    steel = (0.30, 0.32, 0.35)
    xt = -L / 2 + 3.0
    i = 0
    while xt < L / 2:
        out.append(box_visual(f"truss_{i}", (0.20, W, 0.50), (xt, 0, H - 0.25), steel, ind))
        xt += 6.0
        i += 1
    for j, yp in enumerate(np.arange(-W / 2 + 4, W / 2, 4.0)):
        out.append(box_visual(f"purlin_{j}", (L, 0.10, 0.15), (0, float(yp), H - 0.58), steel, ind))
    for j, yp in enumerate((-W / 4, W / 4)):
        out.append(cyl_visual(f"duct_{j}", 0.40, L, (0, yp, H - 1.3), (0, math.pi / 2, 0),
                              (0.74, 0.76, 0.78), ind, rough=0.3, spec=0.6))

    # zemin işaretleri: 2 mm kalınlık, collision yok
    zf, lw = 0.004, 0.10

    def stripe(name, x1, y1, x2, y2, rgb=yellow, w=lw):
        ln = math.hypot(x2 - x1, y2 - y1)
        yaw = math.atan2(y2 - y1, x2 - x1)
        return box_visual(name, (ln, w, 0.002), ((x1 + x2) / 2, (y1 + y2) / 2, zf), rgb, ind,
                          rpy=(0, 0, yaw))

    # her raf sırasının koridora bakan kenarı boyunca ince çizgi
    xa, xb = x0, x0 + nb * bw
    for row in rk["rows"]:
        ye = row["y0"] + depth + 0.05 if row["facing"] > 0 else row["y0"] - 0.05
        out.append(stripe(f"rowline_{row['id']}", xa, ye, xb, ye, w=0.05))
    # raf bloğunun etrafında yürüme yolu
    ys_ = [r["y0"] for r in rk["rows"]]
    m = 2.0
    bx0, bx1 = xa - m, xb + m
    by0, by1 = min(ys_) - m, max(ys_) + depth + m
    for k, (p1, p2) in enumerate((((bx0, by0), (bx1, by0)), ((bx1, by0), (bx1, by1)),
                                  ((bx1, by1), (bx0, by1)), ((bx0, by1), (bx0, by0)))):
        out.append(stripe(f"walk_{k}", *p1, *p2))
    # taralı yasak alanlar (fotoğraf 2), duvar diplerinde
    for k, (cx, cy) in enumerate(((L / 2 - 4, W / 2 - 6), (-L / 2 + 4, W / 2 - 6),
                                  (L / 2 - 4, -W / 2 + 6), (-L / 2 + 4, -W / 2 + 6))):
        hx, hy = 1.5, 1.0
        c = [(cx - hx, cy - hy), (cx + hx, cy - hy), (cx + hx, cy + hy), (cx - hx, cy + hy)]
        for e in range(4):
            out.append(stripe(f"zone_{k}_{e}", *c[e], *c[(e + 1) % 4]))
        out.append(stripe(f"zone_{k}_x1", *c[0], *c[2]))
        out.append(stripe(f"zone_{k}_x2", *c[1], *c[3]))

    # duvar dibi donanımı. Duvar boyunca 12 m'de bir yangın dolabı + tüp.
    def on_wall(px, py, nx, ny, d_off):
        """Duvar iç yüzünden d_off kadar içerideki nokta."""
        return px + nx * d_off, py + ny * d_off

    walls = (((L / 2, 0), (-1, 0), W), ((-L / 2, 0), (1, 0), W),
             ((0, W / 2), (0, -1), L), ((0, -W / 2), (0, 1), L))
    n_cab = 0
    for wi, ((px, py), (nx, ny), ln) in enumerate(walls):
        rx, ry = -ny, nx
        along = (nx == 0)
        for s_ in np.arange(-ln / 2 + 6, ln / 2 - 3, 12.0):
            s_ = float(s_)
            cx, cy = on_wall(px + rx * s_, py + ry * s_, nx, ny, 0.13)
            size = (0.75, 0.25, 0.90) if along else (0.25, 0.75, 0.90)
            out.append(box_visual(f"cab_{n_cab}", size, (cx, cy, 0.95), red, ind))
            out.append(box_collision(f"cab_{n_cab}_c", size, (cx, cy, 0.95), ind))
            gx, gy = on_wall(px + rx * s_, py + ry * s_, nx, ny, 0.256)
            gsize = (0.55, 0.004, 0.55) if along else (0.004, 0.55, 0.55)
            out.append(box_visual(f"cab_{n_cab}_glass", gsize, (gx, gy, 1.0),
                                  (0.70, 0.78, 0.80), ind))
            # üstünde yangın tabelası, yanında tüp
            sx_, sy_ = on_wall(px + rx * s_, py + ry * s_, nx, ny, 0.01)
            ssize = (0.30, 0.01, 0.30) if along else (0.01, 0.30, 0.30)
            out.append(box_visual(f"cab_{n_cab}_sign", ssize, (sx_, sy_, 1.75), red, ind))
            tx, ty = on_wall(px + rx * (s_ + 0.7), py + ry * (s_ + 0.7), nx, ny, 0.12)
            out.append(cyl_visual(f"ext_{n_cab}", 0.08, 0.50, (tx, ty, 0.35), (0, 0, 0), red, ind,
                                  rough=0.3, spec=0.5))
            out.append(cyl_visual(f"ext_{n_cab}_top", 0.03, 0.08, (tx, ty, 0.64), (0, 0, 0),
                                  black, ind))
            n_cab += 1

    # sarmal kapı (bir uç duvarda) ve iki yanında koruma direkleri
    # Kapı iki yangın dolabının arasında (dolaplar bu duvarda y=-2 ve 10).
    dx_, dy_, dw, dh = L / 2, 6.0, 4.0, 4.5
    out.append(box_visual("door", (0.06, dw, dh), (dx_ - 0.04, dy_, dh / 2), (0.58, 0.60, 0.62), ind))
    for k in range(int(dh / 0.15)):
        out.append(box_visual(f"door_rib_{k}", (0.02, dw, 0.012),
                              (dx_ - 0.075, dy_, 0.1 + k * 0.15), (0.44, 0.46, 0.48), ind))
    out.append(box_visual("door_sign", (0.01, 0.60, 0.30), (dx_ - 0.01, dy_ - dw / 2 - 0.8, 2.2),
                          (0.10, 0.30, 0.65), ind))
    n_b = 0
    for yb in (dy_ - dw / 2 - 0.4, dy_ + dw / 2 + 0.4):
        for xb_ in (dx_ - 0.6, dx_ - 1.6):
            out.append(cyl_visual(f"bollard_{n_b}", 0.08, 1.0, (xb_, yb, 0.5), (0, 0, 0),
                                  yellow, ind, rough=0.4, spec=0.3))
            for band in (0.30, 0.62):
                out.append(cyl_visual(f"bollard_{n_b}_b{band}", 0.082, 0.12, (xb_, yb, band),
                                      (0, 0, 0), black, ind))
            out.append(f"""{ind}<collision name="bollard_{n_b}_c"><pose>{pose(xb_, yb, 0.5)}</pose>
{ind}  <geometry><cylinder><radius>0.08</radius><length>1.0</length></cylinder></geometry></collision>
""")
            n_b += 1

    # boş palet yığınları (ham ve mavi), bir köşede
    if "wood_raw.png" not in textures:
        textures["wood_raw.png"] = wood_texture()
    if "wood_blue.png" not in textures:
        textures["wood_blue.png"] = wood_texture(PALLET_BLUE)
    meshes.setdefault("pallet_stack.obj", pallet_obj(PALLET_W, 1.20))
    for k, (cx, cy, n, tex) in enumerate(((-L / 2 + 1.0, W / 2 - 1.2, 8, "wood_raw.png"),
                                          (-L / 2 + 2.0, W / 2 - 1.2, 5, "wood_blue.png"),
                                          (-L / 2 + 1.0, W / 2 - 2.8, 3, "wood_raw.png"))):
        for j in range(n):
            out.append(mesh_visual(f"stack_{k}_{j}", "pallet_stack.obj",
                                   (cx, cy, PALLET_H / 2 + j * PALLET_H), None, ind,
                                   texture=tex))
        out.append(box_collision(f"stack_{k}_c", (PALLET_W, 1.20, n * PALLET_H),
                                 (cx, cy, n * PALLET_H / 2), ind))

    out.append("    </link>\n  </model>\n")
    print(f"  bina ayrıntısı : {n_cab} yangın dolabı, {n_b} koruma direği, kapı, palet yığını")
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

def build(cfg) -> tuple[str, list, dict, dict]:
    rng = random.Random(cfg["seed"])
    textures: dict = {}
    meshes: dict = {}
    manifest: list = []
    slots = [] if realism(cfg)["pallets"] else None
    lg = cfg["lighting"]
    ar, ag, ab, aa = lg["ambient"]
    br, bg, bb, ba = lg["background"]

    yaw = world_yaw_rad(cfg)
    check_aisles(cfg)

    # SIRA ÖNEMLİ. inventory() manifest'i CONFIG çerçevesinde doldurur;
    # rotate_manifest onu dünyaya çevirir. aisle_markers() ise zaten dünya
    # koordinatı yazdığı için dönüşten SONRA çağrılmalı -- önce çağrılsaydı
    # ArUco konumları bir kez fazladan dönerdi.
    body = [
        building(cfg, textures),
        racking(cfg, textures, meshes),
        inventory(cfg, rng, textures, manifest, slots,
                  meshes if realism(cfg)["cartons"] else None,
                  meshes if realism(cfg)["film"] else None),
    ]
    if slots is not None:
        body.append(pallets(cfg, slots, meshes, textures))
    rotate_manifest(manifest, yaw)
    body.append(aisle_markers(cfg, textures, manifest))
    if realism(cfg)["building"]:
        body.append(building_details(cfg, textures, meshes))
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
    return sdf, manifest, textures, meshes


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
    sdf, manifest, textures, meshes = build(cfg)

    assets = args.out / "gz" / "models" / ASSET_MODEL
    tex_dir = assets / "materials" / "textures"
    mesh_dir = assets / "meshes"
    for d in (tex_dir, mesh_dir, args.out / "gz" / "worlds", args.out / "out"):
        d.mkdir(parents=True, exist_ok=True)

    # eski dokuları temizle: tohum veya yerleşim değişince artık dosyalar kalmasın
    for old in (*tex_dir.glob("*.png"), *mesh_dir.glob("*.obj")):
        old.unlink()

    (assets / "model.config").write_text(MODEL_CONFIG)
    (assets / "model.sdf").write_text(ASSET_STUB_SDF)
    (mesh_dir / "label_quad.obj").write_text(LABEL_QUAD_OBJ)
    (mesh_dir / "floor_tile.obj").write_text(floor_mesh_obj(
        cfg["building"]["length"], cfg["building"]["width"], FLOOR_TILE_M))
    for name, text in meshes.items():
        (mesh_dir / name).write_text(text)
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
