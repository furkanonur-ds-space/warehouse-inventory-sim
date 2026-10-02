"""Zorlu senaryo: kutuları nizamdan çıkarır.

gen_world.py'nin kusursuz deposunda her koli rafın ön kenarından tam 20 mm
içeride, dimdik ve göz içinde eşit aralıkla duruyor. Gerçek depo böyle değil.
Bu modül her koliye EN FAZLA BİR bozulma verir ve ne verdiğini yer gerçeğine
yazar; böylece tek uçuş, bozulma türüne ve seviyesine göre ayrıştırılabilir.

NEDEN KOLİ BAŞINA TEK BOZULMA. Aynı koliye hem kayma hem dönme verilseydi o
kolinin okunmaması ikisinden hangisine yazılır, söylenemezdi. Bozulmalar
FARKLI kolilere dağıtılınca her biri ayrı bir deney, aynı uçuşta. Bozulmasız
kalanlar (`none`) aynı uçuşun kontrol grubu.

NEDEN YÜZ BAŞINA DÖNEN BİR SIRA. Rastgele atamada bir hücre (örneğin
"dönme, ağır") tesadüfen G yüzüne yığılabilirdi, ve G zaten en zor yüz: sonuç
bozulmayı değil yüzü ölçerdi. Her yüz kendi karıştırılmış hücre döngüsünü
sırayla tüketir, yani her hücre her yüze yaklaşık eşit düşer.

NEDEN KÜÇÜLTMEK YOK. Bir koli bir hücreyi tam büyüklüğüyle alamıyorsa
(komşusuna çarpar, rafın arkasından taşar ya da koridorda aracın payını
yer) o hücre küçültülüp verilmez; koli sıradaki sığan hücreyi alır, sığmayan
sonraki koliye kalır. Küçültülseydi "15 derece" diye raporlanan satırın
içinde 6 derecelik koliler olurdu.

AYRI TOHUM. Bu modül kendi random.Random'ını kullanır. gen_world'ün ana
üretecinden tek bir çekiliş fazla yapılsa SKU'lar ve renkler kayardı, ve
bozulmalı dünya temiz dünyayla kıyaslanamazdı. Kapalıyken hiçbir şey
çekilmez; dünya bit bit eskisiyle aynıdır.

Çerçeve: her şey CONFIG çerçevesinde (raf koşusu X, koridorlar Y). Dünyaya
dönüş gen_world'ün işi, burada yapılmaz.
"""

from __future__ import annotations

import json
import math
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

#: Sırası raporun da sırası.
ARMS = ("push_in", "pull_out", "slide", "yaw", "tilt", "empty")

#: Birimi: raporda ve yer gerçeğinde değerin yanında yazılır.
UNITS = {"push_in": "m", "pull_out": "m", "slide": "m",
         "yaw": "deg", "tilt": "deg", "empty": "", "none": ""}

#: İşaretli bozulmalar: sola/sağa, öne/arkaya rastgele.
SIGNED = {"slide", "yaw", "tilt"}

#: Koli ile komşusu, dikme veya raf arkası arasında her zaman kalacak boşluk.
#: Sıfır olursa iki koli aynı yüzeyi paylaşır ve z-fighting olur.
MARGIN_M = 0.005


@dataclass(frozen=True)
class Cell:
    arm: str           # ARMS'tan biri ya da "none"
    level: int         # 1 hafif, 2 orta, 3 ağır; none ve empty için 0/1
    value: float       # büyüklük, işaretsiz
    sign: int = 1

    @property
    def signed(self) -> float:
        return self.value * self.sign

    def record(self) -> dict:
        return {"arm": self.arm, "level": self.level,
                "value": round(self.signed, 6), "unit": UNITS[self.arm]}


CONTROL = Cell("none", 0, 0.0)


# --------------------------------------------------------------- dönüşüm

def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_to_mat(r: float, p: float, y: float) -> np.ndarray:
    """SDF sırası: R = Rz(y) Ry(p) Rx(r)."""
    cp, sp = math.cos(p), math.sin(p)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return rot_z(y) @ ry @ rot_x(r)


def mat_to_rpy(m: np.ndarray) -> tuple[float, float, float]:
    p = math.asin(max(-1.0, min(1.0, -m[2, 0])))
    if abs(math.cos(p)) > 1e-9:
        r = math.atan2(m[2, 1], m[2, 2])
        y = math.atan2(m[1, 0], m[0, 0])
    else:                                   # gimbal kilidi; burada oluşmaz
        r = math.atan2(-m[1, 2], m[1, 1])
        y = 0.0
    # Çarpım artığı 1e-17'ler ve -0.0'lar SDF'e öyle yazılmasın.
    return tuple(0.0 if abs(v) < 1e-12 else v + 0.0 for v in (r, p, y))


@dataclass
class Placed:
    """Bir kolinin bozulmadan sonraki hali, config çerçevesinde."""
    centre: np.ndarray       # gövde merkezi
    rot: np.ndarray          # 3x3, gövdenin yönelimi

    def point(self, local) -> np.ndarray:
        """Kolinin kendi çerçevesindeki bir noktanın yeri."""
        return self.centre + self.rot @ np.asarray(local, dtype=float)


def place(cell: Cell, centre, dims, facing: int, level_z: float) -> Placed:
    """Hücreyi koliye uygular. Geometri kontrolü burada yapılmaz, fits()'te."""
    c = np.array(centre, dtype=float)
    rot = np.eye(3)
    _, dy, dz = dims
    v = cell.signed
    if cell.arm == "push_in":
        c[1] -= facing * cell.value
    elif cell.arm == "pull_out":
        c[1] += facing * cell.value
    elif cell.arm == "slide":
        c[0] += v
    elif cell.arm == "yaw":
        rot = rot_z(math.radians(v))
    elif cell.arm == "tilt":
        # Koşu eksenine (X) göre devrilme: kolinin üstü koridora ya da rafın
        # arkasına yatar. Bir alt kenarı tablaya oturacak şekilde kaldırılır;
        # yoksa yarısı tablanın içine gömülürdü.
        a = math.radians(v)
        rot = rot_x(a)
        c[2] = level_z + (dz / 2) * math.cos(a) + (dy / 2) * abs(math.sin(a))
    return Placed(c, rot)


def corners(p: Placed, dims) -> np.ndarray:
    dx, dy, dz = dims
    s = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)])
    return p.centre + (s * np.array([dx, dy, dz]) / 2) @ p.rot.T


@dataclass
class Slot:
    """Kolinin yerleşebileceği kutu: komşular, dikmeler, raf arkası, koridor."""
    x_lo: float
    x_hi: float
    y_back: float            # rafın arka düzlemi
    y_face: float            # rafın koridora bakan düzlemi
    facing: int
    max_protrusion: float    # bu yüzde koridora en fazla bu kadar taşabilir


def protrusion(p: Placed, dims, slot: Slot) -> float:
    ys = corners(p, dims)[:, 1]
    return (ys.max() - slot.y_face) if slot.facing > 0 else (slot.y_face - ys.min())


def fits(p: Placed, dims, slot: Slot) -> bool:
    k = corners(p, dims)
    if k[:, 0].min() < slot.x_lo or k[:, 0].max() > slot.x_hi:
        return False
    if slot.facing > 0 and k[:, 1].min() < slot.y_back + MARGIN_M:
        return False
    if slot.facing < 0 and k[:, 1].max() > slot.y_back - MARGIN_M:
        return False
    return protrusion(p, dims, slot) <= slot.max_protrusion + 1e-9


# ------------------------------------------------------------- koridor payı

def face_caps(cfg: dict, layout_path: Path) -> dict[str, dict]:
    """Her yüz için kolinin koridora en fazla ne kadar taşabileceği.

    Araç bir yüzü taradığı sırada o yüzden `standoff` uzakta uçar ve gövdesi
    iki yana `vehicle_half_span` açılır. Aradaki boşluk aracın payıdır;
    dışarı kayan ya da devrilen bir koli bu payı yer. Pay
    `min_side_clearance`'ın altına inecekse o koli o bozulmayı ALAMAZ.

    Uçuş geometrisi scanner/layout.json'dan VERİ olarak okunur, değiştirilmez.
    Rota Furkan'ın; burada yalnız rotanın gerçekte nereden geçtiği soruluyor,
    ikinci bir kopyası tutulsaydı biri değişince bu kontrol sessizce yanlış
    yere bakardı.
    """
    st = cfg["stress"]
    layout = json.loads(Path(layout_path).read_text())
    half = float(layout["vehicle_half_span"])
    default = float(layout.get("shelf_standoff", 0.8))
    need = float(st.get("min_side_clearance", 0.05))
    out = {}
    for face in layout["aisle_faces"]:
        standoff = float(face.get("standoff", default))
        out[face["name"]] = {"standoff": standoff, "half_span": half,
                             "max_protrusion": standoff - half - need}
    return out


# --------------------------------------------------------------- atama

class Assigner:
    def __init__(self, cfg: dict):
        st = cfg["stress"]
        self.rng = random.Random(st["seed"])
        arms = st.get("arms") or {}
        unknown = set(arms) - set(ARMS)
        if unknown:
            raise SystemExit(f"stress.arms: bilinmeyen bozulma {sorted(unknown)}; "
                             f"geçerli olanlar {list(ARMS)}")
        self.cells = []
        for arm in ARMS:
            for li, value in enumerate(arms.get(arm) or []):
                if arm == "empty":
                    # empty'nin büyüklüğü yok; sayı kaç hücre olacağını söyler
                    self.cells += [Cell("empty", 1, 0.0)] * int(value)
                else:
                    self.cells.append(Cell(arm, li + 1, float(value)))
        n_ctrl = int(st.get("control_cells", 0))
        self.cells += [CONTROL] * n_ctrl
        if not any(c.arm != "none" for c in self.cells):
            raise SystemExit("stress açık ama stress.arms boş; hiçbir koli bozulmaz")
        self.pending: dict[str, list[Cell]] = {}
        self.given: dict[tuple, dict] = {}       # (arm, level) -> {yüz: sayı}

    def _cycle(self) -> list[Cell]:
        cyc = [Cell(c.arm, c.level, c.value,
                    self.rng.choice((-1, 1)) if c.arm in SIGNED else 1)
               for c in self.cells]
        self.rng.shuffle(cyc)
        return cyc

    def pick(self, face: str, ok) -> Cell:
        """Yüzün sırasından bu kolinin tam büyüklüğüyle alabildiği ilk hücre."""
        q = self.pending.setdefault(face, [])
        for _ in range(2):
            if not q:
                q.extend(self._cycle())
            for i, cell in enumerate(q):
                if ok(cell):
                    q.pop(i)
                    return self._count(face, cell)
            q.extend(self._cycle())
        return self._count(face, CONTROL)

    def _count(self, face: str, cell: Cell) -> Cell:
        per = self.given.setdefault((cell.arm, cell.level), {})
        per[face] = per.get(face, 0) + 1
        return cell

    def summary(self) -> list[str]:
        faces = sorted({f for per in self.given.values() for f in per})
        lines = [f"    {'bozulma':<16}" + "".join(f"{f:>4}" for f in faces) + "  top"]
        keys = sorted(self.given, key=lambda k: (
            (ARMS + ("none",)).index(k[0]), k[1]))
        for arm, level in keys:
            per = self.given[(arm, level)]
            name = arm if arm in ("none", "empty") else f"{arm} L{level}"
            lines.append(f"    {name:<16}" + "".join(f"{per.get(f, 0):>4}" for f in faces)
                         + f"  {sum(per.values()):>3}")
        # Bir yüzde hiç verilemeyen hücre: o yüz için o soru sorulmadı.
        for arm, level, value in {(c.arm, c.level, c.value) for c in self.cells
                                  if c.arm != "none"}:
            missing = [f for f in faces if not self.given.get((arm, level), {}).get(f)]
            if missing:
                lines.append(f"    UYARI: {arm} L{level} ({value:g} {UNITS[arm]}) "
                             f"{','.join(missing)} yüzünde hiçbir koliye sığmadı")
        return lines
