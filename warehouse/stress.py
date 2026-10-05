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

#: Kolinin kendisine yapılanlar.
GEOMETRY_ARMS = ("push_in", "pull_out", "slide", "yaw", "tilt", "empty")
#: Etiketlere yapılanlar; koli yerinde ve dimdik durur.
LABEL_ARMS = ("fade", "smudge", "tear", "wrinkle", "skew", "decoy")
#: Sırası raporun da sırası. YENİ BOZULMALAR SONA EKLENİR: hücre döngüsü bu
#: sırayla kurulduğu için araya giren bir ad, aynı `use` listesiyle üretilmiş
#: eski bir dünyayı bile değiştirirdi.
ARMS = GEOMETRY_ARMS + LABEL_ARMS

#: Birimi: raporda ve yer gerçeğinde değerin yanında yazılır.
UNITS = {"push_in": "m", "pull_out": "m", "slide": "m",
         "yaw": "deg", "tilt": "deg", "empty": "", "none": "",
         "fade": "contrast", "smudge": "cover", "tear": "cover",
         "wrinkle": "module", "skew": "deg", "decoy": "set"}

#: İşaretli bozulmalar: sola/sağa, öne/arkaya rastgele.
SIGNED = {"slide", "yaw", "tilt", "skew"}

#: Etiket dokusunu değiştirenler (skew ve decoy dokuya değil yerleşime dokunur).
TEXTURE_ARMS = {"fade", "smudge", "tear", "wrinkle"}

#: Eski etiketin, kolinin asıl etiketlerinin altında bıraktığı boşluk.
DECOY_GAP_M = 0.010

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
        # Bu uçuşta hangileri açık. Yazılmazsa değeri olan her bozulma.
        use = st.get("use")
        use = set(arms) if use is None else set(use)
        if use - set(arms):
            raise SystemExit(f"stress.use: {sorted(use - set(arms))} için "
                             "stress.arms'ta değer yok")
        self.cells = []
        for arm in (a for a in ARMS if a in use):
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
        # Etiket kusurlarının rastgeleliği (lekenin yeri, yırtığın köşesi,
        # eski etiketin yükü) ayrı bir üreteçten: atamanın sırasını kaydırmasın.
        self.tex_rng = np.random.default_rng(int(st["seed"]) + 1)
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


# --------------------------------------------------------- etiket kusurları
#
# Hepsi PIL görüntüsü alır, yenisini döndürür; girdiye dokunmaz. Rastgelelik
# yalnız verilen numpy üretecinden gelir, o da Assigner'ın tohumundan: aynı
# tohum aynı lekeyi aynı yere koyar.
#
# Büyüklükler, kusurun koda ne yaptığını söyleyen birimlerle verilir, piksel
# ya da "biraz" değil: solma kalan KONTRAST, leke ve yırtık etiketin örtülen
# PAYI, kırışıklık kodun MODÜLÜ cinsinden genlik. Bir eşik bulunduğunda
# gerçek dünyadaki karşılığı da böylece okunur.

def fade(img, contrast: float):
    """Soluk baskı: siyah ile kağıt arasındaki farkın yalnız `contrast` kadarı kalır.

    Termal etiketler ısıda ve ışıkta tam böyle solar: kağıt beyaz kalır,
    siyah griye döner.
    """
    a = np.asarray(img, dtype=np.float32)
    return _pil(255.0 - (255.0 - a) * contrast)


def _pil(a):
    from PIL import Image
    return Image.fromarray(np.clip(a, 0, 255).astype(np.uint8), "RGB")


def _blobs(shape, cover: float, rng, soft: float):
    """Toplam alanı yaklaşık `cover` olan yumuşak kenarlı lekelerin maskesi."""
    from PIL import Image, ImageDraw, ImageFilter
    h, w = shape
    mask = Image.new("L", (w, h), 0)
    d = ImageDraw.Draw(mask)
    target = cover * w * h
    covered = 0.0
    for _ in range(400):
        if covered >= target:
            break
        rx = rng.uniform(0.05, 0.22) * w
        ry = rng.uniform(0.05, 0.22) * h if h > w / 3 else rng.uniform(0.25, 0.6) * h
        cx, cy = rng.uniform(0, w), rng.uniform(0, h)
        d.ellipse([cx - rx, cy - ry, cx + rx, cy + ry], fill=255)
        covered = float((np.asarray(mask) > 127).sum())
    mask = mask.filter(ImageFilter.GaussianBlur(soft))
    return np.asarray(mask, dtype=np.float32)[..., None] / 255.0


#: Lekenin örtücülüğü. Dokular üzerinde zbar ile ölçüldü (40 QR, kameranın A
#: yüzündeki 3.1 px/modül'e küçültülmüş): 0.65'te etiketin %60'ı kaplansa
#: bile okunuyordu, yarı saydam leke kontrastı düşürür ama modülü gizlemez;
#: 0.9'da %5'lik leke bile QR'ı öldürüyordu, çünkü koyu leke siyah modül
#: gibi okunur. 0.75'te düşüş kademeli: %5'te 33/40, %20'de 30, %45'te 18.
SMUDGE_OPACITY = 0.75


def smudge(img, cover: float, rng, module_px: float):
    """Kir, parmak izi, toz: etiketin `cover` kadarının üstünde koyu yarı saydam leke."""
    a = np.asarray(img, dtype=np.float32)
    m = _blobs(a.shape[:2], cover, rng, soft=max(1.0, module_px * 0.6))
    dirt = np.array([70.0, 62.0, 52.0])
    return _pil(a * (1 - SMUDGE_OPACITY * m) + dirt * SMUDGE_OPACITY * m)


def tear(img, cover: float, rng, cardboard_rgb):
    """Bir köşesi yırtılmış etiket: `cover` kadar alan gitmiş, altından koli görünür.

    Saydam doku yerine kolinin rengi boyanır: etiket düz bir dörtgen, delik
    açılamaz. Kenar tırtıklı, çünkü kağıt düz yırtılmaz.
    """
    from PIL import Image, ImageDraw
    a = img.copy()
    w, h = a.size
    # Üçgenin dik kenarları, alanı cover*w*h olacak şekilde. Uzun ince barkod
    # etiketinde dikey kenar etiketin boyuna yakın tutulur, yoksa yatay kenar
    # etiketten taşar; kare QR etiketinde iki kenar birbirine yakın.
    if w > 3 * h:
        leg_y = rng.uniform(0.75, 1.0) * h
        leg_x = 2 * cover * w * h / leg_y
    else:
        r = rng.uniform(0.6, 1.6)
        leg_x = math.sqrt(2 * cover * w * h * r)
        leg_y = math.sqrt(2 * cover * w * h / r)
    leg_x, leg_y = min(w, leg_x), min(h, leg_y)
    corner = rng.integers(4)
    pts = [(0.0, 0.0)]
    n = 9
    for i in range(n + 1):
        t = i / n
        jag = rng.normal(0, 0.04)
        pts.append((leg_x * (1 - t) * (1 + jag), leg_y * t * (1 - jag)))
    flip = [(x if corner in (0, 2) else w - x, y if corner in (0, 1) else h - y)
            for x, y in pts]
    ImageDraw.Draw(a).polygon(flip, fill=tuple(int(c * 255) for c in cardboard_rgb))
    return a


def wrinkle(img, amp_modules: float, rng, module_px: float):
    """Buruşuk etiket: yüzey dalgalanır, çizgiler kayar, kırışıkta gölge kalır.

    Etiket düz bir dörtgen olduğu için dalga dokunun içinde yapılır: her
    piksel, birkaç sinüsün toplamı kadar yer değiştirir. Genlik modül
    cinsinden, çünkü kodu bozan şey modül sınırının ne kadar kaydığıdır.
    """
    import cv2
    a = np.asarray(img, dtype=np.float32)
    h, w = a.shape[:2]
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    amp = amp_modules * module_px
    dx = np.zeros_like(xx)
    dy = np.zeros_like(yy)
    shade = np.zeros_like(xx)
    for _ in range(3):
        th = rng.uniform(0, math.pi)
        lam = rng.uniform(5, 12) * module_px
        ph = rng.uniform(0, 2 * math.pi)
        phase = (xx * math.cos(th) + yy * math.sin(th)) * 2 * math.pi / lam + ph
        dx += amp / 3 * np.sin(phase) * -math.sin(th)
        dy += amp / 3 * np.sin(phase) * math.cos(th)
        shade += np.cos(phase)
    out = cv2.remap(a, xx + dx, yy + dy, cv2.INTER_LINEAR,
                    borderMode=cv2.BORDER_CONSTANT, borderValue=(255, 255, 255))
    # Kırışıkların ışığı: genlikle birlikte koyulaşır, en ağırda %18.
    out *= (1 - min(0.18, 0.06 * amp_modules) * (0.5 + 0.5 * shade / 3))[..., None]
    return _pil(out)


def damage(img, cell: Cell, rng, module_px: float, cardboard_rgb):
    """Dokuya yapılan kusur; dokuya dokunmayan bir hücrede görüntü aynen döner."""
    if cell.arm == "fade":
        return fade(img, cell.value)
    if cell.arm == "smudge":
        return smudge(img, cell.value, rng, module_px)
    if cell.arm == "tear":
        return tear(img, cell.value, rng, cardboard_rgb)
    if cell.arm == "wrinkle":
        return wrinkle(img, cell.value, rng, module_px)
    return img


# ---------------------------------------------------- etiket yerleşimi

def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rects_inside(rects, x_lo, x_hi, z_lo, z_hi) -> bool:
    """Dörtgenlerin (köşe listeleri, x-z düzleminde) hepsi bu aralıkta mı."""
    for k in rects:
        xs, zs = [p[0] for p in k], [p[1] for p in k]
        if min(xs) < x_lo or max(xs) > x_hi or min(zs) < z_lo or max(zs) > z_hi:
            return False
    return True


def rect(cx, cz, w, h, centre=None, angle=0.0):
    """Yüz düzleminde bir etiketin köşeleri, istenirse `centre` etrafında dönmüş."""
    pts = [(cx - w / 2, cz - h / 2), (cx + w / 2, cz - h / 2),
           (cx + w / 2, cz + h / 2), (cx - w / 2, cz + h / 2)]
    if not angle:
        return pts
    ox, oz = centre
    c, s = math.cos(angle), math.sin(angle)
    return [(ox + (x - ox) * c - (z - oz) * s, oz + (x - ox) * s + (z - oz) * c)
            for x, z in pts]
