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
#: Kolinin kendi biçimi ve görünüşü: şişmiş, ezik, başka renk.
SHAPE_ARMS = ("bulge", "dent", "colour")
#: Sırası raporun da sırası. YENİ BOZULMALAR SONA EKLENİR: hücre döngüsü bu
#: sırayla kurulduğu için araya giren bir ad, aynı `use` listesiyle üretilmiş
#: eski bir dünyayı bile değiştirirdi.
ARMS = GEOMETRY_ARMS + LABEL_ARMS + SHAPE_ARMS

#: Birimi: raporda ve yer gerçeğinde değerin yanında yazılır.
UNITS = {"push_in": "m", "pull_out": "m", "slide": "m",
         "yaw": "deg", "tilt": "deg", "empty": "", "none": "",
         "fade": "contrast", "smudge": "cover", "tear": "cover",
         "wrinkle": "module", "skew": "deg", "decoy": "set",
         "bulge": "m", "dent": "m", "colour": "kind"}

#: colour hücresinin değeri bir tür, büyüklük değil.
COLOURS = {1: "white", 2: "printed", 3: "dark"}
#: Beyaz ve koyu kolinin rengi. Beyaz: çift oluklu beyaz mukavva; koyu:
#: siyah baskılı ya da ıslanmış karton. İkisi de etiketle kontrastı değiştirir:
#: beyazda etiketin kenarı kaybolur, koyuda koli çerçevenin içinde kararır.
COLOUR_RGB = {"white": (0.90, 0.89, 0.86), "dark": (0.13, 0.12, 0.12)}

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


# ------------------------------------------------------- koli biçimi ve rengi
#
# Şişme ve ezik, kolinin ÖN YÜZÜNÜ eğer; koli kendi modelini alır (kutu
# primitifi eğilemez). Yüz, bir yüzey fonksiyonuyla verilir: f(x, z) ön
# yüzün o noktada koridora doğru ne kadar çıktığı (eksi: içeri göçtüğü),
# kolinin merkezine göre x (koşu boyunca) ve z (yukarı). Kenarlarda f = 0,
# böylece ön yüz yan yüzlere boşluksuz bağlanır. Etiketler aynı f'yi izler:
# düz bir etiket bombeli bir yüze yapıştırılınca onunla birlikte kıvrılır.

def surface(cell: Cell, dims, rng):
    """(f, ayrıntı) - f(x, z) dizi alır dizi döndürür; ayrıntı yer gerçeği için."""
    dx, _, dz = dims
    v = cell.value
    if cell.arm == "bulge":
        # İçi dolu bir kolinin ön yüzü ortasından şişer: kenarda sıfır, ortada v.
        def f(x, z):
            return v * (1 - (2 * x / dx) ** 2) * (1 - (2 * z / dz) ** 2)
        return f, {}
    if cell.arm == "dent":
        # Bir yere çarpılmış: yuvarlak bir çukur, rastgele bir yerde. Kenara
        # doğru sönen bir pencereyle çarpılır ki yüz kenarda düz kalsın.
        x0 = rng.uniform(-0.3, 0.3) * dx
        z0 = rng.uniform(-0.3, 0.3) * dz
        r = rng.uniform(0.25, 0.40) * min(dx, dz)

        def f(x, z):
            win = (1 - (2 * x / dx) ** 8) * (1 - (2 * z / dz) ** 8)
            return -v * np.exp(-((x - x0) ** 2 + (z - z0) ** 2) / (2 * (r / 2) ** 2)) * win
        return f, {"dent_at": [round(float(x0), 4), round(float(z0), 4)],
                   "dent_radius_m": round(float(r), 4)}
    return (lambda x, z: np.zeros_like(np.asarray(x, dtype=float))), {}


def _obj(verts, uvs, tris, header: str) -> str:
    """Düzgün gölgeli OBJ: köşe normalleri üçgen normallerinin ortalaması."""
    v = np.asarray(verts, dtype=float)
    n = np.zeros_like(v)
    for a, b, c in tris:
        n[[a, b, c]] += np.cross(v[b] - v[a], v[c] - v[a])
    n /= np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)
    lines = [f"# {header}"]
    lines += [f"v {x:.5f} {y:.5f} {z:.5f}" for x, y, z in v]
    lines += [f"vt {u:.5f} {w:.5f}" for u, w in uvs]
    lines += [f"vn {x:.5f} {y:.5f} {z:.5f}" for x, y, z in n]
    lines += [f"f {a+1}/{a+1}/{a+1} {b+1}/{b+1}/{b+1} {c+1}/{c+1}/{c+1}" for a, b, c in tris]
    return "\n".join(lines) + "\n"


def _grid(nu, nv, point, uv, verts, uvs, tris, outward):
    """Bir yüzü nu x nv kareye böler; üçgenler dışa bakacak sırayla eklenir.

    Sıra önemli: Gazebo arkası dönük üçgeni çizmez, ters sarılmış bir yüz
    kameradan görünmez olurdu. Her üçgenin normali `outward` ile
    karşılaştırılıp gerekirse çevrilir, sıra varsayılmaz.
    """
    base = len(verts)
    for j in range(nv + 1):
        for i in range(nu + 1):
            a, b = i / nu, j / nv
            verts.append(point(a, b))
            uvs.append(uv(a, b))
    for j in range(nv):
        for i in range(nu):
            q = [base + j * (nu + 1) + i, base + j * (nu + 1) + i + 1,
                 base + (j + 1) * (nu + 1) + i + 1, base + (j + 1) * (nu + 1) + i]
            for t in ((q[0], q[1], q[2]), (q[0], q[2], q[3])):
                pa, pb, pc = (np.asarray(verts[k]) for k in t)
                if np.dot(np.cross(pb - pa, pc - pa), outward(pa, pb, pc)) < 0:
                    t = (t[0], t[2], t[1])
                tris.append(t)


#: Ön yüzün ızgarası. 24 x 18: XS kolide 13 x 16 mm'lik kareler, en büyük
#: şişmede bile yüzey eğrisinden sapma bir milimetrenin altında.
FRONT_GRID = (24, 18)
#: Baskılı dokuda ön yüze ayrılan pay; geri kalanı öbür yüzlerin düz rengi.
PRINT_U = 0.75


def carton_obj(dims, facing: int, f, printed: bool = False) -> str:
    """Kolinin modeli, kendi merkezinde: ön yüzü f ile eğilmiş, öbürleri düz."""
    dx, dy, dz = dims
    hx, hy, hz = dx / 2, dy / 2, dz / 2
    verts, uvs, tris = [], [], []
    flat = (lambda a, b: (0.5 * (1 + PRINT_U), 0.5))
    centre = np.zeros(3)

    def away(pa, pb, pc):
        return (pa + pb + pc) / 3 - centre

    # Ön yüz: koridora bakan, y = facing * hy, f kadar dışarı. Doku, koridordan
    # bakana göre düz okunsun diye u, bakanın sağına doğru büyür.
    def front(a, b):
        x, z = -hx + a * dx, -hz + b * dz
        return (x, facing * (hy + float(f(x, z))), z)

    def front_uv(a, b):
        u = (1 - a) if facing > 0 else a
        return (u * PRINT_U, b) if printed else flat(a, b)

    _grid(*FRONT_GRID, front, front_uv, verts, uvs, tris, away)
    # Arka ve yanlar düz: kamera onları ancak açıdan görür.
    _grid(1, 1, lambda a, b: (-hx + a * dx, -facing * hy, -hz + b * dz), flat,
          verts, uvs, tris, away)
    for sx_ in (-1, 1):
        _grid(1, 1, lambda a, b, s=sx_: (s * hx, -hy + a * dy, -hz + b * dz), flat,
              verts, uvs, tris, away)
    for sz in (-1, 1):
        _grid(1, 1, lambda a, b, s=sz: (-hx + a * dx, -hy + b * dy, s * hz), flat,
              verts, uvs, tris, away)
    return _obj(verts, uvs, tris, f"koli {dx}x{dy}x{dz}, ön yüz {'+y' if facing > 0 else '-y'}")


#: Eğri etiketin ızgarası.
LABEL_GRID = (16, 12)


def label_obj(w: float, h: float, lift) -> str:
    """Kıvrılmış etiket: etiket_quad.obj ile aynı çerçeve ve UV, gerçek boyda.

    lift(u, v): etiketin kendi düzleminde (u sağa, v yukarı, metre) yüzeyin
    etiket merkezine göre ne kadar dışarıda olduğu. Ölçek 1 ile kullanılır;
    ölçeklenseydi kıvrım da ölçeklenirdi.
    """
    verts, uvs, tris = [], [], []

    def pt(a, b):
        u, v = (a - 0.5) * w, (b - 0.5) * h
        return (u, v, float(lift(u, v)))

    _grid(*LABEL_GRID, pt, lambda a, b: (a, b), verts, uvs, tris,
          lambda pa, pb, pc: np.array([0.0, 0.0, 1.0]))
    return _obj(verts, uvs, tris, f"kıvrık etiket {w}x{h}")


#: Baskılı kolilerin marka adları. Uydurma: gerçek bir markaya benzemesin.
BRANDS = ("NORDVIK", "ALTAY", "KORUNA", "MERIDA", "TUNDRA", "SELVA", "OKAPI", "VESTA")


def print_texture(rng, dims):
    """Baskılı kolinin dokusu: renkli zemin, marka yazısı, şeritler, ürün barkodu.

    Ürün barkodu bilerek ÇÖZÜLEMEZ: çubuklar rastgele, başı ve sonu yok. Gerçek
    kolilerde ürün barkodu vardır ve YOLO'nun barkod sınıfı onu görecektir;
    sorulan şey o kutunun dedektörü ve sayımı şaşırtıp şaşırtmadığı, zbar'ın
    başka bir numara okuması değil (o, decoy'un sorusu).
    """
    from PIL import Image, ImageDraw
    import gen_labels as gl
    dx, _, dz = dims
    W = 768
    w_front = int(W * PRINT_U)
    H = max(64, int(w_front * dz / dx))
    base = tuple(int(c) for c in rng.integers(40, 230, 3))
    ink = tuple(255 - c for c in base)
    img = Image.new("RGB", (W, H), base)
    d = ImageDraw.Draw(img)
    # Öbür yüzlerin düz rengi: dokunun sağ şeridi.
    d.rectangle([w_front, 0, W, H], fill=base)
    # Şeritler.
    for k in range(int(rng.integers(2, 5))):
        y = int(rng.uniform(0, H))
        d.rectangle([0, y, w_front, y + int(H * rng.uniform(0.03, 0.08))], fill=ink)
    # Marka adı, üst yarıda büyük.
    name = BRANDS[int(rng.integers(len(BRANDS)))]
    font = gl._font(int(H * 0.22))
    d.text((int(w_front * 0.06), int(H * 0.06)), name, fill=ink, font=font)
    # Ürün barkodu, bir alt köşede.
    bx = int(w_front * rng.choice([0.06, 0.70]))
    by, bw, bh = int(H * 0.68), int(w_front * 0.22), int(H * 0.22)
    d.rectangle([bx - 6, by - 6, bx + bw + 6, by + bh + 6], fill=(250, 250, 250))
    x = bx
    while x < bx + bw:
        t = int(rng.integers(1, 5))
        if rng.random() < 0.55:
            d.rectangle([x, by, min(bx + bw, x + t), by + bh], fill=(10, 10, 10))
        x += t + int(rng.integers(1, 4))
    return img, {"brand": name, "base_rgb": list(base)}


# ------------------------------------------------------------------- ışık
#
# Bozulma kolilere değil bütün depoya: genel ışık kısılır, tavan lambalarının
# bir kısmı söner. Tek bir uçuş yine de bir eğri verir, çünkü her etiketin
# aldığı ışık farklıdır - sönen lambanın altındakiler karanlıkta kalır. Bu
# yüzden her etiketin ışığı HESAPLANIP yer gerçeğine yazılır; rapor sonucu
# ışığa göre dilimler.
#
# Model, render'ın kendi varsayımlarıyla: gölge kapalı (lambalar rafların
# içinden geçer, gen_world.lighting'deki cast_shadows false), nokta lamba
# Ogre'nin sönümüyle 1 / (c + l*d + q*d^2) ve menzil dışında sıfır, yüzeyin
# normaline göre kosinüs; üstüne sahnenin ortam ışığı. Mutlak bir lüks değil,
# aynı etiketin temiz dünyadaki ışığına ORAN: model sabitleri yanlış olsa
# bile bu oran, ışığın ne kadar kısıldığını sıralar.

LAMP_ATTENUATION = (0.3, 0.05, 0.005)     # gen_world.lighting ile aynı


def lamps(cfg: dict) -> list[tuple[int, tuple]]:
    """Tavan lambaları, gen_world.lighting'in sırasıyla: (sıra, (x, y, z)), config çerçevesi."""
    lt = cfg["lighting"]["ceiling_lights"]
    out, i = [], 0
    for xc in lt["x_positions"]:
        for yc in lt["y_positions"]:
            out.append((i, (xc, yc, lt["height"])))
            i += 1
    return out


def light_plan(cfg: dict) -> dict:
    """Bu dünyanın ışığı: ortam, lamba rengi, sönen lambalar.

    `lights_stress` kapalıysa temiz dünyanınki. Sönecek lambalar, sayı
    verilirse tohumla seçilir; `dead` listesi verilirse o.
    """
    lg = cfg["lighting"]
    st = cfg.get("lights_stress") or {}
    plan = {"ambient": list(lg["ambient"]), "diffuse": list(lg["ceiling_lights"]["diffuse"]),
            "dead": set(), "on": False}
    if not st.get("enabled"):
        return plan
    a, d = float(st.get("ambient_scale", 1.0)), float(st.get("diffuse_scale", 1.0))
    plan["ambient"] = [c * a for c in plan["ambient"][:3]] + [plan["ambient"][3]]
    plan["diffuse"] = [c * d for c in plan["diffuse"][:3]] + [plan["diffuse"][3]]
    n = len(lamps(cfg))
    if st.get("dead") is not None and len(st["dead"]):
        plan["dead"] = {int(i) for i in st["dead"]}
        if any(i < 0 or i >= n for i in plan["dead"]):
            raise SystemExit(f"lights_stress.dead: lamba sırası 0..{n - 1} olmalı")
    else:
        k = int(round(float(st.get("dead_fraction", 0.0)) * n))
        plan["dead"] = set(random.Random(st.get("seed", 0)).sample(range(n), k))
    plan["on"] = True
    return plan


def illuminance(p, normal, plan: dict, cfg: dict) -> float:
    """Bir yüzey noktasının aldığı ışık, modelin birimiyle (bkz. yukarı)."""
    rng_max = cfg["lighting"]["ceiling_lights"]["attenuation_range"]
    c, l, q = LAMP_ATTENUATION
    n = np.asarray(normal, dtype=float)
    p = np.asarray(p, dtype=float)
    lum = float(np.mean(plan["diffuse"][:3]))
    e = float(np.mean(plan["ambient"][:3]))
    for i, pos in lamps(cfg):
        if i in plan["dead"]:
            continue
        v = np.asarray(pos, dtype=float) - p
        d = float(np.linalg.norm(v))
        if d >= rng_max or d <= 1e-9:
            continue
        e += lum * max(0.0, float(n @ v) / d) / (c + l * d + q * d * d)
    return e
