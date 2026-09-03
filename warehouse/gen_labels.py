#!/usr/bin/env python3
"""Depo simülasyonu için barkod / QR etiket dokuları üretir.

Kodlar piksel piksel elle çizilir (kütüphanelerin kendi render'ı yerine
matrisleri kullanılır). Sebebi: modül başına düşen piksel sayısı bu işin
tamamının belirleyici parametresi -- yeniden boyutlandırma veya kütüphaneye
özgü bir yuvarlama, kodu sessizce okunamaz hale getirebilir. Burada her
modülün kaç piksel olduğu tam olarak bilinir.

Doğrudan çalıştırıldığında okunabilirlik bütçesini raporlar ve örnek
etiketler üretir:

    .venv/bin/python tools/gen_labels.py --budget
    .venv/bin/python tools/gen_labels.py --samples out/
"""

from __future__ import annotations

import argparse
import math
import os
from dataclasses import dataclass
from pathlib import Path

import cv2
import qrcode
from barcode import Code128
from barcode.writer import BaseWriter
from PIL import Image, ImageDraw, ImageFont

PROJECT_ROOT = Path(__file__).resolve().parent.parent

_FONT_CANDIDATES = [
    "/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf",
    "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf",
    "/usr/share/fonts/truetype/liberation/LiberationMono-Bold.ttf",
]


def _font(size_px: int) -> ImageFont.FreeTypeFont:
    for path in _FONT_CANDIDATES:
        if os.path.exists(path):
            return ImageFont.truetype(path, size_px)
    return ImageFont.load_default()


def _centered_text(draw: ImageDraw.ImageDraw, box, text: str, fill=(0, 0, 0)) -> None:
    """`box` = (x0, y0, x1, y1) dikdörtgenine sığacak en büyük yazıyı ortalar."""
    x0, y0, x1, y1 = box
    max_w, max_h = x1 - x0, y1 - y0
    if max_w <= 2 or max_h <= 2:
        return
    size = max_h
    while size > 4:
        font = _font(size)
        left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
        if right - left <= max_w and bottom - top <= max_h:
            draw.text(
                (x0 + (max_w - (right - left)) / 2 - left,
                 y0 + (max_h - (bottom - top)) / 2 - top),
                text, font=font, fill=fill,
            )
            return
        size -= 1


# --------------------------------------------------------------------------
# okunabilirlik bütçesi
# --------------------------------------------------------------------------

def px_per_m(width_px: int, hfov_rad: float, distance_m: float) -> float:
    """Verilen mesafede görüntünün metre başına kaç piksel çözdüğü."""
    view_width_m = 2.0 * distance_m * math.tan(hfov_rad / 2.0)
    return width_px / view_width_m


def px_per_module(width_px: int, hfov_rad: float, distance_m: float,
                  module_size_m: float) -> float:
    return px_per_m(width_px, hfov_rad, distance_m) * module_size_m


#: Bir kod, modül başına bu kadar pikselin altında güvenilir çözülmez.
MIN_PX_PER_MODULE = 3.0
COMFORT_PX_PER_MODULE = 4.0


@dataclass
class BudgetRow:
    code: str
    camera: str
    distance_m: float
    module_mm: float
    px_per_module: float
    # Hangi koridorun okuma mesafesi olduğu; koridorlar eşit genişlikte
    # olmadığı için satırın hangi koridora ait olduğu tabloda görünmeli.
    aisle: str = ""

    @property
    def verdict(self) -> str:
        if self.px_per_module >= COMFORT_PX_PER_MODULE:
            return "RAHAT"
        if self.px_per_module >= MIN_PX_PER_MODULE:
            return "SINIRDA"
        return "OKUNMAZ"


# --------------------------------------------------------------------------
# QR
# --------------------------------------------------------------------------

def qr_matrix(payload: str, version: int, error_correction: str = "M"):
    """QR modül matrisini döndürür (quiet zone yok). Versiyon sabittir:
    payload sığmazsa hata verir -- sessizce büyümesi modül boyutunu
    küçültüp okunabilirliği bozardı."""
    ec = {
        "L": qrcode.constants.ERROR_CORRECT_L,
        "M": qrcode.constants.ERROR_CORRECT_M,
        "Q": qrcode.constants.ERROR_CORRECT_Q,
        "H": qrcode.constants.ERROR_CORRECT_H,
    }[error_correction]
    qr = qrcode.QRCode(version=version, error_correction=ec, border=0)
    qr.add_data(payload)
    qr.make(fit=False)          # fit=False -> sığmazsa DataOverflowError
    matrix = qr.get_matrix()
    expected = 17 + 4 * version
    assert len(matrix) == expected, f"beklenen {expected} modül, gelen {len(matrix)}"
    return matrix


def qr_module_count(version: int) -> int:
    return 17 + 4 * version


def draw_matrix(img: Image.Image, matrix, origin_px, module_px: int) -> None:
    draw = ImageDraw.Draw(img)
    ox, oy = origin_px
    for r, row in enumerate(matrix):
        for c, on in enumerate(row):
            if on:
                x = ox + c * module_px
                y = oy + r * module_px
                draw.rectangle([x, y, x + module_px - 1, y + module_px - 1], fill=(0, 0, 0))


# --------------------------------------------------------------------------
# ArUco (koridor başı markörleri)
# --------------------------------------------------------------------------
#
# Eski projede burası AprilTag/tag36h11'e sabitlenmişti. Bu projede markör
# ailesi config'ten geliyor (`codes.aisle_marker.dictionary`), çünkü Furkan'ın
# hattı DICT_5X5_100 kullanıyor ve markör haritası onun id'lerine bağlı.
# generateImageMarker çağrısı aynen korundu -- değişen tek şey sözlüğün
# nereden geldiği.

_DICT_CACHE: dict[str, "cv2.aruco.Dictionary"] = {}


def aruco_dictionary(name: str):
    """`"DICT_5X5_100"` gibi bir isimden OpenCV sözlüğü. Sözlük nesnesi
    pahalı değil ama her markör için yeniden kurmanın da anlamı yok."""
    if name not in _DICT_CACHE:
        if not hasattr(cv2.aruco, name):
            raise ValueError(f"bilinmeyen ArUco sözlüğü: {name}")
        _DICT_CACHE[name] = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
    return _DICT_CACHE[name]


def aruco_modules(name: str) -> int:
    """Markörün TOPLAM modül sayısı (kenar uzunluğu cinsinden).

    Veri bitleri + her kenarda 1 modüllük siyah çerçeve. DICT_5X5_100 için
    5 + 2 = 7. Bu ailenin tanımı, bizim tasarım kararımız değil; okunabilirlik
    bütçesi bu sayıya bölerek modül boyutunu buluyor."""
    return aruco_dictionary(name).markerSize + 2


def aruco_matrix(marker_id: int, dict_name: str):
    """ArUco deseni, modül ızgarası olarak (True = siyah).
    generateImageMarker'a modül sayısı kadar piksel istenince kütüphane
    tam 1 piksel/modül, antialiasing'siz saf 0/255 matris üretiyor --
    doğrulandı (7x7, sadece 0 ve 255), ekstra eşikleme gerekmiyor."""
    n = aruco_modules(dict_name)
    img = cv2.aruco.generateImageMarker(aruco_dictionary(dict_name), marker_id, n)
    return [[bool(v < 128) for v in row] for row in img]


# --------------------------------------------------------------------------
# Code128
# --------------------------------------------------------------------------

class _RawWriter(BaseWriter):
    """python-barcode'un çizim yapmayan writer'ı: sadece modül dizisini almak
    için. Kütüphanenin kendi ImageWriter'ı mm/dpi üzerinden hesaplayıp
    yuvarlıyor; burada modülleri doğrudan çiziyoruz."""

    def __init__(self):
        super().__init__(self._noop, self._noop, self._noop, self._noop)

    @staticmethod
    def _noop(*args, **kwargs):
        return None


def code128_modules(payload: str) -> str:
    """Code128 sembolünün '1'/'0' dizisi (quiet zone hariç)."""
    code = Code128(payload, writer=_RawWriter())
    return "".join(code.build())


# --------------------------------------------------------------------------
# yük (payload) biçimleri -- gen_world.py da bunları kullanır
# --------------------------------------------------------------------------

def placard_payload(index: int) -> str:
    """Kutu barkodu yükü: kutunun sıra numarası, 4 hane -- "0001".."0432".

    Kutuya özel, QR gibi, ama QR'ın taşıdığı hiçbir alanı tekrar etmiyor: QR
    kutunun adresini ve SKU'sunu söylüyor, barkod kutunun kendi numarasını.

    Neden 4 hane. Code128'in C modu rakam ÇİFTLERİNİ tek sembolde kodluyor,
    yani çift sayıda rakam tek sayıdan ucuz: 4 hane 57 modül, 3 hane 68, 5
    hane 79. 432 kutu 4 haneye rahat sığıyor.

    Bu, çubukların 190 mm'ye inmesini sağlayan şey. Sebep en dar koridor:
    orada kamera raftan 0.227 m uzakta ve gördüğü alanın tamamı 0.262 m.
    Eski 5 haneli yük 263 mm ediyordu -- karenin tam genişliği kadar, yani
    1D kod hiçbir zaman tamamen kadraja girmiyordu ve 2026-09-03 koşusunda G
    yüzünün 54 kutusundan 3'ü okundu. 57 modül 190 mm'ye sığdığında modül
    genişliği 3.33 mm'de kalıyor: okunabilirlik hiç değişmiyor, barkod %28
    daralıyor.

    SKU'nun rakamları kullanılamazdı: ne ilk ne son 4 hanesi 432 kutuda
    tekil (421 ve 428 farklı değer)."""
    return "%04d" % index


def placard_caption(index: int) -> str:
    return "%04d" % index


def box_payload(sku: str, row_id: str, bay: int, level: int) -> str:
    return f"WH1|{row_id}|{bay:02d}|{level}|{sku}"


def aruco_payload(marker_id: int, dict_name: str) -> str:
    """ArUco yalnızca bir id taşır; ground truth'ta okunabilir olsun diye
    sözlük adıyla birlikte yazılır."""
    return f"{dict_name}:{marker_id}"


PLACARD_SAMPLE = placard_payload(432)


# --------------------------------------------------------------------------
# etiket üreticileri
# --------------------------------------------------------------------------

def _canvas(label_wh_m, px_per_m_tex: float, max_px: int):
    """Etiket tuvalini oluşturur; uzun kenar `max_px`i aşarsa ölçek düşürülür."""
    w_m, h_m = label_wh_m
    scale = px_per_m_tex
    longest = max(w_m, h_m) * scale
    if longest > max_px:
        scale = max_px / max(w_m, h_m)
    w_px = max(8, int(round(w_m * scale)))
    h_px = max(8, int(round(h_m * scale)))
    return Image.new("RGB", (w_px, h_px), (255, 255, 255)), scale


def make_box_label(payload: str, caption: str, spec: dict,
                   px_per_m_tex: float, max_px: int) -> tuple[Image.Image, float]:
    """Kutu üstündeki QR etiketi. (görüntü, modül_boyutu_m) döndürür."""
    img, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    w_px, h_px = img.size

    n = qr_module_count(spec["qr_version"])
    # Modül pikselini tam sayıya yuvarla: kesirli modül genişliği komşu
    # modüllerin farklı boyutta çıkmasına, yani kodun bozulmasına yol açar.
    module_px = max(1, int(round(spec["code"] * scale / n)))
    qr_px = module_px * n
    module_m = spec["code"] / n

    matrix = qr_matrix(payload, spec["qr_version"], spec.get("qr_error_correction", "M"))
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    qr_area_h = h_px - caption_px
    draw_matrix(img, matrix,
                ((w_px - qr_px) // 2, (qr_area_h - qr_px) // 2), module_px)

    draw = ImageDraw.Draw(img)
    if caption_px > 4:
        pad = max(2, int(0.10 * caption_px))
        _centered_text(draw, (pad, qr_area_h, w_px - pad, h_px - pad), caption)
    # ince çerçeve: etiketi kutunun kartonundan ayırır
    draw.rectangle([0, 0, w_px - 1, h_px - 1], outline=(40, 40, 40), width=max(1, w_px // 220))
    return img, module_m


def make_bay_placard(payload: str, caption: str, spec: dict,
                     px_per_m_tex: float, max_px: int) -> tuple[Image.Image, float]:
    """Konum barkodu (Code128) -- kutunun ön yüzünde, QR etiketin altında."""
    img, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    w_px, h_px = img.size

    bits = code128_modules(payload)
    n = len(bits)
    module_px = max(1, int(round(spec["bar_width"] * scale / n)))
    bars_px = module_px * n
    module_m = spec["bar_width"] / n

    bar_h = int(round(spec["bar_height"] * scale))
    x0 = (w_px - bars_px) // 2
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    y0 = max(0, (h_px - caption_px - bar_h) // 2)

    draw = ImageDraw.Draw(img)
    for i, bit in enumerate(bits):
        if bit == "1":
            x = x0 + i * module_px
            draw.rectangle([x, y0, x + module_px - 1, y0 + bar_h - 1], fill=(0, 0, 0))

    if caption_px > 4:
        pad = max(2, int(0.12 * caption_px))
        _centered_text(draw, (x0, y0 + bar_h + pad, x0 + bars_px, h_px - pad), caption)

    # ÇERÇEVE YOK -- BİLEREK. QR etiketinde olduğu gibi ince bir kenarlık
    # çizmek Code128'i tamamen çözülemez hale getiriyordu: kenarlık, zbar
    # için sessiz alanın (quiet zone) içinde duran bir çubuk oluyor. Etikette
    # çubukların iki yanında (0.320-0.280)/2 = 20 mm boşluk var; bu zaten
    # standardın istediği 10 modülün (35.4 mm) altında, kenarlık da onu
    # 3 modüle indiriyordu. QR'da sorun çıkmıyor çünkü QR'ın sessiz alan
    # şartı 4 modül ve etikette 5 modül var.
    return img, module_m


# zbar'ın çözülmüş bir QR için döndürdüğü poligonun, dokuya çizilen GERÇEK
# modül sınırına oranı. 20 kutu etiketi üzerinde ölçüldü, hepsinde birebir
# 0.9880 (rasterleme deterministik). Konum kestirimi bu poligona dayandığı
# için etkin kenar hesabına giriyor; ihmal edilirse mesafe %1.2 hatalı çıkar.
ZBAR_QR_CORNER_RATIO = 0.9880


def box_label_geometry(spec: dict, px_per_m_tex: float,
                       max_px: int) -> tuple[float, float]:
    """Kutu QR'ının `(etkin_kenar_m, merkez_yükseklik_ofseti_m)`.

    Kenar: config'teki `code` değeri DEĞİL. Modül boyutu dokuya çizilirken tam
    sayı piksele yuvarlanıyor (örn. 9.6 -> 10 px), o yüzden gerçek kenar daha
    büyük; üstüne zbar'ın poligon oranı uygulanıyor.
    Ofset: QR etiketin ortasında değil, altındaki caption şeridinin üstünde
    kalan alana ortalanıyor -- yani QR merkezi ETİKET merkezinin biraz
    ÜSTÜNDE. Ground truth etiket merkezini verdiği için bu düzeltilmezse
    konum kestiriminde sabit bir +z sapması kalır.

    make_box_label ile aynı hesap. aisle_marker_geometry'nin ArUco için
    yaptığının kutu QR'ı karşılığı: geometriyi iki yerde ayrı tutmak sessiz
    bir ölçek/konum hatası kaynağı olur.
    """
    w_m, h_m = spec["label"]
    _, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    n = qr_module_count(spec["qr_version"])
    module_px = max(1, int(round(spec["code"] * scale / n)))
    qr_px = module_px * n

    h_px = max(8, int(round(h_m * scale)))
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    qr_area_h = h_px - caption_px
    top = (qr_area_h - qr_px) // 2
    # Dokuda y aşağı büyür, dünyada +Z yukarı: işaret ters.
    rise = ((h_px / 2.0) - (top + qr_px / 2.0)) / scale
    return (qr_px / scale) * ZBAR_QR_CORNER_RATIO, rise


def placard_geometry(spec: dict, px_per_m_tex: float,
                     max_px: int) -> tuple[float, float, float]:
    """Barkod ÇUBUKLARININ `(genişlik_m, yükseklik_m, merkez_ofseti_m)`.

    Çubuklar etiketin ortasında değil: altta caption şeridi var, çubuklar
    onun üstünde kalan alana ortalanıyor -- yani çubukların merkezi ETİKET
    merkezinin ÜSTÜNDE (make_bay_placard ile aynı hesap). Ölçüldü: bu 13 mm
    ihmal edilince barkod ROI'si etiketin altına kayıyor.

    Genişlik de `bar_width` değil: modül boyutu tam sayı piksele yuvarlanıyor
    (box_label_geometry'deki aynı yuvarlama).
    """
    _, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    h_px = max(8, int(round(spec["label"][1] * scale)))
    n = len(code128_modules("A0101"))          # yük uzunluğu sabit (sıra+göz+seviye)
    module_px = max(1, int(round(spec["bar_width"] * scale / n)))
    bars_px = module_px * n
    bar_h = int(round(spec["bar_height"] * scale))
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    y0 = max(0, (h_px - caption_px - bar_h) // 2)
    rise = ((h_px / 2.0) - (y0 + bar_h / 2.0)) / scale
    return bars_px / scale, bar_h / scale, rise


def aisle_marker_geometry(spec: dict, px_per_m_tex: float,
                          max_px: int) -> tuple[float, float, float]:
    """Koridor markörünün etiket içindeki GERÇEK yerleşimi.

    `(tag_kenar_m, dx_m, dy_m)` döndürür: tag'in dünyadaki kenar uzunluğu ve
    merkezinin ETİKET merkezine göre kayması. Etiket düzleminde u -> +X,
    v -> +Y (bkz. label_quad.obj).

    Neden ayrı bir fonksiyon: tag etiketin tam ortasında DEĞİL -- altta bir
    caption şeridi var, tag onun üstündeki alana ortalanıyor. Ayrıca modül
    boyutu tam sayı piksele yuvarlandığı için gerçek kenar `spec["code"]`
    ile birebir aynı değil. 3. aşamadaki lokalizasyon bu iki ayrıntıyı
    bilmek zorunda; make_aisle_marker ile aynı hesabı iki yerde tutmak
    sessiz bir konum hatası kaynağı olurdu.
    """
    n = aruco_modules(spec["dictionary"])
    _, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    h_px = max(8, int(round(spec["label"][1] * scale)))
    module_px = max(1, int(round(spec["code"] * scale / n)))
    tag_px = module_px * n
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    tag_area_h = h_px - caption_px

    # Dokuda y aşağı doğru büyür, dünyada +Y yukarı (v) doğru: işaret ters.
    top = (tag_area_h - tag_px) // 2
    dy_px = (h_px / 2.0) - (top + tag_px / 2.0)
    return tag_px / scale, 0.0, dy_px / scale


def make_aisle_marker(marker_id: int, caption: str, spec: dict,
                      px_per_m_tex: float, max_px: int) -> tuple[Image.Image, float]:
    """Koridor başı markörü: ArUco + kalın çerçeve (zeminden ayrışması için).
    `caption` sadece görsel hata ayıklama içindir, markörün kendisine
    kodlanmıyor -- ArUco yalnızca `marker_id`yi taşır."""
    img, scale = _canvas(spec["label"], px_per_m_tex, max_px)
    w_px, h_px = img.size

    n = aruco_modules(spec["dictionary"])
    module_px = max(1, int(round(spec["code"] * scale / n)))
    tag_px = module_px * n
    module_m = spec["code"] / n

    draw = ImageDraw.Draw(img)
    border = max(2, int(0.02 * min(w_px, h_px)))
    draw.rectangle([0, 0, w_px - 1, h_px - 1], outline=(0, 0, 0), width=border)

    matrix = aruco_matrix(marker_id, spec["dictionary"])
    caption_px = int(round(spec.get("caption_height", 0.0) * scale))
    tag_area_h = h_px - caption_px
    draw_matrix(img, matrix, ((w_px - tag_px) // 2, (tag_area_h - tag_px) // 2), module_px)

    if caption_px > 4:
        pad = max(2, int(0.12 * caption_px))
        _centered_text(draw, (border + pad, tag_area_h, w_px - border - pad, h_px - border - pad),
                       caption)
    return img, module_m


# --------------------------------------------------------------------------
# CLI
# --------------------------------------------------------------------------

def aisle_half_widths(cfg: dict) -> dict[int, float]:
    """Her koridorun merkezinden raf yüzüne mesafesi -- okunabilirlik
    bütçesinin en önemli girdisi.

    Eskiden 1.5 m sabitiydi, sonra tek bir türetilmiş sayı oldu. Artık
    KORİDOR BAŞINA: bu depoda koridorlar 2.40 m'den 0.50 m'ye daralıyor, yani
    okuma mesafesi de koridordan koridora değişiyor ve tek bir satır tabloyu
    üç koridor için yanlış yapardı.
    """
    rk = cfg["racking"]
    depth = rk["depth"]
    faces = [row["y0"] + depth if row["facing"] > 0 else row["y0"]
             for row in rk["rows"]]
    out = {}
    for aisle in rk["aisles"]:
        yc = aisle["y_center"]
        near = min((abs(f - yc) for f in faces), default=None)
        if near is not None:
            out[aisle["id"]] = near
    if not out:
        raise ValueError("racking.aisles / racking.rows boş -- koridor genişliği türetilemedi")
    return out


def aisle_half_width(cfg: dict) -> float:
    """En kötü durum okuma mesafesi: en GENİŞ koridorun yarısı. Kamera orada
    raf yüzüne en uzakta durur, yani modül başına px orada en düşüktür."""
    return max(aisle_half_widths(cfg).values())


def compute_budget(cfg: dict) -> list[BudgetRow]:
    """Okunabilirlik bütçesi.

    Bu projede aşağı bakan kamera YOK (Furkan'ın x500_scanner'ında sadece iki
    yan kamera var), o yüzden koridor markörü için bir satır üretilmiyor --
    markörler şu hâliyle dekoratif. Aşağı bakan kamera eklenirse buraya
    ArUco satırı da girmeli.
    """
    codes = cfg["codes"]
    cams = {c["name"]: c for c in cfg["cameras"]}
    halves = aisle_half_widths(cfg)

    # Yan kameralar birbirinin aynası; bütçe için ilki temsil eder.
    scan_name = next((n for n in ("left", "right") if n in cams), None)
    if scan_name is None:
        raise ValueError("config'te 'left'/'right' tarama kamerası yok")
    cam = cams[scan_name]

    # Her koridor için orta çizgi ve karşı yüz mesafesi. Koridorlar artık eşit
    # değil, o yüzden tek bir mesafe listesi yerine koridor başına iki satır:
    # drone orta çizgide durursa yakın yüz, karşıya bakarsa uzak yüz.
    measurements: list[tuple[str, float]] = []
    for aid in sorted(halves):
        half = halves[aid]
        measurements.append((f"k{aid} orta", round(half, 2)))
        measurements.append((f"k{aid} karşı", round(2 * half, 2)))

    rows: list[BudgetRow] = []
    box_module = codes["box_label"]["code"] / qr_module_count(codes["box_label"]["qr_version"])
    placard_module = codes["box_placard"]["bar_width"] / len(code128_modules(PLACARD_SAMPLE))
    for name, module in (("kutu QR", box_module),
                         ("kutu barkodu", placard_module)):
        for label, d in measurements:
            rows.append(BudgetRow(name, scan_name, d, module * 1000,
                                  px_per_module(cam["width"], cam["hfov"], d, module),
                                  label))
    return rows


def _load_cfg(path: Path) -> dict:
    import yaml
    with open(path) as fh:
        return yaml.safe_load(fh)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", type=Path,
                    default=PROJECT_ROOT / "config" / "warehouse.yaml")
    ap.add_argument("--budget", action="store_true",
                    help="okunabilirlik bütçesini tablo olarak yazdır")
    ap.add_argument("--samples", type=Path, metavar="DIZIN",
                    help="her kod tipinden bir örnek etiket üret")
    args = ap.parse_args()

    cfg = _load_cfg(args.config)

    if args.budget or not args.samples:
        rows = compute_budget(cfg)
        print(f"{'kod':<16}{'kamera':<8}{'koridor':<10}{'mesafe':>8}"
              f"{'modül':>10}{'px/modül':>11}  sonuç")
        print("-" * 72)
        last = None
        for r in rows:
            if last is not None and r.code != last:
                print()
            last = r.code
            print(f"{r.code:<16}{r.camera:<8}{r.aisle:<10}{r.distance_m:>7.2f}m"
                  f"{r.module_mm:>9.2f}mm{r.px_per_module:>11.2f}  {r.verdict}")
        halves = aisle_half_widths(cfg)
        print(f"\neşik: >={MIN_PX_PER_MODULE} px/modül okunur, "
              f">={COMFORT_PX_PER_MODULE} rahat")
        print("koridor merkezinden raf yüzüne (yerleşimden türetildi):")
        for aid in sorted(halves):
            print(f"  koridor {aid}: {halves[aid]:.3f} m "
                  f"(net koridor {2*halves[aid]:.2f} m)")

    if args.samples:
        out = args.samples
        out.mkdir(parents=True, exist_ok=True)
        codes = cfg["codes"]
        ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
        img, m = make_box_label(box_payload("SKU48213", "A", 3, 2), "SKU48213",
                                codes["box_label"], ppm, maxpx)
        img.save(out / "ornek_kutu_etiketi.png")
        print(f"kutu etiketi     {img.size[0]}x{img.size[1]} px, modül {m*1000:.2f} mm")
        img, m = make_bay_placard(PLACARD_SAMPLE, placard_caption(432),
                                  codes["box_placard"], ppm, maxpx)
        img.save(out / "ornek_kutu_barkodu.png")
        print(f"kutu barkodu     {img.size[0]}x{img.size[1]} px, modül {m*1000:.2f} mm")
        am = codes["aisle_marker"]
        img, m = make_aisle_marker(1, "", am, ppm, maxpx)
        img.save(out / "ornek_koridor_aruco.png")
        print(f"koridor ArUco    {img.size[0]}x{img.size[1]} px, modül {m*1000:.2f} mm "
              f"({am['dictionary']}, {aruco_modules(am['dictionary'])}x"
              f"{aruco_modules(am['dictionary'])} modül)")
        print(f"\n-> {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
