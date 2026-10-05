#!/usr/bin/env python3
"""
Check that a barcode stuck on crooked is read, and filed where it is.

    .venv/bin/python perception/test_level.py

No simulator. The world's own QR label and barcode label are drawn as one
block, as gen_world sticks them on, turned through a range of angles and
pasted into a grey frame. The decoder has to read the barcode at every angle,
and the polygon it hands back has to sit on the bars in the FRAME's pixels -
that is what the linker and the placement read.

It also checks the reason: with the levelling pass switched off, the turned
barcodes must NOT read. Otherwise this would pass on a decoder that never
needed the fix, and say nothing.

A synthetic frame is not the camera's; the 2026-10-05 label flight's saved
frames are the measurement (25 of the 32 barcodes turned 12 and 25 degrees
recovered, every one linked to its own QR). This guards the geometry.
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE.parent / "warehouse"))

import barcode_scanner as bs                              # noqa: E402
import gen_labels as gl                                   # noqa: E402

PX_PER_MODULE = 4.0        # what the hires sees on face A, about
# Across the bars the polygon has to sit on them. Along them it need not: zbar
# often returns the extent of the scanlines it used, not of the symbol, which
# is why the Linker leaves that axis free by half a bar width. The same here.
ACROSS_PX = 6.0


def block(scale: float):
    cfg = gl._load_cfg(HERE.parent / "warehouse" / "warehouse.yaml")
    codes = cfg["codes"]
    ppm, maxpx = codes["texture_px_per_m"], codes["max_texture_px"]
    qr, _ = gl.make_box_label("WH1|A|01|1|SKU12345", "", codes["box_label"], ppm, maxpx)
    bar, _ = gl.make_bay_placard("0042", "0042", codes["box_placard"], ppm, maxpx)
    qr = np.asarray(qr.convert("L"))
    bar = np.asarray(bar.convert("L"))
    w = max(qr.shape[1], bar.shape[1])
    gap = 2
    img = np.full((qr.shape[0] + gap + bar.shape[0], w), 255, np.uint8)
    img[:qr.shape[0], (w - qr.shape[1]) // 2:(w + qr.shape[1]) // 2] = qr
    top = qr.shape[0] + gap
    img[top:, (w - bar.shape[1]) // 2:(w + bar.shape[1]) // 2] = bar
    img = cv2.resize(img, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
    bar_centre = (w / 2 * scale, (top + bar.shape[0] / 2) * scale)
    return img, bar_centre


def frame_with(angle: float):
    """The block turned by `angle` degrees, in a 1024x768 grey frame."""
    scale = PX_PER_MODULE / 7.0            # the textures draw 7 px per module
    img, (bx, by) = block(scale)
    h, w = img.shape
    # Room to turn, with a white margin like the carton face around it.
    pad = int(0.6 * max(h, w))
    big = np.full((h + 2 * pad, w + 2 * pad), 255, np.uint8)
    big[pad:pad + h, pad:pad + w] = img
    centre = (big.shape[1] / 2, big.shape[0] / 2)
    m = cv2.getRotationMatrix2D(centre, angle, 1.0)
    turned = cv2.warpAffine(big, m, (big.shape[1], big.shape[0]), borderValue=255)
    frame = np.full((768, 1024), 128, np.uint8)
    y0, x0 = (768 - turned.shape[0]) // 2, (1024 - turned.shape[1]) // 2
    frame[y0:y0 + turned.shape[0], x0:x0 + turned.shape[1]] = turned
    p = m @ np.array([bx + pad, by + pad, 1.0])
    half_bars = PX_PER_MODULE * len(gl.code128_modules("0042")) / 2
    return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR), (p[0] + x0, p[1] + y0), m, half_bars


def main() -> int:
    failures = 0
    decoder = bs.Decoder()
    plain = bs.Decoder()
    plain._level = lambda *a, **k: []
    for angle in (0.0, 5.0, -5.0, 12.0, -12.0, 18.0, 25.0, -25.0):
        frame, (tx, ty), m, half_bars = frame_with(angle)
        qrs, bars = decoder(frame)
        _, bars_plain = plain(frame)
        got = [b for b in bars if b[0] == "0042"]
        read = bool(got)
        along = across = 0.0
        if got:
            d = np.array([got[0][1][:, 0].mean() - tx, got[0][1][:, 1].mean() - ty])
            # Into the block's own axes: the inverse of its turn.
            along, across = (m[:, :2].T @ d)
        off = abs(across)
        need_level = abs(angle) > 8.2
        bad = (not read or off > ACROSS_PX or abs(along) > half_bars
               or (need_level and bool(bars_plain))
               or (not need_level and "0042" in decoder.last_levelled))
        failures += bad
        print(f"{angle:+6.1f} deg  read {read!s:<5}  without levelling "
              f"{bool(bars_plain)!s:<5}  levelled {'0042' in decoder.last_levelled!s:<5}  "
              f"across {off:4.1f} px  along {abs(along):5.1f} px"
              + ("  !! FAILED" if bad else ""))
    if failures:
        print(f"{failures} FAILED")
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
