"""
YOLO finds the labels; zbar reads the crop instead of the whole frame.

WHAT THIS IS FOR, AND WHAT IT IS NOT FOR
----------------------------------------
The reader in barcode_scanner.py hands zbar the whole frame. zbar then has to
find a symbol and read it in one pass, and on a 1024x768 Gazebo frame a
distant Code128 is a few pixels per module in a picture that is mostly shelf.

This module puts a locator in front of that: a YOLO trained on real warehouse
photographs (13 classes, `qr` and `barkod` among them) says where the labels
are, the frame is cropped to each one, the crop is upscaled, and zbar reads
THAT. Same decoder, same symbologies, more pixels per module.

It does NOT fix a symbol that the frame cut in half. Face G loses codes
because the camera stands 0.29 m away and only 110 mm of the pass has the
whole symbol inside the frame - that is framing, and a crop cannot invent
pixels that were never rendered. Expect this to help where a symbol is SMALL,
not where it is CLIPPED. The A/B in scripts/yolo_replay_ab.sh is what settles
which of the two a given run is losing.

The second thing it buys is diagnostic and does not depend on decoding at all:
YOLO reports a label it can see even when zbar cannot read it. A frame with
`seen 2, read 1` says the label was framed and unreadable; a frame with
`seen 0` says it was never framed. The 17 boxes that give up their barcode are
one or the other and the run has never been able to say which.

WEIGHTS live outside this repository, with the dataset that trained them:
  ~/Desktop/warehouse_dataset/birlesik/model/yolo26s_640/weights/best.pt
Nothing here regenerates them; this is a consumer, like the report tools.

ENVIRONMENT. ultralytics and torch are NOT in the simulator's .venv, which is
the one that can reach gz-transport. Importing this module is therefore
optional and lazy: barcode_scanner.py only builds a CodeFinder when --yolo is
given, so a venv without torch runs exactly as it did before.
"""

from __future__ import annotations

import time
from pathlib import Path

import cv2
import numpy as np

# The two label classes, as the trained model names them. The model knows 13;
# these are the only ones a code reader has any use for. `etiket` is the
# placard as a whole - kept because a placard found with no code read inside
# it is precisely the diagnostic case above.
CODE_CLASSES = ("qr", "barkod")
PLACARD_CLASS = "etiket"

# The box itself. Not a code and never cropped for decoding - a crop of a
# whole carton has nothing in it zbar wants. It is here to be COUNTED: a
# reading tells you what was read, and only a box detector can tell you what
# was there to read. In a warehouse that is not this one there is no ground
# truth to score against, so the box count is the only denominator available.
#
# What it is not: a box seen in a frame with no code decoded in it is NOT a
# missed code. A carton is visible from down the aisle long before its label
# is readable, and this run reads 432 of 432 while most frames carry a box
# with no code in them. The per-frame number is a rate, not a fault; the
# fault, if there is one, only appears after the same physical box has been
# followed across the frames that saw it.
BOX_CLASS = "koli"

DEFAULT_WEIGHTS = Path.home() / (
    "Desktop/warehouse_dataset/birlesik/model/yolo26s_640/weights/best.pt")

# Measured on the sim's own saved frames, 2026-09-22: at conf 0.25 the model
# put a qr box in 100 of 100 frames at 0.82 mean confidence and a barkod box
# in 13 (those frames are the ones where the barcode did NOT read, so that is
# the worst case by construction). Below 0.15 it starts boxing shelf edges.
DEFAULT_CONF = 0.25

# How much to grow a detected box before cropping, as a fraction of its own
# size. A Code128 needs its quiet zone: zbar wants white either side of the
# bars and the model's box stops at the bars. 0.35 was enough on every frame
# tried here; it is a fraction rather than a constant so a near label and a
# far one both get a quiet zone proportional to the symbol.
DEFAULT_MARGIN = 0.35

# Upscale a crop until its long side reaches this, so zbar gets modules wide
# enough to sample. Cubic, because nearest turns a 2 px module into a staircase
# and zbar reads the staircase as bars.
DEFAULT_MIN_SIDE = 320


def require_device(device) -> None:
    """
    Refuse a GPU that is not there, NOW, rather than on the first frame.

    This cost a flight on 2026-09-28. The driver was not ready, ultralytics
    raised on the first frame it was given, and both readers died seconds
    after the scan started - so the vehicle flew all 24 waypoints, the QR side
    scored 420 of 420, and the barcode side left two empty files. Nothing said
    anything was wrong until the reports were run.

    The reader starts a few seconds BEFORE the scan does, so a check here is
    seen while there is still nothing to lose.

    There is deliberately no automatic fall back to the CPU. On 2026-09-17 a
    silent CPU fallback in the training pipeline dropped it to 9 s a step and
    locked the machine; here it would run the decoder far behind the camera
    and quietly change every number the run reports. `--yolo-device cpu` is
    there for anyone who means it.
    """
    if str(device).strip().lower() in ("cpu", "mps"):
        return
    import torch
    if torch.cuda.is_available():
        return
    raise SystemExit(
        f"DO NOT FLY: CUDA device '{device}' was asked for and torch cannot "
        "see a GPU.\n"
        "  torch.cuda.is_available() is False, so every frame would raise and "
        "this reader\n"
        "  would leave the flight with empty files - which is what happened on "
        "2026-09-28.\n"
        "  Check nvidia-smi. If it works, the driver was not ready yet: "
        "restart and try again.\n"
        "  To read on the CPU on purpose, and much slower than the camera: "
        "--yolo-device cpu")


class CodeFinder:
    """Where the labels are in a frame, according to the trained model."""

    def __init__(self, weights: Path | str = DEFAULT_WEIGHTS,
                 conf: float = DEFAULT_CONF, imgsz: int = 640,
                 device: str | int = 0):
        weights = Path(weights)
        if not weights.exists():
            raise FileNotFoundError(
                f"no YOLO weights at {weights}\n"
                "  they live with the dataset that trained them, not in this "
                "repository; pass --yolo-weights to point somewhere else")
        require_device(device)
        from ultralytics import YOLO          # lazy: see the module docstring
        self._model = YOLO(str(weights))
        self._names = self._model.names
        self.conf = conf
        self.imgsz = imgsz
        self.device = device
        self.ms = 0.0                          # last call, for the HUD

    def __call__(self, frame_bgr: np.ndarray) -> list[dict]:
        """Every label the model sees, in frame coordinates."""
        t0 = time.perf_counter()
        r = self._model.predict(frame_bgr, conf=self.conf, imgsz=self.imgsz,
                                device=self.device, verbose=False)[0]
        self.ms = (time.perf_counter() - t0) * 1e3
        out = []
        for box, cls, conf in zip(r.boxes.xyxy.tolist(),
                                  r.boxes.cls.tolist(),
                                  r.boxes.conf.tolist()):
            name = self._names[int(cls)]
            if name not in CODE_CLASSES and name not in (PLACARD_CLASS,
                                                          BOX_CLASS):
                continue
            x0, y0, x1, y1 = box
            out.append({"cls": name, "conf": float(conf),
                        "box": (float(x0), float(y0), float(x1), float(y1))})
        return out


def crop_for(frame: np.ndarray, box, margin: float = DEFAULT_MARGIN,
             min_side: int = DEFAULT_MIN_SIDE):
    """
    The crop zbar should see, plus what is needed to map its answer back.

    Returns (crop, origin_xy, scale) where a point p in the crop is the frame
    point origin + p / scale. Getting this wrong is not a small error: the
    Linker ties a barcode to a QR by where their polygons sit, and a polygon
    left in crop coordinates would tie every code to whatever is nearest the
    top left of the frame.
    """
    h, w = frame.shape[:2]
    x0, y0, x1, y1 = box
    mx, my = (x1 - x0) * margin, (y1 - y0) * margin
    x0 = max(0, int(x0 - mx)); y0 = max(0, int(y0 - my))
    x1 = min(w, int(x1 + mx)); y1 = min(h, int(y1 + my))
    if x1 - x0 < 8 or y1 - y0 < 8:
        return None, (0, 0), 1.0
    crop = frame[y0:y1, x0:x1]
    scale = 1.0
    long_side = max(crop.shape[:2])
    if long_side < min_side:
        scale = min_side / long_side
        crop = cv2.resize(crop, None, fx=scale, fy=scale,
                          interpolation=cv2.INTER_CUBIC)
    return crop, (x0, y0), scale


def to_frame(poly: np.ndarray, origin, scale: float) -> np.ndarray:
    """A polygon zbar returned in crop coordinates, put back in the frame."""
    return (np.asarray(poly, dtype=np.float32) / scale
            + np.asarray(origin, dtype=np.float32)).astype(np.int32)


def covers(box, poly) -> bool:
    """Is this decoded code inside that box?

    The code's polygon CENTRE against the box, not an overlap test. A QR sits
    on the face of its carton, so its centre is inside the carton's box
    whenever it belongs to it; an overlap test would also tie a code to the
    carton behind the one it is printed on, which is the pair most often in
    the same frame.
    """
    x0, y0, x1, y1 = box
    cx = float(sum(px for px, _ in poly)) / len(poly)
    cy = float(sum(py for _, py in poly)) / len(poly)
    return x0 <= cx <= x1 and y0 <= cy <= y1


def coverage(regions: list[dict], polys) -> tuple[list[dict], int]:
    """
    Every box the model saw, each told whether a code was decoded inside it.

    Returns the box records and how many of them carried no code. The count is
    a per-frame rate and not a fault list: see BOX_CLASS above for why a box
    without a code in one frame is the normal case and not a miss.
    """
    boxes = []
    uncovered = 0
    for r in regions:
        if r["cls"] != BOX_CLASS:
            continue
        hit = any(covers(r["box"], poly) for poly in polys)
        if not hit:
            uncovered += 1
        boxes.append({"box": [round(v, 1) for v in r["box"]],
                      "conf": round(r["conf"], 3), "covered": hit})
    return boxes, uncovered
