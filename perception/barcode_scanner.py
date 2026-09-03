#!/usr/bin/env python3
"""
Read the CODE128 bay placards off the scanning camera, live, in its own process.

    python3 perception/barcode_scanner.py                 # live, with a window
    python3 perception/barcode_scanner.py --headless      # no window
    python3 perception/barcode_scanner.py --replay out/frames

NOTHING IN `scanner/` IS TOUCHED, IMPORTED OR WRITTEN TO. This is a second
subscriber on a camera topic Gazebo is already publishing: it runs beside a
flight, or after one on saved frames, and if it crashes the flight does not
notice. The only files it reads from the flight side are `scanner/layout.json`
(for the world and model name) and `warehouse/warehouse.yaml` (for the label
geometry), both read-only.

WHAT THE BARCODE SAYS. It names the box, as the QR does: the payload is the
digits of that box's own SKU, `55414` where the QR reads
`WH1|A|01|1|SKU55414`. The two are deliberately not the same string. They
carry the same fact by different routes, in different symbologies, decoded by
different libraries, so reading both is evidence that either can be read on
its own rather than a copy of one measurement.

It used to name the slot instead, which all three boxes of a bay level shared,
and could then say only that a shelf was occupied. It now identifies the box,
which means a barcode read where the QR failed is a box recovered rather than
a hint.

Each reading is still linked to the QR sitting directly above it in the same
frame, and the two are compared: the link is what makes a disagreement
visible.

HOW THE LINK IS MADE. The world generator puts the placard immediately below
the box label, on the same vertical centre line: the drop from the QR symbol to
the bars is `qr_rise + label_h/2 + LABEL_GAP + placard_h/2 - bar_rise`, all of
which come out of `warehouse/gen_labels.py` rather than being re-measured here.
The QR's own corners give the pixels-per-metre scale in that frame, so the
placard's expected pixel position follows, and the barcode is linked to the
nearest QR whose prediction it lands on.

Two details that are easy to get wrong, both of which cost a working link:

  * the symbols are not centred in their labels - each label carries a caption
    strip, so the QR sits `qr_rise` above its label centre and the bars sit
    `bar_rise` above theirs. Ignoring both shifts the prediction by over a
    centimetre, which is a third of the tolerance.
  * zbar does not return a rotated box for a linear symbol. In a good part of
    the readings the polygon is TWO POINTS, the leading edge of the bars only,
    and its midpoint sits half a bar-width away from the true centre. The
    matching test therefore allows free travel along the bar axis and keeps
    full tolerance vertically, rather than pretending the centre is known.

DECODING is zbar (`pyzbar`), for both symbologies, on the grey frame. It reads
QR and CODE128 in one pass. Install it into the project venv with
`.venv/bin/pip install pyzbar`; the shared library it binds to is already on
this machine.

The camera callback only stores the newest frame. Decoding and drawing happen
in the main loop, so a slow frame drops the one behind it instead of backing up
a queue - the same reason the flight code captures its pose inside the callback
and does its work outside.

OUTPUTS (both under `out/`, neither touched by anything else):
    barcode_readings.jsonl   every reading: payload, polygon, quality, the QR
                             it was linked to and how far off the prediction it
                             landed
    barcode_inventory.json   one record per box whose barcode was read, with
                             whether it agreed with the QR beside it
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "warehouse"))

import gen_labels as gl                      # noqa: E402  (world generator)
from gen_world import LABEL_GAP              # noqa: E402

LAYOUT = REPO_ROOT / "scanner" / "layout.json"
CONFIG = REPO_ROOT / "warehouse" / "warehouse.yaml"
READINGS = REPO_ROOT / "out" / "barcode_readings.jsonl"
SUMMARY = REPO_ROOT / "out" / "barcode_inventory.json"

# How far from its predicted place a barcode may land and still be called the
# one belonging to that QR. Boxes sit about 0.4 m apart along the shelf, so
# 0.12 m cannot reach the neighbour's placard.
LINK_TOL_M = 0.12

# A QR smaller than this in the frame gives a pixels-per-metre scale too noisy
# to predict anything from.
MIN_QR_SIDE_PX = 12.0


# --------------------------------------------------------------- geometry

class Linker:
    """
    Where the placard should be in the frame, given the QR above it.

    Every number comes from the same functions that drew the labels, so a
    change to the label layout moves the prediction with it instead of leaving
    a constant here to go stale.
    """

    def __init__(self, cfg: dict, tol_m: float = LINK_TOL_M):
        codes = cfg["codes"]
        ppm_tex, max_px = codes["texture_px_per_m"], codes["max_texture_px"]
        label_h = codes["box_label"]["label"][1]
        placard_h = codes["box_placard"]["label"][1]

        # The QR side as ZBAR REPORTS IT: zbar's polygon runs through the
        # centres of the outer modules, not the outer edge of the quiet zone,
        # so the side it returns is slightly smaller than the drawn symbol.
        # gen_labels carries that ratio; using the drawn size instead would
        # scale every prediction by about 1%.
        self.qr_side_m, qr_rise = gl.box_label_geometry(
            codes["box_label"], ppm_tex, max_px)
        self.bar_w_m, _, bar_rise = gl.placard_geometry(
            codes["box_placard"], ppm_tex, max_px)

        # QR symbol centre down to bar centre, in metres.
        self.drop_m = (qr_rise + label_h / 2.0 + LABEL_GAP
                       + placard_h / 2.0 - bar_rise)
        self.tol_m = tol_m

    def predict(self, qr_poly: np.ndarray):
        """Expected bar centre in pixels, and the frame's pixels per metre."""
        n = len(qr_poly)
        side_px = float(np.mean([np.linalg.norm(qr_poly[i] - qr_poly[(i + 1) % n])
                                 for i in range(n)]))
        if side_px < MIN_QR_SIDE_PX:
            return None
        px_per_m = side_px / self.qr_side_m
        # The vehicle flies level and the labels are upright, so "down the
        # box face" is +y in the image.
        return (float(qr_poly[:, 0].mean()),
                float(qr_poly[:, 1].mean()) + self.drop_m * px_per_m,
                px_per_m)

    def link(self, qrs: list, bar_poly: np.ndarray):
        """
        The QR this barcode belongs to, as (payload, distance_m), or (None, None).

        `qrs` is [(payload, polygon)] from the same frame.
        """
        bx = float(bar_poly[:, 0].mean())
        by = float(bar_poly[:, 1].mean())
        # Two points means zbar gave only the leading edge of the bars; the
        # centre is then unknown along the bar axis by half a bar width, and
        # which side is unknown too, so that axis is left free.
        slack_x = self.bar_w_m / 2.0 if len(bar_poly) <= 2 else 0.0

        best, best_d = None, float("inf")
        for payload, poly in qrs:
            pred = self.predict(poly)
            if pred is None:
                continue
            px, py, px_per_m = pred
            dx = abs(bx - px) / px_per_m
            dy = abs(by - py) / px_per_m
            d = math.hypot(max(dx - slack_x, 0.0), dy)
            if d < best_d:
                best, best_d = payload, d
        if best is None or best_d > self.tol_m:
            return None, None
        return best, round(best_d, 4)


def barcode_of_qr(payload: str):
    """
    The barcode payload the box's own QR implies: `WH1|A|03|3|SKU55414` ->
    `55414`.

    Built with the generator's own function so the two cannot disagree about
    how the one is derived from the other.

    The barcode used to name the slot, which three boxes shared, so this could
    only ever say which shelf the reading belonged to. It now names the box,
    and a reading either matches the QR beside it or does not.
    """
    parts = payload.split("|")
    if len(parts) < 5:
        return None
    try:
        return gl.placard_payload(parts[4])
    except (ValueError, TypeError):
        return None


# ---------------------------------------------------------------- decode

class Decoder:
    """
    zbar over the grey frame, QR and CODE128 in one pass.

    A second pass on an Otsu-thresholded copy is tried only when the first
    pass found no barcode. Gazebo renders the white quiet zone as mid grey and
    the contrast varies across the frame, which is the same reason the flight
    code thresholds its crops; doing it unconditionally would cost time on the
    frames that already decode.
    """

    def __init__(self):
        from pyzbar import pyzbar
        from pyzbar.pyzbar import ZBarSymbol
        self._zbar = pyzbar
        self._symbols = [ZBarSymbol.QRCODE, ZBarSymbol.CODE128]
        self._code128 = ZBarSymbol.CODE128

    def __call__(self, frame_bgr: np.ndarray):
        gray = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        results = list(self._zbar.decode(gray, symbols=self._symbols))
        if not any(r.type == "CODE128" for r in results):
            _, binary = cv2.threshold(gray, 0, 255,
                                      cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            extra = self._zbar.decode(binary, symbols=[self._code128])
            seen = {(r.type, r.data) for r in results}
            results += [r for r in extra if (r.type, r.data) not in seen]

        qrs, bars = [], []
        for r in results:
            poly = polygon_of(r)
            payload = r.data.decode("utf-8", "replace")
            (qrs if r.type == "QRCODE" else bars).append(
                (payload, poly, int(r.quality)))
        return qrs, bars


def polygon_of(result) -> np.ndarray:
    """
    zbar's corners as an array, falling back to its bounding rect.

    Linear symbols sometimes come back with an empty or two point polygon.
    Both are kept as they are rather than padded out: how many points there
    are is what tells the linker whether the centre is trustworthy.
    """
    if result.polygon:
        return np.array([[p.x, p.y] for p in result.polygon], dtype=np.float64)
    r = result.rect
    return np.array([[r.left, r.top], [r.left + r.width, r.top],
                     [r.left + r.width, r.top + r.height],
                     [r.left, r.top + r.height]], dtype=np.float64)


# ------------------------------------------------------------------ draw

COL_QR = (120, 230, 120)
COL_BAR_OK = (60, 190, 255)
COL_BAR_FREE = (90, 90, 235)
COL_HUD = (235, 235, 235)


def draw(frame: np.ndarray, qrs, bars, stats: dict) -> np.ndarray:
    """The live picture: what was decoded, where, and what it was tied to."""
    view = frame.copy()
    for payload, poly, _q in qrs:
        pts = poly.astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(view, [pts], True, COL_QR, 2)
        x, y = int(poly[:, 0].min()), int(poly[:, 1].min())
        label(view, payload, (x, y - 6), COL_QR)
    for payload, poly, quality, linked, dist in bars:
        colour = COL_BAR_OK if linked else COL_BAR_FREE
        if len(poly) >= 3:
            cv2.polylines(view, [poly.astype(np.int32).reshape(-1, 1, 2)],
                          True, colour, 2)
        else:
            # Only the leading edge came back. Drawing it as the line it is
            # says more than a made up box around it.
            p = poly.astype(int)
            cv2.line(view, tuple(p[0]), tuple(p[-1]), colour, 2)
        x, y = int(poly[:, 0].min()), int(poly[:, 1].max())
        text = payload if not linked else f"{payload} -> {linked.split('|')[-1]}"
        label(view, text, (x, y + 16), colour)

    lines = [
        f"frame {stats['frames']}   {stats['fps']:.1f} fps   "
        f"decode {stats['decode_ms']:.1f} ms",
        f"QR {stats['qr_hits']}   barcode {stats['bar_hits']}   "
        f"linked {stats['linked']}",
        f"boxes with a barcode: {stats['boxes']}   "
        f"agrees: {stats['agree']}   disagrees: {stats['disagree']}",
    ]
    for i, line in enumerate(lines):
        label(view, line, (12, 26 + 24 * i), COL_HUD, scale=0.62)
    return view


def label(img, text, org, colour, scale: float = 0.5) -> None:
    """Text with a dark backing, so it stays readable over a pale shelf."""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0),
                3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, colour,
                1, cv2.LINE_AA)


# ------------------------------------------------------------------ main

class Session:
    """Everything the run accumulates, and the two files it leaves behind."""

    def __init__(self, readings: Path, summary: Path, source: str):
        self.readings_path = readings
        self.summary_path = summary
        self.source = source
        self.boxes: dict[str, dict] = {}
        self.unlinked: dict[str, int] = {}
        self.frames = 0
        # Frame rate is measured between arrivals, not from the decode time:
        # the question the window has to answer is how much of the camera's
        # output is actually being looked at.
        self.fps = 0.0
        self.last_t = None
        self.qr_hits = 0
        self.bar_hits = 0
        self.linked = 0
        readings.parent.mkdir(parents=True, exist_ok=True)
        self._fh = readings.open("a")

    def record(self, payload, poly, quality, linked, dist, frame_no) -> None:
        self.bar_hits += 1
        now = datetime.now().isoformat(timespec="seconds")
        self._fh.write(json.dumps({
            "t": now, "frame": frame_no, "symbology": "CODE128",
            "payload": payload, "quality": quality,
            "polygon": [[round(float(x), 1), round(float(y), 1)] for x, y in poly],
            "linked_qr": linked, "link_error_m": dist,
        }, ensure_ascii=False) + "\n")
        if not linked:
            self.unlinked[payload] = self.unlinked.get(payload, 0) + 1
            return
        self.linked += 1
        rec = self.boxes.setdefault(linked, {
            "qr": linked, "barcode": payload, "readings": 0,
            "best_quality": quality, "expected_barcode": barcode_of_qr(linked),
            "first_seen": now, "last_seen": now, "disagreements": [],
        })
        rec["readings"] += 1
        rec["last_seen"] = now
        rec["best_quality"] = max(rec["best_quality"], quality)
        if payload != rec["barcode"]:
            # Two different barcodes linked to one box. Worth keeping rather
            # than overwriting: it is either a misread or a bad link, and both
            # are invisible if the last one silently wins.
            rec["disagreements"].append(payload)

    def flush(self) -> None:
        self._fh.flush()
        agree, disagree = self.tally()
        self.summary_path.write_text(json.dumps({
            "generated": datetime.now().isoformat(timespec="seconds"),
            "source": self.source,
            "frames_processed": self.frames,
            "qr_readings": self.qr_hits,
            "barcode_readings": self.bar_hits,
            "barcode_readings_linked": self.linked,
            "boxes_with_barcode": len(self.boxes),
            "barcode_agrees": agree,
            "barcode_disagrees": disagree,
            "unlinked_payloads": self.unlinked,
            "boxes": sorted(self.boxes.values(), key=lambda r: r["qr"]),
        }, indent=2, ensure_ascii=False))

    def tally(self) -> tuple[int, int]:
        agree = sum(1 for r in self.boxes.values()
                    if r["expected_barcode"] == r["barcode"])
        return agree, len(self.boxes) - agree

    def close(self) -> None:
        self.flush()
        self._fh.close()


def topic_from_layout(path: Path) -> str:
    layout = json.loads(path.read_text())
    model = f"{layout['model']}_0"
    return (f"/world/{layout['world']}/model/{model}"
            f"/link/camera_hires_link/sensor/camera/image")


def live(args, decoder, linker, session) -> int:
    """Subscribe to the camera and work on the newest frame there is."""
    os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
    os.environ.setdefault("GZ_IP", "127.0.0.1")
    import gz.transport13 as trans
    from gz.msgs10.image_pb2 import Image

    latest = {"frame": None}

    def on_image(msg):
        # Keep the callback to a copy and nothing else. Decoding here would
        # block the transport thread and stale frames would pile up behind it.
        try:
            img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
                (msg.height, msg.width, 3))
            latest["frame"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        except Exception:
            pass

    node = trans.Node()
    if not node.subscribe(Image, args.topic, on_image):
        print(f"could not subscribe to {args.topic}")
        return 1
    print(f"listening on {args.topic}")
    print("waiting for the first frame; is the simulator running?")

    seen_first = False
    last = time.time()
    try:
        while True:
            frame = latest["frame"]
            if frame is None:
                time.sleep(0.05)
                continue
            if not seen_first:
                print(f"first frame: {frame.shape[1]}x{frame.shape[0]}")
                seen_first = True
            latest["frame"] = None
            if not step(frame, decoder, linker, session, args):
                break
            # Nothing to do until the next frame arrives; the camera runs at
            # 10 Hz and spinning here would burn a core the simulator wants.
            elapsed = time.time() - last
            if elapsed < 0.02:
                time.sleep(0.02 - elapsed)
            last = time.time()
    except KeyboardInterrupt:
        print("\nstopped")
    return 0


def replay(args, decoder, linker, session) -> int:
    """Run the same pipeline over saved frames, with no simulator involved."""
    files = sorted(p for p in Path(args.replay).iterdir()
                   if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    if not files:
        print(f"no images in {args.replay}")
        return 1
    print(f"replaying {len(files)} frames from {args.replay}")
    for path in files:
        frame = cv2.imread(str(path))
        if frame is None:
            continue
        if not step(frame, decoder, linker, session, args, wait=args.wait):
            break
    return 0


def step(frame, decoder, linker, session, args, wait: int = 1) -> bool:
    """One frame: decode, link, record, draw. False means the user asked to stop."""
    t0 = time.perf_counter()
    qrs, bars = decoder(frame)
    decode_ms = (time.perf_counter() - t0) * 1000.0
    session.frames += 1
    session.qr_hits += len(qrs)

    qr_polys = [(payload, poly) for payload, poly, _ in qrs]
    drawn_bars = []
    for payload, poly, quality in bars:
        linked, dist = linker.link(qr_polys, poly)
        session.record(payload, poly, quality, linked, dist, session.frames)
        drawn_bars.append((payload, poly, quality, linked, dist))

    now = time.perf_counter()
    if session.last_t is not None:
        instant = 1.0 / max(now - session.last_t, 1e-6)
        session.fps = instant if session.fps == 0.0 else \
            0.85 * session.fps + 0.15 * instant
    session.last_t = now

    if session.frames % args.flush_every == 0:
        session.flush()

    if args.headless:
        if session.frames % 20 == 0:
            agree, disagree = session.tally()
            print(f"frame {session.frames}  {decode_ms:5.1f} ms  "
                  f"QR {len(qrs)}  barcode {len(bars)}  "
                  f"boxes {len(session.boxes)}  agree {agree}  "
                  f"disagree {disagree}")
        return True

    agree, disagree = session.tally()
    view = draw(frame, qrs, drawn_bars, {
        "frames": session.frames, "fps": session.fps, "decode_ms": decode_ms,
        "qr_hits": session.qr_hits, "bar_hits": session.bar_hits,
        "linked": session.linked, "boxes": len(session.boxes),
        "agree": agree, "disagree": disagree,
    })
    if args.scale != 1.0:
        view = cv2.resize(view, None, fx=args.scale, fy=args.scale,
                          interpolation=cv2.INTER_AREA)
    cv2.imshow("barcode scanner", view)
    key = cv2.waitKey(wait) & 0xFF
    if key in (ord("q"), 27):
        return False
    if key == ord("s"):
        shot = REPO_ROOT / "out" / f"barcode_frame_{session.frames:05d}.png"
        cv2.imwrite(str(shot), view)
        print(f"saved {shot}")
    return True


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", help="camera topic; defaults to the scanning "
                                    "camera named in scanner/layout.json")
    ap.add_argument("--replay", type=Path,
                    help="read a directory of saved frames instead of the "
                         "simulator")
    ap.add_argument("--headless", action="store_true",
                    help="no window; print progress instead")
    ap.add_argument("--scale", type=float, default=1.0,
                    help="scale the window, e.g. 0.7 for a 1280 wide camera")
    ap.add_argument("--wait", type=int, default=1,
                    help="milliseconds to hold each replayed frame; 0 waits "
                         "for a key")
    ap.add_argument("--tolerance", type=float, default=LINK_TOL_M,
                    help="how far a barcode may sit from its predicted place "
                         "and still be linked, in metres")
    ap.add_argument("--readings", type=Path, default=READINGS)
    ap.add_argument("--summary", type=Path, default=SUMMARY)
    ap.add_argument("--flush-every", type=int, default=50,
                    help="write the summary every N frames, so a run that is "
                         "interrupted still leaves one")
    args = ap.parse_args()

    try:
        decoder = Decoder()
    except ImportError:
        print("pyzbar is not installed in this environment.")
        print("  .venv/bin/pip install pyzbar")
        print("It binds to libzbar, which is already present on this machine.")
        return 1

    cfg = yaml.safe_load(CONFIG.read_text())
    linker = Linker(cfg, args.tolerance)
    print(f"placard sits {linker.drop_m*100:.1f} cm below the QR, "
          f"tolerance {args.tolerance*100:.0f} cm")

    if not args.topic:
        args.topic = topic_from_layout(LAYOUT)
    source = str(args.replay) if args.replay else args.topic
    session = Session(args.readings, args.summary, source)

    try:
        rc = replay(args, decoder, linker, session) if args.replay \
            else live(args, decoder, linker, session)
    finally:
        session.close()
        if not args.headless:
            cv2.destroyAllWindows()
        agree, disagree = session.tally()
        print(f"\n{session.frames} frames · {session.bar_hits} barcode readings "
              f"({session.linked} linked) · {len(session.boxes)} boxes")
        print(f"barcode agrees {agree}, disagrees {disagree}")
        print(f"readings: {args.readings}")
        print(f"summary : {args.summary}")
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
