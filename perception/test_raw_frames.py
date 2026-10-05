#!/usr/bin/env python3
"""
Check that --save-raw keeps every Nth frame untouched, and says what it kept.

    .venv/bin/python perception/test_raw_frames.py

No simulator. Ten frames, each a grey field with the world's own QR and
barcode labels pasted in at a different place, go through the reader with
--replay, exactly as a flight's frames go through step(). Then:

  * the frames kept are the ones on the stride, and only those;
  * each is pixel-identical to the frame the decoder saw - an offline noise
    study on a frame that was already altered measures the alteration;
  * index.jsonl has one line per image, naming it, with the frame number the
    readings carry and the codes the decoder read in it;
  * the summary counts them;
  * the limit stops it.
"""
from __future__ import annotations

import json
import subprocess
import sys
import tempfile
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from test_level import block                              # noqa: E402

N_FRAMES = 10
EVERY = 3


def frames(where: Path) -> list[np.ndarray]:
    img, _ = block(1.0)
    out = []
    for i in range(N_FRAMES):
        f = np.full((768, 1024), 128, np.uint8)
        y, x = 100 + 20 * i, 100 + 30 * i
        f[y:y + img.shape[0], x:x + img.shape[1]] = img
        f = cv2.cvtColor(f, cv2.COLOR_GRAY2BGR)
        # Named so the replay order is the frame order: frame k is file k-1.
        cv2.imwrite(str(where / f"in_{i:03d}.png"), f)
        out.append(f)
    return out


def run(tmp: Path, src: Path, limit: int) -> tuple[Path, dict]:
    raw = tmp / f"raw_{limit}"
    summary = tmp / f"summary_{limit}.json"
    subprocess.run(
        [sys.executable, str(HERE / "barcode_scanner.py"), "--headless",
         "--replay", str(src), "--wait", "0",
         "--readings", str(tmp / f"readings_{limit}.jsonl"),
         "--summary", str(summary),
         "--save-raw", str(raw), "--save-raw-every", str(EVERY),
         "--save-raw-limit", str(limit)],
        check=True, capture_output=True, text=True)
    return raw, json.loads(summary.read_text())


def main() -> int:
    fails = []
    with tempfile.TemporaryDirectory() as t:
        tmp = Path(t)
        src = tmp / "in"
        src.mkdir()
        given = frames(src)

        raw, summary = run(tmp, src, limit=100)
        want = [k for k in range(1, N_FRAMES + 1) if k % EVERY == 0]
        index = [json.loads(line) for line in
                 (raw / "index.jsonl").read_text().splitlines()]
        kept = sorted(p.name for p in raw.glob("*.png"))

        if [r["frame"] for r in index] != want:
            fails.append(f"index frames {[r['frame'] for r in index]}, "
                         f"want {want}")
        if kept != sorted(r["file"] for r in index):
            fails.append(f"images {kept} do not match the index")
        for r in index:
            got = cv2.imread(str(raw / r["file"]), cv2.IMREAD_UNCHANGED)
            if got is None or not np.array_equal(got, given[r["frame"] - 1]):
                fails.append(f"{r['file']} is not the frame the decoder saw")
            if r["qr"] != ["WH1|A|01|1|SKU12345"]:
                fails.append(f"{r['file']}: qr {r['qr']}")
            if r["barcode"] != ["0042"]:
                fails.append(f"{r['file']}: barcode {r['barcode']}")
        rf = summary.get("raw_frames") or {}
        if rf.get("saved") != len(want) or rf.get("every") != EVERY:
            fails.append(f"summary says {rf}")

        raw, summary = run(tmp, src, limit=2)
        kept = sorted(raw.glob("*.png"))
        if len(kept) != 2 or summary["raw_frames"]["saved"] != 2:
            fails.append(f"limit 2 kept {len(kept)}, summary "
                         f"{summary['raw_frames']}")

    for f in fails:
        print("FAIL", f)
    print(f"{'ok' if not fails else 'FAILED'}: every {EVERY} of {N_FRAMES} "
          f"frames kept untouched, indexed, counted, capped")
    return 1 if fails else 0


if __name__ == "__main__":
    raise SystemExit(main())
