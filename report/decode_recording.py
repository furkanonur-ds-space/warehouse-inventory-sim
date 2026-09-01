"""
Read a recorded flight back through the decoder, offline.

A scan that misses a code has two possible reasons and they call for opposite
fixes. Either the code never reached the decoder, because the frame holding it
was discarded before we looked, or it reached the decoder and was not read.
Watching the video answers neither: a code visible to a person may still be
one the detector cannot find.

So this puts the recording through the same decode_qr the flight uses, with no
time limit and nothing else competing, and reports which codes come back. What
the flight missed but this finds was there to be had and was thrown away.
What neither finds is a detector problem.

    python3 report/decode_recording.py out/video/camera_hires_link.mp4

Add --depth to say how far the camera was from the shelf, which decides how
the frame is split up; without it every frame is read whole.
"""
import argparse
import json
import os
import sys
import time
from collections import Counter

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, os.path.join(ROOT, "scanner"))
import scanner as s


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("video")
    parser.add_argument("--depth", type=float, default=None,
                        help="metres from the shelf, for the strip sizing")
    parser.add_argument("--hfov", type=float, default=None,
                        help="degrees, defaults to the hires camera's")
    parser.add_argument("--every", type=int, default=1,
                        help="read one frame in N, to go faster")
    args = parser.parse_args()

    cam = s.CameraDecoder("replay",
                          args.hfov or s.CAMERA_HFOV_DEG, 0.0)
    cam.depth = args.depth

    truth = json.load(open(os.path.join(ROOT, "warehouse/ground_truth.json"),
                           encoding="utf-8"))["codes"]
    boxes = {c["payload"] for c in truth if c.get("type") == "box_qr"}

    scan_path = os.path.join(ROOT, "out/inventory_scanned.json")
    flew = set()
    if os.path.exists(scan_path):
        flew = {v["id"] for v in json.load(open(scan_path,
                                                encoding="utf-8"))["items"]}

    capture = cv2.VideoCapture(args.video)
    if not capture.isOpened():
        raise SystemExit("cannot open %s" % args.video)

    seen = Counter()
    frames = 0
    read = 0
    start = time.time()
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        frames += 1
        if args.every > 1 and frames % args.every:
            continue
        read += 1
        for value, _cx, _cy, _w, _h in s.decode_qr(frame, cam):
            seen[value] += 1
        if read % 200 == 0:
            print("  %d frames, %d codes so far" % (read, len(seen)),
                  file=sys.stderr)
    capture.release()

    print("\n%s" % os.path.basename(args.video))
    print("  %d frames, %d read, %.0f s" % (frames, read, time.time() - start))
    print("  distinct codes found: %d" % len(seen))

    known = {v for v in seen if v in boxes}
    print("  of those, real box codes: %d" % len(known))

    if flew:
        recovered = sorted(known - flew)
        print("\n  codes this recording holds that the flight did not read: %d"
              % len(recovered))
        for value in recovered:
            print("    %-24s seen in %d frames" % (value, seen[value]))
        if not recovered:
            print("    none")

        lost = sorted(flew - known)
        print("\n  codes the flight read that are not in this recording: %d"
              % len(lost))
        if lost:
            print("    (the recorder drops frames when it falls behind)")

    thin = sorted((n, v) for v, n in seen.items() if v in boxes)[:8]
    print("\n  the codes with the fewest frames on them")
    for count, value in thin:
        mark = "  <- flight missed this" if value not in flew else ""
        print("    %-24s %3d frames%s" % (value, count, mark))


if __name__ == "__main__":
    main()
