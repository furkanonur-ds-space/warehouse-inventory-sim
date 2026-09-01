"""
Check the recorder before a flight relies on it.

Three things can go wrong and only one of them is obvious. It might not write
a playable file. It might block the camera callback, which would cost frames
the scan needs. Or it might let its queue grow when it falls behind, which
turns a recording into a memory leak.
"""
import os
import sys
import tempfile
import time

import cv2
import numpy as np

sys.path.insert(0, "/home/furk/starling/scanner")
import scanner as s

failures = []


def check(label, ok, detail=""):
    print("  %-50s %s" % (label, "ok" if ok else "FAILED " + detail))
    if not ok:
        failures.append(label)


out = tempfile.mkdtemp()
path = os.path.join(out, "clip.mp4")

print("1. it writes a file that can be read back")
rec = s.Recorder("test", path, fps=10)
rec.start()
frames = [np.random.randint(0, 255, (768, 1024, 3), dtype=np.uint8)
          for _ in range(4)]
for i in range(30):
    rec.offer(frames[i % 4])
    time.sleep(0.02)
rec.finish()

check("the file exists", os.path.exists(path))
if os.path.exists(path):
    check("it is not empty", os.path.getsize(path) > 1000,
          "(%d bytes)" % os.path.getsize(path))
    cap = cv2.VideoCapture(path)
    got, frame = cap.read()
    count = 0
    while got:
        count += 1
        got, frame = cap.read()
    cap.release()
    check("it plays back", count > 0, "(%d frames read)" % count)
    print("     wrote %d, dropped %d, read back %d"
          % (rec.written, rec.dropped, count))

print("\n2. offering a frame does not block the caller")
rec2 = s.Recorder("slow", os.path.join(out, "slow.mp4"), fps=10)
rec2.start()
frame = np.random.randint(0, 255, (768, 1024, 3), dtype=np.uint8)
start = time.perf_counter()
for _ in range(200):
    rec2.offer(frame)
elapsed = (time.perf_counter() - start) / 200 * 1000
check("offer costs under a millisecond", elapsed < 1.0,
      "(%.3f ms)" % elapsed)
print("     %.3f ms an offer, against 44 ms to decode a frame" % elapsed)

print("\n3. the queue drops rather than grows")
check("queue stayed at its limit", len(rec2.queue) <= rec2.queue.maxlen,
      "(%d, limit %d)" % (len(rec2.queue), rec2.queue.maxlen))
check("and said so", rec2.dropped > 0, "(dropped %d)" % rec2.dropped)
print("     offered 200 with a queue of %d, dropped %d"
      % (rec2.queue.maxlen, rec2.dropped))
rec2.finish()

print("\n4. it is off unless asked for")
check("RECORD_VIDEO defaults off", s.RECORD_VIDEO is False)

print("\n%s" % ("all checks passed" if not failures
                else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
sys.exit(1 if failures else 0)
