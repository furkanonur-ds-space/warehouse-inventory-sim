"""
Check the two camera path without a simulator.

Three things that a flight cannot separate cleanly:

  1. A code seen dead ahead by the hires lands on the near face, and the same
     code seen by the rear camera lands on the far face.
  2. A code off to one side lands on the correct side. The centre of the frame
     cannot show this: bearing is zero there, so a left for right swap in the
     rear camera's geometry would look perfect and still put every off axis
     code on the wrong part of the shelf.
  3. A frame arriving while the previous one is still decoding is dropped
     rather than queued.
"""
import glob
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scanner as s

# The generated label textures, which is where a real code of the real size
# comes from. Taking one of these rather than drawing a fresh QR is what makes
# the test measure the pipeline instead of a code it invented for itself.
def texture_dir():
    """
    Where the generated label textures ended up.

    Three places, in order: a directory the layout names, the generator's own
    output tree before it is installed, and the PX4 tree that setup_px4.sh
    copies it into. The last one used to be a path under one developer's home
    directory, which meant these tests only ran on that machine.  PX4_DIR is
    the same override setup_px4.sh and launch_sim.sh take.
    """
    here = os.path.dirname(os.path.abspath(s.LAYOUT_PATH))
    px4 = os.environ.get("PX4_DIR") or os.path.expanduser("~/PX4-Autopilot")
    candidates = []
    if s.LAYOUT.get("texture_dir"):
        candidates.append(os.path.join(here, s.LAYOUT["texture_dir"]))
    candidates += [
        os.path.join(here, "..", "warehouse", "generated", "gz", "models",
                     "warehouse_assets", "materials", "textures"),
        os.path.join(px4, "Tools", "simulation", "gz", "models",
                     "warehouse_assets", "materials", "textures"),
    ]
    for candidate in candidates:
        if os.path.isdir(candidate):
            return os.path.normpath(candidate)
    raise SystemExit("no label textures; run setup_px4.sh first")


TEX = texture_dir()

# The first leg of the route, whatever the layout makes it. Taking the lane,
# the heading and the two camera distances from build_route rather than naming
# them here is what keeps this a test of the pipeline and not of a copy of the
# numbers.
_ROUTE = s.build_route()
# By position, not by count: a route entry has gained a field three times now
# and each time this line was the thing that broke.
_FIRST = _ROUTE[0]
LANE_X, YAW = _FIRST[0], _FIRST[3]
HIRES_DEPTH, REAR_DEPTH = _FIRST[4], _FIRST[5]
DRONE_Y = -5.0

# The distance the labels are drawn at, which is not the distance the
# pipeline places them with. Drawing them at the real flight distance made
# this a legibility test by accident: the rear camera reads about eleven of
# every twelve labels at its true 1.10 m, so a pass depended on which label
# came first out of the texture directory. Whether a code lands on the right
# face and the right side follows from its bearing, and the bearing comes
# from the pixel position, so neither depends on how large it is drawn.
RENDER_M = 0.70


def frame_with_label(dist, width, height, hfov_deg, offset_frac=0.0):
    """One label at its true angular size, offset_frac of half a frame aside."""
    label = cv2.imread(sorted(glob.glob(os.path.join(TEX, "box_*.png")))[0])
    scale = width / (2 * dist * math.tan(math.radians(hfov_deg) / 2))
    w = int(0.10 * scale)
    h = int(0.15 * scale)
    small = cv2.resize(label, (w, h), interpolation=cv2.INTER_AREA)
    img = np.full((height, width, 3), 160, dtype=np.uint8)
    cx = int(width / 2 + offset_frac * width / 2)
    y0 = (height - h) // 2
    x0 = max(0, min(width - w, cx - w // 2))
    img[y0:y0 + h, x0:x0 + w] = small
    return img


class Stamp:
    def __init__(self, seconds):
        self.sec = int(seconds)
        self.nsec = int((seconds - int(seconds)) * 1e9)


class Header:
    def __init__(self, seconds):
        self.stamp = Stamp(seconds)


class Msg:
    """A camera frame, carrying the simulation time it was taken at."""

    def __init__(self, img, taken_at=None):
        self.data = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).tobytes()
        self.height, self.width = img.shape[:2]
        self.header = Header(SIM_NOW if taken_at is None else taken_at)


def place(callback, img, taken_at=None):
    """
    Run one frame through a camera callback and return what it recorded.

    The callback hands the frame to that camera's decoder thread and returns,
    so pending is still empty the instant it comes back. Wait for the decoder
    to go idle rather than sleeping a guessed amount: a fixed sleep would pass
    on this machine and fail on a slower one, which is the sort of test that
    is worse than no test. A frame the callback rejected before the handover
    leaves the decoder idle already, so this returns at once for those.
    """
    cam = s.HIRES if callback is s.on_hires_image else s.REAR
    s.inventory.clear()
    s.pending.clear()
    callback(Msg(img, taken_at))
    deadline = time.time() + 10.0
    while (cam.queue or cam.working) and time.time() < deadline:
        time.sleep(0.005)
    while s.pending:
        qr, cx, cy, fw, fh, pose, hfov, cam, depth = s.pending.popleft()
        s.record_detection(qr, cx, cy, fw, fh, pose, hfov, cam, depth)
    return dict(s.inventory)


# Each camera decodes on a thread of its own, so those threads have to be
# running for a callback to do anything at all.
for _cam in (s.HIRES, s.REAR):
    _cam.start()

s.origin_pos = {"n": s.SPAWN_Y, "e": s.SPAWN_X, "d": 0.0}
s.current_pos = {"n": DRONE_Y, "e": LANE_X, "d": -(0.75 - s.GROUND_OFFSET)}
s.current_yaw = {"deg": YAW}
# What the flight loop does on entering a lane. The distance each camera is
# given is to its own lens, not to base_link, because that is what turns a
# bearing into a position.
s.HIRES.depth = HIRES_DEPTH - s.HIRES_MOUNT_X
s.REAR.depth = REAR_DEPTH + s.REAR_MOUNT_X

# A plausible simulation time, and a history for the frames to be placed
# against. Two samples either side of the moment the frames are taken at is
# enough: the vehicle is standing still here, because what is under test is
# which face and which side a code lands on, not the lookup itself. The lookup
# has its own test.
SIM_NOW = 120.0
s.sim_now["s"] = SIM_NOW
s.pose_history[:] = [
    (SIM_NOW - 1.0, DRONE_Y, LANE_X, -(0.75 - s.GROUND_OFFSET), YAW),
    (SIM_NOW + 1.0, DRONE_Y, LANE_X, -(0.75 - s.GROUND_OFFSET), YAW),
]

failures = []


def check(label, got, want):
    ok = got == want
    print("  %-58s %s" % (label, "ok" if ok else "FAILED  got %r want %r"
                          % (got, want)))
    if not ok:
        failures.append(label)


print("lane x=%.2f, heading %+.0f, hires lens at %.2f m, rear lens at %.2f m\n"
      % (LANE_X, YAW, s.HIRES.depth, s.REAR.depth))

print("1. a code dead ahead lands on the face that camera looks at")
near = place(s.on_hires_image,
             frame_with_label(RENDER_M, 1024, 768, 60.0))
far = place(s.on_rear_image,
            frame_with_label(RENDER_M, 1280, 800, 90.0))
check("hires, centred", [v["shelf"] for v in near.values()], ["A"])
check("rear, centred", [v["shelf"] for v in far.values()], ["B"])

print("\n2. a code off to one side lands on the correct side")
# Gazebo points the optical axis along +x of the camera frame with +y to the
# left, so the right of an image is -y of that frame. The hires shares
# base_link's orientation, so the right of its image is the vehicle's right,
# and at heading -90, facing west, that points north: +y in the world. The
# rear camera is mounted turned through 180 degrees about the vertical, so the
# right of its image is the vehicle's left, -y at the same heading.
for name, callback, width, height, hfov, expect_sign in [
        ("hires", s.on_hires_image, 1024, 768, 60.0, +1),
        ("rear", s.on_rear_image, 1280, 800, 90.0, -1)]:
    for side, frac in (("right of frame", +0.5), ("left of frame", -0.5)):
        got = place(callback,
                    frame_with_label(RENDER_M, width, height, hfov, frac))
        dy = [v["estimated_y"] - DRONE_Y for v in got.values()]
        want = expect_sign if frac > 0 else -expect_sign
        check("%s, %s -> y %s of the vehicle"
              % (name, side, "below" if want < 0 else "above"),
              [d < 0 for d in dy], [want < 0])

print("\n3. a frame offered to a full decoder is dropped, not queued")
# The decoder is stopped so the queue can be filled on purpose. There is no
# busy flag to set any more: decoding happens on the camera's own thread, and
# what fills is the queue in front of it. The old flag was set and cleared
# inside one callback on a thread that runs callbacks one at a time, so it was
# never true when anything looked at it, and every run reported zero drops
# while a quarter of the frames were being thrown away at the age limit.
s.HIRES.finish()
s.HIRES.queue.clear()
s.HIRES.dropped = 0
for _ in range(s.DECODE_QUEUE_DEPTH + 1):
    s.on_hires_image(Msg(frame_with_label(RENDER_M, 1024, 768, 60.0)))
check("sheds one once the queue is full", s.HIRES.dropped, 1)
check("and never grows past its depth", len(s.HIRES.queue),
      s.DECODE_QUEUE_DEPTH)
s.HIRES.queue.clear()
s.HIRES.start()

print("\n4. a camera with no face on this lane records nothing")
s.REAR.depth = None
got = place(s.on_rear_image, frame_with_label(RENDER_M, 1280, 800, 90.0))
check("rear ignored when the lane has no far face", got, {})
s.REAR.depth = REAR_DEPTH

print("\n5. a frame that arrives late is thrown away")
s.HIRES.stale = 0
fresh = place(s.on_hires_image, frame_with_label(RENDER_M, 1024, 768, 60.0))
check("a frame taken now is read", len(fresh), 1)
s.inventory.clear()
old = place(s.on_hires_image,
            frame_with_label(RENDER_M, 1024, 768, 60.0),
            taken_at=SIM_NOW - 2 * s.MAX_FRAME_AGE_S)
# Dropping this one is a throughput choice, not a correctness one: the history
# would have placed it correctly. It is dropped so that a decoder slower than
# the camera cannot build a queue that grows all flight.
check("a frame twice the age limit is dropped", old, {})
check("and counted", s.HIRES.stale, 1)

print("\n6. each camera holds its own detector")
check("wechat detectors differ", s.HIRES.detector is not s.REAR.detector, True)
check("fallback detectors differ", s.HIRES.fallback is not s.REAR.fallback, True)

for _cam in (s.HIRES, s.REAR):
    _cam.finish()

print("\n%s" % ("all checks passed" if not failures
                else "%d FAILED: %s" % (len(failures), ", ".join(failures))))
sys.exit(1 if failures else 0)
