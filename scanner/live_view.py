"""
Live camera viewer with a decode overlay.

Opens a window showing the scanning camera, marks any QR code it finds, and
prints the vehicle's altitude alongside the altitude the current shelf level
expects. Useful for seeing whether a missed code was out of frame, out of
focus, or simply never in view.

Run it in a second terminal while a mission is flying.

    python3 live_view.py                  # scanning camera
    python3 live_view.py --cam down       # downward tracking camera
    python3 live_view.py --save frames/   # also write annotated frames

Press q in the window to quit.
"""
import os
os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

import argparse
import asyncio
import math
import threading
import time

import cv2
import numpy as np
import gz.transport13 as trans
from gz.msgs10.image_pb2 import Image
from mavsdk import System

# Read from the same layout the scanner flies, so the two cannot drift apart.
# Hardcoding a world and model name here meant this quietly watched a topic
# that no longer existed after the vehicle was renamed.
import json
_HERE = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(_HERE, "layout.json"), encoding="utf-8") as _h:
    LAYOUT = json.load(_h)

WORLD = LAYOUT["world"]
DRONE = LAYOUT["model"] + "_0"
CAMERAS = {
    "hires": f"/world/{WORLD}/model/{DRONE}/link/camera_hires_link/sensor/camera/image",
    "down":  f"/world/{WORLD}/model/{DRONE}/link/camera_track_down_link/sensor/camera/image",
    "front": f"/world/{WORLD}/model/{DRONE}/link/camera_track_front_link/sensor/camera/image",
    "rear":  f"/world/{WORLD}/model/{DRONE}/link/camera_track_rear_link/sensor/camera/image",
}

# Shelf levels, for the altitude readout
LEVEL_ALTITUDES = {"L%d" % (i + 1): z
                   for i, z in enumerate(LAYOUT["flight_z"])}
SPAWN_X, SPAWN_Y = LAYOUT["spawn_x"], LAYOUT["spawn_y"]
GROUND_OFFSET = LAYOUT["ground_offset"]

latest = {"frame": None}
telemetry = {"n": 0.0, "e": 0.0, "d": 0.0, "yaw": 0.0, "connected": False}
qr_detector = cv2.QRCodeDetector()


def on_image(msg):
    try:
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            (msg.height, msg.width, 3))
        latest["frame"] = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    except Exception:
        pass


async def telemetry_loop():
    """Read position and heading in the background."""
    drone = System()
    try:
        await drone.connect(system_address="udp://:14541")
    except Exception:
        return

    async for state in drone.core.connection_state():
        if state.is_connected:
            telemetry["connected"] = True
            break

    async def positions():
        async for odom in drone.telemetry.position_velocity_ned():
            telemetry["n"] = odom.position.north_m
            telemetry["e"] = odom.position.east_m
            telemetry["d"] = odom.position.down_m

    async def attitude():
        async for att in drone.telemetry.attitude_euler():
            telemetry["yaw"] = att.yaw_deg

    await asyncio.gather(positions(), attitude())


def start_telemetry_thread():
    def runner():
        try:
            asyncio.run(telemetry_loop())
        except Exception:
            pass
    t = threading.Thread(target=runner, daemon=True)
    t.start()


def annotate(frame):
    """Mark detected codes and overlay the altitude readout."""
    out = frame.copy()
    h, w = out.shape[:2]

    # Centre cross, so it is obvious whether a code sits on the optical axis
    cv2.line(out, (w//2 - 15, h//2), (w//2 + 15, h//2), (0, 200, 255), 1)
    cv2.line(out, (w//2, h//2 - 15), (w//2, h//2 + 15), (0, 200, 255), 1)

    decoded = []
    try:
        ok, points = qr_detector.detectMulti(out)
        if not ok or points is None:
            ok, points = qr_detector.detect(out)
        if ok and points is not None:
            for quad in points:
                p = quad.astype(int)
                cv2.polylines(out, [p], True, (0, 255, 0), 2)

                x1 = max(0, p[:, 0].min() - 8)
                y1 = max(0, p[:, 1].min() - 8)
                x2 = min(w, p[:, 0].max() + 8)
                y2 = min(h, p[:, 1].max() + 8)
                crop = frame[y1:y2, x1:x2]
                if crop.size == 0:
                    continue
                gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                _, binary = cv2.threshold(
                    gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
                data, _, _ = qr_detector.detectAndDecode(binary)
                if not data:
                    big = cv2.resize(binary, None, fx=3.0, fy=3.0,
                                     interpolation=cv2.INTER_CUBIC)
                    data, _, _ = qr_detector.detectAndDecode(big)

                label = data if data else "detected, not decoded"
                colour = (0, 255, 0) if data else (0, 165, 255)
                cv2.putText(out, label, (x1, max(14, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1)
                if data:
                    decoded.append(data)
    except Exception:
        pass

    # Telemetry overlay.
    #
    # GROUND_OFFSET is added for the same reason the scanner adds it: NED
    # heights are measured from an origin captured on the ground, where
    # base_link already sits that far up, and flight_z are world heights.
    # Without it the readout disagrees with the altitude actually commanded.
    alt = -telemetry["d"] + GROUND_OFFSET
    gx = telemetry["e"] + SPAWN_X
    gy = telemetry["n"] + SPAWN_Y

    nearest = min(LEVEL_ALTITUDES.items(), key=lambda kv: abs(kv[1] - alt))
    level_name, level_alt = nearest
    delta = alt - level_alt

    lines = [
        f"x {gx:+6.2f}  y {gy:+6.2f}  alt {alt:5.2f} m  yaw {telemetry['yaw']:+6.1f}",
        f"nearest {level_name} at {level_alt:.2f} m, off by {delta:+.2f} m",
    ]
    if not telemetry["connected"]:
        lines.append("telemetry not connected")

    for i, line in enumerate(lines):
        y = 20 + i * 20
        cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 0, 0), 3)
        cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (255, 255, 255), 1)

    # A band marking how far off centre a code can be and still fit
    band = int(h * 0.42)
    cv2.line(out, (0, h//2 - band), (w, h//2 - band), (80, 80, 80), 1)
    cv2.line(out, (0, h//2 + band), (w, h//2 + band), (80, 80, 80), 1)

    return out, decoded


def main(cam, save_dir):
    topic = CAMERAS[cam]
    node = trans.Node()
    if not node.subscribe(Image, topic, on_image):
        print(f"[ERROR] could not subscribe to {topic}")
        return
    print(f"[INFO] Viewing {cam} camera. Press q to quit.")

    start_telemetry_thread()

    if save_dir:
        os.makedirs(save_dir, exist_ok=True)

    seen = set()
    frame_index = 0
    while True:
        frame = latest["frame"]
        if frame is None:
            time.sleep(0.05)
            continue

        annotated, decoded = annotate(frame)
        for d in decoded:
            if d not in seen:
                seen.add(d)
                print(f"  decoded {d}   (total {len(seen)})")

        cv2.imshow(f"{cam} camera", annotated)

        if save_dir and frame_index % 10 == 0:
            cv2.imwrite(os.path.join(save_dir, f"{frame_index:05d}.png"),
                        annotated)
        frame_index += 1

        if cv2.waitKey(30) & 0xFF == ord("q"):
            break

    cv2.destroyAllWindows()
    print(f"\n[INFO] {len(seen)} distinct codes seen during this session")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--cam", choices=list(CAMERAS), default="hires")
    parser.add_argument("--save", default=None,
                        help="directory to write annotated frames into")
    args = parser.parse_args()
    main(args.cam, args.save)
