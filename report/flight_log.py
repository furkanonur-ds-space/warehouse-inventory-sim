#!/usr/bin/env python3
"""
Record where the vehicle actually was, for the whole flight.

    # in its own terminal, while the simulator runs, alongside scanner.py
    python3 report/flight_log.py
    # file: out/flight_log.csv  (t_s, wall_ms, x, y, z, yaw_deg) - Ctrl-C ends it

PASSIVE BY DESIGN. This reads one Gazebo topic and nothing else. It does not
connect to PX4, does not speak MAVLink, and sends nothing anywhere, so running
it cannot change the flight it is recording. That separation is the whole
point: a logger that perturbs the run produces a log of a different run.

It is also why the estimate is not recorded here. The estimate lives inside
PX4 and getting it would mean opening a MAVLink connection, which PX4 counts
as a ground station - a second one appearing and disappearing mid-flight is a
real disturbance. PX4 logs its own estimate at full rate to
`build/px4_sitl_default/rootfs/log/<date>/<time>.ulg`; that file and this one
are the two halves, and they can be compared afterwards without either side
interfering with the flight.

GROUND TRUTH NEVER ENTERS CONTROL. It is read here to answer "where was the
vehicle really", which is a question only a recording can answer after the
fact. `scanner.py` does not read this file and does not know it exists.

Columns are Gazebo world coordinates, metres: x east, y north, z up.
yaw_deg is measured from world +X, counter-clockwise.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time
from pathlib import Path

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

import gz.transport13 as trans                                   # noqa: E402
from gz.msgs10.pose_v_pb2 import Pose_V                          # noqa: E402

from warehouse_model import REPO_ROOT                            # noqa: E402

LAYOUT_PATH = REPO_ROOT / "scanner" / "layout.json"


def yaw_from_quaternion(q) -> float:
    """Heading about world +Z, degrees counter-clockwise from +X."""
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.degrees(math.atan2(siny, cosy))


def main() -> int:
    layout = json.loads(LAYOUT_PATH.read_text())
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--world", default=layout["world"])
    ap.add_argument("--model", default=layout["model"] + "_0",
                    help="gz model name, as printed by PX4: 'model: ...'")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "out" / "flight_log.csv")
    ap.add_argument("--rate", type=float, default=10.0, help="samples per second")
    ap.add_argument("--quiet", action="store_true",
                    help="write the file without printing live lines")
    args = ap.parse_args()

    topic = f"/world/{args.world}/dynamic_pose/info"
    state = {"pose": None}

    def on_pose(msg):
        for p in msg.pose:
            if p.name == args.model:
                state["pose"] = (p.position.x, p.position.y, p.position.z,
                                 yaw_from_quaternion(p.orientation))
                return

    node = trans.Node()
    if not node.subscribe(Pose_V, topic, on_pose):
        print(f"could not subscribe to {topic}")
        return 1

    args.out.parent.mkdir(parents=True, exist_ok=True)
    period = 1.0 / args.rate
    rows = 0
    t0 = None
    duration = 0.0
    last = None
    distance = 0.0
    z_min, z_max = float("inf"), float("-inf")

    print(f"recording {args.model} in world {args.world}")
    print(f"topic : {topic}")
    print(f"file  : {args.out}")
    print("Ctrl-C to stop\n")

    handle = args.out.open("w", encoding="utf-8")
    handle.write("t_s,wall_ms,x,y,z,yaw_deg\n")
    try:
        while True:
            time.sleep(period)
            pose = state["pose"]
            if pose is None:
                # Nothing has arrived yet. The usual reason is that the model
                # has not spawned, or GZ_IP is unset and discovery is not
                # finding the simulator.
                continue
            x, y, z, yaw = pose
            now = time.time()
            if t0 is None:
                t0 = now
            duration = now - t0
            t = duration
            handle.write(f"{t:.3f},{int(now*1000)},{x:.4f},{y:.4f},{z:.4f},{yaw:.2f}\n")
            rows += 1
            if rows % 20 == 0:
                handle.flush()      # so the file is readable during the flight
            if last is not None:
                distance += math.dist((x, y, z), last)
            last = (x, y, z)
            z_min, z_max = min(z_min, z), max(z_max, z)
            if not args.quiet and rows % 10 == 0:
                print(f"\r{t:7.1f} s  x {x:+7.2f}  y {y:+7.2f}  z {z:5.2f}  "
                      f"yaw {yaw:+7.1f}  {rows} samples", end="", flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        handle.close()

    print()
    if not rows:
        print("\nno samples recorded.")
        print("Is the simulator running, and is GZ_IP=127.0.0.1 set? Without it")
        print("gz-transport discovery is unreliable and topics arrive empty.")
        return 1

    if duration:
        print(f"{rows} samples over {duration:.0f} s "
              f"({rows/duration:.1f} Hz effective)")
    else:
        print(f"{rows} samples")
    print(f"path length : {distance:.1f} m")
    print(f"altitude    : {z_min:.2f} to {z_max:.2f} m")
    print(f"\n{args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
