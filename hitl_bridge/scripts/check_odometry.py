#!/usr/bin/env python3
"""Check the ODOMETRY stream the VIO path carries.

Frame mistakes are the expensive kind here: a sign error in z or in the
quaternion produces a vehicle that is convinced it is upside down or
underground, and on the aircraft that is voxl-vision-hub flying on it. So
this checks the values against what the vehicle is actually doing, not just
that messages arrive.

    python3 scripts/check_odometry.py --expect rest
    python3 scripts/check_odometry.py --expect climb

Run it before the simulator; it binds the port and waits.
"""

import argparse
import math
import sys
import time

from pymavlink import mavutil

MAV_FRAME_LOCAL_NED = 1
MAV_FRAME_BODY_FRD = 12

EXPECTED_HZ = 250.0
RATE_TOLERANCE = 0.1

# The model is spawned 0.25 m above the ground, level. In NED, up is
# negative, so a correct z is about -0.25 and never positive while flying.
REST_Z = -0.25
REST_Z_TOL = 0.1


def euler_from_quaternion(q):
    """Roll, pitch, yaw in radians from (w, x, y, z)."""
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sin_pitch)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=14570)
    parser.add_argument("--seconds", type=float, default=4.0)
    parser.add_argument("--expect", choices=["rest", "climb"], default="rest")
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        "udpin:127.0.0.1:%d" % args.port, dialect="common")
    print("listening for ODOMETRY on udp %d" % args.port)

    samples = []
    started = None
    deadline = time.time() + args.seconds + 30.0

    while time.time() < deadline:
        msg = link.recv_match(type="ODOMETRY", blocking=True, timeout=1.0)
        if msg is None:
            if started is not None:
                break
            continue
        if started is None:
            started = time.time()
        samples.append(msg)
        if time.time() - started >= args.seconds:
            break

    if not samples:
        print("FAIL no ODOMETRY arrived on port %d" % args.port)
        return 1

    span_s = (samples[-1].time_usec - samples[0].time_usec) / 1e6
    if span_s <= 0:
        print("FAIL timestamps did not advance")
        return 1

    failures = []
    hz = len(samples) / span_s
    ok = abs(hz - EXPECTED_HZ) <= EXPECTED_HZ * RATE_TOLERANCE
    print()
    print("ODOMETRY %.1f Hz over %.2f s of simulated time   expected %.0f   %s"
          % (hz, span_s, EXPECTED_HZ, "ok" if ok else "FAIL"))
    if not ok:
        failures.append("rate %.1f Hz" % hz)

    first, last = samples[0], samples[-1]

    if first.frame_id != MAV_FRAME_LOCAL_NED:
        failures.append("frame_id %d, expected %d (LOCAL_NED)"
                        % (first.frame_id, MAV_FRAME_LOCAL_NED))
    if first.child_frame_id != MAV_FRAME_BODY_FRD:
        failures.append("child_frame_id %d, expected %d (BODY_FRD)"
                        % (first.child_frame_id, MAV_FRAME_BODY_FRD))
    print("frames: %d / %d (LOCAL_NED / BODY_FRD)"
          % (first.frame_id, first.child_frame_id))

    print("first sample: x %.3f  y %.3f  z %.3f   q %.3f %.3f %.3f %.3f"
          % (first.x, first.y, first.z,
             first.q[0], first.q[1], first.q[2], first.q[3]))
    print("last  sample: x %.3f  y %.3f  z %.3f   vz %.3f"
          % (last.x, last.y, last.z, last.vz))

    # Level at the start. Only roll and pitch are checked, not heading: a
    # model sitting with its nose along Gazebo's +x is pointing east, and
    # east in a north-referenced frame is a 90 degree heading. PX4's own
    # GZBridge::rotateQuaternion produces the same, so a quaternion of
    # (0.707, 0, 0, 0.707) here is right rather than wrong, and an earlier
    # version of this check called it a failure.
    roll, pitch, yaw = euler_from_quaternion(first.q)
    print("attitude at rest: roll %.1f deg, pitch %.1f deg, heading %.1f deg"
          % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw)))
    if abs(math.degrees(roll)) > 3.0:
        failures.append("rolled %.1f deg at rest" % math.degrees(roll))
    if abs(math.degrees(pitch)) > 3.0:
        failures.append("pitched %.1f deg at rest" % math.degrees(pitch))
    if abs(first.z - REST_Z) > REST_Z_TOL:
        failures.append("starting z %.3f, expected about %.2f (NED: up is "
                        "negative)" % (first.z, REST_Z))

    if args.expect == "rest":
        if abs(last.z - first.z) > 0.1:
            failures.append("z moved %.3f m while meant to be at rest"
                            % (last.z - first.z))
    else:
        climbed = first.z - last.z  # positive when going up in NED
        print("climbed %.2f m (z went %.2f to %.2f)"
              % (climbed, first.z, last.z))
        if climbed < 1.0:
            failures.append("did not climb: %.2f m" % climbed)
        if last.vz > -0.5:
            failures.append("climbing but vz is %.2f; in FRD a climb is "
                            "negative" % last.vz)

    print()
    if failures:
        print("FAIL: " + "; ".join(failures))
        return 1
    print("PASS: the odometry says what the vehicle is doing")
    return 0


if __name__ == "__main__":
    sys.exit(main())
