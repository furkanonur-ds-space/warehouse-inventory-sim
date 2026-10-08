#!/usr/bin/env python3
"""Print the Python route in the same format the C tool prints.

The comparison is the whole point of this file, so it formats rather than
interprets: same field order, same six decimal places, same word for a lane
with nothing behind it.
"""

import contextlib
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scanner"))

import route  # noqa: E402


def print_limits(waypoints):
    """Where each camera ends up and how much shelf it sees from there.

    The same as the C tool's --limits. It exists because the waypoints do
    not carry the mount offsets or the lens: those reach only a warning that
    this warehouse never triggers, so a centimetre of error in them passed
    every check there was.
    """
    last_x = None
    for x, _y, _z, _yaw, hires, rear in waypoints:
        if x == last_x:
            continue
        last_x = x
        depth = hires - route.HIRES_MOUNT_X
        print("lane %.3f hires depth %.6f limit %.6f"
              % (x, depth,
                 route.half_frame_m(route.CAMERA_HFOV_DEG,
                                    route.HIRES_FRAME_PX, depth)))
        if rear is None:
            continue
        depth = rear + route.REAR_MOUNT_X
        print("lane %.3f rear  depth %.6f limit %.6f"
              % (x, depth,
                 route.half_frame_m(route.TRACKING_HFOV_DEG,
                                    route.REAR_FRAME_PX, depth)))


def main():
    # build_route narrates what it chose on stdout, and the C tool sends the
    # same remarks to stderr. Send Python's there too, so the comparison is
    # of waypoints and not of commentary.
    with contextlib.redirect_stdout(sys.stderr):
        waypoints = route.build_route()

    if "--limits" in sys.argv[1:]:
        print_limits(waypoints)
        return 0

    for x, y, z, yaw, hires, rear in waypoints:
        tail = "none" if rear is None else "%.6f" % rear
        print("%.6f %.6f %.6f %.6f %.6f %s" % (x, y, z, yaw, hires, tail))
    return 0


if __name__ == "__main__":
    sys.exit(main())
