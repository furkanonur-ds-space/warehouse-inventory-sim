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


def main():
    # build_route narrates what it chose on stdout, and the C tool sends the
    # same remarks to stderr. Send Python's there too, so the comparison is
    # of waypoints and not of commentary.
    with contextlib.redirect_stdout(sys.stderr):
        waypoints = route.build_route()

    for x, y, z, yaw, hires, rear in waypoints:
        tail = "none" if rear is None else "%.6f" % rear
        print("%.6f %.6f %.6f %.6f %.6f %s" % (x, y, z, yaw, hires, tail))
    return 0


if __name__ == "__main__":
    sys.exit(main())
