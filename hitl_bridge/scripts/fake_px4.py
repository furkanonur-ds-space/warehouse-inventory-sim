#!/usr/bin/env python3
"""Stand in for PX4 so stage 3 can be tested without the drone.

It does what the flight controller does at the socket: listens for
HIL_SENSOR, and answers with HIL_ACTUATOR_CONTROLS at 200 Hz to whichever
address the sensors came from. It does not fly the vehicle; it holds the four
motors at a fixed value, which is enough to tell whether the bridge decodes
the message, scales it and reaches the rotors.

    python3 scripts/fake_px4.py --throttle 0.7 --seconds 6

Throttle 0 leaves the vehicle on the ground. A value near 0.55 roughly
balances this airframe's weight; 0.7 climbs.
"""

import argparse
import sys
import time

from pymavlink import mavutil
from pymavlink.dialects.v20 import common as mavlink2

# VOXL2 sends this for a quad: motors 0 to 3 carry a value.
FLAGS_QUAD = 0x0F


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", type=int, default=14560)
    parser.add_argument("--throttle", type=float, default=0.7)
    parser.add_argument("--seconds", type=float, default=6.0)
    parser.add_argument("--rate", type=float, default=200.0)
    args = parser.parse_args()

    link = mavutil.mavlink_connection(
        "udpin:127.0.0.1:%d" % args.port, dialect="common")
    print("fake px4 listening on udp %d" % args.port)

    controls = [0.0] * 16
    for i in range(4):
        controls[i] = args.throttle

    started = None
    sent = 0
    received = 0
    next_send = 0.0
    deadline = time.time() + args.seconds + 30.0

    while time.time() < deadline:
        msg = link.recv_match(blocking=True, timeout=0.5)
        if msg is not None and msg.get_type() == "HIL_SENSOR":
            received += 1
            if started is None:
                started = time.time()
                next_send = started
                print("sensors arriving, answering with throttle %.2f on "
                      "four motors" % args.throttle)

        if started is None:
            continue

        now = time.time()
        if now >= next_send:
            # mav.srcSystem is left at PX4's own ids; the bridge does not
            # filter on them yet, and the document does not ask it to.
            link.mav.hil_actuator_controls_send(
                int(now * 1e6), controls, 0, FLAGS_QUAD)
            sent += 1
            next_send += 1.0 / args.rate

        if now - started >= args.seconds:
            break

    if started is None:
        print("FAIL no HIL_SENSOR arrived; is the simulator running?")
        return 1

    elapsed = time.time() - started
    print("received %d HIL_SENSOR, sent %d HIL_ACTUATOR_CONTROLS in %.1f s "
          "(%.0f Hz)" % (received, sent, elapsed, sent / elapsed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
