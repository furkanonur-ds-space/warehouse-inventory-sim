#!/usr/bin/env python3
"""
The leg arithmetic, against the formulas it was moved out of.

flight.py was split out of goto_waypoint and hold_heading so that the C port
can be checked against it without flying. A split is the easiest kind of
change to get wrong quietly: the vehicle still flies, just a little
differently, and nothing says so until codes go missing.

So this file does not call flight.py twice and compare it with itself. It
carries its own copy of the formulas as they read before the split, written
out from the old source, and asserts the two agree. If someone changes how a
leg is flown on purpose, this suite is meant to fail and to be updated
deliberately.
"""

import math
import sys

import flight

FAILED = 0


def check(name, got, want, tolerance=1e-12):
    global FAILED
    if isinstance(want, (int, float)):
        ok = abs(got - want) <= tolerance
    else:
        ok = got == want
    if not ok:
        print("   FAIL %s: got %r, expected %r" % (name, got, want))
        FAILED += 1
    return ok


# --- the formulas as they read before the split ------------------------

def old_leg(start, target):
    start_n, start_e, start_d = start
    target_n, target_e, target_d = target
    leg_length = math.sqrt((target_n - start_n) ** 2 +
                           (target_e - start_e) ** 2 +
                           (target_d - start_d) ** 2)
    vertical_span = abs(target_d - start_d)
    horizontal_span = math.sqrt((target_n - start_n) ** 2 +
                                (target_e - start_e) ** 2)
    is_climb = vertical_span > horizontal_span
    speed = 0.15 if is_climb else 1.0
    return leg_length, is_climb, speed, leg_length / speed + 20.0


def old_setpoints(start, target, dt=0.1):
    leg_length, _is_climb, speed, _max_time = old_leg(start, target)
    start_n, start_e, start_d = start
    target_n, target_e, target_d = target
    points = []
    travelled = 0.0
    while True:
        travelled = min(leg_length, travelled + speed * dt)
        fraction = 1.0 if leg_length == 0 else travelled / leg_length
        points.append((start_n + (target_n - start_n) * fraction,
                       start_e + (target_e - start_e) * fraction,
                       start_d + (target_d - start_d) * fraction,
                       fraction))
        if travelled >= leg_length:
            return points


def old_sweep(start_yaw, yaw_deg, dt=0.1):
    delta = (yaw_deg - start_yaw + 180) % 360 - 180
    duration = abs(delta) / 30.0
    if duration < 0.1:
        duration = 0.1
    steps = int(duration / dt)
    return [start_yaw + delta * ((i + 1) / steps) for i in range(steps)]


# --- the legs worth checking -------------------------------------------
#
# A cruise down an aisle, a level change, a lane change that also steps up,
# a leg of no length at all, and a leg short enough to finish inside one
# tick. The last two are where an off-by-one or a division by zero lives.
LEGS = [
    ((0.0, 0.0, -0.75), (18.0, 0.0, -0.75)),
    ((0.0, 0.0, -0.75), (0.0, 0.0, -2.45)),
    ((0.0, 0.0, -0.75), (0.0, 4.26, -2.45)),
    ((1.0, 2.0, -3.0), (1.0, 2.0, -3.0)),
    ((0.0, 0.0, -0.75), (0.05, 0.0, -0.75)),
    ((-8.4, -9.0, -0.696), (-8.4, 9.0, -0.696)),
]

TURNS = [
    (-90.0, 90.0),     # the half turn at the end of an aisle
    (90.0, -90.0),
    (0.0, 0.0),        # no turn at all
    (170.0, -170.0),   # the short way round is 20 degrees, not 340
    (-179.0, 179.0),
    (0.0, 0.4),        # smaller than one tick
]


def main():
    print("== leg plans")
    for start, target in LEGS:
        plan = flight.leg_plan(start, target)
        length, is_climb, speed, max_time = old_leg(start, target)
        name = "%s -> %s" % (start, target)
        check("%s length" % name, plan["length"], length)
        check("%s is_climb" % name, plan["is_climb"], is_climb)
        check("%s speed" % name, plan["speed"], speed)
        check("%s max_time" % name, plan["max_time"], max_time)
    print("   %d legs" % len(LEGS))

    print("== setpoint streams")
    total = 0
    for start, target in LEGS:
        got = flight.leg_setpoints(start, target)
        want = old_setpoints(start, target)
        name = "%s -> %s" % (start, target)
        if not check("%s point count" % name, len(got), len(want)):
            continue
        total += len(got)
        for i, (a, b) in enumerate(zip(got, want)):
            for axis in range(4):
                check("%s point %d axis %d" % (name, i, axis),
                      a[axis], b[axis])
    print("   %d setpoints" % total)

    print("== heading sweeps")
    steps = 0
    for start_yaw, target_yaw in TURNS:
        got = flight.heading_sweep(start_yaw, target_yaw)
        want = old_sweep(start_yaw, target_yaw)
        name = "%.0f -> %.0f" % (start_yaw, target_yaw)
        if not check("%s step count" % name, len(got), len(want)):
            continue
        steps += len(got)
        for i, (a, b) in enumerate(zip(got, want)):
            check("%s step %d" % (name, i), a, b)
        if got:
            check("%s ends on target" % name,
                  (got[-1] - target_yaw + 180) % 360 - 180, 0.0, 1e-9)
    print("   %d yaw steps" % steps)

    print()
    if FAILED:
        print("%d checks failed" % FAILED)
        return 1
    print("all checks passed")
    return 0


if __name__ == "__main__":
    sys.exit(main())
