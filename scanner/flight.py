"""
What to command, and when, to fly one leg of the route.

Split out of scanner.py for the same reason route.py was: this is the other
half that has to run on the vehicle, and on the vehicle it will be C.

Only the arithmetic lives here. Nothing in this file knows whether the
vehicle arrived, where a marker says it really is, or whether the altitude
settled; those are answers that come back from the aircraft, and they stay
in scanner.py with the autopilot connection. What is here is the part that
can be worked out in advance and therefore checked against the port without
flying: how fast this leg is flown, how long it should take, where the
setpoint sits at each tick, and how a heading change is swept.

The division matters for more than tidiness. A setpoint stream that agrees
between two languages is something a test can assert in milliseconds; a
settle loop is only honest in a real flight, which costs seventeen minutes.
"""

import math

# --- HOW A LEG IS FLOWN ------------------------------------------------
#
# One speed in every aisle, fixed at 1 m/s by a decision above this work.
# See route.py and the commits for why slowing down was tried and was not
# the answer to the codes that were being missed.
CRUISE_SPEED = 1.0             # m/s along an aisle
CLIMB_SPEED = 0.15             # m/s when changing shelf level

# Vertical moves need their own, much slower rate. Measured with the
# horizontal rate applied to both: a 0.65 m level change left the vehicle
# 0.65 m behind the setpoint, and the error was still 0.35 m part way along
# the next aisle. The camera only tolerates 0.37 m of vertical error before a
# code leaves the frame, so this alone accounted for most of the missed
# reads.

SETPOINT_DT = 0.1              # seconds between setpoints
TIMEOUT_MARGIN = 20.0          # seconds of slack added to expected leg time

# A sudden 180 degree yaw setpoint makes the vehicle spin at its maximum
# rate, which blurs the tracking cameras and destroys VIO. The target is
# swept instead, slowly enough that the features stay trackable.
YAW_SWEEP_RATE = 30.0          # degrees a second


def leg_plan(start, target):
    """
    How this leg is flown: its length, whether it is a climb, and how long
    it may take before something is wrong.

    A leg that is mostly vertical is a level change and is flown at the
    climb speed. Deciding by which span is larger rather than by whether the
    altitude changed at all keeps a lane change that also steps up a level
    from crawling the whole way across the warehouse.
    """
    start_n, start_e, start_d = start
    target_n, target_e, target_d = target

    vertical_span = abs(target_d - start_d)
    horizontal_span = math.sqrt((target_n - start_n) ** 2 +
                                (target_e - start_e) ** 2)
    length = math.sqrt(horizontal_span ** 2 + vertical_span ** 2)
    is_climb = vertical_span > horizontal_span
    speed = CLIMB_SPEED if is_climb else CRUISE_SPEED

    return {
        "length": length,
        "is_climb": is_climb,
        "speed": speed,
        "max_time": length / speed + TIMEOUT_MARGIN,
    }


def leg_setpoints(start, target, dt=SETPOINT_DT):
    """
    Every setpoint this leg commands, in order, until it reaches the end.

    The setpoint advances on a timer rather than waiting for the vehicle to
    arrive. A tolerance-gated stepper produces stop-start motion, and the
    camera needs each box to cross a steady sequence of frames.

    What comes back is the ideal path only: scanner.py still shifts each one
    by whatever the markers say the drift is, and still holds at the end
    until the vehicle has caught up.
    """
    plan = leg_plan(start, target)
    length = plan["length"]
    speed = plan["speed"]

    start_n, start_e, start_d = start
    target_n, target_e, target_d = target

    points = []
    travelled = 0.0
    while True:
        travelled = min(length, travelled + speed * dt)
        fraction = 1.0 if length == 0 else travelled / length
        points.append((start_n + (target_n - start_n) * fraction,
                       start_e + (target_e - start_e) * fraction,
                       start_d + (target_d - start_d) * fraction,
                       fraction))
        if travelled >= length:
            return points


def heading_sweep(start_yaw, target_yaw, dt=SETPOINT_DT,
                  rate=YAW_SWEEP_RATE):
    """
    The yaw targets that carry the vehicle from one heading to another.

    Turning the short way round: a change from -90 to +90 is 180 degrees
    whichever way it is written, but from +170 to -170 it is 20 and not 340.
    """
    delta = (target_yaw - start_yaw + 180.0) % 360.0 - 180.0
    duration = abs(delta) / rate
    if duration < 0.1:
        duration = 0.1
    steps = int(duration / dt)

    return [start_yaw + delta * ((i + 1) / steps) for i in range(steps)]
