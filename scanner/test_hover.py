"""
Take off, hold still, and say whether the airframe is flyable.

A scan takes thirteen minutes and answers one question at the end of it. This
takes about a minute and answers the question that has to come first: does the
vehicle hold an attitude and a position, or does it fight itself.

Written for changing airframe. The Starling 2 is a seventh of the x500's mass
with half its arm, so the same demand produces a very different response, and
the first attempt at flying it was abandoned because nobody had measured what
it was doing.

    python3 test_hover.py

Reports, over a twenty second hold at 1.5 m:

  attitude    how far roll and pitch wander from level. A tuned vehicle sits
              within a degree or so. Several degrees, oscillating, means the
              rate loop is too stiff for the airframe.
  position    how far it drifts from where it was put. This is the number the
              scan cares about: the rear camera has about 50 mm of margin at
              the distance it flies.
  altitude    whether it holds height, and what throttle it takes to.
"""
import asyncio
import math
import os
import statistics
import sys
import time

os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"

from mavsdk import System
from mavsdk.offboard import PositionNedYaw, OffboardError

HOLD_S = 20.0
CLIMB_S = 8.0
HEIGHT_M = 1.5

samples = []          # (t, roll, pitch, yaw, n, e, d)


async def collect(drone, stop_at):
    async for o in drone.telemetry.odometry():
        q = o.q
        roll = math.degrees(math.atan2(
            2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))
        pitch = math.degrees(math.asin(
            max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        yaw = math.degrees(math.atan2(
            2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        samples.append((time.monotonic(), roll, pitch, yaw,
                        o.position_body.x_m, o.position_body.y_m,
                        o.position_body.z_m))
        if time.monotonic() > stop_at:
            return


def spread(values):
    return max(values) - min(values)


async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("connecting")
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    print("waiting for a position estimate")
    async for health in drone.telemetry.health():
        if health.is_local_position_ok:
            break

    start = None
    async for p in drone.telemetry.position_velocity_ned():
        start = (p.position.north_m, p.position.east_m, p.position.down_m)
        break
    print("standing at n %.2f e %.2f d %.2f" % start)

    await drone.offboard.set_position_ned(
        PositionNedYaw(start[0], start[1], start[2], 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as err:
        print("offboard refused: %s" % err)
        return 1
    await drone.action.arm()

    print("climbing to %.1f m over %.0f s" % (HEIGHT_M, CLIMB_S))
    steps = int(CLIMB_S / 0.1)
    for i in range(steps):
        d = start[2] - HEIGHT_M * (i + 1) / steps
        await drone.offboard.set_position_ned(
            PositionNedYaw(start[0], start[1], d, 0.0))
        await asyncio.sleep(0.1)

    target = PositionNedYaw(start[0], start[1], start[2] - HEIGHT_M, 0.0)
    print("holding for %.0f s" % HOLD_S)
    collector = asyncio.create_task(collect(drone, time.monotonic() + HOLD_S))
    stop = time.monotonic() + HOLD_S
    while time.monotonic() < stop:
        await drone.offboard.set_position_ned(target)
        await asyncio.sleep(0.1)
    await collector

    print("landing")
    try:
        await drone.action.land()
    except Exception:
        pass

    if len(samples) < 20:
        print("only %d samples; something went wrong" % len(samples))
        return 1

    # Ignore the first two seconds of the hold, which is still settling.
    t0 = samples[0][0]
    held = [s for s in samples if s[0] - t0 > 2.0] or samples

    roll = [s[1] for s in held]
    pitch = [s[2] for s in held]
    north = [s[4] for s in held]
    east = [s[5] for s in held]
    down = [s[6] for s in held]

    print("\n%d samples over %.1f s of hold" % (len(held), held[-1][0] - held[0][0]))
    print("\n  attitude")
    print("    roll   mean %+.2f deg   spread %.2f   worst %+.2f"
          % (statistics.mean(roll), spread(roll), max(roll, key=abs)))
    print("    pitch  mean %+.2f deg   spread %.2f   worst %+.2f"
          % (statistics.mean(pitch), spread(pitch), max(pitch, key=abs)))

    print("\n  position, against where it was asked to stay")
    dn = [n - target.north_m for n in north]
    de = [e - target.east_m for e in east]
    dd = [d - target.down_m for d in down]
    radial = [math.sqrt(a * a + b * b) for a, b in zip(dn, de)]
    print("    lateral  median %.3f m   worst %.3f m" % (statistics.median(radial),
                                                         max(radial)))
    print("    height   median %+.3f m   spread %.3f m"
          % (statistics.median(dd), spread(dd)))

    print("\n  verdict")
    bad = []
    if spread(roll) > 4 or spread(pitch) > 4:
        bad.append("attitude is wandering by more than 4 degrees; the rate "
                   "loop is likely too stiff")
    if max(radial) > 0.30:
        bad.append("drifts more than 0.30 m from the hold point, which is "
                   "wider than the rear camera's margin")
    if spread(dd) > 0.20:
        bad.append("altitude wanders by more than 0.20 m")
    if bad:
        for line in bad:
            print("    %s" % line)
        return 1
    print("    holds attitude, position and height. Worth flying a scan.")
    return 0


sys.exit(asyncio.run(main()))
