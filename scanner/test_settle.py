#!/usr/bin/env python3
"""
When a waypoint counts as reached.

On 5 November 2026 the settle test passed 0.3 s after the move into the
0.50 m aisle, on position alone, while the vehicle was still swinging. It
began the next leg 25 degrees off its heading and 15 cm off its lane and
lost the codes it passed in that state. The settle test now asks for the
vehicle to be still and on its heading too. These cases are taken from that
flight's log (out/flight_log.csv, the start of leg 20).

No simulator.
"""

import sys

import scanner as ws

results = []


def case(name, want, alt_err=0.0, lateral=0.0, speed=0.0, yaw_err=0.0):
    got = ws.waypoint_settled(alt_err, lateral, speed, yaw_err)
    ok = got == want
    results.append(ok)
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}: "
          f"{'settled' if got else 'not settled'}"
          f"{'' if ok else ' (wrong)'}")


print("=" * 68)
print("  WAYPOINT SETTLE TEST")
print("=" * 68)

case("on target, still, on heading", True)
case("on target, 1.9 deg off heading", True, yaw_err=1.9)
case("heading across the wrap, 359 vs 0 deg", True, yaw_err=359.0)
case("heading across the wrap, -359 deg", True, yaw_err=-359.0)

# The 5 November state: in position, but turning and moving.
case("25 deg off heading (5 Nov, start of leg 20)", False, yaw_err=25.0)
case("still swinging, 0.3 m/s", False, speed=0.3)
case("drifting, 0.06 m/s", False, speed=0.06)
case("3 deg off heading", False, yaw_err=3.0)

# Position still matters.
case("6 cm low", False, alt_err=0.06)
case("9 cm off the lane", False, lateral=0.09)

print()
passed = sum(results)
print(f"  {passed}/{len(results)} passed")
if passed != len(results):
    sys.exit(1)
