#!/usr/bin/env python3
"""Print the Python setpoints and yaw sweeps in the C tool's format.

Both read the same legs.json, so neither holds a list of its own: a
scenario that only one of them flies proves nothing about the other.
"""

import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "..", "scanner"))

import flight  # noqa: E402


def main():
    if len(sys.argv) != 2:
        print("usage: %s <legs.json>" % sys.argv[0], file=sys.stderr)
        return 2

    with open(sys.argv[1], encoding="utf-8") as handle:
        scenarios = json.load(handle)

    for index, leg in enumerate(scenarios["legs"]):
        start = tuple(leg["start"])
        target = tuple(leg["target"])
        plan = flight.leg_plan(start, target)
        print("leg %d length %.9f climb %d speed %.9f max_time %.9f"
              % (index, plan["length"], 1 if plan["is_climb"] else 0,
                 plan["speed"], plan["max_time"]))
        points = flight.leg_setpoints(start, target)
        print("leg %d points %d" % (index, len(points)))
        for i, (n, e, d, fraction) in enumerate(points):
            print("leg %d point %d %.9f %.9f %.9f %.9f"
                  % (index, i, n, e, d, fraction))

    for index, turn in enumerate(scenarios["turns"]):
        steps = flight.heading_sweep(turn["from"], turn["to"])
        print("turn %d steps %d" % (index, len(steps)))
        for i, yaw in enumerate(steps):
            print("turn %d step %d %.9f" % (index, i, yaw))

    return 0


if __name__ == "__main__":
    sys.exit(main())
