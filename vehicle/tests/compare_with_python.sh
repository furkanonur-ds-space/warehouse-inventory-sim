#!/bin/bash
#
# The port's only real test: the same layout must give the same waypoints in
# both languages, value for value, and the same remarks about them.
#
#   ./tests/compare_with_python.sh [layout.json]
#
# The remarks are compared because the waypoints alone do not cover
# everything the route knows. Where each camera sits on the body, for one,
# reaches only the warning about a band of codes too tall for the frame; a
# version of this test that compared waypoints alone passed with that
# constant a centimetre out.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAYOUT="${1:-$HERE/../scanner/layout.json}"
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"

if [ ! -x "$HERE/build/route_tool" ]; then
    echo "build it first: ./scripts/build.sh"
    exit 1
fi

"$HERE/build/route_tool" "$LAYOUT" > /tmp/route_c.txt 2> /tmp/says_c.txt
c_status=$?
"$PY" "$HERE/tests/emit_route_python.py" > /tmp/route_py.txt 2> /tmp/says_py.txt
py_status=$?

if [ $c_status -ne 0 ] || [ $py_status -ne 0 ]; then
    echo "FAIL one of the two did not run (C $c_status, Python $py_status)"
    exit 1
fi

failed=0

c_lines=$(wc -l < /tmp/route_c.txt)
py_lines=$(wc -l < /tmp/route_py.txt)
if diff -u /tmp/route_py.txt /tmp/route_c.txt > /tmp/route_diff.txt; then
    echo "ok   the same $c_lines waypoints"
else
    echo "FAIL the routes differ (Python $py_lines waypoints, C $c_lines)"
    head -20 /tmp/route_diff.txt
    failed=1
fi

# Sorted, because the two need not decide to mention things in the same
# order to be saying the same things.
sort /tmp/says_c.txt > /tmp/says_c_sorted.txt
sort /tmp/says_py.txt > /tmp/says_py_sorted.txt
says=$(wc -l < /tmp/says_c_sorted.txt)
if diff -u /tmp/says_py_sorted.txt /tmp/says_c_sorted.txt > /tmp/says_diff.txt; then
    echo "ok   the same $says remarks about the route"
else
    echo "FAIL the remarks differ"
    head -20 /tmp/says_diff.txt
    failed=1
fi

# Where each camera ends up, which the waypoints do not carry and this
# warehouse's warnings never exercise.
"$HERE/build/route_tool" --limits "$LAYOUT" > /tmp/limits_c.txt 2> /dev/null
"$PY" "$HERE/tests/emit_route_python.py" --limits > /tmp/limits_py.txt 2> /dev/null
limits=$(wc -l < /tmp/limits_c.txt)
if diff -u /tmp/limits_py.txt /tmp/limits_c.txt > /tmp/limits_diff.txt; then
    echo "ok   the same $limits camera standoffs and frame limits"
else
    echo "FAIL the camera standoffs or frame limits differ"
    head -20 /tmp/limits_diff.txt
    failed=1
fi

# What the vehicle would actually command: the setpoint stream down each
# leg and the yaw sweep through each turn. Both read tests/legs.json, so
# neither is asked about a scenario the other never saw.
LEGS_FILE="$HERE/tests/legs.json"
"$HERE/build/flight_tool" "$LEGS_FILE" > /tmp/flight_c.txt 2> /dev/null
c_status=$?
"$PY" "$HERE/tests/emit_flight_python.py" "$LEGS_FILE" > /tmp/flight_py.txt 2> /dev/null
py_status=$?

if [ $c_status -ne 0 ] || [ $py_status -ne 0 ]; then
    echo "FAIL the flight tools did not run (C $c_status, Python $py_status)"
    failed=1
else
    commands=$(wc -l < /tmp/flight_c.txt)
    if diff -u /tmp/flight_py.txt /tmp/flight_c.txt > /tmp/flight_diff.txt; then
        echo "ok   the same $commands setpoints and yaw steps"
    else
        echo "FAIL the setpoints or yaw steps differ"
        head -20 /tmp/flight_diff.txt
        failed=1
    fi
fi

if [ $failed -ne 0 ]; then
    exit 1
fi
echo "PASS both languages give the same route, fly it the same way, and say the same about it"
