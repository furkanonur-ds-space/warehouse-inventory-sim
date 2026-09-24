#!/bin/bash
#
# The port's only real test: the same layout must give the same waypoints in
# both languages, value for value.
#
#   ./tests/compare_with_python.sh [layout.json]
#
# Exits non-zero on the first difference and prints it, since one waypoint
# out of place is a vehicle flying down the wrong lane.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAYOUT="${1:-$HERE/../scanner/layout.json}"
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"

if [ ! -x "$HERE/build/route_tool" ]; then
    echo "build it first: ./scripts/build.sh"
    exit 1
fi

"$HERE/build/route_tool" "$LAYOUT" > /tmp/route_c.txt
c_status=$?
"$PY" "$HERE/tests/emit_route_python.py" > /tmp/route_py.txt
py_status=$?

if [ $c_status -ne 0 ] || [ $py_status -ne 0 ]; then
    echo "FAIL one of the two did not run (C $c_status, Python $py_status)"
    exit 1
fi

c_lines=$(wc -l < /tmp/route_c.txt)
py_lines=$(wc -l < /tmp/route_py.txt)

if ! diff -u /tmp/route_py.txt /tmp/route_c.txt > /tmp/route_diff.txt; then
    echo "FAIL the two routes differ (Python $py_lines waypoints, C $c_lines)"
    head -20 /tmp/route_diff.txt
    exit 1
fi

echo "PASS both languages give the same $c_lines waypoints"
