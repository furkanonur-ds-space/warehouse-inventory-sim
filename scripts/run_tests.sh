#!/bin/bash
#
# Run everything that can be checked without a simulator.
#
#   ./scripts/run_tests.sh
#
# All suites take seconds and need no flight, which is the point: a scan takes
# thirteen minutes and only tells you the total. These say which piece of the
# geometry is wrong.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"
if [ ! -x "$PY" ]; then
    PY="$HERE/.venv/bin/python"
fi
if [ ! -x "$PY" ]; then
    echo "no interpreter; set PY, for example"
    echo "  PY=~/autonomous_landing/venv/bin/python $0"
    exit 1
fi

cd "$HERE/scanner" || exit 1
failed=0

run() {
    echo "== $1"
    out=$("$PY" "$1" 2>&1 | grep -v libprotobuf | grep -v DynamicFactory)
    echo "$out" | tail -2 | sed 's/^/   /'
    echo "$out" | grep -qiE "all checks passed|8/8 passed" \
        || { echo "   !! FAILED"; failed=$((failed + 1)); }
    echo
}

# The pose a code is filed against, which is where a whole scan went wrong
# once: frames were placed against wherever the vehicle had reached by the
# time they were decoded rather than where it was when they were taken.
run test_pose_history.py

# Which face a code lands on and which side of the vehicle. The centre of a
# frame cannot show the second one, since the bearing is zero there.
run test_two_camera.py

# Reading a frame in strips, because the detector loses codes when several
# share one. Checks both halves: that more are found, and that a strip
# coordinate is mapped back to the frame, since the bearing to a box comes
# from where its code sits in the frame.
run test_strips.py

# The marker correction geometry.
run test_drift_correction.py

# Where the optical axis goes, and whether the codes fit the frame at all.
# The vertical half of the geometry, which had never been checked and which
# cost the narrowest aisle two codes a run: its frame is 0.18 m tall and its
# codes sat 0.049 m below the axis.
run test_framing.py

if [ "$failed" -eq 0 ]; then
    echo "all suites passed"
else
    echo "$failed suite(s) failed"
fi
exit "$failed"
