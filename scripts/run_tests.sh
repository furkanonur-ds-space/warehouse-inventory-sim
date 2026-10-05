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

# Whether a suite passed is its exit status, not a phrase in its output. The
# phrase was "all checks passed|8/8 passed", which stopped being true the day
# a suite gained a ninth check, and would also have called a suite that
# printed the phrase and then crashed a pass.
run() {
    echo "== $1"
    out=$("$PY" "$1" 2>&1)
    status=$?
    printf '%s\n' "$out" | grep -v libprotobuf | grep -v DynamicFactory \
        | tail -2 | sed 's/^/   /'
    if [ "$status" -ne 0 ]; then
        echo "   !! FAILED (exit $status)"
        failed=$((failed + 1))
    fi
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
run test_decode_scale.py

# The report layer has geometry of its own now: a barcode reading carries a
# pose and a pixel position, and turning those into a box is the scanner's
# arithmetic written against a different yaw convention. A sign error there
# puts every code on the wrong side of the aisle, and the shelf snap hides it.
cd "$HERE/report" || exit 1
run test_barcode_inventory.py
# The carton warning, by subtraction: one carton's codes are removed from a
# copy of the last run and the warning has to name that carton and no other.
# Skips itself, rather than failing, on a checkout that has never flown with
# SAVE_BOXES=1 and so has no box log to work from.
run test_box_inventory.py
# The stressed-world report, over an invented run whose answer is known.
run test_stress_report.py
# Old labels: a QR naming another place, a barcode beaten by a nearer one
# under the same QR. Both rules truth-free; the test builds the old labels.
run test_stale_labels.py
# The light report, over an invented run that reads only the brighter labels.
run test_light_report.py

# A barcode on a label stuck on crooked: read off a levelled copy round its
# QR, and filed back in the frame's own pixels.
cd "$HERE/perception" || exit 1
run test_level.py
# Raw frames: every Nth, untouched, indexed, for --replay.
run test_raw_frames.py

# The stressed world itself: no carton inside another, in the rack, on its
# deck, out of the vehicle's way, labels on its face, and the clean world
# unchanged when it is off.
cd "$HERE/warehouse" || exit 1
run test_stress.py
# The dimmed world: dark lamps gone, the rest turned down, and the light
# written into ground truth equal to the light the SDF's own lamps give.
run test_lights.py
# The shadowed world: chosen lamps cast shadows, and the lamps each label
# cannot see, found again from the SDF's own boxes by a different method.
run test_shadows.py

if [ "$failed" -eq 0 ]; then
    echo "all suites passed"
else
    echo "$failed suite(s) failed"
fi
exit "$failed"
