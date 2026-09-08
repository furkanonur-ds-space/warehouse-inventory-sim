#!/bin/bash
#
# Put the vehicle into a drift test, and take it back out again.
#
#   ./scripts/drift_test.sh status      which state the installed model is in
#   ./scripts/drift_test.sh on          wire it for a drift test
#   ./scripts/drift_test.sh off         wire it back for an ordinary flight
#
# Why this exists. build_c27_drone.py --drift moves the odometry publisher onto
# a private topic, so PX4 gets nothing on the one it reads and the relay has to
# supply it. That is deliberate: a harness the vehicle depends on cannot be
# forgotten halfway through a run. But it is written into the installed model
# and nothing on disk says so afterwards. Build with --drift on a Friday,
# come back on Monday, fly, and the vehicle sits on the ground with no position
# source and no message explaining why. That happened often enough to be worth
# a script.
#
# So: this is the only way the flag should be set, it says out loud which state
# you are in, and launch_sim.sh refuses to start a drift wired vehicle unless
# you asked for one.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"
[ -x "$PY" ] || PY="$HERE/.venv/bin/python"
SDF="$HOME/PX4-Autopilot/Tools/simulation/gz/models/x500_c27/model.sdf"

wired() {
    [ -f "$SDF" ] && grep -q "odom_covariance_topic" "$SDF"
}

case "${1:-status}" in

status)
    if [ ! -f "$SDF" ]; then
        echo "no model installed at $SDF"
        echo "  build one:  $PY scanner/build_c27_drone.py"
        exit 1
    fi
    if wired; then
        echo "DRIFT TEST. The odometry goes to a private topic and PX4 has no"
        echo "position source of its own. Nothing flies without the relay:"
        echo "  $PY scanner/inject_drift.py --rate 0.01"
        echo "and launch_sim.sh needs DRIFT_TEST=1."
        echo
        echo "  back to normal:  ./scripts/drift_test.sh off"
    else
        echo "normal. PX4 reads the odometry directly, no relay involved."
        echo "  into a drift test:  ./scripts/drift_test.sh on"
    fi
    ;;

on)
    cd "$HERE/scanner" || exit 1
    "$PY" build_c27_drone.py --drift > /dev/null || exit 1
    wired || { echo "build claimed to succeed but the model is not wired"; exit 1; }
    echo "wired for a drift test. In three terminals, in this order:"
    echo "  1  DRIFT_TEST=1 ./scripts/launch_sim.sh nvidia"
    echo "  2  $PY scanner/inject_drift.py --rate 0.01"
    echo "  3  wait for Ready for takeoff, then run the scan"
    echo
    echo "The relay has to be up before the vehicle is armed. Started late it"
    echo "still works, but the flight begins with PX4 having had no position"
    echo "source, which is not the test you meant to run."
    echo
    echo "When the test is over:  ./scripts/drift_test.sh off"
    ;;

off)
    cd "$HERE/scanner" || exit 1
    "$PY" build_c27_drone.py > /dev/null || exit 1
    if wired; then
        echo "the rebuild left the drift wiring in place; look at $SDF"
        exit 1
    fi
    echo "back to normal. PX4 reads the odometry directly."
    echo "Restart the simulator for it to pick the model up."
    ;;

*)
    echo "usage: $0 [status|on|off]"
    exit 2
    ;;
esac
