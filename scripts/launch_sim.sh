#!/bin/bash
#
# Start PX4 SITL and Gazebo for the C27 scanner.
#
#   ./scripts/launch_sim.sh            render on whatever Mesa picks
#   ./scripts/launch_sim.sh nvidia     render on the discrete GPU
#   HEADLESS=0 ./scripts/launch_sim.sh watch the scene in a window
#
# It prints nothing while it runs. Output goes to a file, and HEADLESS
# defaults to 1, so there is no window either: a Gazebo window costs frames
# the scan wants, and piping the output through anything that cannot drain its
# protobuf chatter blocks it on write and collapses the real time factor. Set
# HEADLESS=0 to watch a run anyway, which is worth doing once to see the route
# rather than to fly a scan that will be scored.
#
# Watch it with:
#
#   tail -f "$LOG" | grep -v libprotobuf
#
# Once "Ready for takeoff" appears, run the scan in another terminal.
set -u

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
PY="${PY:-$HOME/autonomous_landing/venv/bin/python}"
if [ ! -x "$PY" ]; then
    PY="$HERE/.venv/bin/python"
fi
if [ ! -x "$PY" ]; then
    echo "no interpreter; set PY, for example"
    echo "  PY=~/autonomous_landing/venv/bin/python $0"
    exit 1
fi
LOGDIR="${LOGDIR:-$HERE/out}"

if [ ! -d "$PX4" ]; then
    echo "no PX4 checkout at $PX4; set PX4_DIR"
    exit 1
fi

# The spawn point has to match layout.json, which is what the scanner treats
# as the origin of everything it commands.
read -r X Y < <("$PY" - "$HERE/scanner/layout.json" <<'PYEOF'
import json, sys
layout = json.load(open(sys.argv[1], encoding="utf-8"))
print(layout["spawn_x"], layout["spawn_y"])
PYEOF
)
if [ -z "${X:-}" ]; then
    echo "could not read the spawn point out of scanner/layout.json"
    exit 1
fi

mkdir -p "$LOGDIR"
export PX4_GZ_MODEL_POSE="$X,$Y,0.30,0,0,0"
# PX4 starts the gui when HEADLESS is empty, not when it is zero: the test in
# px4-rc.gzsim is [ -z "${HEADLESS}" ]. HEADLESS=0 is a non-empty string and
# leaves the window shut, which reads as this script ignoring the request.
case "${HEADLESS:-1}" in
    0|no|false|off) unset HEADLESS ;;
    *)              export HEADLESS=1 ;;
esac

if [ "${1:-}" = "nvidia" ]; then
    # Without this Mesa picks the integrated GPU and the simulation runs at
    # about a sixth of the speed.
    export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
    LOG="$LOGDIR/sitl_nvidia.log"
else
    LOG="$LOGDIR/sitl.log"
fi

echo "spawning at $X $Y, logging to $LOG"
echo "wait for: Ready for takeoff"
cd "$PX4" || exit 1
# stdin is held open because PX4's console spins redrawing its prompt at EOF,
# which produced a 7.5 GB log the first time this was run in the background.
exec tail -f /dev/null | make px4_sitl gz_x500_c27_warehouse > "$LOG" 2>&1
