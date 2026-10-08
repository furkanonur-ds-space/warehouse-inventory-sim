#!/bin/bash
# Real PX4, on this machine, standing where the VOXL2 will stand.
#
#   terminal 1:  ./scripts/rehearse_px4.sh
#   terminal 2:  ./scripts/run_hitl.sh -i 127.0.0.1 --no-vio -g
#
# PX4 waits in terminal 1 until the bridge starts sending; the pxh> prompt
# appears once sensors arrive. Then, in terminal 1:
#
#   commander arm
#   commander takeoff
#   commander land
#
# This is the rehearsal for the board. It runs the same flight stack against
# the same bridge over the same UDP messages, so a fault found here is a
# fault in the bridge and not in the board, and it costs nothing to find.
#
# What differs from the board, and why it does not matter here: the
# airframe is PX4's generic none_iris rather than ModalAI's HITL parameters,
# so the arm lengths differ slightly. Motor order and spin direction are the
# same standard quad X on both, which docs/voxl2_compatibility.md checked.
#
# Nothing under ~/PX4-Autopilot is changed. The startup files are copied and
# one line of the copy is edited, and PX4 keeps its parameters in a working
# directory of its own, so the warehouse simulation's settings are untouched.
set -e
cd "$(dirname "$0")/.."
ROOT="$(pwd)"
PX4_BUILD="$HOME/PX4-Autopilot/build/px4_sitl_default"

if [ ! -x "$PX4_BUILD/bin/px4" ]; then
  echo "PX4 SITL is not built at $PX4_BUILD"
  exit 1
fi

ETC="$ROOT/build/px4_etc"
WORK="$ROOT/build/px4_work"
rm -rf "$ETC"
cp -r "$PX4_BUILD/etc" "$ETC"
mkdir -p "$WORK"

# The one change: talk to the simulator over UDP on 14560, as the board
# does, instead of waiting for a TCP connection on 4560. With TCP, PX4 blocks
# at startup until something connects, and the bridge never would.
RC="$ETC/init.d-posix/px4-rc.mavlinksim"
sed -i 's|simulator_mavlink start -c \$simulator_tcp_port|simulator_mavlink start -u 14560|' "$RC"
if ! grep -q "simulator_mavlink start -u 14560" "$RC"; then
  echo "ERROR could not switch the simulator link to UDP in $RC"
  exit 1
fi

# A bench has no radio and no ground station, and PX4 would otherwise refuse
# to arm for the lack of either. The magnetometer, GPS and EKF checks are
# left as they are on purpose: passing them is part of what this tests.
cat >> "$RC" <<'EOF'

# Added by hitl-bridge/scripts/rehearse_px4.sh: no radio, no ground station.
param set COM_RC_IN_MODE 4
param set NAV_RCL_ACT 0
param set NAV_DLL_ACT 0
EOF

echo "PX4 is starting and will wait for the bridge."
echo "In another terminal:  ./scripts/run_hitl.sh -i 127.0.0.1 --no-vio -g"
echo

cd "$WORK"
exec env PX4_SYS_AUTOSTART=10016 "$PX4_BUILD/bin/px4" "$ETC"
