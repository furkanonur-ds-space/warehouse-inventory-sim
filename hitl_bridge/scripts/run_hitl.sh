#!/bin/bash
# Run the HITL world against a real board, in real time, until interrupted.
#
#   ./scripts/run_hitl.sh -i 192.168.101.2          the board over USB-C
#   ./scripts/run_hitl.sh -i 192.168.101.2 -g       and open the Gazebo window
#   ./scripts/run_hitl.sh -i 127.0.0.1              PX4 on this machine
#   ./scripts/run_hitl.sh -i 192.168.101.2 --no-vio GPS only, no ODOMETRY
#
# The same job as ModalAI's hitl-gz-start.sh, which is not published: put
# the board's address into the world and start Gazebo.
#
# Unlike run_sim.sh this never stops on its own. PX4 on the board runs on
# the wall clock, so the simulation has to keep up with it for as long as
# the session lasts; a fixed number of steps would end the flight mid-air.
set -e
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

ADDR=""
GUI=0
VIO=1
while [ $# -gt 0 ]; do
  case "$1" in
    -i) ADDR="$2"; shift 2 ;;
    -g) GUI=1; shift ;;
    --no-vio) VIO=0; shift ;;
    *) echo "unknown option: $1"; exit 2 ;;
  esac
done

if [ -z "$ADDR" ]; then
  echo "usage: $0 -i <board address> [-g] [--no-vio]"
  echo "the board over USB-C is usually 192.168.101.2"
  exit 2
fi

if [ ! -f build/libhitl_bridge.so ]; then
  echo "build it first: ./scripts/build.sh"
  exit 1
fi

# A generated copy, so the world in git keeps its defaults and a session
# against one board cannot leave its address behind for the next.
mkdir -p build
WORLD=build/hitl_world.sdf
sed -e "s|<mavlink_addr>[^<]*</mavlink_addr>|<mavlink_addr>$ADDR</mavlink_addr>|" \
    -e "s|<vio_mavlink_addr>[^<]*</vio_mavlink_addr>|<vio_mavlink_addr>$ADDR</vio_mavlink_addr>|" \
    worlds/hitl_x500.sdf > "$WORLD"
if [ "$VIO" -eq 0 ]; then
  sed -i "s|<en_vio_output>true</en_vio_output>|<en_vio_output>false</en_vio_output>|" "$WORLD"
fi

# Check the substitution happened rather than trusting sed: a world that
# still says 127.0.0.1 sends everything to this machine and looks, from
# here, exactly like a board that is not answering.
if ! grep -q "<mavlink_addr>$ADDR</mavlink_addr>" "$WORLD"; then
  echo "ERROR the board address did not make it into $WORLD"
  exit 1
fi

# Keep Gazebo's own discovery on this machine. It finds the GUI and the
# server by multicast, and with WSL sharing Windows' network (mirrored mode,
# which the board needs) there is no route for it: "Exception sending a
# multicast message: Network is unreachable". PX4 starts Gazebo the same way.
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$ROOT/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH"

echo "HITL world: sensors to $ADDR:14560, odometry $([ "$VIO" -eq 1 ] && echo "to $ADDR:14570" || echo off)"
echo "Watch for '| back: actuators' rising above 0 Hz: that is the board answering."
echo "Ctrl+C to stop."

if [ "$GUI" -eq 1 ]; then
  exec gz sim -r "$WORLD"
else
  exec gz sim -s -r "$WORLD"
fi
