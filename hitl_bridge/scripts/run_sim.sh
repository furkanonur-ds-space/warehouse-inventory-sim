#!/bin/bash
# Run the HITL world headless. Stage 1 sends nothing to any drone, so this
# is safe to run on its own.
#
#   ./scripts/run_sim.sh            run until interrupted
#   ./scripts/run_sim.sh 1250       run 1250 steps (5 s of simulated time)
set -e
cd "$(dirname "$0")/.."
ROOT="$(pwd)"

# Gazebo's discovery stays on this machine; see run_hitl.sh.
export GZ_IP=127.0.0.1
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ROOT/build:$GZ_SIM_SYSTEM_PLUGIN_PATH"
export GZ_SIM_RESOURCE_PATH="$ROOT/models:$HOME/PX4-Autopilot/Tools/simulation/gz/models:$GZ_SIM_RESOURCE_PATH"

if [ -n "$1" ]; then
  exec gz sim -s -r --iterations "$1" worlds/hitl_x500.sdf
else
  exec gz sim -s -r worlds/hitl_x500.sdf
fi
