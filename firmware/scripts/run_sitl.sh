#!/bin/bash
# Run ArduPlane SITL connected to Gazebo via paraglider_bridge.py
#
# Three terminals needed:
#   T1: gz sim server + GUI (paraglider world)
#   T2: paraglider_bridge.py (this script auto-starts it)
#   T3: ArduPlane SITL (this script)
#
# Usage:
#   ./run_sitl.sh               # full stack (gazebo + bridge + AP)
#   ./run_sitl.sh ap-only       # only ArduPlane SITL (assume gz + bridge running)

set -e
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
AP=$HOME/ardupilot
GZ_RES=$HOME/.gz/models

export GZ_SIM_RESOURCE_PATH=$GZ_RES:$GZ_SIM_RESOURCE_PATH

mode=${1:-full}

if [ "$mode" = "full" ]; then
  echo "[1/3] starting gz sim server (paused)"
  pkill -9 -f "gz sim" 2>/dev/null || true
  sleep 1
  gz sim -s "$ROOT/sdf/world.sdf" -v 2 > /tmp/gz_server.log 2>&1 &
  sleep 5

  echo "[2/3] starting bridge"
  python3 "$ROOT/firmware/bridge/paraglider_bridge.py" > /tmp/bridge.log 2>&1 &
  sleep 2

  echo "[3/3] starting gz sim GUI"
  gz sim -g > /tmp/gz_gui.log 2>&1 &
  sleep 3
fi

echo "[ap] launching ArduPlane SITL with JSON physics + paraglider params"
cd "$AP"
"$AP/Tools/autotest/sim_vehicle.py" \
  -v ArduPlane \
  -f JSON \
  --add-param-file="$ROOT/firmware/params/paraglider.parm" \
  --console --map \
  -L CMAC
