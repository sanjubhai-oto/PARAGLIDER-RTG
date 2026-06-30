#!/usr/bin/env bash
# Build + test the pusher feed-forward patch in ArduCopter SITL.
# RUN THIS ON YOUR MAC (it has the ArduPilot source + toolchain).
#
#   ./run_sitl_mac.sh /path/to/ardupilot /path/to/PARAGLIDER-RTG
#
# It applies the patch (if needed), builds Copter SITL, launches a hexa, and runs
# the probe that confirms the ATC_PUSH_* feed-forward is live.
set -e

AP="${1:?usage: run_sitl_mac.sh <ardupilot_dir> <paraglider_repo_dir>}"
REPO="${2:?usage: run_sitl_mac.sh <ardupilot_dir> <paraglider_repo_dir>}"
PATCH="$REPO/firmware/copter_pusher/ardupilot_patch/pusher_ff.patch"
PROBE="$REPO/firmware/copter_pusher/sim/sitl_pusher_probe.py"

cd "$AP"

# 1. apply patch if not already in
if ! grep -q "Pusher (X-axis propulsion) torque feed-forward" \
        libraries/AC_AttitudeControl/AC_AttitudeControl_Multi.cpp 2>/dev/null; then
  echo "==> applying pusher_ff.patch"
  git apply "$PATCH" || { echo "patch rejected -> apply ardupilot_patch/MANUAL_APPLY.md by hand"; exit 1; }
else
  echo "==> patch already present"
fi

# 2. build SITL copter
echo "==> building Copter SITL"
./waf configure --board sitl
./waf copter

# 3. launch SITL (hexa) headless, then run the probe
echo "==> launching SITL hexa"
pip3 install --quiet pymavlink >/dev/null 2>&1 || true
Tools/autotest/sim_vehicle.py -v ArduCopter -f hexa --no-rebuild \
    --out udp:127.0.0.1:14550 --daemon >/tmp/sitl.log 2>&1 &
SITL_PID=$!
trap "kill $SITL_PID 2>/dev/null || true" EXIT
sleep 25   # let EKF settle / GPS lock

echo "==> running probe"
python3 "$PROBE" --conn udp:127.0.0.1:14550 --gain 0.12

echo "==> done (SITL log: /tmp/sitl.log)"
