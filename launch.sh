#!/usr/bin/env bash
# PARAGLIDER-RTG launch script
# Starts: gz server, gz GUI, bridge, ArduPlane SITL, MAVLink forwarder
# Applies runtime params and leaves stack ready for QGroundControl.

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ARDUPILOT_DIR="${ARDUPILOT_DIR:-$HOME/ardupilot}"

echo "==> PARAGLIDER-RTG launch"

# -----------------------------------------------------------------------------
# 0. Sanity checks
# -----------------------------------------------------------------------------
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "ERROR: ArduPilot not at $ARDUPILOT_DIR. Run ./setup.sh first."
    exit 1
fi
if [ ! -f "$ARDUPILOT_DIR/build/sitl/bin/arduplane" ]; then
    echo "ERROR: arduplane binary missing. Run: cd $ARDUPILOT_DIR && ./waf plane"
    exit 1
fi
if ! command -v gz &>/dev/null; then
    echo "ERROR: gz (Gazebo Harmonic) not found. Run ./setup.sh first."
    exit 1
fi

# -----------------------------------------------------------------------------
# 1. Kill stale processes
# -----------------------------------------------------------------------------
echo "==> Killing stale processes"
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|sim_vehicle|mav_forw" | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {} 2>/dev/null || true
for port in 14550 14551 14552 5760 9002; do
    for p in $(lsof -iTCP:$port -t 2>/dev/null) $(lsof -iUDP:$port -t 2>/dev/null); do
        kill -9 $p 2>/dev/null || true
    done
done
sleep 4

# -----------------------------------------------------------------------------
# 2. Gazebo server
# -----------------------------------------------------------------------------
echo "==> Starting Gazebo physics server"
export GZ_SIM_RESOURCE_PATH="$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH"
gz sim -s -r "$PROJECT_DIR/sdf/world.sdf" -v 2 > /tmp/gz.log 2>&1 &
sleep 5

# -----------------------------------------------------------------------------
# 3. Gazebo GUI
# -----------------------------------------------------------------------------
echo "==> Starting Gazebo GUI"
gz sim -g > /tmp/gz_gui.log 2>&1 &
sleep 4

# -----------------------------------------------------------------------------
# 4. Python bridge (start BEFORE AP so AP gets sensor JSON)
# -----------------------------------------------------------------------------
echo "==> Starting bridge"
python3 -u "$PROJECT_DIR/firmware/bridge/paraglider_bridge.py" > /tmp/bridge.log 2>&1 &
sleep 2

# -----------------------------------------------------------------------------
# 5. ArduPlane SITL (via sim_vehicle.py wrapper)
# -----------------------------------------------------------------------------
echo "==> Starting ArduPlane SITL"
cd /tmp
"$ARDUPILOT_DIR/Tools/autotest/sim_vehicle.py" -v ArduPlane -f JSON \
    --add-param-file="$PROJECT_DIR/firmware/params/paraglider.parm" \
    --no-mavproxy --no-rebuild --wipe-eeprom -L CMAC > /tmp/ap.log 2>&1 &
disown
sleep 14

# -----------------------------------------------------------------------------
# 6. MAVLink forwarder
# -----------------------------------------------------------------------------
echo "==> Starting MAVLink forwarder"
python3 -u "$PROJECT_DIR/firmware/scripts/mav_forwarder.py" > /tmp/fwd.log 2>&1 &
disown
sleep 4

# -----------------------------------------------------------------------------
# 7. Apply runtime params + save EEPROM
# -----------------------------------------------------------------------------
echo "==> Applying runtime params"
python3 -u <<EOF
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
m.wait_heartbeat()
def setp(n, v):
    m.mav.param_set_send(m.target_system, m.target_component, n.encode(), v,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)

setp('SCHED_LOOP_RATE',  25)
setp('ARMING_SKIPCHK',   4194303)
setp('AHRS_EKF_TYPE',    10)
setp('INS_USE2',         0)
setp('INS_USE3',         0)
setp('COMPASS_USE',      0)
setp('BRD_SAFETYENABLE', 0)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(2)
hb = m.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
print(f"  mode={hb.custom_mode} armed={(hb.base_mode & 128) != 0}")
EOF

# -----------------------------------------------------------------------------
echo ""
echo "============================================"
echo "  Stack ready!"
echo "============================================"
echo ""
echo "Open QGroundControl and connect to UDP 14550."
echo "  1. Arm the vehicle"
echo "  2. Advance throttle stick"
echo "  3. Roll/pitch sticks drive brake arms"
echo ""
echo "Logs:"
echo "  /tmp/gz.log       (Gazebo server)"
echo "  /tmp/gz_gui.log   (Gazebo GUI)"
echo "  /tmp/bridge.log   (bridge)"
echo "  /tmp/ap.log       (ArduPlane SITL)"
echo "  /tmp/fwd.log      (MAVLink forwarder)"
echo ""
echo "Kill all with: ./kill.sh"
