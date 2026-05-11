#!/bin/bash
# Reliable SITL launcher with PID tracking — replaces all running processes.

set -e
ROOT="$HOME/paraglider_sim"
PID_DIR="/tmp/paraglider_pids"
mkdir -p "$PID_DIR"

# 1. Nuclear kill all SITL processes — including by PID file + by name
echo "=== killing all SITL/sim processes ==="
for f in "$PID_DIR"/*.pid; do
  [ -f "$f" ] && kill -9 "$(cat "$f")" 2>/dev/null && rm "$f"
done

# Find all relevant python/arduplane processes by command pattern, kill them
ps -axww -o pid,command | awk '
  /sim_vehicle\.py.*ArduPlane/ ||
  /arduplane.*--model.*JSON/ ||
  /paraglider_bridge\.py/ ||
  /mav_forwarder\.py/ ||
  /mavproxy\.py.*5760/ {print $1}
' | xargs -I{} kill -9 {} 2>/dev/null

# Free SITL ports
for port in 9002 14550 14551 14552 5760; do
  for p in $(lsof -iUDP:$port -t 2>/dev/null) $(lsof -iTCP:$port -t 2>/dev/null); do
    kill -9 "$p" 2>/dev/null
  done
done

sleep 4
echo "=== all clean ==="

# 2. gz with autoplay
export GZ_SIM_RESOURCE_PATH="$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH"
if ! pgrep -f "gz sim -s" >/dev/null; then
  gz sim -s -r "$ROOT/sdf/world.sdf" -v 2 > /tmp/gz.log 2>&1 &
  echo $! > "$PID_DIR/gz_server.pid"
  echo "1. gz sim server PID $!"
  sleep 5
fi

if ! pgrep -f "gz sim -g" >/dev/null; then
  gz sim -g > /tmp/gz_gui.log 2>&1 &
  echo $! > "$PID_DIR/gz_gui.pid"
  echo "2. gz sim GUI PID $!"
  sleep 3
fi

# 3. bridge
python3 -u "$ROOT/firmware/bridge/paraglider_bridge.py" > /tmp/bridge.log 2>&1 &
echo $! > "$PID_DIR/bridge.pid"
echo "3. bridge PID $!"
sleep 2

# 4. AP via sim_vehicle WITHOUT --slave so no parent check
cd /tmp
nohup "$HOME/ardupilot/Tools/autotest/sim_vehicle.py" -v ArduPlane -f JSON \
  --add-param-file="$ROOT/firmware/params/paraglider.parm" \
  --no-mavproxy --no-rebuild --wipe-eeprom -L CMAC \
  > /tmp/ap.log 2>&1 &
AP_PID=$!
disown $AP_PID 2>/dev/null
echo $AP_PID > "$PID_DIR/sim_vehicle.pid"
echo "4. sim_vehicle PID $AP_PID"
sleep 12

# 5. mav_forwarder
python3 -u "$ROOT/firmware/scripts/mav_forwarder.py" > /tmp/fwd.log 2>&1 &
echo $! > "$PID_DIR/fwd.pid"
echo "5. forwarder PID $!"
sleep 5

# 6. QGC
if ! pgrep -f QGroundControl >/dev/null; then
  open -a /Users/sanju/Desktop/QGroundControl.app
  echo "6. QGC launched"
  sleep 4
fi

echo ""
echo "=== verify ==="
echo "fwd log:"
tail -8 /tmp/fwd.log
echo ""
echo "tcp 5760:"
lsof -iTCP:5760 2>&1 | head -3
echo ""
echo "Test mavlink:"
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
hb = m.wait_heartbeat(timeout=10)
print(f'  mode={hb.custom_mode if hb else None} armed={(hb.base_mode & 128) != 0 if hb else None}')
"
echo ""
echo "Procs:"
ps aux | grep -E "arduplane|sim_vehicle|paraglider_bridge|mav_forwarder|gz sim|QGroundControl" | grep -v grep | awk '{print "  "$2,$11,$12}'
