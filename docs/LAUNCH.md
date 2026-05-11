# Launch Procedure

## Full Cold Start (proven working)

Run each block in order. Sleeps are required.

### 1. Kill stale processes + free ports

```bash
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|sim_vehicle|mav_forw" | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {} 2>/dev/null
for port in 14550 14551 14552 5760 9002; do
  for p in $(lsof -iTCP:$port -t 2>/dev/null) $(lsof -iUDP:$port -t 2>/dev/null); do
    kill -9 $p 2>/dev/null
  done
done
sleep 4
```

### 2. Gazebo physics server

```bash
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH
gz sim -s -r ~/paraglider_sim/sdf/world.sdf -v 2 > /tmp/gz.log 2>&1 &
sleep 5
```

### 3. Gazebo GUI

```bash
gz sim -g > /tmp/gz_gui.log 2>&1 &
sleep 4
```

### 4. Bridge (must start BEFORE AP so AP gets sensor JSON)

```bash
python3 -u ~/paraglider_sim/firmware/bridge/paraglider_bridge.py > /tmp/bridge.log 2>&1 &
sleep 2
```

### 5. ArduPlane SITL

```bash
cd /tmp
~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON \
  --add-param-file=$HOME/paraglider_sim/firmware/params/paraglider.parm \
  --no-mavproxy --no-rebuild --wipe-eeprom -L CMAC > /tmp/ap.log 2>&1 &
disown
sleep 14
```

**Critical flags:**
- `-f JSON`: SITL JSON I/O backend (works with bridge)
- `--no-mavproxy`: don't start mavproxy daemon (unstable on macOS)
- `--no-rebuild`: skip waf build (already built)
- `--wipe-eeprom`: clean param state (apply runtime params via MAVLink below)
- `-L CMAC`: home location

### 6. Mavlink forwarder (TCP 5760 ↔ UDP 14550/14551)

```bash
python3 -u ~/paraglider_sim/firmware/scripts/mav_forwarder.py > /tmp/fwd.log 2>&1 &
disown
sleep 4
```

### 7. Apply runtime params + save EEPROM

```bash
python3 -u <<'EOF'
from pymavlink import mavutil
import time
m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
m.wait_heartbeat()
def setp(n, v):
    m.mav.param_set_send(m.target_system, m.target_component, n.encode(), v,
                         mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
    time.sleep(0.15)

setp('SCHED_LOOP_RATE',  25)        # gyro check minimum rate = 25*1.8 = 45 Hz
setp('ARMING_SKIPCHK',   4194303)   # bypass all pre-arm checks (bitmask all bits)
setp('AHRS_EKF_TYPE',    10)        # SITL fake AHRS (no EKF GPS sanity)
setp('INS_USE2',         0)
setp('INS_USE3',         0)         # single IMU only
setp('COMPASS_USE',      0)
setp('BRD_SAFETYENABLE', 0)         # safety always off
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(2)
print("params saved")
EOF
```

### 8. Open QGroundControl

```bash
open -a /Users/sanju/Desktop/QGroundControl.app
```

QGC auto-connects to UDP 14550.

## Verify Stack

```bash
# Heartbeat check
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
hb = m.wait_heartbeat(timeout=5)
print(f'mode={hb.custom_mode} armed={(hb.base_mode & 128) != 0}' if hb else 'NO HEARTBEAT')
"

# Vehicle pose
gz model -m paraglider_uav --pose

# Process list
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|mav_forw" | grep -v grep | awk '{print $2, $11}'
```

Expected: mode=12 (LOITER), armed=False, pose at (0, 0, ~0.10), 4-5 processes alive.

## Arming + Flight

In QGroundControl:
1. Switch mode to MANUAL (or keep LOITER — firmware override forces pilot throttle in any mode)
2. Click **Arm**
3. Advance throttle slider → vehicle accelerates forward
4. Stick UP (pitch back) → climb assist
5. Stick LEFT/RIGHT → bank turn

Or via pymavlink direct TCP (forwarder strips RC overrides):

```bash
python3 -u <<'EOF'
from pymavlink import mavutil
import time, subprocess
m = mavutil.mavlink_connection('tcp:127.0.0.1:5760', source_system=255)
m.wait_heartbeat()
m.mav.set_mode_send(m.target_system, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 0)  # MANUAL
time.sleep(1)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(2)
# Full throttle for 8s
t0 = time.time()
while time.time() - t0 < 8:
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                     1500, 1500, 2000, 1500, 1500, 1500, 1500, 1500)
    time.sleep(0.05)
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
EOF
```

## Common Commands

```bash
# Get pose
gz model -m paraglider_uav --pose

# Send motor command directly (bypass AP)
gz topic -t /paraglider_uav/command/motor_speed -m gz.msgs.Actuators -p 'velocity: [400.0]'

# Move arm directly
gz topic -t /arm/left -m gz.msgs.Double -p 'data: 0.4'

# Check gz topic activity
gz topic -i -t /paraglider_uav/command/motor_speed

# Test motor wrench
gz topic -i -t /world/paraglider_world/wrench

# Kill everything
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|sim_vehicle|mav_forw" | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {}
```

## Restart Just Bridge

When tuning bridge constants without full restart:

```bash
ps aux | grep paraglider_bridge | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {}
for p in $(lsof -iUDP:9002 -t); do kill -9 $p; done
sleep 2
python3 -u ~/paraglider_sim/firmware/bridge/paraglider_bridge.py > /tmp/bridge.log 2>&1 &
```

## Restart Just AP (after firmware rebuild)

```bash
ps aux | grep -E "arduplane|sim_vehicle" | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {}
for p in $(lsof -iTCP:5760 -t); do kill -9 $p; done
sleep 2
cd /tmp
~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON \
  --add-param-file=$HOME/paraglider_sim/firmware/params/paraglider.parm \
  --no-mavproxy --no-rebuild --wipe-eeprom -L CMAC > /tmp/ap.log 2>&1 &
disown
sleep 14

# Re-apply params (wipe-eeprom resets)
# ... run params script from step 7
```
