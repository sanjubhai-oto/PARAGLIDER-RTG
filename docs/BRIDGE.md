# Python Bridge (`firmware/bridge/paraglider_bridge.py`)

Translates between ArduPilot SITL JSON protocol and Gazebo topics. Also applies bounded virtual physics (thrust, lift, attitude restoration) directly to fuselage via EntityWrench.

## Run

```bash
python3 -u ~/paraglider_sim/firmware/bridge/paraglider_bridge.py > /tmp/bridge.log 2>&1 &
```

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    paraglider_bridge.py                 │
├─────────────────────────────────────────────────────────┤
│                                                         │
│  UDP 9002 RX  ←──────── ArduPlane (PWM[16] packets)    │
│       ↓                                                 │
│   parse PWM                                             │
│   unpack: throttle, ail, elev, arm_l, arm_r, sw_up      │
│       ↓                                                 │
│   ┌─────────────────────────────────────────────────┐  │
│   │ Apply Virtual Physics                            │  │
│   │ - Motor RPM → gz topic /paraglider_uav/...      │  │
│   │ - Arm positions → gz topics /arm/left, /arm/right│  │
│   │ - EntityWrench (force + torque) → world wrench  │  │
│   │ - PWM watchdog (zero forces if AP silent >0.5s) │  │
│   └─────────────────────────────────────────────────┘  │
│       ↓                                                 │
│   gz subscriber callbacks:                              │
│       on_pose() → update state.pos/att/quat            │
│       on_imu()  → update state.gyro/accel              │
│       ↓                                                 │
│   build SITL JSON (timestamp, imu, vel, att, lat/lon/alt│
│       ↓                                                 │
│  UDP 9002 TX  ─────────→ ArduPlane (sensor reply)      │
│                                                         │
└─────────────────────────────────────────────────────────┘
```

## Configuration Constants

```python
THRUST_FWD_MAX  = 5.0     # N forward thrust at full throttle
VEHICLE_WEIGHT  = 3.53    # N upward force always-on (cancels gravity-affected mass × g)
THRUST_UP       = 0.8     # N extra upward at full throttle
PITCH_UP_FORCE  = 2.0     # N extra upward when stick BACK
PITCH_DOWN_FORCE= 1.0     # N "spill" lift reduction when stick FORWARD
PITCH_UP_TORQUE = 0.05    # N·m nose-up torque
PITCH_DOWN_TORQUE = 0.05  # N·m nose-down
ROLL_TORQUE     = 0.05    # N·m roll torque from aileron
K_RP            = 0.8     # N·m/rad attitude restoring stiffness
D_RP            = 0.40    # N·m·s/rad angular damping
MAX_RPM         = 800.0   # rad/s motor visual spin at full throttle
ARM_MAX         = 0.50    # rad arm joint full pull (clipped)
```

## Force/Torque Mapping

### Forces (world frame, applied to `paraglider_uav::fuselage`)

```python
yaw = state.att[2]  # from IMU/pose
fwd_x = sin(yaw)
fwd_y = -cos(yaw)   # body -Y direction in world frame
rgt_x = cos(yaw)
rgt_y = sin(yaw)

fwd_mag = throttle * THRUST_FWD_MAX
up_mag  = VEHICLE_WEIGHT
        + throttle * THRUST_UP
        + pitch_up * PITCH_UP_FORCE
        - pitch_down * PITCH_DOWN_FORCE
lat_mag = ail_norm * 1.0   # 1 N lateral push from aileron

force_x = fwd_mag * fwd_x + lat_mag * rgt_x
force_y = fwd_mag * fwd_y + lat_mag * rgt_y
force_z = up_mag
```

### Torques (world frame)

```python
roll  = state.att[0]
pitch = state.att[1]
omega_x, omega_y, omega_z = state.gyro

# Attitude restoration (auto-level)
torque_x = -K_RP * roll  - D_RP * omega_x
torque_y = -K_RP * pitch - D_RP * omega_y
torque_z = -ail_norm * 0.05 - 0.1 * omega_z   # roll input -> yaw turn (banking)
```

### Direct arm control (bypasses firmware mixer)

```python
ail_pos = max(0,  ail_norm)   # 0..1 right
ail_neg = max(0, -ail_norm)   # 0..1 left
pd_norm = max(0,  elev_norm)  # 0..1 pitch-down (brake)
arm_l = min(ARM_MAX, (ail_neg + pd_norm) * ARM_MAX)
arm_r = min(ARM_MAX, (ail_pos + pd_norm) * ARM_MAX)
```

## PWM Watchdog

If no PWM packet received from ArduPlane for >0.5s, bridge publishes zero EntityWrench. Prevents the "vehicle drifts kilometers" bug when AP dies.

```python
if time.time() - last_pwm_time > 0.5 and ap_addr is not None:
    zw = EntityWrench()
    zw.entity.name = "paraglider_uav::fuselage"
    zw.entity.type = Entity.LINK
    pub_wrench.publish(zw)   # all force/torque = 0
```

## SITL JSON Reply Format

```python
payload = {
    "timestamp": elapsed_seconds,
    "imu": {
        "gyro":       [gx_ned, gy_ned, gz_ned],
        "accel_body": [ax_ned, ay_ned, az_ned]
    },
    "velocity":  [vx_ned, vy_ned, vz_ned],
    "attitude":  [roll, pitch, -yaw],     # ENU → NED yaw flip
    "latitude":  HOME_LAT + offset_y * deg_per_m,
    "longitude": HOME_LON + offset_x * deg_per_m,
    "altitude":  HOME_ALT + offset_z
}
msg = b"\n" + json.dumps(payload).encode() + b"\n"
sock.sendto(msg, ap_addr)
```

Notes:
- Sent at ~250 Hz (gated by `if now - last_send < 0.004: continue`)
- ENU (gz convention: X=right, Y=fwd, Z=up) → NED (AP: X=fwd, Y=right, Z=down) swap
- AP requires `attitude` field or main loop stalls
- Velocity clamped to ±50 m/s to prevent EKF blowup

## Socket Setup

```python
rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.bind(("127.0.0.1", AP_RX_PORT))   # 9002
rx.settimeout(0.001)                  # near non-blocking
```

**Critical:** `settimeout(0.001)` allows send loop to hit 250Hz independent of RX. Earlier `settimeout(0.1)` caused 76Hz IMU rate which failed ArduPlane's gyro health check (`Gyro rate < SCHED_LOOP_RATE * 1.8`).

## Gz Topics

| Topic | Direction | Message type | Purpose |
|-------|-----------|--------------|---------|
| `/paraglider_uav/command/motor_speed` | bridge → gz | `Actuators` | Motor RPM (visual only with motorConstant=0) |
| `/arm/left`, `/arm/right` | bridge → gz | `Double` | Arm joint positions in radians |
| `/world/paraglider_world/wrench` | bridge → gz | `EntityWrench` | Force + torque on fuselage link |
| `/world/.../dynamic_pose/info` | gz → bridge | `Pose_V` | Vehicle pose (callback `on_pose`) |
| `/world/.../imu_sensor/imu` | gz → bridge | `IMU` | Gyro + accel (callback `on_imu`) |

## Dependencies

```bash
pip install pymavlink
pip install gz-transport13 gz-msgs10   # gz Python bindings
# Or: brew install gz-transport13 gz-msgs10
```

Bindings imported:
```python
from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.entity_wrench_pb2 import EntityWrench
from gz.msgs10.entity_pb2 import Entity
```
