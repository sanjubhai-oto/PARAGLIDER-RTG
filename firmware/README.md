# ArduParaglider — paraglider configuration of ArduPilot

Native firmware build of ArduPlane with paraglider arm-control mixer added in C++.
**No Lua. No external scripts. All ArduPilot infrastructure intact.**

## Inheritance from ArduPlane (all features keep working)

| Feature | Status |
|---------|--------|
| Flight modes (MANUAL, STABILIZE, FBWA, FBWB, CRUISE, AUTOTUNE, AUTO, RTL, LOITER, GUIDED, ACRO, CIRCLE, TAKEOFF, QSTABILIZE, etc.) | ✓ all available |
| MAVLink commands (MAV_CMD_*) | ✓ all available |
| Mission Planner / QGroundControl | ✓ |
| EKF3 attitude/position estimator | ✓ |
| TECS (Total Energy Control System) — altitude+airspeed via throttle+pitch | ✓ |
| L1 navigation controller | ✓ |
| RTL, AUTO missions, geofence | ✓ |
| Failsafes (RC, battery, GCS) | ✓ |
| Parameter set (RLL_RATE_*, PTCH_RATE_*, NAVL1_*, TECS_*, etc.) | ✓ all standard |
| Logging (DataFlash) | ✓ |
| Tuning utilities (in-flight tune, autotune) | ✓ |

## Paraglider-specific addition

C++ block in `ArduPlane/servos.cpp` (after `set_throttle()`, before `servos_output()`):

```cpp
// ArduParaglider arm-control mixer
const float ail  = SRV_Channels::get_output_scaled(SRV_Channel::k_aileron)  / 4500.0f;
const float elev = SRV_Channels::get_output_scaled(SRV_Channel::k_elevator) / 4500.0f;
const float arm_common = (elev < 0.0f) ? -elev : 0.0f;
constexpr float ARM_MAX = 0.85f;
float arm_left  = clamp(arm_common + ail, 0, ARM_MAX);
float arm_right = clamp(arm_common - ail, 0, ARM_MAX);
SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_left,  arm_left  * 4500.0f);
SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_right, arm_right * 4500.0f);
```

This converts ArduPlane's standard aileron/elevator demand into two paraglider arm servos. **No "brake" terminology** — these are physical arm pulls that modulate canopy aerodynamics.

## Physical control flow

```
   throttle stick UP     ──► motor RPM up ──► forward thrust
                                                 │
                                                 ▼
                              canopy generates lift (aero), pitches up
                              ──► climbs at glide angle (NOT vertical)

   pitch stick DOWN      ──► AP elev demand (-)  ──► arm_common = |elev|
                                                 ──► both arms pull
                                                 ──► canopy panels ↑ Cl
                                                 ──► nose-down moment

   pitch stick UP/CENTER ──► AP elev demand (≥0) ──► arm_common = 0
                                                 ──► arms released
                                                 ──► natural climb under thrust

   roll stick RIGHT      ──► AP aileron demand (+) ──► arm_left += , arm_right −=
                                                 ──► asymmetric lift
                                                 ──► proverse turn right (yaw+roll)

   roll stick LEFT       ──► turn left (mirror)
```

## Servo mapping

| Channel | Function ID | Purpose |
|---------|-------------|---------|
| SERVO1 | 4 (k_aileron)         | AP roll demand source |
| SERVO2 | 19 (k_elevator)       | AP pitch demand source |
| SERVO3 | 70 (k_throttle)       | Motor PWM 1000-2000 → bridge → motor RPM |
| SERVO5 | 24 (k_flaperon_left)  | **Left arm** (firmware-mixed) |
| SERVO6 | 25 (k_flaperon_right) | **Right arm** (firmware-mixed) |

## Bridge topics (Gazebo)

| Topic | Type | Direction | Notes |
|-------|------|-----------|-------|
| `/paraglider_uav/command/motor_speed` | Actuators | bridge → gz | Motor RPM 0..1500 |
| `/arm/left`  | Double | bridge → gz | Left arm angle (rad, 0..0.5) |
| `/arm/right` | Double | bridge → gz | Right arm angle (rad, 0..0.5) |
| `/world/.../dynamic_pose/info` | Pose_V | gz → bridge | Pose for AP state |
| `/world/.../imu` | IMU | gz → bridge | Gyro+accel for AP state |
| UDP 9002 | binary | AP → bridge | servo_packet_16 |
| UDP src-addr (auto) | JSON | bridge → AP | imu/vel/pos/att (NED) |

## Build / run

```bash
# Rebuild firmware
cd ~/ardupilot && ./waf plane

# Stack
gz sim -s ~/paraglider_sim/sdf/world.sdf -v 2 &
python3 ~/paraglider_sim/firmware/bridge/paraglider_bridge.py &
~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON \
  --add-param-file=~/paraglider_sim/firmware/params/paraglider.parm \
  --no-mavproxy -L CMAC &
mavproxy.py --master=tcp:127.0.0.1:5760 \
  --out=udpout:127.0.0.1:14550 \
  --out=udpout:127.0.0.1:14551 \
  --daemon &
gz sim -g &     # GUI
open -a /Users/sanju/Desktop/QGroundControl.app
```

## Modes you can test

| Mode | What it does | Paraglider implementation |
|------|--------------|---------------------------|
| MANUAL | Pass RC stick to servos | Direct stick → arms |
| STABILIZE | Auto-level roll & pitch | AP runs PIDs, mixer drives arms |
| FBWA | Roll-stabilized; pitch-up cmd → climb | TECS + arm mixer |
| FBWB | FBWA + altitude hold | TECS holds alt via throttle, mixer turns vehicle |
| CRUISE | Heading + altitude hold | L1 + TECS + arms |
| AUTO | Waypoint mission | Full mission system |
| RTL | Return to launch | Standard plane RTL |
| LOITER | Circle a point | Standard plane LOITER |
| GUIDED | Mission Planner / QGC click-to-fly | Standard plane GUIDED |

## Tuning notes (paraglider-specific)

- `RLL_RATE_FF = 0.4`, `RLL_RATE_P = 0.10` — feed-forward dominant (slow servo response)
- `PTCH_RATE_FF = 0.20`, `PTCH_RATE_P = 0.30` — moderate pitch authority via arm pull
- `TECS_CLMB_MAX = 1.0` m/s — slow climb (paramotor reality)
- `TRIM_ARSPD_CM = 500` — 5 m/s cruise (small RC paramotor)
- `INS_GYRO_FILTER = 15` — vibration mitigation
- `NAVL1_PERIOD = 22` — slow vehicle, long L1 distance

## Source files

| File | Purpose |
|------|---------|
| `~/ardupilot/ArduPlane/servos.cpp` | C++ paraglider arm mixer (see lines ~963-1003) |
| `firmware/params/paraglider.parm` | ArduPlane parameter set for paraglider config |
| `firmware/bridge/paraglider_bridge.py` | Gazebo Harmonic ↔ AP SITL UDP-JSON bridge |
| `sdf/model.sdf` / `sdf/world.sdf` | Gazebo paraglider model + world |

## Why ArduPlane not custom vehicle class

Building a brand-new `ArduParaglider/` vehicle directory would mean re-implementing:
- 50+ flight modes
- EKF integration
- TECS, L1, mission, geofence, RTL
- 1000+ params
- All MAVLink message handlers

Adding ~30 lines to ArduPlane's mixer keeps every feature working AND adds paraglider control law. Industry-standard approach for derivative airframes (ArduPlane already supports flying-wing, delta, vtail, quadplane via in-tree mixers).
