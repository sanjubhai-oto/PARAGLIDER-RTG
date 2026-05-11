# Architecture

## Stack Components

```
┌─────────────────┐  UDP 14550   ┌──────────────────┐  TCP 5760  ┌──────────────────┐
│ QGroundControl  │ ←──────────→ │  mav_forwarder   │ ←────────→ │   ArduPlane      │
│  (macOS app)    │              │   (Python)       │            │   SITL binary    │
└─────────────────┘              └──────────────────┘            └──────────────────┘
                                                                          │
                                                                          │ UDP 9002
                                                                          │ (SITL JSON)
                                                                          ↓
┌─────────────────┐              ┌──────────────────┐            ┌──────────────────┐
│  gz sim -g      │ ←─ TCP gz ── │  paraglider_     │ ←─UDP─→    │   bridge sends   │
│  (3D GUI)       │              │  bridge.py       │            │   PWM out,       │
└─────────────────┘              │                  │            │   reads sensors  │
       ↑                         │  - reads PWM     │            └──────────────────┘
       │                         │  - publishes     │
       │                         │    motor speed   │
       │                         │  - applies wrench│
       │                         │  - subscribes    │
       │                         │    pose, IMU     │
       │                         └──────────────────┘
       │                                  ↓
       │                                  │ gz topics
       │                                  ↓
┌─────────────────┐                       │
│  gz sim -s      │ ←─────────────────────┘
│  (physics)      │
│  - DART engine  │
│  - LiftDrag OFF │ (disabled — bridge handles aero virtually)
│  - ApplyLinkWrench plugin
└─────────────────┘
```

## Data Flow

### Pilot → Vehicle

1. **QGC** sends RC override packets via UDP to `localhost:14550`
2. **mav_forwarder.py** forwards UDP→TCP to ArduPlane on `localhost:5760`
3. **ArduPlane** processes RC inputs through custom `servos.cpp` mixer:
   - Reads `rc().channel(N)->norm_input()` directly (raw RC, bypasses AP attitude controller)
   - Computes arm positions: roll → differential, pitch-down → both pull
   - Sets `k_flaperon_left`, `k_flaperon_right`, `k_scripting1` servo outputs
   - Always overrides `k_throttle` from RC3 (bypasses TECS auto-throttle)
4. ArduPlane sends 16 PWM values via UDP 9002 to bridge
5. **paraglider_bridge.py** unpacks PWM, reads:
   - `pwm[0]` aileron (roll) — for direct lateral force + roll torque + arm visuals
   - `pwm[1]` elevator (pitch) — for pitch-up/down force + arm visuals
   - `pwm[2]` throttle — for forward thrust + climb force + motor RPM
   - `pwm[3]` rudder — unused
   - `pwm[4]/[5]` arm servos — fallback for visual deflection
   - `pwm[6]` scripting1 — pitch-up assist from firmware (currently unused; bridge reads RC2 directly)
6. Bridge publishes:
   - `gz.msgs.Actuators` to `/paraglider_uav/command/motor_speed` — propeller spin (visual only, motorConstant=0)
   - `gz.msgs.Double` to `/arm/left`, `/arm/right` — joint position commands for arms
   - `gz.msgs.EntityWrench` to `/world/paraglider_world/wrench` — bounded force + torque on fuselage link

### Vehicle → Pilot

1. **Gazebo** simulates physics with DART engine
2. Bridge subscribes to:
   - `/world/paraglider_world/dynamic_pose/info` — vehicle pose (position + orientation)
   - `/world/.../imu_sensor/imu` — IMU readings (gyro + accel)
3. Bridge constructs SITL JSON payload (timestamp, IMU, velocity, attitude, lat/lon/alt)
4. Sends JSON back to ArduPlane via UDP 9002 (same socket, reply to AP's source addr)
5. ArduPlane processes sensor data through EKF3 (with `AHRS_EKF_TYPE=10` for SITL backend)
6. Telemetry (HEARTBEAT, ATTITUDE, GPS, SYS_STATUS) sent to QGC via forwarder

## Port Assignments

| Port | Protocol | Source → Dest | Purpose |
|------|----------|---------------|---------|
| 5760 | TCP | forwarder ↔ ArduPlane | MAVLink |
| 9002 | UDP | bridge ↔ ArduPlane | SITL JSON (PWM out, sensors in) |
| 14550 | UDP | QGC ↔ forwarder | MAVLink GCS |
| 14551 | UDP | test scripts ↔ forwarder | MAVLink test channel |
| 14552 | UDP | forwarder bound for QGC replies | MAVLink |

## Critical Design Decisions

### 1. DART physics, not Bullet Featherstone
Bullet Featherstone caused massive drift (vehicle slid 0.4 m/s sideways at idle) because it ignores `<velocity_decay>` and mishandles friction on articulated bodies. DART works correctly.

### 2. Bounded thrust via ApplyLinkWrench
Original `MulticopterMotorModel` with `motorConstant·ω²` was numerically unstable. Tiny `motorConstant` changes (0.001 → 0.01) caused 100× trajectory differences. Replaced with explicit `EntityWrench` published by bridge — linear in throttle, capped at `THRUST_FWD_MAX`.

### 3. Bridge handles aero (LiftDrag plugins disabled)
`gz-sim-lift-drag-system` produces unbounded lift force at high speed (force ∝ v²) leading to rocket-to-orbit behavior. Bridge instead applies:
- Constant `VEHICLE_WEIGHT` upward force (cancels gravity, simulates inflated canopy)
- Variable upward from throttle/pitch input
- Quadratic body-frame drag (real damping)

### 4. Auto-level in bridge, not firmware
World-frame restoring torques: `τ = -K·attitude_error - D·angular_velocity`. Prevents tumble. Active in bridge at 50 Hz, no firmware PID needed.

### 5. Custom firmware mixer reads raw RC
ArduPlane normally routes RC through attitude controller. Custom `servos.cpp` block reads `rc().channel(N)->norm_input()` directly. Arms respond to RC even in LOITER mode. Throttle override forces RC3 to motor regardless of mode.

### 6. PWM watchdog in bridge
If no PWM packet from AP for 0.5s, bridge publishes zero wrench. Prevents vehicle drifting kilometers when AP dies.

## Process Lifecycle

| Process | Started by | Killed on exit | Restart on death |
|---------|------------|-----------------|------------------|
| `gz sim -s` (server) | manual launch | independent | manual |
| `gz sim -g` (GUI) | manual launch | independent | manual |
| `paraglider_bridge.py` | manual launch | independent | manual (bridge logs to `/tmp/bridge.log`) |
| `arduplane` | `sim_vehicle.py` wrapper | dies if parent dies | restart `sim_vehicle.py` |
| `sim_vehicle.py` | manual launch | parent process — keep alive | shouldn't exit if launched with `&` + `disown` |
| `mav_forwarder.py` | manual launch | independent | restart manually |
| QGroundControl | user | manual | reopen + reconnect |

**Critical:** `arduplane` binary dies if launched directly (no parent). Always use `sim_vehicle.py` wrapper.
