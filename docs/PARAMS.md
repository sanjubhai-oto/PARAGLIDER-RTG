# ArduPilot Parameters Reference

## Source

Static params in `firmware/params/paraglider.parm` loaded by `sim_vehicle.py --add-param-file`.

Runtime params applied via MAVLink after AP boots (see LAUNCH.md step 7).

## Critical Runtime Params (apply via MAVLink)

| Param | Value | Purpose |
|-------|-------|---------|
| `SCHED_LOOP_RATE` | 25 | Main loop rate. Gyro check requires data ≥ rate × 1.8 = 45 Hz. Lower than default 50 to allow bridge ~250Hz IMU send |
| `ARMING_SKIPCHK` | 4194303 | Bitmask all bits set — skips all pre-arm checks |
| `ARMING_REQUIRE` | 1 | Allow arming (default) |
| `AHRS_EKF_TYPE` | 10 | **SITL fake AHRS** — bypasses EKF3 GPS sanity, uses bridge state directly |
| `INS_USE` | 1 | Use IMU 1 |
| `INS_USE2` | 0 | Don't use IMU 2 (kills "Gyros inconsistent" pre-arm error) |
| `INS_USE3` | 0 | Same |
| `COMPASS_USE` | 0 | Disable compass (sim mag inconsistencies) |
| `COMPASS_USE2` | 0 | |
| `COMPASS_USE3` | 0 | |
| `BRD_SAFETYENABLE` | 0 | Safety switch always off (no physical button in sim) |

**Note:** ArduPilot 4.5+ renamed `ARMING_CHECK` → `ARMING_SKIPCHK`. Old name not present in current builds.

## Static Params (`paraglider.parm`)

### Frame + Mode

```
FRAME_CLASS      0       # plane
ARMING_REQUIRE   1
FLTMODE1         0       # MANUAL
FLTMODE2         0       # MANUAL
FLTMODE3         2       # STABILIZE
FLTMODE4         5       # FBWA
FLTMODE5         6       # FBWB
FLTMODE6         12      # LOITER
FLTMODE_CH       8       # mode switch on RC8
```

### RC Calibration

```
RC1_MIN          1100
RC1_MAX          1900
RC1_TRIM         1500
RC2_MIN          1100
RC2_MAX          1900
RC2_TRIM         1500
RC3_MIN          1000
RC3_MAX          2000
RC3_TRIM         1500    # mid-stick = 0% throttle (SITL default)
```

### Servo Functions

```
SERVO1_FUNCTION  4       # k_aileron (unused physically)
SERVO2_FUNCTION  19      # k_elevator (unused)
SERVO3_FUNCTION  70      # k_throttle -> motor (bridge passthrough)
SERVO5_FUNCTION  24      # k_flaperon_left -> left arm
SERVO6_FUNCTION  25      # k_flaperon_right -> right arm
SERVO7_FUNCTION  94      # k_scripting1 -> pitch-up climb assist (bridge PWM[6])
```

### Servo Limits

```
SERVO3_MIN       1000
SERVO3_MAX       2000
SERVO3_TRIM      1000    # 0% throttle at trim
SERVO5_MIN       1000
SERVO5_MAX       2000
SERVO5_TRIM      1000    # arms idle at 1000us PWM
SERVO6_MIN       1000
SERVO6_MAX       2000
SERVO6_TRIM      1000
SERVO7_MIN       1000
SERVO7_MAX       2000
SERVO7_TRIM      1000
```

### Disable Components

```
COMPASS_ENABLE   0
EK3_ENABLE       0       # we use AHRS_EKF_TYPE=10 instead
EK2_ENABLE       0
```

### Auto-Throttle Tuning (irrelevant — firmware overrides)

```
TRIM_THROTTLE    40
THR_MIN          0
THR_MAX          100
```

## Why Pre-arm Bypassed

Real paramotor flying in sim hits these arming-block errors due to SITL quirks:

1. **"Arm: AHRS: not using configured AHRS type"** — EKF3 not ready or doesn't initialize properly with bridge JSON
   - Fix: `AHRS_EKF_TYPE=10` SITL fake
2. **"Arm: GPS 1: not healthy"** — fix_type=6 + 16 sats but health flag false (SITL quirk)
   - Fix: `ARMING_SKIPCHK=4194303` bypass
3. **"Arm: Gyros inconsistent"** — IMUs 2/3 disagree with IMU 1
   - Fix: `INS_USE2=0`, `INS_USE3=0` (single IMU only)
4. **"Arm: Compass not healthy"** — sim mag inconsistent
   - Fix: `COMPASS_USE=0`
5. **"PreArm: Gyro 0 rate 76Hz < loop rate*1.8 90Hz"** — bridge IMU send rate was 76Hz, threshold 90Hz
   - Fix: `SCHED_LOOP_RATE=25` (threshold becomes 45) + bridge non-blocking recv

After all bypasses, vehicle arms cleanly via `MAV_CMD_COMPONENT_ARM_DISARM`.

## Force-Arm Magic (last-resort bypass)

If standard arm fails, use force-arm magic:

```python
m.mav.command_long_send(
    m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0,
    1,        # arm
    21196,    # force-arm magic
    0, 0, 0, 0, 0
)
```

This bypasses arming checks via the magic value `21196`. Equivalent to disabling all checks.

## Verifying Param Loaded

```python
m.mav.param_request_read_send(m.target_system, m.target_component,
                              b'ARMING_SKIPCHK', -1)
msg = m.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
print(f"{msg.param_id.strip()} = {msg.param_value}")
```

Use type fallback `MAV_PARAM_TYPE_INT16/INT32/REAL32` if int param doesn't readback — ArduPilot is picky about types.

## Reboot AP via MAVLink

After setting `SCHED_LOOP_RATE` (requires reboot):

```python
m.mav.command_long_send(m.target_system, m.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(15)
m.wait_heartbeat()  # AP back up
```

**Warning:** Rebooting AP causes its TCP connection to drop. Forwarder reconnects automatically. Bridge keeps running. Vehicle should NOT drift due to PWM watchdog.
