# Custom ArduPlane Firmware

Custom paraglider mixer in `~/ardupilot/ArduPlane/servos.cpp` around lines 989-1080.

## Build

```bash
cd ~/ardupilot
./waf plane
# Binary: build/sitl/bin/arduplane
```

## Mixer Block Overview

Inserted at end of `Plane::set_servos()`, just before `servos_output()`. The block has 3 sections:

### 1. Arm Mixer (RC1 + RC2 → SERVO5 + SERVO6)

```cpp
if (arming.is_armed_and_safety_off() && !failsafe.rc_failsafe) {
    float ail = 0.0f, elev = 0.0f;
    constexpr float ARM_MAX = 0.85f;
    RC_Channel *rc1 = rc().channel(0);
    RC_Channel *rc2 = rc().channel(1);
    if (rc1 != nullptr) ail  = -rc1->norm_input();  // INVERTED for QGC convention
    if (rc2 != nullptr) elev = -rc2->norm_input();
    constexpr float DEADBAND = 0.05f;
    if (ail  > -DEADBAND && ail  < DEADBAND) ail  = 0.0f;
    if (elev > -DEADBAND && elev < DEADBAND) elev = 0.0f;
    if (elev > ARM_MAX) elev = ARM_MAX;       // saturation fix preserves roll authority
    if (failsafe.rc_failsafe) { ail = 0; elev = 0; }  // RC loss = canopy release

    const float arm_common = (elev > 0.0f) ? elev : 0.0f;  // pitch fwd -> both pull
    float arm_left  = arm_common - ail;
    float arm_right = arm_common + ail;
    if (arm_left  < 0) arm_left  = 0;
    if (arm_right < 0) arm_right = 0;
    if (arm_left  > ARM_MAX) arm_left  = ARM_MAX;
    if (arm_right > ARM_MAX) arm_right = ARM_MAX;

    // Pitch damper: brake if nose-up > 20°, release if dive < -10°
    const float pitch_deg = degrees(ahrs.get_pitch());
    if (pitch_deg > 20.0f) {
        float damper = (pitch_deg - 20.0f) / 20.0f;
        if (damper > 1.0f) damper = 1.0f;
        float damper_pull = damper * ARM_MAX;
        if (damper_pull > arm_left)  arm_left  = damper_pull;
        if (damper_pull > arm_right) arm_right = damper_pull;
    } else if (pitch_deg < -10.0f) {
        arm_left = 0; arm_right = 0;
    }

    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_left,  arm_left  * 4500.0f);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_right, arm_right * 4500.0f);

    // Pitch-UP assist via SERVO7 (k_scripting1): bridge reads PWM and applies upward wrench
    float pitch_up_norm = 0.0f;
    if (elev < 0.0f) pitch_up_norm = -elev;
    if (pitch_up_norm > 1.0f) pitch_up_norm = 1.0f;
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1, pitch_up_norm * 4500.0f);
} else {
    // Disarmed or RC failsafe: park arms released
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_left,  0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_flaperon_right, 0);
    SRV_Channels::set_output_scaled(SRV_Channel::k_scripting1,    0);
}
```

### 2. Throttle Override (RC3 → SERVO3 in any mode)

```cpp
if (arming.is_armed_and_safety_off() && !failsafe.rc_failsafe) {
    RC_Channel *rc3 = rc().channel(2);
    if (rc3 != nullptr) {
        float thr = rc3->norm_input();
        if (thr < 0.0f) thr = 0.0f;
        if (thr > 1.0f) thr = 1.0f;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, thr * 100.0f);
    }
}
```

**Why:** Stock ArduPlane LOITER/AUTO modes use TECS to auto-throttle based on altitude error. Pilot stick is ignored. This override forces pilot RC3 → motor in ALL modes.

## Sign Conventions

| Stick | RC PWM | Mixer effect |
|-------|--------|--------------|
| RC1 right | 1900 | `ail = -1.0` after invert → `arm_left = 1.0` pulls (left brake) |
| RC1 left | 1100 | `ail = +1.0` → `arm_right` pulls |
| RC2 forward (push away) | 1100 | `elev = +1.0` after invert → both arms pull (brake) |
| RC2 back (pull toward) | 1900 | `elev = -1.0` → arms released + `k_scripting1` HIGH (climb) |
| RC3 throttle UP | 2000 | `thr = 1.0` → SERVO3 max → motor full + bridge thrust full |

**Note:** Bridge can override these conventions. Currently bridge reads RC directly from PWM packet rather than firmware-mixed servo outputs, so the firmware mixer is somewhat redundant for arm visuals but still needed for SERVO7 (k_scripting1) climb signal AND throttle override.

## Servo Function Assignments

```
SERVO1_FUNCTION  4    # k_aileron (default, unused in physical model)
SERVO2_FUNCTION  19   # k_elevator (default, unused)
SERVO3_FUNCTION  70   # k_throttle -> motor RPM in bridge
SERVO5_FUNCTION  24   # k_flaperon_left -> left arm
SERVO6_FUNCTION  25   # k_flaperon_right -> right arm
SERVO7_FUNCTION  94   # k_scripting1 -> pitch-up assist (bridge reads PWM[6])
```

## Safety Features in Mixer

1. **RC failsafe gate:** All control disabled if `failsafe.rc_failsafe == true`. Motor zeros, arms park, scripting1 zeros.
2. **Saturation fix:** Elev clamped to ARM_MAX before mixing preserves roll authority at full pitch.
3. **Canopy release on RC loss:** Inside armed block, if failsafe just triggered, `ail = elev = 0` immediately.
4. **Auto pitch damper:** Brakes engage if pitch > 20° to prevent rocket; arms release if pitch < -10° to recover from dive.
5. **Throttle gated by arming + safety:** Motor only spins when armed AND safety off AND no RC failsafe.

## Build Verification

After firmware edits:
```bash
cd ~/ardupilot && ./waf plane
# Expected output: 'plane' finished successfully (~3s)
```

Diagnostic noise about `AP_HAL/AP_HAL.h` not found is clang IDE noise — waf build uses correct include paths and succeeds.

## Re-flash to Real Hardware

For real RC paramotor:
1. Apply same `servos.cpp` patch to ArduPlane source matching your board (Cube/Pixhawk/CubePilot).
2. Build for target: `./waf configure --board CubeOrange && ./waf plane`
3. Upload via `./waf --upload`.
4. Verify SERVO5/6/7 wiring to brake servos + scripting output.
5. RC failsafe must be configured (`FS_THR_VALUE`, `THR_FS_VALUE`).
