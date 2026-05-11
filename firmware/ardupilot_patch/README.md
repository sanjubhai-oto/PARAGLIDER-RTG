# ArduPilot Patch

`servos.cpp.patch` is the custom paraglider mixer patch against upstream ArduPlane `ArduPlane/servos.cpp`.

## Apply Patch

```bash
# Clone ArduPilot if not already done
cd ~
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot

# Apply patch
git apply ~/paraglider_sim/firmware/ardupilot_patch/servos.cpp.patch

# Build
./waf configure --board sitl
./waf plane
# Binary: build/sitl/bin/arduplane
```

## What the Patch Does

Inserts a custom mixer block at end of `Plane::set_servos()` (around line 958+) that:

1. **Reads raw RC** directly (bypasses ArduPlane attitude controller)
2. **Maps RC1/RC2 → brake arm servos** (`k_flaperon_left`, `k_flaperon_right`)
3. **Auto pitch damper** brakes at >20° pitch, releases at <-10°
4. **RC failsafe gate** zeros all control if RC signal lost
5. **Throttle override** forces pilot RC3 → `k_throttle` in ANY ArduPlane mode (bypasses LOITER TECS)
6. **SERVO7 climb signal** outputs pitch-up assist on `k_scripting1` for bridge to read

See [../../docs/FIRMWARE.md](../../docs/FIRMWARE.md) for full code walkthrough.

## Tested Against

ArduPilot master, commit `0bf95671bb` (May 2026). Patch should apply to any recent master, but inspect rejections if upstream `set_servos()` was refactored.

## Verifying Patch Applied

```bash
cd ~/ardupilot
grep -c "ArduParaglider arm-control mixer" ArduPlane/servos.cpp
# Should output: 1
```

## Building for Real Hardware

For real flight controller (Cube, Pixhawk, etc.):

```bash
./waf configure --board CubeOrange     # or your board
./waf plane
./waf --upload
```

Verify SERVO5/6/7 wired to brake servos + scripting output.
