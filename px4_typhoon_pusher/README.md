# Typhoon H480 + Pusher — PX4 SITL (Gazebo Classic) for macOS

A **stock PX4 `typhoon_h480` hexacopter** with an added **rear pusher motor** that
gives forward velocity, plus **manual pusher control** after takeoff and a live
web dashboard.

> **Why run this on your Mac (not the cloud session):** `typhoon_h480` is a
> **Gazebo *Classic*** model — there is no gz-Harmonic typhoon in PX4. Gazebo
> Classic can't be installed on the Ubuntu 24.04 cloud container (EOL, no
> packages). Your Mac has the working PX4 + Gazebo setup, so this package is
> built to drop into your `PX4-Autopilot` checkout.

---

## What you get
- **Vehicle:** the unmodified `typhoon_h480` hexa-X (6 rotors, real PX4 mixer/EKF) …
- **…plus a pusher:** a 7th forward-facing motor on PX4 output 7, driven **manually**
  (not auto-coupled) via `MAV_CMD_DO_SET_ACTUATOR` (Offboard Actuator Set 1).
- **Flight flow:** auto arm → takeoff to 10 m → hover → switch to **ALTITUDE** mode →
  you type a pusher % to push forward (ALTITUDE holds height but lets the pusher
  move the aircraft; POSITION mode would fight it).
- **Dashboard:** `http://localhost:8088` — mode, alt, speed, attitude, RC, motor + pusher outputs.

## Files
```
px4_typhoon_pusher/
  scripts/
    apply_pusher.py    # adds the pusher to the typhoon model + writes the airframe
    fly_manual.py      # takeoff -> hover -> MANUAL pusher control (prompt)
    dashboard.py       # Flask webui on :8088
    start_all.sh       # launch PX4+Gazebo+dashboard
```

## Prerequisites (macOS)
```bash
# 1. PX4 toolchain + Gazebo Classic (PX4 docs: dev env on macOS)
#    https://docs.px4.io/main/en/dev_setup/dev_env_mac.html
brew install --cask gazebo            # or PX4's macos setup script
git clone https://github.com/PX4/PX4-Autopilot.git ~/PX4-Autopilot --recursive

# 2. Python deps for the helper + dashboard
pip3 install pymavlink flask
```

## Install the pusher (one time)
```bash
cd <this repo>/px4_typhoon_pusher/scripts
python3 apply_pusher.py --px4 ~/PX4-Autopilot
```
This patches `typhoon_h480.sdf.jinja` (adds the pusher rotor + motor plugin +
mavlink control channel, `input_index 6`), writes airframe
`6012_gazebo-classic_typhoon_h480_pusher`, and registers it in CMake.

## Run
```bash
cd <this repo>/px4_typhoon_pusher/scripts
PX4=~/PX4-Autopilot ./start_all.sh
# then, in another terminal:
python3 fly_manual.py
```
Open **http://localhost:8088** for the dashboard, and watch the Gazebo Classic
window. After it hovers, type e.g. `60` to spin the pusher to 60% → the hexa
accelerates forward; `land` to come down.

## Manual control notes
- **`fly_manual.py`** automates only takeoff/hover; the pusher is **yours** to
  drive at the prompt (`0`–`100`). For stick control of attitude, use a joystick
  in QGroundControl (Vehicle Setup → Joystick) while the pusher runs.
- The pusher is **not** in the attitude mixer — it's pure forward thrust, so it
  doesn't disturb roll/pitch/yaw (the hexa stabilizes itself).

## Tuning / gotchas (verify on your PX4 version)
- **Output function value:** the airframe sets `PWM_MAIN_FUNC7 2000`
  (`Offboard_Actuator_Set1`). If your PX4 build uses a different `OutputFunction`
  enum value, set it via **QGC → Actuators** or edit the airframe.
- **Pusher thrust:** tune `motorConstant` (1.2e-05) in the SDF pusher plugin for
  more/less forward thrust.
- **Pusher pose/direction:** the pusher link spin axis is body **+X** (forward).
  Adjust its `<pose>` in `apply_pusher.py` if you want it elsewhere.
- **Two MAVLink clients:** `fly_manual.py` uses UDP 14540, `dashboard.py` uses
  14550. If your PX4 SITL only streams one, point both at the same port.
