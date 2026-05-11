# Paraglider SITL Simulator

Autonomous RC paramotor SITL stack: **Gazebo Harmonic + ArduPilot SITL + custom firmware mixer + Python bridge**.

## What This Is

End-to-end software-in-the-loop simulator for a small RC paramotor (0.38 kg vehicle, pusher prop, 2 brake-line "arms"). The vehicle responds to:

- **Throttle (RC3)** → motor RPM (visual) + bounded forward thrust + slight upward
- **Roll (RC1)** → bank left/right via differential brake arms + lateral force
- **Pitch (RC2)** → stick BACK = climb assist (upward force + nose-up torque); stick FORWARD = brake/dive

Vehicle armed via QGroundControl on UDP 14550.

## Why Custom

Standard ArduPlane assumes fixed-wing dynamics with elevator/rudder. Paragliders steer via asymmetric brake lines and have no traditional pitch/yaw control surfaces. This stack:

- Replaces stock ArduPlane fixed-wing mixer with custom paraglider mixer in `servos.cpp`
- Bypasses ArduPilot's auto-throttle (LOITER/TECS) so pilot stick directly drives motor in ANY mode
- Replaces unstable `MulticopterMotorModel` (force = motorConstant·ω², unbounded) with bounded `EntityWrench` from Python bridge
- Auto-level controller in bridge prevents tumbling without active stabilization in firmware

## Quick Start

```bash
# 1. Kill any stale processes
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|sim_vehicle|mav_forw" | grep -v grep | awk '{print $2}' | xargs -I{} kill -9 {}
for port in 14550 14551 14552 5760 9002; do
  for p in $(lsof -iTCP:$port -t) $(lsof -iUDP:$port -t); do kill -9 $p 2>/dev/null; done
done

# 2. Launch stack (in order)
export GZ_SIM_RESOURCE_PATH=$HOME/.gz/models:$GZ_SIM_RESOURCE_PATH
gz sim -s -r ~/paraglider_sim/sdf/world.sdf -v 2 > /tmp/gz.log 2>&1 &        # server
sleep 5
gz sim -g > /tmp/gz_gui.log 2>&1 &                                            # GUI
sleep 4
python3 -u ~/paraglider_sim/firmware/bridge/paraglider_bridge.py > /tmp/bridge.log 2>&1 &
sleep 2
cd /tmp && ~/ardupilot/Tools/autotest/sim_vehicle.py -v ArduPlane -f JSON \
  --add-param-file=$HOME/paraglider_sim/firmware/params/paraglider.parm \
  --no-mavproxy --no-rebuild --wipe-eeprom -L CMAC > /tmp/ap.log 2>&1 &
disown
sleep 14
python3 -u ~/paraglider_sim/firmware/scripts/mav_forwarder.py > /tmp/fwd.log 2>&1 &
sleep 4

# 3. Apply runtime params via MAVLink (see PARAMS.md)
# 4. Connect QGroundControl to UDP 14550
# 5. Arm + advance throttle
```

Detailed launch in [LAUNCH.md](LAUNCH.md).

## Documentation

| File | Topic |
|------|-------|
| [ARCHITECTURE.md](ARCHITECTURE.md) | Full stack diagram, data flow, port assignments |
| [PHYSICS.md](PHYSICS.md) | `model.sdf` parameters, mass/inertia, joints, aero |
| [FIRMWARE.md](FIRMWARE.md) | Custom ArduPlane mixer in `servos.cpp` |
| [BRIDGE.md](BRIDGE.md) | Python bridge force/torque mapping, watchdog |
| [LAUNCH.md](LAUNCH.md) | Launch sequence, common commands, QGC setup |
| [PARAMS.md](PARAMS.md) | ArduPilot parameter reference |
| [TROUBLESHOOTING.md](TROUBLESHOOTING.md) | Known issues + fixes |
| [CHANGELOG.md](CHANGELOG.md) | Development timeline + lessons learned |

## Dependencies

- macOS (tested on Darwin 25.4.0) or Linux
- Gazebo Harmonic (`brew install gz-harmonic`)
- ArduPilot SITL (`~/ardupilot/` cloned + built with `./waf plane`)
- Python 3.14+ with `pymavlink`, `gz-transport13`, `gz-msgs10`
- QGroundControl

## Status

✅ Vehicle moves forward on throttle
✅ Arms respond to RC1/RC2 sticks (direct visual + flight control)
✅ Auto-level prevents tumbling
✅ Bounded thrust (no rocket-to-orbit)
✅ PWM watchdog (vehicle freezes if AP dies)
⚠️ Sustained level cruise needs further tuning (vehicle climbs but altitude oscillates)
⚠️ Active stabilization in firmware not implemented (open-loop physics + bridge auto-level only)

## License

Project files: MIT.
ArduPilot fork: GPLv3 (ArduPilot license).
