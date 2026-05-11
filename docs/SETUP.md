# Full Setup Guide

End-to-end instructions to clone + build + run PARAGLIDER-RTG on **macOS** or **Ubuntu/Linux**.

## Prerequisites

- 8GB+ RAM
- 10GB+ disk space (ArduPilot build + dependencies)
- macOS 12+ or Ubuntu 22.04+
- Internet connection (for cloning + apt/brew installs)

## Quick Setup (Automated)

```bash
git clone https://github.com/sanjubhai-oto/PARAGLIDER-RTG.git
cd PARAGLIDER-RTG
./setup.sh
```

The script auto-detects macOS vs Linux and:
1. Installs Gazebo Harmonic + Python deps
2. Clones + builds ArduPilot SITL
3. Applies the paraglider firmware patch
4. Symlinks the model to `~/.gz/models/paraglider_uav`
5. Prints next steps for QGroundControl

After setup completes:
```bash
./launch.sh           # start full stack
# In QGroundControl: connect to UDP 14550, arm, fly
./kill.sh             # stop everything
```

---

## Manual Setup (if `setup.sh` fails)

### macOS

```bash
# 1. Homebrew + Gazebo Harmonic
brew install gz-harmonic python@3 git wget

# 2. Python packages
python3 -m pip install --user pymavlink MAVProxy
# gz Python bindings come with brew gz-harmonic

# 3. ArduPilot
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-mac.sh -y

# 4. Apply paraglider patch
git apply ~/PARAGLIDER-RTG/firmware/ardupilot_patch/servos.cpp.patch
./waf configure --board sitl
./waf plane

# 5. Symlink model to gz model path
mkdir -p ~/.gz/models
ln -s ~/PARAGLIDER-RTG ~/.gz/models/paraglider_uav

# 6. QGroundControl: download from https://qgroundcontrol.com
```

### Ubuntu 22.04+ / Debian

```bash
# 1. Gazebo Harmonic repo
sudo apt-get update
sudo apt-get install -y wget lsb-release gnupg curl
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
    -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    | sudo tee /etc/apt/sources.list.d/gazebo-stable.list
sudo apt-get update

# 2. Install Gazebo + Python + build tools
sudo apt-get install -y gz-harmonic python3 python3-pip git build-essential cmake

# 3. Python packages
python3 -m pip install --user pymavlink MAVProxy
python3 -m pip install --user gz-transport13 gz-msgs10

# 4. ArduPilot
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y

# 5. Apply paraglider patch
git apply ~/PARAGLIDER-RTG/firmware/ardupilot_patch/servos.cpp.patch
./waf configure --board sitl
./waf plane

# 6. Symlink model
mkdir -p ~/.gz/models
ln -s ~/PARAGLIDER-RTG ~/.gz/models/paraglider_uav

# 7. QGroundControl
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x QGroundControl.AppImage
```

---

## Running

```bash
cd ~/PARAGLIDER-RTG
./launch.sh
```

Stack starts in order: `gz server → gz GUI → bridge → ArduPlane SITL → forwarder → params applied`.

Then open QGroundControl and connect to UDP 14550 (default GCS port).

## Stopping

```bash
./kill.sh
```

Kills all sim processes, frees ports.

---

## Verification

After `launch.sh`, verify:

```bash
# Heartbeat
python3 -c "
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:127.0.0.1:14551')
hb = m.wait_heartbeat(timeout=5)
print(f'mode={hb.custom_mode} armed={(hb.base_mode & 128) != 0}' if hb else 'NO HEARTBEAT')
"
# Expected: mode=12 (LOITER), armed=False

# Vehicle pose
gz model -m paraglider_uav --pose
# Expected: at (0, 0, ~0.10), RPY ~0

# Processes alive
ps aux | grep -E "gz sim|paraglider_bridge|arduplane|mav_forw" | grep -v grep | awk '{print $11, $12}'
# Expected: 5 processes (gz server, gz GUI, bridge, arduplane, mav_forwarder)
```

---

## Flying

In QGroundControl:
1. **Switch mode** — MANUAL or LOITER (firmware override means throttle works in either)
2. **Click Arm**
3. **Advance throttle stick** → vehicle accelerates forward
4. **Pitch back (stick toward you)** → climb assist
5. **Pitch forward (push stick away)** → brake/dive
6. **Roll left/right** → bank turn (differential brake arms)

Vehicle should NOT flip thanks to auto-level torques from bridge. If it does, see [TROUBLESHOOTING.md](TROUBLESHOOTING.md).

---

## Updating

After `git pull`:

```bash
cd ~/PARAGLIDER-RTG
# If firmware patch changed:
cd ~/ardupilot
git stash             # save any local edits
git apply ~/PARAGLIDER-RTG/firmware/ardupilot_patch/servos.cpp.patch
./waf plane
```

---

## File Layout

```
PARAGLIDER-RTG/
├── README.md                       # main project README + control logic
├── setup.sh                        # cross-platform installer
├── launch.sh                       # start full stack
├── kill.sh                         # stop everything
├── .gitignore
├── sdf/
│   ├── world.sdf                   # Gazebo world (physics, ground, plugins)
│   ├── model.sdf                   # vehicle model (links, joints, sensors)
│   └── model.config                # gz model metadata
├── firmware/
│   ├── bridge/
│   │   └── paraglider_bridge.py    # ArduPilot ↔ Gazebo bridge + virtual physics
│   ├── scripts/
│   │   └── mav_forwarder.py        # TCP↔UDP MAVLink forwarder
│   ├── params/
│   │   └── paraglider.parm         # ArduPilot static params
│   └── ardupilot_patch/
│       ├── servos.cpp.patch        # custom mixer patch
│       └── README.md
├── meshes/                         # STL + DAE assets (Fusion 360 export)
├── scripts/                        # utilities (STEP→STL conversion, etc.)
└── docs/
    ├── images/                     # README screenshots + renders
    ├── README.md
    ├── ARCHITECTURE.md
    ├── PHYSICS.md
    ├── FIRMWARE.md
    ├── BRIDGE.md
    ├── LAUNCH.md
    ├── PARAMS.md
    ├── TROUBLESHOOTING.md
    ├── SETUP.md                    # this file
    └── CHANGELOG.md
```

---

## Common Issues

| Symptom | Fix |
|---------|-----|
| `gz: command not found` | Install Gazebo Harmonic via brew/apt — see Prerequisites |
| `arduplane binary missing` | `cd ~/ardupilot && ./waf plane` |
| `Patch failed to apply` | Inspect `~/ardupilot/ArduPlane/servos.cpp.rej` — manually merge |
| `Address already in use` | `./kill.sh` first |
| QGC can't connect | Check forwarder is running: `ps aux \| grep mav_forw` |

More in [TROUBLESHOOTING.md](TROUBLESHOOTING.md).
