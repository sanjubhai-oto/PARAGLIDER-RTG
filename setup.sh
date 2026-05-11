#!/usr/bin/env bash
# PARAGLIDER-RTG setup script
# Auto-detects macOS / Ubuntu and installs all dependencies.
# Run once after `git clone`.

set -e

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "==> PARAGLIDER-RTG setup"
echo "    Project: $PROJECT_DIR"

OS="$(uname -s)"
case "$OS" in
    Darwin*) PLATFORM="macos" ;;
    Linux*)  PLATFORM="linux" ;;
    *) echo "Unsupported OS: $OS"; exit 1 ;;
esac
echo "    Platform: $PLATFORM"

# -----------------------------------------------------------------------------
# 1. System dependencies
# -----------------------------------------------------------------------------
echo ""
echo "==> Step 1/5: System dependencies"

if [ "$PLATFORM" = "macos" ]; then
    if ! command -v brew &>/dev/null; then
        echo "ERROR: Homebrew not installed. Install from https://brew.sh"
        exit 1
    fi
    echo "Installing via Homebrew..."
    brew install --quiet gz-harmonic python@3 git wget curl || true
elif [ "$PLATFORM" = "linux" ]; then
    if ! command -v apt-get &>/dev/null; then
        echo "ERROR: apt-get not found. This setup supports Ubuntu/Debian."
        echo "For other distros, install: gazebo-harmonic, python3, python3-pip, git manually."
        exit 1
    fi
    echo "Adding Gazebo Harmonic repo..."
    sudo apt-get update
    sudo apt-get install -y wget lsb-release gnupg curl
    sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list >/dev/null
    sudo apt-get update
    sudo apt-get install -y gz-harmonic python3 python3-pip git build-essential cmake \
        python3-numpy python3-pyparsing python3-psutil python3-future python3-lxml
fi

# -----------------------------------------------------------------------------
# 2. Python packages
# -----------------------------------------------------------------------------
echo ""
echo "==> Step 2/5: Python packages"
python3 -m pip install --user --upgrade pymavlink MAVProxy

# Gazebo Python bindings (gz-transport13, gz-msgs10)
if [ "$PLATFORM" = "macos" ]; then
    # macOS: provided by brew gz-harmonic
    echo "macOS: gz Python bindings come with brew package"
elif [ "$PLATFORM" = "linux" ]; then
    # Ubuntu: install gz Python bindings via pip if available
    python3 -m pip install --user gz-transport13 gz-msgs10 2>/dev/null || \
        echo "WARN: gz-transport13/gz-msgs10 Python bindings not on PyPI for this Python. May need to build from source."
fi

# -----------------------------------------------------------------------------
# 3. ArduPilot SITL
# -----------------------------------------------------------------------------
echo ""
echo "==> Step 3/5: ArduPilot SITL"

ARDUPILOT_DIR="$HOME/ardupilot"
if [ ! -d "$ARDUPILOT_DIR" ]; then
    echo "Cloning ArduPilot..."
    git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git "$ARDUPILOT_DIR"
else
    echo "ArduPilot already at $ARDUPILOT_DIR"
fi

cd "$ARDUPILOT_DIR"
if [ -f Tools/environment_install/install-prereqs-ubuntu.sh ] && [ "$PLATFORM" = "linux" ]; then
    echo "Running ArduPilot Ubuntu prereqs..."
    Tools/environment_install/install-prereqs-ubuntu.sh -y || true
fi
if [ -f Tools/environment_install/install-prereqs-mac.sh ] && [ "$PLATFORM" = "macos" ]; then
    echo "Running ArduPilot macOS prereqs..."
    Tools/environment_install/install-prereqs-mac.sh -y || true
fi

# Apply paraglider patch
echo "Applying paraglider servos.cpp patch..."
if ! grep -q "ArduParaglider arm-control mixer" ArduPlane/servos.cpp 2>/dev/null; then
    git apply "$PROJECT_DIR/firmware/ardupilot_patch/servos.cpp.patch" || {
        echo "WARN: Patch failed to apply cleanly. Inspect rejects in ArduPlane/servos.cpp.rej"
    }
else
    echo "Patch already applied"
fi

echo "Building ArduPlane SITL..."
./waf configure --board sitl
./waf plane

# -----------------------------------------------------------------------------
# 4. Gazebo model install
# -----------------------------------------------------------------------------
echo ""
echo "==> Step 4/5: Gazebo model"

GZ_MODELS="$HOME/.gz/models"
mkdir -p "$GZ_MODELS"
if [ ! -L "$GZ_MODELS/paraglider_uav" ] && [ ! -d "$GZ_MODELS/paraglider_uav" ]; then
    echo "Symlinking model to $GZ_MODELS/paraglider_uav..."
    ln -s "$PROJECT_DIR" "$GZ_MODELS/paraglider_uav"
fi

# -----------------------------------------------------------------------------
# 5. QGroundControl
# -----------------------------------------------------------------------------
echo ""
echo "==> Step 5/5: QGroundControl"
echo "Download from: https://qgroundcontrol.com/downloads/"
if [ "$PLATFORM" = "macos" ]; then
    echo "macOS: Drag QGroundControl.app to /Applications or ~/Desktop"
elif [ "$PLATFORM" = "linux" ]; then
    echo "Linux: Download the AppImage and make executable: chmod +x QGroundControl.AppImage"
fi

# -----------------------------------------------------------------------------
echo ""
echo "============================================"
echo "  Setup complete!"
echo "============================================"
echo ""
echo "Launch the simulator with:"
echo "  cd $PROJECT_DIR"
echo "  ./launch.sh"
echo ""
echo "Then open QGroundControl and connect to UDP 14550."
