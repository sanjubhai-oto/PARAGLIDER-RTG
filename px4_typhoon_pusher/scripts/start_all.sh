#!/usr/bin/env bash
# Launch PX4 SITL (gazebo-classic typhoon_h480 + pusher) + dashboard, on macOS/Linux-with-classic.
# Prereqs: PX4-Autopilot built once, Gazebo Classic installed, apply_pusher.py already run.
set -u
PX4=${PX4:-$HOME/PX4-Autopilot}
HERE="$(cd "$(dirname "$0")" && pwd)"

echo "[start] killing stale sims"
pkill -f 'px4' 2>/dev/null; pkill -f 'gzserver' 2>/dev/null; pkill -f 'gzclient' 2>/dev/null
pkill -f 'dashboard.py' 2>/dev/null; sleep 2

echo "[start] launching PX4 SITL + Gazebo-classic typhoon+pusher"
cd "$PX4"
# HEADLESS=1 ./... for no GUI. Default shows the Gazebo Classic GUI window.
( make px4_sitl gazebo-classic_typhoon_h480_pusher >/tmp/px4_typhoon.log 2>&1 & )
echo "[start] waiting 40s for PX4 + Gazebo boot..."
sleep 40

echo "[start] launching dashboard on http://localhost:8088"
( python3 "$HERE/dashboard.py" >/tmp/typhoon_dashboard.log 2>&1 & )
sleep 2

cat <<EOF

============================================================
  Typhoon H480 + Pusher SITL is up.
  - Gazebo Classic GUI: the 3D view (this machine's screen)
  - Dashboard:          http://localhost:8088
  - QGroundControl:     connects automatically (UDP 14550)

  Fly it (auto takeoff -> hover -> MANUAL pusher):
      python3 $HERE/fly_manual.py

  In ALTITUDE mode, type a pusher % (0-100) to drive forward.
============================================================
EOF
