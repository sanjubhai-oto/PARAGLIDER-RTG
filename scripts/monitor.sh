#!/bin/bash
# Real-time paraglider telemetry. Prints pose, velocity, joints every 0.5s.
set -u
WORLD=paraglider_world
MODEL=paraglider_uav

printf "%-7s %-22s %-22s %-18s %-18s\n" "t(s)" "pos(x y z)" "rpy(r p y)" "lin_vel" "ang_vel"
echo "------------------------------------------------------------------------------------------"

start=$(date +%s.%N)
while true; do
  now=$(date +%s.%N)
  t=$(awk "BEGIN{printf \"%.2f\", $now-$start}")

  # Pose
  pose=$(gz model -m $MODEL --pose 2>/dev/null | grep -A2 "Pose \[" | head -3)
  pos=$(echo "$pose" | sed -n '2p' | tr -d '[]' | xargs)
  rpy=$(echo "$pose" | sed -n '3p' | tr -d '[]' | xargs)

  # Velocity from dynamic_pose (computed by GUI broadcaster). Use gz topic for instant snapshot.
  lin=$(timeout 0.3 gz topic -e -t /world/$WORLD/dynamic_pose/info -n 1 2>/dev/null \
    | awk '/name: "paraglider_uav"/{found=1} found && /position/{getline; getline; exit}')

  printf "%-7s %-22s %-22s\n" "$t" "$pos" "$rpy"
  sleep 0.5
done
