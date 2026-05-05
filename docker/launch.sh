#!/usr/bin/env bash
# launch — start the container and ros2-launch one of our targets in one shot.
#
# Usage:
#   ./launch.sh teleop                  # joy + safety + DBW
#   ./launch.sh sensors                 # 4 cameras + GPS
#   ./launch.sh dbw                     # ODrive + Mega + cmd_vel translator
#   ./launch.sh cart route_oval         # full autonomy, route at /data/route_oval/
#
# Assumes setup_isaac_ros.sh has been run and the workspace built.

set -euo pipefail

target="${1:-cart}"
route="${2:-route_oval}"

case "$target" in
  cart)     launch="cart_bringup cart.launch.py route_yaml:=/data/$route/waypoints.yaml" ;;
  teleop)   launch="cart_bringup teleop.launch.py" ;;
  sensors)  launch="cart_bringup sensors.launch.py" ;;
  dbw)      launch="cart_bringup dbw.launch.py" ;;
  *)
    echo "unknown target '$target'" >&2
    echo "valid: cart | teleop | sensors | dbw" >&2
    exit 1
    ;;
esac

ISAAC_ROS_WS="/mnt/nova_ssd/workspaces/isaac_ros-dev"
xhost +local:docker 2>/dev/null || true

exec sudo -E isaac-ros run -- bash -lc "
  cd $ISAAC_ROS_WS &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch $launch
"
