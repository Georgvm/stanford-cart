#!/usr/bin/env bash
# enter — start (or reuse) the Isaac ROS container and drop into a shell.

set -euo pipefail

ISAAC_ROS_WS="/mnt/nova_ssd/workspaces/isaac_ros-dev"
export ISAAC_ROS_WS

if ! command -v isaac-ros >/dev/null 2>&1; then
  echo "isaac-ros-cli not installed. Run ./setup_isaac_ros.sh first." >&2
  exit 1
fi

# Allow X11 from the container so RViz works if you run it from inside.
xhost +local:docker 2>/dev/null || true

# isaac-ros run handles --runtime nvidia / --gpus all / device passthrough.
# We just request a shell.
exec sudo -E isaac-ros run -- bash
