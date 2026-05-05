#!/usr/bin/env bash
# One-button demo arm. The cart MUST already be launched (see README).
set -euo pipefail
echo "Arming in 3..."
sleep 1; echo "2..."; sleep 1; echo "1..."; sleep 1
ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: true"
echo "Demo started. Watch:"
echo "  ros2 topic echo /cart_safety/status"
echo "Press any key to STOP."
read -n 1
ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: false"
echo "Demo stopped."
