#!/usr/bin/env bash
# Drive manually with the existing PS5 controller, record everything Modal needs.
# Usage: ./scripts/record_route.sh route_oval
set -euo pipefail
ROUTE="${1:-route_oval}"
OUT="$HOME/cart_recordings/${ROUTE}_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUT"
echo "Recording to: $OUT"
echo "Press Ctrl+C when route is done."
ros2 bag record \
  -o "$OUT/bag" \
  /fix \
  /front_wide/image_raw /front_wide/camera_info \
  /front_narrow/image_raw /front_narrow/camera_info \
  /left/image_raw /left/camera_info \
  /right/image_raw /right/camera_info \
  /tf /tf_static
