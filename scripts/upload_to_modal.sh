#!/usr/bin/env bash
# Send an NMEA log to Modal, get back waypoints.yaml.
# Usage: ./scripts/upload_to_modal.sh ~/cart_recordings/gps.log route_oval
set -euo pipefail
LOG="${1:?path to nmea log}"
ROUTE="${2:?route name}"
modal run modal_pipeline/process_route.py::main \
  --nmea-log "$LOG" --route-name "$ROUTE"
mkdir -p "./out/$ROUTE"
modal volume get stanford-cart-data "$ROUTE" "./out/"
echo "Done. Open ./out/$ROUTE/route_preview.html"
