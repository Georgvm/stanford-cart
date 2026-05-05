#!/usr/bin/env bash
# cart_deps — install Python packages our nodes need that aren't in the
# base Isaac ROS image. Run INSIDE the container after `enter.sh`.
#
# What's NOT in the base image:
#   transformers, torch (CPU/CUDA), pillow  → for depth_anything_node
#   ultralytics (YOLOv8)                     → for perception_estop
#   pynmea2, pyserial                        → for gps_mega_node + record_nmea
#
# Torch on Jetson is special — must be the NVIDIA-built wheel for aarch64
# CUDA, not the upstream pip torch (which is x86_64-only or CPU-only).

set -euo pipefail

if [[ ! -f /etc/nv_tegra_release ]]; then
  echo "doesn't look like a Jetson — refusing to install Jetson torch wheels" >&2
  exit 1
fi

echo "[cart_deps] non-torch deps via pip"
pip install --upgrade pip
pip install \
  pynmea2 \
  pyserial \
  pillow \
  transformers \
  ultralytics \
  pyyaml

# Torch wheel for JP7 / Jazzy. NVIDIA publishes these on the developer index.
# If this URL ever 404s, grep `pip search torch` on https://pypi.jetson-ai-lab.dev
# or check https://forums.developer.nvidia.com/c/agx-orin/.
echo "[cart_deps] installing Jetson torch wheel"
pip install --extra-index-url https://pypi.jetson-ai-lab.dev/jp7/cu130 \
  torch torchvision || {
    echo "[cart_deps] Jetson torch wheel install failed."
    echo "  Check https://pypi.jetson-ai-lab.dev/ for the correct JP/CU url."
    echo "  CPU fallback (slow, but works for smoke tests):"
    echo "    pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu"
    exit 1
  }

echo "[cart_deps] done. Try:"
echo "  python -c 'import torch; print(torch.cuda.is_available())'"
