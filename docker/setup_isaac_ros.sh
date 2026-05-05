#!/usr/bin/env bash
# setup_isaac_ros — one-time bootstrap on the Thor.
#
# After this finishes, you can `./enter.sh` into the container and
# `colcon build --packages-select cart_bringup cart_sensors cart_dbw cart_safety`.
#
# Idempotent. Safe to re-run.

set -euo pipefail

# ---- 1. NVMe workspace mount path -------------------------------------------
ISAAC_ROS_WS_HOST="/mnt/nova_ssd/workspaces/isaac_ros-dev"
echo "[setup] workspace host path: $ISAAC_ROS_WS_HOST"

if ! mountpoint -q /mnt/nova_ssd; then
  echo "[setup] WARNING: /mnt/nova_ssd is not a mountpoint." >&2
  echo "        Isaac ROS expects an NVMe mounted at /mnt/nova_ssd. If you" >&2
  echo "        haven't installed the NVMe yet, set ISAAC_ROS_WS_HOST to" >&2
  echo "        somewhere on / and accept the IO penalty for now." >&2
  read -rp "        Continue anyway? [y/N] " ans
  [[ "$ans" == "y" || "$ans" == "Y" ]] || exit 1
fi

sudo mkdir -p "$ISAAC_ROS_WS_HOST/src"
sudo chown -R "$USER:$USER" "$(dirname "$ISAAC_ROS_WS_HOST")"

# ---- 2. Install isaac-ros-cli ----------------------------------------------
if ! command -v isaac-ros >/dev/null 2>&1; then
  echo "[setup] installing isaac-ros-cli..."
  # NVIDIA NGC apt repo — already configured by JetPack on the Thor.
  sudo apt-get update
  sudo apt-get install -y isaac-ros-cli
else
  echo "[setup] isaac-ros-cli already installed: $(isaac-ros --version 2>&1 | head -1)"
fi

# ---- 3. Initialize Docker integration --------------------------------------
echo "[setup] initializing Isaac ROS Docker integration"
echo "[setup] this pulls the content-hashed image (~10-20 GB, may take a while)"
sudo isaac-ros init docker

# ---- 4. Symlink our cart_ws/src into the Isaac ROS workspace ---------------
CART_WS_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/cart_ws/src"
echo "[setup] linking cart packages from $CART_WS_SRC"
for pkg in cart_bringup cart_sensors cart_dbw cart_safety cart_msgs; do
  src="$CART_WS_SRC/$pkg"
  dst="$ISAAC_ROS_WS_HOST/src/$pkg"
  if [[ ! -d "$src" ]]; then
    echo "[setup]   skip $pkg (not present)"
    continue
  fi
  rm -f "$dst" 2>/dev/null || true
  ln -sfn "$src" "$dst"
  echo "[setup]   $pkg → $src"
done

# ---- 5. Pull the standard Isaac ROS source repos ---------------------------
echo "[setup] cloning Isaac ROS source packages we depend on"
cd "$ISAAC_ROS_WS_HOST/src"
declare -a ISAAC_REPOS=(
  "isaac_ros_common"
  "isaac_ros_visual_slam"
  "isaac_ros_image_pipeline"
  "isaac_ros_nvblox"
  "isaac_ros_dnn_inference"
  "isaac_ros_compression"
)
for repo in "${ISAAC_REPOS[@]}"; do
  if [[ -d "$repo" ]]; then
    echo "[setup]   $repo already present, skipping clone"
    continue
  fi
  echo "[setup]   cloning $repo"
  git clone "https://github.com/NVIDIA-ISAAC-ROS/${repo}.git" || \
    echo "[setup]   WARNING: clone of $repo failed (network?)"
done

echo
echo "[setup] done. Next:"
echo "  ./enter.sh                # shell into the container"
echo "  ./cart_deps.sh            # inside container — install Python deps"
echo "  colcon build              # inside container"
echo "  source install/setup.bash # inside container"
echo "  ros2 launch cart_bringup teleop.launch.py"
