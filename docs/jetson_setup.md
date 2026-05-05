# Jetson Thor setup

## JetPack + Isaac ROS

Verify version compatibility:
- https://nvidia-isaac-ros.github.io/getting_started/index.html

```bash
cat /etc/nv_tegra_release   # JetPack version

# Isaac ROS dev container
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git \
  ~/workspaces/isaac_ros-dev/src/isaac_ros_common
cd ~/workspaces/isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh
# You're now inside the container. ROS 2 is sourced.
```

## Required Isaac ROS packages (inside dev container)

```bash
cd /workspaces/isaac_ros-dev/src
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_visual_slam.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nvblox.git
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline.git

cd /workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

## Nav2 + robot_localization

```bash
sudo apt install \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-robot-localization
```

## Our packages

```bash
cd /workspaces/isaac_ros-dev/src
ln -s ~/stanford-cart/cart_ws/src/cart_dbw .
ln -s ~/stanford-cart/cart_ws/src/cart_sensors .
ln -s ~/stanford-cart/cart_ws/src/cart_safety .
ln -s ~/stanford-cart/cart_ws/src/cart_bringup .
cd /workspaces/isaac_ros-dev
colcon build --packages-select cart_dbw cart_sensors cart_safety cart_bringup
source install/setup.bash
```

## Python deps for hardware + perception

```bash
pip install pyserial pynmea2 odrive ultralytics transformers torch torchvision pillow opencv-python
# Pre-download YOLO + depth weights so first launch isn't slow:
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
python -c "from transformers import pipeline; pipeline('depth-estimation', 'depth-anything/Depth-Anything-V2-Small-hf')"
```

## udev for stable port names (recommended)

The Mega and ODrive shift `/dev/ttyACM*` indices across reboots. Pin them:

```bash
# Plug in ODrive only, run:
udevadm info -a -n /dev/ttyACM0 | grep -E 'ATTRS\{(idVendor|serial)\}' | head -4
# Plug in Pedal Mega only:
udevadm info -a -n /dev/ttyACM0 | grep -E 'ATTRS\{(idVendor|serial)\}' | head -4
# Plug in GPS Mega only:
udevadm info -a -n /dev/ttyACM0 | grep -E 'ATTRS\{(idVendor|serial)\}' | head -4

# Then write /etc/udev/rules.d/99-cart.rules:
SUBSYSTEM=="tty", ATTRS{idVendor}=="1209", ATTRS{serial}=="<odrive-serial>", SYMLINK+="cart_odrive"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{serial}=="<pedal-mega-serial>", SYMLINK+="cart_pedals"
SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{serial}=="<gps-mega-serial>", SYMLINK+="cart_gps"

sudo udevadm control --reload && sudo udevadm trigger
```

Then in `cart.yaml`:
```yaml
mega_pedals:
  ros__parameters:
    port: /dev/cart_pedals
gps_mega:
  ros__parameters:
    port: /dev/cart_gps
```

## Sensor topics — what cart.launch.py expects

| Sensor | Topic | Type |
|---|---|---|
| GPS | `/fix` | sensor_msgs/NavSatFix |
| GPS course | `/vel` | geometry_msgs/TwistStamped |
| front_narrow | `/front_narrow/image_raw`, `/camera_info` | sensor_msgs/Image, CameraInfo |
| front_wide | `/front_wide/image_raw`, `/camera_info` | (same) |
| left | `/left/image_raw`, `/camera_info` | (same) |
| right | `/right/image_raw`, `/camera_info` | (same) |
| e-stop | `/estop_pressed` | std_msgs/Bool (latched, published by mega_pedals) |
| Nav2 cmd | `/cmd_vel_nav` | geometry_msgs/Twist |
| Safety out | `/cmd_vel` | geometry_msgs/Twist |
| DBW in | `/cmd_steer`, `/cmd_throttle`, `/cmd_brake` | std_msgs/Float64 |
