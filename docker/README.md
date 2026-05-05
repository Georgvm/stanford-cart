# Isaac ROS 4.4 on Thor — container bring-up for the cart

Isaac ROS does not publish stable Docker tags. NVIDIA distributes a CLI
(`isaac-ros-cli`) that computes a content-hashed image tag from the layered
Dockerfiles, pulls it from NGC, and runs it with the right flags. Pinning a
Dockerfile `FROM` to a hash will rot the next time NVIDIA bumps the base.

So our flow is:

  1. `setup_isaac_ros.sh`  — one-time, installs the CLI and initializes the
                              workspace at `/mnt/nova_ssd/workspaces/isaac_ros-dev/`
                              and symlinks our `cart_ws/src/*` packages into it.
  2. `enter.sh`            — opens a shell in the container, with our packages
                              already mounted and ready to colcon-build.
  3. `launch.sh <target>`  — convenience: start the container and run one of
                              our launch files (cart, teleop, sensors, dbw).

JetPack 7.1 is required for Isaac ROS 4.2+. JP 7.0 (R38.4) is what shipped
on this Thor; upgrade with `sudo apt install nvidia-jetpack` after adding
the JP7.1 apt repo, or re-flash via SDK Manager. Wait until after MVP if
you can — re-flash carries some risk of bricking.

OpenCV: JP7 ships OpenCV 4.8 system-wide; Isaac ROS pins 4.6. The CLI
handles this inside the container, but if you ever build Isaac ROS packages
NATIVELY on the Thor (outside the container), purge first:
  sudo apt purge -y "*opencv*" && sudo apt autoremove -y

`argus_camera` is not supported on Thor (NVIDIA forum, Apr 2026). Our cams
are USB so this doesn't bite us — `usb_camera_node.py` (cart_sensors) is
the producer. If you ever add a CSI / GMSL camera to this rig, you'll need
the Holoscan Sensor Bridge or a third-party driver.

## Files
- `setup_isaac_ros.sh`  — bootstrap, run once on the Thor
- `enter.sh`            — interactive shell into the running container
- `launch.sh`           — start container + ros2 launch in one shot
- `cart_deps.sh`        — pip-installs Python deps not in the base image
                          (Depth Anything V2, ultralytics, transformers, etc.)
                          Run inside the container after `enter.sh`.
