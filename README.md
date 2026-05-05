# Stanford Self-Driving Golf Cart — Isaac ROS + Nav2 stack

Closed-course GPS-waypoint follower with cuVSLAM localization correction,
nvblox 3D obstacle costmap, and a YOLO+Depth fallback emergency-stop.

Built fresh as ROS 2 nodes. Hardware interface (ODrive S1 steering,
Arduino Mega pedal serial protocol) reverse-engineered from the team's
existing `clart` repo, but **none of that Python is reused** — those
nodes are custom for this stack.

## Architecture

```
                            ┌─ Modal H200 (offline) ─┐
                            │  NMEA log → smoothed   │
                            │  waypoints.yaml +      │
                            │  route_preview.html    │
                            └──────────┬─────────────┘
                                       │  modal volume get
                                       ▼
                          ┌──────────── Jetson Thor ───────────┐
                          │                                    │
   4× USB cams ─► usb_camera_node × 4 (front_narrow,           │
                            front_wide [vert flip], left,      │
                            right)                             │
                          │       │                            │
   GPS NEO-6M ─► gps_mega_node ─► /fix                         │
                          │                                    │
                          │  /front_*/image_raw + /fix         │
                          ▼                                    │
                    cuVSLAM (Isaac ROS)                        │
                    nvblox  (Isaac ROS) ─► local_costmap layer │
                          │                                    │
                    EKF local + global + navsat_transform      │
                          │  /odometry/filtered/global         │
                          ▼                                    │
                         Nav2 (Smac planner + Pure Pursuit     │
                          + GPS waypoint follower)             │
                          │  /cmd_vel_nav                      │
                          ▼                                    │
                  ┌── safety_gate ──┐                          │
                  │  ▲              │                          │
                  │  ├ /run_demo    │  /cmd_vel                │
                  │  ├ /estop       │   ▼                      │
                  │  └ /perception/ │ cmd_vel_to_dbw           │
                  │    nearest_obs  │   │                      │
                  └─────────────────┘   ▼                      │
                                     /cmd_steer (column °)      │
                                     /cmd_throttle (0..1)       │
                                     /cmd_brake (0..1)          │
                                        │       │              │
                              odrive_steering   mega_pedals    │
                                  │             │              │
                                  ▼             ▼              │
                              ODrive S1     Arduino Mega      │
                              (USB-C)       (USB serial)      │
                                              │  ▲            │
                                              │  └ EVT,ESTOP   │
                                              ▼               │
                                        Pedal actuators       │
                          └────────────────────────────────────┘
```

## Layout

```
cart_ws/src/
├── cart_dbw/         # ODrive + Mega bridges + cmd_vel translator
├── cart_sensors/     # USB cameras + GPS
├── cart_safety/      # safety gate, perception fallback, demo runner
└── cart_bringup/     # launch + Nav2 + EKF + cart.yaml configs

modal_pipeline/       # H200 NMEA-log → waypoints.yaml
scripts/              # record_route, upload_to_modal, run_demo
docs/                 # runbook, jetson setup, integration notes
WHAT_I_NEED.md        # READ THIS — info/access still required
```

## Topic flow at a glance

Authority chain — every command goes through these in order, no bypass:

```
Nav2 → /cmd_vel_nav → safety_gate → /cmd_vel → cmd_vel_to_dbw → /cmd_steer + /cmd_throttle + /cmd_brake → ODrive + Mega
```

The Mega's own 300 ms heartbeat watchdog will retract pedals even if
every Python node above it dies. The ODrive's 300 ms axis watchdog will
idle the steering motor if the host stops feeding `watchdog_feed()`.
Both watchdogs are enforced by hardware-side firmware — software bugs
upstream cannot defeat them.

## Quick start

```bash
# 1. Modal (any laptop):
pip install modal
modal token new
modal volume create stanford-cart-data

# 2. Jetson Thor (inside Isaac ROS dev container):
cd ~/workspaces/isaac_ros-dev/src
ln -s ~/stanford-cart/cart_ws/src/cart_dbw .
ln -s ~/stanford-cart/cart_ws/src/cart_sensors .
ln -s ~/stanford-cart/cart_ws/src/cart_safety .
ln -s ~/stanford-cart/cart_ws/src/cart_bringup .
cd ~/workspaces/isaac_ros-dev
colcon build --symlink-install
source install/setup.bash
```

## End-to-end

```bash
# Drive manually, record the loop:
./scripts/record_route.sh route_oval
# (Use your existing PS5 controller for the manual drive — that's untouched.)
# Save the GPS NMEA stream to a log file alongside the bag.

# Process on Modal:
./scripts/upload_to_modal.sh ~/recordings/route_oval/gps.log route_oval
# Verify in browser: ./out/route_oval/route_preview.html

# Copy waypoints to the cart:
scp ./out/route_oval/waypoints.yaml jetson:/data/route_oval/

# On the Jetson — launch:
ros2 launch cart_bringup cart.launch.py route_yaml:=/data/route_oval/waypoints.yaml

# Watch:
ros2 topic echo /cart_safety/status

# Arm:
./scripts/run_demo.sh
```

## What WORKS today vs OPEN

Works (code complete, untested on hardware):
- ODrive steering bridge, ROS 2 node with hardware watchdog feed
- Mega pedal bridge, with full G/B/H/D protocol + EVT,ESTOP parsing
- 4-camera USB bringup at known indices, front_wide vertical flip
- GPS NMEA reader (assumes a SECOND Mega port; see open issues)
- Nav2 GPS waypoint follower wired to safety_gate → cmd_vel_to_dbw
- Modal H200 pipeline: NMEA log → smoothed waypoints.yaml + preview map

Open:
- **No IMU** — BerryIMU pending. cuVSLAM enabled with `enable_imu_fusion: false`;
  EKF imu0 commented out. Will drift in orientation. Fine for slow GPS-routed
  driving; not fine for tight maneuvers or GPS-denied stretches.
- **GPS port assumption**: this code expects GPS NMEA on a separate USB serial
  port from the pedal Mega. The cart's existing `pedal_control.ino` doesn't
  forward NMEA. Either flash a second Mega with the validation sketch, or extend
  pedal_control.ino to forward Serial1 bytes alongside its STAT lines.
- **Camera indices**: `cart.yaml` uses [0, 6, 2, 10] from `record_cameras.py`,
  but `autoware_infer.py` SLUGS imply a different order. Verify with
  `v4l2-ctl --list-devices` on the actual Jetson before trusting.
- **nvblox depth source**: needs either a monocular-depth node publishing
  `/front_wide/depth`, or true stereo from front_narrow+front_wide. TODO.
- **Camera intrinsics**: `usb_camera_node` publishes a placeholder
  `CameraInfo`. Calibrate with `camera_calibration` package before depending
  on cuVSLAM accuracy.
- **Wheelbase / turning radius / max safe speed**: guessed in `cart.yaml`.
  Measure on the actual cart.

See `WHAT_I_NEED.md` for the full list.

## Five-person split

| Person | Track | Owns |
|---|---|---|
| ME 1 | Verify DBW + e-stop on the cart | Steering responds to /cmd_steer; pedals respect /cmd_throttle+/cmd_brake; hardware e-stop tested 10×; wireless kill in operator's hand. |
| EE 1 | Sensors | All 4 cams publishing on canonical topics; intrinsics calibrated; GPS port plumbed (with second Mega if needed); BerryIMU plumbed when it arrives. |
| CS 1 | Platform | Isaac ROS dev container on Thor; Modal account; networking; recording infra; install + colcon build clean. |
| CS 2 | Mapping | Drive route (manual), capture NMEA log, run Modal pipeline, validate `route_preview.html`. |
| CS 3 | Nav stack | Tune `nav2_params.yaml` lookahead/desired_vel/turning radius; wire perception_estop calibration; behavior tree. |

## Honest scope

Day 1-2: closed-course GPS waypoint follower with emergency stop, on a clear
loop somewhere quiet (Lake Lag, Oval at 6am, road behind the dish), at 3-8 mph,
safety driver in seat the whole time. That's a real demo and a legitimate
hackathon ship. It is **not** a Stanford-campus shuttle. v2 (week 2+) layers
on the H200-built 3DGS map + ACE relocalizer + pedestrian-aware planner —
see `docs/v2_roadmap.md`.
