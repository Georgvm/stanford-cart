# Day-of runbook

## First-time bring-up on the Thor

Before you can do any of the day-of stuff below you need:
1. **Native ROS 2 Jazzy** — for teleop + GPS recording (no Isaac ROS deps)
2. **Isaac ROS 4.4 Docker** — for cuVSLAM + nvblox + Nav2 (full autonomy)

```bash
# (1) Native Jazzy — already installed if you ran the bring-up block.
ros2 --version  # should print "Jazzy"

# (2) Build the workspace natively for teleop / sensor / DBW testing
cd ~/stanford-cart/cart_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select \
  cart_msgs cart_sensors cart_dbw cart_safety cart_bringup
source install/setup.bash

# (3) Isaac ROS Docker — for the autonomy stack
cd ~/stanford-cart/docker
./setup_isaac_ros.sh        # one-time, ~20 min, pulls ~10 GB image
./enter.sh                  # shell into the container
./cart_deps.sh              # inside container — depth-anything / yolo / torch
cd /workspaces/isaac_ros-dev
colcon build
source install/setup.bash
```

## Recording a route (no autonomy required, MVP path)

```bash
# Terminal 1: drive the cart with the PS5 controller.
ros2 launch cart_bringup teleop.launch.py
# Hold L1 (dead-man), drive the loop with the sticks.

# Terminal 2: record raw NMEA from the GPS Mega to a file.
~/stanford-cart/scripts/record_nmea.py --route route_oval

# When done, ctrl-c the recorder. It prints the output path.
# Pull it to your laptop and run through Modal:
scp caddy@192.168.55.1:~/recordings/route_oval/<ts>/gps.log /tmp/
~/stanford-cart/scripts/upload_to_modal.sh /tmp/gps.log route_oval
# Output: ~/stanford-cart/out/route_oval/{waypoints.yaml, route_preview.html}
# scp waypoints.yaml back to the Thor at /data/route_oval/.
```

## Pre-flight (every drive)

- [ ] Cart battery > 50%
- [ ] Hardware e-stop cuts motor power. Test 3×.
- [ ] Wireless kill switch in operator's hand. Test.
- [ ] Safety driver in seat, hand on physical e-stop, foot near brake
- [ ] Closed area confirmed clear of pedestrians/cyclists/dogs
- [ ] Spotter walking ahead of the cart

## Launch

```bash
# Bring up sensors first; verify they're publishing.
ros2 launch cart_bringup sensors.launch.py
ros2 topic hz /fix /front_wide/image_raw /front_narrow/image_raw

# Bring up DBW; verify steering responds to a manual /cmd_steer pub.
ros2 launch cart_bringup dbw.launch.py
# In another terminal — TEST FIRST WITH /run_demo TRUE:
ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: true"
ros2 topic pub --once /cmd_steer std_msgs/msg/Float64 "data: 30.0"
# Wheel should turn ~30° right. Then back to 0:
ros2 topic pub --once /cmd_steer std_msgs/msg/Float64 "data: 0.0"
ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: false"

# If both subsystems pass, bring up the full stack:
ros2 launch cart_bringup cart.launch.py \
  route_yaml:=/data/route_oval/waypoints.yaml

# Watch the safety status:
ros2 topic echo /cart_safety/status
```

## Arm

```bash
./scripts/run_demo.sh
```

The cart will move. Spotter and safety driver own the cart from this moment.

## Abort criteria

Stop immediately if:
- `/cart_safety/status` shows anything other than `OK ...` for >2 s
- Cart heading visibly diverges from the route
- Anyone on the route except spotter and safety driver
- GPS dropout (`/fix` topic stops publishing)
- Cart begins to oscillate
- Anything you weren't expecting

How to abort, in priority order:
1. Hardware e-stop in seat (cuts 48V — Mega → publishes `EVT,ESTOP,1` →
   safety_gate latches → ODrive idles + Mega slams brake)
2. Wireless kill switch (same effect at the hardware layer)
3. `ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: false"`
4. `Ctrl+C` the launch terminal (Mega's 300 ms watchdog will brake
   regardless ~300 ms later)

## Failure modes

| Symptom | Likely cause | Fix |
|---|---|---|
| Cart turns into curb | lookahead too short | Raise `lookahead_dist` 2.5→3.5 |
| Cart oscillates straight | lookahead too short OR no IMU drift | Raise lookahead. If BerryIMU is in, re-enable EKF imu0. |
| Stops at nothing | reflective surface fooling depth | Filter classes / NMS in perception_estop |
| Stops too far from people | depth heuristic mis-calibrated | Calibrate `_disparity_to_meters` against known distance |
| Cart cuts corner | min turning radius too low | Raise `minimum_turning_radius` |
| GPS jumps under trees | multipath | Trust cuVSLAM more, GPS less in EKF |
| ODrive disarms mid-drive | watchdog tripped | Raise `watchdog_timeout_s` slightly; check Jetson load |
| Mega FAILSAFE message | host watchdog tripped (300 ms) | Check `/pedals/state` — likely safety_gate stopped publishing or `mega_pedals_node` died |
| Cart accelerates harder than expected | check `fsd_gas_limit` | Verify `cart.yaml` says 0.25, not 0.45 or 0.68 |
