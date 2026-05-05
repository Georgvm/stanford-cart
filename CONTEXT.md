# CONTEXT.md — briefing for a fresh Claude instance

You're being handed a self-driving golf cart project mid-stream. This file
is the full briefing so you can pick up immediately. Read end-to-end before
your first response.

---

## The user

- Stanford team building a self-driving golf cart. Hackathon-style velocity
  but real hardware.
- Wants outcomes, not status updates. Says "stop worrying, just do" — so
  *implement, don't ask for permission on every step.* When uncertain about
  reversible local actions (file edits, builds, scripts) just go.
- Hates time-boxing ("stop thinking in terms of time"). Don't say "this will
  take 4–6 weeks" — describe what to build, then build it.
- No fake simulation, no toy demos. The path is the real one: Isaac ROS +
  Nav2 on real cameras and a real cart.
- **Never put Claude / Anthropic attribution in commits, PRs, code comments,
  or generated output.** This is a hard rule.
- They prefer terse responses. State what changed and what's next; skip the
  preamble.
- They asked you to keep working on the Thor without the round-trip of
  "Claude on Mac → user copy/pastes commands → user pastes back output."
  This file makes that possible.

---

## The goal

Drive across Stanford's campus by clicking a destination on a map. The
cart figures out where it is from its 4 cameras and avoids people in real
time. Like the NVIDIA Carter / Nova robot demos, on a golf cart, on
Stanford.

**Two phases:**
1. **Scan phase** — drive the cart around campus once with cuVSLAM
   (Isaac ROS Visual SLAM) running. cuVSLAM watches the cameras, builds a
   3D map, saves it. ~30 min of teleop driving.
2. **Drive phase** — load the saved map. Publish a goal pose in RViz or via
   `/goal_pose`. Nav2 plans a path. cuVSLAM keeps the cart localized
   against the map. nvblox builds a live 3D occupancy grid from camera
   depth → Nav2 costmap. safety_gate watchdogs everything. cmd_vel_to_dbw
   translates Nav2's Twist into ODrive + Mega DBW commands. Cart drives.

Teleop isn't a separate goal — it's the means to do the scan phase, and
also the only way to physically move the cart while you're debugging.

---

## Hardware

- **Compute:** NVIDIA Jetson AGX Thor, **JetPack 7.0 (L4T R38.4, Dec 2025)**,
  Ubuntu 24.04 (Noble). Hostname `caddy-thor`, user `caddy`.
- **NVMe:** mounted at `/mnt/nova_ssd` (Isaac ROS Docker workspace lives
  here — `/mnt/nova_ssd/workspaces/isaac_ros-dev/`).
- **Cameras:** 4× USB UVC. left, right, front-narrow (varifocal 2.8-12mm),
  front-wide (~170° fisheye, **mounted upside down — needs vertical flip**).
- **GPS:** USB receiver wired directly to the Thor (NOT through the Mega).
  The cart's existing setup. Plug into Thor USB.
- **Steering:** ODrive S1 over USB-C → motor → belt (3:1) → steering column.
- **Pedals:** Arduino Mega running `pedal_control.ino` (servo-driven gas/brake
  pedals + hardware e-stop), USB-serial to Thor.
- **Custom accelerometer:** camera-grounded measurement app the team built —
  better than a drifting IMU. Not yet integrated; treat as the IMU input
  when ready. BerryIMU was the prior plan; superseded.
- **No LiDAR.**
- **PS5 DualSense controller** for teleop (joy → teleop_twist_joy).

**Power/safety chain:** Hardware e-stop in seat → cuts 48V → motor goes
limp. Mega's 300ms heartbeat watchdog will retract pedals even if every
Python node dies. ODrive's 300ms axis watchdog will idle steering if the
host stops feeding it. Both are firmware-side; software bugs upstream
cannot defeat them.

---

## Software state — what's done

### ROS 2 Jazzy installed natively on the Thor

`/opt/ros/jazzy/` exists. `ros2` works (note: `ros2 --version` doesn't
exist; verify via `printenv ROS_DISTRO` or `ros2 pkg list`). Installed
packages include `ros-jazzy-desktop`, `ros-dev-tools`,
`python3-colcon-common-extensions`, `ros-jazzy-robot-localization`,
`ros-jazzy-nav2-bringup`, `ros-jazzy-joy`, `ros-jazzy-teleop-twist-joy`.

`source /opt/ros/jazzy/setup.bash` is appended to `~/.bashrc`.

### Repo: `~/stanford-cart` (assumed rsync'd from Mac)

If it's not there yet, see "Bringing the repo onto the Thor" below.

```
stanford-cart/
├── CONTEXT.md                  # this file
├── README.md                   # architecture + topic flow diagram
├── WHAT_I_NEED.md              # P0/P1 unblockers (older — partly resolved)
├── cart_ws/src/
│   ├── cart_msgs/              # empty stub (no package.xml — skip in builds)
│   ├── cart_sensors/           # USB cameras + GPS + Depth Anything
│   │   └── cart_sensors/{usb_camera_node,gps_mega_node,
│   │                       depth_anything_node}.py
│   ├── cart_dbw/               # ODrive + Mega + cmd_vel→DBW
│   │   └── cart_dbw/{odrive_steering_node,mega_pedals_node,
│   │                  cmd_vel_to_dbw}.py
│   ├── cart_safety/            # safety gate, perception fallback,
│   │   │                       # demo runner, engage button
│   │   └── cart_safety/{safety_gate,perception_estop,
│   │                     demo_runner,engage_button}.py
│   └── cart_bringup/
│       ├── launch/{cart,sensors,dbw,teleop,tf_static}.launch.py
│       ├── config/{cart,ekf,nav2_params}.yaml
│       └── behavior_trees/follow_gps_route.xml
├── modal_pipeline/             # offline NMEA → waypoints (works end-to-end)
│   └── process_route.py
├── docker/                     # Isaac ROS 4.4 container scripts
│   └── {setup_isaac_ros,enter,launch,cart_deps}.sh + README.md
├── scripts/{record_route.sh,upload_to_modal.sh,run_demo.sh,record_nmea.py}
└── docs/{runbook,jetson_setup,v2_roadmap}.md
```

### What works (scaffolded — most untested on hardware yet)

- `cart.launch.py` wires the full graph: tf_static → sensors → DBW →
  depth_anything → cuVSLAM → nvblox → ekf_local + ekf_global +
  navsat_transform → Nav2 → perception_estop → demo_runner.
- `teleop.launch.py` wires joy → teleop_twist_joy → engage_button (L1
  dead-man → /run_demo) → safety_gate → DBW.
- `safety_gate.py` is the only thing publishing `/cmd_vel` into the DBW
  chain. Gates on `/run_demo`, `/estop_pressed`, obstacle distance, and
  watchdog.
- `depth_anything_node.py` runs Depth Anything V2 Small on
  `/front_wide/image_raw`, publishes `/front_wide/depth` for nvblox.
- `gps_mega_node.py` autodetects USB GPS — prefers dedicated GPS chipset
  VIDs (u-blox, Garmin, SiRF, MediaTek) over Arduino-class VIDs. Falls
  back to highest-serial Arduino port if no dedicated GPS found.
- `tf_static.launch.py` publishes base_link → optical frames + GPS frame.
  Distances are **approximate** — measure on the cart and update.
- Modal pipeline is verified end-to-end (synthetic NMEA → smoothed
  waypoints → folium preview map). Modal volume is `stanford-cart-data`,
  Modal profile is `caddycompute`.

### What hasn't been verified on hardware

Everything past compile-clean. No camera has been opened. No motor has
turned. No GPS fix has been received.

---

## Critical path — do these in order

1. **Plug all hardware into the Thor's USB ports** (cameras × 4, GPS, Mega,
   ODrive). Verify with:
   ```
   lsusb
   ls /dev/video* /dev/ttyACM* /dev/ttyUSB*
   v4l2-ctl --list-devices
   ```
   The current `cart.yaml` assumes `/dev/video0` = front_narrow,
   `/dev/video6` = front_wide, `/dev/video2` = left, `/dev/video10` = right.
   These are guesses from the team's older `record_cameras.py`. Verify
   the actual indices and update `cart_ws/src/cart_bringup/config/cart.yaml`
   if they're wrong.

2. **Build the workspace:**
   ```
   cd ~/stanford-cart/cart_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select cart_sensors cart_dbw cart_safety cart_bringup
   source install/setup.bash
   ```
   (Skip `cart_msgs` — empty stub. Skip rosdep — it'll choke on
   isaac_ros_visual_slam / nvblox_ros which live in the Isaac ROS Docker.)

3. **Bring up sensors first, verify they're publishing:**
   ```
   ros2 launch cart_bringup sensors.launch.py
   # In another terminal:
   ros2 topic hz /front_narrow/image_raw /front_wide/image_raw /fix
   ```
   If a camera node fails to open `/dev/videoN`, double-check device index
   in `cart.yaml` and `sensors.launch.py`.

4. **Bench-test DBW:**
   ```
   ros2 launch cart_bringup dbw.launch.py
   # IN ANOTHER TERMINAL:
   ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: true"
   ros2 topic pub --once /cmd_steer std_msgs/msg/Float64 "data: 30.0"
   # Wheel should turn ~30° right.
   ros2 topic pub --once /cmd_steer std_msgs/msg/Float64 "data: 0.0"
   ros2 topic pub --once /run_demo std_msgs/msg/Bool "data: false"
   ```

5. **Wire teleop, drive the cart manually (this is the scan phase tooling):**
   ```
   ros2 launch cart_bringup teleop.launch.py
   ```
   Hold L1 (DualSense), drive with sticks. cmd_vel_nav → safety_gate
   → cmd_vel → cmd_vel_to_dbw → ODrive + Mega.

6. **Install Isaac ROS 4.4 Docker workspace** (for cuVSLAM + nvblox + Nav2):
   ```
   cd ~/stanford-cart/docker
   ./setup_isaac_ros.sh   # one-time, ~20 min, pulls ~10 GB image
   ./enter.sh             # shell into the container
   ./cart_deps.sh         # inside container — installs torch / yolo / depth-anything
   cd /workspaces/isaac_ros-dev
   colcon build
   source install/setup.bash
   ```
   **NOTE:** Isaac ROS 4.2+ wants JetPack 7.1. This Thor is on JP 7.0.
   If `setup_isaac_ros.sh` hits version errors, upgrade JP first
   (re-flash via SDK Manager — risky — only do this after step 5 works).
   Isaac ROS 4.0.x supports JP 7.0 as a fallback if 4.4 won't go.

7. **Scan the campus** — drive teleop around the routes you care about
   with `ros2 launch cart_bringup cart.launch.py` running (cuVSLAM is
   active). The map gets built incrementally. Save it (cuVSLAM map save
   API; check Isaac ROS docs for the current command — it changes by
   version).

8. **Drive autonomously** — load the map, publish a goal pose in RViz,
   watch Nav2 plan + execute, with cuVSLAM localizing and nvblox doing
   live obstacle avoidance.

---

## Bringing the repo onto the Thor (if not present)

The repo lives at `~/stanford-cart/` on the user's Mac. It's **not in git**
(no remote). Options:
- **rsync over WiFi** (best, when SSH works):
  ```
  # FROM THE MAC:
  rsync -avz --exclude='.venv' --exclude='out' --exclude='__pycache__' \
    ~/stanford-cart/ caddy@<thor-ip>:~/stanford-cart/
  ```
- **rsync over USB-C** (Thor is at `192.168.55.1` over USB-C bridge if
  l4tbr0 is up — but sshd may not be reachable on this interface).
- **USB stick.**

Whichever you use, after copying, `chmod +x scripts/*.sh docker/*.sh
scripts/record_nmea.py` to preserve executable bits.

---

## Known gotchas — don't repeat these

### Stanford WiFi network blocks `raw.githubusercontent.com`

DNS doesn't resolve it. So `git clone` from GitHub raw URLs and ROS install
scripts that fetch the GPG key from GitHub will fail. Workarounds we used:
- ROS 2 install: fetched GPG key from `keyserver.ubuntu.com` over HTTPS
  instead of GitHub.
- For other tools: try `packages.ros.org` (works), `pypi.org` (works),
  Ubuntu mirrors (work). Avoid `*.githubusercontent.com`.

### Thor's bash terminal mangled long pastes

Bracketed paste mode codes (`^[[200~`) were leaking through into the input
because readline wasn't stripping them, and visually-wrapped lines from
chat were getting hard-newlines + 2-space indent inserted on copy. Fixed
on this Thor with:
```
echo 'set enable-bracketed-paste off' >> ~/.inputrc
bind 'set enable-bracketed-paste off'
```
If a future paste corrupts again, re-run those.

### `isaac_ros_argus_camera` is dead on Thor

NVIDIA forum (Apr 2026): "not supported, no update planned." Doesn't
matter for us because all cameras are USB. If anyone ever adds a CSI/GMSL
camera, you need Holoscan Sensor Bridge or a third-party driver.

### OpenCV version conflict

JP7 ships OpenCV 4.8 system-wide. Isaac ROS pins 4.6. Inside the Docker
container this is handled. If you ever build Isaac ROS packages NATIVELY
on the Thor (outside the container), purge OpenCV first:
```
sudo apt purge -y "*opencv*" && sudo apt autoremove -y
```

### `ros2 --version` doesn't exist

Use `printenv ROS_DISTRO` (when sourced) or `ros2 pkg list | head -1`.
The earlier conversation kept tripping on this.

### SSH from Mac → Thor over WiFi is intermittent

Stanford WiFi NAT/firewall closes connections quickly. Re-try if it
times out. The Thor's IP changes occasionally; check with
`ip -br addr | grep -v DOWN` on the Thor when reconnecting.

---

## Modal pipeline (works, leave it alone)

Modal account: profile `caddycompute`, volume `stanford-cart-data`.

End-to-end flow:
```
~/stanford-cart/scripts/record_nmea.py --route route_oval
# Records raw NMEA from the GPS USB serial port to ~/recordings/route_oval/<ts>/gps.log

# On the Mac (or wherever Modal CLI is):
~/stanford-cart/scripts/upload_to_modal.sh /tmp/gps.log route_oval
# Output: ~/stanford-cart/out/route_oval/{waypoints.yaml, route_preview.html}

# scp the waypoints to the Thor:
scp ~/stanford-cart/out/route_oval/waypoints.yaml caddy@<thor>:/data/route_oval/

# On the Thor:
ros2 launch cart_bringup cart.launch.py route_yaml:=/data/route_oval/waypoints.yaml
```

The smoothing + downsampling logic is in `modal_pipeline/process_route.py`.
A synthetic NMEA generator for testing is in
`modal_pipeline/_make_fake_nmea.py`.

---

## Things to look up rather than guess

- **Isaac ROS quickstart for Thor:**
  https://nvidia-isaac-ros.github.io/getting_started/index.html
- **Isaac ROS release notes (current is 4.4, 2026-04-30):**
  https://nvidia-isaac-ros.github.io/releases/index.html
- **cuVSLAM map save / load API** — version-dependent; check the docs
  for whatever Isaac ROS version actually got installed.
- **JetPack 7.1 upgrade docs** — only consult when ready to re-flash.

---

## When you finish a meaningful unit of work

Update `docs/runbook.md` if the bring-up sequence changed. Update this
CONTEXT.md if a major architectural fact changed (different Isaac ROS
version got installed, different GPS topology, etc.). Don't update for
small tactical changes.

Don't add commits with Claude attribution. Don't write summary docs the
user didn't ask for. Don't add features beyond what was asked.

---

## Open tasks at handoff time

1. Plug in hardware on the Thor and enumerate (`lsusb`, `/dev/video*`,
   `/dev/tty*`).
2. Verify `cart.yaml` device indices against actual.
3. Build the workspace.
4. Smoke-test sensors launch.
5. Bench-test DBW launch with manual `/cmd_steer` pubs.
6. Run teleop launch end-to-end.
7. Install Isaac ROS Docker workspace.
8. Drive the scan phase.
9. Drive autonomously.

If you finish a task, edit this list (or remove it once you've moved past
it). Don't keep status synced with the Mac-side conversation — that's not
the path forward; the user explicitly wants to drop the round-trip.
