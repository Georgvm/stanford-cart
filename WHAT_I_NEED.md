# What I need from you

Code is scaffolded. None of it has touched real hardware. Below is what
unblocks the next step.

## P0 — blockers, can't even verify build without these

1. **Modal account** — run on the laptop deploying:
   ```
   pip install modal
   modal token new
   modal volume create stanford-cart-data
   ```

2. **Jetson Thor SSH** — host/user, JetPack version (`cat /etc/nv_tegra_release`).
   Confirm Isaac ROS version compatible with that JetPack
   (https://nvidia-isaac-ros.github.io/getting_started/index.html).

3. **`v4l2-ctl --list-devices` output from the Jetson.** The camera indices
   in `cart.yaml` are best-guess from your `record_cameras.py` defaults.
   Need ground truth before the cameras come up reliably.

4. **Confirm DBW topology**: are the ODrive steering and the pedal Mega the
   only two USB-serial-ish things on the Jetson? Or is the GPS Mega a separate
   port? If just one Mega, do you want to extend `pedal_control.ino` to
   forward NMEA, or flash a second Mega with the validation sketch?

## P1 — needed before first cart drive

5. **Cart geometry — measure on the cart**:
   - Wheelbase (m, front axle to rear axle)
   - Track width (m)
   - Min turning radius (m) — drive a tightest circle, halve the diameter
   - Max safe v0 speed (m/s)
   These go into `cart.yaml` (`wheelbase_m`, `minimum_turning_radius`,
   `max_linear_velocity`).

6. **Steering ratio** — the cmd_vel_to_dbw node assumes 1:1 between
   "column degrees commanded" and "road-wheel degrees actually steered."
   If your steering rack ratio is different (e.g. 4° at column = 1° at wheel),
   we need to insert that ratio. Tell me what 30° at the column does to the
   physical wheels (eyeball ok).

7. **IMU status** — has the BerryIMU arrived? If yes, what's its driver
   topology (i2c, USB, ROS package)? The launch file currently runs cuVSLAM
   with `enable_imu_fusion: false` and EKF without imu0. Re-enable both as
   soon as the IMU is on the bus.

8. **GPS plan** — confirm one of:
   - (a) Pedal Mega will be reflashed with combined firmware that emits both
     STAT and NMEA → we add a tee in `mega_pedals_node` to also publish /fix
   - (b) Second Arduino with `sensor_validation.ino` on a separate USB port →
     `gps_mega_node` finds it via VID + serial-number fallback (current code)
   - (c) Move GPS off the Mega entirely (e.g. u-blox USB receiver direct) →
     swap `gps_mega_node` for `nmea_navsat_driver`

9. **Hardware e-stop test** — verify pressing it cuts 48V (motor goes limp).
   Test 10 times. The Mega-side software e-stop already publishes
   `/estop_pressed` via `mega_pedals_node` parsing `EVT,ESTOP,1`.

10. **Wireless kill switch** — does one exist? If not, designate someone with
    a hand on the seat-mounted hardware e-stop for every drive.

## P1 — needed before route demo

11. **Route choice**: Lake Lag perimeter, Oval at 6am, road behind the dish?
    Send a screenshot. Closed area, weak foot traffic, ideally a loop with
    no intersections.

12. **NMEA log capture during manual drive** — the Modal pipeline expects
    a plain text file with NMEA sentences. Easiest: while driving manually,
    `tee` the existing `sensor_test.py` output to a file:
    ```
    uv run python scripts/sensor_test.py | tee ~/recordings/route_oval/gps.log
    ```

## What I assumed (push back if wrong)

- ROS 2 Humble in the Isaac ROS dev container on Jetson Thor
- The cart accepts `/cmd_steer` (Float64 column °), `/cmd_throttle` (0..1),
  `/cmd_brake` (0..1) once `dbw.launch.py` is up
- ODrive S1 is the only ODrive on the bus (`odrive.find_any` picks it)
- Pedal Mega and GPS Mega (if separate) are both Arduino-class USB VIDs;
  ODrive is `0x1209` and is explicitly excluded
- limits.py `FSD_GAS_LIMIT=0.25` is the right autonomy throttle ceiling for
  v1 — `mega_pedals_node` enforces it
- cuVSLAM uses front_narrow + front_wide as a multi-cam input (not stereo;
  no baseline assumed)
- nvblox depth comes from a monocular-depth node publishing `/front_wide/depth`
  (not yet written — placeholder in launch)
- `base_link` at rear axle center, x forward, y left, z up (REP-103)
- v1 demo is closed course, not real campus traffic

If any of those are wrong, tell me which one(s) and I'll regenerate the
affected configs.
