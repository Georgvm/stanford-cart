"""
run_isaac_ros — full Isaac ROS 4.4 pipeline against the recorded Caddy
training dataset, on Modal H100.

Pipeline inside the container:
  dataset_playback_node  → /front_*/image_raw + /odom/arkit + /fix
       │
       ├─→ depth_anything_node → /front_wide/depth
       ├─→ cuVSLAM             → /visual_slam/tracking/odometry
       ├─→ nvblox              → ESDF / costmap
       └─→ EKF (local + global) + navsat_transform → /odometry/filtered/*

`ros2 bag record -a` captures every topic. After playback ends, the bag is
post-processed → trajectory_overhead.png + trajectories.csv on the volume.

NVIDIA does NOT publish a stable pullable Isaac ROS 4.4 x86_64 image on
NGC — they ship apt packages and an `isaac-ros-cli` that composes images
locally. So this Modal image builds from CUDA base → ROS 2 Jazzy → Isaac
ROS apt packages → our cart_ws. First build is slow (~20-30 min); cached
after that.

Pull results:
  modal volume get stanford-cart-data output ./out/

Usage:
  modal run modal_pipeline/run_isaac_ros.py
  modal run modal_pipeline/run_isaac_ros.py --speed 2.0 --duration-s 200
"""

import os
import pathlib

import modal

app = modal.App("stanford-cart-isaac-ros")
volume = modal.Volume.from_name("stanford-cart-data")
REPO_ROOT = pathlib.Path(__file__).parent.parent


# Build steps in plain English:
#   1. CUDA 13.0 runtime on Ubuntu 24.04 (Isaac ROS 4.4 reqs CUDA 13 + driver 580+)
#   2. Add ROS 2 Jazzy apt repo, install ros-jazzy-desktop + nav2 + robot_localization
#   3. Add Isaac ROS apt repo (release-4 jazzy main), install cuVSLAM + nvblox +
#      image_pipeline + dnn_image_encoder
#   4. pip install our Python runtime deps (transformers, ultralytics, etc.)
#   5. Copy cart_ws/src into the image, colcon build our packages
#   6. Set NVIDIA_DRIVER_CAPABILITIES=all so cuVSLAM finds CUDA runtime libs

# Pre-built Isaac ROS 4.4 x86_64 base — has CUDA 13, ROS 2 Jazzy, cuVSLAM,
# nvblox, image_pipeline, VPI, nvsci, nitros, and every other Jetson-flavored
# dep we kept hitting. ~39 GB. Tag is content-hashed; refresh via the Isaac
# ROS CLI (`isaac-ros` prints/pulls the current value). The amd64 sibling of
# the arm64 image NVIDIA publishes for Jetson.
ISAAC_ROS_IMAGE = "nvcr.io/nvidia/isaac/ros:noble-ros2_jazzy_d3e84470d576702a380478a513fb3fc6-amd64"

isaac_image = (
    modal.Image.from_registry(
        ISAAC_ROS_IMAGE,
        secret=modal.Secret.from_name("ngc-credentials"),
    )
    .env({
        "DEBIAN_FRONTEND": "noninteractive",
        "NVIDIA_DRIVER_CAPABILITIES": "all",
        "NVIDIA_VISIBLE_DEVICES": "all",
        # Ubuntu 24.04 marks the system Python as externally-managed (PEP 668);
        # in a container that's pointless. Override.
        "PIP_BREAK_SYSTEM_PACKAGES": "1",
    })
    # Our Python runtime deps (transformers + torch + ultralytics for
    # depth_anything + perception_estop). Base image already has rclpy,
    # cv_bridge, sensor_msgs etc.
    .pip_install(
        "transformers",
        "pillow",
        "matplotlib",
        "utm",
        "pynmea2",
    )
    # Copy our workspace + build it on top of the preinstalled Isaac ROS
    # workspace. cart_msgs is skipped (empty stub).
    .add_local_dir(
        str(REPO_ROOT / "cart_ws" / "src"),
        "/workspaces/cart_ws/src",
        copy=True,
    )
    .add_local_file(
        str(REPO_ROOT / "modal_pipeline" / "_extract_trajectories.py"),
        "/workspaces/_extract_trajectories.py",
        copy=True,
    )
    .run_commands(
        "bash -lc 'source /opt/ros/jazzy/setup.bash && "
        "cd /workspaces/cart_ws && "
        "colcon build --packages-select cart_sensors cart_dbw cart_safety cart_bringup "
        "  --event-handlers console_direct+'",
    )
)


@app.function(
    image=isaac_image,
    gpu="H100",
    volumes={"/data": volume},
    secrets=[modal.Secret.from_name("ngc-credentials")],
    timeout=60 * 30,
)
def run_dataset_replay(speed: float = 2.0, duration_s: int = 400) -> str:
    """Launch the full Isaac ROS pipeline against the dataset, record a
    rosbag, post-process, return the output dir on the volume."""
    import signal
    import subprocess
    import time

    dataset = "/data/dataset/Caddy-Training-Data-2026-05-03_16-08-00"
    if not pathlib.Path(dataset).is_dir():
        raise RuntimeError(f"dataset not found at {dataset}")

    ts = time.strftime("%Y%m%d_%H%M%S")
    out_dir = pathlib.Path(f"/data/output/{ts}")
    out_dir.mkdir(parents=True, exist_ok=True)
    bag_path = out_dir / "run.bag"

    setup = (
        "source /opt/ros/jazzy/setup.bash && "
        "source /workspaces/cart_ws/install/setup.bash"
    )

    print(f"[run] dataset={dataset}  speed={speed}×  out={out_dir}")

    # ---- ONE-OFF inspection of the image so we know what's in there. ----
    # Writes to inspect.txt — pull alongside the bag to see.
    inspect_script = (
        "echo '=== /workspaces ==='; ls -la /workspaces/ 2>&1; "
        "echo '=== /workspaces/isaac_ros-dev (if exists) ==='; "
        "ls /workspaces/isaac_ros-dev/ 2>&1; "
        "ls /workspaces/isaac_ros-dev/install/ 2>&1; "
        "echo '=== /opt/ros/jazzy/share isaac packages ==='; "
        "ls /opt/ros/jazzy/share/ | grep -i isaac 2>&1; "
        "ls /opt/ros/jazzy/share/ | grep -i nvblox 2>&1; "
        "ls /opt/ros/jazzy/share/ | grep -i nitros 2>&1; "
        "echo '=== ros2 pkg list | grep isaac ==='; "
        "ros2 pkg list 2>&1 | grep -iE '(isaac|nvblox|nitros)' ; "
        "echo '=== find all visual_slam files ==='; "
        "find / -name '*visual_slam*' 2>/dev/null | head -30; "
        "echo '=== find all nvblox files ==='; "
        "find / -name '*nvblox*' 2>/dev/null | head -30"
    )
    subprocess.run(
        ["bash", "-c", f"{setup} && {inspect_script} > {out_dir}/inspect.txt 2>&1"],
        check=False,
    )
    print(f"[run] inspection written to {out_dir}/inspect.txt")

    procs: list[tuple[subprocess.Popen, str]] = []

    def spawn(name: str, cmd: str) -> subprocess.Popen:
        log = open(out_dir / f"{name}.log", "w")
        p = subprocess.Popen(
            ["bash", "-c", f"{setup} && {cmd}"],
            stdout=log, stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        procs.append((p, name))
        return p

    # 1. Full launch graph: tf_static + playback + cuVSLAM + EKF + navsat.
    spawn("launch", (
        f"ros2 launch cart_bringup dataset_replay.launch.py "
        f"dataset_dir:={dataset} speed:={speed}"
    ))

    # 2. Depth source — heavy, fail-isolated.
    spawn("depth", "ros2 run cart_sensors depth_anything_node")

    # Give cuVSLAM a beat to initialize.
    time.sleep(8)

    # 3. nvblox node — depends on /front_wide/depth + image_raw.
    # Isaac ROS 4.4 packages it under `nvblox_ros`. If executable name has
    # changed, we'll see "package not found" in nvblox.log and pivot.
    spawn("nvblox", (
        "ros2 run nvblox_ros nvblox_node --ros-args "
        "  -p use_sim_time:=false "
        "  -p global_frame:=odom "
        "  -p voxel_size:=0.1 "
        "  -p esdf:=true "
        "  -p esdf_2d:=true "
        "  -r color/image:=/front_wide/image_raw "
        "  -r color/camera_info:=/front_wide/camera_info "
        "  -r depth/image:=/front_wide/depth "
        "  -r depth/camera_info:=/front_wide/depth/camera_info"
    ))

    # First check what nvblox is actually called in this image — log it so
    # the next iteration knows what to call.
    spawn("inspect", (
        "ros2 pkg list 2>&1 | grep -i nvblox && "
        "ros2 pkg executables nvblox_ros 2>&1 | head -20"
    ))

    # 4. Bag record everything.
    spawn("bag", f"ros2 bag record -a -s mcap -o {bag_path}")

    print(f"[run] running for {duration_s}s ...")
    time.sleep(duration_s)

    print("[run] stopping (reverse order) ...")
    for proc, name in reversed(procs):
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=15)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except ProcessLookupError:
                pass
        print(f"[run]   {name} stopped")

    print("[run] post-processing rosbag")
    subprocess.run(
        ["bash", "-c",
         f"{setup} && python3 /workspaces/_extract_trajectories.py {bag_path} {out_dir}"],
        check=False,
    )

    # Persist volume changes so `modal volume get` sees the new files.
    volume.commit()

    print(f"[run] done. output: {out_dir}")
    print(f"      pull with: modal volume get stanford-cart-data "
          f"{str(out_dir).replace('/data/', '')} ./out/")
    return str(out_dir)


@app.function(
    image=isaac_image,
    secrets=[modal.Secret.from_name("ngc-credentials")],
    timeout=120,
)
def inspect_image() -> str:
    """Pure-Python inspection of the Isaac ROS container's package layout.
    Returns a string we can print locally so we know what to launch."""
    import subprocess
    import os

    out: list[str] = []

    def run(label: str, cmd: str) -> None:
        out.append(f"\n=== {label} ===")
        try:
            r = subprocess.run(["bash", "-c", cmd], capture_output=True,
                               text=True, timeout=20)
            out.append(r.stdout)
            if r.stderr.strip():
                out.append("STDERR: " + r.stderr)
        except Exception as e:
            out.append(f"ERROR: {e!r}")

    setup = "source /opt/ros/jazzy/setup.bash"

    run("workspaces top-level", "ls -la /workspaces/")
    run("isaac_ros-dev contents",
        "ls -la /workspaces/isaac_ros-dev/ 2>&1 || echo 'no isaac_ros-dev dir'")
    run("isaac_ros-dev install (if any)",
        "ls /workspaces/isaac_ros-dev/install/ 2>&1 || echo 'no install dir'")
    run("isaac_ros-dev src (if any)",
        "ls /workspaces/isaac_ros-dev/src/ 2>&1 || echo 'no src dir'")
    run("/opt/ros/jazzy/share isaac packages",
        "ls /opt/ros/jazzy/share/ | grep -iE '(isaac|nvblox|nitros|gxf)' || echo none")
    run("ros2 pkg list grep isaac/nvblox",
        f"{setup} && ros2 pkg list 2>&1 | grep -iE '(isaac|nvblox|nitros|gxf)' || echo none")
    run("find visual_slam (limit 30)",
        "find / -name '*visual_slam*' 2>/dev/null | grep -v /proc | head -30")
    run("find nvblox (limit 30)",
        "find / -name '*nvblox*' 2>/dev/null | grep -v /proc | grep -v /__modal | head -30")
    run("env vars hinting workspace",
        "env | grep -iE '(ROS|ISAAC|AMENT)' | sort")
    run("rosdep cache for isaac",
        "ls /etc/ros/rosdep/sources.list.d/ 2>&1; "
        "find /var/lib/ros -name '*isaac*' 2>/dev/null | head -10")

    return "\n".join(out)


@app.local_entrypoint()
def main(speed: float = 2.0, duration_s: int = 400):
    out_dir = run_dataset_replay.remote(speed=speed, duration_s=duration_s)
    print(f"\nDone. Output dir on volume: {out_dir}")
    rel = out_dir.replace("/data/", "")
    print(f"Pull with: modal volume get stanford-cart-data {rel} ./out/")


@app.local_entrypoint()
def inspect():
    """modal run modal_pipeline/run_isaac_ros.py::inspect — print the
    Isaac ROS image's package layout to stdout. Use this to figure out
    which packages / launch files / executables actually exist."""
    print(inspect_image.remote())
