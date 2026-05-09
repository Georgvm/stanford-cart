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

isaac_image = (
    # NOTE: NO add_python — using the Ubuntu/ROS-supplied python3.12 so
    # catkin_pkg + ament tools are on the same interpreter as ros2 cli.
    # Adding a separate Python (e.g. add_python="3.12") splits the
    # environment and breaks colcon build of cart_bringup.
    modal.Image.from_registry("nvidia/cuda:13.0.0-runtime-ubuntu24.04")
    .env({
        "DEBIAN_FRONTEND": "noninteractive",
        "NVIDIA_DRIVER_CAPABILITIES": "all",
        "NVIDIA_VISIBLE_DEVICES": "all",
    })
    .apt_install(
        "software-properties-common", "curl", "gnupg", "lsb-release",
        "ca-certificates", "build-essential", "git", "wget", "python3-pip",
    )
    # --- ROS 2 Jazzy --------------------------------------------------------
    .run_commands(
        "add-apt-repository -y universe",
        # GPG key via HTTPS GET (no dirmngr, no keyserver protocol — Modal's
        # builder doesn't have ~/.gnupg or dirmngr running).
        "curl -sSL 'https://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654&options=mr' "
        "  -o /tmp/ros.key.asc && "
        "gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg /tmp/ros.key.asc",
        "echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] "
        "http://packages.ros.org/ros2/ubuntu noble main' "
        "> /etc/apt/sources.list.d/ros2.list",
        "apt-get update && apt-get install -y "
        "  ros-jazzy-desktop "
        "  ros-dev-tools "
        "  python3-colcon-common-extensions "
        "  ros-jazzy-robot-localization "
        "  ros-jazzy-nav2-bringup "
        "  ros-jazzy-rosbag2-storage-mcap",
    )
    # --- Isaac ROS 4.4 apt repo + packages ---------------------------------
    # Apt codename is the Ubuntu codename (`noble`), NOT the ROS distro
    # (`jazzy`). Verified: https://isaac.download.nvidia.com/isaac-ros/release-4.4/dists/noble/InRelease
    .run_commands(
        "curl -sSL https://isaac.download.nvidia.com/isaac-ros/repos.key "
        "  -o /tmp/isaac-ros.asc && "
        "gpg --dearmor -o /usr/share/keyrings/nvidia-isaac-ros.gpg /tmp/isaac-ros.asc",
        "echo 'deb [arch=amd64 signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] "
        "https://isaac.download.nvidia.com/isaac-ros/release-4.4 noble main' "
        "> /etc/apt/sources.list.d/isaac-ros.list",
        "apt-get update && apt-get install -y "
        "  ros-jazzy-isaac-ros-visual-slam "
        "  ros-jazzy-isaac-ros-image-pipeline "
        "  ros-jazzy-isaac-ros-nvblox "
        "  ros-jazzy-isaac-ros-dnn-image-encoder",
    )
    # --- Python runtime deps for our nodes ---------------------------------
    .pip_install(
        "opencv-python-headless",
        "pynmea2",
        "pyserial",
        "transformers",
        "torch",
        "torchvision",
        "pillow",
        "pyyaml",
        "scipy",
        "matplotlib",
        "numpy",
        "utm",
    )
    # --- Copy our workspace + build ----------------------------------------
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


@app.local_entrypoint()
def main(speed: float = 2.0, duration_s: int = 400):
    out_dir = run_dataset_replay.remote(speed=speed, duration_s=duration_s)
    print(f"\nDone. Output dir on volume: {out_dir}")
    rel = out_dir.replace("/data/", "")
    print(f"Pull with: modal volume get stanford-cart-data {rel} ./out/")
