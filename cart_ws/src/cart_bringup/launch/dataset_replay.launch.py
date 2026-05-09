"""
Dataset replay launch — replay a recorded session into the same launch
graph that runs on the live cart.

  ros2 launch cart_bringup dataset_replay.launch.py \
       dataset_dir:=/path/to/Caddy-Training-Data-2026-05-03_16-08-00 \
       speed:=1.0

Wires:
  dataset_playback (cart_sensors)
   → /front_*/image_raw + /fix + /odom/arkit (+ /imu/arkit if imu_publish:=true)
   → tf_static (optical frames)
   → cuVSLAM (consumes the camera streams)
   → ekf_local + ekf_global + navsat_transform (consume cuVSLAM odom + ARKit
     odom + GPS — robot_localization happily fuses the redundancy)

NOT wired here: nvblox (needs a depth source — depth_anything could run on
the dataset front_wide stream too, but it's heavy and decoupled from this
test), Nav2 (no goals to plan to during pure playback), DBW (no actuators
in playback). Layer those in once cuVSLAM tracks happily on the dataset.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cart_bringup")
    ekf_params = os.path.join(pkg, "config", "ekf.yaml")

    dataset_dir = LaunchConfiguration("dataset_dir")
    speed = LaunchConfiguration("speed")
    loop = LaunchConfiguration("loop")
    imu_publish = LaunchConfiguration("imu_publish")

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tf_static.launch.py")
        )
    )

    playback = Node(
        package="cart_sensors",
        executable="dataset_playback_node",
        name="dataset_playback",
        parameters=[{
            "dataset_dir": dataset_dir,
            "speed": speed,
            "loop": loop,
            "imu_publish": imu_publish,
            "odom_frame": "arkit_odom",
            "base_frame": "base_link",
        }],
        output="screen",
    )

    cuvslam = Node(
        package="isaac_ros_visual_slam",
        executable="visual_slam_node",
        name="visual_slam",
        parameters=[{
            "use_sim_time": False,
            "denoise_input_images": False,
            "rectified_images": False,
            "enable_imu_fusion": False,
            "publish_map_to_odom_tf": False,
            "publish_odom_to_base_tf": False,
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "num_cameras": 2,
            "min_num_images": 2,
        }],
        remappings=[
            ("visual_slam/image_0", "/front_narrow/image_raw"),
            ("visual_slam/camera_info_0", "/front_narrow/camera_info"),
            ("visual_slam/image_1", "/front_wide/image_raw"),
            ("visual_slam/camera_info_1", "/front_wide/camera_info"),
        ],
    )

    ekf_local = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_local",
        parameters=[ekf_params],
        remappings=[("/odometry/filtered", "/odometry/filtered/local")],
    )

    ekf_global = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_global",
        parameters=[ekf_params],
        remappings=[("/odometry/filtered", "/odometry/filtered/global")],
    )

    navsat = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        parameters=[ekf_params],
        remappings=[
            ("/gps/fix", "/fix"),
            ("/odometry/filtered", "/odometry/filtered/global"),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("dataset_dir", description="Path to the recorded session directory"),
        DeclareLaunchArgument("speed", default_value="1.0"),
        DeclareLaunchArgument("loop", default_value="false"),
        DeclareLaunchArgument("imu_publish", default_value="false"),
        tf_static,
        playback,
        cuvslam,
        ekf_local,
        ekf_global,
        navsat,
    ])
