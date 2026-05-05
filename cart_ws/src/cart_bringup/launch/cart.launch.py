"""Top-level: sensors + cuVSLAM + nvblox + EKF + Nav2 + DBW + safety + demo runner.

Run on the Jetson Thor:
  ros2 launch cart_bringup cart.launch.py route_yaml:=/data/route_oval/waypoints.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = get_package_share_directory("cart_bringup")
    cart_yaml = os.path.join(pkg, "config", "cart.yaml")
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")
    ekf_params = os.path.join(pkg, "config", "ekf.yaml")

    route_yaml = LaunchConfiguration("route_yaml")

    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "sensors.launch.py")
        )
    )

    dbw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "dbw.launch.py")
        )
    )

    tf_static = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, "launch", "tf_static.launch.py")
        )
    )

    depth = Node(
        package="cart_sensors",
        executable="depth_anything_node",
        name="depth_anything",
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
            "enable_imu_fusion": False,   # NO IMU yet (BerryIMU pending)
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

    nvblox = Node(
        package="nvblox_ros",
        executable="nvblox_node",
        name="nvblox_node",
        parameters=[{
            "use_sim_time": False,
            "global_frame": "odom",
            "voxel_size": 0.1,
            "esdf": True,
            "esdf_2d": True,
            "max_integration_distance_m": 8.0,
        }],
        remappings=[
            ("color/image", "/front_wide/image_raw"),
            ("color/camera_info", "/front_wide/camera_info"),
            # Depth source: monocular depth from Depth Anything published as
            # /front_wide/depth, OR true stereo if calibrated. TODO.
            ("depth/image", "/front_wide/depth"),
            ("depth/camera_info", "/front_wide/camera_info"),
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

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py",
            ])
        ]),
        launch_arguments={
            "use_sim_time": "false",
            "params_file": nav2_params,
            "autostart": "true",
        }.items(),
    )

    perception_estop = Node(
        package="cart_safety",
        executable="perception_estop",
        name="perception_estop",
        parameters=[cart_yaml],
        output="screen",
    )

    demo_runner = Node(
        package="cart_safety",
        executable="demo_runner",
        name="demo_runner",
        parameters=[{
            "route_yaml": route_yaml,
            "loop": False,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("route_yaml",
                              default_value="/data/route_oval/waypoints.yaml"),
        tf_static,
        sensors,
        dbw,
        depth,
        cuvslam,
        nvblox,
        ekf_local,
        ekf_global,
        navsat,
        nav2,
        perception_estop,
        demo_runner,
    ])
