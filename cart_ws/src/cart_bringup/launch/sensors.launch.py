"""Sensors only: 4 cameras + GPS. No DBW, no Nav2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction
from launch_ros.actions import Node, PushRosNamespace


def _camera(slug: str, params_file: str):
    return GroupAction([
        PushRosNamespace(slug),
        Node(
            package="cart_sensors",
            executable="usb_camera_node",
            name="usb_camera",
            parameters=[{
                # cart.yaml top-level keys for this slug
            }, {"yaml_section": slug}, params_file],
            output="screen",
        ),
    ])


def generate_launch_description():
    pkg = get_package_share_directory("cart_bringup")
    cart_yaml = os.path.join(pkg, "config", "cart.yaml")

    # NOTE: rclpy parameters in cart.yaml are nested under per-camera top-level
    # keys (front_narrow / front_wide / left / right). Plain `parameters=[file]`
    # only loads keys nested under the node's name. So we pass each camera its
    # own param file or inline params per camera. Here we inline-overlay.

    cams = []
    for slug, idx, flip, frame in [
        ("front_narrow", 0, "none", "front_narrow_optical"),
        ("front_wide",   6, "vertical", "front_wide_optical"),
        ("left",         2, "none", "left_optical"),
        ("right",       10, "none", "right_optical"),
    ]:
        cams.append(GroupAction([
            PushRosNamespace(slug),
            Node(
                package="cart_sensors",
                executable="usb_camera_node",
                name="usb_camera",
                parameters=[{
                    "device_index": idx,
                    "width": 640,
                    "height": 480,
                    "fps": 30,
                    "fourcc": "MJPG",
                    "frame_id": frame,
                    "flip": flip,
                    "publish_hz": 20.0,
                }],
                output="screen",
            ),
        ]))

    gps = Node(
        package="cart_sensors",
        executable="gps_mega_node",
        name="gps_mega",
        parameters=[cart_yaml],
        output="screen",
    )

    return LaunchDescription([*cams, gps])
