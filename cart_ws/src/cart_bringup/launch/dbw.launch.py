"""DBW only: ODrive + Mega + cmd_vel translator. Useful for bench testing
without sensors or Nav2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cart_bringup")
    cart_yaml = os.path.join(pkg, "config", "cart.yaml")

    return LaunchDescription([
        Node(
            package="cart_dbw",
            executable="odrive_steering_node",
            name="odrive_steering",
            parameters=[cart_yaml],
            output="screen",
        ),
        Node(
            package="cart_dbw",
            executable="mega_pedals_node",
            name="mega_pedals",
            parameters=[cart_yaml],
            output="screen",
        ),
        Node(
            package="cart_dbw",
            executable="cmd_vel_to_dbw",
            name="cmd_vel_to_dbw",
            parameters=[cart_yaml],
            output="screen",
        ),
        # Safety gate sits in front of cmd_vel_to_dbw even on bench tests so
        # the same safety invariants are exercised. Without /run_demo true,
        # nothing moves.
        Node(
            package="cart_safety",
            executable="safety_gate",
            name="safety_gate",
            parameters=[cart_yaml],
            output="screen",
        ),
    ])
