"""
Teleop launch — PS5 controller → /cmd_vel_nav (through safety_gate → DBW).

Topic flow:
  joy_node  → /joy
  teleop_twist_joy → /cmd_vel_nav   (axis mapping below)
  engage_button → /run_demo (latched Bool, true while L1 held)
  → safety_gate → /cmd_vel → cmd_vel_to_dbw → ODrive + Mega

Why /cmd_vel_nav and not /cmd_vel directly: keep the safety chain consistent
with autonomy. The same gate, watchdog, and limits exercise on both paths.

Why the engage button publishes /run_demo: safety_gate already gates on
/run_demo. Holding L1 is a dead-man switch. Release = stop. No new topic.

PS5 (DualSense) joystick mapping (works with the standard joy node):
  axes[1] = left stick Y     → linear.x  (push up = forward)
  axes[3] = right stick X    → angular.z (push left = left turn)
  buttons[4] = L1            → engage (dead-man)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("cart_bringup")
    cart_yaml = os.path.join(pkg, "config", "cart.yaml")

    joy_dev = LaunchConfiguration("joy_dev")
    include_dbw = LaunchConfiguration("include_dbw")  # set false for sim/bench

    joy = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "device_id": 0,
            "deadzone": 0.05,
            "autorepeat_rate": 20.0,
        }],
    )

    teleop = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy",
        parameters=[{
            "axis_linear.x": 1,
            "axis_angular.yaw": 3,
            "scale_linear.x": 1.5,            # m/s @ full deflection (under safety cap)
            "scale_angular.yaw": 0.6,         # rad/s @ full deflection
            "enable_button": -1,              # use our own dead-man via /run_demo
            "require_enable_button": False,
        }],
        remappings=[
            ("/cmd_vel", "/cmd_vel_nav"),     # route through safety_gate
        ],
    )

    engage = Node(
        package="cart_safety",
        executable="engage_button",
        name="engage_button",
        parameters=[{
            "engage_button_index": 4,         # L1 on DualSense
        }],
        output="screen",
    )

    safety = Node(
        package="cart_safety",
        executable="safety_gate",
        name="safety_gate",
        parameters=[cart_yaml],
        output="screen",
    )

    # DBW chain (optional — bench tests may want to run teleop without
    # actuators). When false, you'll see /cmd_vel get published but no motion.
    dbw_nodes = [
        Node(
            package="cart_dbw",
            executable="cmd_vel_to_dbw",
            name="cmd_vel_to_dbw",
            parameters=[cart_yaml],
            output="screen",
            condition=None,
        ),
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
    ]

    return LaunchDescription([
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("include_dbw", default_value="true"),
        joy,
        teleop,
        engage,
        safety,
        *dbw_nodes,
    ])
