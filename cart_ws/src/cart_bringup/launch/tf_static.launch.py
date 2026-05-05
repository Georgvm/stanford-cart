"""
tf_static — base_link → camera/gps frames.

Approximate measurements; remeasure on the cart and update.
Convention: ROS REP-103 (x-fwd, y-left, z-up). Optical frames are the
camera-coords convention (z-fwd, x-right, y-down) — that's why each cam
has a 90-degree rotation chain in its quaternion.

Quick refresher on the optical-frame quaternion (rpy = -pi/2, 0, -pi/2):
    qx, qy, qz, qw = (-0.5, 0.5, -0.5, 0.5)
That converts a body-frame (x-fwd) into camera-frame (z-fwd).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


# (parent, child, x, y, z, qx, qy, qz, qw)
_OPTICAL_Q = ("-0.5", "0.5", "-0.5", "0.5")

_TFS = [
    # Cameras — measure x (forward), y (left+), z (up) from base_link.
    ("base_link", "front_narrow_optical", "0.95", "0.0",  "1.30", *_OPTICAL_Q),
    ("base_link", "front_wide_optical",   "0.95", "0.0",  "1.20", *_OPTICAL_Q),
    ("base_link", "left_optical",         "0.40", "0.55", "1.20", *_OPTICAL_Q),
    ("base_link", "right_optical",        "0.40", "-0.55","1.20", *_OPTICAL_Q),
    # GPS antenna (assume roof-center of cart).
    ("base_link", "gps",                  "0.20", "0.0",  "1.50",
     "0.0", "0.0", "0.0", "1.0"),
]


def generate_launch_description():
    nodes = []
    for parent, child, x, y, z, qx, qy, qz, qw in _TFS:
        nodes.append(Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name=f"static_tf_{child}",
            arguments=[x, y, z, qx, qy, qz, qw, parent, child],
        ))
    return LaunchDescription(nodes)
