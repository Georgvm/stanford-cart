"""
demo_runner — load a waypoints.yaml, wait for /run_demo, then send the
GPS waypoint list to Nav2's FollowGPSWaypoints action server.
"""

import math

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import Bool

from geographic_msgs.msg import GeoPose, GeoPoseStamped
from nav2_msgs.action import FollowGPSWaypoints


class DemoRunnerNode(Node):
    def __init__(self):
        super().__init__("demo_runner")
        self.declare_parameter("route_yaml", "/data/route_oval/waypoints.yaml")
        self.declare_parameter("loop", False)
        self.path = self.get_parameter("route_yaml").value
        self.loop = bool(self.get_parameter("loop").value)

        self.run = False
        self.in_flight = False

        latching = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, "/run_demo", self._on_run_demo, latching)
        self.client = ActionClient(self, FollowGPSWaypoints, "follow_gps_waypoints")
        self.create_timer(1.0, self._tick)
        self.get_logger().info(f"demo_runner up. route={self.path}")

    def _on_run_demo(self, msg: Bool):
        self.run = bool(msg.data)

    def _load(self):
        with open(self.path) as f:
            data = yaml.safe_load(f)
        out = []
        for wp in data["waypoints"]:
            gp = GeoPose()
            gp.position.latitude = float(wp["latitude"])
            gp.position.longitude = float(wp["longitude"])
            gp.position.altitude = 0.0
            yaw = float(wp.get("yaw", 0.0))
            gp.orientation.z = math.sin(yaw / 2.0)
            gp.orientation.w = math.cos(yaw / 2.0)
            stamped = GeoPoseStamped()
            stamped.header.frame_id = "map"
            stamped.pose = gp
            out.append(stamped)
        return out

    def _tick(self):
        if not self.run or self.in_flight:
            return
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn("FollowGPSWaypoints server not up yet")
            return
        wps = self._load()
        self.get_logger().info(f"sending {len(wps)} waypoints")
        goal = FollowGPSWaypoints.Goal()
        goal.gps_poses = wps
        self.in_flight = True
        future = self.client.send_goal_async(goal, feedback_callback=self._fb)
        future.add_done_callback(self._goal_done)

    def _fb(self, fb):
        self.get_logger().info(f"current waypoint: {fb.feedback.current_waypoint}")

    def _goal_done(self, future):
        gh = future.result()
        if not gh.accepted:
            self.in_flight = False
            self.get_logger().error("goal rejected")
            return
        gh.get_result_async().add_done_callback(self._result_done)

    def _result_done(self, future):
        result = future.result().result
        self.get_logger().info(f"done. missed={list(result.missed_waypoints)}")
        self.in_flight = False
        if not self.loop:
            self.run = False


def main():
    rclpy.init()
    rclpy.spin(DemoRunnerNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
