#!/usr/bin/env python3
"""
Unified MATLAB control bridge with safety watchdog.

For each crazyflie (cf_prefix), provides:
  Subscribe:  /<cf_prefix>/cmd_vel_twist   (geometry_msgs/Twist)
  Publish:    /<cf_prefix>/cmd_velocity_world (VelocityWorld)

Safety feature: if no cmd_vel_twist received for 1 second, auto-land at (0,0,0).
Extend this file to add more MATLAB-compatible control interfaces.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from crazyflie_interfaces.msg import VelocityWorld
from crazyflie_interfaces.srv import Land


class CfControlBridge(Node):
    def __init__(self):
        super().__init__("cf_control_bridge")

        self.declare_parameter("cf_prefixes", [""])
        cf_prefixes = self.get_parameter("cf_prefixes").get_parameter_value().string_array_value

        if not cf_prefixes:
            self.get_logger().warn("No cf_prefixes given, defaulting to ['cf_1']")
            cf_prefixes = ["cf_1"]

        self.declare_parameter("cmd_timeout", 1.0)
        self._cmd_timeout = self.get_parameter("cmd_timeout").get_parameter_value().double_value

        self._pubs = {}          # prefix → VelocityWorld publisher
        self._last_cmd = {}      # prefix → last command timestamp (seconds)
        self._landing = {}       # prefix → whether already triggered landing
        self._land_clients = {}  # prefix → Land service client

        for prefix in cf_prefixes:
            sub = self.create_subscription(
                Twist,
                prefix + "/cmd_vel_twist",
                lambda msg, p=prefix: self._vel_callback(msg, p),
                10,
            )
            pub = self.create_publisher(VelocityWorld, prefix + "/cmd_velocity_world", 10)
            self._pubs[prefix] = pub
            self._last_cmd[prefix] = self.get_clock().now().nanoseconds * 1e-9
            self._landing[prefix] = False
            self._land_clients[prefix] = self.create_client(Land, prefix + "/land")

            self.get_logger().info(
                f"[{prefix}] bridge ready: /{prefix}/cmd_vel_twist → /{prefix}/cmd_velocity_world"
            )

        # Watchdog timer at ~5 Hz
        self._timer = self.create_timer(0.2, self._watchdog)

    def _vel_callback(self, msg: Twist, prefix: str):
        """Forward velocity command and reset watchdog."""
        now = self.get_clock().now().nanoseconds * 1e-9
        self._last_cmd[prefix] = now
        self._landing[prefix] = False

        vw = VelocityWorld()
        vw.header.stamp = self.get_clock().now().to_msg()
        vw.vel.x = msg.linear.x
        vw.vel.y = msg.linear.y
        vw.vel.z = msg.linear.z
        vw.yaw_rate = msg.angular.z
        self._pubs[prefix].publish(vw)

    def _watchdog(self):
        """Check for command timeout and trigger auto-land if needed."""
        now = self.get_clock().now().nanoseconds * 1e-9
        for prefix in self._pubs:
            if self._landing[prefix]:
                continue
            elapsed = now - self._last_cmd[prefix]
            if elapsed > self._cmd_timeout:
                self.get_logger().warn(
                    f"[{prefix}] No cmd_vel_twist for {elapsed:.1f}s, auto-landing..."
                )
                self._landing[prefix] = True
                self._call_land(prefix)

    def _call_land(self, prefix: str):
        """Send a land request to the crazyflie."""
        req = Land.Request()
        req.height = 0.0
        req.duration.sec = 2
        req.duration.nanosec = 0
        req.group_mask = 0
        self._land_clients[prefix].call_async(req)


def main():
    rclpy.init()
    node = CfControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
