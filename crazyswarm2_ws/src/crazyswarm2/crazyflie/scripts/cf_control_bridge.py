#!/usr/bin/env python3
"""
Bridge: stores latest cmd_vel_twist and pose, forwards both at 100 Hz.
  - sub /cf_1/my_vel_cmd (Twist)        → store → pub /cf_0/cmd_velocity_world (VelocityWorld)
  - sub /cf_1/my_pose (PoseStamped)            → store → pub /cf_0/state (PoseStamped)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from crazyflie_interfaces.msg import VelocityWorld


class CfControlBridge(Node):
    def __init__(self):
        super().__init__("cf_control_bridge")

        self._latest_cmd = None
        self._latest_pose = None

        self._cmd_pub = self.create_publisher(VelocityWorld, "/cf_1/cmd_velocity_world", 10)
        
        self._state_pub = self.create_publisher(PoseStamped, "/cf_1/my_state", 10)

        qos_cmd = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Twist, "/cf_1/my_cmd_vel", self._on_cmd, qos_cmd)
        
        self.create_subscription(PoseStamped, "/cf_1/pose", self._on_pose, 10)

        self._timer = self.create_timer(0.01, self._forward)

        self._cmd_count = 0
        self._pose_count = 0

        self.get_logger().info("Bridge ready at 100 Hz")

    def _on_cmd(self, msg: Twist):
        self._latest_cmd = msg
        self._cmd_count += 1

    def _on_pose(self, msg: PoseStamped):
        self._latest_pose = msg
        self._pose_count += 1

    def _forward(self):
        if self._latest_cmd is not None:
            vw = VelocityWorld()
            vw.header.stamp = self.get_clock().now().to_msg()
            vw.vel.x = self._latest_cmd.linear.x
            vw.vel.y = self._latest_cmd.linear.y
            vw.vel.z = self._latest_cmd.linear.z
            vw.yaw_rate = self._latest_cmd.angular.z
            self._cmd_pub.publish(vw)

        if self._latest_pose is not None:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self._latest_pose.header.frame_id
            msg.pose = self._latest_pose.pose
            self._state_pub.publish(msg)

        # if self._cmd_count % 10 == 1:
        #     self.get_logger().info(
        #         f"Fwd: cmd #{self._cmd_count} pose #{self._pose_count}"
        #     )


def main():
    rclpy.init()
    node = CfControlBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
