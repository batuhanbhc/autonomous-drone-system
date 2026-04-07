#!/usr/bin/env python3
"""
fake_mcu_bridge.py
Mimics mcu_bridge for SITL: reads local_position/odom and publishes
the same Vector3Stamped topic that control_gate expects.
  vector.x = z_world_m   (position.z from odom)
  vector.y = vz_mps      (twist.linear.z from odom)
  vector.z = agl_m       (same as z_world for SITL)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped


class FakeMcuBridge(Node):
    def __init__(self):
        super().__init__('fake_mcu_bridge')

        self.declare_parameter('drone_id', 0)
        drone_id = self.get_parameter('drone_id').value
        base_ns = f'/drone_{drone_id}'

        odom_topic  = f'{base_ns}/mavros/local_position/odom'
        pub_topic   = f'{base_ns}/mcu_bridge/vertical_estimate'  # match your yaml

        self.get_logger().info(f'Subscribing: {odom_topic}')
        self.get_logger().info(f'Publishing:  {pub_topic}')

        qos_sub = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
        qos_pub = QoSProfile(depth=10,
                             reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)

        self.pub = self.create_publisher(Vector3Stamped, pub_topic, qos_pub)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, qos_sub)

    def on_odom(self, msg: Odometry):
        z  = msg.pose.pose.position.z
        vz = msg.twist.twist.linear.z

        out = Vector3Stamped()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.vector.x = z    # z_world_m
        out.vector.y = vz   # vz_world_mps
        out.vector.z = z    # agl_m (same as z for SITL, no ground offset)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = FakeMcuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()