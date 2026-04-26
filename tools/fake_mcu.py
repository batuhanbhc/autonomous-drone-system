#!/usr/bin/env python3
"""
fake_mcu_bridge.py
Mimics mcu_bridge for SITL: reads local_position/odom and publishes
the same McuVerticalEstimate topic that control_gate expects.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from drone_msgs.msg import McuVerticalEstimate


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

        self.pub = self.create_publisher(McuVerticalEstimate, pub_topic, qos_pub)
        self.sub = self.create_subscription(Odometry, odom_topic, self.on_odom, qos_sub)

    def on_odom(self, msg: Odometry):
        z  = msg.pose.pose.position.z
        vz = msg.twist.twist.linear.z

        out = McuVerticalEstimate()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'world'
        out.z_world_m = z
        out.vz_world_mps = vz
        out.agl_m = z
        out.ekf_initialized = True
        out.lidar_accepted = True
        out.latest_lidar_m = z
        out.lidar_age_ms = 0
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
