#!/usr/bin/env python3
from __future__ import annotations
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from drone_msgs.msg import GcsHeartbeat


class HeartbeatNode(Node):
    def __init__(self):
        super().__init__('gcs_heartbeat_node')

        self.declare_parameter('gcs_id', 0)
        self.declare_parameter('drone_id', 0)

        self._gcs_id   = self._read_uint8_param('gcs_id', default=0)
        self._drone_id = self._read_nonneg_int_param('drone_id', default=0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        topic = f'/drone_{self._drone_id}/gcs/heartbeat'
        self._pub   = self.create_publisher(GcsHeartbeat, topic, qos)
        self._timer = self.create_timer(1.0, self._tick)
        self.get_logger().info(f'Heartbeat on {topic}')

    def _read_uint8_param(self, name, default=0):
        try:
            val = int(self.get_parameter(name).value)
            return val if 0 <= val <= 255 else default
        except Exception:
            return default

    def _read_nonneg_int_param(self, name, default=0):
        try:
            val = int(self.get_parameter(name).value)
            return val if val >= 0 else default
        except Exception:
            return default

    def _tick(self):
        msg = GcsHeartbeat()
        msg.stamp  = self.get_clock().now().to_msg()
        msg.gcs_id = self._gcs_id
        self._pub.publish(msg)


def main(argv=None):
    rclpy.init(args=argv)
    node = HeartbeatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()