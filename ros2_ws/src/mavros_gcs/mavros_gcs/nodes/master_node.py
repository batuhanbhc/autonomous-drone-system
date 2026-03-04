#!/usr/bin/env python3
"""
master_node.py

Single-terminal master that runs BOTH:
  - TeleopKeyboardNode
  - InfoPanelNode
"""

from __future__ import annotations

import signal
import time
from typing import Optional

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from drone_msgs.msg import GcsHeartbeat

from mavros_gcs.nodes.teleop_keyboard import TeleopKeyboardNode
from mavros_gcs.nodes.info_panel import InfoPanelNode


class HeartbeatNode(Node):
    def __init__(self) -> None:
        super().__init__("gcs_heartbeat_node")

        # Allow setting via CLI:
        #   --ros-args -p gcs_id:=1 -p drone_id:=7
        self.declare_parameter("gcs_id", 0)
        self.declare_parameter("drone_id", 0)

        self._gcs_id: int = self._read_uint8_param("gcs_id", default=0)
        self._drone_id: int = self._read_nonneg_int_param("drone_id", default=0)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        topic = f"/drone_{self._drone_id}/gcs/heartbeat"
        self._pub = self.create_publisher(GcsHeartbeat, topic, qos)
        self._timer = self.create_timer(1.0, self._tick)

        self.get_logger().info(
            f"Publishing GCS heartbeat on '{topic}' (gcs_id={self._gcs_id}, drone_id={self._drone_id})"
        )

    def _read_uint8_param(self, name: str, default: int = 0) -> int:
        p = self.get_parameter(name).value
        try:
            val = int(p)
        except (TypeError, ValueError):
            self.get_logger().warn(f"{name}='{p}' is not an integer; defaulting to {default}")
            return default

        if not (0 <= val <= 255):
            self.get_logger().warn(f"{name}={val} out of range [0..255]; defaulting to {default}")
            return default

        return val

    def _read_nonneg_int_param(self, name: str, default: int = 0) -> int:
        p = self.get_parameter(name).value
        try:
            val = int(p)
        except (TypeError, ValueError):
            self.get_logger().warn(f"{name}='{p}' is not an integer; defaulting to {default}")
            return default

        if val < 0:
            self.get_logger().warn(f"{name}={val} must be >= 0; defaulting to {default}")
            return default

        return val

    def _tick(self) -> None:
        msg = GcsHeartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        msg.gcs_id = self._gcs_id
        self._pub.publish(msg)


class MasterNode:
    def __init__(self) -> None:
        self._executor: Optional[MultiThreadedExecutor] = None
        self._teleop: Optional[TeleopKeyboardNode] = None
        self._panel: Optional[InfoPanelNode] = None
        self._heartbeat: Optional[HeartbeatNode] = None
        self._shutdown_requested = False

    def start(self) -> None:
        self._teleop = TeleopKeyboardNode()
        self._panel = InfoPanelNode()
        self._heartbeat = HeartbeatNode()

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._teleop)
        time.sleep(0.2)  # Give teleop a bit time to initialize itself
        self._executor.add_node(self._panel)
        self._executor.add_node(self._heartbeat)

        self._install_signal_handlers()

        try:
            self._executor.spin()
        finally:
            self.stop()

    def _install_signal_handlers(self) -> None:
        def _handler(signum, frame):
            if self._shutdown_requested:
                return
            self._shutdown_requested = True
            try:
                rclpy.shutdown()
            except Exception:
                pass

        signal.signal(signal.SIGINT, _handler)
        signal.signal(signal.SIGTERM, _handler)

    def stop(self) -> None:
        # Stop UI cleanly first so the terminal is restored
        if self._panel is not None:
            try:
                self._panel.shutdown_ui()
            except Exception:
                pass

        # Remove/destroy nodes
        if self._executor is not None:
            try:
                for n in (self._teleop, self._panel, self._heartbeat):
                    if n is not None:
                        try:
                            self._executor.remove_node(n)
                        except Exception:
                            pass
            except Exception:
                pass

        for attr in ("_teleop", "_panel", "_heartbeat"):
            n = getattr(self, attr)
            if n is not None:
                try:
                    n.destroy_node()
                except Exception:
                    pass
                setattr(self, attr, None)

        if self._executor is not None:
            try:
                self._executor.shutdown()
            except Exception:
                pass
            self._executor = None

        try:
            rclpy.try_shutdown()
        except Exception:
            pass


def main(argv=None) -> None:
    rclpy.init(args=argv)
    MasterNode().start()