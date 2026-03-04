#!/usr/bin/env python3
"""
master_node.py

Single-terminal master that runs BOTH:
  - TeleopKeyboardNode
  - InfoPanelNode
"""

from __future__ import annotations

import os
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

        self._gcs_id: int = self._read_gcs_id_uint8()

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub = self.create_publisher(GcsHeartbeat, f"/gcs_{self._gcs_id}/heartbeat", qos)
        self._timer = self.create_timer(1.0, self._tick)

    def _read_gcs_id_uint8(self) -> int:
        raw = os.environ.get("GCS_ID")
        if raw is None or raw.strip() == "":
            return 0

        try:
            val = int(raw, 0)  # allows "42", "0x2A", etc.
        except ValueError:
            self.get_logger().warn(f"GCS_ID='{raw}' is not an integer; defaulting to 0")
            return 0

        if not (0 <= val <= 255):
            self.get_logger().warn(f"GCS_ID={val} out of range [0..255]; defaulting to 0")
            return 0

        return val

    def _tick(self) -> None:
        msg = GcsHeartbeat()
        msg.stamp = self.get_clock().now().to_msg()
        msg.gcs_id = self._gcs_id
        self._pub.publish(msg)


class MasterNode:
    """
    Not a ROS node itself—just a small orchestration wrapper.
    """
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
        time.sleep(0.5)  # Give teleop a bit time to initialize itself
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
    rclpy.init(args=[] if argv is None else argv)
    MasterNode().start()