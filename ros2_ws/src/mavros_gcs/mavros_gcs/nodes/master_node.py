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

from mavros_gcs.nodes.teleop_keyboard import TeleopKeyboardNode
from mavros_gcs.nodes.info_panel import InfoPanelNode


class MasterNode:
    """
    Not a ROS node itself—just a small orchestration wrapper.
    """
    def __init__(self) -> None:
        self._executor: Optional[MultiThreadedExecutor] = None
        self._teleop: Optional[TeleopKeyboardNode] = None
        self._panel: Optional[InfoPanelNode] = None
        self._shutdown_requested = False

    def start(self) -> None:
        self._teleop = TeleopKeyboardNode()
        self._panel = InfoPanelNode()

        self._executor = MultiThreadedExecutor(num_threads=4)
        self._executor.add_node(self._teleop)
        time.sleep(0.5) # Give teleop a bit time to initialize itself
        self._executor.add_node(self._panel)

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
                if self._teleop is not None:
                    try:
                        self._executor.remove_node(self._teleop)
                    except Exception:
                        pass
                if self._panel is not None:
                    try:
                        self._executor.remove_node(self._panel)
                    except Exception:
                        pass
            except Exception:
                pass

        if self._teleop is not None:
            try:
                self._teleop.destroy_node()
            except Exception:
                pass
            self._teleop = None

        if self._panel is not None:
            try:
                self._panel.destroy_node()
            except Exception:
                pass
            self._panel = None

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