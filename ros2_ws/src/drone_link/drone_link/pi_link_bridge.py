from __future__ import annotations

import threading
import time

from drone_msgs.msg import Toggle
from rclpy.node import Node
import rclpy

from drone_link.bridge_common import (
    SocketBridgeBase,
    TOPIC_STREAM_ACTIVE,
    build_bridge_config,
    load_yaml_from_pkg,
)


class PiLinkBridge(Node, SocketBridgeBase):
    def __init__(self) -> None:
        Node.__init__(self, "pi_link_bridge")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("config_pkg", "mavros_config")
        self.declare_parameter("config_rel", "config/control_params.yaml")
        self.declare_parameter("gcs_host", "")

        drone_id = int(self.get_parameter("drone_id").value)
        config_pkg = str(self.get_parameter("config_pkg").value)
        config_rel = str(self.get_parameter("config_rel").value)
        root_cfg = load_yaml_from_pkg(config_pkg, config_rel)

        self._config = build_bridge_config(drone_id, root_cfg)
        SocketBridgeBase.__init__(self, self, self._config)

        self._gcs_host = str(self.get_parameter("gcs_host").value).strip()
        if not self._gcs_host:
            self._gcs_host = str(root_cfg.get("link", {}).get("gcs_host", "")).strip()
        if not self._gcs_host:
            raise RuntimeError("pi_link_bridge requires a non-empty gcs_host parameter")

        self._inbound_publishers = {}
        self._connect_thread = threading.Thread(target=self._connect_loop, daemon=True)

        for topic_id, inbound in self._config.inbound_topics.items():
            self._inbound_publishers[topic_id] = self.create_publisher(
                inbound.msg_type, inbound.topic, inbound.qos
            )

        for topic_id, outbound in self._config.outbound_topics.items():
            self.create_subscription(
                outbound.msg_type,
                outbound.topic,
                self._make_outbound_callback(topic_id, outbound.throttle_hz),
                outbound.qos,
            )

        self.get_logger().info(
            f"Pi link bridge connecting to {self._gcs_host}:{self._config.port} "
            f"(ROS_LOCALHOST_ONLY expected)"
        )
        self._connect_thread.start()

    def destroy_node(self):
        self.stop()
        if self._connect_thread.is_alive():
            self._connect_thread.join(timeout=2.0)
        return super().destroy_node()

    def _make_outbound_callback(self, topic_id: int, throttle_hz: float):
        def callback(msg):
            if topic_id == TOPIC_STREAM_ACTIVE and isinstance(msg, Toggle):
                self.set_stream_active(bool(msg.state))
            if not self.should_send_topic(topic_id, throttle_hz):
                return
            self.send_ros_message(topic_id, msg)

        return callback

    def _connect_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                import socket
                sock = socket.create_connection((self._gcs_host, self._config.port), timeout=5.0)
                sock.settimeout(None)
                self._set_socket(sock)
                self.get_logger().info(
                    f"Connected to GCS bridge at {self._gcs_host}:{self._config.port}"
                )
                self.receive_loop(sock, self._config.inbound_topics, self._inbound_publishers)
            except OSError as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warn(
                        f"Waiting for GCS bridge {self._gcs_host}:{self._config.port}: {exc}"
                    )
                    time.sleep(self._config.reconnect_s)


def main(argv=None):
    rclpy.init(args=argv)
    node = PiLinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
