from __future__ import annotations

import socket
import threading
import time

from drone_msgs.msg import Toggle
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from rclpy.node import Node
import rclpy

from drone_link.bridge_common import (
    MAX_UDP_PAYLOAD,
    SocketBridgeBase,
    TOPIC_STREAM_ACTIVE,
    build_bridge_config,
    load_yaml_from_pkg,
    serialize_video_packet,
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
        self._video_thread = threading.Thread(target=self._video_loop, daemon=True)
        self._video_socket = None

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

        self.create_subscription(
            FFMPEGPacket,
            self._config.video_topic,
            self._on_video_packet,
            self._make_video_qos(),
        )

        self.get_logger().info(
            f"Pi link bridge connecting to {self._gcs_host}:{self._config.port} "
            f"(ROS_LOCALHOST_ONLY expected)"
        )
        self._connect_thread.start()
        self._video_thread.start()

    def destroy_node(self):
        self.stop()
        if self._connect_thread.is_alive():
            self._connect_thread.join(timeout=2.0)
        self._close_video_socket()
        if self._video_thread.is_alive():
            self._video_thread.join(timeout=2.0)
        return super().destroy_node()

    def _make_video_qos(self):
        from drone_link.bridge_common import make_best_effort_qos
        return make_best_effort_qos()

    def _make_outbound_callback(self, topic_id: int, throttle_hz: float):
        def callback(msg):
            if topic_id == TOPIC_STREAM_ACTIVE and isinstance(msg, Toggle):
                self.set_stream_active(bool(msg.state))
            if not self.should_send_topic(topic_id, throttle_hz):
                return
            self.send_ros_message(topic_id, msg)

        return callback

    def _on_video_packet(self, msg: FFMPEGPacket) -> None:
        if not self.is_stream_active():
            return
        payload = serialize_video_packet(msg)
        if self._config.video_transport == "udp" and len(payload) > MAX_UDP_PAYLOAD:
            self.get_logger().warn(
                f"Dropping oversize UDP video packet ({len(payload)} bytes). "
                "Use h264 or tcp transport for large frames."
            )
            return
        self._send_video_payload(payload)

    def _connect_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                sock = socket.create_connection((self._gcs_host, self._config.port), timeout=5.0)
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

    def _close_video_socket(self) -> None:
        sock = self._video_socket
        self._video_socket = None
        if sock is None:
            return
        try:
            sock.shutdown(socket.SHUT_RDWR)
        except OSError:
            pass
        try:
            sock.close()
        except OSError:
            pass

    def _ensure_video_socket(self):
        if self._config.video_transport == "udp":
            if self._video_socket is None:
                self._video_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            return self._video_socket

        if self._video_socket is not None:
            return self._video_socket

        sock = socket.create_connection((self._gcs_host, self._config.video_port), timeout=5.0)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._video_socket = sock
        self.get_logger().info(
            f"Connected video bridge to {self._gcs_host}:{self._config.video_port} "
            f"over {self._config.video_transport}"
        )
        return sock

    def _video_loop(self) -> None:
        if self._config.video_transport == "udp":
            self.get_logger().info(
                f"Video bridge using UDP to {self._gcs_host}:{self._config.video_port}"
            )
            return

        while not self._stop_event.is_set():
            try:
                self._ensure_video_socket()
                while self._video_socket is not None and not self._stop_event.is_set():
                    time.sleep(1.0)
            except OSError as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warn(
                        f"Waiting for GCS video bridge {self._gcs_host}:{self._config.video_port}: {exc}"
                    )
                    self._close_video_socket()
                    time.sleep(self._config.reconnect_s)

    def _send_video_payload(self, payload: bytes) -> None:
        try:
            sock = self._ensure_video_socket()
            if self._config.video_transport == "udp":
                sock.sendto(payload, (self._gcs_host, self._config.video_port))
            else:
                from drone_link.bridge_common import MAGIC, VIDEO_HEADER
                sock.sendall(VIDEO_HEADER.pack(MAGIC, len(payload)) + payload)
        except OSError as exc:
            self.get_logger().warn(f"Video bridge send failed: {exc}")
            self._close_video_socket()


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
