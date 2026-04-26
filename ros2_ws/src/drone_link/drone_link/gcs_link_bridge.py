from __future__ import annotations

import socket
import threading
import time

from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from rclpy.node import Node
import rclpy

from drone_link.bridge_common import (
    MAGIC,
    SocketBridgeBase,
    VIDEO_HEADER,
    build_bridge_config,
    deserialize_video_packet,
    load_yaml_from_pkg,
    make_best_effort_qos,
    recvall,
)


class GcsLinkBridge(Node, SocketBridgeBase):
    def __init__(self) -> None:
        Node.__init__(self, "gcs_link_bridge")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("config_pkg", "mavros_config")
        self.declare_parameter("config_rel", "config/control_params.yaml")
        self.declare_parameter("listen_host", "0.0.0.0")

        drone_id = int(self.get_parameter("drone_id").value)
        config_pkg = str(self.get_parameter("config_pkg").value)
        config_rel = str(self.get_parameter("config_rel").value)
        root_cfg = load_yaml_from_pkg(config_pkg, config_rel)

        self._config = build_bridge_config(drone_id, root_cfg)
        SocketBridgeBase.__init__(self, self, self._config)

        self._listen_host = str(self.get_parameter("listen_host").value).strip() or "0.0.0.0"
        self._server_thread = threading.Thread(target=self._server_loop, daemon=True)
        self._video_thread = threading.Thread(target=self._video_server_loop, daemon=True)

        self._telemetry_publishers = {}
        self._video_publisher = self.create_publisher(
            FFMPEGPacket,
            f"/drone_{self._config.drone_id}/camera/stream/out",
            make_best_effort_qos(),
        )

        for topic_id, outbound in self._config.outbound_topics.items():
            self._telemetry_publishers[topic_id] = self.create_publisher(
                outbound.msg_type, outbound.topic, outbound.qos
            )

        for topic_id, inbound in self._config.inbound_topics.items():
            self.create_subscription(
                inbound.msg_type,
                inbound.topic,
                self._make_inbound_callback(topic_id),
                inbound.qos,
            )

        self.get_logger().info(
            f"GCS link bridge listening on {self._listen_host}:{self._config.port} "
            f"(ROS_LOCALHOST_ONLY expected)"
        )
        self._server_thread.start()
        self._video_thread.start()

    def destroy_node(self):
        self.stop()
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=2.0)
        if self._video_thread.is_alive():
            self._video_thread.join(timeout=2.0)
        return super().destroy_node()

    def _make_inbound_callback(self, topic_id: int):
        def callback(msg):
            self.send_ros_message(topic_id, msg)

        return callback

    def _server_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._listen_host, self._config.port))
        server.listen(1)
        server.settimeout(1.0)

        try:
            while not self._stop_event.is_set():
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                self._set_socket(client)
                self.get_logger().info(f"Pi link connected from {addr[0]}:{addr[1]}")
                self.receive_loop(
                    client,
                    self._config.outbound_topics,  # type: ignore[arg-type]
                    self._telemetry_publishers,
                )
        finally:
            try:
                server.close()
            except OSError:
                pass

    def _video_server_loop(self) -> None:
        if self._config.video_transport == "udp":
            self._video_udp_loop()
        else:
            self._video_tcp_loop()

    def _video_udp_loop(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self._listen_host, self._config.video_port))
        sock.settimeout(1.0)
        self.get_logger().info(
            f"GCS video bridge listening on {self._listen_host}:{self._config.video_port} over udp"
        )
        try:
            while not self._stop_event.is_set():
                try:
                    payload, _addr = sock.recvfrom(65535)
                except socket.timeout:
                    continue
                self._publish_video_payload(payload)
        finally:
            try:
                sock.close()
            except OSError:
                pass

    def _video_tcp_loop(self) -> None:
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self._listen_host, self._config.video_port))
        server.listen(1)
        server.settimeout(1.0)
        self.get_logger().info(
            f"GCS video bridge listening on {self._listen_host}:{self._config.video_port} over tcp"
        )
        try:
            while not self._stop_event.is_set():
                try:
                    client, addr = server.accept()
                except socket.timeout:
                    continue
                client.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                self.get_logger().info(f"Pi video link connected from {addr[0]}:{addr[1]}")
                try:
                    while not self._stop_event.is_set():
                        header = recvall(client, VIDEO_HEADER.size)
                        magic, payload_len = VIDEO_HEADER.unpack(header)
                        if magic != MAGIC:
                            raise ConnectionError("Invalid video bridge frame magic")
                        payload = recvall(client, payload_len)
                        self._publish_video_payload(payload)
                except (ConnectionError, OSError) as exc:
                    if not self._stop_event.is_set():
                        self.get_logger().warn(f"Video bridge connection closed: {exc}")
                finally:
                    try:
                        client.close()
                    except OSError:
                        pass
                    time.sleep(0.2)
        finally:
            try:
                server.close()
            except OSError:
                pass

    def _publish_video_payload(self, payload: bytes) -> None:
        try:
            self._video_publisher.publish(deserialize_video_packet(payload))
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"Dropping malformed video payload: {exc}")


def main(argv=None):
    rclpy.init(args=argv)
    node = GcsLinkBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
