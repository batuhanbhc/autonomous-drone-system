from __future__ import annotations

from dataclasses import dataclass
import socket
import struct
import threading
import time
from typing import Any, Callable, Dict, Optional

from ament_index_python.packages import get_package_share_directory
from drone_msgs.msg import (
    DroneInfo,
    DroneState,
    GcsHeartbeat,
    McuVerticalEstimate,
    TeleopAction,
    TeleopCommand,
    Toggle,
)
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket
from mavros_msgs.msg import ExtendedState, GPSRAW, State, StatusText
from nav_msgs.msg import Odometry
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.serialization import deserialize_message, serialize_message
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32
import yaml


MAGIC = b"DLNK"
HEADER = struct.Struct("!4sHI")
VIDEO_HEADER = struct.Struct("!4sI")
MAX_UDP_PAYLOAD = 65507

TOPIC_TELEOP_COMMAND = 1
TOPIC_TELEOP_ACTION = 2
TOPIC_GCS_HEARTBEAT = 3
TOPIC_RECORD_CMD = 4
TOPIC_STREAM_CMD = 5
TOPIC_MAVROS_STATE = 101
TOPIC_MAVROS_BATTERY = 102
TOPIC_MAVROS_EXTENDED_STATE = 103
TOPIC_MAVROS_STATUSTEXT = 104
TOPIC_MAVROS_ODOM = 105
TOPIC_MAVROS_GPS = 106
TOPIC_CMD_GATE_STATE = 107
TOPIC_CMD_GATE_INFO = 108
TOPIC_MCU_VERTICAL_ESTIMATE = 109
TOPIC_ALTITUDE_OUTPUT = 110
TOPIC_RECORD_ACTIVE = 111
TOPIC_STREAM_ACTIVE = 112
def make_reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def make_best_effort_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


@dataclass(frozen=True)
class OutboundTopic:
    topic_id: int
    topic: str
    msg_type: Any
    qos: QoSProfile
    throttle_hz: float = 0.0


@dataclass(frozen=True)
class InboundTopic:
    topic_id: int
    topic: str
    msg_type: Any
    qos: QoSProfile


@dataclass(frozen=True)
class BridgeConfig:
    drone_id: int
    port: int
    reconnect_s: float
    video_port: int
    video_transport: str
    video_topic: str
    outbound_topics: Dict[int, OutboundTopic]
    inbound_topics: Dict[int, InboundTopic]


def load_yaml_from_pkg(pkg: str, rel: str) -> dict:
    yaml_path = get_package_share_directory(pkg) + "/" + rel
    with open(yaml_path, "r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a dict: {yaml_path}")
    return data


def build_bridge_config(drone_id: int, root_cfg: dict) -> BridgeConfig:
    base = f"/drone_{drone_id}"
    mavros_topics = root_cfg.get("mavros_topics", {})
    custom_topics = root_cfg.get("custom_topics", {})
    link = root_cfg.get("link", {})
    streaming = root_cfg.get("streaming", {})

    reliable_qos = make_reliable_qos()
    best_effort_qos = make_best_effort_qos()
    sensor_qos = qos_profile_sensor_data

    telemetry_cfg = link.get("telemetry", {})

    outbound_topics = {
        TOPIC_MAVROS_STATE: OutboundTopic(
            TOPIC_MAVROS_STATE, base + mavros_topics["state"], State, reliable_qos),
        TOPIC_MAVROS_BATTERY: OutboundTopic(
            TOPIC_MAVROS_BATTERY, base + mavros_topics["battery"], BatteryState, sensor_qos),
        TOPIC_MAVROS_EXTENDED_STATE: OutboundTopic(
            TOPIC_MAVROS_EXTENDED_STATE, base + mavros_topics["extended_state"], ExtendedState, reliable_qos),
        TOPIC_MAVROS_STATUSTEXT: OutboundTopic(
            TOPIC_MAVROS_STATUSTEXT, base + mavros_topics["statustext"], StatusText, sensor_qos),
        TOPIC_MAVROS_ODOM: OutboundTopic(
            TOPIC_MAVROS_ODOM,
            base + mavros_topics["odom"],
            Odometry,
            sensor_qos,
            float(telemetry_cfg.get("odom_hz", 10.0)),
        ),
        TOPIC_MAVROS_GPS: OutboundTopic(
            TOPIC_MAVROS_GPS, base + mavros_topics["gps1_raw"], GPSRAW, sensor_qos),
        TOPIC_CMD_GATE_STATE: OutboundTopic(
            TOPIC_CMD_GATE_STATE, base + "/cmd_gate/state", DroneState, reliable_qos),
        TOPIC_CMD_GATE_INFO: OutboundTopic(
            TOPIC_CMD_GATE_INFO, base + "/cmd_gate/info", DroneInfo, reliable_qos),
        TOPIC_MCU_VERTICAL_ESTIMATE: OutboundTopic(
            TOPIC_MCU_VERTICAL_ESTIMATE,
            base + custom_topics["mcu_bridge"],
            McuVerticalEstimate,
            sensor_qos,
            float(telemetry_cfg.get("vertical_estimate_hz", 10.0)),
        ),
        TOPIC_ALTITUDE_OUTPUT: OutboundTopic(
            TOPIC_ALTITUDE_OUTPUT,
            base + custom_topics["alt_ctrl_output"],
            Float32,
            sensor_qos,
            float(telemetry_cfg.get("altitude_output_hz", 5.0)),
        ),
        TOPIC_RECORD_ACTIVE: OutboundTopic(
            TOPIC_RECORD_ACTIVE, base + "/camera/record/active", Toggle, reliable_qos),
        TOPIC_STREAM_ACTIVE: OutboundTopic(
            TOPIC_STREAM_ACTIVE, base + "/camera/stream/active", Toggle, reliable_qos),
    }

    inbound_topics = {
        TOPIC_TELEOP_COMMAND: InboundTopic(
            TOPIC_TELEOP_COMMAND, base + custom_topics["manual_command"], TeleopCommand, reliable_qos),
        TOPIC_TELEOP_ACTION: InboundTopic(
            TOPIC_TELEOP_ACTION, base + custom_topics["manual_action"], TeleopAction, best_effort_qos),
        TOPIC_GCS_HEARTBEAT: InboundTopic(
            TOPIC_GCS_HEARTBEAT, base + "/gcs/heartbeat", GcsHeartbeat, best_effort_qos),
        TOPIC_RECORD_CMD: InboundTopic(
            TOPIC_RECORD_CMD, base + "/camera/record/cmd", Toggle, reliable_qos),
        TOPIC_STREAM_CMD: InboundTopic(
            TOPIC_STREAM_CMD, base + "/camera/stream/cmd", Toggle, reliable_qos),
    }

    return BridgeConfig(
        drone_id=drone_id,
        port=int(link.get("port", 50010)),
        reconnect_s=float(link.get("reconnect_s", 1.0)),
        video_port=int(link.get("video", {}).get("port", 50020)),
        video_transport=str(link.get("video", {}).get("transport", "udp")).strip().lower() or "udp",
        video_topic=f"{base}/camera/stream/out",
        outbound_topics=outbound_topics,
        inbound_topics=inbound_topics,
    )


def recvall(sock: socket.socket, size: int) -> bytes:
    chunks = bytearray()
    while len(chunks) < size:
        chunk = sock.recv(size - len(chunks))
        if not chunk:
            raise ConnectionError("Socket closed while receiving data")
        chunks.extend(chunk)
    return bytes(chunks)


class SocketBridgeBase:
    def __init__(self, node, config: BridgeConfig) -> None:
        self._node = node
        self._config = config
        self._send_lock = threading.Lock()
        self._socket_lock = threading.Lock()
        self._socket: Optional[socket.socket] = None
        self._last_send_s: Dict[int, float] = {}
        self._stop_event = threading.Event()
        self._stream_active = False

    def stop(self) -> None:
        self._stop_event.set()
        self._close_socket()

    def _close_socket(self) -> None:
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass

    def _set_socket(self, sock: socket.socket) -> None:
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        with self._socket_lock:
            self._socket = sock

    def _get_socket(self) -> Optional[socket.socket]:
        with self._socket_lock:
            return self._socket

    def _send_payload(self, topic_id: int, payload: bytes) -> bool:
        sock = self._get_socket()
        if sock is None:
            return False

        frame = HEADER.pack(MAGIC, topic_id, len(payload)) + payload
        try:
            with self._send_lock:
                sock.sendall(frame)
            return True
        except OSError as exc:
            self._node.get_logger().warn(f"Bridge send failed on topic_id={topic_id}: {exc}")
            self._close_socket()
            return False

    def send_ros_message(self, topic_id: int, msg) -> bool:
        return self._send_payload(topic_id, serialize_message(msg))

    def should_send_topic(self, topic_id: int, throttle_hz: float) -> bool:
        if throttle_hz <= 0.0:
            return True
        now_s = time.monotonic()
        min_period = 1.0 / throttle_hz
        last = self._last_send_s.get(topic_id)
        if last is not None and (now_s - last) < min_period:
            return False
        self._last_send_s[topic_id] = now_s
        return True

    def set_stream_active(self, active: bool) -> None:
        self._stream_active = active

    def is_stream_active(self) -> bool:
        return self._stream_active

    def handle_incoming_ros_message(self, binding, payload: bytes, publishers: Dict[int, Any]) -> None:
        publishers[binding.topic_id].publish(deserialize_message(payload, binding.msg_type))

    def receive_loop(
        self,
        sock: socket.socket,
        inbound_topics: Dict[int, InboundTopic],
        publishers: Dict[int, Any],
        on_disconnect: Optional[Callable[[], None]] = None,
    ) -> None:
        try:
            while not self._stop_event.is_set():
                header = recvall(sock, HEADER.size)
                magic, topic_id, payload_len = HEADER.unpack(header)
                if magic != MAGIC:
                    raise ConnectionError("Invalid bridge frame magic")
                payload = recvall(sock, payload_len)
                inbound = inbound_topics.get(topic_id)
                if inbound is None:
                    self._node.get_logger().warn(f"Dropping unknown inbound topic_id={topic_id}")
                    continue
                self.handle_incoming_ros_message(inbound, payload, publishers)
        except (ConnectionError, OSError) as exc:
            if not self._stop_event.is_set():
                self._node.get_logger().warn(f"Bridge connection closed: {exc}")
        finally:
            self._close_socket()
            if on_disconnect is not None:
                on_disconnect()


def serialize_video_packet(msg: FFMPEGPacket) -> bytes:
    return serialize_message(msg)


def deserialize_video_packet(payload: bytes) -> FFMPEGPacket:
    return deserialize_message(payload, FFMPEGPacket)
