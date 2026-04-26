#!/usr/bin/env python3
import os
import threading
import time

from ament_index_python.packages import get_package_share_directory
import av
import cv2
import rclpy
from rclpy.node import Node
import yaml


def _load_yaml(pkg: str, rel: str) -> dict:
    yaml_path = os.path.join(get_package_share_directory(pkg), rel)
    with open(yaml_path, "r", encoding="utf-8") as handle:
        return yaml.safe_load(handle) or {}


class StreamViewer(Node):
    def __init__(self):
        super().__init__("stream_viewer")

        self.declare_parameter("drone_id", 0)
        self.declare_parameter("rotate", False)
        self.declare_parameter("config_pkg", "mavros_config")
        self.declare_parameter("config_rel", "config/control_params.yaml")
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("stream_codec", "")
        self.declare_parameter("stream_transport", "")

        self._rotate = bool(self.get_parameter("rotate").value)
        drone_id = int(self.get_parameter("drone_id").value)
        config_pkg = str(self.get_parameter("config_pkg").value)
        config_rel = str(self.get_parameter("config_rel").value)
        cfg = _load_yaml(config_pkg, config_rel)

        self._codec_name = (
            str(self.get_parameter("stream_codec").value).strip().lower()
            or str(cfg.get("streaming", {}).get("codec", "mjpeg")).strip().lower()
            or "mjpeg"
        )
        self._transport = (
            str(self.get_parameter("stream_transport").value).strip().lower()
            or str(cfg.get("link", {}).get("video", {}).get("transport", "udp")).strip().lower()
            or "udp"
        )
        self._listen_host = str(self.get_parameter("listen_host").value).strip() or "0.0.0.0"
        self._video_port = int(cfg.get("link", {}).get("video", {}).get("port", 50020))

        self.lock = threading.Lock()
        self.latest = None
        self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)

        self._window = f"Drone {drone_id} stream"
        self._gui_thread = threading.Thread(target=self._gui_loop, daemon=True)

        self.get_logger().info(
            f"Stream viewer listening via {self._transport} on {self._listen_host}:{self._video_port} "
            f"(configured codec={self._codec_name})"
        )

        self._receiver_thread.start()
        self._gui_thread.start()

    def _stream_url(self) -> str:
        if self._transport == "tcp":
            return f"tcp://{self._listen_host}:{self._video_port}?listen=1&tcp_nodelay=1"
        return (
            f"udp://{self._listen_host}:{self._video_port}"
            "?fifo_size=1000000&overrun_nonfatal=1"
        )

    def _receiver_loop(self) -> None:
        while rclpy.ok():
            try:
                url = self._stream_url()
                with av.open(url, mode="r", format="nut") as container:
                    self.get_logger().info(f"Video stream connected on {url}")
                    for frame in container.decode(video=0):
                        if not rclpy.ok():
                            return
                        bgr = frame.to_ndarray(format="bgr24")
                        if self._rotate:
                            bgr = cv2.rotate(bgr, cv2.ROTATE_180)
                        with self.lock:
                            self.latest = bgr
            except Exception as exc:  # noqa: BLE001
                if not rclpy.ok():
                    return
                self.get_logger().warn(f"stream receive error: {exc}")
                time.sleep(1.0)

    def _gui_loop(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                frame = self.latest
            if frame is not None:
                cv2.imshow(self._window, frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
                break
        cv2.destroyAllWindows()


def main(argv=None):
    rclpy.init(args=argv)
    node = StreamViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
