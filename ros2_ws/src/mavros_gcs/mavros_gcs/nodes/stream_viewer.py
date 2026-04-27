#!/usr/bin/env python3
import os
import shutil
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import av
import cv2
import numpy as np
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
        self.declare_parameter("viewer_backend", "auto")

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
        requested_backend = str(self.get_parameter("viewer_backend").value).strip().lower() or "auto"
        self._backend = self._select_backend(requested_backend)

        self.lock = threading.Lock()
        self.latest = None
        self._first_frame_logged = False
        self._receiver_thread = None

        self._window = f"Drone {drone_id} stream"

        self.get_logger().info(
            f"Stream viewer listening via {self._transport} on {self._listen_host}:{self._video_port} "
            f"(configured codec={self._codec_name}, backend={self._backend})"
        )

        if self._backend == "opencv":
            self._receiver_thread = threading.Thread(target=self._receiver_loop, daemon=True)
            self._receiver_thread.start()

    def _select_backend(self, requested_backend: str) -> str:
        if requested_backend == "ffplay":
            return "ffplay"
        if requested_backend == "opencv":
            return "opencv"
        return "ffplay" if shutil.which("ffplay") else "opencv"

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
                        if not self._first_frame_logged:
                            self._first_frame_logged = True
                            self.get_logger().info(
                                f"First video frame received ({bgr.shape[1]}x{bgr.shape[0]})"
                            )
            except Exception as exc:  # noqa: BLE001
                if not rclpy.ok():
                    return
                self.get_logger().warn(f"stream receive error: {exc}")
                time.sleep(1.0)

    def _placeholder_frame(self) -> np.ndarray:
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(
            frame,
            "Waiting for video stream...",
            (70, 220),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            frame,
            f"{self._transport.upper()} {self._listen_host}:{self._video_port}",
            (100, 270),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (180, 180, 180),
            2,
            cv2.LINE_AA,
        )
        return frame

    def gui_loop(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self._window, 960, 720)
        cv2.moveWindow(self._window, 80, 80)
        try:
            cv2.setWindowProperty(self._window, cv2.WND_PROP_TOPMOST, 1)
        except cv2.error:
            pass
        while rclpy.ok():
            with self.lock:
                frame = self.latest
            cv2.imshow(self._window, frame if frame is not None else self._placeholder_frame())
            key = cv2.waitKey(30) & 0xFF
            if key == ord("q"):
                rclpy.shutdown()
                break
        cv2.destroyAllWindows()

    def ffplay_loop(self) -> None:
        cmd = [
            "ffplay",
            "-fflags",
            "nobuffer",
            "-flags",
            "low_delay",
            "-framedrop",
            "-probesize",
            "32",
            "-analyzeduration",
            "0",
            "-window_title",
            self._window,
            "-x",
            "960",
            "-y",
            "720",
            "-f",
            "nut",
        ]
        if self._rotate:
            cmd.extend(["-vf", "hflip,vflip"])
        cmd.append(self._stream_url())

        self.get_logger().info(f"Launching ffplay viewer on {self._stream_url()}")
        proc = subprocess.Popen(cmd)
        try:
            while rclpy.ok() and proc.poll() is None:
                time.sleep(0.1)
        finally:
            if proc.poll() is None:
                proc.terminate()
                try:
                    proc.wait(timeout=1.0)
                except subprocess.TimeoutExpired:
                    proc.kill()


def main(argv=None):
    rclpy.init(args=argv)
    node = StreamViewer()
    try:
        if node._backend == "ffplay":
            node.ffplay_loop()
        else:
            spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
            spin_thread.start()
            try:
                node.gui_loop()
            finally:
                spin_thread.join(timeout=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.try_shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
