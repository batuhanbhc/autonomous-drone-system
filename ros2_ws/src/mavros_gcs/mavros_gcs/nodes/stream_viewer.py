#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import av
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket


class StreamViewer(Node):
    def __init__(self):
        super().__init__('stream_viewer')

        self.declare_parameter('drone_id', 0)
        self.declare_parameter('rotate', False)                      
        self._rotate = bool(self.get_parameter('rotate').value)      
        drone_id = int(self.get_parameter('drone_id').value)

        self.codec        = av.CodecContext.create('h264', 'r')
        self.lock         = threading.Lock()
        self.latest       = None
        self.got_keyframe = False

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        topic = f'/drone_{drone_id}/camera/stream/out'
        self.sub = self.create_subscription(
            FFMPEGPacket, topic, self.callback, qos)
        self.get_logger().info(f'Subscribed to {topic}')

        self._window = f'Drone {drone_id} stream'
        self._gui_thread = threading.Thread(
            target=self._gui_loop, daemon=True)
        self._gui_thread.start()

    def callback(self, msg: FFMPEGPacket):
        if not self.got_keyframe:
            if not (msg.flags & 1):
                return
            self.got_keyframe = True
        try:
            pkt = av.Packet(bytes(msg.data))
            pkt.pts = msg.pts
            pkt.dts = msg.pts
            for f in self.codec.decode(pkt):
                bgr = f.reformat(format='bgr24').to_ndarray()
                if self._rotate:
                    bgr = cv2.rotate(bgr, cv2.ROTATE_180)
                with self.lock:
                    self.latest = bgr
        except Exception as e:
            self.get_logger().warn(f'decode error: {e}')

    def _gui_loop(self):
        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        while rclpy.ok():
            with self.lock:
                frame = self.latest
            if frame is not None:
                cv2.imshow(self._window, frame)
            key = cv2.waitKey(30) & 0xFF
            if key == ord('q'):
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


if __name__ == '__main__':
    main()