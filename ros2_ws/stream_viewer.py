#!/usr/bin/env python3
import argparse
import threading
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2
import av
from ffmpeg_image_transport_msgs.msg import FFMPEGPacket

class StreamViewer(Node):
    def __init__(self, drone_id: int):
        super().__init__(f'stream_viewer_drone_{drone_id}')
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
                rgb = f.reformat(format='rgb24').to_ndarray()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                with self.lock:
                    self.latest = bgr
        except Exception as e:
            self.get_logger().warn(f'decode error: {e}')

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--drone-id', type=int, default=0)
    args = parser.parse_args()

    rclpy.init()
    node = StreamViewer(drone_id=args.drone_id)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    window = f'Drone {args.drone_id} stream'
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    while rclpy.ok():
        with node.lock:
            frame = node.latest
        if frame is not None:
            cv2.imshow(window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()