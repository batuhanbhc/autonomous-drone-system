#!/usr/bin/env python3
"""
tune.py — quick CLI for mavros_gate on-air tuning

Usage:
  python3 tune.py <drone_id> height <meters>
  python3 tune.py <drone_id> pid [--kp P] [--ki I] [--kd D]

Examples:
  python3 tune.py 0 height 2.5
  python3 tune.py 0 pid --ki 0.1
  python3 tune.py 0 pid --kp 1.5 --kd 0.4
  python3 tune.py 1 pid --kp 1.2 --ki 0.05 --kd 0.3
"""

import argparse
import sys
import rclpy
from rclpy.node import Node
from drone_msgs.srv import SetTargetHeight, SetPidGains


class TuneClient(Node):
    def __init__(self):
        super().__init__("tune_client")

    def set_height(self, drone_id: int, height: float):
        cli = self.create_client(
            SetTargetHeight,
            f"/drone_{drone_id}/control_gate/set_target_height"
        )
        if not cli.wait_for_service(timeout_sec=3.0):
            print(f"[ERROR] set_target_height service not available for drone_{drone_id}")
            return
        req = SetTargetHeight.Request()
        req.target_agl_m = float(height)
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.done():
            r = fut.result()
            print(f"[{'OK' if r.success else 'FAIL'}] {r.message}")
        else:
            print("[ERROR] Service call timed out.")

    def set_pid(self, drone_id: int, kp, ki, kd):
        # Fetch current gains first if any are omitted
        if None in (kp, ki, kd):
            print("[INFO] Some gains omitted — use -1.0 sentinel for unchanged fields.")

        cli = self.create_client(
            SetPidGains,
            f"/drone_{drone_id}/altitude_controller/set_pid_gains"
        )
        if not cli.wait_for_service(timeout_sec=3.0):
            print(f"[ERROR] set_pid_gains service not available for drone_{drone_id}")
            return
        req = SetPidGains.Request()
        req.kp = float(kp) if kp is not None else -1.0
        req.ki = float(ki) if ki is not None else -1.0
        req.kd = float(kd) if kd is not None else -1.0
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        if fut.done():
            r = fut.result()
            print(f"[{'OK' if r.success else 'FAIL'}] {r.message}")
        else:
            print("[ERROR] Service call timed out.")


def main():
    parser = argparse.ArgumentParser(
        description="On-air tuning CLI for mavros_gate",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument("drone_id", type=int, help="Drone ID (e.g. 0)")

    sub = parser.add_subparsers(dest="cmd", required=True)

    # --- height sub-command ---
    h = sub.add_parser("height", help="Set altitude hold target")
    h.add_argument("meters", type=float, help="Target height in metres (AGL)")

    # --- pid sub-command ---
    p = sub.add_parser("pid", help="Tune PID gains (omit any to leave unchanged)")
    p.add_argument("--kp", type=float, default=None, metavar="P")
    p.add_argument("--ki", type=float, default=None, metavar="I")
    p.add_argument("--kd", type=float, default=None, metavar="D")

    args = parser.parse_args()

    if args.cmd == "pid" and args.kp is None and args.ki is None and args.kd is None:
        parser.error("pid: supply at least one of --kp, --ki, --kd")

    rclpy.init()
    node = TuneClient()
    try:
        if args.cmd == "height":
            node.set_height(args.drone_id, args.meters)
        elif args.cmd == "pid":
            node.set_pid(args.drone_id, args.kp, args.ki, args.kd)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()