#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from teleop_msgs.msg import TeleopAction, TeleopCommand


class CommandGate(Node):
    def __init__(self) -> None:
        super().__init__('command_gate')

        # QoS for /teleop/action: Best Effort
        action_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        # QoS for /teleop/command: Reliable
        command_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._action_sub = self.create_subscription(
            TeleopAction,
            '/teleop/action',
            self._on_teleop_action,
            action_qos,
        )

        self._command_sub = self.create_subscription(
            TeleopCommand,
            '/teleop/command',
            self._on_teleop_command,
            command_qos,
        )

        self.get_logger().info('command_gate node started. Subscribed to /teleop/action and /teleop/command.')

    def _on_teleop_action(self, msg: TeleopAction) -> None:
        # Dummy callback: just print/log what is received
        self.get_logger().info(
            f"[TeleopAction] stamp={msg.stamp.sec}.{msg.stamp.nanosec:09d} "
            f"source='{msg.source}' "
            f"vx={msg.float_1:.3f} vy={msg.float_2:.3f} vz={msg.float_3:.3f} yaw_rate={msg.float_4:.3f}"
        )

    def _on_teleop_command(self, msg: TeleopCommand) -> None:
        # Dummy callback: just print/log what is received
        self.get_logger().info(
            f"[TeleopCommand] stamp={msg.stamp.sec}.{msg.stamp.nanosec:09d} "
            f"source='{msg.source}' "
            f"command_id={msg.command_id} "
            f"float_1={msg.float_1:.3f} float_2={msg.float_2:.3f} bool_1={msg.bool_1}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()