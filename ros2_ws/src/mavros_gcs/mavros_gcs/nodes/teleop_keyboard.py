#!/usr/bin/env python3
"""
ROS 2 teleop keyboard node.

- Reads linux input events from /dev/input/eventX via evdev
- Runs CommandManager.tick() at configured hz
- Publishes:
    * /teleop/command (Reliable)
    * /teleop/action  (BestEffort)

CLI examples:
  ros2 run <your_pkg> teleop_keyboard --ros-args -p keyboard_device:=/dev/input/event3 -p teleop_yaml:=/path/to/teleop.yaml
"""

import sys
import threading
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from evdev import InputDevice, ecodes

from mavros_gcs.teleop_utils.definitions import NoEchoTerminal, KeyState, TELEOP_CONFIG
from mavros_gcs.teleop_utils.commands import (
    Command,
    KillConfirm, KillSwitch, Arm, Disarm,
    ConsoleToggle, ControlToggle, ModeLand, ModeLoiter, ModeRTL, Takeoff,
    SpeedDown, SpeedUp, VelocityYaw,
)
from mavros_gcs.teleop_utils.command_helpers import assign_priorities_from_list_order
from mavros_gcs.teleop_utils.params import load_teleop_yaml_from_pkg
from mavros_gcs.teleop_utils.command_manager import CommandManager

from mavros_gcs.teleop_utils.teleop_io import init_teleop_io
from teleop_msgs.msg import TeleopCommand, TeleopAction


class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")

        self.declare_parameter("keyboard_device", "")
        self._keyboard_device = self.get_parameter("keyboard_device").value

        if not self._keyboard_device:
            raise RuntimeError("Set params: keyboard_device")

        # -------------------------
        # QoS profiles 
        self._qos_command = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._qos_action = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub_command = self.create_publisher(TeleopCommand, "/teleop/command", self._qos_command)
        self._pub_action = self.create_publisher(TeleopAction, "/teleop/action", self._qos_action)

        # -------------------------
        # Load teleop yaml
        self.declare_parameter("teleop_pkg", "mavros_config")
        self.declare_parameter("teleop_rel", "config/teleop_params.yaml")

        teleop_pkg = self.get_parameter("teleop_pkg").value
        teleop_rel = self.get_parameter("teleop_rel").value

        teleop_cfg = load_teleop_yaml_from_pkg(teleop_pkg, teleop_rel)
        self.teleop_cfg = teleop_cfg

        self._hz = float(teleop_cfg.get("hz", 8.0))
        vel_categories = teleop_cfg["velocity"]["categories"]
        vel_start_index = int(teleop_cfg["velocity"]["categories"].get("default_start_index", 0))
        yaw_rate = float(teleop_cfg["velocity"].get("yaw_rate", 1.0))
        cmd_params = teleop_cfg["commands"]["params"]
        self.cmd_params = cmd_params

        # -------------------------
        # Key handling
        self._key_state = KeyState()
        self._stop = threading.Event()
        
        # -------------------------
        # Initialize ROS-agnostic IO bridge (publishing + logging)
        init_teleop_io(
            logger=self.get_logger(),
            publish_command_fn=self._publish_command_msg,
            publish_action_fn=self._publish_action_msg,
            now_fn=self.get_clock().now,  # returns rclpy Time
            source_name=self.get_name(),
        )

        # -------------------------
        # Build commands list
        # NOTE: manager is referenced in lambdas, so we define manager after commands list,
        manager: Optional[CommandManager] = None

        commands: List[Command] = [
            KillConfirm(
                config=TELEOP_CONFIG["KILL_CONFIRM"],
                latch=cmd_params["KILL_CONFIRM"]["latch"],
                activation_time_s=cmd_params["KILL_CONFIRM"]["activation_time_s"],
                is_window_open_fn=lambda: manager.kill_window_open(),
                clear_window_fn=lambda: manager.clear_kill_window(),
            ),
            KillSwitch(
                config=TELEOP_CONFIG["KILL_SWITCH"],
                hook_fn=lambda seconds=2.0: manager.arm_kill_window(seconds),
                latch=cmd_params["KILL_SWITCH"]["latch"],
                activation_time_s=cmd_params["KILL_SWITCH"]["activation_time_s"],
                kill_window_s=cmd_params["KILL_SWITCH"]["kill_window_s"]
            ),
            Arm(
                config=TELEOP_CONFIG["ARM"],
                hook_fn=None,
                latch=cmd_params["ARM"]["latch"],
                activation_time_s=cmd_params["ARM"]["activation_time_s"],
            ),
            Disarm(
                config=TELEOP_CONFIG["DISARM"],
                hook_fn=None,
                latch=cmd_params["DISARM"]["latch"],
                activation_time_s=cmd_params["DISARM"]["activation_time_s"],
            ),
            ControlToggle(
                config=TELEOP_CONFIG["CONTROL_TOGGLE"],
                hook_fn=None,
                latch=cmd_params["CONTROL_TOGGLE"]["latch"],
                activation_time_s=cmd_params["CONTROL_TOGGLE"]["activation_time_s"],
            ),
            ConsoleToggle(
                config=TELEOP_CONFIG["CONSOLE_TOGGLE"],
                hook_fn=lambda: manager.toggle_console(),
                latch=cmd_params["CONSOLE_TOGGLE"]["latch"],
                activation_time_s=cmd_params["CONSOLE_TOGGLE"]["activation_time_s"],
            ),
            ModeLand(
                config=TELEOP_CONFIG["LAND"],
                hook_fn=None,
                latch=cmd_params["LAND"]["latch"],
                activation_time_s=cmd_params["LAND"]["activation_time_s"],
            ),
            ModeRTL(
                config=TELEOP_CONFIG["RTL"],
                hook_fn=None,
                latch=cmd_params["RTL"]["latch"],
                activation_time_s=cmd_params["RTL"]["activation_time_s"],
            ),
            ModeLoiter(
                config=TELEOP_CONFIG["LOITER"],
                hook_fn=None,
                latch=cmd_params["LOITER"]["latch"],
                activation_time_s=cmd_params["LOITER"]["activation_time_s"],
            ),
            Takeoff(
                config=TELEOP_CONFIG["TAKEOFF"],
                hook_fn=None,
                latch=cmd_params["TAKEOFF"]["latch"],
                activation_time_s=cmd_params["TAKEOFF"]["activation_time_s"],
            ),
            SpeedUp(
                config=TELEOP_CONFIG["SPEED_UP"],
                hook_fn=lambda: manager.increment_velocity(),
                latch=cmd_params["SPEED_UP"]["latch"],
                activation_time_s=cmd_params["SPEED_UP"]["activation_time_s"],
            ),
            SpeedDown(
                config=TELEOP_CONFIG["SPEED_DOWN"],
                hook_fn=lambda: manager.decrement_velocity(),
                latch=cmd_params["SPEED_DOWN"]["latch"],
                activation_time_s=cmd_params["SPEED_DOWN"]["activation_time_s"],
            ),
            VelocityYaw(
                config=TELEOP_CONFIG["ACTION"],
                hook_fn=lambda: manager.get_velocity(),
                latch=cmd_params["VEL_YAW"]["latch"],
                activation_time_s=cmd_params["VEL_YAW"]["activation_time_s"],
                yaw_rate=yaw_rate,
            ),
        ]

        assign_priorities_from_list_order(commands)

        manager = CommandManager(
            commands=commands,
            hz=self._hz,
            vel_categories=vel_categories,
            vel_start_index=vel_start_index,
        )
        self._manager = manager

        # -------------------------
        # Create ROS timer for tick loop
        period_s = 1.0 / max(self._hz, 1e-6)
        self._timer = self.create_timer(period_s, self._on_tick)

        # -------------------------
        # Start keyboard reader thread
        self._dev = None
        self._kbd_thread = threading.Thread(target=self._keyboard_read_loop, name="teleop_keyboard_read", daemon=False)
        self._kbd_thread.start()

        self.get_logger().info(
            f"TeleopKeyboardNode started: device={self._keyboard_device}, hz={self._hz}"
        )

    # -------------------------
    # ROS timer tick -> manager.tick()
    def _on_tick(self) -> None:
        try:
            state_tuple = self._key_state.snapshot()  # (state, last_event_t)
            self._manager.tick(state_tuple)
        except Exception as e:
            self.get_logger().error(f"Tick error: {e}")

    # -------------------------
    # Keyboard event loop
    def _keyboard_read_loop(self) -> None:
        try:
            dev = InputDevice(self._keyboard_device)
            self._dev = dev
        except Exception as e:
            self.get_logger().fatal(f"Failed to open input device '{self._keyboard_device}': {e}")
            rclpy.shutdown()
            return

        with NoEchoTerminal(sys.stdin.fileno()):
            try:
                for event in dev.read_loop():
                    if self._stop.is_set():
                        break
                    if event.type != ecodes.EV_KEY:
                        continue

                    key_name = ecodes.KEY.get(event.code, f"UNKNOWN_{event.code}")
                    # event.value: 1=press, 0=release, 2=autorepeat
                    if event.value == 1:
                        self._key_state.add_key(key_name)
                    elif event.value == 0:
                        self._key_state.remove_key(key_name)
                    else:
                        # ignore autorepeat at evdev level
                        pass
            except Exception as e:
                if not self._stop.is_set():
                    self.get_logger().error(f"Keyboard loop error: {e}")
            finally:
                # ensure device closes; if ECHO was disabled, leaving the 'with' normally restores it
                try:
                    dev.close()
                except Exception:
                    pass
                
    # -------------------------
    # Publish helpers used by ROS-agnostic layer
    def _publish_command_msg(self, command_name: str, float_1: float = 0.0, float_2: float = 0.0, bool_1: bool = True) -> None:
        msg = TeleopCommand()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self.get_name()

        # Map name->id
        msg.command_id = int(self.cmd_params[command_name]["command_id"])

        msg.float_1 = float(float_1)
        msg.float_2 = float(float_2)
        msg.bool_1 = bool_1
        self._pub_command.publish(msg)

    def _publish_action_msg(self, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        msg = TeleopAction()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self.get_name()
        msg.float_1 = float(vx)
        msg.float_2 = float(vy)
        msg.float_3 = float(vz)
        msg.float_4 = float(yaw_rate)
        self._pub_action.publish(msg)

    # -------------------------
    def destroy_node(self) -> bool:
        self._stop.set()

        # close device to unblock dev.read_loop()
        if self._dev is not None:
            try:
                self._dev.close()
            except Exception:
                pass

        # Wait for thread so NoEchoTerminal.__exit__ runs
        if hasattr(self, "_kbd_thread") and self._kbd_thread.is_alive():
            self._kbd_thread.join(timeout=2.0)

        return super().destroy_node()




def main(argv=None):
    rclpy.init(args=argv)
    node = TeleopKeyboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

if __name__ == "__main__":
    main()