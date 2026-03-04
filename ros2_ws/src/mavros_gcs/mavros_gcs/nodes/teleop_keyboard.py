#!/usr/bin/env python3
"""
ROS 2 teleop keyboard node.

- Reads linux input events from /dev/input/eventX via evdev
- Runs CommandManager.tick() at configured hz
- Publishes:
    * /teleop/command (Reliable)
    * /teleop/action  (BestEffort)
"""

import os
import sys
import time
import yaml
import threading
from typing import List, Optional
from evdev import InputDevice, ecodes

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from mavros_gcs.teleop_utils.definitions import NoEchoTerminal, KeyState, TELEOP_CONFIG
from mavros_gcs.teleop_utils.command_helpers import assign_priorities_from_list_order
from mavros_gcs.teleop_utils.command_manager import CommandManager
from mavros_gcs.teleop_utils.teleop_io import init_teleop_io
from mavros_gcs.teleop_utils.print_manual import teleop_manual_text
from mavros_gcs.teleop_utils.commands import (
    Command, KillConfirm, KillSwitch, Arm, Disarm,
    KeyboardToggle, ControlToggle, Land, RTL, Takeoff, Guided,
    SpeedDown, SpeedUp, VelocityYaw, PressSafetySwitch
)

from drone_msgs.msg import TeleopCommand, TeleopAction

from ament_index_python.packages import get_package_share_directory

# -------------------------
# Teleop key "manual" printer

def load_yaml_from_pkg(pkg: str, rel: str):
    """
    Loads a YAML file from a ROS2 package share directory and returns the FULL root dict.
    """
    yaml_path = os.path.join(get_package_share_directory(pkg), rel)
    with open(yaml_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    if not isinstance(data, dict):
        raise RuntimeError(f"YAML root must be a dict: {yaml_path}")

    return data



class TeleopKeyboardNode(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        
        self.declare_parameter("drone_id", 0)
        drone_id = int(self.get_parameter("drone_id").value)

        # ... after reading YAML topic_command/topic_action ...
        base = f"/drone_{drone_id}"

        # -------------------------
        # Load config YAML
        self.declare_parameter("config_pkg", "mavros_config")
        self.declare_parameter("config_rel", "config/control_params.yaml")

        config_pkg = self.get_parameter("config_pkg").value
        config_rel = self.get_parameter("config_rel").value

        root_cfg = load_yaml_from_pkg(config_pkg, config_rel)

        teleop_cfg = root_cfg.get("teleop", {})
        topics_cfg = root_cfg.get("custom_topics", {})
        
        if not isinstance(teleop_cfg, dict):
            raise RuntimeError("YAML key 'teleop' must be a dict")
        if not isinstance(topics_cfg, dict):
            raise RuntimeError("YAML key 'custom_topics' must be a dict")
        
        # -------------------------
        # hz from YAML
        self._hz = float(teleop_cfg.get("hz", 8.0))

        # -------------------------
        # keyboard devices from YAML (try first then second)
        devices = teleop_cfg.get("keyboard_devices", [])
        if not isinstance(devices, list):
            devices = []
        self._keyboard_devices = [str(x) for x in devices if str(x)]
        if len(self._keyboard_devices) < 2:
            raise RuntimeError("YAML must provide teleop.keyboard_devices: [path1, path2]")
        
        # -------------------------
        # topic paths from YAML
        topic_command = str(topics_cfg.get("manual_command", "/teleop/command"))
        topic_action = str(topics_cfg.get("manual_action", "/teleop/action"))

        topic_command = base + topic_command
        topic_action = base + topic_action

        # -------------------------
        # teleop command parameters from YAML
        vel_categories = teleop_cfg["velocity"]["categories"]
        vel_start_index = int(teleop_cfg["velocity"]["categories"].get("default_start_index", 0))
        yaw_rate = float(teleop_cfg["velocity"].get("yaw_rate", 1.0))
        cmd_params = teleop_cfg["commands"]["params"]
        self.cmd_params = cmd_params

        # -------------------------
        # QoS profiles 
        self._qos_command = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._qos_action = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self._pub_command = self.create_publisher(TeleopCommand, topic_command, self._qos_command)
        self._pub_action = self.create_publisher(TeleopAction, topic_action, self._qos_action)

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
                hook_fn=None,
                latch=cmd_params["KILL_CONFIRM"]["latch"],
                activation_time_s=cmd_params["KILL_CONFIRM"]["activation_time_s"],
            ),
            KillSwitch(
                config=TELEOP_CONFIG["KILL_SWITCH"],
                hook_fn=None,
                latch=cmd_params["KILL_SWITCH"]["latch"],
                activation_time_s=cmd_params["KILL_SWITCH"]["activation_time_s"],
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
            KeyboardToggle(
                config=TELEOP_CONFIG["KEYBOARD_TOGGLE"],
                hook_fn=lambda: manager.toggle_keyboard(),
                latch=cmd_params["KEYBOARD_TOGGLE"]["latch"],
                activation_time_s=cmd_params["KEYBOARD_TOGGLE"]["activation_time_s"],
            ),
            Land(
                config=TELEOP_CONFIG["LAND"],
                hook_fn=None,
                latch=cmd_params["LAND"]["latch"],
                activation_time_s=cmd_params["LAND"]["activation_time_s"],
            ),
            RTL(
                config=TELEOP_CONFIG["RTL"],
                hook_fn=None,
                latch=cmd_params["RTL"]["latch"],
                activation_time_s=cmd_params["RTL"]["activation_time_s"],
            ),
            Guided(
                config=TELEOP_CONFIG["GUIDED"],
                hook_fn=None,
                latch=cmd_params["GUIDED"]["latch"],
                activation_time_s=cmd_params["GUIDED"]["activation_time_s"],
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
            PressSafetySwitch(
                config=TELEOP_CONFIG["PRESS_SAFETY_SWITCH"],
                hook_fn=None,
                latch=cmd_params["PRESS_SAFETY_SWITCH"]["latch"],
                activation_time_s=cmd_params["PRESS_SAFETY_SWITCH"]["activation_time_s"],
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
            f"TeleopKeyboardNode started: devices={self._keyboard_devices}, hz={self._hz}"
        )

        # Print manual once at startup (to stdout).
        time.sleep(0.2)
        print(teleop_manual_text(), flush=True)

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
        dev = None
        last_err = None

        for path in self._keyboard_devices[:2]:
            try:
                dev = InputDevice(path)
                self._dev = dev
                self.get_logger().info(f"Keyboard opened: {path}")
                break
            except Exception as e:
                last_err = e
                self.get_logger().warning(f"Failed to open input device '{path}': {e}")

        if dev is None:
            self.get_logger().fatal(
                f"Failed to open any keyboard device (tried {self._keyboard_devices[:2]}): {last_err}"
            )
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
                    if event.value == 1:
                        self._key_state.add_key(key_name)
                    elif event.value == 0:
                        self._key_state.remove_key(key_name)
            except Exception as e:
                if not self._stop.is_set():
                    self.get_logger().error(f"Keyboard loop error: {e}")
            finally:
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
        try:
            msg.command_id = getattr(TeleopCommand, command_name)
        except AttributeError:
            raise ValueError(f"Unknown command name: {command_name}")

        msg.float_1 = float(float_1)
        msg.float_2 = float(float_2)
        msg.bool_1 = bool_1
        self._pub_command.publish(msg)

    def _publish_action_msg(self, command_name: str, vx: float, vy: float, vz: float, yaw_rate: float) -> None:
        msg = TeleopAction()
        msg.stamp = self.get_clock().now().to_msg()
        msg.source = self.get_name()
        msg.vx = float(vx)
        msg.vy = float(vy)
        msg.vz = float(vz)
        msg.yaw_rate = float(yaw_rate)

        # Map name->id
        try:
            msg.command_id = getattr(TeleopCommand, command_name)
        except AttributeError:
            raise ValueError(f"Unknown command name: {command_name}")
        
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