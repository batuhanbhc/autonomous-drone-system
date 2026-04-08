"""
mavros_gate_compositor.launch.py

Launches the single-process compositor that runs control_gate, mcu_bridge,
and altitude_controller in one OS process under a MultiThreadedExecutor with
intra-process communication enabled.

This is the preferred production launch file.  Use the legacy
mavros_gate.launch.py only if you need to run the nodes in separate processes
(e.g. for isolated crash boundaries or separate logging).

Arguments
─────────
  drone_id    (default 0)          – namespace suffix: /drone_<id>/...
  serial_port (default /dev/ttyACM0) – MCU serial port for vertical estimates
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value="0",
        description="Drone ID used to namespace all topics (e.g. /drone_0/...)",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/ttyACM0",
        description="Serial port for the MCU bridge (vertical estimate packets)",
    )

    compositor_node = Node(
        package="mavros_gate",
        executable="mavros_gate_compositor",
        name="mavros_gate_compositor",
        output="screen",
        parameters=[
            {"drone_id":    LaunchConfiguration("drone_id")},
            {"serial_port": LaunchConfiguration("serial_port")},
        ],
    )

    return LaunchDescription([
        drone_id_arg,
        serial_port_arg,
        compositor_node,
    ])