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

    control_gate_node = Node(
        package="mavros_gate",
        executable="control_gate",
        name="control_gate",
        output="screen",
        parameters=[
            {"drone_id": LaunchConfiguration("drone_id")}
        ],
    )

    """mcu_bridge_node = Node(
        package="mavros_gate",
        executable="mcu_bridge",
        name="mcu_bridge",
        output="screen",
        parameters=[
            {"drone_id": LaunchConfiguration("drone_id")},
            {"serial_port": LaunchConfiguration("serial_port")},
        ],
    )"""

    altitude_controller_node = Node(
        package="mavros_gate",
        executable="altitude_controller",
        name="altitude_controller",
        output="screen",
        parameters=[
            {"drone_id": LaunchConfiguration("drone_id")}
        ],
    )

    return LaunchDescription([
        drone_id_arg,
        #serial_port_arg,
        control_gate_node,
        #mcu_bridge_node,
        altitude_controller_node,
    ])