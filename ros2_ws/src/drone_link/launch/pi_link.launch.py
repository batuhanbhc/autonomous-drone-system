from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value="0",
        description="Drone ID used to namespace all topics.",
    )
    gcs_host_arg = DeclareLaunchArgument(
        "gcs_host",
        default_value="",
        description="GCS bridge host. Empty string uses the node default/parameter value.",
    )

    node = Node(
        package="drone_link",
        executable="pi_link_bridge",
        name="pi_link_bridge",
        output="screen",
        parameters=[
            {"drone_id": LaunchConfiguration("drone_id")},
            {"gcs_host": LaunchConfiguration("gcs_host")},
        ],
    )

    return LaunchDescription([
        drone_id_arg,
        gcs_host_arg,
        node,
    ])
