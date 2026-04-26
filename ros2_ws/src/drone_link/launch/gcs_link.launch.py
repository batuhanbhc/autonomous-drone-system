from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        "drone_id",
        default_value="0",
        description="Drone ID used to namespace all topics.",
    )
    listen_host_arg = DeclareLaunchArgument(
        "listen_host",
        default_value="0.0.0.0",
        description="Address for the GCS bridge TCP server.",
    )

    node = Node(
        package="drone_link",
        executable="gcs_link_bridge",
        name="gcs_link_bridge",
        output="screen",
        parameters=[
            {"drone_id": LaunchConfiguration("drone_id")},
            {"listen_host": LaunchConfiguration("listen_host")},
        ],
    )

    return LaunchDescription([
        SetEnvironmentVariable("ROS_LOCALHOST_ONLY", "1"),
        drone_id_arg,
        listen_host_arg,
        node,
    ])
