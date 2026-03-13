from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="drone_camera",
            executable="camera_capture_node",
            name="camera_capture",
            output="screen",
            emulate_tty=True,
        )
    ])