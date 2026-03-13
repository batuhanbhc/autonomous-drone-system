from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="drone_pipeline",
            executable="camera_capture_node",
            name="camera_capture",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="drone_pipeline",
            executable="flight_logger_node",
            name="flight_logger",
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="drone_pipeline",
            executable="save_video_node",
            name="save_video",
            output="screen",
            emulate_tty=True,
        ),
    ])