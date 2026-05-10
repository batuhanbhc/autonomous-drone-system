import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory


def create_session_dir():
    share_dir = get_package_share_directory("mavros_config")
    config_path = os.path.join(share_dir, "config", "control_params.yaml")

    with open(config_path, "r", encoding="utf-8") as f:
        root = yaml.safe_load(f)

    logs_path = root["flight_params"]["logs_path"]
    os.makedirs(logs_path, exist_ok=True)

    dir_count = sum(
        1 for entry in os.scandir(logs_path) if entry.is_dir()
    )
    session_dir = os.path.join(logs_path, f"{dir_count + 1:04d}")
    os.makedirs(session_dir, exist_ok=False)
    return session_dir


def launch_setup(context, *args, **kwargs):
    stream_codec = LaunchConfiguration("stream_codec")
    gcs_host = LaunchConfiguration("gcs_host")
    configured_session_dir = LaunchConfiguration("session_dir").perform(context).strip()
    session_dir = configured_session_dir or create_session_dir()

    container = ComposableNodeContainer(
        name="drone_pipeline_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="drone_pipeline",
                plugin="drone_pipeline::CameraCapture",
                name="camera_capture",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="drone_pipeline",
                plugin="drone_pipeline::VisionPipeline",
                name="vision_pipeline",
                parameters=[{"stream_codec": stream_codec}, {"session_dir": session_dir}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="drone_pipeline",
                plugin="drone_pipeline::VideoStreamer",
                name="video_streamer",
                parameters=[{"stream_codec": stream_codec}, {"gcs_host": gcs_host}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="drone_pipeline",
                plugin="drone_pipeline::AutonomousController",
                name="autonomous_controller",
                parameters=[{"session_dir": session_dir}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    flight_logger = Node(
        package="drone_pipeline",
        executable="flight_logger_node",
        name="flight_logger",
        parameters=[{"session_dir": session_dir}],
        output="screen",
        emulate_tty=True,
    )

    return [container, flight_logger]


def generate_launch_description():
    stream_codec_arg = DeclareLaunchArgument(
        "stream_codec",
        default_value="",
        description="Stream codec override: 'mjpeg' or 'h264'. Empty string uses mavros_config.",
    )
    gcs_host_arg = DeclareLaunchArgument(
        "gcs_host",
        default_value="",
        description="GCS host for direct video streaming. Empty string uses mavros_config if set.",
    )
    session_dir_arg = DeclareLaunchArgument(
        "session_dir",
        default_value="",
        description="Optional shared session directory for CSV/video outputs.",
    )

    return LaunchDescription([
        stream_codec_arg,
        gcs_host_arg,
        session_dir_arg,
        OpaqueFunction(function=launch_setup),
    ])
