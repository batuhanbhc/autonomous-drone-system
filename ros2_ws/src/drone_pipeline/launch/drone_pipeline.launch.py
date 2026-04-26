from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

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
    stream_codec = LaunchConfiguration("stream_codec")
    gcs_host = LaunchConfiguration("gcs_host")

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
                parameters=[{"stream_codec": stream_codec}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="drone_pipeline",
                plugin="drone_pipeline::VideoStreamer",
                name="video_streamer",
                parameters=[{"stream_codec": stream_codec}, {"gcs_host": gcs_host}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    # FlightLogger is disabled — odom/gps per-frame data is now written
    # directly by the MJPEG writer thread inside VisionPipeline.
    # Re-enable by uncommenting below when needed.
    #
    # flight_logger = Node(
    #     package="drone_pipeline",
    #     executable="flight_logger_node",
    #     name="flight_logger",
    #     output="screen",
    #     emulate_tty=True,
    # )

    return LaunchDescription([stream_codec_arg, gcs_host_arg, container])
