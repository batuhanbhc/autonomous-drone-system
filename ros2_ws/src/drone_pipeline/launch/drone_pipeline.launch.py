from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
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
                plugin="drone_pipeline::RecordVideo",
                name="record_video",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="drone_pipeline",
                plugin="drone_pipeline::VisionPipeline",
                name="vision_pipeline",
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        container,
        Node(
            package="drone_pipeline",
            executable="flight_logger_node",
            name="flight_logger",
            output="screen",
            emulate_tty=True,
        ),
    ])