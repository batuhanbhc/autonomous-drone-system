from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    drone_id_arg = DeclareLaunchArgument(
        'drone_id', default_value='0',
        description='Drone ID used to namespace all topics'
    )
    gcs_id_arg = DeclareLaunchArgument(
        'gcs_id', default_value='0',
        description='GCS ID embedded in heartbeat messages'
    )

    drone_id = LaunchConfiguration('drone_id')
    gcs_id   = LaunchConfiguration('gcs_id')

    teleop = Node(
        package='mavros_gcs',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        parameters=[{'drone_id': drone_id}],
        output='screen',
    )

    info_panel = Node(
        package='mavros_gcs',
        executable='info_panel',
        name='info_panel',
        parameters=[{'drone_id': drone_id}],
        output='own_log',
        prefix='gnome-terminal --geometry=220x50 -- ',
        emulate_tty=True,
    )

    stream_viewer = Node(
        package='mavros_gcs',
        executable='stream_viewer',
        name='stream_viewer',
        parameters=[{'drone_id': drone_id}],
        output='screen',
        emulate_tty=True,
    )

    heartbeat = Node(
        package='mavros_gcs',
        executable='gcs_heartbeat',
        name='gcs_heartbeat_node',
        parameters=[{'drone_id': drone_id, 'gcs_id': gcs_id}],
        output='screen',
    )

    return LaunchDescription([
        drone_id_arg,
        gcs_id_arg,
        teleop,
        info_panel,
        stream_viewer,
        heartbeat,
    ])