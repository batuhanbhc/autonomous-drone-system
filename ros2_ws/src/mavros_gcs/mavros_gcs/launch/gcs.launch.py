from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
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
    rotate_arg = DeclareLaunchArgument(
        'rotate', default_value='false',
        description='Rotate stream viewer 180 degrees'
    )
    stream_codec_arg = DeclareLaunchArgument(
        'stream_codec', default_value='',
        description="Stream codec override for the viewer ('mjpeg' or 'h264'). Empty uses mavros_config."
    )
    stream_transport_arg = DeclareLaunchArgument(
        'stream_transport', default_value='',
        description="Stream transport override for the viewer ('udp' or 'tcp'). Empty uses mavros_config."
    )
    listen_host_arg = DeclareLaunchArgument(
        'listen_host', default_value='0.0.0.0',
        description='Listen address for the local GCS bridge server'
    )
    use_odom_arg = DeclareLaunchArgument(          # NEW
        'use_odom', default_value='true',
        description='Enable odometry subscription in info_panel'
    )

    drone_id = LaunchConfiguration('drone_id')
    gcs_id   = LaunchConfiguration('gcs_id')
    rotate   = LaunchConfiguration('rotate')
    stream_codec = LaunchConfiguration('stream_codec')
    stream_transport = LaunchConfiguration('stream_transport')
    listen_host = LaunchConfiguration('listen_host')
    use_odom = LaunchConfiguration('use_odom')     # NEW

    gcs_bridge = Node(
        package='drone_link',
        executable='gcs_link_bridge',
        name='gcs_link_bridge',
        parameters=[{'drone_id': drone_id, 'listen_host': listen_host}],
        output='screen',
    )

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
        parameters=[{'drone_id': drone_id, 'use_odom': use_odom}],  # CHANGED
        output='own_log',
        prefix='gnome-terminal --geometry=220x50 -- ',
        emulate_tty=True,
    )
    stream_viewer = Node(
        package='mavros_gcs',
        executable='stream_viewer',
        name='stream_viewer',
        parameters=[{
            'drone_id': drone_id,
            'rotate': rotate,
            'stream_codec': stream_codec,
            'stream_transport': stream_transport,
            'listen_host': listen_host,
        }],
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
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),
        drone_id_arg,
        gcs_id_arg,
        rotate_arg,
        stream_codec_arg,
        stream_transport_arg,
        listen_host_arg,
        use_odom_arg,                                               # NEW
        gcs_bridge,
        teleop,
        info_panel,
        stream_viewer,
        heartbeat,
    ])
