"""MAVSDK telemetry + RViz, with no flight commands."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from violet_rviz.bag_recording import default_bag_directory, launch_recording


def launch_nodes(context):
    # Rewrite the shipped layout for the chosen namespace; overrides remain usable.
    import tempfile
    namespace = LaunchConfiguration('vehicle_ns').perform(context).strip('/')
    config = LaunchConfiguration('rviz_config').perform(context)
    with open(config, encoding='utf-8') as source:
        content = source.read().replace('/drone1/', f'/{namespace}/')
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
    with handle:
        handle.write(content)
    return [Node(package='rviz2', executable='rviz2', arguments=['-d', handle.name], output='screen')]


def generate_launch_description():
    default_config = os.path.join(get_package_share_directory('violet_rviz'), 'rviz', 'pf_los4.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_ns', default_value='drone1'),
        DeclareLaunchArgument('connection_url', default_value='udp://:14540'),
        DeclareLaunchArgument('telemetry_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('discovery_timeout_s', default_value='30.0'),
        DeclareLaunchArgument('system_id', default_value='0', description='0 selects first autopilot'),
        DeclareLaunchArgument('record_bag', default_value='false'),
        DeclareLaunchArgument('bag_directory', default_value=default_bag_directory(
            get_package_share_directory('violet_rviz'))),
        DeclareLaunchArgument('kill_channel', default_value='6'),
        DeclareLaunchArgument('kill_threshold', default_value='1700'),
        DeclareLaunchArgument('kill_active_high', default_value='true'),
        DeclareLaunchArgument('camera_enabled', default_value='true'),
        DeclareLaunchArgument('camera_url', default_value='tcp://192.168.1.38:8888'),
        DeclareLaunchArgument('camera_frame', default_value='camera_optical_frame'),
        DeclareLaunchArgument('history_length', default_value='5000'),
        DeclareLaunchArgument('marker_diameter', default_value='0.6'),
        DeclareLaunchArgument('rviz_config', default_value=default_config),
        OpaqueFunction(function=launch_recording),
        Node(package='violet_rviz', executable='mavsdk_telemetry_bridge',
             namespace=LaunchConfiguration('vehicle_ns'), output='screen', parameters=[{
                 'kill_channel': ParameterValue(LaunchConfiguration('kill_channel'), value_type=int),
                 'kill_threshold': ParameterValue(LaunchConfiguration('kill_threshold'), value_type=int),
                 'kill_active_high': ParameterValue(LaunchConfiguration('kill_active_high'), value_type=bool),
                 'connection_url': LaunchConfiguration('connection_url'),
                 'telemetry_rate_hz': ParameterValue(LaunchConfiguration('telemetry_rate_hz'), value_type=float),
                 'discovery_timeout_s': ParameterValue(LaunchConfiguration('discovery_timeout_s'), value_type=float),
                 'system_id': ParameterValue(LaunchConfiguration('system_id'), value_type=int),
             }]),
        Node(package='violet_rviz', executable='mavsdk_visualization',
             namespace=LaunchConfiguration('vehicle_ns'), output='screen', parameters=[{
                 'history_length': ParameterValue(LaunchConfiguration('history_length'), value_type=int),
                 'marker_diameter': ParameterValue(LaunchConfiguration('marker_diameter'), value_type=float),
             }]),
        Node(package='violet_rviz', executable='camera_stream',
             namespace=LaunchConfiguration('vehicle_ns'), output='screen',
             condition=IfCondition(LaunchConfiguration('camera_enabled')), parameters=[{
                 'camera_url': LaunchConfiguration('camera_url'),
                 'camera_frame': LaunchConfiguration('camera_frame'),
             }]),
        # Keep telemetry in NED while giving RViz a world frame with Up as +Z.
        # Rotation maps NED [N,E,D] to display ENU [E,N,-D].
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='ned_display_transform', output='screen', arguments=[
                 '--x', '0', '--y', '0', '--z', '0',
                 '--qx', '0.7071067811865476', '--qy', '0.7071067811865476',
                 '--qz', '0', '--qw', '0',
                 '--frame-id', 'rviz_world', '--child-frame-id', 'map_ned',
             ]),
        OpaqueFunction(function=launch_nodes),
    ])
