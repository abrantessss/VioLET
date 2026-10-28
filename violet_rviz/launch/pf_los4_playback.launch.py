"""RViz and RC plot conversion for recorded bags; no live vehicle connection."""
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def playback_nodes(context):
    namespace = LaunchConfiguration('vehicle_ns').perform(context).strip('/')
    config = LaunchConfiguration('rviz_config').perform(context)
    with open(config, encoding='utf-8') as source:
        content = source.read().replace('/drone1/', f'/{namespace}/' if namespace else '/')
    handle = tempfile.NamedTemporaryFile(mode='w', suffix='.rviz', delete=False)
    with handle:
        handle.write(content)
    return [
        Node(package='violet_rviz', executable='rc_status_plot', namespace=namespace,
             output='screen', parameters=[{'use_sim_time': True}]),
        Node(package='rviz2', executable='rviz2', arguments=['-d', handle.name],
             output='screen', parameters=[{'use_sim_time': True}]),
    ]


def generate_launch_description():
    config = os.path.join(get_package_share_directory('violet_rviz'), 'rviz', 'pf_los4.rviz')
    return LaunchDescription([
        DeclareLaunchArgument('vehicle_ns', default_value='drone1'),
        DeclareLaunchArgument('rviz_config', default_value=config),
        OpaqueFunction(function=playback_nodes),
    ])
