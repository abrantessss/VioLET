#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rviz_config = os.path.join(
        get_package_share_directory('violet_rviz'),
        'rviz',
        'violet_plots.rviz',
    )

    vehicle_ns_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone1')
    plot_data_topic_arg = DeclareLaunchArgument('plot_data_topic', default_value='')
    output_prefix_arg = DeclareLaunchArgument('output_prefix', default_value='rviz_plots')
    history_length_arg = DeclareLaunchArgument('history_length', default_value='2000')
    desired_velocity_path_scaled_arg = DeclareLaunchArgument(
        'desired_velocity_path_scaled',
        default_value='false',
    )
    rviz_config_arg = DeclareLaunchArgument('rviz_config', default_value=rviz_config)

    bridge_node = Node(
        package='violet_rviz',
        executable='plot_bridge',
        name='violet_plot_bridge',
        output='screen',
        parameters=[{
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'plot_data_topic': LaunchConfiguration('plot_data_topic'),
            'output_prefix': LaunchConfiguration('output_prefix'),
            'history_length': LaunchConfiguration('history_length'),
            'desired_velocity_path_scaled': LaunchConfiguration(
                'desired_velocity_path_scaled'),
        }],
    )

    rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    return LaunchDescription([
        vehicle_ns_arg,
        plot_data_topic_arg,
        output_prefix_arg,
        history_length_arg,
        desired_velocity_path_scaled_arg,
        rviz_config_arg,
        bridge_node,
        rviz_process,
    ])
