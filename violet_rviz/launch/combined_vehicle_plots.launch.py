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

    arguments = [
        DeclareLaunchArgument('vehicle_ns', default_value='drone1'),
        DeclareLaunchArgument('plot_data_topic', default_value=''),
        DeclareLaunchArgument('attitude_reference_topic', default_value=''),
        DeclareLaunchArgument('force_reference_topic', default_value=''),
        DeclareLaunchArgument('force_achieved_topic', default_value=''),
        DeclareLaunchArgument('velocity_tracking_topic', default_value=''),
        DeclareLaunchArgument('vertical_velocity_tracking_topic', default_value=''),
        DeclareLaunchArgument('output_prefix', default_value='rviz_plots'),
        DeclareLaunchArgument('history_length', default_value='2000'),
        DeclareLaunchArgument('tracking_publish_rate_hz', default_value='20.0'),
        DeclareLaunchArgument('rviz_config', default_value=rviz_config),
    ]

    bridge_node = Node(
        package='violet_rviz',
        executable='plot_bridge',
        name='combined_vehicle_plot_bridge',
        output='screen',
        parameters=[{
            'vehicle_ns': LaunchConfiguration('vehicle_ns'),
            'plot_data_topic': LaunchConfiguration('plot_data_topic'),
            'attitude_reference_topic': LaunchConfiguration('attitude_reference_topic'),
            'force_reference_topic': LaunchConfiguration('force_reference_topic'),
            'force_achieved_topic': LaunchConfiguration('force_achieved_topic'),
            'velocity_tracking_topic': LaunchConfiguration('velocity_tracking_topic'),
            'vertical_velocity_tracking_topic': LaunchConfiguration(
                'vertical_velocity_tracking_topic'),
            'output_prefix': LaunchConfiguration('output_prefix'),
            'history_length': LaunchConfiguration('history_length'),
            'tracking_publish_rate_hz': LaunchConfiguration('tracking_publish_rate_hz'),
        }],
    )

    rviz_process = ExecuteProcess(
        cmd=['rviz2', '-d', LaunchConfiguration('rviz_config')],
        output='screen',
    )

    return LaunchDescription(arguments + [bridge_node, rviz_process])
