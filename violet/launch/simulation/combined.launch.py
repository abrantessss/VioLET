#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

  gazebo_dir = get_package_share_directory('violet_gazebo')
  violet_dir = get_package_share_directory('violet')

  world = os.path.join(gazebo_dir, 'worlds', 'combined.world')
  default_world_launch = os.path.join(gazebo_dir, 'launch/worlds/default_world.launch.py')
  default_vehicle_launch = os.path.join(gazebo_dir, 'launch/vehicles/default_vehicle.launch.py')

  return LaunchDescription([
    DeclareLaunchArgument(
      'gui',
      default_value='true',
      description='Launch Gazebo client GUI'
    ),
    DeclareLaunchArgument(
      'shuttle_config_yaml',
      default_value=os.path.join(violet_dir, 'config', 'shuttle.yaml'),
      description='Path to the shuttle autopilot config YAML file'
    ),
    DeclareLaunchArgument(
      'easyglider_config_yaml',
      default_value=os.path.join(violet_dir, 'config', 'easyglider.yaml'),
      description='Path to the easyglider autopilot config YAML file'
    ),

    ExecuteProcess(
      cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
      shell=False
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(default_world_launch),
      launch_arguments={
        'gui': LaunchConfiguration('gui'),
        'world': world,
      }.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(default_vehicle_launch),
      launch_arguments={
        'x': '0',
        'y': '0',
        'z': '2.4',
        'R': '0',
        'P': '0',
        'Y': '-1.5781',
        'vehicle_id': '1',
        'vehicle_model': 'shuttle',
        'vehicle_sdf_model': 'shuttle_arm',
        'gazebo_dir': gazebo_dir,
        'config_yaml': LaunchConfiguration('shuttle_config_yaml'),
        'start_xrce_agent': 'false',
      }.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource(default_vehicle_launch),
      launch_arguments={
        'x': '-0.00108377',
        'y': '-0.223943',
        'z': '2',
        'R': '0',
        'P': '0',
        'Y': '-1.5781',
        'vehicle_id': '2',
        'vehicle_model': 'easyglider',
        'vehicle_sdf_model': 'easyglider',
        'gazebo_dir': gazebo_dir,
        'config_yaml': LaunchConfiguration('easyglider_config_yaml'),
        'start_xrce_agent': 'false',
      }.items()
    ),
  ])
