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

  world = os.path.join(gazebo_dir, 'worlds', 'shuttle_gimbal.world')
  default_world_launch = os.path.join(gazebo_dir, 'launch/worlds/default_world.launch.py')
  default_vehicle_launch = os.path.join(gazebo_dir, 'launch/vehicles/default_vehicle.launch.py')

  return LaunchDescription([
    DeclareLaunchArgument(
      'gui',
      default_value='true',
      description='Launch Gazebo client GUI'
    ),
    DeclareLaunchArgument(
      'config_yaml',
      default_value=os.path.join(violet_dir, 'config', 'shuttle_gimbal.yaml'),
      description='Path to the shuttle gimbal autopilot config YAML file'
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
        'z': '5.4',
        'R': '0',
        'P': '0',
        'Y': '0',
        'vehicle_id': '1',
        'vehicle_model': 'shuttle',
        'vehicle_sdf_model': 'shuttle',
        'gazebo_dir': gazebo_dir,
        'config_yaml': LaunchConfiguration('config_yaml'),
        'start_xrce_agent': 'false',
      }.items()
    ),
  ])
