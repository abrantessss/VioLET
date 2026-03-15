#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  # ----------------------------------------
  # ---------- LAUNCH SIMULATION -----------
  # ----------------------------------------

  world_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/worlds/helipad.launch.py'))
  )
  
  vehicle_launch_file = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/vehicles/shuttle.launch.py')),
    launch_arguments={
      'config_yaml': LaunchConfiguration('config_yaml')
    }.items()
  )

  config_yaml_arg = DeclareLaunchArgument(
      'config_yaml',
      default_value=os.path.join(get_package_share_directory('violet'), 'config', 'shuttle.yaml')
  )
     
  return LaunchDescription([
    config_yaml_arg,
    world_launch_file,
    vehicle_launch_file
  ])
