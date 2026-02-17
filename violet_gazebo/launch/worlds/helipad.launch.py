#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  # Get world model path
  world = os.path.join(get_package_share_directory('violet_gazebo'), 'worlds', 'helipad.world')

  return LaunchDescription([

    # Launch default world launch file
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([get_package_share_directory('violet_gazebo'), '/launch/worlds/default_world.launch.py']),
      launch_arguments={'world': world}.items()
    )
  ])