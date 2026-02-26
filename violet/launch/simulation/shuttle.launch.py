#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

  # ----------------------------------------
  # ---------- LAUNCH SIMULATION -----------
  # ----------------------------------------

  world_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/worlds/helipad.launch.py'))
  )
  
  vehicle_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/vehicles/shuttle.launch.py'))
    )
     
  return LaunchDescription([
    world_launch_file,
    vehicle_launch_file
  ])