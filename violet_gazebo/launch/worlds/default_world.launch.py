#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

  # Get PX4 directory
  PX4_DIR = os.environ['PX4_DIR']

  # Get Gazebo Classic launch directory
  gazebo_dir = os.path.join(get_package_share_directory('gazebo_ros'), 'launch')

  # Get world model path
  world = os.path.join(get_package_share_directory('violet_gazebo'), 'worlds', 'empty.world')

  return LaunchDescription([
    # Get VioLET and PX4 Gazebo models and plugins
    SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', PX4_DIR + '/build/px4_sitl_default/build_gazebo-classic'),
    SetEnvironmentVariable('GAZEBO_MODEL_PATH', PX4_DIR + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models' + ':' + get_package_share_directory('violet_gazebo') + '/models'),

    # Declare launching variables
    DeclareLaunchArgument('gui', default_value='true'),
    DeclareLaunchArgument('world', default_value=world),

    # Launch Gazebo server
    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([gazebo_dir, '/gzserver.launch.py']),
      launch_arguments={'world': LaunchConfiguration('world'), 'verbose': 'true'}.items()
    ),

    IncludeLaunchDescription(
      PythonLaunchDescriptionSource([gazebo_dir, '/gzclient.launch.py']),
      condition=LaunchConfigurationEquals('gui', 'true')
    )
  ])