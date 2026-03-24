#!/usr/bin/env python3
import os
import yaml
from datetime import datetime

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

def setup_bag_recording(context, *args, **kwargs):
    config_yaml = LaunchConfiguration('config_yaml').perform(context)

    with open(config_yaml, 'r') as f:
        config = yaml.safe_load(f)

    controller_type = config['/**']['ros__parameters']['controllers']['type']

    # Create name: type + date
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_name = f"{controller_type}_{timestamp}"

    # Target folder
    bag_dir = os.path.expanduser('~/VioLET/src/VioLET/violet_plots/bags')
    os.makedirs(bag_dir, exist_ok=True)

    return [
        ExecuteProcess(
            cmd=[
                'ros2', 'bag', 'record',
                '-a',  # record ALL topics
                '-o', os.path.join(bag_dir, bag_name)
            ],
            output='screen'
        )
    ]

def generate_launch_description():

  # ----------------------------------------
  # ---------- LAUNCH SIMULATION -----------
  # ----------------------------------------

  world_launch_file = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/worlds/helipad.launch.py'))
  )
  
  vehicle_launch_file = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/vehicles/easyglider.launch.py')),
    launch_arguments={
      'config_yaml': LaunchConfiguration('config_yaml')
    }.items()
  )
     
  config_yaml_arg = DeclareLaunchArgument(
      'config_yaml',
      default_value=os.path.join(get_package_share_directory('violet'), 'config', 'easyglider.yaml')
  )
     
  return LaunchDescription([
    config_yaml_arg,
    OpaqueFunction(function=setup_bag_recording), 
    world_launch_file,
    vehicle_launch_file
  ])