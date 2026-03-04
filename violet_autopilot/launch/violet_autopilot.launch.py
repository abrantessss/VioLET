#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
  # ----------------------------------------
  # ---- DECLARE THE LAUNCH ARGUMENTS ------
  # ----------------------------------------
  
  # Namespace and ID of the vehicle as parameter received by the launch file
  id_arg = DeclareLaunchArgument('vehicle_id', default_value='1')
  namespace_arg = DeclareLaunchArgument('vehicle_ns', default_value='drone')

  # Get the name of the .yaml topics and setup configuration 
  config_yaml_arg = DeclareLaunchArgument(
    'config_yaml', 
    default_value=os.path.join(get_package_share_directory('violet_autopilot'), 'config', 'config.yaml')
  )

  # Create communication node
  autopilot_node = Node(
    package='violet_autopilot',
    namespace=[
      LaunchConfiguration('vehicle_ns'), 
      LaunchConfiguration('vehicle_id')
    ],
    executable='violet_autopilot',
    name='violet_autopilot',
    output="screen",
    emulate_tty=True,
    parameters=[
      # Pass the file which contains the topics configuration
      LaunchConfiguration('config_yaml'), 
      {
        'vehicle_id': LaunchConfiguration('vehicle_id'),
        'vehicle_ns': LaunchConfiguration('vehicle_ns')
      }
    ]
  )

  return LaunchDescription([
    id_arg,
    namespace_arg,
    config_yaml_arg,

    # Launch file
    autopilot_node
  ])