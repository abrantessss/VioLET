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

    # Launch default vehicle launch file
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_gazebo'), 'launch/vehicles/default_vehicle.launch.py')),
        launch_arguments={ #ENU coordinates
            'x': '-10',
            'y': '0',
            'z': '0.1',
            'R': '0',
            'P': '0',
            'Y': '-1.5781',
            'vehicle_id': '1',
            'vehicle_model': 'plane',
        }.items()
    )
  ])