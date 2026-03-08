#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import (
  ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable,
  RegisterEventHandler, LogInfo, OpaqueFunction, IncludeLaunchDescription
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart
from ament_index_python.packages import get_package_share_directory

PX4_DIR = os.environ['PX4_DIR']
PX4_TMP_DIR = os.path.join(os.environ['HOME'], 'tmp', 'px4_dir')
os.makedirs(PX4_TMP_DIR, exist_ok=True)


def vehicle_launch(context, *args, **kwargs):
  vehicle_model = str(LaunchConfiguration('vehicle_model').perform(context))   
  vehicle_id = int(LaunchConfiguration('vehicle_id').perform(context))
  vehicle_auto = int(LaunchConfiguration('vehicle_auto').perform(context))
  port_increment = vehicle_id - 1

  x = LaunchConfiguration('x').perform(context)
  y = LaunchConfiguration('y').perform(context)
  z = LaunchConfiguration('z').perform(context)
  R = LaunchConfiguration('R').perform(context)
  P = LaunchConfiguration('P').perform(context)
  Y = LaunchConfiguration('Y').perform(context)

  environment = os.environ.copy()

  # New Gazebo / PX4 GZ variables
  environment['PX4_SIMULATOR'] = 'gz'
  environment['PX4_SIM_MODEL'] = vehicle_model      
  environment['PX4_GZ_MODEL_POSE'] = f'{x},{y},{z},{R},{P},{Y}'
  environment['PX4_SYS_AUTOSTART'] = str(vehicle_auto)
  environment['PX4_GZ_WORLD'] = 'default'
  environment['PX4_UXRCE_DDS_NS'] = f"drone{vehicle_id}"
  environment['PX4_SIM_SPEED_FACTOR'] = '1'
  #environment['HEADLESS'] = '1'

  # Add custom models through Gazebo resource path
  px4_gz_dir = os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz')
  px4_gz_plugin_path = os.path.join(
    PX4_DIR, 'build', 'px4_sitl_default', 'src', 'modules', 'simulation', 'gz_plugins'
  )
  model_dir = LaunchConfiguration('model_dir').perform(context)
  world_dir = LaunchConfiguration('world_dir').perform(context)

  environment['PX4_GZ_MODELS'] = model_dir
  environment['PX4_GZ_WORLDS'] = world_dir
  prev_plugin_path = environment.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
  environment['GZ_SIM_SYSTEM_PLUGIN_PATH'] = ':'.join(
    [p for p in [px4_gz_plugin_path, prev_plugin_path] if p]
  )

  gazebo_dir = str(LaunchConfiguration('gazebo_dir').perform(context))
  extra_resource_paths = [
    px4_gz_dir,
    os.path.join(px4_gz_dir, 'models'),
    os.path.join(px4_gz_dir, 'worlds'),
    gazebo_dir,
    os.path.join(gazebo_dir, 'models'),
    os.path.join(gazebo_dir, 'worlds'),
  ]
  prev_resource_path = environment.get('GZ_SIM_RESOURCE_PATH', '')
  environment['GZ_SIM_RESOURCE_PATH'] = ':'.join(
    [p for p in extra_resource_paths + [prev_resource_path] if p]
  )

  # Launch PX4 SITL; PX4 will start Gazebo and spawn the model
  px4_sitl_process = ExecuteProcess(
    cmd=[
      os.path.join(PX4_DIR, 'build', 'px4_sitl_default', 'bin', 'px4'),
      os.path.join(PX4_DIR, 'ROMFS', 'px4fmu_common'),
      '-s',
      os.path.join(PX4_DIR, 'ROMFS', 'px4fmu_common', 'init.d-posix', 'rcS'),
      '-i',
      str(port_increment),
    ],
    cwd=PX4_TMP_DIR,
    output='screen',
    env=environment,
  ) 

  xrce_agent = ExecuteProcess(
    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    env=environment
  )

  interface_launch_file = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('violet_interface'),
        'launch/violet_interface.launch.py'
      )
    ),
    launch_arguments={
      'vehicle_id': LaunchConfiguration('vehicle_id'),
      'vehicle_ns': 'drone',
    }.items(),
  )

  autopilot_launch_file = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('violet_autopilot'),
        'launch/violet_autopilot.launch.py'
      )
    ),
    launch_arguments={
      'vehicle_id': LaunchConfiguration('vehicle_id'),
      'vehicle_ns': 'drone',
    }.items(),
  )

  return [
    xrce_agent,
    px4_sitl_process,
    interface_launch_file,
    autopilot_launch_file
  ]


def generate_launch_description():
  return LaunchDescription([
    # custom gz world models
    DeclareLaunchArgument(
      'gazebo_dir',
      default_value=get_package_share_directory('violet_gz'),
      description='Path to Gazebo resources'
    ),

    # Use a new-Gazebo PX4 model, not classic iris
    DeclareLaunchArgument('vehicle_model', default_value='gz_x500'),
    DeclareLaunchArgument('vehicle_id', default_value='1'),
    DeclareLaunchArgument('vehicle_auto', default_value='4001'),
    DeclareLaunchArgument('x', default_value='0.0'),
    DeclareLaunchArgument('y', default_value='0.0'),
    DeclareLaunchArgument('z', default_value='0'),
    DeclareLaunchArgument('R', default_value='0.0'),
    DeclareLaunchArgument('P', default_value='0.0'),
    DeclareLaunchArgument('Y', default_value='0.0'),

    DeclareLaunchArgument(
      'model_dir',
      default_value=os.path.join(os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz'), 'models'),
      description='Directory containing custom Gazebo models'
    ),
    DeclareLaunchArgument(
      'world_dir',
      default_value=os.path.join(os.path.join(PX4_DIR, 'Tools', 'simulation', 'gz'), 'worlds'),
      description='Directory containing custom Gazebo worlds'
    ),
    
    OpaqueFunction(function=vehicle_launch)
  ])
