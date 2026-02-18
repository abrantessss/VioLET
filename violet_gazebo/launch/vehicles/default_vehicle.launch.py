#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, LogInfo, OpaqueFunction
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

# Get PX4 directory and create temporary directory
PX4_DIR = os.environ['PX4_DIR']
PX4_TMP_DIR = os.environ['HOME'] + '/tmp/px4_dir'
os.makedirs(PX4_TMP_DIR, exist_ok=True)

def vehicle_launch(context, *args, **kwargs):
  
  # Define the vehicle model to launch
  vehicle_model = str(LaunchConfiguration('vehicle_model').perform(context))
  vehicle_id  = int(LaunchConfiguration('vehicle_id').perform(context))
  port_increment = vehicle_id - 1

  # Get environment variables
  environment = os.environ
  environment['PX4_SIM_MODEL'] = 'gazebo-classic_' + vehicle_model
  environment['ROS_VERSION'] = '2'

  # Get PX4 and VioLET gazebo directories
  px4_gazebo_dir = os.path.join(PX4_DIR, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic')
  gazebo_dir = str(LaunchConfiguration('gazebo_dir').perform(context))

  # Get UAV model
  model = os.path.join(gazebo_dir, 'models/' + vehicle_model + '/' + vehicle_model + str(vehicle_id) + '.sdf')

  print(model)

  model_gen_process = ExecuteProcess(
    cmd=[
      os.path.join(px4_gazebo_dir, 'scripts/jinja_gen.py',),
      os.path.join(gazebo_dir, 'models/' + vehicle_model + '/' + vehicle_model + '.sdf.jinja'),
      gazebo_dir,
      '--mavlink_id=' + str(vehicle_id),
      '--mavlink_udp_port=' + str(14540 + port_increment),
      '--mavlink_tcp_port=' + str(4560 + port_increment),
      '--gst_udp_port=' + str(5600 + port_increment),
      '--video_uri=' + str(5600 + port_increment),
      '--mavlink_cam_udp_port=' + str(14530 + port_increment),
      '--output-file=' + model,
      '--generate_ros_models=True'
    ],
    env=environment,
    output='screen'
  )

  # Spawn the vehicle model
  spawn_model = Node(
    package='gazebo_ros',
    executable='spawn_entity.py',
    arguments=[
      '-entity', 'drone' + str(vehicle_id),
      '-file', model,
      '-x', LaunchConfiguration('x').perform(context),
      '-y', LaunchConfiguration('y').perform(context),
      '-z', LaunchConfiguration('z').perform(context),
      '-R', LaunchConfiguration('R').perform(context),
      '-P', LaunchConfiguration('P').perform(context),
      '-Y', LaunchConfiguration('Y').perform(context),
      '-robot_namespace', 'drone' + str(vehicle_id)
    ],
    output='screen'
  )

  # Launch PX4 simulator
  px4_sitl_process = ExecuteProcess(
    cmd=[
      PX4_DIR + '/build/px4_sitl_default/bin/px4',
      PX4_DIR + '/ROMFS/px4fmu_common/',
      '-s',
      PX4_DIR + '/ROMFS/px4fmu_common/init.d-posix/rcS',
      '-i ' + str(port_increment)
    ],
    prefix='bash -c "$0 $@"',
    cwd=PX4_TMP_DIR,
    output='screen',
    env=environment,
    shell=False
  )

  return [model_gen_process,
    
    # Launch vehicle after sdf model is generated
    RegisterEventHandler(
      OnProcessExit(
        target_action=model_gen_process,
        on_exit=[
          LogInfo(msg='Vehicle SDF model generated'),
          spawn_model
        ]
      )
    ),

    RegisterEventHandler(
      OnProcessExit(
        target_action=spawn_model,
        on_exit=[
          LogInfo(msg='Vehicle spawned in gazebo'),
          px4_sitl_process
        ]
      )
    )
  ]


def generate_launch_description():
  '''
    Launch Gazebo with the drone running the PX4 stack with communication ROS2 based
  '''

  return LaunchDescription([
    # Define the environment variables so that gazebo can discover PX4 3D models and plugins
    SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', PX4_DIR + '/build/px4_sitl_default/build_gazebo'),
    SetEnvironmentVariable('GAZEBO_MODEL_PATH', PX4_DIR + '/Tools/sitl_gazebo/models' + ':' + get_package_share_directory('violet_gazebo') + '/models'),

    # Define variables used in launch vehicle
    DeclareLaunchArgument('gazebo_dir', default_value=os.path.join(PX4_DIR, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'), description='Path to gazebo directory where UAV model is located'),
    DeclareLaunchArgument('vehicle_model', default_value='iris', description='UAV model name'),
    DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network'),
    DeclareLaunchArgument('x', default_value='0.0', description='X position expressed in ENU'),
    DeclareLaunchArgument('y', default_value='0.0', description='Y position expressed in ENU'),
    DeclareLaunchArgument('z', default_value='1.8', description='Z position expressed in ENU'),
    DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
    DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
    DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),
    
    
    OpaqueFunction(function=vehicle_launch)
  ])