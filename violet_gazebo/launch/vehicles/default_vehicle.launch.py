#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, RegisterEventHandler, LogInfo, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
  vehicle_sdf_model = str(LaunchConfiguration('vehicle_sdf_model').perform(context))
  if not vehicle_sdf_model:
    vehicle_sdf_model = vehicle_model
  vehicle_id  = int(LaunchConfiguration('vehicle_id').perform(context))
  vehicle_id_str = str(vehicle_id)
  port_increment = vehicle_id - 1
  start_xrce_agent = str(LaunchConfiguration('start_xrce_agent').perform(context)).lower() == 'true'
  vehicle_namespace = 'drone' + str(vehicle_id)
  px4_instance_dir = os.path.join(PX4_TMP_DIR, vehicle_namespace)
  os.makedirs(px4_instance_dir, exist_ok=True)
  config_yaml = str(LaunchConfiguration('config_yaml').perform(context))

  # Get environment variables
  environment = dict(os.environ)
  environment['PX4_SIM_MODEL'] = 'gazebo-classic_' + vehicle_model
  environment['PX4_UXRCE_DDS_NS'] = vehicle_namespace
  environment['ROS_VERSION'] = '2'

  # Get PX4 and VioLET gazebo directories
  px4_gazebo_dir = os.path.join(PX4_DIR, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic')
  gazebo_dir = str(LaunchConfiguration('gazebo_dir').perform(context))
  model_dir = os.path.join(gazebo_dir, 'models', vehicle_sdf_model)
  jinja_template = os.path.join(model_dir, vehicle_model + '.sdf.jinja')
  if not os.path.exists(jinja_template):
    fallback_templates = sorted(
      file_name for file_name in os.listdir(model_dir)
      if file_name.endswith('.sdf.jinja')
    )
    if not fallback_templates:
      raise FileNotFoundError(f'No SDF Jinja template found in {model_dir}')
    jinja_template = os.path.join(model_dir, fallback_templates[0])

  # Get UAV model
  model = os.path.join(model_dir, vehicle_model + str(vehicle_id) + '.sdf')

  model_gen_process = ExecuteProcess(
    cmd=[
      os.path.join(px4_gazebo_dir, 'scripts/jinja_gen.py',),
      jinja_template,
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
      '-entity', vehicle_namespace,
      '-file', model,
      '-x', LaunchConfiguration('x').perform(context),
      '-y', LaunchConfiguration('y').perform(context),
      '-z', LaunchConfiguration('z').perform(context),
      '-R', LaunchConfiguration('R').perform(context),
      '-P', LaunchConfiguration('P').perform(context),
      '-Y', LaunchConfiguration('Y').perform(context),
      '-robot_namespace', vehicle_namespace
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
    cwd=px4_instance_dir,
    output='screen',
    env=environment,
    shell=False
  )

  # Start the Micro XRCE-DDS agent
  xrce_agent = ExecuteProcess(
    cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
    #output='screen',
    env=environment,
    shell=False
  )

  # Call interface package launch file 
  interface_launch_file = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_interface'), 'launch/violet_interface.launch.py')),
    # Define costume launch arguments/parameters 
    launch_arguments={
      'vehicle_id': vehicle_id_str,
      'vehicle_ns': 'drone',
    }.items(),
  )

  # Call autopilot package launch file 
  autopilot_launch_file = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('violet_autopilot'), 'launch/violet_autopilot.launch.py')),
    # Define costume launch arguments/parameters 
    launch_arguments={
      'vehicle_id': vehicle_id_str,
      'vehicle_ns': 'drone',
      'config_yaml': config_yaml
    }.items(),
  )

  spawn_exit_actions = [LogInfo(msg='Vehicle spawned in gazebo')]
  if start_xrce_agent:
    # A single Micro XRCE-DDS agent can serve multiple PX4 instances.
    spawn_exit_actions.append(xrce_agent)
  spawn_exit_actions.extend([
    px4_sitl_process,
    interface_launch_file,
    autopilot_launch_file,
  ])

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
        on_exit=spawn_exit_actions
      )
    )
  ]


def generate_launch_description():
  '''
    Launch Gazebo with the drone running the PX4 stack with communication ROS2 based
  '''

  return LaunchDescription([
    # Define the environment variables so that gazebo can discover PX4 3D models and plugins
    SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', PX4_DIR + '/build/px4_sitl_default/build_gazebo-classic'),
    SetEnvironmentVariable('GAZEBO_MODEL_PATH', PX4_DIR + '/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models' + ':' + get_package_share_directory('violet_gazebo') + '/models'),

    # Define variables used in launch vehicle
    DeclareLaunchArgument('gazebo_dir', default_value=os.path.join(PX4_DIR, 'Tools/simulation/gazebo-classic/sitl_gazebo-classic'), description='Path to gazebo directory where UAV model is located'),
    DeclareLaunchArgument('vehicle_model', default_value='iris', description='UAV model name'),
    DeclareLaunchArgument('vehicle_sdf_model', default_value='', description='Gazebo model folder used for the generated SDF; defaults to vehicle_model'),
    DeclareLaunchArgument('vehicle_id', default_value='1', description='Drone ID in the network'),
    DeclareLaunchArgument('start_xrce_agent', default_value='true', description='Whether to start the shared Micro XRCE-DDS agent'),
    DeclareLaunchArgument(
      'config_yaml',
      default_value=os.path.join(get_package_share_directory('violet_autopilot'), 'config', 'config.yaml'),
      description='Path to the autopilot parameter file'
    ),
    DeclareLaunchArgument('x', default_value='0.0', description='X position expressed in ENU'),
    DeclareLaunchArgument('y', default_value='0.0', description='Y position expressed in ENU'),
    DeclareLaunchArgument('z', default_value='1.8', description='Z position expressed in ENU'),
    DeclareLaunchArgument('R', default_value='0.0', description='Roll orientation expressed in ENU'),
    DeclareLaunchArgument('P', default_value='0.0', description='Pitch orientation expressed in ENU'),
    DeclareLaunchArgument('Y', default_value='0.0', description='Yaw orientation expressed in ENU'),
    
    
    OpaqueFunction(function=vehicle_launch)
  ])
