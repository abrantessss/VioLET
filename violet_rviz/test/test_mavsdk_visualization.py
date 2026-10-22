"""Coordinate, trajectory, marker, and plot wiring checks without a vehicle."""
import math
from collections import deque
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from std_msgs.msg import Float32MultiArray
from violet_msgs.msg import Trajectory
from violet_rviz.mavsdk_visualization import MavsdkVisualization, ned_coordinates, trajectory_points


def test_ned_coordinates():
    assert ned_coordinates([10.0, 20.0, -30.0]) == [10.0, 20.0, -30.0]


@pytest.mark.parametrize('kind', [2, 3])
def test_closed_reference_paths(kind):
    msg = Trajectory(path_type=kind)
    if kind == 2:
        msg.circle = [1.0, 2.0, -3.0, 10.0, 4.0]
    else:
        msg.lemniscate = [1.0, 2.0, -3.0, 10.0, 4.0]
    points = trajectory_points(msg)
    assert len(points) == 361
    assert points[0] == pytest.approx(points[-1])
    assert all(p[2] == -3.0 for p in points)
    assert points[0] == pytest.approx([11.0, 2.0, -3.0])
    if kind == 3:
        assert points[90] == pytest.approx([1.0, 2.0, -3.0])


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class Harness:
    pose_cb = MavsdkVisualization.pose_cb
    velocity_cb = MavsdkVisualization.velocity_cb
    attitude_cb = MavsdkVisualization.attitude_cb
    outputs_cb = MavsdkVisualization.outputs_cb
    controls_cb = MavsdkVisualization.controls_cb
    scalar = MavsdkVisualization.scalar

    def __init__(self):
        from violet_rviz.actuator_outputs import ACTUATOR_NAMES, DEFAULT_CHANNELS
        self.actuator_channels = list(DEFAULT_CHANNELS)
        self.actuator_min, self.actuator_trim, self.actuator_max = [1000.]*5, [1500.]*5, [2000.]*5
        self.actuator_encoding = 'pwm'
        self.diameter = 0.6
        self.history = deque(maxlen=2)
        self.path_pub, self.pose_pub, self.point_pub, self.marker_pub = [Publisher() for _ in range(4)]
        keys = ['velocity/speed', 'velocity/east', 'velocity/north', 'velocity/down',
                'attitude/roll', 'attitude/pitch', 'attitude/yaw']
        keys += [f'actuators/output_{i}' for i in range(16)]
        keys += [f'actuators/control_{i}' for i in range(8)]
        keys += [f'actuators/{name}' for name in ACTUATOR_NAMES]
        self.scalars = {key: Publisher() for key in keys}


def test_path_history_and_current_ball():
    node = Harness()
    for i in range(3):
        pose = PoseStamped()
        pose.header.frame_id = 'map_ned'
        pose.pose.position.x = float(i)
        pose.pose.orientation.w = 1.0
        node.pose_cb(pose)
    assert len(node.path_pub.messages[-1].poses) == 2
    marker = node.marker_pub.messages[-1]
    assert marker.type == marker.SPHERE
    assert marker.pose.position.x == 2.0
    assert marker.scale.x == marker.scale.y == marker.scale.z == 0.6
    pose.pose.position.x = math.nan
    node.pose_cb(pose)
    assert len(node.marker_pub.messages) == 3


def test_velocity_attitude_and_all_actuator_topics():
    node = Harness()
    twist = TwistStamped()
    twist.twist.linear.x, twist.twist.linear.y = 3.0, 4.0
    node.velocity_cb(twist)
    assert node.scalars['velocity/speed'].messages[-1].data == 5.0
    assert node.scalars['velocity/north'].messages[-1].data == 3.0
    assert node.scalars['velocity/east'].messages[-1].data == 4.0
    attitude = Vector3Stamped()
    attitude.vector.x = 0.2
    node.attitude_cb(attitude)
    assert node.scalars['attitude/roll'].messages[-1].data == pytest.approx(0.2)
    node.outputs_cb(Float32MultiArray(data=[float(i) for i in range(16)]))
    assert node.scalars['actuators/output_15'].messages[-1].data == 15.0
    node.controls_cb(Float32MultiArray(data=[math.nan]+[0.1]*7))
    assert not node.scalars['actuators/control_0'].messages
    assert len(node.scalars['actuators/control_7'].messages) == 1


def test_path_marker_and_graphs_in_rviz_layout():
    config = yaml.safe_load((Path(__file__).resolve().parents[1]/'rviz'/'pf_los4.rviz').read_text())
    assert config['Visualization Manager']['Global Options']['Fixed Frame'] == 'rviz_world'
    displays = config['Visualization Manager']['Displays']
    classes = {d['Class'] for d in displays}
    assert 'rviz_default_plugins/Image' in classes
    assert 'rviz_default_plugins/Marker' in classes
    path_group = next(d for d in displays if d.get('Name') == 'Vehicle Trajectory')
    assert all(d['Class'] == 'rviz_default_plugins/Path' for d in path_group['Displays'])
    assert len([d for d in displays if d['Class'] == 'rviz_2d_plot_plugin/Plot2D']) == 8


def test_ned_display_transform(tmp_path, monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    from importlib.util import module_from_spec, spec_from_file_location
    launch_path = Path(__file__).resolve().parents[1] / 'launch' / 'pf_los4_rviz.launch.py'
    spec = spec_from_file_location('pf_launch', launch_path)
    module = module_from_spec(spec)
    spec.loader.exec_module(module)
    launch = module.generate_launch_description()
    from launch_ros.actions import Node
    from launch import LaunchContext
    from launch.utilities import perform_substitutions
    transform = next(action for action in launch.entities if isinstance(action, Node)
                     and action.node_package == 'tf2_ros')
    context = LaunchContext()
    context.extend_locals({'ros_specific_arguments': {'name': '__node:=ned_display_transform'}})
    args = [perform_substitutions(context, arg) for arg in transform.cmd]
    def value(flag):
        return args[args.index(flag) + 1]
    assert value('--frame-id') == 'rviz_world'
    assert value('--child-frame-id') == 'map_ned'
    x, y, z, w = [float(value('--q' + axis)) for axis in 'xyzw']
    # q=(sqrt(1/2),sqrt(1/2),0,0): North -> display Y,
    # East -> display X, Down -> negative display Z.
    rotation = [
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ]
    assert rotation[0] == pytest.approx([0, 1, 0])
    assert rotation[1] == pytest.approx([1, 0, 0])
    assert rotation[2] == pytest.approx([0, 0, -1])


class DiagnosticsHarness(Harness):
    los4_config_cb = MavsdkVisualization.los4_config_cb
    los4_state_cb = MavsdkVisualization.los4_state_cb
    publish_reference = MavsdkVisualization.publish_reference

    def __init__(self):
        super().__init__()
        from builtin_interfaces.msg import Time
        self.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time(sec=10)))
        self.gamma_pub, self.gamma_xy_pub, self.desired_pub = Publisher(), Publisher(), Publisher()
        keys = ['controller_state', 'gamma', 'gamma_dot', 'velocity/desired', 'velocity/gamma_speed']
        keys += [f'path_error/{axis}' for axis in ('north', 'east', 'down', 'norm')]
        keys += [f'{prefix}/{axis}' for prefix in ('attitude_desired', 'attitude_error')
                 for axis in ('roll', 'pitch', 'heading')]
        self.scalars.update({key: Publisher() for key in keys})


def test_full_config_reference_before_vehicle_motion():
    node = DiagnosticsHarness()
    node.los4_config_cb(Float32MultiArray(data=[1., 1., 2., 0., 0., -30., 200., 0., -30.,
                                              1., 2., -3., 50., 40., 22.]))
    path = node.desired_pub.messages[-1]
    assert len(path.poses) == 361
    assert path.header.frame_id == 'map_ned'
    assert path.poses[0].pose.position.x == 51
    assert path.poses[0].pose.position.y == 2
    assert path.poses[0].pose.position.z == -3
    node.los4_config_cb(Float32MultiArray(data=[1., 1., 1., 0., 0., -30., 200., 0., -30.,
                                              1., 2., -3., 50., 40., 22.]))
    assert len(node.desired_pub.messages[-1].poses) == 2


def test_gamma_speed_path_error_and_wrapped_attitude_error():
    node = DiagnosticsHarness()
    values = [1., 2., 1., 0., 1., 22., 50., 0., -30., 0., 50., 0.,
              52., 3., -31., 0., 22., 0., 0.1, 0.2, -3.1, 0.3, 0.4, 3.1, 15.]
    node.los4_state_cb(Float32MultiArray(data=values))
    assert node.scalars['velocity/desired'].messages[-1].data == 22
    assert node.scalars['velocity/gamma_speed'].messages[-1].data == 50
    assert node.scalars['path_error/norm'].messages[-1].data == pytest.approx(math.sqrt(14))
    assert node.scalars['attitude_error/roll'].messages[-1].data == pytest.approx(0.2)
    assert node.scalars['attitude_error/heading'].messages[-1].data == pytest.approx(6.2-2*math.pi, abs=1e-6)
    assert node.gamma_pub.messages[-1].pose.position.x == 50
    assert len(node.gamma_xy_pub.messages[-1].poses) == 1
    count = len(node.scalars['attitude_desired/roll'].messages)
    values[24] = 0
    node.los4_state_cb(Float32MultiArray(data=values))
    assert len(node.scalars['attitude_desired/roll'].messages) == count
    values[0] = 2  # unsupported wire version
    node.los4_state_cb(Float32MultiArray(data=values))
    assert len(node.gamma_pub.messages) == 2
