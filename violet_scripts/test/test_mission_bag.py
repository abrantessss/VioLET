"""Check bag naming, buffered mission commands, and serialized readback."""
from types import SimpleNamespace

import pytest
import rosbag2_py
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from violet_msgs.msg import State, Trajectory

from violet_scripts.easyglider_mission_base import EasyGliderMissionNode
from violet_scripts.mission_bag import MissionBag


class MissionHarness:
    start_recording = EasyGliderMissionNode.start_recording
    record_message = EasyGliderMissionNode.record_message
    results_cb = EasyGliderMissionNode.results_cb
    timer_cb = EasyGliderMissionNode.timer_cb

    def __init__(self, directory, enabled=True):
        self.parameters = {'bag_directory': str(directory), 'record_bag': enabled}
        self.vehicle_ns = 'drone2'
        self.mission_label = 'circle'
        self.bag = None
        self.recording_pending = False
        self.recording_started_at = None
        self.pending_messages = []
        self.timestamp = 1000000000

    def get_parameter(self, name):
        return SimpleNamespace(value=self.parameters[name])

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=self.timestamp))

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)


def results():
    msg = Float64MultiArray()
    # Gains need not be adjacent or listed in filename order.
    msg.layout.dim = [
        MultiArrayDimension(label='los4', size=0),
        MultiArrayDimension(label='k2', size=1),
        MultiArrayDimension(label='position', size=3),
        MultiArrayDimension(label='k1', size=1),
    ]
    msg.data = [18.0, 1.0, 2.0, -30.0, 0.5]
    return msg


def test_easyglider_preserves_command_and_early_telemetry(tmp_path):
    node = MissionHarness(tmp_path)
    trajectory = Trajectory(path_type=2, circle=[1.0, 2.0, -30.0, 100.0, 18.0])
    node.start_recording(trajectory)
    assert node.bag is None
    node.timestamp += 1000000
    node.record_message('fmu/telemetry/state', State())
    node.timestamp += 1000000
    node.results_cb(results())
    path = node.bag.path
    assert path.name.startswith('los4_circle_k10.5_k218_')
    assert not node.pending_messages and not node.recording_pending
    node.timestamp += 1000000
    node.results_cb(results())
    node.bag.close()
    assert (path / 'metadata.yaml').is_file()

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    records = []
    while reader.has_next():
        records.append(reader.read_next())
    assert len(records) == 4
    topic, payload, timestamp = records[0]
    assert topic == '/drone2/fmu/mode/follow'
    assert timestamp == 1000000000
    assert list(deserialize_message(payload, Trajectory).circle) == list(trajectory.circle)
    assert records[1][0] == '/drone2/fmu/telemetry/state'
    assert records[2][0] == '/drone2/fmu/telemetry/controller_results'
    assert deserialize_message(records[2][1], Float64MultiArray).data == results().data


def test_shuttle_filename_remains_compatible(tmp_path):
    node = MissionHarness(tmp_path)
    bag = MissionBag(node, tmp_path, 'mellinger', 'line', '0.0005')
    assert bag.path.name.startswith('mellinger_line_kphi0.0005_')
    bag.close()


def test_disabled_recording(tmp_path):
    node = MissionHarness(tmp_path, enabled=False)
    node.start_recording(Trajectory())
    node.results_cb(results())
    assert node.bag is None and not node.recording_pending
    assert not node.pending_messages
    assert not list(tmp_path.iterdir())


def test_missing_controller_results_reports_failure(tmp_path):
    node = MissionHarness(tmp_path)
    node.start_recording(Trajectory())
    node.recording_started_at -= 6.0
    with pytest.raises(RuntimeError, match='No LOS4 controller results'):
        node.timer_cb()


def test_nonfinite_gains_rejected(tmp_path):
    node = MissionHarness(tmp_path)
    node.start_recording(Trajectory())
    msg = results()
    msg.data[0] = float('nan')
    with pytest.raises(RuntimeError, match='Non-finite LOS4 k2'):
        node.results_cb(msg)


def test_combined_records_priming_and_following_in_one_bag(tmp_path):
    from violet_scripts.combined_vehicle_mission_base import CombinedVehicleMissionNode

    class CombinedHarness(MissionHarness):
        publish_trajectory = CombinedVehicleMissionNode.publish_trajectory
        record_message = CombinedVehicleMissionNode.record_message

        def __init__(self, directory):
            super().__init__(directory)
            self.shuttle_ns = 'drone3'
            self.trajectory = None
            self.commands = []
            self.follow_pub = SimpleNamespace(publish=self.commands.append)

        def build_trajectory(self):
            return Trajectory(path_type=2, circle=[0.0, 0.0, -3.0, 30.0, 2.0])

    node = CombinedHarness(tmp_path)
    node.publish_trajectory()
    path = node.bag.path
    assert path.name.startswith('combined_circle_20')
    assert '_kphi' not in path.name and '__' not in path.name
    node.publish_trajectory()
    assert node.bag.path == path
    node.record_message('/drone3/fmu/telemetry/state', State())
    node.record_message('/drone4/fmu/telemetry/state', State())
    node.record_message('/drone3/fmu/telemetry/controller_results', results())
    node.bag.close()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = []
    while reader.has_next():
        topics.append(reader.read_next()[0])
    assert topics.count('/drone3/fmu/mode/follow') == 2
    assert '/drone3/fmu/telemetry/state' in topics
    assert '/drone4/fmu/telemetry/state' in topics
    assert '/drone3/fmu/telemetry/controller_results' in topics
    assert len(node.commands) == 2
