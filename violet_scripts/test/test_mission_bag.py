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


def test_optional_wind_and_airspeed_are_recorded_without_conversion():
    from types import SimpleNamespace
    from px4_msgs.msg import Wind, AirspeedValidated
    written = []
    node = SimpleNamespace(record_message=lambda suffix, msg: written.append((suffix, msg)))
    wind = Wind(windspeed_north=-5.0, windspeed_east=0.0)
    airspeed = AirspeedValidated(true_airspeed_m_s=20.0)
    EasyGliderMissionNode.wind_cb(node, wind)
    EasyGliderMissionNode.airspeed_cb(node, airspeed)
    assert written == [('fmu/out/wind', wind), ('fmu/out/airspeed_validated_v1', airspeed)]


@pytest.mark.parametrize('controller', ['los4', 'los4_wind'])
def test_both_wind_experiments_store_sensor_messages_and_distinct_controller_name(tmp_path, controller):
    from px4_msgs.msg import Wind, AirspeedValidated
    node = MissionHarness(tmp_path)
    node.start_recording(Trajectory(path_type=2))
    wind = Wind(windspeed_north=-5.0, windspeed_east=0.2, variance_north=0.1, variance_east=0.1)
    airspeed = AirspeedValidated(true_airspeed_m_s=20.0, airspeed_source=1)
    EasyGliderMissionNode.wind_cb(node, wind)
    msg = results()
    msg.layout.dim[0].label = controller
    msg.layout.dim.append(MultiArrayDimension(label='record_wind_airspeed', size=1))
    msg.data.append(1.0)
    node.results_cb(msg)
    EasyGliderMissionNode.airspeed_cb(node, airspeed)
    path = node.bag.path
    assert path.name.startswith(f'{controller}_circle_')
    node.bag.close()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    records = {}
    while reader.has_next():
        topic, payload, _ = reader.read_next()
        records[topic] = payload
    saved_wind = deserialize_message(records['/drone2/fmu/out/wind'], Wind)
    saved_airspeed = deserialize_message(records['/drone2/fmu/out/airspeed_validated_v1'], AirspeedValidated)
    assert saved_wind.windspeed_north == pytest.approx(wind.windspeed_north)
    assert saved_wind.windspeed_east == pytest.approx(wind.windspeed_east)
    assert saved_wind.variance_north == pytest.approx(wind.variance_north)
    assert saved_airspeed.true_airspeed_m_s == pytest.approx(airspeed.true_airspeed_m_s)
    assert saved_airspeed.airspeed_source == airspeed.airspeed_source


def test_normal_los4_excludes_wind_and_airspeed_from_bag(tmp_path):
    from px4_msgs.msg import Wind, AirspeedValidated
    node = MissionHarness(tmp_path)
    node.start_recording(Trajectory(path_type=2))
    EasyGliderMissionNode.wind_cb(node, Wind())
    EasyGliderMissionNode.airspeed_cb(node, AirspeedValidated())
    node.results_cb(results())  # No recording flag: ordinary LOS4 defaults to off.
    EasyGliderMissionNode.wind_cb(node, Wind())
    EasyGliderMissionNode.airspeed_cb(node, AirspeedValidated())
    path = node.bag.path
    node.bag.close()
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='sqlite3'),
                rosbag2_py.ConverterOptions('', ''))
    topics = {topic.name for topic in reader.get_all_topics_and_types()}
    assert '/drone2/fmu/out/wind' not in topics
    assert '/drone2/fmu/out/airspeed_validated_v1' not in topics
    assert '/drone2/fmu/telemetry/controller_results' in topics
