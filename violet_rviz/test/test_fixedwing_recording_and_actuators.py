from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml
from launch import LaunchContext
from violet_rviz.actuator_outputs import normalized_output
from violet_rviz.bag_recording import default_bag_directory, launch_recording, recording_command
from violet_rviz.mavsdk_visualization import MavsdkVisualization


def test_pwm_scaling_and_missing_outputs():
    assert normalized_output(1000) == -1
    assert normalized_output(1500) == 0
    assert normalized_output(2000) == 1
    assert normalized_output(1100, 1100, 1550, 1900) == -1
    assert normalized_output(1550, 1100, 1550, 1900) == 0
    assert normalized_output(2100) == 1
    assert normalized_output(0) is None
    assert normalized_output(float('nan')) is None
    assert normalized_output(-0.3, encoding='normalized') == -0.3


def test_recording_off_creates_nothing(tmp_path):
    context = LaunchContext()
    context.launch_configurations['record_bag'] = 'false'
    assert launch_recording(context) == []
    assert not list(tmp_path.iterdir())


def test_recording_destination_and_topic_selection(tmp_path):
    import re
    command = recording_command(tmp_path/'bags', 'drone1', datetime(2026, 10, 15, 12, 30, 0))
    assert command[command.index('--storage')+1] == 'sqlite3'
    assert Path(command[-1]).parent == tmp_path/'bags'
    assert Path(command[-1]).name.startswith('real_fixedwing_test_data_20261015_123000')
    pattern = command[command.index('--regex')+1]
    for topic in ('/drone1/telemetry/camera/image_raw', '/drone1/telemetry/los4/state',
                  '/drone1/telemetry/flight_mode', '/drone1/rviz_plots/path_error/norm', '/tf_static'):
        assert re.match(pattern, topic)
    assert not re.match(pattern, '/drone2/telemetry/pose')
    share = tmp_path/'workspace/install/violet_rviz/share/violet_rviz'
    (tmp_path/'workspace/src/VioLET/violet_plots').mkdir(parents=True)
    assert default_bag_directory(share) == str(tmp_path/'workspace/src/VioLET/violet_plots/bags')


def test_five_named_actuators_fixed_range_and_mode_marker():
    root = Path(__file__).resolve().parents[1]
    config = yaml.safe_load((root/'rviz/pf_los4.rviz').read_text())
    displays = config['Visualization Manager']['Displays']
    actuator = next(d for d in displays if d.get('Name') == 'Actuator Outputs')
    assert actuator['Series']['Series Count'] == 5
    assert actuator['Y Axis'] == {'Auto Scale': False, 'Y Min': -1, 'Y Max': 1}
    assert actuator['Series']['Series 1']['Label'] == 'Motor (MAIN 8)'
    assert any(d.get('Topic', {}).get('Value') == '/drone1/rviz_plots/vehicle_mode' for d in displays)


def test_mode_status_shows_actual_mode_and_stale_state():
    from builtin_interfaces.msg import Time
    class Clock:
        nanoseconds = 1000000000
        def now(self):
            return self
        def to_msg(self):
            return Time(sec=int(self.nanoseconds/1e9))
    clock = Clock()
    messages = []
    node = SimpleNamespace(get_clock=lambda: clock, mode_status={}, status_position=None,
                           mode_pub=SimpleNamespace(publish=messages.append))
    for key, data in [('flight_mode', 'OFFBOARD'), ('armed', True), ('kill_active', False)]:
        MavsdkVisualization.status_cb(node, key, SimpleNamespace(data=data))
    MavsdkVisualization.publish_status(node)
    assert 'Mode: OFFBOARD' in messages[-1].text
    assert 'Armed: YES' in messages[-1].text
    assert 'Kill CH6: OFF' in messages[-1].text
    clock.nanoseconds += 4000000000
    MavsdkVisualization.publish_status(node)
    assert 'Mode: UNKNOWN' in messages[-1].text
    assert 'Kill CH6: UNKNOWN' in messages[-1].text


def test_recording_enabled_creates_process(tmp_path):
    context = LaunchContext()
    context.launch_configurations.update({'record_bag': 'true', 'bag_directory': str(tmp_path/'bags'),
                                          'vehicle_ns': 'drone1'})
    assert len(launch_recording(context)) == 1
