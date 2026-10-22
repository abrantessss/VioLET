"""Environmental diagnostics, speed/wind plots and rosbag topic coverage."""
from pathlib import Path
from types import SimpleNamespace
import math
import re
import yaml
import pytest
from violet_rviz.mavsdk_visualization import MavsdkVisualization
from violet_rviz.bag_recording import recording_command


def test_environment_plots_only_valid_measurements():
    samples = {}
    node = SimpleNamespace(scalar=lambda key, value: samples.__setitem__(key, value))
    packet = [1., 3., 1., 20., 19., 3., 4., .1, .1, 1., 1., 1.]
    MavsdkVisualization.environment_cb(node, SimpleNamespace(data=packet))
    assert samples['velocity/airspeed'] == 20
    assert samples['wind/speed'] == 5
    assert samples['wind/north'] == 3
    assert samples['wind/east'] == 4
    samples.clear()
    packet[9] = packet[10] = 0
    MavsdkVisualization.environment_cb(node, SimpleNamespace(data=packet))
    assert 'velocity/airspeed' not in samples and 'wind/speed' not in samples
    samples.clear()
    MavsdkVisualization.environment_cb(node, SimpleNamespace(data=[1., 3.]))
    assert not samples


def test_speed_and_wind_graph_configuration():
    config = yaml.safe_load((Path(__file__).parents[1]/'rviz/pf_los4.rviz').read_text())
    displays = config['Visualization Manager']['Displays']
    speed = next(d for d in displays if d.get('Name') == 'Vehicle Velocity')
    series = [v for k, v in speed['Series'].items() if k != 'Series Count']
    assert speed['Series']['Series Count'] == len(series) == 4
    assert {s['Topic'].split('/')[-1] for s in series} == {'speed', 'airspeed', 'desired', 'gamma_speed'}
    wind = next(d for d in displays if d.get('Name') == 'Wind Speed')
    assert wind['Enabled'] and wind['Series']['Series Count'] == 3
    assert wind['Series']['Series 1']['Topic'].endswith('/wind/speed')


def test_environment_and_wind_topics_in_bag(tmp_path):
    command = recording_command(tmp_path, 'drone1')
    pattern = command[command.index('--regex')+1]
    for topic in ('telemetry/true_airspeed', 'telemetry/calibrated_airspeed', 'telemetry/wind',
                  'telemetry/los4/environment', 'telemetry/airspeed_valid', 'telemetry/wind_valid',
                  'telemetry/wind_enabled', 'rviz_plots/wind/speed', 'rviz_plots/velocity/airspeed'):
        assert re.match(pattern, '/drone1/' + topic)
