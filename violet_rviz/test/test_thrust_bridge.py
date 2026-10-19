"""Verify decoding, controller switching, and RViz thrust topic wiring."""
import math
from pathlib import Path

import pytest
import yaml
from std_msgs.msg import Float64MultiArray, MultiArrayDimension

from violet_rviz.plot_bridge import PlotBridge, THRUST_LABELS, _thrust_values


def message(controller, blocks, offset=0):
    msg = Float64MultiArray()
    msg.layout.data_offset = offset
    msg.layout.dim.append(MultiArrayDimension(label=controller, size=0))
    msg.data = [99.0] * offset
    for label, values in blocks:
        msg.layout.dim.append(MultiArrayDimension(label=label, size=len(values)))
        msg.data.extend(values)
    return msg


@pytest.mark.parametrize('controller, maximum', [('mellinger', 134.0417344), ('los4', 104.72)])
def test_standalone_thrust(controller, maximum):
    msg = message(controller, [('position', [1.0, 2.0, 3.0]),
                               ('Tmax', [maximum]), ('T', [12.0])], offset=2)
    assert _thrust_values(msg) == {'thrust/Tmax': maximum, 'thrust/T': 12.0}


def test_combined_motor_thrusts_and_limits():
    blocks = [('gamma', [0.2])]
    for index, label in enumerate(('T1', 'T2', 'T3', 'T4', 'Tf')):
        blocks.extend([(label, [float(index)]),
                       (label + 'max', [104.72 if label == 'Tf' else 33.51])])
    decoded = _thrust_values(message('combined', blocks))
    assert len(decoded) == 10
    assert decoded['thrust/Tf'] == 4.0
    assert decoded['thrust/Tfmax'] == 104.72
    assert decoded['thrust/T1max'] == 33.51
    assert 'thrust/T' not in decoded


def test_nonfinite_and_truncated_data():
    assert _thrust_values(message('los4', [('T', [math.nan]), ('Tmax', [math.inf])])) == {}
    msg = message('mellinger', [('T', [12.0]), ('Tmax', [134.0])])
    msg.data.pop()
    assert _thrust_values(msg) == {}


def test_switching_controller_does_not_republish_stale_thrust():
    class Harness:
        controller_results_cb = PlotBridge.controller_results_cb
        publish_tracking_values = PlotBridge.publish_tracking_values

        def __init__(self):
            self.latest_tracking_values = {f'thrust/{label}': math.nan for label in THRUST_LABELS}
            self.published = {}

        def _publish_scalar(self, key, value):
            self.published[key] = value

    bridge = Harness()
    bridge.controller_results_cb(message('mellinger', [('T', [12.0]), ('Tmax', [134.0])]))
    bridge.publish_tracking_values()
    assert bridge.published['thrust/T'] == 12.0
    bridge.published.clear()
    bridge.controller_results_cb(message('combined', [('Tf', [5.0]), ('Tfmax', [104.72])]))
    bridge.publish_tracking_values()
    assert bridge.published == {'thrust/Tf': 5.0, 'thrust/Tfmax': 104.72}


def test_rviz_series_match_bridge_topics():
    config_file = Path(__file__).resolve().parents[1] / 'rviz' / 'violet_plots.rviz'
    config = yaml.safe_load(config_file.read_text())
    displays = config['Visualization Manager']['Displays']
    thrust = next(display for display in displays if display.get('Name') == 'Thrust')
    series = thrust['Series']
    assert series['Series Count'] == len(THRUST_LABELS)
    expected = {f'/drone1/rviz_plots/thrust/{label}' for label in THRUST_LABELS}
    actual = {series[f'Series {index + 1}']['Topic'] for index in range(len(THRUST_LABELS))}
    assert actual == expected
    for index in range(len(THRUST_LABELS)):
        assert series[f'Series {index + 1}']['Unit'] == 'N'


def test_combined_attitude_errors_are_wrapped():
    from violet_rviz.plot_bridge import _attitude_error_values
    decoded = _attitude_error_values(message('combined', [('attitude_error', [0.1, -0.2, 2*math.pi+0.3])]))
    assert decoded == pytest.approx({'attitude_errors/roll': 0.1,
                                     'attitude_errors/pitch': -0.2, 'attitude_errors/yaw': 0.3})


def test_easyglider_attitude_and_heading_errors():
    from violet_rviz.plot_bridge import _attitude_error_values
    blocks = [('attitude_error_roll_pitch', [0.1, -0.2]), ('heading_error', [0.4])]
    assert _attitude_error_values(message('los4', blocks)) == pytest.approx({
        'attitude_errors/roll': 0.1, 'attitude_errors/pitch': -0.2, 'attitude_errors/yaw': 0.4})
    decoded = _attitude_error_values(message('los4', blocks[:1]))
    assert 'attitude_errors/yaw' not in decoded


def test_shuttle_attitude_errors_from_row_major_rotations():
    from violet_rviz.plot_bridge import _attitude_error_values
    import numpy as np

    def rotation(roll, pitch, yaw):
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return [cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr,
                sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, -sp, cp*sr, cp*cr]

    measured = rotation(0.1, -0.2, 3.1)
    desired = rotation(0.3, -0.1, -3.1)
    decoded = _attitude_error_values(message('mellinger', [('R', measured), ('Rd', desired)]))
    assert decoded == pytest.approx({'attitude_errors/roll': 0.2,
                                     'attitude_errors/pitch': 0.1,
                                     'attitude_errors/yaw': 2*math.pi-6.2})
    assert np.isfinite(list(decoded.values())).all()


def test_attitude_display_replaces_speed_altitude_display():
    config_file = Path(__file__).resolve().parents[1] / 'rviz' / 'violet_plots.rviz'
    displays = yaml.safe_load(config_file.read_text())['Visualization Manager']['Displays']
    assert not any(d.get('Name') == 'Speed and Altitude Tracking' for d in displays)
    display = next(d for d in displays if d.get('Name') == 'Attitude Errors')
    assert display['Layout']['Horizontal Alignment'] == 'Right'
    assert display['Layout']['Vertical Alignment'] == 'Bottom'
    assert display['Series']['Series Count'] == 3


def test_body_rates_decode_and_rviz_topics():
    from violet_rviz.plot_bridge import BODY_RATE_KEYS, _body_rate_values
    msg = message('combined', [('angular_velocity', [0.1, -0.2, 0.3]),
                               ('desired_body_rate', [0.4, -0.5, 0.6])], offset=2)
    values = _body_rate_values(msg)
    assert values['body_rates/measured/pitch'] == pytest.approx(-0.2)
    assert values['body_rates/desired/yaw'] == pytest.approx(0.6)
    assert set(values) == set(BODY_RATE_KEYS)
    config = yaml.safe_load((Path(__file__).parents[1] / 'rviz/violet_plots.rviz').read_text())
    display = next(d for d in config['Visualization Manager']['Displays']
                   if d.get('Name') == 'Body Rates')
    topics = {v['Topic'] for k, v in display['Series'].items() if k != 'Series Count'}
    assert topics == {f'/drone1/rviz_plots/{key}' for key in BODY_RATE_KEYS}
    msg.data.pop()
    assert _body_rate_values(msg) == {}
    assert _body_rate_values(message('other', [])) == {}

