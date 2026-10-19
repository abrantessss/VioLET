"""Exercise actual bridge, RViz topics and SQLite recording with fake PX4."""
import os
import signal
import socket
import sqlite3
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
from visualization_msgs.msg import Marker
from test_controller_link import FakePx4
from violet_rviz.bag_recording import recording_command
from violet_rviz.mavsdk_visualization import MavsdkVisualization


def main(controller, bridge, bag_root):
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0))
    port = reserve.getsockname()[1]
    reserve.close()
    bridge_log = tempfile.TemporaryFile()
    receiver = subprocess.Popen([bridge, '--ros-args', '-r', '__ns:=/drone1',
                    '-p', f'connection_url:=udp://:{port}'], stdout=bridge_log, stderr=subprocess.STDOUT)
    command = recording_command(bag_root, 'drone1')
    bag = Path(command[-1])
    bag_log = tempfile.TemporaryFile()
    recorder = subprocess.Popen(command, stdout=bag_log, stderr=subprocess.STDOUT)
    rclpy.init(args=['--ros-args', '-r', '__ns:=/drone1'])
    plots = MavsdkVisualization()
    observer = rclpy.create_node('observer')
    executor = SingleThreadedExecutor()
    executor.add_node(plots)
    executor.add_node(observer)
    modes, actuators = [], {}
    handles = [observer.create_subscription(Marker, '/drone1/rviz_plots/vehicle_mode',
                                            lambda msg: modes.append(msg.text), 10)]
    for name in ('motor', 'elevator', 'rudder', 'right_aileron', 'left_aileron'):
        handles.append(observer.create_subscription(Float32, '/drone1/rviz_plots/actuators/'+name,
                       lambda msg, key=name: actuators.update({key: msg.data}), 10))
    fake = FakePx4(controller, [f'udp://127.0.0.1:{port}', '--test-gamma', 'true'])
    try:
        end = time.monotonic()+12
        while time.monotonic()<end and (len(actuators)<5 or not any('Mode: MANUAL' in x for x in modes)):
            executor.spin_once(timeout_sec=0.05)
        assert len(actuators)==5, f'Actuator mapping did not arrive: {actuators}'
        expected = {'motor':0.6, 'elevator':-0.2, 'rudder':-0.4, 'right_aileron':-0.6, 'left_aileron':-0.8}
        assert all(abs(actuators[name]-value)<1e-5 for name,value in expected.items()), actuators
        assert any('Mode: MANUAL' in x and 'Armed: YES' in x and 'Kill CH6: OFF' in x for x in modes)
        fake.mode=6
        fake.kill_pwm=1900
        end = time.monotonic()+3
        while time.monotonic()<end:
            executor.spin_once(timeout_sec=0.05)
        assert any('Mode: OFFBOARD' in x and 'Kill CH6: ACTIVE' in x for x in modes), modes[-3:]
        assert not fake.controls(), 'Test emitted flight commands'
    finally:
        fake.close()
        receiver.send_signal(signal.SIGINT)
        receiver.wait(timeout=6)
        recorder.send_signal(signal.SIGINT)
        recorder.wait(timeout=15)
        executor.shutdown()
        observer.destroy_node()
        plots.destroy_node()
        rclpy.shutdown()
    assert recorder.returncode==0, 'Recorder failed'
    assert (bag/'metadata.yaml').exists(), 'Bag metadata missing'
    connection = sqlite3.connect(next(bag.glob('*.db3')))
    counts = dict(connection.execute('SELECT topics.name, count(messages.id) FROM topics JOIN messages ON topics.id=messages.topic_id GROUP BY topics.id'))
    connection.close()
    for topic in ('telemetry/flight_mode','telemetry/armed','telemetry/kill_active','telemetry/rc_channels',
                  'telemetry/los4/state','rviz_plots/actuators/motor','rviz_plots/path_error/norm'):
        assert counts.get('/drone1/'+topic,0)>0, f'Bag missing {topic}: {counts}'
    print('Actual ROS mode/kill display, all 5 actuator mappings and SQLite bag recording passed')
    print(bag)


if __name__ == '__main__':
    main(*sys.argv[1:])
