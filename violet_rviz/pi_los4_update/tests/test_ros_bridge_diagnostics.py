"""Check Pi TUNNEL -> actual ROS bridge -> diagnostic topics using fake PX4."""
import signal
import socket
import subprocess
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from std_msgs.msg import Float32MultiArray
from test_controller_link import FakePx4


def main(controller, bridge):
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0))
    port = reserve.getsockname()[1]
    reserve.close()
    process = subprocess.Popen([bridge, '--ros-args', '-r', '__ns:=/los4_test',
                                '-p', f'connection_url:=udp://:{port}', '-p', 'telemetry_rate_hz:=20.0'],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    rclpy.init()
    node = rclpy.create_node('diagnostic_test')
    configs, states = [], []
    config_sub = node.create_subscription(Float32MultiArray, '/los4_test/telemetry/los4/config',
                    lambda msg: configs.append(list(msg.data)),
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    state_sub = node.create_subscription(Float32MultiArray, '/los4_test/telemetry/los4/state',
                    lambda msg: states.append(list(msg.data)), qos_profile_sensor_data)
    fake = FakePx4(controller, [f'udp://127.0.0.1:{port}', '--test-gamma', 'true'])
    try:
        end = time.monotonic()+10
        while time.monotonic()<end and (not configs or len(states)<5):
            rclpy.spin_once(node, timeout_sec=0.1)
        assert configs and len(configs[-1])==15, 'Configuration did not reach ROS'
        assert len(states)>=5 and len(states[-1])==25, 'State did not reach ROS'
        assert states[-1][4]==1, 'Gamma test velocity not preserved'
        assert states[-1][3]>states[0][3], 'Gamma does not advance'
        assert not fake.controls(), 'Diagnostic test emitted flight commands'
        print('Pi -> MAVLink forwarding -> compiled ROS bridge -> config/state topics passed')
    finally:
        fake.close()
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=5)
        node.destroy_node()
        rclpy.shutdown()
        if process.returncode != 0:
            raise AssertionError(output.decode())


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
