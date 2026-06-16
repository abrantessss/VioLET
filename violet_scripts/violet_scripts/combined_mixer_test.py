#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import WrenchStamped
from px4_msgs.msg import ActuatorMotors, ActuatorServos
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from violet_msgs.msg import Mode, Trajectory

MIXER_NORM_THRUST = [
    [-1.0,   1.0,   1.0,  -1.0, 0.0],
    [ 1.0,  -1.0,   1.0,  -1.0, 8.2],
    [ 1.0,   1.0,  -1.0,  -1.0, 0.0],
    [ 0.0,   0.0,   0.0,   0.0, 1.0],
    [ 0.0,   0.0,   0.0,   0.0, 0.0],
    [-0.25, -0.25, -0.25, -0.25, 0.0],
]

TESTS = [
    #('A quad collective -Fz', [0.0, 0.0, 0.0, 0.0, 0.0, -1.0], 'all four quad thrust fractions and commands should be 1.0'),
    #('B fixed prop +Fx', [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], 'fixed prop thrust fraction should increase; clipping is acceptable'),
    ('C prop + quad lift', [0.0, 0.0, 0.0, 0.02, 0.0, -0.6], 'prop fraction near 0.1; q1/q3 compensate fixed-prop pitch moment'),
]


def transpose(matrix):
    return [list(row) for row in zip(*matrix)]


def matmul(a, b):
    return [
        [sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))]
        for i in range(len(a))
    ]


def matvec(a, x):
    return [sum(row[j] * x[j] for j in range(len(x))) for row in a]


def solve(matrix, rhs):
    n = len(rhs)
    aug = [matrix[i][:] + [rhs[i]] for i in range(n)]

    for col in range(n):
        pivot = max(range(col, n), key=lambda row: abs(aug[row][col]))
        if abs(aug[pivot][col]) < 1e-12:
            raise RuntimeError('singular mixer validation system')
        aug[col], aug[pivot] = aug[pivot], aug[col]

        pivot_value = aug[col][col]
        for j in range(col, n + 1):
            aug[col][j] /= pivot_value

        for row in range(n):
            if row == col:
                continue
            factor = aug[row][col]
            for j in range(col, n + 1):
                aug[row][j] -= factor * aug[col][j]

    return [aug[i][n] for i in range(n)]


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def fmt(values):
    return '[' + ', '.join(f'{value:+.3f}' for value in values) + ']'


class CombinedMixerTestNode(Node):
    def __init__(self):
        super().__init__('combined_mixer_test_node')
        self.declare_parameter('shuttle_ns', 'drone1')
        self.declare_parameter('fixed_wing_ns', 'drone2')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('arm_seconds', 2.0)
        self.declare_parameter('prime_seconds', 5.0)
        self.declare_parameter('test_seconds', 50.0)
        self.declare_parameter('damping', 0.01)
        self.declare_parameter('motor_min', -1.0)
        self.declare_parameter('motor_max', 1.0)
        self.declare_parameter('servo_min', -1.0)
        self.declare_parameter('servo_max', 1.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_follow', True)

        self.shuttle_ns = self.get_parameter('shuttle_ns').value.strip('/')
        self.fixed_wing_ns = self.get_parameter('fixed_wing_ns').value.strip('/')
        self.publish_rate_hz = self.get_parameter('publish_rate_hz').value
        self.arm_seconds = self.get_parameter('arm_seconds').value
        self.prime_seconds = self.get_parameter('prime_seconds').value
        self.test_seconds = self.get_parameter('test_seconds').value
        self.damping = self.get_parameter('damping').value
        self.motor_min = self.get_parameter('motor_min').value
        self.motor_max = self.get_parameter('motor_max').value
        self.servo_min = self.get_parameter('servo_min').value
        self.servo_max = self.get_parameter('servo_max').value
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_follow = self.get_parameter('auto_follow').value

        self.latest_shuttle_motors = None
        self.latest_fixed_wing_motors = None
        self.latest_fixed_wing_servos = None

        self.arm_pubs = [
            self.create_publisher(Mode, f'/{self.shuttle_ns}/fmu/mode/arm', qos_profile_sensor_data),
            self.create_publisher(Mode, f'/{self.fixed_wing_ns}/fmu/mode/arm', qos_profile_sensor_data),
        ]
        self.follow_pub = self.create_publisher(
            Trajectory,
            f'/{self.shuttle_ns}/fmu/mode/follow',
            qos_profile_sensor_data,
        )
        self.wrench_pub = self.create_publisher(
            WrenchStamped,
            f'/{self.shuttle_ns}/fmu/in/combined_wrench',
            qos_profile_sensor_data,
        )

        self.create_subscription(
            ActuatorMotors,
            f'/{self.shuttle_ns}/fmu/in/actuator_motors',
            self.on_shuttle_motors,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ActuatorMotors,
            f'/{self.fixed_wing_ns}/fmu/in/actuator_motors',
            self.on_fixed_wing_motors,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ActuatorServos,
            f'/{self.fixed_wing_ns}/fmu/in/actuator_servos',
            self.on_fixed_wing_servos,
            qos_profile_sensor_data,
        )

    def on_shuttle_motors(self, msg):
        self.latest_shuttle_motors = list(msg.control[:4])

    def on_fixed_wing_motors(self, msg):
        self.latest_fixed_wing_motors = [msg.control[0]]

    def on_fixed_wing_servos(self, msg):
        self.latest_fixed_wing_servos = list(msg.control[:4])

    def expected_thrust_fractions_raw(self, wrench):
        mixer_t = transpose(MIXER_NORM_THRUST)
        normal_matrix = matmul(mixer_t, MIXER_NORM_THRUST)
        normal_rhs = matvec(mixer_t, wrench)
        return solve(normal_matrix, normal_rhs)

    def expected_published_actuators(self, wrench):
        s_raw = self.expected_thrust_fractions_raw(wrench)
        s = [clamp(value, 0.0, 1.0) for value in s_raw]
        u = [math.sqrt(value) for value in s]
        shuttle_motors = [clamp(value, self.motor_min, self.motor_max) for value in u[:4]]
        fixed_wing_motor = [clamp(u[4], 0.0, self.motor_max)]
        fixed_wing_servos = [0.0, 0.0, 0.0, 0.0]
        return s_raw, s, u, shuttle_motors, fixed_wing_motor, fixed_wing_servos

    def publish_arm(self):
        msg = Mode()
        for pub in self.arm_pubs:
            pub.publish(msg)

    def publish_follow_trigger(self):
        msg = Trajectory()
        msg.path_type = 0
        self.follow_pub.publish(msg)

    def publish_wrench(self, wrench):
        msg = WrenchStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'combined_body'
        msg.wrench.torque.x = wrench[0]
        msg.wrench.torque.y = wrench[1]
        msg.wrench.torque.z = wrench[2]
        msg.wrench.force.x = wrench[3]
        msg.wrench.force.y = wrench[4]
        msg.wrench.force.z = wrench[5]
        self.wrench_pub.publish(msg)

    def stream_wrench(self, wrench, seconds):
        period = 1.0 / max(self.publish_rate_hz, 1.0)
        end_time = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_wrench(wrench)
            if self.auto_follow:
                self.publish_follow_trigger()
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(period)

    def log_result(self, label, wrench, note):
        s_raw, s, u, shuttle_motors, fixed_wing_motor, fixed_wing_servos = (
            self.expected_published_actuators(wrench)
        )
        achieved = matvec(MIXER_NORM_THRUST, s)
        error = [wrench[i] - achieved[i] for i in range(6)]
        clipped = any(abs(s_raw[i] - s[i]) > 1e-9 for i in range(len(s)))

        self.get_logger().info(f'Test: {label} - {note}')
        self.get_logger().info(f'  command wrench [Mx My Mz Fx Fy Fz] = {fmt(wrench)}')
        self.get_logger().info(f'  expected raw thrust fractions = {fmt(s_raw)}')
        self.get_logger().info(f'  expected clipped thrust fractions = {fmt(s)}')
        if clipped:
            self.get_logger().info('  clipping applied to thrust fractions')
        self.get_logger().info(f'  expected actuator commands sqrt(s) = {fmt(u)}')
        self.get_logger().info(f'  achieved B*s = {fmt(achieved)}, residual = {fmt(error)}')
        self.get_logger().info(f'  expected shuttle motors = {fmt(shuttle_motors)}')
        self.get_logger().info(f'  observed shuttle motors = {fmt(self.latest_shuttle_motors or [math.nan] * 4)}')
        self.get_logger().info(f'  expected fixed prop = {fmt(fixed_wing_motor)}')
        self.get_logger().info(f'  observed fixed prop = {fmt(self.latest_fixed_wing_motors or [math.nan])}')
        self.get_logger().info(f'  expected servos [disabled] = {fmt(fixed_wing_servos)}')
        self.get_logger().info(f'  observed servos = {fmt(self.latest_fixed_wing_servos or [math.nan] * 4)}')

    def run(self):
        if self.auto_arm:
            self.get_logger().info(f'Arming /{self.shuttle_ns} and /{self.fixed_wing_ns}')
            end_time = time.monotonic() + self.arm_seconds
            while rclpy.ok() and time.monotonic() < end_time:
                self.publish_arm()
                rclpy.spin_once(self, timeout_sec=0.05)
                time.sleep(0.1)

        self.get_logger().info('Priming combined direct controller with zero wrench')
        self.stream_wrench([0.0] * 6, self.prime_seconds)

        self.get_logger().info('Starting axis validation. Watch the vehicle response for the note on each test.')
        for label, wrench, note in TESTS:
            self.stream_wrench(wrench, self.test_seconds)
            self.log_result(label, wrench, note)
            self.stream_wrench([0.0] * 6, 2.0)


def main(args=None):
    rclpy.init(args=args)
    node = CombinedMixerTestNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
