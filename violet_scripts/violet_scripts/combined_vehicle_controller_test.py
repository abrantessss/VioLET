#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from px4_msgs.msg import ActuatorMotors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64

from violet_msgs.msg import Mode, State, Trajectory


# Test configuration. Change these values directly before running the script.
SHUTTLE_NS = 'drone1'
FIXED_WING_NS = 'drone2'
VA = 0.5
PUBLISH_RATE_HZ = 20.0
ARM_SECONDS = 2.0
PRIME_SECONDS = 3.0
TEST_SECONDS = 60.0
HEADING_ELEVATION_DEGREES = 60.0
HEADING_YAW_DEGREES = 170.0
AUTO_ARM = True
AUTO_FOLLOW = True
SHOW_PLOT = True
PLOT_FILE = ''


def fmt(values):
    return '[' + ', '.join(f'{value:+.3f}' for value in values) + ']'


class CombinedVehicleControllerTestNode(Node):
    def __init__(self):
        super().__init__('combined_vehicle_controller_test_node')

        self.shuttle_ns = SHUTTLE_NS.strip('/')
        self.fixed_wing_ns = FIXED_WING_NS.strip('/')
        self.va = float(VA)
        self.publish_rate_hz = float(PUBLISH_RATE_HZ)
        self.arm_seconds = float(ARM_SECONDS)
        self.prime_seconds = float(PRIME_SECONDS)
        self.test_seconds = float(TEST_SECONDS)
        elevation = math.radians(float(HEADING_ELEVATION_DEGREES))
        yaw = math.radians(float(HEADING_YAW_DEGREES))
        horizontal = math.cos(elevation)
        # NED uses positive z downward, so positive elevation means climbing.
        self.heading = [
            horizontal * math.cos(yaw),
            horizontal * math.sin(yaw),
            -math.sin(elevation),
        ]
        self.auto_arm = bool(AUTO_ARM)
        self.auto_follow = bool(AUTO_FOLLOW)
        self.show_plot = bool(SHOW_PLOT)
        self.plot_file = str(PLOT_FILE)

        if self.va < 0.0 or not math.isfinite(self.va):
            raise ValueError('VA must be finite and non-negative')
        if self.publish_rate_hz <= 0.0:
            raise ValueError('publish_rate_hz must be positive')

        self.latest_attitude = [math.nan] * 3
        self.latest_velocity = [math.nan] * 3
        self.latest_wrench = [math.nan] * 5
        self.latest_shuttle_motors = [math.nan] * 4
        self.latest_fixed_wing_motor = [math.nan]
        self.samples = []
        self.sample_start_time = None

        self.arm_pubs = [
            self.create_publisher(
                Mode, f'/{self.shuttle_ns}/fmu/mode/arm', qos_profile_sensor_data),
            self.create_publisher(
                Mode, f'/{self.fixed_wing_ns}/fmu/mode/arm', qos_profile_sensor_data),
        ]
        self.follow_pub = self.create_publisher(
            Trajectory,
            f'/{self.shuttle_ns}/fmu/mode/follow',
            qos_profile_sensor_data,
        )
        self.heading_pub = self.create_publisher(
            Vector3Stamped,
            f'/{self.shuttle_ns}/controller/in/heading_setpoint',
            qos_profile_sensor_data,
        )
        self.airspeed_pub = self.create_publisher(
            Float64,
            f'/{self.shuttle_ns}/controller/in/airspeed_setpoint',
            qos_profile_sensor_data,
        )

        self.create_subscription(
            State,
            f'/{self.shuttle_ns}/fmu/telemetry/state',
            self.on_state,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            WrenchStamped,
            f'/{self.shuttle_ns}/fmu/in/combined_wrench',
            self.on_wrench,
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
            self.on_fixed_wing_motor,
            qos_profile_sensor_data,
        )

    def on_state(self, msg):
        self.latest_attitude = [float(value) for value in msg.attitude]
        self.latest_velocity = [float(value) for value in msg.inertial_velocity]

    def on_wrench(self, msg):
        self.latest_wrench = [
            float(msg.wrench.torque.x),
            float(msg.wrench.torque.y),
            float(msg.wrench.torque.z),
            float(msg.wrench.force.x),
            float(msg.wrench.force.z),
        ]

    def on_shuttle_motors(self, msg):
        self.latest_shuttle_motors = list(msg.control[:4])

    def on_fixed_wing_motor(self, msg):
        self.latest_fixed_wing_motor = [msg.control[0]]

    def publish_arm(self):
        msg = Mode()
        for publisher in self.arm_pubs:
            publisher.publish(msg)

    def publish_follow_trigger(self):
        msg = Trajectory()
        msg.path_type = 0
        self.follow_pub.publish(msg)

    def publish_guidance(self, heading):
        heading_msg = Vector3Stamped()
        heading_msg.header.stamp = self.get_clock().now().to_msg()
        heading_msg.header.frame_id = 'ned'
        heading_msg.vector.x = float(heading[0])
        heading_msg.vector.y = float(heading[1])
        heading_msg.vector.z = float(heading[2])
        self.heading_pub.publish(heading_msg)

        airspeed_msg = Float64()
        airspeed_msg.data = self.va
        self.airspeed_pub.publish(airspeed_msg)

        if self.auto_follow:
            self.publish_follow_trigger()

        self.record_sample(heading)

    def record_sample(self, heading):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.sample_start_time is None:
            self.sample_start_time = now

        attitude_sp = [0.0, 0.0, math.atan2(heading[1], heading[0])]
        self.samples.append({
            'time': now - self.sample_start_time,
            'attitude_sp': attitude_sp,
            'attitude': self.latest_attitude[:],
            'thrust': [self.latest_wrench[3], -self.latest_wrench[4]],
        })

    def spin_period(self):
        rclpy.spin_once(self, timeout_sec=0.0)
        time.sleep(1.0 / self.publish_rate_hz)

    def stream_heading(self, heading, seconds):
        end_time = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end_time:
            self.publish_guidance(heading)
            self.spin_period()

    def log_state(self, label):
        self.get_logger().info(f'{label} complete')
        self.get_logger().info(
            f'  attitude [roll pitch yaw] rad = {fmt(self.latest_attitude)}')
        self.get_logger().info(f'  inertial velocity [x y z] m/s = {fmt(self.latest_velocity)}')
        self.get_logger().info(f'  wrench [Mx My Mz Fx Fz] = {fmt(self.latest_wrench)}')
        self.get_logger().info(f'  shuttle motors = {fmt(self.latest_shuttle_motors)}')
        self.get_logger().info(f'  fixed-wing motor = {fmt(self.latest_fixed_wing_motor)}')

    def plot_results(self):
        if not self.samples:
            self.get_logger().warn('No samples were recorded; skipping plot')
            return

        try:
            import matplotlib.pyplot as plt
        except ImportError:
            self.get_logger().error('matplotlib is unavailable; cannot plot test results')
            return

        times = [sample['time'] for sample in self.samples]
        labels = ['roll', 'pitch', 'yaw']
        figure, axes = plt.subplots(4, 1, sharex=True, figsize=(12, 11))

        for axis, label in enumerate(labels):
            commanded = [sample['attitude_sp'][axis] for sample in self.samples]
            measured = [sample['attitude'][axis] for sample in self.samples]
            axes[axis].plot(times, commanded, '--', label=f'{label} command')
            axes[axis].plot(times, measured, label=f'{label} measured')
            axes[axis].set_ylabel(f'{label} [rad]')
            axes[axis].grid(True)
            axes[axis].legend(loc='upper right')

        tx = [sample['thrust'][0] for sample in self.samples]
        tz = [sample['thrust'][1] for sample in self.samples]
        axes[3].plot(times, tx, label='Tx command')
        axes[3].plot(times, tz, label='Tz command')
        axes[3].set_ylabel('thrust [N]')
        axes[3].set_xlabel('time [s]')
        axes[3].grid(True)
        axes[3].legend(loc='upper right')

        figure.suptitle(
            'Combined vehicle controller test — '
            f'Va={self.va:.2f} m/s, '
            f'elevation={HEADING_ELEVATION_DEGREES:.1f} deg, '
            f'yaw={HEADING_YAW_DEGREES:.1f} deg')
        figure.tight_layout()

        if self.plot_file:
            figure.savefig(self.plot_file, dpi=150)
            self.get_logger().info(f'Saved test plot to {self.plot_file}')
        if self.show_plot:
            plt.show()
        else:
            plt.close(figure)

    def run(self):
        if self.auto_arm:
            self.get_logger().info(
                f'Arming /{self.shuttle_ns} and /{self.fixed_wing_ns}')
            end_time = time.monotonic() + self.arm_seconds
            while rclpy.ok() and time.monotonic() < end_time:
                self.publish_arm()
                self.spin_period()

        self.get_logger().info(
            f'Priming with h={fmt(self.heading)}, Va={self.va:.2f} m/s')
        self.stream_heading(self.heading, self.prime_seconds)

        self.get_logger().info(
            f'Holding one heading for {self.test_seconds:.1f} s: '
            f'h={fmt(self.heading)}, Va={self.va:.2f} m/s')
        self.stream_heading(self.heading, self.test_seconds)
        self.log_state('Combined vehicle controller test')
        self.plot_results()


def main(args=None):
    rclpy.init(args=args)
    node = CombinedVehicleControllerTestNode()
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
