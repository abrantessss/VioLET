#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Vector3Stamped, WrenchStamped
from px4_msgs.msg import ActuatorMotors, ActuatorServos, VehicleRatesSetpoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from violet_msgs.msg import Mode, State, Trajectory


AXES = {'roll': 0, 'pitch': 1, 'yaw': 2}

TESTS = [
    ('A roll +rate', [0.6, 0.0, 0.0], 'positive roll rate should create positive roll torque'),
    ('B pitch +rate', [0.0, 0.6, 0.0], 'positive pitch rate should create positive pitch torque'),
    ('C yaw +rate', [0.0, 0.0, 0.6], 'positive yaw rate should create positive yaw torque'),
    #('D combined +rates', [0.6, 0.6, 0.6], 'multi-axis step should publish all requested torque axes'),
]


def clamp(value, minimum, maximum):
    if not math.isfinite(value):
        return value
    return max(minimum, min(maximum, value))


def fmt(values):
    return '[' + ', '.join(f'{value:+.3f}' for value in values) + ']'


def fmt_metric(value, unit='s'):
    if value is None or not math.isfinite(value):
        return 'n/a'
    if unit == '%':
        return f'{value:.1f}%'
    if unit:
        return f'{value:.3f} {unit}'
    return f'{value:.2f}'


class CombinedRateControllerTestNode(Node):
    def __init__(self):
        super().__init__('combined_rate_controller_test_node')
        self.declare_parameter('shuttle_ns', 'drone1')
        self.declare_parameter('fixed_wing_ns', 'drone2')
        self.declare_parameter('test_seconds', 30.0)
        self.declare_parameter('settling_time', 1.0)
        self.declare_parameter('tolerance', 0.05)
        self.declare_parameter('force_setpoint',[1.0, 0.0, -6.0])
        self.declare_parameter('gains.roll.kp', 0.25)
        self.declare_parameter('gains.pitch.kp', 0.25)
        self.declare_parameter('gains.yaw.kp', 0.10)
        self.declare_parameter('output_limits.roll', [-2.0, 2.0])
        self.declare_parameter('output_limits.pitch', [-2.0, 2.0])
        self.declare_parameter('output_limits.yaw', [-1.0, 1.0])
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('auto_follow', True)
        self.declare_parameter('show_plot', True)
        self.declare_parameter('live_plot', False)

        self.shuttle_ns = self.get_parameter('shuttle_ns').value.strip('/')
        self.fixed_wing_ns = self.get_parameter('fixed_wing_ns').value.strip('/')
        self.test_seconds = self.get_parameter('test_seconds').value
        self.settling_time = self.get_parameter('settling_time').value
        self.tolerance = self.get_parameter('tolerance').value
        self.force_setpoint = list(self.get_parameter('force_setpoint').value)
        self.gains = [
            self.get_parameter('gains.roll.kp').value,
            self.get_parameter('gains.pitch.kp').value,
            self.get_parameter('gains.yaw.kp').value,
        ]
        self.output_limits = [
            list(self.get_parameter('output_limits.roll').value),
            list(self.get_parameter('output_limits.pitch').value),
            list(self.get_parameter('output_limits.yaw').value),
        ]
        self.auto_arm = self.get_parameter('auto_arm').value
        self.auto_follow = self.get_parameter('auto_follow').value
        self.show_plot = self.get_parameter('show_plot').value
        self.live_plot = self.get_parameter('live_plot').value

        if len(self.force_setpoint) != 3:
            self.get_logger().warn('force_setpoint must have three entries; using zeros')
            self.force_setpoint = [0.0, 0.0, 0.0]
        for axis, limits in enumerate(self.output_limits):
            if len(limits) != 2:
                self.get_logger().warn(f'output limit axis {axis} must have two entries; using [-1, 1]')
                self.output_limits[axis] = [-1.0, 1.0]
            elif limits[0] > limits[1]:
                self.output_limits[axis] = [limits[1], limits[0]]

        self.latest_omega = [math.nan, math.nan, math.nan]
        self.latest_wrench = [math.nan, math.nan, math.nan, math.nan, math.nan]
        self.latest_shuttle_motors = None
        self.latest_fixed_wing_motors = None
        self.latest_fixed_wing_servos = None
        self.samples = []
        self.plt = None
        self.plot_fig = None
        self.plot_axes = None
        self.plot_lines = None

        self.arm_pubs = [
            self.create_publisher(Mode, f'/{self.shuttle_ns}/fmu/mode/arm', qos_profile_sensor_data),
            self.create_publisher(Mode, f'/{self.fixed_wing_ns}/fmu/mode/arm', qos_profile_sensor_data),
        ]
        self.follow_pub = self.create_publisher(
            Trajectory,
            f'/{self.shuttle_ns}/fmu/mode/follow',
            qos_profile_sensor_data,
        )
        self.rate_pub = self.create_publisher(
            VehicleRatesSetpoint,
            f'/{self.shuttle_ns}/controller/in/rate_setpoint',
            qos_profile_sensor_data,
        )
        self.force_pub = self.create_publisher(
            Vector3Stamped,
            f'/{self.shuttle_ns}/controller/in/force_setpoint',
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
            self.on_fixed_wing_motors,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            ActuatorServos,
            f'/{self.fixed_wing_ns}/fmu/in/actuator_servos',
            self.on_fixed_wing_servos,
            qos_profile_sensor_data,
        )

    def on_state(self, msg):
        self.latest_omega = [
            float(msg.angular_velocity[0]),
            float(msg.angular_velocity[1]),
            float(msg.angular_velocity[2]),
        ]

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

    def on_fixed_wing_motors(self, msg):
        self.latest_fixed_wing_motors = [msg.control[0]]

    def on_fixed_wing_servos(self, msg):
        self.latest_fixed_wing_servos = list(msg.control[:4])

    def expected_torque(self, sp, omega=None):
        omega = self.latest_omega if omega is None else omega
        torque = []
        for axis in range(3):
            if math.isfinite(omega[axis]):
                raw = self.gains[axis] * (sp[axis] - omega[axis])
            else:
                raw = math.nan
            torque.append(clamp(raw, self.output_limits[axis][0], self.output_limits[axis][1]))
        return torque

    def publish_arm(self):
        msg = Mode()
        for pub in self.arm_pubs:
            pub.publish(msg)

    def publish_follow_trigger(self):
        msg = Trajectory()
        msg.path_type = 0
        self.follow_pub.publish(msg)

    def publish_force_setpoint(self):
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'combined_body'
        msg.vector.x = float(self.force_setpoint[0])
        msg.vector.y = float(self.force_setpoint[1])
        msg.vector.z = float(self.force_setpoint[2])
        self.force_pub.publish(msg)

    def publish_rate_setpoint(self, sp, reset_integral=False):
        msg = VehicleRatesSetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.roll = float(sp[0])
        msg.pitch = float(sp[1])
        msg.yaw = float(sp[2])
        msg.thrust_body = [0.0, 0.0, 0.0]
        msg.reset_integral = reset_integral
        self.rate_pub.publish(msg)

    def publish_allocator_setpoint(self, sp, reset_integral=False):
        self.publish_rate_setpoint(sp, reset_integral)
        self.publish_force_setpoint()
        if self.auto_follow:
            self.publish_follow_trigger()

    def record_sample(self, sp, label, force=False):
        sample = (
            self.get_clock().now().nanoseconds * 1e-9,
            label,
            sp[:],
            self.latest_omega[:],
            self.expected_torque(sp),
            self.latest_wrench[:],
        )

        if not force and self.samples:
            previous = self.samples[-1]
            unchanged = (
                previous[1] == sample[1] and
                previous[2] == sample[2] and
                previous[3] == sample[3] and
                previous[5] == sample[5])
            if unchanged:
                return

        self.samples.append(sample)
        if self.live_plot:
            self.update_live_plot()

    def stream_rate_setpoint(self, sp, seconds, label='segment', reset_integral=False):
        end_time = time.monotonic() + seconds
        first = True
        while rclpy.ok() and time.monotonic() < end_time:
            if self.auto_follow:
                self.publish_allocator_setpoint(sp, reset_integral and first)
            else:
                self.publish_rate_setpoint(sp, reset_integral and first)
                self.publish_force_setpoint()
            rclpy.spin_once(self, timeout_sec=0.0)

            self.record_sample(sp, label, force=first)
            first = False
        self.record_sample(sp, label, force=True)

    def log_result(self, label, sp, note):
        expected = self.expected_torque(sp)
        torque_error = [
            expected[i] - self.latest_wrench[i]
            if math.isfinite(expected[i]) and math.isfinite(self.latest_wrench[i])
            else math.nan
            for i in range(3)
        ]

        self.get_logger().info(f'Test: {label} - {note}')
        self.get_logger().info(f'  command omega_sp [roll pitch yaw] rad/s = {fmt(sp)}')
        self.get_logger().info(f'  observed omega [roll pitch yaw] rad/s = {fmt(self.latest_omega)}')
        self.get_logger().info(f'  expected torque from P gains = {fmt(expected)}')
        self.get_logger().info(f'  observed wrench [Mx My Mz Fx Fz] = {fmt(self.latest_wrench)}')
        self.get_logger().info(f'  torque residual expected-observed = {fmt(torque_error)}')
        self.get_logger().info(f'  observed shuttle motors = {fmt(self.latest_shuttle_motors or [math.nan] * 4)}')
        self.get_logger().info(f'  observed fixed prop = {fmt(self.latest_fixed_wing_motors or [math.nan])}')
        self.get_logger().info(f'  observed servos [disabled] = {fmt(self.latest_fixed_wing_servos or [math.nan] * 4)}')

    def load_matplotlib(self):
        if not self.show_plot:
            return False
        if self.plt is not None:
            return True
        try:
            import matplotlib.pyplot as plt
        except (AttributeError, ImportError) as error:
            self.get_logger().error(
                'Could not import matplotlib for plotting. Your Python is likely mixing '
                'NumPy 2.x from ~/.local with an older system Matplotlib built for NumPy 1.x. '
                f'Original error: {error}')
            self.show_plot = False
            return False
        self.plt = plt
        return True

    def sample_series(self):
        if not self.samples:
            return [], [], [], [], []
        t0 = self.samples[0][0]
        return (
            [row[0] - t0 for row in self.samples],
            [row[1] for row in self.samples],
            [row[2] for row in self.samples],
            [row[3] for row in self.samples],
            [row[5][:3] for row in self.samples],
        )

    def crossing_time(self, times, values, threshold, direction):
        for index, value in enumerate(values):
            if not math.isfinite(value):
                continue
            crossed = value >= threshold if direction > 0.0 else value <= threshold
            if not crossed:
                continue
            if index == 0:
                return times[0]

            previous_value = values[index - 1]
            previous_time = times[index - 1]
            if not math.isfinite(previous_value) or value == previous_value:
                return times[index]

            fraction = (threshold - previous_value) / (value - previous_value)
            fraction = clamp(fraction, 0.0, 1.0)
            return previous_time + fraction * (times[index] - previous_time)
        return None

    def step_response_metrics(self, label, axis):
        segment_samples = [
            row for row in self.samples
            if row[1] == label and math.isfinite(row[3][axis])
        ]
        if len(segment_samples) < 2:
            return None

        start_time = segment_samples[0][0]
        times = [row[0] - start_time for row in segment_samples]
        values = [row[3][axis] for row in segment_samples]
        initial = values[0]
        final = segment_samples[-1][2][axis]
        step_size = final - initial
        if abs(step_size) < 1e-9:
            return None

        direction = 1.0 if step_size > 0.0 else -1.0
        beyond_final = [direction * (value - final) for value in values if math.isfinite(value)]
        overshoot = 100.0 * max(0.0, max(beyond_final)) / abs(step_size)

        rise_start = self.crossing_time(times, values, initial + 0.1 * step_size, direction)
        rise_end = self.crossing_time(times, values, initial + 0.9 * step_size, direction)
        rise_time = (
            rise_end - rise_start
            if rise_start is not None and rise_end is not None and rise_end >= rise_start
            else None
        )

        settle_time = None
        for index, _ in enumerate(segment_samples):
            remaining_values = values[index:]
            if all(abs(value - final) <= self.tolerance for value in remaining_values):
                settle_time = times[index]
                break

        return {
            'overshoot': overshoot,
            'rise_time': rise_time,
            'settle_time': settle_time,
        }

    def log_step_response_metrics(self):
        axis_names = ('roll', 'pitch', 'yaw')
        self.get_logger().info(
            f'Step response metrics by rate controller (settle band: +/-{self.tolerance:.3f} rad/s)')
        for label, sp, _ in TESTS:
            active_axes = [axis for axis, value in enumerate(sp) if abs(value) > 1e-9]
            if not active_axes:
                continue

            self.get_logger().info(f'  {label}:')
            for axis in active_axes:
                metrics = self.step_response_metrics(f'{label} step', axis)
                if metrics is None:
                    self.get_logger().info(f'    {axis_names[axis]}: insufficient data')
                    continue

                self.get_logger().info(
                    f'    {axis_names[axis]}: '
                    f'overshoot={fmt_metric(metrics["overshoot"], "%")}, '
                    f'rise_time={fmt_metric(metrics["rise_time"])}, '
                    f'settle_time={fmt_metric(metrics["settle_time"])}')

    def setup_live_plot(self):
        if not self.live_plot or self.plot_fig is not None or not self.load_matplotlib():
            return

        self.plt.ion()
        self.plot_fig, self.plot_axes = self.plt.subplots(4, 1, sharex=True, figsize=(12, 10))
        axis_names = ('roll', 'pitch', 'yaw')
        self.plot_lines = {'sp': [], 'omega': [], 'torque': []}

        for axis, name in enumerate(axis_names):
            sp_line, = self.plot_axes[axis].step([], [], where='post', label=f'{name} step')
            omega_line, = self.plot_axes[axis].plot([], [], label=f'{name} measured')
            self.plot_lines['sp'].append(sp_line)
            self.plot_lines['omega'].append(omega_line)
            self.plot_axes[axis].set_ylabel('rad/s')
            self.plot_axes[axis].grid(True, alpha=0.3)
            self.plot_axes[axis].legend(loc='upper right')

        for name in axis_names:
            torque_line, = self.plot_axes[3].plot([], [], label=f'{name} torque')
            self.plot_lines['torque'].append(torque_line)
        self.plot_axes[3].set_ylabel('Nm')
        self.plot_axes[3].set_xlabel('time [s]')
        self.plot_axes[3].grid(True, alpha=0.3)
        self.plot_axes[3].legend(loc='upper right')
        self.plot_fig.suptitle('Combined rate controller step response')
        self.plot_fig.tight_layout()
        self.plot_fig.show()

    def update_live_plot(self, force=False):
        if not self.live_plot or self.plot_fig is None or not self.samples:
            return

        time_s, _, setpoints, omegas, torques = self.sample_series()
        for axis in range(3):
            self.plot_lines['sp'][axis].set_data(time_s, [sp[axis] for sp in setpoints])
            self.plot_lines['omega'][axis].set_data(time_s, [omega[axis] for omega in omegas])
            self.plot_lines['torque'][axis].set_data(time_s, [torque[axis] for torque in torques])

        for axis in self.plot_axes:
            axis.relim()
            axis.autoscale_view()
        self.plot_fig.canvas.draw()
        self.plot_fig.canvas.flush_events()

    def plot_samples(self):
        if not self.show_plot or not self.samples or not self.load_matplotlib():
            return

        if self.plot_fig is not None:
            self.update_live_plot(force=True)
            self.plt.ioff()
            self.plt.show()
            return

        time_s, labels, setpoints, omegas, torques = self.sample_series()
        fig, axes = self.plt.subplots(4, 1, sharex=True, figsize=(12, 10))
        axis_names = ('roll', 'pitch', 'yaw')

        for axis, name in enumerate(axis_names):
            sp_values = [sp[axis] for sp in setpoints]
            omega_values = [omega[axis] for omega in omegas]
            axes[axis].step(time_s, sp_values, where='post', label=f'{name} step')
            axes[axis].plot(time_s, omega_values, label=f'{name} measured')
            axes[axis].set_ylabel('rad/s')
            axes[axis].grid(True, alpha=0.3)
            axes[axis].legend(loc='upper right')

        for axis, name in enumerate(axis_names):
            axes[3].plot(time_s, [torque[axis] for torque in torques], label=f'{name} torque')
        axes[3].set_ylabel('Nm')
        axes[3].set_xlabel('time [s]')
        axes[3].grid(True, alpha=0.3)
        axes[3].legend(loc='upper right')

        segment_start_times = []
        last_label = None
        for stamp, label in zip(time_s, labels):
            if label != last_label:
                segment_start_times.append((stamp, label))
                last_label = label

        for axis in axes:
            ymin, ymax = axis.get_ylim()
            for stamp, label in segment_start_times:
                axis.axvline(stamp, color='0.75', linewidth=0.8, linestyle='--')
                if label != 'zero':
                    axis.text(stamp, ymax, label, rotation=90, va='top', ha='right', fontsize=8)
            axis.set_ylim(ymin, ymax)

        fig.suptitle('Combined rate controller step response')
        fig.tight_layout()
        self.plt.show()

    def segment_passed(self, label):
        segment_samples = [row for row in self.samples if row[1] == label]
        if not segment_samples:
            return False
        min_time = segment_samples[-1][0] - self.settling_time
        recent = [row for row in segment_samples if row[0] >= min_time]
        if not recent:
            return False

        active_axes = [axis for axis in range(3) if abs(recent[-1][2][axis]) > 1e-9]
        for axis in active_axes:
            errors = [
                abs(row[2][axis] - row[3][axis])
                for row in recent
                if math.isfinite(row[3][axis])
            ]
            if not errors or sum(errors) / len(errors) > self.tolerance:
                return False
        return True

    def run(self):
        if self.auto_arm:
            self.get_logger().info(
                f'Arming /{self.shuttle_ns} and /{self.fixed_wing_ns} with zero allocator setpoint')
            self.publish_arm()
            self.publish_allocator_setpoint([0.0, 0.0, 0.0], reset_integral=True)
            rclpy.spin_once(self, timeout_sec=0.0)

        self.setup_live_plot()

        self.get_logger().info('Starting rate validation. Watch the vehicle response for the note on each test.')
        results = []
        for label, sp, note in TESTS:
            self.get_logger().info(f'Starting test: {label} - {note}')
            half_test_seconds = 0.5 * self.test_seconds
            self.stream_rate_setpoint(sp, half_test_seconds, f'{label} step', reset_integral=True)
            self.stream_rate_setpoint(
                [0.0, 0.0, 0.0],
                half_test_seconds,
                f'{label} return',
                reset_integral=False)
            self.log_result(label, sp, note)
            results.append((label, self.segment_passed(f'{label} step')))

        for label, passed in results:
            self.get_logger().info(f'{label}: {"PASS" if passed else "CHECK LOGS"}')
        self.log_step_response_metrics()
        self.plot_samples()


def main(args=None):
    rclpy.init(args=args)
    node = CombinedRateControllerTestNode()
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
