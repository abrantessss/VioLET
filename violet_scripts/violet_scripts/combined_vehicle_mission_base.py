#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from violet_msgs.msg import AutopilotPlot, Mode, PlotData, State, Trajectory


class CombinedVehicleMissionNode(Node):
    def __init__(self, node_name, mission_label):
        super().__init__(node_name)

        self.mission_label = mission_label
        self.declare_parameter('shuttle_ns', 'drone1')
        self.declare_parameter('fixed_wing_ns', 'drone2')
        self.declare_parameter('arm_seconds', 2.0)
        self.declare_parameter('prime_seconds', 3.0)
        self.declare_parameter('mission_seconds', 240.0)
        self.shuttle_ns = self.get_parameter('shuttle_ns').value.strip('/')
        self.fixed_wing_ns = self.get_parameter('fixed_wing_ns').value.strip('/')
        self.arm_seconds = float(self.get_parameter('arm_seconds').value)
        self.prime_seconds = float(self.get_parameter('prime_seconds').value)
        self.mission_seconds = float(self.get_parameter('mission_seconds').value)

        shuttle_prefix = f'/{self.shuttle_ns}'
        self.arm_pubs = [
            self.create_publisher(
                Mode, f'{shuttle_prefix}/fmu/mode/arm', qos_profile_sensor_data),
            self.create_publisher(
                Mode, f'/{self.fixed_wing_ns}/fmu/mode/arm', qos_profile_sensor_data),
        ]
        self.follow_pub = self.create_publisher(
            Trajectory, f'{shuttle_prefix}/fmu/mode/follow', qos_profile_sensor_data)
        self.plot_data_pub = self.create_publisher(
            PlotData, f'{shuttle_prefix}/plots/data', qos_profile_sensor_data)

        self.create_subscription(
            State,
            f'{shuttle_prefix}/fmu/telemetry/state',
            self.state_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            AutopilotPlot,
            f'{shuttle_prefix}/fmu/telemetry/autopilot_plot',
            self.autopilot_plot_cb,
            qos_profile_sensor_data,
        )

        self.current_state = None
        self.current_autopilot_plot = None
        self.phase = 'WAIT_FOR_STATE'
        self.phase_start_time = None
        self.mission_start_time = None
        self.plot_samples_seen = 0
        self.trajectory = None
        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info(
            f'Combined {mission_label} mission waiting for shuttle telemetry...')

    def state_cb(self, msg):
        self.current_state = msg
        self.publish_plot_data_if_ready()

    def autopilot_plot_cb(self, msg):
        self.current_autopilot_plot = msg
        self.publish_plot_data_if_ready()

    def build_trajectory(self):
        raise NotImplementedError

    def publish_arm(self):
        msg = Mode()
        for publisher in self.arm_pubs:
            publisher.publish(msg)

    def publish_trajectory(self):
        if self.trajectory is None:
            self.trajectory = self.build_trajectory()
        self.follow_pub.publish(self.trajectory)

    def publish_plot_data_if_ready(self):
        if self.phase != 'FOLLOWING':
            return
        if self.current_state is None or self.current_autopilot_plot is None:
            return

        self.plot_samples_seen += 1
        if self.plot_samples_seen <= 5:
            return

        elapsed = max(0.0, time.time() - self.mission_start_time)
        msg = PlotData()
        msg.header.frame_id = self.current_state.header.frame_id
        msg.header.stamp.sec = int(elapsed)
        msg.header.stamp.nanosec = int((elapsed % 1.0) * 1e9)
        msg.position = self.current_state.position
        msg.inertial_velocity = self.current_state.inertial_velocity
        msg.attitude = self.current_state.attitude
        msg.angular_velocity = self.current_state.angular_velocity
        msg.gamma = self.current_autopilot_plot.gamma
        msg.vd = self.current_autopilot_plot.vd
        msg.pd = self.current_autopilot_plot.pd
        msg.dpd_dgamma = self.current_autopilot_plot.dpd_dgamma
        msg.gains = self.current_autopilot_plot.gains
        self.plot_data_pub.publish(msg)

    def timer_cb(self):
        now = time.time()
        if self.phase == 'WAIT_FOR_STATE':
            if self.current_state is not None:
                self.get_logger().info(
                    f'Telemetry received; arming /{self.shuttle_ns} and '
                    f'/{self.fixed_wing_ns}')
                self.phase_start_time = now
                self.phase = 'ARMING'

        elif self.phase == 'ARMING':
            self.publish_arm()
            if now - self.phase_start_time >= self.arm_seconds:
                self.get_logger().info(
                    f'Priming combined {self.mission_label} trajectory for '
                    f'{self.prime_seconds:.1f} s')
                self.phase_start_time = now
                self.phase = 'PRIMING'

        elif self.phase == 'PRIMING':
            # Match the controller test: arm first, then stream the command.
            # Repeated publication keeps gamma at its initial value while the
            # vehicle takes off and settles onto the initial path heading.
            self.publish_trajectory()
            if now - self.phase_start_time >= self.prime_seconds:
                self.get_logger().info(
                    f'Following combined {self.mission_label} trajectory')
                self.publish_trajectory()
                self.mission_start_time = now
                self.plot_samples_seen = 0
                self.phase = 'FOLLOWING'

        elif self.phase == 'FOLLOWING':
            if now - self.mission_start_time >= self.mission_seconds:
                self.get_logger().info('Mission plot window complete; shutting down')
                rclpy.shutdown()


def spin_mission(node_factory, args=None):
    rclpy.init(args=args)
    node = node_factory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
