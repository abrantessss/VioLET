#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from violet_msgs.msg import AutopilotPlot, Mode, PlotData, State, Trajectory


class EasyGliderMissionNode(Node):
    def __init__(self, node_name, mission_label):
        super().__init__(node_name)

        self.mission_label = mission_label
        self.declare_parameter('vehicle_ns', 'drone1')
        self.vehicle_ns = self.get_parameter('vehicle_ns').get_parameter_value().string_value
        vehicle_prefix = f'/{self.vehicle_ns}'

        self.arm_pub = self.create_publisher(Mode, f'{vehicle_prefix}/fmu/mode/arm', qos_profile_sensor_data)
        self.takeoff_pub = self.create_publisher(Mode, f'{vehicle_prefix}/fmu/mode/takeoff', qos_profile_sensor_data)
        self.land_pub = self.create_publisher(Mode, f'{vehicle_prefix}/fmu/mode/land', qos_profile_sensor_data)
        self.loiter_pub = self.create_publisher(Mode, f'{vehicle_prefix}/fmu/mode/loiter', qos_profile_sensor_data)
        self.follow_pub = self.create_publisher(Trajectory, f'{vehicle_prefix}/fmu/mode/follow', qos_profile_sensor_data)
        self.plot_data_pub = self.create_publisher(
            PlotData,
            f'{vehicle_prefix}/plots/data',
            qos_profile_sensor_data,
        )

        self.state_sub = self.create_subscription(
            State,
            f'{vehicle_prefix}/fmu/telemetry/state',
            self.state_cb,
            qos_profile_sensor_data,
        )
        self.autopilot_plot_sub = self.create_subscription(
            AutopilotPlot,
            f'{vehicle_prefix}/fmu/telemetry/autopilot_plot',
            self.autopilot_plot_cb,
            qos_profile_sensor_data,
        )

        self.current_state = None
        self.current_autopilot_plot = None
        self.phase = 'WAIT_FOR_STATE'
        self.start_time = None
        self.done_start_time = None
        self.plotting_started = False
        self.plot_skip_count = 5
        self.plot_samples_seen = 0
        self.shutdown_requested = False

        self.timer = self.create_timer(0.05, self.timer_cb)
        self.get_logger().info(f'{self.mission_label} mission node started, waiting for telemetry...')

    def state_cb(self, msg):
        self.current_state = msg
        self.publish_plot_data_if_ready()

    def autopilot_plot_cb(self, msg):
        self.current_autopilot_plot = msg
        self.publish_plot_data_if_ready()

    def publish_plot_data_if_ready(self):
        if self.phase != 'DONE':
            return

        if self.done_start_time is None:
            return

        if self.current_state is None or self.current_autopilot_plot is None:
            return

        self.plot_samples_seen += 1
        if self.plot_samples_seen <= self.plot_skip_count:
            return

        if not self.plotting_started:
            self.get_logger().info(f'Publishing merged plot telemetry on /{self.vehicle_ns}/plots/data')
            self.plotting_started = True

        elapsed = max(0.0, time.time() - self.done_start_time)
        plot_msg = PlotData()
        plot_msg.header.frame_id = self.current_state.header.frame_id
        plot_msg.header.stamp.sec = int(elapsed)
        plot_msg.header.stamp.nanosec = int((elapsed % 1.0) * 1e9)
        plot_msg.position = self.current_state.position
        plot_msg.inertial_velocity = self.current_state.inertial_velocity
        plot_msg.attitude = self.current_state.attitude
        plot_msg.angular_velocity = self.current_state.angular_velocity
        plot_msg.gamma = self.current_autopilot_plot.gamma
        plot_msg.vd = self.current_autopilot_plot.vd
        plot_msg.pd = self.current_autopilot_plot.pd
        plot_msg.dpd_dgamma = self.current_autopilot_plot.dpd_dgamma
        plot_msg.gains = self.current_autopilot_plot.gains
        self.plot_data_pub.publish(plot_msg)

    def build_trajectory(self):
        raise NotImplementedError

    def timer_cb(self):
        if self.phase == 'WAIT_FOR_STATE':
            if self.current_state is not None:
                self.get_logger().info('Telemetry received. Waiting 3s for vehicle to be fully armable...')
                self.start_time = time.time()
                self.phase = 'WAIT_ARMABLE'

        elif self.phase == 'WAIT_ARMABLE':
            if time.time() - self.start_time > 3.0:
                self.get_logger().info('Sending ARM command...')
                self.arm_pub.publish(Mode())
                self.start_time = time.time()
                self.phase = 'ARMING'

        elif self.phase == 'ARMING':
            if time.time() - self.start_time > 3.0:
                self.get_logger().info('Sending TAKEOFF command...')
                self.takeoff_pub.publish(Mode())
                self.start_time = time.time()
                self.phase = 'TAKEOFF_WAIT'

        elif self.phase == 'TAKEOFF_WAIT':
            if time.time() - self.start_time > 15.0:
                self.get_logger().info('Sending LAND command...')
                self.land_pub.publish(Mode())
                self.start_time = time.time()
                self.phase = 'LAND_WAIT'

        elif self.phase == 'LAND_WAIT':
            if time.time() - self.start_time > 1.0:
                self.get_logger().info('Sending LOITER command...')
                self.loiter_pub.publish(Mode())
                self.start_time = time.time()
                self.phase = 'LOITER_WAIT'

        elif self.phase == 'LOITER_WAIT':
            if time.time() - self.start_time > 5.0:
                self.get_logger().info(f'Sending {self.mission_label.upper()} trajectory...')
                traj = self.build_trajectory()
                self.follow_pub.publish(traj)
                self.done_start_time = time.time()
                self.plot_samples_seen = 0
                self.phase = 'DONE'

        elif self.phase == 'DONE':
            if (
                not self.shutdown_requested
                and self.done_start_time is not None
                and time.time() - self.done_start_time > 90.0
            ):
                self.get_logger().info(f'Plot window complete. Shutting down {self.get_name()}...')
                self.shutdown_requested = True
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
