#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from violet_msgs.msg import AutopilotPlot, Mode, PlotData, State, Trajectory


class ShuttleMissionNode(Node):
    def __init__(self, node_name, mission_label):
        super().__init__(node_name)

        self.mission_label = mission_label

        self.arm_pub = self.create_publisher(Mode, '/drone1/fmu/mode/arm', qos_profile_sensor_data)
        self.takeoff_pub = self.create_publisher(Mode, '/drone1/fmu/mode/takeoff', qos_profile_sensor_data)
        self.follow_pub = self.create_publisher(Trajectory, '/drone1/fmu/mode/follow', qos_profile_sensor_data)
        self.plot_data_pub = self.create_publisher(
            PlotData,
            '/drone1/plots/data',
            qos_profile_sensor_data,
        )

        self.state_sub = self.create_subscription(
            State,
            '/drone1/fmu/telemetry/state',
            self.state_cb,
            qos_profile_sensor_data,
        )
        self.autopilot_plot_sub = self.create_subscription(
            AutopilotPlot,
            '/drone1/fmu/telemetry/autopilot_plot',
            self.autopilot_plot_cb,
            qos_profile_sensor_data,
        )

        self.current_state = None
        self.current_autopilot_plot = None
        self.current_pos = None
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
        self.current_pos = msg.position
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
            self.get_logger().info('Publishing merged plot telemetry on /drone1/plots/data')
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

    def build_final_trajectory(self):
        raise NotImplementedError

    def timer_cb(self):
        if self.phase == 'WAIT_FOR_STATE':
            if self.current_pos is not None:
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
                self.phase = 'TAKEOFF'

        elif self.phase == 'TAKEOFF':
            if time.time() - self.start_time > 10.0:
                self.get_logger().info('Sending WAYPOINT [0, 0, -8]...')
                traj = Trajectory()
                traj.path_type = 0
                traj.waypoint = [0.0, 0.0, -8.0]
                self.follow_pub.publish(traj)
                self.phase = 'GOTO_STAGING'

        elif self.phase == 'GOTO_STAGING':
            if self.current_pos is not None and self.is_near([0.0, 0.0, -8.0]):
                self.get_logger().info(f'Reached waypoint! Sending {self.mission_label.upper()} path...')
                traj = self.build_final_trajectory()
                self.follow_pub.publish(traj)
                self.done_start_time = time.time()
                self.plot_samples_seen = 0
                self.phase = 'DONE'

        elif self.phase == 'DONE':
            if (
                not self.shutdown_requested
                and self.done_start_time is not None
                and time.time() - self.done_start_time > 70.0
            ):
                self.get_logger().info(f'Plot window complete. Shutting down {self.get_name()}...')
                self.shutdown_requested = True
                rclpy.shutdown()

    def is_near(self, waypoint):
        dx = self.current_pos[0] - waypoint[0]
        dy = self.current_pos[1] - waypoint[1]
        dz = self.current_pos[2] - waypoint[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        return dist < 0.5


def spin_mission(node_factory, args=None):
    rclpy.init(args=args)
    node = node_factory()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
