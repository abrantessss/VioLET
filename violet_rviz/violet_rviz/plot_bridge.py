#!/usr/bin/env python3
import math
from collections import deque

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped, Quaternion, Vector3Stamped, WrenchStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float32
from violet_msgs.msg import PlotData


def _norm3(values):
    return math.sqrt(sum(float(value) * float(value) for value in values[:3]))


def _quat_from_rpy(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return Quaternion(
        x=sr * cp * cy - cr * sp * sy,
        y=cr * sp * cy + sr * cp * sy,
        z=cr * cp * sy - sr * sp * cy,
        w=cr * cp * cy + sr * sp * sy,
    )


class PlotBridge(Node):
    def __init__(self):
        super().__init__('violet_plot_bridge')

        self.declare_parameter('vehicle_ns', 'drone1')
        self.declare_parameter('plot_data_topic', '')
        self.declare_parameter('output_prefix', 'rviz_plots')
        self.declare_parameter('history_length', 2000)
        self.declare_parameter('publish_paths', True)
        self.declare_parameter('desired_velocity_path_scaled', False)
        self.declare_parameter('attitude_reference_topic', '')
        self.declare_parameter('force_reference_topic', '')
        self.declare_parameter('force_achieved_topic', '')
        self.declare_parameter('velocity_tracking_topic', '')
        self.declare_parameter('vertical_velocity_tracking_topic', '')
        self.declare_parameter('tracking_publish_rate_hz', 20.0)

        vehicle_ns = self.get_parameter('vehicle_ns').value.strip('/')
        output_prefix = self.get_parameter('output_prefix').value.strip('/')
        plot_data_topic = self.get_parameter('plot_data_topic').value
        if not plot_data_topic:
            plot_data_topic = f'/{vehicle_ns}/plots/data'
        attitude_reference_topic = self.get_parameter('attitude_reference_topic').value
        force_reference_topic = self.get_parameter('force_reference_topic').value
        force_achieved_topic = self.get_parameter('force_achieved_topic').value
        velocity_tracking_topic = self.get_parameter('velocity_tracking_topic').value
        vertical_velocity_tracking_topic = self.get_parameter(
            'vertical_velocity_tracking_topic').value
        if not attitude_reference_topic:
            attitude_reference_topic = f'/{vehicle_ns}/controller/out/attitude_reference'
        if not force_reference_topic:
            force_reference_topic = f'/{vehicle_ns}/fmu/in/combined_wrench'
        if not force_achieved_topic:
            force_achieved_topic = f'/{vehicle_ns}/controller/out/achieved_wrench'
        if not velocity_tracking_topic:
            velocity_tracking_topic = f'/{vehicle_ns}/controller/out/velocity_tracking'
        if not vertical_velocity_tracking_topic:
            vertical_velocity_tracking_topic = (
                f'/{vehicle_ns}/controller/out/vertical_velocity_tracking')

        self.base_topic = f'/{vehicle_ns}/{output_prefix}'
        self.history_length = int(self.get_parameter('history_length').value)
        self.publish_paths = bool(self.get_parameter('publish_paths').value)
        self.desired_velocity_path_scaled = bool(
            self.get_parameter('desired_velocity_path_scaled').value)
        tracking_publish_rate_hz = max(
            1.0, float(self.get_parameter('tracking_publish_rate_hz').value))

        self.latest_tracking_values = {
            'yaw/reference': math.nan,
            'yaw/measured': math.nan,
            'forces/tx_reference': math.nan,
            'forces/tx_achieved': math.nan,
            'forces/tz_reference': math.nan,
            'forces/tz_achieved': math.nan,
            'velocity/vx': math.nan,
            'velocity/vy': math.nan,
            'velocity/vz': math.nan,
            'velocity/u_reference': math.nan,
            'velocity/u_measured': math.nan,
            'velocity/lateral': math.nan,
            'velocity/up_reference': math.nan,
            'velocity/up_measured': math.nan,
        }
        self.previous_measured_yaw_raw = None
        self.measured_yaw_unwrapped = 0.0

        self.scalar_publishers = {
            'errors/norm': self.create_publisher(Float32, f'{self.base_topic}/errors/norm', 10),
            'errors/x_abs': self.create_publisher(Float32, f'{self.base_topic}/errors/x_abs', 10),
            'errors/y_abs': self.create_publisher(Float32, f'{self.base_topic}/errors/y_abs', 10),
            'errors/z_abs': self.create_publisher(Float32, f'{self.base_topic}/errors/z_abs', 10),
            'velocity/uav': self.create_publisher(Float32, f'{self.base_topic}/velocity/uav', 10),
            'velocity/target_path': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/target_path', 10),
            'velocity/desired': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/desired', 10),
            'velocity/desired_path': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/desired_path', 10),
            'velocity/desired_matlab': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/desired_matlab', 10),
            'gamma_dot': self.create_publisher(Float32, f'{self.base_topic}/gamma_dot', 10),
            'vd': self.create_publisher(Float32, f'{self.base_topic}/vd', 10),
            'yaw/reference': self.create_publisher(
                Float32, f'{self.base_topic}/yaw/reference', 10),
            'yaw/measured': self.create_publisher(
                Float32, f'{self.base_topic}/yaw/measured', 10),
            'forces/tx_reference': self.create_publisher(
                Float32, f'{self.base_topic}/forces/tx_reference', 10),
            'forces/tx_achieved': self.create_publisher(
                Float32, f'{self.base_topic}/forces/tx_achieved', 10),
            'forces/tz_reference': self.create_publisher(
                Float32, f'{self.base_topic}/forces/tz_reference', 10),
            'forces/tz_achieved': self.create_publisher(
                Float32, f'{self.base_topic}/forces/tz_achieved', 10),
            'velocity/vx': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/vx', 10),
            'velocity/vy': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/vy', 10),
            'velocity/vz': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/vz', 10),
            'velocity/u_reference': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/u_reference', 10),
            'velocity/u_measured': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/u_measured', 10),
            'velocity/lateral': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/lateral', 10),
            'velocity/up_reference': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/up_reference', 10),
            'velocity/up_measured': self.create_publisher(
                Float32, f'{self.base_topic}/velocity/up_measured', 10),
        }

        self.position_pub = self.create_publisher(
            PointStamped, f'{self.base_topic}/position', 10)
        self.desired_position_pub = self.create_publisher(
            PointStamped, f'{self.base_topic}/desired_position', 10)
        self.path_pub = self.create_publisher(Path, f'{self.base_topic}/path', 10)
        self.desired_path_pub = self.create_publisher(
            Path, f'{self.base_topic}/desired_path', 10)

        self.path_history = deque(maxlen=self.history_length)
        self.desired_path_history = deque(maxlen=self.history_length)

        plot_data_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(PlotData, plot_data_topic, self.plot_data_cb, plot_data_qos)
        self.create_subscription(
            Vector3Stamped,
            attitude_reference_topic,
            self.attitude_reference_cb,
            plot_data_qos,
        )
        self.create_subscription(
            WrenchStamped,
            force_reference_topic,
            self.force_reference_cb,
            plot_data_qos,
        )
        self.create_subscription(
            WrenchStamped,
            force_achieved_topic,
            self.force_achieved_cb,
            plot_data_qos,
        )
        self.create_subscription(
            Vector3Stamped,
            velocity_tracking_topic,
            self.velocity_tracking_cb,
            plot_data_qos,
        )
        self.create_subscription(
            Vector3Stamped,
            vertical_velocity_tracking_topic,
            self.vertical_velocity_tracking_cb,
            plot_data_qos,
        )
        self.tracking_timer = self.create_timer(
            1.0 / tracking_publish_rate_hz,
            self.publish_tracking_values,
        )
        self.get_logger().info(
            f'Bridging {plot_data_topic} to {self.base_topic} for RViz plots')

    def plot_data_cb(self, msg):
        frame_id = msg.header.frame_id or 'map'
        stamp = msg.header.stamp

        position = [float(value) for value in msg.position]
        desired_position = [float(value) for value in msg.pd]
        dpd_dgamma = [float(value) for value in msg.dpd_dgamma]
        velocity = [float(value) for value in msg.inertial_velocity]

        error = [position[i] - desired_position[i] for i in range(3)]
        path_rate_scale = _norm3(dpd_dgamma)
        gamma_dot_path = float(msg.gamma) * path_rate_scale
        vd_path = float(msg.vd) * path_rate_scale
        desired_matlab = vd_path if self.desired_velocity_path_scaled else float(msg.vd)

        self._publish_scalar('errors/norm', _norm3(error))
        self._publish_scalar('errors/x_abs', abs(error[0]))
        self._publish_scalar('errors/y_abs', abs(error[1]))
        self._publish_scalar('errors/z_abs', abs(error[2]))
        self._publish_scalar('velocity/uav', _norm3(velocity))
        self._publish_scalar('velocity/target_path', gamma_dot_path)
        self._publish_scalar('velocity/desired', float(msg.vd))
        self._publish_scalar('velocity/desired_path', vd_path)
        self._publish_scalar('velocity/desired_matlab', desired_matlab)
        self._publish_scalar('gamma_dot', float(msg.gamma))
        self._publish_scalar('vd', float(msg.vd))
        measured_yaw_raw = float(msg.attitude[2])
        if self.previous_measured_yaw_raw is None:
            self.measured_yaw_unwrapped = measured_yaw_raw
        else:
            measured_yaw_delta = math.atan2(
                math.sin(measured_yaw_raw - self.previous_measured_yaw_raw),
                math.cos(measured_yaw_raw - self.previous_measured_yaw_raw),
            )
            self.measured_yaw_unwrapped += measured_yaw_delta
        self.previous_measured_yaw_raw = measured_yaw_raw
        self.latest_tracking_values['yaw/measured'] = self.measured_yaw_unwrapped
        self.latest_tracking_values['velocity/vx'] = velocity[0]
        self.latest_tracking_values['velocity/vy'] = velocity[1]
        self.latest_tracking_values['velocity/vz'] = velocity[2]

        position_msg = self._point_msg(stamp, frame_id, position)
        desired_position_msg = self._point_msg(stamp, frame_id, desired_position)
        self.position_pub.publish(position_msg)
        self.desired_position_pub.publish(desired_position_msg)

        if self.publish_paths:
            self.path_history.append(self._pose_msg(stamp, frame_id, position, msg.attitude))
            self.desired_path_history.append(
                self._pose_msg(stamp, frame_id, desired_position, None))
            self.path_pub.publish(self._path_msg(stamp, frame_id, self.path_history))
            self.desired_path_pub.publish(
                self._path_msg(stamp, frame_id, self.desired_path_history))

    def attitude_reference_cb(self, msg):
        self.latest_tracking_values['yaw/reference'] = float(msg.vector.z)

    def force_reference_cb(self, msg):
        self.latest_tracking_values['forces/tx_reference'] = float(msg.wrench.force.x)
        self.latest_tracking_values['forces/tz_reference'] = -float(msg.wrench.force.z)

    def force_achieved_cb(self, msg):
        self.latest_tracking_values['forces/tx_achieved'] = float(msg.wrench.force.x)
        self.latest_tracking_values['forces/tz_achieved'] = -float(msg.wrench.force.z)

    def velocity_tracking_cb(self, msg):
        self.latest_tracking_values['velocity/u_reference'] = float(msg.vector.x)
        self.latest_tracking_values['velocity/u_measured'] = float(msg.vector.y)
        self.latest_tracking_values['velocity/lateral'] = float(msg.vector.z)

    def vertical_velocity_tracking_cb(self, msg):
        self.latest_tracking_values['velocity/up_reference'] = float(msg.vector.x)
        self.latest_tracking_values['velocity/up_measured'] = float(msg.vector.y)

    def publish_tracking_values(self):
        for key, value in self.latest_tracking_values.items():
            if math.isfinite(value):
                self._publish_scalar(key, value)

    def _publish_scalar(self, key, value):
        msg = Float32()
        msg.data = float(value)
        self.scalar_publishers[key].publish(msg)

    @staticmethod
    def _point_msg(stamp, frame_id, values):
        msg = PointStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.point.x = float(values[0])
        msg.point.y = float(values[1])
        msg.point.z = float(values[2])
        return msg

    @staticmethod
    def _pose_msg(stamp, frame_id, position, attitude):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        if attitude is not None and len(attitude) >= 3:
            msg.pose.orientation = _quat_from_rpy(
                float(attitude[0]), float(attitude[1]), float(attitude[2]))
        else:
            msg.pose.orientation.w = 1.0
        return msg

    @staticmethod
    def _path_msg(stamp, frame_id, poses):
        msg = Path()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.poses = list(poses)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = PlotBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
