"""RViz paths, position marker and scalar plots for the MAVSDK receiver."""
import math
from collections import deque

import rclpy
from violet_rviz.actuator_outputs import ACTUATOR_NAMES, DEFAULT_CHANNELS, normalized_output
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from geometry_msgs.msg import PointStamped, PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String, Float32, Float32MultiArray
from visualization_msgs.msg import Marker
from violet_msgs.msg import Trajectory


def ned_coordinates(position):
    return [float(value) for value in position]


def trajectory_points(msg, samples=361):
    if msg.path_type == 0:
        return [ned_coordinates(msg.waypoint)]
    if msg.path_type == 1:
        return [ned_coordinates(msg.line[:3]), ned_coordinates(msg.line[3:6])]
    if msg.path_type not in (2, 3):
        return []
    parameters = msg.circle if msg.path_type == 2 else msg.lemniscate
    cx, cy, cz, scale = map(float, parameters[:4])
    points = []
    for index in range(samples):
        gamma = 2 * math.pi * index / (samples - 1)
        if msg.path_type == 2:
            point = [cx + scale * math.cos(gamma), cy + scale * math.sin(gamma), cz]
        else:
            point = [cx + scale * math.cos(gamma),
                     cy + scale * math.sin(gamma) * math.cos(gamma), cz]
        points.append(ned_coordinates(point))
    return points


class MavsdkVisualization(Node):
    def __init__(self):
        super().__init__('mavsdk_visualization')
        self.declare_parameter('history_length', 5000)
        self.declare_parameter('marker_diameter', 0.6)
        length = int(self.get_parameter('history_length').value)
        diameter = float(self.get_parameter('marker_diameter').value)
        if length <= 0 or not math.isfinite(diameter) or diameter <= 0:
            raise ValueError('history_length and marker_diameter must be positive')
        self.actuator_channels = list(self.declare_parameter('actuator_channels', list(DEFAULT_CHANNELS)).value)
        self.actuator_encoding = self.declare_parameter('actuator_encoding', 'pwm').value
        self.actuator_min = list(self.declare_parameter('actuator_pwm_min', [1000.0]*5).value)
        self.actuator_trim = list(self.declare_parameter('actuator_pwm_trim', [1500.0]*5).value)
        self.actuator_max = list(self.declare_parameter('actuator_pwm_max', [2000.0]*5).value)
        if (len(self.actuator_channels) != 5 or len(set(self.actuator_channels)) != 5
                or any(ch < 1 or ch > 32 for ch in self.actuator_channels)
                or any(len(values) != 5 for values in (self.actuator_min, self.actuator_trim, self.actuator_max))):
            raise ValueError('Five unique output channels and five calibration values are required')
        for i in range(5):
            normalized_output(1500.0, self.actuator_min[i], self.actuator_trim[i], self.actuator_max[i], self.actuator_encoding)
        self.mode_status = {}
        self.status_position = None
        self.diameter = diameter
        self.history = deque(maxlen=length)
        self.path_pub = self.create_publisher(Path, 'rviz_plots/path', 10)
        self.desired_pub = self.create_publisher(
            Path, 'rviz_plots/desired_path',
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.point_pub = self.create_publisher(PointStamped, 'rviz_plots/position', 10)
        self.marker_pub = self.create_publisher(Marker, 'rviz_plots/current_position', 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'rviz_plots/pose', 10)
        keys = ['velocity/speed', 'velocity/east', 'velocity/north', 'velocity/down',
                'attitude/roll', 'attitude/pitch', 'attitude/yaw']
        keys += ['velocity/desired', 'velocity/gamma_speed', 'gamma', 'gamma_dot', 'controller_state']
        keys += [f'path_error/{axis}' for axis in ('north', 'east', 'down', 'norm')]
        keys += [f'attitude_desired/{axis}' for axis in ('roll', 'pitch', 'heading')]
        keys += [f'attitude_error/{axis}' for axis in ('roll', 'pitch', 'heading')]
        keys += [f'actuators/output_{index}' for index in range(16)]
        keys += [f'actuators/{name}' for name in ACTUATOR_NAMES]
        keys += [f'actuators/control_{index}' for index in range(8)]
        self.scalars = {key: self.create_publisher(Float32, f'rviz_plots/{key}', 10)
                        for key in keys}
        self.gamma_pub = self.create_publisher(Marker, 'rviz_plots/gamma_position', 10)
        self.gamma_xy_pub = self.create_publisher(Path, 'rviz_plots/gamma_xy', 10)
        self.create_subscription(Float32MultiArray, 'telemetry/los4/config', self.los4_config_cb,
                                 QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Float32MultiArray, 'telemetry/los4/state', self.los4_state_cb,
                                 qos_profile_sensor_data)
        self.mode_pub = self.create_publisher(Marker, 'rviz_plots/vehicle_mode', 10)
        for topic, kind in (('flight_mode', String), ('armed', Bool), ('kill_active', Bool)):
            self.create_subscription(kind, 'telemetry/' + topic,
                                     lambda msg, key=topic: self.status_cb(key, msg), qos_profile_sensor_data)
        self.create_timer(0.2, self.publish_status)
        self.create_subscription(PoseStamped, 'telemetry/pose', self.pose_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'telemetry/velocity', self.velocity_cb, qos_profile_sensor_data)
        self.create_subscription(Vector3Stamped, 'telemetry/attitude', self.attitude_cb, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, 'telemetry/actuator_outputs',
                                 self.outputs_cb, qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, 'telemetry/actuator_controls',
                                 self.controls_cb, qos_profile_sensor_data)
        self.create_subscription(Trajectory, 'fmu/mode/follow', self.trajectory_cb, qos_profile_sensor_data)

    def scalar(self, key, value):
        if math.isfinite(value):
            self.scalars[key].publish(Float32(data=float(value)))

    def pose_cb(self, msg):
        if not all(math.isfinite(v) for v in (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)):
            return
        self.status_position = msg.pose.position
        self.history.append(msg)
        self.path_pub.publish(Path(header=msg.header, poses=list(self.history)))
        self.pose_pub.publish(msg)
        point = PointStamped(header=msg.header, point=msg.pose.position)
        self.point_pub.publish(point)
        marker = Marker()
        marker.header = msg.header
        marker.ns = 'current_position'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = msg.pose.position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self.diameter
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.25, 0.1, 1.0
        self.marker_pub.publish(marker)

    def velocity_cb(self, msg):
        v = msg.twist.linear
        self.scalar('velocity/north', v.x)
        self.scalar('velocity/east', v.y)
        self.scalar('velocity/down', v.z)
        self.scalar('velocity/speed', math.sqrt(v.x*v.x + v.y*v.y + v.z*v.z))

    def attitude_cb(self, msg):
        for axis, value in zip(('roll', 'pitch', 'yaw'), (msg.vector.x, msg.vector.y, msg.vector.z)):
            self.scalar(f'attitude/{axis}', value)

    def outputs_cb(self, msg):
        for index, value in enumerate(msg.data[:16]):
            self.scalar(f'actuators/output_{index}', value)

        for i, name in enumerate(ACTUATOR_NAMES):
            channel = self.actuator_channels[i]-1
            if channel < len(msg.data):
                value = normalized_output(msg.data[channel], self.actuator_min[i], self.actuator_trim[i],
                                          self.actuator_max[i], self.actuator_encoding)
                if value is not None:
                    self.scalar(f'actuators/{name}', value)

    def status_cb(self, key, msg):
        self.mode_status[key] = (msg.data, self.get_clock().now().nanoseconds)

    def publish_status(self):
        now = self.get_clock().now()
        def value(key, timeout):
            sample = self.mode_status.get(key)
            return sample[0] if sample and (now.nanoseconds-sample[1])/1e9 < timeout else None
        mode, armed, kill = value('flight_mode', 3), value('armed', 3), value('kill_active', 0.6)
        marker = Marker()
        marker.header.frame_id = 'map_ned'
        marker.header.stamp = now.to_msg()
        marker.ns = 'vehicle_mode'
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        if self.status_position is not None:
            marker.pose.position.x = self.status_position.x
            marker.pose.position.y = self.status_position.y
            marker.pose.position.z = self.status_position.z
        marker.pose.position.z -= 2.0  # above the vehicle in NED
        marker.scale.z = 0.7
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (1.0, 0.2, 0.1, 1.0) if kill else (0.9, 0.95, 1.0, 1.0)
        marker.text = ('Mode: ' + (mode or 'UNKNOWN') + '\nArmed: '
                       + ('UNKNOWN' if armed is None else ('YES' if armed else 'NO'))
                       + '\nKill CH6: ' + ('UNKNOWN' if kill is None else ('ACTIVE' if kill else 'OFF')))
        marker.lifetime.sec = 1
        self.mode_pub.publish(marker)

    def controls_cb(self, msg):
        for index, value in enumerate(msg.data[:8]):
            self.scalar(f'actuators/control_{index}', value)

    def publish_reference(self, points):
        if not points or not all(math.isfinite(v) for p in points for v in p):
            return
        path = Path()
        path.header.frame_id = 'map_ned'
        path.header.stamp = self.get_clock().now().to_msg()
        for point in points:
            pose = PoseStamped(header=path.header)
            pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = point
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.desired_pub.publish(path)

    def trajectory_cb(self, msg):
        self.publish_reference(trajectory_points(msg))

    def los4_config_cb(self, msg):
        v = msg.data
        if len(v) != 15 or v[0] != 1 or v[1] != 1 or not all(math.isfinite(x) for x in v):
            return
        if v[2] not in (1, 2, 3) or v[12] <= 0 or v[13] <= 0 or v[14] <= 0:
            return
        trajectory = Trajectory(path_type=int(v[2]))
        if trajectory.path_type == 1:
            trajectory.line = list(v[3:9]) + [float(v[14])]
        elif trajectory.path_type == 2:
            trajectory.circle = list(v[9:12]) + [float(v[12]), float(v[14])]
        else:
            trajectory.lemniscate = list(v[9:12]) + [float(v[13]), float(v[14])]
        self.publish_reference(trajectory_points(trajectory))

    def los4_state_cb(self, msg):
        v = msg.data
        if len(v) != 25 or v[0] != 1 or v[1] != 2 or not all(math.isfinite(x) for x in v[:12]):
            return
        if not math.isfinite(v[24]) or v[24] != int(v[24]) or not 0 <= v[24] <= 15:
            return
        flags = int(v[24])
        self.scalar('controller_state', v[2])
        self.scalar('gamma', v[3])
        self.scalar('gamma_dot', v[4])
        self.scalar('velocity/desired', v[5])
        self.scalar('velocity/gamma_speed', abs(v[4]) * math.sqrt(sum(x*x for x in v[9:12])))
        marker = Marker()
        marker.header.frame_id = 'map_ned'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'gamma_position'
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = v[6:9]
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = self.diameter * 0.8
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = 0.1, 0.9, 0.95, 1.0
        marker.lifetime.nanosec = 300000000
        self.gamma_pub.publish(marker)
        pose = PoseStamped(header=marker.header, pose=marker.pose)
        self.gamma_xy_pub.publish(Path(header=marker.header, poses=[pose]))
        if flags & 1 and all(math.isfinite(x) for x in v[12:15]):
            error = [v[12+i] - v[6+i] for i in range(3)]
            for axis, value in zip(('north', 'east', 'down'), error):
                self.scalar(f'path_error/{axis}', value)
            self.scalar('path_error/norm', math.sqrt(sum(x*x for x in error)))
        if flags & 2 and all(math.isfinite(x) for x in v[15:18]):
            for axis, value in zip(('north', 'east', 'down'), v[15:18]):
                self.scalar(f'velocity/{axis}', value)
            self.scalar('velocity/speed', math.sqrt(sum(x*x for x in v[15:18])))
        if flags & 4 and all(math.isfinite(x) for x in v[18:21]):
            for axis, value in zip(('roll', 'pitch', 'yaw'), v[18:21]):
                self.scalar(f'attitude/{axis}', value)
        if flags & 8 and all(math.isfinite(x) for x in v[21:24]):
            for axis, value in zip(('roll', 'pitch', 'heading'), v[21:24]):
                self.scalar(f'attitude_desired/{axis}', value)
            if flags & 4 and all(math.isfinite(x) for x in v[18:21]):
                for axis, measured, desired in zip(('roll', 'pitch', 'heading'), v[18:21], v[21:24]):
                    error = math.atan2(math.sin(desired-measured), math.cos(desired-measured))
                    self.scalar(f'attitude_error/{axis}', error)


def main(args=None):
    rclpy.init(args=args)
    node = MavsdkVisualization()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
