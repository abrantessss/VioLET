"""Expose RC status and horizontal ground speed as live/playback plot scalars.

Uses recorded telemetry topics as well as live telemetry; sends no vehicle commands.
"""
import math

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, Float32, Float32MultiArray


def channel_pwm(values, channel=7):
    if channel < 1 or channel > len(values):
        return None
    value = float(values[channel - 1])
    return value if math.isfinite(value) and 800 <= value <= 2200 else None


class RcStatusPlot(Node):
    def __init__(self):
        super().__init__('rc_status_plot')
        self.channel = self.declare_parameter('rc_channel', 7).value
        if not isinstance(self.channel, int) or not 1 <= self.channel <= 18:
            raise ValueError('rc_channel must be an integer from 1 to 18')
        self.channel_pub = self.create_publisher(Float32, 'rviz_plots/rc/channel7', 10)
        self.armed_pub = self.create_publisher(Float32, 'rviz_plots/rc/armed', 10)
        self.horizontal_speed_pub = self.create_publisher(
            Float32, 'rviz_plots/velocity/horizontal_speed', 10)
        self.create_subscription(Float32MultiArray, 'telemetry/rc_channels',
                                 self.rc_cb, qos_profile_sensor_data)
        self.create_subscription(Bool, 'telemetry/armed', self.armed_cb, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, 'telemetry/velocity',
                                 self.velocity_cb, qos_profile_sensor_data)

    def rc_cb(self, msg):
        value = channel_pwm(msg.data, self.channel)
        if value is not None:
            self.channel_pub.publish(Float32(data=value))

    def armed_cb(self, msg):
        self.armed_pub.publish(Float32(data=1.0 if msg.data else 0.0))

    def velocity_cb(self, msg):
        velocity = msg.twist.linear
        if math.isfinite(velocity.x) and math.isfinite(velocity.y):
            self.horizontal_speed_pub.publish(Float32(data=math.hypot(velocity.x, velocity.y)))


def main(args=None):
    rclpy.init(args=args)
    node = RcStatusPlot()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
