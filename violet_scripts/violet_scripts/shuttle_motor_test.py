#!/usr/bin/env python3
"""One-motor Offboard test for the standalone shuttle in SITL."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import qos_profile_sensor_data
from px4_msgs.msg import ActuatorMotors, OffboardControlMode, VehicleCommand, VehicleStatus


class ShuttleMotorTest(Node):
    def __init__(self):
        super().__init__('shuttle_motor_test')
        for name, default in [('vehicle_ns', 'drone1'), ('vehicle_id', 1),
                              ('motor', 1), ('command', 0.1), ('duration', 2.0)]:
            self.declare_parameter(name, default)
        self.motor = self.get_parameter('motor').value
        self.value = float(self.get_parameter('command').value)
        self.duration = float(self.get_parameter('duration').value)
        self.vehicle_id = self.get_parameter('vehicle_id').value
        if not 1 <= self.motor <= 4:
            raise ValueError('motor must be 1..4')
        if not math.isfinite(self.value) or not 0.0 < self.value <= 0.2:
            raise ValueError('SITL identification test requires 0 < command <= 0.2')
        if not math.isfinite(self.duration) or not 0.0 < self.duration <= 5.0:
            raise ValueError('duration must be in (0, 5] seconds')
        prefix = '/' + self.get_parameter('vehicle_ns').value.strip('/') + '/fmu'
        self.motors = self.create_publisher(ActuatorMotors, prefix + '/in/actuator_motors', 10)
        self.heartbeat = self.create_publisher(OffboardControlMode, prefix + '/in/offboard_control_mode', 10)
        self.commands = self.create_publisher(VehicleCommand, prefix + '/in/vehicle_command', 10)
        version = int(getattr(VehicleStatus, 'MESSAGE_VERSION', 0))
        topic = prefix + '/out/vehicle_status' + (f'_v{version}' if version else '')
        self.status = None
        self.status_time = 0.0
        self.owns_vehicle = False
        self.create_subscription(VehicleStatus, topic, self.on_status, qos_profile_sensor_data)

    def on_status(self, msg):
        self.status = msg
        self.status_time = time.monotonic()

    def fresh(self):
        return self.status is not None and time.monotonic() - self.status_time < 1.0

    def send_command(self, command, p1=0.0, p2=0.0):
        msg = VehicleCommand()
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        msg.command = command
        msg.param1 = float(p1)
        msg.param2 = float(p2)
        msg.target_system = self.vehicle_id
        msg.target_component = 1
        msg.source_system = 255
        msg.source_component = 190
        msg.from_external = True
        self.commands.publish(msg)

    def stream(self, value=0.0):
        now = self.get_clock().now().nanoseconds // 1000
        hb = OffboardControlMode()
        hb.timestamp = now
        hb.direct_actuator = True
        self.heartbeat.publish(hb)
        msg = ActuatorMotors()
        msg.timestamp = now
        msg.timestamp_sample = now
        # Stop every unused output explicitly; selected motor uses one-based numbering.
        msg.control = [0.0] * len(msg.control)
        msg.control[self.motor - 1] = float(value)
        self.motors.publish(msg)

    def tick(self, value=0.0):
        self.stream(value)
        rclpy.spin_once(self, timeout_sec=0.02)
        time.sleep(0.02)

    def run(self):
        deadline = time.monotonic() + 5.0
        while rclpy.ok() and not self.fresh() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.fresh():
            raise RuntimeError('No fresh PX4 vehicle status; check namespace and DDS message version')
        if self.status.arming_state != VehicleStatus.ARMING_STATE_DISARMED:
            raise RuntimeError('Start this test with the shuttle disarmed')
        self.get_logger().info('SITL shuttle only: stop other mission/controller publishers before testing.')
        self.owns_vehicle = True
        # Establish Offboard heartbeat and zero motor commands before mode/arm requests.
        deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.tick()
        deadline = time.monotonic() + 8.0
        next_request = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            if not self.fresh():
                raise RuntimeError('PX4 status lost')
            if (self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED):
                break
            if time.monotonic() >= next_request:
                self.send_command(176, 1, 6)  # PX4 custom mode OFFBOARD
                if self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                    self.send_command(400, 1)  # Arm normally; no force-arm bypass.
                next_request = time.monotonic() + 0.5
            self.tick()
        else:
            raise RuntimeError('PX4 did not enter armed Offboard mode; inspect preflight checks')
        self.get_logger().info(f'Testing motor {self.motor}, command {self.value}, {self.duration}s')
        deadline = time.monotonic() + self.duration
        while rclpy.ok() and time.monotonic() < deadline:
            if (not self.fresh() or self.status.nav_state != VehicleStatus.NAVIGATION_STATE_OFFBOARD
                    or self.status.arming_state != VehicleStatus.ARMING_STATE_ARMED):
                raise RuntimeError('PX4 left armed Offboard mode or status became stale')
            self.tick(self.value)

    def stop(self):
        if not self.owns_vehicle or not rclpy.ok():
            return
        deadline = time.monotonic() + 3.0
        next_request = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            self.tick()
            if time.monotonic() >= next_request:
                self.send_command(400, 0)
                next_request = time.monotonic() + 0.2
            if self.fresh() and self.status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                self.get_logger().info('Motor commands zero; shuttle disarmed.')
                return
        self.get_logger().error('Disarm not confirmed; zero commands sent. Check PX4 immediately.')


def main(args=None):
    # Keep ROS alive during Ctrl+C cleanup so zero/disarm commands can be sent.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = None
    try:
        node = ShuttleMotorTest()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop()
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
