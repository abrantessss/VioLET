"""Fake PX4: validate outbound commands and RC lifecycle without an aircraft."""
import math
import signal
import socket
import subprocess
import sys
import threading
import tempfile
import time
from pymavlink.dialects.v20 import common as mav


class FakePx4:
    def __init__(self, executable, options):
        reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        reserve.bind(('127.0.0.1', 0))
        self.destination = ('127.0.0.1', reserve.getsockname()[1])
        reserve.close()
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('127.0.0.1', 0))
        self.sock.settimeout(0.01)
        self.encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
        self.decoder = mav.MAVLink(None)
        self.mode, self.pwm, self.armed, self.air = 1, 1900, True, True
        self.position = True
        self.kill_pwm = 1100
        self.speed = 14.0
        self.bad = False
        self.messages = []
        self.stopping = False
        self.log = tempfile.TemporaryFile()
        self.process = subprocess.Popen([executable, f'udp://:{self.destination[1]}', '10', '--controller-enabled', 'false', '--test-gamma', 'false', '--offline', 'false', *options],
                                        stdout=self.log, stderr=subprocess.STDOUT)
        self.thread = threading.Thread(target=self.pump)
        self.thread.start()

    def send(self, message):
        self.sock.sendto(message.pack(self.encoder), self.destination)

    def pump(self):
        while not self.stopping:
            flags = mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (mav.MAV_MODE_FLAG_SAFETY_ARMED if self.armed else 0)
            self.send(self.encoder.heartbeat_encode(mav.MAV_TYPE_FIXED_WING, mav.MAV_AUTOPILOT_PX4,
                      flags, self.mode << 16, mav.MAV_STATE_ACTIVE))
            self.send(self.encoder.extended_sys_state_encode(
                mav.MAV_VTOL_STATE_UNDEFINED, mav.MAV_LANDED_STATE_IN_AIR if self.air else mav.MAV_LANDED_STATE_ON_GROUND))
            channels = [1500] * 18
            channels[6] = self.pwm
            channels[5] = self.kill_pwm
            self.send(self.encoder.rc_channels_encode(1000, 18, *channels, 100))
            self.send(self.encoder.attitude_encode(1000, 0, 0, math.pi/2, 0, 0, 0))
            self.send(self.encoder.attitude_quaternion_encode(1000, 1, 0, 0, 0, 0, 0, 0))
            outputs = [1100.,1200.,1300.,1400.,1500.,1600.,1700.,1800.]+[0.]*24
            self.send(self.encoder.actuator_output_status_encode(1000000, 255, outputs))
            if self.position:
                self.send(self.encoder.local_position_ned_encode(1000, float('nan') if self.bad else 50,
                                                                0, -30, 0, self.speed, 0))
            try:
                data, _ = self.sock.recvfrom(4096)
                for message in self.decoder.parse_buffer(data) or []:
                    self.messages.append((time.monotonic(), message))
                    if message.get_type() == 'COMMAND_LONG':
                        if message.command == mav.MAV_CMD_DO_SET_MODE:
                            self.mode = int(message.param2)
                        self.send(self.encoder.command_ack_encode(message.command, mav.MAV_RESULT_ACCEPTED,
                                  0, 0, message.get_srcSystem(), message.get_srcComponent()))
            except socket.timeout:
                pass
            time.sleep(0.01)

    def wait(self, predicate, seconds=6):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert self.process.poll() is None, 'pf_los4 unexpectedly exited'
            if predicate():
                return
            time.sleep(0.02)
        raise AssertionError('Timed out waiting for controller lifecycle')

    def controls(self, after=0):
        return [m for t, m in self.messages if t > after and
                (m.get_type() == 'SET_ATTITUDE_TARGET' or
                 (m.get_type() == 'COMMAND_LONG' and m.command == mav.MAV_CMD_DO_SET_MODE))]

    def close(self):
        self.process.send_signal(signal.SIGINT)
        self.process.wait(timeout=6)
        self.log.seek(0)
        output = self.log.read()
        self.log.close()
        self.stopping = True
        self.thread.join(timeout=2)
        self.sock.close()
        assert self.process.returncode == 0, output.decode()
        assert not any(m.get_type() == 'COMMAND_LONG' and m.command in (
            mav.MAV_CMD_COMPONENT_ARM_DISARM, mav.MAV_CMD_NAV_TAKEOFF) for _, m in self.messages), 'Arm/takeoff command generated'
        return output.decode()


def test(executable):
    for options in ([], ['--controller-enabled', 'true', '--test-gamma', 'true']):
        fake = FakePx4(executable, options)
        try:
            time.sleep(2)
            assert not fake.controls(), 'Inhibit/test emitted control commands'
        finally:
            output = fake.close()
        if options:
            assert 'gamma_dot=1.0000' in output

    live = ['--controller-enabled', 'true', '--rc-channel', '7', '--return-mode', 'manual']
    fake = FakePx4(executable, live)
    try:
        fake.armed = False
        time.sleep(2)
        assert not fake.controls(), 'High switch at startup activated'
        fake.pwm = 1500  # middle is Manual
        time.sleep(0.3)
        assert not fake.controls(), 'Disarmed vehicle received control setpoints'
        fake.armed = True
        fake.wait(lambda: any(m.get_type()=='SET_ATTITUDE_TARGET' for m in fake.controls()))
        time.sleep(1.4)
        fake.pwm = 1900
        time.sleep(0.2)
        assert not any(m.get_type()=='COMMAND_LONG' and m.command==mav.MAV_CMD_DO_SET_MODE
                       and m.param2==6 for m in fake.controls()), 'Script requested Offboard entry'
        fake.mode = 6  # PX4 follows the RC flight-mode switch
        time.sleep(0.4)
        fake.kill_pwm = 1900
        fake.wait(lambda: fake.mode == 1)
        killed = time.monotonic()
        time.sleep(0.4)
        assert not [m for m in fake.controls(killed+0.15) if m.get_type()=='SET_ATTITUDE_TARGET'], 'Kill did not stop commands'
        fake.kill_pwm = 1100
        fake.pwm = 1500
        time.sleep(1.6)
        fake.pwm = 1900
        fake.mode = 6
        time.sleep(0.4)
        fake.position = False
        fake.wait(lambda: fake.mode == 1)
        time.sleep(0.4)
        assert fake.mode == 1, 'Fault retried mode entry automatically'
        assert not any(m.get_type()=='COMMAND_LONG' and m.command==mav.MAV_CMD_DO_SET_MODE
                       and m.param2==6 for m in fake.controls()), 'Mode-entry command emitted'
    finally:
        output = fake.close()
    assert 'LOS4_ACTIVE' in output and 'FAULT_LATCHED' in output
    print('Hard inhibit, test isolation, RC-owned Offboard entry, kill and stale release passed')


if __name__ == '__main__':
    test(sys.argv[1])
