"""Exercise bidirectional MAVLink routing on loopback with a fake autopilot."""
import signal
import socket
import subprocess
import sys
import time
from pymavlink.dialects.v20 import common as mav


def bound_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1', 0))
    sock.settimeout(0.1)
    return sock


def test(executable):
    desktop = bound_socket()
    autopilot = bound_socket()
    reserve = bound_socket()
    input_port = reserve.getsockname()[1]
    reserve.close()
    process = subprocess.Popen([executable, f'udp://:{input_port}', '2',
                                f'udp://127.0.0.1:{desktop.getsockname()[1]}', '--test-gamma', 'false', '--offline', 'false'],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    encoder = mav.MAVLink(None, srcSystem=1, srcComponent=1)
    pc_encoder = mav.MAVLink(None, srcSystem=245, srcComponent=190)
    desktop_decoder, autopilot_decoder = mav.MAVLink(None), mav.MAVLink(None)
    peer = None
    got_heartbeat, got_attitude, got_request, got_ack = False, False, False, False
    try:
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline and not (got_heartbeat and got_attitude and got_request and got_ack):
            if process.poll() is not None:
                raise AssertionError(process.stdout.read().decode())
            heartbeat = encoder.heartbeat_encode(mav.MAV_TYPE_QUADROTOR, mav.MAV_AUTOPILOT_PX4,
                                                 0, 0, mav.MAV_STATE_STANDBY)
            autopilot.sendto(heartbeat.pack(encoder), ('127.0.0.1', input_port))
            attitude = encoder.attitude_quaternion_encode(1000, 1, 0, 0, 0, 0, 0, 0)
            autopilot.sendto(attitude.pack(encoder), ('127.0.0.1', input_port))
            try:
                data, peer = desktop.recvfrom(4096)
                for message in desktop_decoder.parse_buffer(data) or []:
                    if message.get_srcSystem() == 1 and message.get_srcComponent() == 1:
                        got_heartbeat |= message.get_type() == 'HEARTBEAT'
                        got_attitude |= message.get_type() == 'ATTITUDE_QUATERNION'
                        got_ack |= (message.get_type() == 'COMMAND_ACK' and
                                    message.target_system == 245 and message.target_component == 190)
            except socket.timeout:
                pass
            if peer:
                request = pc_encoder.command_long_encode(1, 1, mav.MAV_CMD_REQUEST_MESSAGE,
                                                         0, 4242, 0, 0, 0, 0, 0, 0)
                desktop.sendto(request.pack(pc_encoder), peer)
            try:
                data, _ = autopilot.recvfrom(4096)
                for message in autopilot_decoder.parse_buffer(data) or []:
                    if message.get_type() == 'COMMAND_LONG':
                        ack = encoder.command_ack_encode(message.command, mav.MAV_RESULT_ACCEPTED,
                                                         0, 0, message.get_srcSystem(), message.get_srcComponent())
                        autopilot.sendto(ack.pack(encoder), ('127.0.0.1', input_port))
                    if (message.get_type() == 'COMMAND_LONG' and message.get_srcSystem() == 245
                            and message.param1 == 4242):
                        got_request = True
            except socket.timeout:
                pass
        assert got_heartbeat, 'Autopilot heartbeat did not reach desktop'
        assert got_attitude, 'Autopilot attitude did not reach desktop'
        assert got_request, 'Desktop telemetry request did not reach autopilot'
        assert got_ack, 'Autopilot acknowledgement did not reach desktop'
    finally:
        process.send_signal(signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            output, _ = process.communicate()
        desktop.close()
        autopilot.close()
        assert b'Camera' not in output, 'Camera output remains in telemetry-only executable'


if __name__ == '__main__':
    test(sys.argv[1])
