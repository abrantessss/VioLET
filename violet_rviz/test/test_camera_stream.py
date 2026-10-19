"""Check decoded image layout and interrupted pipe handling."""
import io
import subprocess
import threading
from types import SimpleNamespace

import pytest
from builtin_interfaces.msg import Time
from violet_rviz.camera_stream import CameraStream, decoder_command, frame_to_image, read_frame


class PartialPipe(io.BytesIO):
    def read(self, size):
        return super().read(min(size, 7))


def test_partial_reads_and_disconnect():
    pipe = PartialPipe(bytes(range(30)))
    assert read_frame(pipe, 24) == bytes(range(24))
    assert read_frame(pipe, 24) is None


def test_ffmpeg_frame_to_ros_image():
    # Exercise the actual decoder output format with a synthetic H.264 frame.
    encoded = subprocess.run([
        'ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'color=c=red:s=64x48',
        '-frames:v', '1', '-c:v', 'libx264', '-f', 'h264', 'pipe:1',
    ], check=True, stdout=subprocess.PIPE).stdout
    decoded = subprocess.run([
        'ffmpeg', '-v', 'error', '-f', 'h264', '-i', 'pipe:0',
        '-pix_fmt', 'bgr24', '-f', 'rawvideo', 'pipe:1',
    ], input=encoded, check=True, stdout=subprocess.PIPE).stdout
    image = frame_to_image(decoded, 64, 48, Time(sec=12), 'camera_optical_frame')
    assert image.header.stamp.sec == 12
    assert image.header.frame_id == 'camera_optical_frame'
    assert image.encoding == 'bgr8'
    assert image.step == 192
    assert len(image.data) == 64 * 48 * 3
    assert image.data[2] > 240 and image.data[0] < 10 and image.data[1] < 10
    with pytest.raises(ValueError):
        frame_to_image(decoded[:-1], 64, 48, Time(), 'camera_optical_frame')


def test_low_latency_decoder(tmp_path):
    source = tmp_path / 'stream.h264'
    subprocess.run([
        'ffmpeg', '-v', 'error', '-f', 'lavfi', '-i', 'color=c=red:s=64x48:r=15',
        '-frames:v', '45', '-c:v', 'libx264', '-tune', 'zerolatency',
        '-g', '15', '-f', 'h264', str(source),
    ], check=True)
    result = subprocess.run(decoder_command(str(source), 64, 48),
                            check=True, stdout=subprocess.PIPE, timeout=10)
    assert len(result.stdout) >= 64 * 48 * 3
    assert len(result.stdout) % (64 * 48 * 3) == 0


def test_publish_only_newest_frame():
    messages = []
    node = SimpleNamespace(
        frame_lock=threading.Lock(), latest_frame=None,
        width=2, height=2, frame_id='camera_optical_frame',
        publisher=SimpleNamespace(publish=messages.append))
    # Receiving replaces the slot while publication is paused.
    for value in range(5):
        node.latest_frame = (bytes([value] * 12), Time(sec=value))
    CameraStream.publish_latest(node)
    CameraStream.publish_latest(node)
    assert len(messages) == 1
    assert messages[0].header.stamp.sec == 4
    assert bytes(messages[0].data) == bytes([4] * 12)
